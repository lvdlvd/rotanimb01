// rotanimb01 — HITL sensor simulator for ArduPlane (design in ../DESIGN.md).
//
// M2 state: M0 skeleton (clock, console on USART1, vector manifest, fault
// machinery, heartbeat) + the 8-channel PWM capture (lib/pwm capture on TIM2+TIM3,
// 1 us tick; TIM2's 32-bit counter doubles as the harness microsecond clock)
// + the host CAN link (lib/fdcan, FDCAN1 on PA11/PA12, classic 1 Mbit).
//
// CAN dictionary: 29-bit bit-field headers and big-endian payloads per the
// shared in-house convention — layout and the harness's MSGID allocation
// (the 0x40 block, both LCCs) in canmsg.h.
//
// M5: TIM7 ticks at 10 kHz; the main loop consumes ticks and runs the table
// scheduler — physics integration on every 5th tick (2 kHz), and a Bresenham
// rate accumulator per device (acc += hz; acc >= 10000 -> sample) so any
// configured ODR is served with <= 100 us jitter and an exact long-term
// rate. Samples pull the physics truth, quantize per the LIVE config
// registers and commit. CMD_STATE/CMD_ENV decode into the lag targets;
// stale watchdog: no CMD_STATE for 1 s -> hold last state, flag in STATUS.

#include "device.h" // generated for STM32G474
#include "pinmux.h"

#include "binary.h"
#include "board.h"
#include "canmsg.h"
#include "clock.h"
#include "exti.h"
#include "console.h" // pulls serial.h + tprintf.h
#include "fault.h"
#include "fdcan.h"
#include "gpio.h"
#include "nvic.h"
#include "physics.h"

#include <math.h>
#include "pwm.h"
#include "sensors.h"
#include "startup.h"

extern const isr_t __vectors[]; // the vector table, defined at the foot of the file

static uint8_t tx_buf[1024]; // power-of-two rings
static uint8_t rx_buf[512];
static struct Serial vcp = SERIAL_INITIALIZER(USART1, tx_buf);
static struct Serial vcp_rx = SERIAL_INITIALIZER(USART1, rx_buf);
static struct SerialRXCounters vcp_rxc;

static void cputc(char c) { usart_putc(&USART1, c); } // polled, for the post-mortem dump

// 8x PWM capture from the DUT (DESIGN.md pinout); TIM3 also clocks the FDCAN
// message timestamps (TSCC), so CAN ts is us mod 2^16 on the same timebase.
static struct PWMIn cap2 = PWMIN_INITIALIZER_32(TIM2, PA0, PA1, PB10, PB11);
static struct PWMIn cap3 = PWMIN_INITIALIZER(TIM3, PC6, PC7, PC8, PC9);

// The harness microsecond clock: TIM2's free-running 32-bit 1 MHz counter.
static inline uint32_t now_us(void) { return cap2.tim->CNT; }

// ---- host command mailbox (written by the CAN RX irq, read at thread level)
struct CmdBox {
	uint8_t data[8];
	volatile uint8_t len;
	volatile uint32_t seq;   // increments per received frame
	volatile uint32_t rx_us; // now_us() at reception
};
static struct CmdBox cmd_state, cmd_env, cmd_noise;

static void cmd_store(struct CmdBox *b, const uint8_t *p, size_t len) {
	for (size_t i = 0; i < len && i < 8; i++) {
		b->data[i] = p[i];
	}
	b->len = (uint8_t)len;
	b->rx_us = now_us();
	b->seq++;
}

// ---- CAN TX helpers --------------------------------------------------------
static struct FDCan can1 = FDCAN_INITIALIZER(FDCAN1);
static uint8_t srcid; // hashed from the device UID at boot

static void can_send(uint32_t msgid, const uint8_t *payload, size_t len) {
	static uint8_t seq[4]; // per-message ts_seq, CANMSG_PWM14..DIAG
	uint32_t id29 = canmsg_id29(CANMSG_LCC_MEAS, msgid, 0, srcid, seq[msgid - CANMSG_PWM14]++);
	fdcan_tx(&can1, (uint8_t)msgid, can_header_from29(id29), len, payload); // tag = msgid
}

// IRQ priorities, 2:2 grouping: level[3:2] = preemption group, level[1:0] =
// subpriority (orders pending IRQs only). Handlers in the SAME group can
// never preempt each other — the spislave engine relies on that for SPI3 vs
// the CS EXTIs (both mutate the bus state; select/deselect vs byte-0 decode
// must serialize).
enum { IRQ_PRIORITY_GROUPING_2_2 = 5 };
#define PRIO(grp, sub) ((grp) << 2 | (sub))
static const struct {
	enum IRQn_Type irq;
	uint8_t prio;
} irqprios[] = {
	{SPI3_IRQn, PRIO(0, 0)},      // byte-0 cmd decode, ~72-cycle deadline
	{EXTI0_IRQn, PRIO(0, 1)},     // CS baro: dummy pre-stuff / deselect scrub
	{EXTI1_IRQn, PRIO(0, 1)},     // CS mag
	{EXTI4_IRQn, PRIO(0, 1)},     // CS gyro
	{EXTI15_10_IRQn, PRIO(0, 1)}, // CS accel

	{TIM2_IRQn, PRIO(1, 0)}, // PWM capture ch1-4 (µs timestamps)
	{TIM3_IRQn, PRIO(1, 0)}, // PWM capture ch5-8

	{FDCAN1_IT0_IRQn, PRIO(1, 1)}, // TX events, bus-off
	{FDCAN1_IT1_IRQn, PRIO(1, 1)}, // RX: command mailboxes

	{TIM7_DAC2_4_IRQn, PRIO(2, 1)}, // 10 kHz scheduler tick (counter only)

	{DMA1_CH1_IRQn, PRIO(2, 0)}, // console TX DMA
	{DMA1_CH2_IRQn, PRIO(2, 0)}, // console RX DMA
	{USART1_IRQn, PRIO(2, 0)},   // console TX kick / RX idle flush
};
#undef PRIO

// ---- the 10 kHz scheduler tick ----------------------------------------------
static volatile uint32_t tim7_ticks;

// DWT cycle counter: measure the physics step on the real core (heartbeat
// reports the max per interval — the flight branch costs more than bench)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define DCB_DEMCR (*(volatile uint32_t *)0xE000EDFC)
static uint32_t phys_cycles_max;

static int16_t sat16(float x) {
	if (x > 32767.0f) {
		return 32767;
	}
	if (x < -32767.0f) {
		return -32767;
	}
	return (int16_t)x;
}

// pull the physics truth, quantize per the live configs, commit
static void sample_gyro(uint32_t now) {
	const struct PhysicsTruth *t = physics_truth();
	float lsb = 32767.0f / gyro_fullscale_dps(); // counts per deg/s
	int16_t xyz[3];
	for (int i = 0; i < 3; i++) {
		xyz[i] = sat16(t->rate[i] * (180.0f / (float)M_PI) * lsb);
	}
	gyro_commit(xyz, now);
}

static void sample_accel(void) {
	const struct PhysicsTruth *t = physics_truth();
	float lsb = 32767.0f / (accel_fullscale_g() * PHYSICS_G); // counts per m/s^2
	int16_t xyz[3];
	for (int i = 0; i < 3; i++) {
		xyz[i] = sat16(t->sforce[i] * lsb);
	}
	accel_commit(xyz, t->t_degc);
}

static void sample_mag(void) {
	const struct PhysicsTruth *t = physics_truth();
	float lsb = mag_lsb_per_ut();
	int32_t xyz[3];
	for (int i = 0; i < 3; i++) {
		xyz[i] = (int32_t)(t->mag[i] * lsb);
	}
	mag_commit(xyz);
}

// ---- host command decode (big-endian, canmsg.h dictionary) -------------------

static bool cmd_snapshot(struct CmdBox *b, uint8_t p[8], uint32_t *seq) {
	uint32_t s;
	do { // seq-stable copy against the RX irq
		s = b->seq;
		for (int i = 0; i < 8; i++) {
			p[i] = b->data[i];
		}
	} while (b->seq != s);
	if (s == *seq || b->len < 8) {
		return false;
	}
	*seq = s;
	return true;
}

static void cmd_decode(void) {
	static uint32_t seq_state, seq_env;
	uint8_t p[8];
	if (cmd_snapshot(&cmd_state, p, &seq_state)) {
		// V cm/s i16, hdot cm/s i16, psidot mrad/s i16, flags u16
		physics_cmd((int16_t)decode_be_uint16(p) * 0.01f,
		            (int16_t)decode_be_uint16(p + 2) * 0.01f,
		            (int16_t)decode_be_uint16(p + 4) * 0.001f);
	}
	if (cmd_snapshot(&cmd_env, p, &seq_env)) {
		// QNH Pa/10 u16, T0 0.1 K u16, B 0.01 uT u16, incl 0.01 deg i16
		physics_env(decode_be_uint16(p) * 10.0f,
		            decode_be_uint16(p + 2) * 0.1f,
		            decode_be_uint16(p + 4) * 0.01f,
		            (int16_t)decode_be_uint16(p + 6) * 0.01f * ((float)M_PI / 180.0f));
	}
}

void Reset_Handler(void) __attribute__((noreturn));
void Reset_Handler(void) {
	narray_init_memory();
	SCB.VTOR = (uint32_t)(uintptr_t)__vectors;
	*(volatile uint32_t *)0xE000ED88 |= 0xfu << 20; // FPU: CP10/CP11 full access
	SCB.SHCSR |= SCB_SHCSR_USGFAULTENA;             // route usage faults to our handler
	SCB.CCR |= SCB_CCR_DIV_0_TRP;                   // div-by-zero -> UsageFault

	board_init(); // clock + peripheral clocks + the whole pinout

	nvic_set_priority_grouping(IRQ_PRIORITY_GROUPING_2_2);
	for (size_t i = 0; i < sizeof irqprios / sizeof irqprios[0]; i++) {
		nvic_set_priority(irqprios[i].irq, irqprios[i].prio);
	}

	dma_set_mux(DMA1_CH1, DMA_REQ_USART1_TX);
	dma_set_mux(DMA1_CH2, DMA_REQ_USART1_RX);
	usart_init(&USART1, clock_usart_hz(1), 115200); // full duplex + RX timeout flush
	console = &vcp;
	serial_dma_rx_start(&vcp_rx, DMA1_CH2);
	nvic_enable(DMA1_CH1_IRQn);
	nvic_enable(DMA1_CH2_IRQn);
	nvic_enable(USART1_IRQn);

	pwmin_init(&cap2, clock_pclk1_timer_hz());
	pwmin_init(&cap3, clock_pclk1_timer_hz());
	nvic_enable(TIM2_IRQn);
	nvic_enable(TIM3_IRQn);

	srcid = canmsg_srcid_self();
	fdcan_init(&can1, clock_fdcan_hz(), 1000000);
	nvic_enable(FDCAN1_IT0_IRQn);
	nvic_enable(FDCAN1_IT1_IRQn);

	// the SPI3 sensor bus: four register-file devices, CS demux on EXTI
	dma_set_mux(DMA1_CH5, DMA_REQ_SPI3_TX);
	sensors_init();
	exti_init(CS_BARO | CS_MAG, true, true);
	exti_init(CS_GYRO, true, true);
	exti_init(CS_ACC, true, true);
	nvic_enable(SPI3_IRQn);
	nvic_enable(EXTI0_IRQn);
	nvic_enable(EXTI1_IRQn);
	nvic_enable(EXTI4_IRQn);
	nvic_enable(EXTI15_10_IRQn);

	DCB_DEMCR |= 1u << 24; // TRCENA
	DWT_CTRL |= 1u;        // CYCCNTENA

	// the 10 kHz scheduler tick
	physics_init();
	RCC.APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	TIM7.PSC = (uint16_t)(clock_pclk1_timer_hz() / 1000000 - 1); // 1 MHz
	TIM7.ARR = 100 - 1;                                          // 10 kHz
	TIM7.DIER = TIM_BASIC_INST_DIER_UIE;
	TIM7.CR1 = TIM_BASIC_INST_CR1_CEN;
	nvic_enable(TIM7_DAC2_4_IRQn);

	fault_report(cputc); // print a crash from the previous run, if any
	tprintf("\nrotanimb01 HITL harness on STM32G474, sysclk = %u Hz, hse = %u Hz, can kernel = %u Hz, srcid %02x\n",
	        (unsigned)clock_sysclk_hz(), (unsigned)clock_hse_hz, (unsigned)clock_fdcan_hz(), srcid);

	// Report state per PWM channel: last sent width and the staleness tracker.
	struct {
		uint16_t sent_w;
		uint32_t count;
		uint32_t change_us; // last time count advanced
	} chan[8] = {0};

	uint32_t t_pwm = now_us(), t_status = t_pwm, t_diag = t_pwm, t_tick = t_pwm;
	for (;;) {
		// console RX echo (link sanity, M0 heritage)
		uint8_t chunk[64];
		size_t n = fifo_read(&vcp_rx.buf, chunk, sizeof chunk);
		if (n) {
			fifo_write(&vcp.buf, chunk, n);
			serial_dma_tx_start(&vcp);
		}

		uint32_t now = now_us();
		sensors_poll(now);
		cmd_decode();

		// the table scheduler: consume 10 kHz ticks; physics every 5th
		// (2 kHz), Bresenham accumulator per device for any configured ODR
		{
			static uint32_t done;
			static uint32_t phys_div, acc_g, acc_a, acc_b, acc_m;
			while (done != tim7_ticks) {
				done++;
				if (++phys_div >= 5) {
					phys_div = 0;
					uint32_t c0 = DWT_CYCCNT;
					physics_step(5 * 100e-6f);
					uint32_t c = DWT_CYCCNT - c0;
					if (c > phys_cycles_max) {
						phys_cycles_max = c;
					}
				}
				uint32_t hz;
				if ((hz = gyro_rate_hz()) != 0 && (acc_g += hz) >= 10000) {
					acc_g -= 10000;
					sample_gyro(now);
				}
				if ((hz = accel_rate_hz()) != 0 && (acc_a += hz) >= 10000) {
					acc_a -= 10000;
					sample_accel();
				}
				if ((hz = baro_rate_hz()) != 0 && (acc_b += hz) >= 10000) {
					acc_b -= 10000;
					const struct PhysicsTruth *t = physics_truth();
					baro_commit(t->t_degc, t->p_pa);
				}
				if ((hz = mag_rate_hz()) != 0 && (acc_m += hz) >= 10000) {
					acc_m -= 10000;
					sample_mag();
				}
			}
		}

		// gather widths; a channel with no completed period for 100 ms reads 0
		uint16_t w[8];
		bool change = false;
		for (int i = 0; i < 8; i++) {
			struct PWMInSample s = pwmin_get(i < 4 ? &cap2 : &cap3, i & 3);
			if (s.count != chan[i].count) {
				chan[i].count = s.count;
				chan[i].change_us = now;
			}
			w[i] = (now - chan[i].change_us > 100000) ? 0 : (uint16_t)s.width_us;
			int d = (int)w[i] - (int)chan[i].sent_w;
			change |= d > 2 || d < -2;
		}
		if (change || (int32_t)(now - t_pwm) >= 20000) { // 50 Hz + on change
			t_pwm = now;
			uint8_t p[8];
			for (int i = 0; i < 4; i++) {
				encode_be_uint16(p + 2 * i, w[i]);
			}
			can_send(CANMSG_PWM14, p, 8);
			for (int i = 0; i < 4; i++) {
				encode_be_uint16(p + 2 * i, w[4 + i]);
			}
			can_send(CANMSG_PWM58, p, 8);
			for (int i = 0; i < 8; i++) {
				chan[i].sent_w = w[i];
			}
		}

		if ((int32_t)(now - t_status) >= 100000) { // STATUS 10 Hz
			t_status = now;
			bool stale = cmd_state.seq == 0 || now - cmd_state.rx_us > 1000000;
			float psi_deg = physics_truth()->psi * (180.0f / (float)M_PI);
			if (psi_deg < 0) {
				psi_deg += 360.0f;
			}
			uint8_t p[8];
			encode_be_uint32(p, now);
			encode_be_uint16(p + 4, (uint16_t)(psi_deg * 100.0f)); // psi 0.01 deg
			encode_be_uint16(p + 6, (uint16_t)((stale ? 1 : 0) | physics_flags()));
			can_send(CANMSG_STATUS, p, 8);
		}

		if ((int32_t)(now - t_diag) >= 1000000) { // DIAG 1 Hz
			t_diag = now;
			uint32_t errs = 0;
			for (int i = 0; i < 4; i++) {
				errs += cap2.ch[i].errs + cap3.ch[i].errs;
			}
			uint8_t p[8];
			encode_be_uint16(p, (uint16_t)can1.status.tx_count);
			encode_be_uint16(p + 2, (uint16_t)can1.status.rx_count[0]);
			encode_be_uint16(p + 4, (uint16_t)(can1.status.rx_ovfl[0] + can1.status.rx_ovfl[1]));
			encode_be_uint16(p + 6, (uint16_t)errs);
			can_send(CANMSG_DIAG, p, 8);
		}

		if ((int32_t)(now - t_tick) >= 1000000) { // console heartbeat 1 Hz
			t_tick = now;
			digitalToggle(LED);
			const struct PhysicsTruth *pt = physics_truth();
			uint32_t pc = phys_cycles_max;
			phys_cycles_max = 0;
			tprintf("t %u us pwm %u %u %u %u %u %u %u %u psi %d cdeg h %d cm p %u Pa phys %u cy spi g/a/b/m %u/%u/%u/%u unexp %u stray %u cmd seq %u can tx %u rx %u lec %u/%u/%u/%u/%u/%u/%u\n",
			        (unsigned)now, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
			        (int)(pt->psi * (18000.0f / (float)M_PI)), (int)(pt->h * 100.0f),
			        (unsigned)pt->p_pa, (unsigned)pc,
			        (unsigned)gyro_dev.frames, (unsigned)accel_dev.frames,
			        (unsigned)baro_dev.frames, (unsigned)mag_dev.frames,
			        (unsigned)(gyro_dev.unexpected + accel_dev.unexpected + baro_dev.unexpected + mag_dev.unexpected),
			        (unsigned)sensor_bus.stray,
			        (unsigned)cmd_state.seq, (unsigned)can1.status.tx_count,
			        (unsigned)can1.status.rx_count[0],
			        (unsigned)can1.status.lec_count[1], (unsigned)can1.status.lec_count[2],
			        (unsigned)can1.status.lec_count[3], (unsigned)can1.status.lec_count[4],
			        (unsigned)can1.status.lec_count[5], (unsigned)can1.status.lec_count[6],
			        (unsigned)can1.status.lec_count[7]);
		}
	}
}

// Serial interrupts (the app owns the vectors and calls the lib handlers).
static void dma1_ch1(void) { serial_dma_tx_handler(&vcp, DMA1_CH1); }
static void dma1_ch2(void) { serial_dma_rx_handler(&vcp_rx, DMA1_CH2); }
static void usart1(void) {
	serial_irq_tx_handler(&vcp, DMA1_CH1);
	serial_irq_rx_handler(&vcp_rx, DMA1_CH2, &vcp_rxc);
}

static void tim2(void) { pwmin_irq_handler(&cap2); }
static void tim3(void) { pwmin_irq_handler(&cap3); }
static void tim7(void) {
	TIM7.SR = 0; // w0c UIF
	tim7_ticks++;
}

// the sensor bus: byte-0 deadline path inlined with constant instances
static void spi3(void) { spislave_irq(&sensor_bus, &SPI3, DMA1_CH5); }
static void sensor_cs(void) { spislave_cs_handler(&sensor_bus); }

// FDCAN1 IT0: tx events + errors — drain the event fifo (updates counters).
static void fdcan1_it0(void) {
	uint8_t tag;
	uint32_t header;
	uint16_t ts;
	while (fdcan_tx_done(&can1, &tag, &header, &ts) >= 0) {
	}
	FDCAN1.IR = FDCAN1.IR & ~(FDCAN_IR_RF0N | FDCAN_IR_RF1N); // w1c all but the RX flags
}

// FDCAN1 IT1: rx fifos — pop everything, latch known commands.
static void fdcan1_it1(void) {
	FDCAN1.IR = FDCAN_IR_RF0N | FDCAN_IR_RF1N; // w1c
	uint8_t fmi, p[8];
	uint32_t header;
	uint16_t ts;
	size_t len = sizeof p;
	while (fdcan_rx(&can1, &fmi, &header, &len, p, &ts) >= 0) {
		if (can_header_isext(header)) {
			uint32_t id29 = can_header_to29(header);
			if (canmsg_lcc(id29) == CANMSG_LCC_TMC) {
				switch (canmsg_msgid(id29)) {
				case CANMSG_CMD_STATE:
					cmd_store(&cmd_state, p, len);
					break;
				case CANMSG_CMD_ENV:
					cmd_store(&cmd_env, p, len);
					break;
				case CANMSG_CMD_NOISE:
					cmd_store(&cmd_noise, p, len);
					break;
				}
			}
		}
		len = sizeof p;
	}
}

extern void _estack(void); // top of stack (linker)

// The vector table IS the program: slot 0 = SP, 1 = Reset (positional), 3-6 the
// core fault handlers, device IRQs by VECTOR(IRQn). Unwired slots stay NULL and
// fault loudly if ever taken.
__attribute__((section(".isr_vector"))) const isr_t __vectors[NVIC_VECTORS] = {
	(isr_t)&_estack, // [0] SP    — positional
	Reset_Handler,   // [1] Reset — positional, no +16
	[VECTOR(HardFault_IRQn)] = HardFault_Handler,
	[VECTOR(MemManage_IRQn)] = MemManage_Handler,
	[VECTOR(BusFault_IRQn)] = BusFault_Handler,
	[VECTOR(UsageFault_IRQn)] = UsageFault_Handler,
	[VECTOR(DMA1_CH1_IRQn)] = dma1_ch1,
	[VECTOR(DMA1_CH2_IRQn)] = dma1_ch2,
	[VECTOR(USART1_IRQn)] = usart1,
	[VECTOR(TIM2_IRQn)] = tim2,
	[VECTOR(TIM3_IRQn)] = tim3,
	[VECTOR(TIM7_DAC2_4_IRQn)] = tim7,
	[VECTOR(SPI3_IRQn)] = spi3,
	[VECTOR(EXTI0_IRQn)] = sensor_cs,
	[VECTOR(EXTI1_IRQn)] = sensor_cs,
	[VECTOR(EXTI4_IRQn)] = sensor_cs,
	[VECTOR(EXTI15_10_IRQn)] = sensor_cs,
	[VECTOR(FDCAN1_IT0_IRQn)] = fdcan1_it0,
	[VECTOR(FDCAN1_IT1_IRQn)] = fdcan1_it1,
};
