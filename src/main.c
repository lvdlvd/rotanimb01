// rotanimb01 — HITL sensor simulator for ArduPlane (design in ../DESIGN.md).
//
// M2 state: M0 skeleton (clock, console on USART1, vector manifest, fault
// machinery, heartbeat) + the 8-channel PWM capture (lib/pwm capture on TIM2+TIM3,
// 1 us tick; TIM2's 32-bit counter doubles as the harness microsecond clock)
// + the host CAN link (lib/fdcan, FDCAN1 on PA11/PA12, classic 1 Mbit).
//
// CAN dictionary: 29-bit bit-field headers and big-endian payloads per the
// shared in-house convention — layout and the harness's MSGID allocation
// (the 0x40 block, both LCCs) in canmsg.h. Commands are stored raw here;
// interpretation (lag filters, physics) is M5. Stale-command watchdog: no
// CMD_STATE for 1 s -> hold, flag in STATUS.

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

void Reset_Handler(void) __attribute__((noreturn));
void Reset_Handler(void) {
	narray_init_memory();
	SCB.VTOR = (uint32_t)(uintptr_t)__vectors;
	*(volatile uint32_t *)0xE000ED88 |= 0xfu << 20; // FPU: CP10/CP11 full access
	SCB.SHCSR |= SCB_SHCSR_USGFAULTENA;             // route usage faults to our handler
	SCB.CCR |= SCB_CCR_DIV_0_TRP;                   // div-by-zero -> UsageFault

	board_init(); // clock + peripheral clocks + the whole pinout

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

		// Static bench-cal sampler until M5's physics: 1 g down, zero rates,
		// mid-latitude field, ISA sea level — at each device's LIVE config
		// rate. The M5 scheduler replaces this loop.
		{
			static uint32_t t_g, t_a, t_b, t_m;
			uint32_t hz;
			if ((hz = gyro_rate_hz()) != 0 && now - t_g >= 1000000u / hz) {
				t_g = now;
				const int16_t zero[3] = {0, 0, 0};
				gyro_commit(zero, now);
			}
			if ((hz = accel_rate_hz()) != 0 && now - t_a >= 1000000u / hz) {
				t_a = now;
				int16_t g1[3] = {0, 0, (int16_t)(32767.0f / accel_fullscale_g())}; // +1 g on Z
				accel_commit(g1, 25.0f);
			}
			if ((hz = baro_rate_hz()) != 0 && now - t_b >= 1000000u / hz) {
				t_b = now;
				baro_commit(15.0f, 101325.0);
			}
			if ((hz = mag_rate_hz()) != 0 && now - t_m >= 1000000u / hz) {
				t_m = now;
				float lsb = mag_lsb_per_ut();
				int32_t field[3] = {(int32_t)(20.0f * lsb), 0, (int32_t)(44.0f * lsb)}; // ~48 uT, incl 65 deg
				mag_commit(field);
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
			uint8_t p[8];
			encode_be_uint32(p, now);
			encode_be_uint16(p + 4, 0); // psi 0.01deg: physics lands at M5
			encode_be_uint16(p + 6, (uint16_t)(stale ? 1 : 0)); // flags, bit0 = cmd stale
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
			tprintf("t %u us pwm %u %u %u %u %u %u %u %u spi g/a/b/m %u/%u/%u/%u unexp %u stray %u cmd seq %u can tx %u rx %u lec %u/%u/%u/%u/%u/%u/%u\n",
			        (unsigned)now, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
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
	[VECTOR(SPI3_IRQn)] = spi3,
	[VECTOR(EXTI0_IRQn)] = sensor_cs,
	[VECTOR(EXTI1_IRQn)] = sensor_cs,
	[VECTOR(EXTI4_IRQn)] = sensor_cs,
	[VECTOR(EXTI15_10_IRQn)] = sensor_cs,
	[VECTOR(FDCAN1_IT0_IRQn)] = fdcan1_it0,
	[VECTOR(FDCAN1_IT1_IRQn)] = fdcan1_it1,
};
