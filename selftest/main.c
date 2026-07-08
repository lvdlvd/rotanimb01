// M4a self-test — the board masters its own sensor bus (v3 made SPI1 free).
//
// SPI1 (master, lib/spi.h, software CS on PB0/PB1/PA2/PA3) is jumpered onto
// the SPI3 sensor bus and replays the flight-tested master's init sequences
// against the four register-file models:
//
//   jumpers:  PA5 -> PC10 (SCK)   PA7 -> PC12 (MOSI)   PA6 -> PC11 (MISO)
//             PB0 -> PC0 (CS baro)   PB1 -> PC1  (CS mag)
//             PA2 -> PA4 (CS gyro)   PA3 -> PB12 (CS accel)
//
// Per device: probe (chip id / whoami), self-test dialogue, soft reset,
// configuration with readback verification, then a data read checked for
// physical plausibility — the baro raw words are forward-compensated on the
// master side and must reproduce the ISA sea level that the slave's trim
// inversion encoded. DRDY pin behavior is asserted by reading the pins.
// Bus speed 1.3 MHz (the no-dummy protocols' measured clean domain).

#include "device.h" // generated for STM32G474
#include "pinmux.h"

#include "binary.h"
#include "board.h"
#include "bmp390lin.h"
#include "clock.h"
#include "console.h"
#include "exti.h"
#include "fault.h"
#include "gpio.h"
#include "nvic.h"
#include "sensors.h"
#include "spi.h"
#include "spislave.h"
#include "startup.h"

extern const isr_t __vectors[];

static uint8_t tx_buf[4096];
static struct Serial vcp = SERIAL_INITIALIZER(USART1, tx_buf);
static void cputc(char c) { usart_putc(&USART1, c); }

// ---- microsecond time (TIM7 16-bit, software-extended by pump()) -----------

static uint32_t time_hi;
static uint16_t time_lo;
static uint32_t now_us(void) {
	uint16_t t = TIM7.CNT;
	if (t < time_lo) {
		time_hi += 0x10000;
	}
	time_lo = t;
	return time_hi | t;
}

// ---- the sampler (bench-cal statics, as in the harness main) ---------------

static void pump(void) {
	uint32_t now = now_us();
	sensors_poll(now);
	static uint32_t t_g, t_a, t_b, t_m;
	uint32_t hz;
	if ((hz = gyro_rate_hz()) != 0 && now - t_g >= 1000000u / hz) {
		t_g = now;
		const int16_t zero[3] = {0, 0, 0};
		gyro_commit(zero, now);
	}
	if ((hz = accel_rate_hz()) != 0 && now - t_a >= 1000000u / hz) {
		t_a = now;
		int16_t g1[3] = {0, 0, (int16_t)(32767.0f / accel_fullscale_g())};
		accel_commit(g1, 25.0f);
	}
	if ((hz = baro_rate_hz()) != 0 && now - t_b >= 1000000u / hz) {
		t_b = now;
		baro_commit(15.0f, 101325.0);
	}
	if ((hz = mag_rate_hz()) != 0 && now - t_m >= 1000000u / hz) {
		t_m = now;
		float lsb = mag_lsb_per_ut();
		int32_t field[3] = {(int32_t)(20.0f * lsb), 0, (int32_t)(44.0f * lsb)};
		mag_commit(field);
	}
}

static void delay_us(uint32_t us) {
	uint32_t t0 = now_us();
	while (now_us() - t0 < us) {
		pump();
	}
}

// ---- master side ------------------------------------------------------------

static struct SPIQ spiq;

enum { DEV_BARO, DEV_MAG, DEV_GYRO, DEV_ACC }; // ss addr
static const struct {
	enum GPIO_Pin cs; // the master's CS OUTPUT pin
	int ndummy;
	char tag;
} master_dev[4] = {
	{PB0, 1, 'B'}, {PB1, 0, 'M'}, {PA2, 0, 'G'}, {PA3, 1, 'A'},
};

static void ss_settle(void) {
	for (volatile int i = 0; i < 96; i++) { // ~1.8 us: a realistic master's gap
	}
}

static void spi1_ss(struct SPI_Type *spi, uint16_t addr, int on) {
	(void)spi;
	if (on) {
		digitalLo(master_dev[addr].cs);
	} else {
		digitalHi(master_dev[addr].cs);
	}
	ss_settle();
}

static int failures;
#define CHECK(cond, ...)                  \
	do {                                  \
		if (!(cond)) {                    \
			failures++;                   \
			tprintf("  FAIL " __VA_ARGS__); \
			tprintf("\n");                \
		}                                 \
	} while (0)

static uint16_t rregs(int dev, uint8_t reg, uint8_t *val, int n) {
	uint8_t buf[32];
	int nd = master_dev[dev].ndummy;
	buf[0] = 0x80 | reg;
	for (int i = 0; i < 1 + nd + n; i++) {
		buf[1 + i] = 0;
	}
	uint16_t r = spiq_xmit(&spiq, dev, 1 + nd + n, buf);
	for (int i = 0; i < n; i++) {
		val[i] = buf[1 + nd + i];
	}
	pump();
	return r;
}

static uint8_t rreg(int dev, uint8_t reg) {
	uint8_t v = 0xEE;
	rregs(dev, reg, &v, 1);
	return v;
}

static void wreg(int dev, uint8_t reg, uint8_t val) {
	uint8_t buf[2] = {reg, val};
	spiq_xmit(&spiq, dev, 2, buf);
	pump();
}

// write a config list, then read every entry back (the mimic's discipline)
struct cfg {
	uint8_t reg, val;
};
static void config_and_verify(int dev, const struct cfg *c, int n, const char *what) {
	for (int i = 0; i < n; i++) {
		wreg(dev, c[i].reg, c[i].val);
	}
	for (int i = 0; i < n; i++) {
		uint8_t v = rreg(dev, c[i].reg);
		CHECK(v == c[i].val, "%s config reg %02x: %02x != %02x", what, c[i].reg, v, c[i].val);
	}
}

// ---- per-device test scripts --------------------------------------------------

static void test_gyro(void) {
	tprintf("gyro:");
	CHECK(rreg(DEV_GYRO, 0x00) == 0x0F, "chip id");
	// self-test dialogue
	wreg(DEV_GYRO, 0x3C, 0x01);
	delay_us(10000);
	uint8_t st = rreg(DEV_GYRO, 0x3C);
	CHECK((st & 0x16) == 0x12, "self test reg %02x", st);
	// soft reset restores the reset state
	wreg(DEV_GYRO, 0x14, 0xB6);
	delay_us(30000);
	CHECK(rreg(DEV_GYRO, 0x00) == 0x0F, "chip id after reset");
	CHECK(rreg(DEV_GYRO, 0x10) == 0x80, "bandwidth reset value");
	static const struct cfg cfg[] = {
		{0x0F, 0x00}, {0x10, 0x81}, {0x16, 0x00}, {0x18, 0x01}, {0x15, 0x80},
	};
	config_and_verify(DEV_GYRO, cfg, 5, "gyro");
	// rates flow: 2 kHz drdy, INT3 active low
	delay_us(2000);
	uint8_t xyz[6];
	rregs(DEV_GYRO, 0x02, xyz, 6);
	int16_t x = (int16_t)decode_le_uint16(&xyz[0]);
	CHECK(x == 0, "rate x %d", x);
	// INT3: goes active (low) on a sample, auto-releases within ~400 us
	delay_us(600); // beyond a 2 kHz period: a fresh sample has fired the pin
	int seen_active = !digitalIn(DRDY_GYRO);
	delay_us(1000);
	pump();
	CHECK(seen_active, "INT3 never went active");
	tprintf(" ok (frames %u)\n", (unsigned)gyro_dev.frames);
}

static void test_accel(void) {
	tprintf("accel:");
	(void)rreg(DEV_ACC, 0x00); // the wake read (I2C-mode quirk, not emulated)
	CHECK(rreg(DEV_ACC, 0x00) == 0x1E, "chip id");
	CHECK(rreg(DEV_ACC, 0x02) == 0x00, "err reg");
	// power-on dance
	wreg(DEV_ACC, 0x7D, 0x04);
	delay_us(50000);
	wreg(DEV_ACC, 0x7C, 0x00);
	delay_us(5000);
	// self-test: positive minus negative deflection >= 2048 LSB
	wreg(DEV_ACC, 0x41, 0x03);
	wreg(DEV_ACC, 0x40, 0xAC);
	delay_us(3000);
	uint8_t raw[6];
	wreg(DEV_ACC, 0x6D, 0x0D);
	delay_us(51000);
	rregs(DEV_ACC, 0x12, raw, 6);
	int16_t pos[3];
	for (int a = 0; a < 3; a++) {
		pos[a] = (int16_t)decode_le_uint16(&raw[2 * a]);
	}
	wreg(DEV_ACC, 0x6D, 0x09);
	delay_us(51000);
	rregs(DEV_ACC, 0x12, raw, 6);
	for (int a = 0; a < 3; a++) {
		int16_t neg = (int16_t)decode_le_uint16(&raw[2 * a]);
		CHECK((int32_t)pos[a] - neg >= (a < 2 ? 2048 : 1024), "selftest axis %d: %d - %d", a, pos[a], neg);
	}
	wreg(DEV_ACC, 0x6D, 0x00);
	// soft reset -> suspended again, then the real config
	wreg(DEV_ACC, 0x7E, 0xB6);
	delay_us(30000);
	CHECK(rreg(DEV_ACC, 0x7C) == 0x03, "pwr_conf reset (suspended)");
	wreg(DEV_ACC, 0x7D, 0x04);
	delay_us(50000);
	wreg(DEV_ACC, 0x7C, 0x00);
	delay_us(5000);
	static const struct cfg cfg[] = {
		{0x41, 0x03}, {0x40, 0xAC}, {0x53, 0x08}, {0x58, 0x04},
	};
	config_and_verify(DEV_ACC, cfg, 4, "accel");
	// 1 g on Z at 24 g range
	delay_us(3000);
	rregs(DEV_ACC, 0x12, raw, 6);
	int16_t z = (int16_t)decode_le_uint16(&raw[4]);
	int16_t want = (int16_t)(32767.0f / 24.0f);
	CHECK(z > want - 8 && z < want + 8, "1g on Z: %d != ~%d", z, want);
	// INT1 (configured active low): asserted by a fresh sample, released by
	// reading INT_STAT_1
	delay_us(1500);
	CHECK(!digitalIn(DRDY_ACC), "INT1 not asserted");
	(void)rreg(DEV_ACC, 0x1D);
	CHECK(digitalIn(DRDY_ACC), "INT1 not released by INT_STAT_1 read");
	tprintf(" ok (frames %u)\n", (unsigned)accel_dev.frames);
}

static void test_baro(void) {
	tprintf("baro:");
	CHECK(rreg(DEV_BARO, 0x00) == 0x60, "chip id");
	// the NVM trim must serialize exactly as the model's trim
	uint8_t trim[21], want[21];
	rregs(DEV_BARO, 0x31, trim, 21);
	bmp390_trim_regs(&baro_trim, want);
	for (int i = 0; i < 21; i++) {
		CHECK(trim[i] == want[i], "trim byte %d: %02x != %02x", i, trim[i], want[i]);
	}
	static const struct cfg cfg[] = {
		{0x1B, 0x33}, {0x1C, 0x00}, {0x1D, 0x00}, {0x1F, 0x00}, {0x19, 0x41},
	};
	config_and_verify(DEV_BARO, cfg, 5, "baro");
	delay_us(12000); // > one 200 Hz sample
	uint8_t st = rreg(DEV_BARO, 0x03);
	CHECK((st & 0x60) == 0x60, "status drdy %02x", st);
	// read the raw words and compensate them the way the DUT would
	uint8_t raw[6];
	rregs(DEV_BARO, 0x04, raw, 6);
	uint32_t praw = decode_le_uint24(&raw[0]);
	uint32_t traw = decode_le_uint24(&raw[3]);
	struct BMP390Cal cal;
	bmp390_cal_init(&cal, &baro_trim);
	double T, P;
	bmp390_forward(&cal, traw, praw, &T, &P);
	CHECK(P > 101325.0 - 2 && P < 101325.0 + 2, "P %d Pa", (int)P);
	CHECK(T > 15.0 - 0.05 && T < 15.0 + 0.05, "T %d cdeg", (int)(T * 100));
	// INT pin (configured active low, drdy_en): asserted after a fresh
	// sample, released by reading INT_STATUS
	delay_us(6000);
	CHECK(!digitalIn(DRDY_BARO), "INT not asserted");
	(void)rreg(DEV_BARO, 0x11);
	CHECK(digitalIn(DRDY_BARO), "INT not released by INT_STATUS read");
	tprintf(" ok (frames %u)\n", (unsigned)baro_dev.frames);
}

static void test_mag(void) {
	tprintf("mag:");
	CHECK(rreg(DEV_MAG, 0x36) == 0x22, "revid");
	// whoami the ArduPilot way: CC registers at their power-on defaults
	uint8_t cc[6];
	rregs(DEV_MAG, 0x04, cc, 6);
	for (int a = 0; a < 3; a++) {
		CHECK(decode_be_uint16(&cc[2 * a]) == 200, "CC%c default", "XYZ"[a]);
	}
	// BIST: STE + POLL -> XOK/YOK/ZOK
	wreg(DEV_MAG, 0x33, 0x8F);
	wreg(DEV_MAG, 0x00, 0x70);
	delay_us(10000);
	uint8_t bist = rreg(DEV_MAG, 0x33);
	CHECK((bist & 0x70) == 0x70, "BIST %02x", bist);
	wreg(DEV_MAG, 0x00, 0x00);
	wreg(DEV_MAG, 0x33, 0x00);
	// configure CC = 150, TMRC 300 Hz, then start continuous mode
	static const struct cfg cfg[] = {
		{0x04, 0x00}, {0x05, 150}, {0x06, 0x00}, {0x07, 150}, {0x08, 0x00}, {0x09, 150}, {0x0B, 0x93},
	};
	config_and_verify(DEV_MAG, cfg, 7, "mag");
	wreg(DEV_MAG, 0x01, 0x71); // CMM: START | XYZ
	delay_us(8000);            // > one 300 Hz period
	CHECK(rreg(DEV_MAG, 0x34) & 0x80, "status drdy");
	// DRDY pin is set by the sample; reading the measurements clears it (DRC1)
	delay_us(4000);
	CHECK(digitalIn(DRDY_MAG), "DRDY not asserted");
	uint8_t meas[9];
	rregs(DEV_MAG, 0x24, meas, 9);
	CHECK(!digitalIn(DRDY_MAG), "DRDY not cleared by measurement read");
	float lsb = 0.3671f * 150.0f + 1.5f;
	int32_t x = (int32_t)decode_be_int24(&meas[0]);
	int32_t z = (int32_t)decode_be_int24(&meas[6]);
	CHECK(x > (int32_t)(20 * lsb) - 3 && x < (int32_t)(20 * lsb) + 3, "field x %d", (int)x);
	CHECK(z > (int32_t)(44 * lsb) - 3 && z < (int32_t)(44 * lsb) + 3, "field z %d", (int)z);
	tprintf(" ok (frames %u)\n", (unsigned)mag_dev.frames);
}

// ---- bring-up ----------------------------------------------------------------

// IRQ priorities, 2:2 grouping, same scheme as the harness proper: SPI3 and
// the CS EXTIs share group 0 (they may never preempt each other — both mutate
// the spislave state), the master's SPI1 RX DMA at group 1, console at 2.
enum { IRQ_PRIORITY_GROUPING_2_2 = 5 };
#define PRIO(grp, sub) ((grp) << 2 | (sub))
static const struct {
	enum IRQn_Type irq;
	uint8_t prio;
} irqprios[] = {
	{SPI3_IRQn, PRIO(0, 0)},      // byte-0 cmd decode, ~72-cycle deadline
	{EXTI0_IRQn, PRIO(0, 1)},     // CS baro
	{EXTI1_IRQn, PRIO(0, 1)},     // CS mag
	{EXTI4_IRQn, PRIO(0, 1)},     // CS gyro
	{EXTI15_10_IRQn, PRIO(0, 1)}, // CS accel

	{DMA1_CH3_IRQn, PRIO(1, 0)}, // master SPI1 RX done

	{DMA1_CH1_IRQn, PRIO(2, 0)}, // console TX DMA
	{USART1_IRQn, PRIO(2, 0)},   // console TX kick
};
#undef PRIO

// master-side pins on top of the harness pinout
static const pinconf_t master_pins[] = {
	PA5_SPI1_SCK | PIN_HIGH,      //% master SCK, jumper to PC10
	PA6_SPI1_MISO | PIN_PULLDOWN, //% master MISO, jumper to PC11
	PA7_SPI1_MOSI | PIN_HIGH,     //% master MOSI, jumper to PC12
	PB0 | PIN_OUTPUT | PIN_HIGH,  //% CS baro out, jumper to PC0
	PB1 | PIN_OUTPUT | PIN_HIGH,  //% CS mag out, jumper to PC1
	PA2 | PIN_OUTPUT | PIN_HIGH,  //% CS gyro out, jumper to PA4
	PA3 | PIN_OUTPUT | PIN_HIGH,  //% CS accel out, jumper to PB12
};

void Reset_Handler(void) __attribute__((noreturn));
void Reset_Handler(void) {
	narray_init_memory();
	SCB.VTOR = (uint32_t)(uintptr_t)__vectors;
	*(volatile uint32_t *)0xE000ED88 |= 0xfu << 20;
	SCB.SHCSR |= SCB_SHCSR_USGFAULTENA;
	SCB.CCR |= SCB_CCR_DIV_0_TRP;

	board_init(); // harness clock + pinout (incl. the sensor bus + CS inputs)

	nvic_set_priority_grouping(IRQ_PRIORITY_GROUPING_2_2);
	for (size_t i = 0; i < sizeof irqprios / sizeof irqprios[0]; i++) {
		nvic_set_priority(irqprios[i].irq, irqprios[i].prio);
	}

	RCC.APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	digitalHi(PB0 | PB1);
	digitalHi(PA2 | PA3);
	gpioConfigAll(master_pins, sizeof master_pins / sizeof master_pins[0]);

	dma_set_mux(DMA1_CH1, DMA_REQ_USART1_TX);
	usart_init_tx(&USART1, clock_usart_hz(1), 115200);
	console = &vcp;
	nvic_enable(DMA1_CH1_IRQn);
	nvic_enable(USART1_IRQn);

	TIM7.PSC = 168 - 1; // 1 MHz
	TIM7.ARR = 0xFFFF;
	TIM7.CR1 = TIM_BASIC_INST_CR1_CEN;

	// the slave side, exactly as the harness runs it
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

	// the master: 1.3 MHz, the no-dummy protocols' measured clean domain
	dma_set_mux(DMA1_CH3, DMA_REQ_SPI1_RX);
	dma_set_mux(DMA1_CH4, DMA_REQ_SPI1_TX);
	spiq_init(&spiq, &SPI1, SPI_CR1_BR_Div128, DMA1_CH3, DMA1_CH4, spi1_ss);
	nvic_enable(DMA1_CH3_IRQn);

	fault_report(cputc);
	tprintf("\nM4a sensor-model self-test, sysclk %u Hz, bus %u Hz\n"
	        "jumpers: PA5>PC10 PA7>PC12 PA6>PC11 PB0>PC0 PB1>PC1 PA2>PA4 PA3>PB12\n",
	        (unsigned)clock_sysclk_hz(), 168000000u / 128);

	test_gyro();
	test_accel();
	test_baro();
	test_mag();

	tprintf("stray %u overlap %u unexpected %u+%u+%u+%u\n",
	        (unsigned)sensor_bus.stray, (unsigned)sensor_bus.overlap,
	        (unsigned)gyro_dev.unexpected, (unsigned)accel_dev.unexpected,
	        (unsigned)baro_dev.unexpected, (unsigned)mag_dev.unexpected);
	tprintf("%s (%d failures)\n", failures == 0 ? "PASS" : "FAIL", failures);

	for (uint32_t spin = 0;; spin++) {
		if (spin >= 20000000) {
			spin = 0;
			digitalToggle(LED);
		}
		pump();
	}
}

static void dma1_ch1(void) { serial_dma_tx_handler(&vcp, DMA1_CH1); }
static void usart1(void) { serial_irq_tx_handler(&vcp, DMA1_CH1); }
static void spi1_rx(void) { spi_rx_dma_handler(&spiq); }
static void spi3(void) { spislave_irq(&sensor_bus, &SPI3, DMA1_CH5); }
static void sensor_cs(void) { spislave_cs_handler(&sensor_bus); }

extern void _estack(void);

__attribute__((section(".isr_vector"))) const isr_t __vectors[NVIC_VECTORS] = {
	(isr_t)&_estack,
	Reset_Handler,
	[VECTOR(HardFault_IRQn)] = HardFault_Handler,
	[VECTOR(MemManage_IRQn)] = MemManage_Handler,
	[VECTOR(BusFault_IRQn)] = BusFault_Handler,
	[VECTOR(UsageFault_IRQn)] = UsageFault_Handler,
	[VECTOR(DMA1_CH1_IRQn)] = dma1_ch1,
	[VECTOR(USART1_IRQn)] = usart1,
	[VECTOR(DMA1_CH3_IRQn)] = spi1_rx,
	[VECTOR(SPI3_IRQn)] = spi3,
	[VECTOR(EXTI0_IRQn)] = sensor_cs,
	[VECTOR(EXTI1_IRQn)] = sensor_cs,
	[VECTOR(EXTI4_IRQn)] = sensor_cs,
	[VECTOR(EXTI15_10_IRQn)] = sensor_cs,
};
