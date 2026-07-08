// sensors.c — the four emulated devices, register maps per doc/REGMAPS.md.

#include "sensors.h"

#include "binary.h"
#include "board.h"
#include "gpio.h"

// ---- register addresses (verified against the datasheets, REGMAPS.md) ------

enum { // BMI088 accel
	A_CHIP_ID = 0x00, A_ERR_REG = 0x02, A_STATUS = 0x03, A_DATA = 0x12,
	A_SENSORTIME = 0x18, A_INT_STAT_1 = 0x1D, A_TEMP_MSB = 0x22, A_TEMP_LSB = 0x23,
	A_FIFO_LEN0 = 0x24, A_FIFO_LEN1 = 0x25, A_FIFO_DATA = 0x26,
	A_CONF = 0x40, A_RANGE = 0x41, A_FIFO_DOWNS = 0x45, A_FIFO_WTM0 = 0x46,
	A_FIFO_WTM1 = 0x47, A_FIFO_CONFIG0 = 0x48, A_FIFO_CONFIG1 = 0x49,
	A_INT1_IO_CTRL = 0x53, A_INT2_IO_CTRL = 0x54,
	A_INT_MAP_DATA = 0x58, A_SELF_TEST = 0x6D, A_PWR_CONF = 0x7C, A_PWR_CTRL = 0x7D,
	A_SOFTRESET = 0x7E,
};
enum { // BMI088 gyro
	G_CHIP_ID = 0x00, G_RATE = 0x02, G_INT_STAT_1 = 0x0A, G_FIFO_STATUS = 0x0E,
	G_RANGE = 0x0F, G_BANDWIDTH = 0x10, G_LPM1 = 0x11, G_RATE_HBW = 0x13,
	G_SOFTRESET = 0x14, G_INT_CTRL = 0x15, G_IO_CONF = 0x16, G_IO_MAP = 0x18,
	G_SELF_TEST = 0x3C, G_FIFO_CONFIG0 = 0x3D, G_FIFO_CONFIG1 = 0x3E,
	G_FIFO_DATA = 0x3F,
};
enum { // BMP390
	B_CHIP_ID = 0x00, B_REV_ID = 0x01, B_ERR_REG = 0x02, B_STATUS = 0x03,
	B_PRESS = 0x04, B_TEMP = 0x07, B_SENSORTIME = 0x0C, B_EVENT = 0x10,
	B_INT_STATUS = 0x11, B_INT_CTRL = 0x19, B_IF_CONF = 0x1A, B_PWR_CTRL = 0x1B,
	B_OSR = 0x1C, B_ODR = 0x1D, B_CONFIG = 0x1F, B_TRIM = 0x31, B_CMD = 0x7E,
};
enum { // RM3100
	M_POLL = 0x00, M_CMM = 0x01, M_CC = 0x04, M_TMRC = 0x0B, M_MEAS = 0x24,
	M_BIST = 0x33, M_STATUS = 0x34, M_HSHAKE = 0x35, M_REVID = 0x36,
};

// ---- write masks (which registers the DUT may write) ------------------------

#define BIT(a) [(a) / 8] |= 1u << ((a) % 8) // (documentation; masks are literal below)

static const uint8_t accel_wmask[SPIDEV_REGS / 8] = {
	// 0x40,0x41,0x45..0x47 | 0x48,0x49 | 0x53,0x54,0x58 | 0x6D | 0x7C,0x7D,0x7E
	[0x40 / 8] = 0xE3, [0x48 / 8] = 0x03, [0x53 / 8] = 0x18, [0x58 / 8] = 0x01,
	[0x6D / 8] = 0x20, [0x7C / 8] = 0x70,
};
static const uint8_t gyro_wmask[SPIDEV_REGS / 8] = {
	// 0x0F | 0x10,0x11,0x13,0x14,0x15,0x16 | 0x18 | 0x3C,0x3D,0x3E
	[0x0F / 8] = 0x80, [0x10 / 8] = 0x7B, [0x18 / 8] = 0x01, [0x3C / 8] = 0x70,
};
static const uint8_t baro_wmask[SPIDEV_REGS / 8] = {
	// 0x15..0x1A (FIFO wtm+cfg, INT_CTRL, IF_CONF) | 0x1B..0x1D | 0x1F | 0x7E
	[0x15 / 8] = 0xE0, [0x18 / 8] = 0xBF, [0x7E / 8] = 0x40,
};
static const uint8_t mag_wmask[SPIDEV_REGS / 8] = {
	// 0x00,0x01 | 0x04..0x09 | 0x0B | 0x33,0x35
	[0x00 / 8] = 0xF3, [0x08 / 8] = 0x0B, [0x33 / 8] = 0x28,
};

// ---- deferred work flags (set in frame hooks, handled by sensors_poll) -----

static volatile bool accel_reset_req, gyro_reset_req, baro_reset_req;
static uint32_t gyro_drdy_set_us;
static volatile bool gyro_drdy_armed;
static volatile bool mag_poll_pending; // single-shot POLL awaiting a sample

// ---- the BMI088 FIFOs (M4b, spec = the ArduPilot driver) --------------------
// Served through the engine's streaming register; content stays flat at the
// buffer head, pops memmove the remainder forward (deselect hook, group-0
// exclusive with the bus IRQs). Pops round len DOWN to whole frames: the
// engine's len may overcount by the TX FIFO prefetch, and the driver only
// reads whole-frame multiples. Accel: headered 7-byte frames (0x84 + xyz LE),
// the only frame type this model emits; drops silently at full (stop-at-full
// mode, and the driver has no accel overrun path). Gyro: headerless 6-byte
// frames; at full the FIFO_STATUS overrun bit is raised, which the driver
// answers by rewriting FIFO_CONFIG1 — any write there clears the FIFO.

static uint8_t accel_fifo[146 * 7]; // ~1 KB, like the part
static uint16_t accel_fifo_fill;
static uint8_t gyro_fifo[100 * 6]; // 100 frames, like the part
static uint16_t gyro_fifo_fill;

static void accel_fifo_sync(struct SPIDev *d) { // fill -> FIFO_LENGTH regs
	encode_le_uint16(&d->reg[A_FIFO_LEN0], accel_fifo_fill);
}

static void gyro_fifo_sync(struct SPIDev *d) { // fill -> frame count, keep overrun
	d->reg[G_FIFO_STATUS] = (d->reg[G_FIFO_STATUS] & 0x80) | (uint8_t)(gyro_fifo_fill / 6);
}

static void fifo_pop(uint8_t *fifo, uint16_t *fill, int len, int framesz) {
	int n = (len / framesz) * framesz;
	if (n > *fill) {
		n = *fill;
	}
	for (int i = n; i < *fill; i++) {
		fifo[i - n] = fifo[i];
	}
	*fill -= (uint16_t)n;
}

// ---- frame hooks: clear-on-read, DRDY release, write side effects ----------
// Run at deselect (priority 0): register flips and flags only.

static bool frame_reads(uint8_t cmd, uint8_t addr, int len, uint8_t lo, uint8_t hi) {
	return (cmd & 0x80) && addr <= hi && addr + len > lo;
}

static void accel_frame(struct SPIDev *d, uint8_t cmd, uint8_t addr, int len) {
	if (cmd & 0x80) {
		if (frame_reads(cmd, addr, len, A_DATA, A_DATA + 5)) {
			d->reg[A_STATUS] &= ~0x80; // drdy_acc: reset on data read
		}
		if (frame_reads(cmd, addr, len, A_INT_STAT_1, A_INT_STAT_1)) {
			d->reg[A_INT_STAT_1] = 0; // acc_drdy: cleared on read of this register
			// INT1 pin follows the latched status
			if (d->reg[A_INT_MAP_DATA] & 0x04) {
				(d->reg[A_INT1_IO_CTRL] & 0x02) ? digitalLo(DRDY_ACC) : digitalHi(DRDY_ACC);
			}
		}
		if (addr == A_FIFO_DATA) { // FIFO drain (the engine's streaming register)
			fifo_pop(accel_fifo, &accel_fifo_fill, len, 7);
			accel_fifo_sync(d);
		}
		return;
	}
	// writes are already stored through the wmask; side effects:
	if (addr <= A_FIFO_CONFIG1 && addr + len > A_FIFO_CONFIG0) {
		accel_fifo_fill = 0; // FIFO config writes clear the FIFO
		accel_fifo_sync(d);
	}
	if (addr <= A_SOFTRESET && addr + len > A_SOFTRESET && d->reg[A_SOFTRESET] == 0xB6) {
		accel_reset_req = true;
	}
}

static void gyro_frame(struct SPIDev *d, uint8_t cmd, uint8_t addr, int len) {
	if (cmd & 0x80) {
		if (addr == G_FIFO_DATA) { // FIFO drain (the engine's streaming register)
			fifo_pop(gyro_fifo, &gyro_fifo_fill, len, 6);
			gyro_fifo_sync(d);
		}
		return; // gyro drdy is time-cleared, not read-cleared
	}
	if (addr <= G_FIFO_CONFIG1 && addr + len > G_FIFO_CONFIG1) {
		gyro_fifo_fill = 0; // FIFO_CONFIG1 writes clear the FIFO and the overrun
		d->reg[G_FIFO_STATUS] = 0;
	}
	if (addr <= G_SOFTRESET && addr + len > G_SOFTRESET && d->reg[G_SOFTRESET] == 0xB6) {
		gyro_reset_req = true;
	}
	if (addr <= G_SELF_TEST && addr + len > G_SELF_TEST && (d->reg[G_SELF_TEST] & 0x01)) {
		d->reg[G_SELF_TEST] = 0x12; // bist_rdy | rate_ok, bist_fail clear
	}
}

static void baro_frame(struct SPIDev *d, uint8_t cmd, uint8_t addr, int len) {
	if (cmd & 0x80) {
		if (frame_reads(cmd, addr, len, B_PRESS, B_PRESS + 2)) {
			d->reg[B_STATUS] &= ~0x20; // drdy_press
		}
		if (frame_reads(cmd, addr, len, B_TEMP, B_TEMP + 2)) {
			d->reg[B_STATUS] &= ~0x40; // drdy_temp
		}
		if (frame_reads(cmd, addr, len, B_EVENT, B_EVENT)) {
			d->reg[B_EVENT] = 0; // por_detected / itf_act_pt: clear-on-read
		}
		if (frame_reads(cmd, addr, len, B_INT_STATUS, B_INT_STATUS)) {
			d->reg[B_INT_STATUS] = 0; // whole register clears after read
			if (d->reg[B_INT_CTRL] & 0x40) {
				(d->reg[B_INT_CTRL] & 0x02) ? digitalLo(DRDY_BARO) : digitalHi(DRDY_BARO);
			}
		}
		return;
	}
	if (addr <= B_CMD && addr + len > B_CMD) {
		if (d->reg[B_CMD] == 0xB6) {
			baro_reset_req = true;
		}
		d->reg[B_CMD] = 0; // CMD always reads as 0x00
	}
}

static void mag_frame(struct SPIDev *d, uint8_t cmd, uint8_t addr, int len) {
	uint8_t hshake = d->reg[M_HSHAKE];
	if (cmd & 0x80) {
		// DRC1: DRDY cleared by reading the measurement results
		if ((hshake & 0x02) && frame_reads(cmd, addr, len, M_MEAS, M_MEAS + 8)) {
			d->reg[M_STATUS] = 0;
			digitalLo(DRDY_MAG);
		}
		return;
	}
	// DRC0: DRDY cleared by any register write
	if (hshake & 0x01) {
		d->reg[M_STATUS] = 0;
		digitalLo(DRDY_MAG);
	}
	if (addr <= M_POLL && addr + len > M_POLL && (d->reg[M_POLL] & 0x70)) {
		if (d->reg[M_BIST] & 0x80) { // STE set: self-test instead of a measurement
			d->reg[M_BIST] |= 0x70;  // XOK | YOK | ZOK
			d->reg[M_STATUS] = 0x80;
			digitalHi(DRDY_MAG); // DRDY pin is active high on the RM3100
		} else {
			mag_poll_pending = true;
		}
		d->reg[M_POLL] = 0;
	}
}

// ---- reset-state fills -------------------------------------------------------

static void accel_reset(struct SPIDev *d) {
	for (int i = 0; i < SPIDEV_REGS; i++) {
		d->reg[i] = 0;
	}
	d->reg[A_CHIP_ID] = 0x1E;
	d->reg[A_STATUS] = 0x10;
	d->reg[A_CONF] = 0xA8;
	d->reg[A_RANGE] = 0x01;
	d->reg[A_FIFO_CONFIG0] = 0x02;
	d->reg[A_FIFO_CONFIG1] = 0x10; // bit4 reads 1
	d->reg[A_PWR_CONF] = 0x03;     // powers up suspended
	accel_fifo_fill = 0;
}

static void gyro_reset(struct SPIDev *d) {
	for (int i = 0; i < SPIDEV_REGS; i++) {
		d->reg[i] = 0;
	}
	d->reg[G_CHIP_ID] = 0x0F;
	d->reg[G_BANDWIDTH] = 0x80;
	d->reg[G_IO_CONF] = 0x0F;
	gyro_fifo_fill = 0;
}

static void baro_reset(struct SPIDev *d) {
	for (int i = 0; i < SPIDEV_REGS; i++) {
		d->reg[i] = 0;
	}
	d->reg[B_CHIP_ID] = 0x60;
	d->reg[B_REV_ID] = 0x01;
	d->reg[B_STATUS] = 0x10;  // cmd_rdy
	d->reg[B_EVENT] = 0x01;   // por_detected
	d->reg[B_INT_CTRL] = 0x02;
	d->reg[0x17] = 0x02; // FIFO_CONFIG_1 reset
	d->reg[0x18] = 0x02; // FIFO_CONFIG_2 reset
	d->reg[B_OSR] = 0x02;
	d->reg[0x15] = 0x01; // FIFO_WTM_0 reset
	bmp390_trim_regs(&baro_trim, &d->reg[B_TRIM]);
}

static void mag_reset(struct SPIDev *d) {
	for (int i = 0; i < SPIDEV_REGS; i++) {
		d->reg[i] = 0;
	}
	// CC defaults 0x00C8 big-endian per axis — ArduPilot's whoami reads these
	for (int a = 0; a < 3; a++) {
		encode_be_uint16(&d->reg[M_CC + 2 * a], 200);
	}
	d->reg[M_TMRC] = 0x96;
	d->reg[M_HSHAKE] = 0x1B;
	d->reg[M_REVID] = 0x22;
}

// ---- the bus ----------------------------------------------------------------

// the synthetic trim baked into the served NVM; replace with a real dump when
// one is read out (bmp390inv golden validates the inversion for any trim)
const struct BMP390Trim baro_trim = {
	.par_t1 = 27000, .par_t2 = 18000, .par_t3 = -10,
	.par_p1 = 21400, .par_p2 = 16100, .par_p3 = 5, .par_p4 = -3,
	.par_p5 = 7000, .par_p6 = 1400, .par_p7 = 20, .par_p8 = -5,
	.par_p9 = 6000, .par_p10 = 3, .par_p11 = -2,
};
static struct BMP390Cal baro_cal;

// devs[] in ascending CS pin order: PC0, PC1, PA4, PB12
struct SPIDev baro_dev = {.cs = CS_BARO, .read_cmd_mask = 0x80, .ndummy = 1, .wmask = baro_wmask, .frame = baro_frame};
struct SPIDev mag_dev = {.cs = CS_MAG, .read_cmd_mask = 0x80, .ndummy = 0, .wmask = mag_wmask, .frame = mag_frame};
struct SPIDev gyro_dev = {.cs = CS_GYRO, .read_cmd_mask = 0x80, .ndummy = 0, .wmask = gyro_wmask, .frame = gyro_frame,
                          .stream = gyro_fifo, .stream_size = sizeof gyro_fifo, .stream_addr = G_FIFO_DATA};
struct SPIDev accel_dev = {.cs = CS_ACC, .read_cmd_mask = 0x80, .ndummy = 1, .wmask = accel_wmask, .frame = accel_frame,
                           .stream = accel_fifo, .stream_size = sizeof accel_fifo, .stream_addr = A_FIFO_DATA};

static struct SPIDev *const devtab[] = {&baro_dev, &mag_dev, &gyro_dev, &accel_dev};
struct SPISlave sensor_bus = SPISLAVE_INITIALIZER(SPI3, DMA1_CH5, devtab, 4, 0x00);

void sensors_init(void) {
	bmp390_cal_init(&baro_cal, &baro_trim);
	accel_reset(&accel_dev);
	gyro_reset(&gyro_dev);
	baro_reset(&baro_dev);
	mag_reset(&mag_dev);
	spislave_init(&sensor_bus, 0 /* mode 0 */, false /* SSM, CS demux */,
	              &RCC.APB1RSTR1, RCC_APB1RSTR1_SPI3RST);
}

void sensors_poll(uint32_t now_us) {
	if (accel_reset_req) {
		accel_reset_req = false;
		accel_reset(&accel_dev);
	}
	if (gyro_reset_req) {
		gyro_reset_req = false;
		gyro_reset(&gyro_dev);
	}
	if (baro_reset_req) {
		baro_reset_req = false;
		baro_reset(&baro_dev);
		baro_dev.reg[B_EVENT] = 0x01; // por_detected after softreset
	}
	// gyro drdy auto-clears 280-400 µs after being set
	if (gyro_drdy_armed && now_us - gyro_drdy_set_us > 300) {
		gyro_drdy_armed = false;
		gyro_dev.reg[G_INT_STAT_1] = 0;
		if (gyro_dev.reg[G_IO_MAP] & 0x01) { // mapped to INT3
			(gyro_dev.reg[G_IO_CONF] & 0x01) ? digitalLo(DRDY_GYRO) : digitalHi(DRDY_GYRO);
		}
	}
}

// ---- rates and scales from the live registers --------------------------------

uint32_t gyro_rate_hz(void) {
	if (gyro_dev.reg[G_LPM1] != 0) {
		return 0; // suspended
	}
	switch (gyro_dev.reg[G_BANDWIDTH] & 0x7F) {
	case 0x00: case 0x01: return 2000;
	case 0x02: return 1000;
	case 0x03: return 400;
	case 0x04: return 200;
	case 0x05: return 100;
	case 0x06: return 200;
	case 0x07: return 100;
	default: return 2000;
	}
}

uint32_t accel_rate_hz(void) {
	if (accel_dev.reg[A_PWR_CTRL] != 0x04 || accel_dev.reg[A_PWR_CONF] != 0x00) {
		return 0; // off or suspended
	}
	static const uint16_t odr[16] = {
		// odr_sel 0x5..0xC = 12.5,25,50,100,200,400,800,1600 Hz (x2 stored, /2 below)
		[0x5] = 25, [0x6] = 50, [0x7] = 100, [0x8] = 200,
		[0x9] = 400, [0xA] = 800, [0xB] = 1600, [0xC] = 3200,
	};
	return odr[accel_dev.reg[A_CONF] & 0x0F] / 2;
}

uint32_t baro_rate_hz(void) {
	uint8_t pwr = baro_dev.reg[B_PWR_CTRL];
	if ((pwr & 0x30) != 0x30 || (pwr & 0x03) == 0) {
		return 0; // not in normal mode with a measurement enabled
	}
	uint8_t sel = baro_dev.reg[B_ODR] & 0x1F;
	uint32_t hz = 200;
	while (sel-- && hz > 0) {
		hz /= 2; // odr_sel halves per step: 0x00 = 200 Hz, 0x01 = 100 Hz, ...
	}
	return hz ? hz : 1;
}

uint32_t mag_rate_hz(void) {
	if (mag_poll_pending) {
		return 1000; // serve the single-shot on the next sampler tick
	}
	if (!(mag_dev.reg[M_CMM] & 0x01)) {
		return 0; // continuous mode not started
	}
	uint8_t tmrc = mag_dev.reg[M_TMRC];
	if (tmrc < 0x92 || tmrc > 0x9D) {
		return 37;
	}
	return 600u >> (tmrc - 0x92); // ~600 Hz at 0x92, halving per step
}

float gyro_fullscale_dps(void) {
	static const float fs[5] = {2000, 1000, 500, 250, 125};
	uint8_t r = gyro_dev.reg[G_RANGE];
	return fs[r > 4 ? 0 : r];
}

float accel_fullscale_g(void) {
	static const float fs[4] = {3, 6, 12, 24}; // BMI088 encoding
	return fs[accel_dev.reg[A_RANGE] & 3];
}

float mag_lsb_per_ut(void) {
	// gain fits the datasheet's 50/100/200-CC points: 0.3671*CC + 1.5
	uint16_t cc = decode_be_uint16(&mag_dev.reg[M_CC]);
	return 0.3671f * (float)cc + 1.5f;
}

// ---- sample commits (quantized values in, registers + DRDY out) --------------

static void gyro_apply(struct SPIDev *d, void *ctx) {
	const uint8_t *buf = ctx;
	for (int i = 0; i < 6; i++) {
		d->reg[G_RATE + i] = buf[i];
	}
	if (d->reg[G_FIFO_CONFIG1] & 0xC0) { // FIFO or STREAM mode
		if (gyro_fifo_fill + 6u <= sizeof gyro_fifo) {
			for (int i = 0; i < 6; i++) {
				gyro_fifo[gyro_fifo_fill + i] = buf[i];
			}
			gyro_fifo_fill += 6;
		} else {
			d->reg[G_FIFO_STATUS] |= 0x80; // overrun: frame dropped
		}
		gyro_fifo_sync(d);
	}
}

bool gyro_commit(const int16_t xyz[3], uint32_t now_us) {
	if (gyro_rate_hz() == 0) {
		return false;
	}
	uint8_t buf[6];
	for (int a = 0; a < 3; a++) {
		encode_le_uint16(&buf[2 * a], (uint16_t)xyz[a]);
	}
	if (!spidev_apply(&sensor_bus, &gyro_dev, gyro_apply, buf)) {
		return false;
	}
	if (gyro_dev.reg[G_INT_CTRL] & 0x80) {
		gyro_dev.reg[G_INT_STAT_1] = 0x80;
		gyro_drdy_set_us = now_us;
		gyro_drdy_armed = true;
		if (gyro_dev.reg[G_IO_MAP] & 0x01) { // drdy -> INT3, per configured polarity
			(gyro_dev.reg[G_IO_CONF] & 0x01) ? digitalHi(DRDY_GYRO) : digitalLo(DRDY_GYRO);
		}
	}
	return true;
}

static void accel_apply(struct SPIDev *d, void *ctx) {
	const uint8_t *buf = ctx; // xyz[6] + temp[2]
	for (int i = 0; i < 6; i++) {
		d->reg[A_DATA + i] = buf[i];
	}
	d->reg[A_TEMP_MSB] = buf[6];
	d->reg[A_TEMP_LSB] = buf[7];
	d->reg[A_STATUS] |= 0x80;
	if (d->reg[A_FIFO_CONFIG1] & 0x40) { // acc_en: frames flow into the FIFO
		if (accel_fifo_fill + 7u <= sizeof accel_fifo) {
			accel_fifo[accel_fifo_fill] = 0x84; // accel data frame header
			for (int i = 0; i < 6; i++) {
				accel_fifo[accel_fifo_fill + 1 + i] = buf[i];
			}
			accel_fifo_fill += 7;
			accel_fifo_sync(d);
		} // full: stop-at-full drops silently
	}
}

bool accel_commit(const int16_t xyz[3], float t_degc) {
	if (accel_rate_hz() == 0) {
		return false;
	}
	int16_t v[3] = {xyz[0], xyz[1], xyz[2]};
	uint8_t st = accel_dev.reg[A_SELF_TEST];
	if (st == 0x0D || st == 0x09) {
		// self-test deflection: the checker wants pos-neg >= 2048 LSB @16g
		int16_t defl = (st == 0x0D) ? 1500 : -1500;
		v[0] = defl;
		v[1] = defl;
		v[2] = (st == 0x0D) ? 1000 : -1000;
	}
	uint8_t buf[8]; // xyz + the 11-bit temperature (0.125 K/LSB, offset 23 degC)
	for (int a = 0; a < 3; a++) {
		encode_le_uint16(&buf[2 * a], (uint16_t)v[a]);
	}
	int32_t t = (int32_t)((t_degc - 23.0f) * 8.0f);
	if (t > 1023) {
		t = 1023;
	}
	if (t < -1024) {
		t = -1024;
	}
	buf[6] = (uint8_t)((t >> 3) & 0xFF); // TEMP_MSB: MSB at the LOWER address
	buf[7] = (uint8_t)((t & 7) << 5);
	if (!spidev_apply(&sensor_bus, &accel_dev, accel_apply, buf)) {
		return false;
	}
	accel_dev.reg[A_INT_STAT_1] = 0x80;
	if (accel_dev.reg[A_INT_MAP_DATA] & 0x04 && (accel_dev.reg[A_INT1_IO_CTRL] & 0x08)) {
		// drdy mapped to INT1 and INT1 output enabled, per configured polarity
		(accel_dev.reg[A_INT1_IO_CTRL] & 0x02) ? digitalHi(DRDY_ACC) : digitalLo(DRDY_ACC);
	}
	return true;
}

bool baro_commit(float t_degc, double p_pa) {
	if (baro_rate_hz() == 0) {
		return false;
	}
	static uint32_t praw_seed;
	uint32_t traw, praw;
	bmp390_inverse(&baro_cal, t_degc, p_pa, &traw, &praw, praw_seed);
	praw_seed = praw;
	uint8_t buf[6];
	encode_le_uint24(&buf[0], praw);
	encode_le_uint24(&buf[3], traw);
	if (!spidev_commit(&sensor_bus, &baro_dev, B_PRESS, buf, 6)) {
		return false;
	}
	baro_dev.reg[B_STATUS] |= (baro_dev.reg[B_PWR_CTRL] & 0x01 ? 0x20 : 0) |
	                          (baro_dev.reg[B_PWR_CTRL] & 0x02 ? 0x40 : 0);
	if (baro_dev.reg[B_INT_CTRL] & 0x40) { // drdy_en
		baro_dev.reg[B_INT_STATUS] |= 0x08;
		(baro_dev.reg[B_INT_CTRL] & 0x02) ? digitalHi(DRDY_BARO) : digitalLo(DRDY_BARO);
	}
	return true;
}

bool mag_commit(const int32_t xyz[3]) {
	bool single = mag_poll_pending;
	if (!single && !(mag_dev.reg[M_CMM] & 0x01)) {
		return false;
	}
	uint8_t buf[9];
	for (int a = 0; a < 3; a++) {
		encode_be_uint24(&buf[3 * a], (uint32_t)xyz[a]);
	}
	if (!spidev_commit(&sensor_bus, &mag_dev, M_MEAS, buf, 9)) {
		return false;
	}
	mag_poll_pending = false;
	mag_dev.reg[M_STATUS] = 0x80;
	digitalHi(DRDY_MAG); // RM3100 DRDY is active high
	return true;
}
