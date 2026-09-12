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
	G_FIFO_WM_ENABLE = 0x1E,
	G_SELF_TEST = 0x3C, G_FIFO_CONFIG0 = 0x3D, G_FIFO_CONFIG1 = 0x3E,
	G_FIFO_DATA = 0x3F,
};
enum { // BMP390
	B_TRIM_CRC = 0x30,
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
	// 0x0F | 0x10,0x11,0x13,0x14,0x15,0x16 | 0x18 | 0x1E | 0x3C,0x3D,0x3E
	[0x0F / 8] = 0x80, [0x10 / 8] = 0x7B, [0x18 / 8] = 0x41, [0x3C / 8] = 0x70,
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
static uint32_t gyro_drdy_set_us, accel_drdy_set_us, baro_drdy_set_us, mag_drdy_set_us;
static volatile bool gyro_drdy_armed, accel_drdy_armed, baro_drdy_armed, mag_drdy_armed;
static volatile bool mag_poll_pending;   // single-shot POLL awaiting a sample
static volatile bool baro_forced_pending; // single-shot FORCED conversion awaiting a sample

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

// The accel FIFO is entered EARLY: PX4's BMI088 driver reads FIFO_LENGTH_0
// (0x24), FIFO_LENGTH_1 and the frames in ONE transaction, counting on the
// real part's address increment to carry it into the 0x26 burst port;
// ArduPilot's reads 0x24-0x25 and 0x26 separately. Both are served by
// mirroring the two length bytes into the head of the stream buffer and
// telling the engine the port may be entered two registers early
// (stream_prefix, see nlib/spislave.h).
#define A_FIFO_PREFIX 2
static struct {
	uint8_t len[A_FIFO_PREFIX]; // mirror of FIFO_LENGTH_0/1 at 0x24/0x25
	uint8_t data[146 * 7];      // ~1 KB, like the part
} accel_fifo;
static_assert(sizeof accel_fifo == A_FIFO_PREFIX + 146 * 7, "no padding: the burst source must be linear");
static uint16_t accel_fifo_fill;
static uint8_t gyro_fifo[100 * 6]; // 100 frames, like the part
static uint16_t gyro_fifo_fill;

static void accel_fifo_sync(struct SPIDev *d) { // fill -> FIFO_LENGTH regs + their mirror
	encode_le_uint16(&d->reg[A_FIFO_LEN0], accel_fifo_fill);
	encode_le_uint16(accel_fifo.len, accel_fifo_fill);
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

// bring-up frame trace: every hook pushes what the engine decoded; the
// heartbeat drains it via sensors_trace_next. Overrun drops the oldest.
static struct FrameTrace {
	uint8_t dev, cmd, addr, len;
} ftrace[128];
static volatile uint32_t ftrace_head;
static uint32_t ftrace_tail;

static void trace_frame(uint8_t dev, uint8_t cmd, uint8_t addr, int len) {
	ftrace[ftrace_head % 128] = (struct FrameTrace){dev, cmd, addr, (uint8_t)len};
	ftrace_head++;
}

int sensors_trace_next(char buf[static 16]) {
	uint32_t head = ftrace_head;
	if (ftrace_tail == head) {
		return 0;
	}
	if (head - ftrace_tail > 128) {
		ftrace_tail = head - 128; // overrun: skip to the oldest survivor
	}
	struct FrameTrace e = ftrace[ftrace_tail % 128];
	ftrace_tail++;
	static const char hex[] = "0123456789abcdef";
	buf[0] = ' ';
	buf[1] = (char)e.dev;
	buf[2] = hex[e.cmd >> 4];
	buf[3] = hex[e.cmd & 15];
	buf[4] = '@';
	buf[5] = hex[e.addr >> 4];
	buf[6] = hex[e.addr & 15];
	buf[7] = '+';
	buf[8] = hex[e.len >> 4];
	buf[9] = hex[e.len & 15];
	buf[10] = 0;
	return 10;
}

static bool frame_reads(uint8_t cmd, uint8_t addr, int len, uint8_t lo, uint8_t hi) {
	return (cmd & 0x80) && addr <= hi && addr + len > lo;
}

static void accel_frame(struct SPIDev *d, uint8_t cmd, uint8_t addr, int len) {
	trace_frame('a', cmd, addr, len);
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
		// FIFO drain (the engine's streaming register). The burst may have
		// been entered up to A_FIFO_PREFIX registers early, in which case
		// those mirrored length bytes are part of len but not of the FIFO.
		if (addr <= A_FIFO_DATA && A_FIFO_DATA - addr <= A_FIFO_PREFIX && addr + len > A_FIFO_DATA) {
			fifo_pop(accel_fifo.data, &accel_fifo_fill, len - (A_FIFO_DATA - addr), 7);
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
	trace_frame('g', cmd, addr, len);
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
	trace_frame('b', cmd, addr, len);
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
	if (addr <= B_PWR_CTRL && addr + len > B_PWR_CTRL) {
		// mode[5:4]: 00 sleep, 01 and 10 FORCED, 11 normal. A forced write runs
		// exactly ONE conversion and the part drops back to sleep by itself;
		// normal mode free-runs at ODR (baro_rate_hz). ArduPilot only ever uses
		// normal mode, PX4's BMP388 driver only ever uses forced.
		uint8_t mode = d->reg[B_PWR_CTRL] & 0x30;
		if ((mode == 0x10 || mode == 0x20) && (d->reg[B_PWR_CTRL] & 0x03)) {
			baro_forced_pending = true;
		}
	}
	if (addr <= B_CMD && addr + len > B_CMD) {
		if (d->reg[B_CMD] == 0xB6) {
			baro_reset_req = true;
		}
		d->reg[B_CMD] = 0; // CMD always reads as 0x00
	}
}

static void mag_frame(struct SPIDev *d, uint8_t cmd, uint8_t addr, int len) {
	trace_frame('m', cmd, addr, len);
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
	// FIFO_WM_ENABLE: stored and echoed only. The watermark interrupt it arms
	// is an INT3/INT4 pin function and those pins are not wired to the DUT, so
	// there is no side effect to model — but the register must EXIST, because a
	// driver that writes it reads it straight back and refuses to run on a
	// mismatch (PX4's BMI088 gyro does exactly that; ArduPilot never touches it).
	d->reg[G_FIFO_WM_ENABLE] = 0x08;
	gyro_fifo_fill = 0;
}

// BMP3xx NVM trim checksum (register 0x30), the algorithm in Bosch's
// bmp3_selftest.c: CRC-8 poly 0x1D, seed 0xFF, final complement, over the 21
// trim bytes at 0x31..0x45. A driver that validates the calibration this way
// refuses to configure the part when it mismatches (PX4's BMP388 driver does;
// ArduPilot's does not read 0x30 at all), so the emulated part has to carry it.
static uint8_t bmp3_trim_crc(const uint8_t *trim) {
	uint8_t crc = 0xFF;
	for (int i = 0; i < 21; i++) {
		uint8_t data = trim[i];
		for (int b = 0; b < 8; b++) {
			bool feedback = ((crc ^ data) & 0x80) != 0;
			crc <<= 1;
			data <<= 1;
			if (feedback) {
				crc ^= 0x1D;
			}
		}
	}
	return crc ^ 0xFF;
}

static void baro_reset(struct SPIDev *d) {
	for (int i = 0; i < SPIDEV_REGS; i++) {
		d->reg[i] = 0;
	}
	d->reg[B_CHIP_ID] = 0x50; // BMP388, like the bench part (BMP390 = 0x60)
	d->reg[B_REV_ID] = 0x01;
	d->reg[B_STATUS] = 0x10;  // cmd_rdy
	d->reg[B_EVENT] = 0x01;   // por_detected
	d->reg[B_INT_CTRL] = 0x02;
	d->reg[0x17] = 0x02; // FIFO_CONFIG_1 reset
	d->reg[0x18] = 0x02; // FIFO_CONFIG_2 reset
	d->reg[B_OSR] = 0x02;
	d->reg[0x15] = 0x01; // FIFO_WTM_0 reset
	bmp390_trim_regs(&baro_trim, &d->reg[B_TRIM]);
	d->reg[B_TRIM_CRC] = bmp3_trim_crc(&d->reg[B_TRIM]);
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

// the REAL trim of the bench BMP388 (BCLBR dump 2026-07-12: 219 106 129 73
// 246 66 253 123 243 35 0 51 103 179 124 243 246 49 65 11 196) — the
// emulated NVM is byte-identical to the actual part on the bench
const struct BMP390Trim baro_trim = {
	.par_t1 = 27355, .par_t2 = 18817, .par_t3 = -10,
	.par_p1 = -702, .par_p2 = -3205, .par_p3 = 35, .par_p4 = 0,
	.par_p5 = 26419, .par_p6 = 31923, .par_p7 = -13, .par_p8 = -10,
	.par_p9 = 16689, .par_p10 = 11, .par_p11 = -60,
};
static struct BMP390Cal baro_cal;

// devs[] in ascending CS pin order: PC0, PC1, PA4, PB12
struct SPIDev baro_dev = {.cs = CS_BARO, .read_cmd_mask = 0x80, .ndummy = 1, .wmask = baro_wmask, .frame = baro_frame};
struct SPIDev mag_dev = {.cs = CS_MAG, .read_cmd_mask = 0x80, .ndummy = 0, .wmask = mag_wmask, .frame = mag_frame};
struct SPIDev gyro_dev = {.cs = CS_GYRO, .read_cmd_mask = 0x80, .ndummy = 0, .wmask = gyro_wmask, .frame = gyro_frame,
                          .stream = gyro_fifo, .stream_size = sizeof gyro_fifo, .stream_addr = G_FIFO_DATA};
struct SPIDev accel_dev = {.cs = CS_ACC, .read_cmd_mask = 0x80, .ndummy = 1, .wmask = accel_wmask, .frame = accel_frame,
                           .stream = (uint8_t *)&accel_fifo, .stream_size = sizeof accel_fifo,
                           .stream_addr = A_FIFO_DATA, .stream_prefix = A_FIFO_PREFIX};

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
	// The other DRDY pins auto-release too, mirroring the real parts'
	// non-latched per-sample pulses. Without this a level that asserted
	// before the DUT armed its EXTIs never produces an edge and never gets
	// read — the pin must re-pulse on every commit. The latched status
	// registers keep their clear-on-read semantics (frame hooks).
	if (accel_drdy_armed && now_us - accel_drdy_set_us > 300) {
		accel_drdy_armed = false;
		accel_dev.reg[A_INT_STAT_1] = 0;
		if (accel_dev.reg[A_INT_MAP_DATA] & 0x04) {
			(accel_dev.reg[A_INT1_IO_CTRL] & 0x02) ? digitalLo(DRDY_ACC) : digitalHi(DRDY_ACC);
		}
	}
	if (baro_drdy_armed && now_us - baro_drdy_set_us > 300) {
		baro_drdy_armed = false;
		if (baro_dev.reg[B_INT_CTRL] & 0x40) {
			(baro_dev.reg[B_INT_CTRL] & 0x02) ? digitalLo(DRDY_BARO) : digitalHi(DRDY_BARO);
		}
	}
	if (mag_drdy_armed && now_us - mag_drdy_set_us > 300) {
		mag_drdy_armed = false;
		digitalLo(DRDY_MAG); // active high; M_STATUS stays until the HSHAKE clear
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
	if (baro_forced_pending) {
		return 1000; // serve the single-shot on the next sampler tick
	}
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
	// TMRC only caps the rate; the cycle counts set the actual measurement
	// duration over the three axes: ~30000/CC Hz (CC=200 -> ~150 Hz,
	// CC=150 -> ~200 Hz, matching the real part on the bench)
	uint32_t cc = ((uint32_t)mag_dev.reg[M_CC] << 8) | mag_dev.reg[M_CC + 1];
	if (cc == 0) {
		cc = 200; // reset default
	}
	uint32_t meas = 30000 / cc;
	uint8_t tmrc = mag_dev.reg[M_TMRC];
	uint32_t cap = (tmrc < 0x92 || tmrc > 0x9D) ? 37 : (600u >> (tmrc - 0x92));
	return meas < cap ? meas : cap;
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
		if (accel_fifo_fill + 7u <= sizeof accel_fifo.data) {
			accel_fifo.data[accel_fifo_fill] = 0x84; // accel data frame header
			for (int i = 0; i < 6; i++) {
				accel_fifo.data[accel_fifo_fill + 1 + i] = buf[i];
			}
			accel_fifo_fill += 7;
			accel_fifo_sync(d);
		} // full: stop-at-full drops silently
	}
}

bool accel_commit(const int16_t xyz[3], float t_degc, uint32_t now_us) {
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
		accel_drdy_set_us = now_us;
		accel_drdy_armed = true;
	}
	return true;
}

bool baro_commit(float t_degc, double p_pa, uint32_t now_us) {
	if (baro_rate_hz() == 0) {
		return false;
	}
	// The pressure arrives already dithered: the sensor noise model lives
	// with the other three in main.c (sample_baro), not here. This layer
	// only quantizes per the live config, as it does for gyro/accel/mag.
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
	if (baro_forced_pending) {
		baro_forced_pending = false;
		baro_dev.reg[B_PWR_CTRL] &= ~0x30; // the forced conversion is done: back to sleep
	}
	if (baro_dev.reg[B_INT_CTRL] & 0x40) { // drdy_en
		baro_dev.reg[B_INT_STATUS] |= 0x08;
		(baro_dev.reg[B_INT_CTRL] & 0x02) ? digitalHi(DRDY_BARO) : digitalLo(DRDY_BARO);
		baro_drdy_set_us = now_us;
		baro_drdy_armed = true;
	}
	return true;
}

bool mag_commit(const int32_t xyz[3], uint32_t now_us) {
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
	mag_drdy_set_us = now_us;
	mag_drdy_armed = true;
	return true;
}
