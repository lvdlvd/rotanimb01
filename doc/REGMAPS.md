# M4 register maps — extracted for verification

Sources, all three in this directory: [BMI088 datasheet rev 1.3](BMI088.pdf)
(§5.2/5.4 maps + §5.3/5.5 descriptions), [BMP390 datasheet rev
1.7](BMP390.pdf) (Table 25 + §4.3, trim Table 24), [RM3100 user manual
V9.0](RM3100.pdf) (Table 5-1 + §5.x). Cross-checked against the hand-extracted
driver headers of an in-house flight controller that masters the real parts.

Access column = what the emulation lets the DUT write (the wmask); "hook"
marks registers whose writes need side effects beyond storage (deselect
frame hook). COR = clear-on-read (also frame-hook work).

## BMI088 accelerometer (SPI2, hw NSS, 1 dummy byte on reads)

| addr      | name              | reset    | access  | model behavior                                                                   |
| --------- | ----------------- | -------- | ------- | -------------------------------------------------------------------------------- |
| 0x00      | ACC_CHIP_ID       | **0x1E** | RO      | probe value (BMI085 would be 0x1F)                                               |
| 0x02      | ACC_ERR_REG       | 0x00     | RO      | 0 (no errors modeled)                                                            |
| 0x03      | ACC_STATUS        | 0x10     | RO      | bit7 drdy_acc, **reset when one acc data reg is read**                           |
| 0x12–0x17 | ACC_X/Y/Z LSB,MSB | 0x00     | RO      | int16 LE per axis, sampler-committed                                             |
| 0x18–0x1A | SENSORTIME_0..2   | 0x00     | RO      | 24-bit LE counter, 1/25.6 kHz units                                              |
| 0x1D      | ACC_INT_STAT_1    | 0x00     | RO      | bit7 acc_drdy, **cleared on read of this register**                              |
| 0x22–0x23 | TEMP_MSB,LSB      | 0x00     | RO      | 11-bit temp, 0.125 K/LSB offset 23 °C; NOTE MSB at the LOWER address             |
| 0x40      | ACC_CONF          | **0xA8** | RW      | bit7 reads 1; osr[6:4], odr[3:0] → drives DRDY rate (12.5 Hz…1.6 kHz = 0x5..0xC) |
| 0x41      | ACC_RANGE         | 0x01     | RW      | range[1:0]: 3/6/12/24 g = 0..3 → LSB/g scale                                     |
| 0x53      | INT1_IO_CTRL      | 0x00     | RW hook | int1_out b3, int1_od b2, int1_lvl b1 → INT1 pin behavior                         |
| 0x54      | INT2_IO_CTRL      | 0x00     | RW hook | idem INT2 (unused by harness wiring)                                             |
| 0x58      | INT_MAP_DATA      | 0x00     | RW hook | int1_drdy b2, int2_drdy b6 → map drdy to pin                                     |
| 0x6D      | ACC_SELF_TEST     | 0x00     | RW hook | 0x0D pos / 0x09 neg / 0x00 off: model deflects data per datasheet §4.6.1         |
| 0x7C      | ACC_PWR_CONF      | **0x03** | RW hook | 0x03 suspend (power-on state!), 0x00 active                                      |
| 0x7D      | ACC_PWR_CTRL      | 0x00     | RW hook | 0x04 accel on, 0x00 off — data stays 0 until on                                  |
| 0x7E      | ACC_SOFTRESET     | 0x00     | WO hook | 0xB6 → full regfile re-init (deferred to thread)                                 |

Semantics the model must honor:
- powers up in **I2C mode until a CSB1 rising edge** (first DUT transaction
  is a throwaway; emulate: before the first deselect, reads return junk?
  → propose: skip emulating, count the frame; VERIFY against AP probe).
- driver **reads back its config writes**: RW regs must store and echo.
- two distinct drdy bits with different clear rules (0x03 bit7 by data-reg
  read; 0x1D bit7 by reading 0x1D) — both from the frame hook.
- data+scale from live ACC_RANGE; DRDY rate from live ACC_CONF.

## BMI088 gyroscope (SPI1, hw NSS, NO dummy byte)

| addr      | name               | reset    | access  | model behavior                                                                                           |
| --------- | ------------------ | -------- | ------- | -------------------------------------------------------------------------------------------------------- |
| 0x00      | GYRO_CHIP_ID       | **0x0F** | RO      | probe value                                                                                              |
| 0x02–0x07 | RATE_X/Y/Z LSB,MSB | n/a      | RO      | int16 LE per axis                                                                                        |
| 0x0A      | GYRO_INT_STAT_1    | n/a      | RO      | bit7 gyro_drdy, **auto-clears after 280–400 µs** (model: clear on a ~300 µs scheduler tick, not on read) |
| 0x0F      | GYRO_RANGE         | 0x00     | RW      | 2000/1000/500/250/125 °/s = 0..4 → LSB/(°/s)                                                             |
| 0x10      | GYRO_BANDWIDTH     | **0x80** | RW      | ODR+filter: 0x80..0x87 (bit7 reads 1); 2000 Hz = 0x80/0x81, 1000 = 0x82, 400 = 0x83 … → drives DRDY rate |
| 0x11      | GYRO_LPM1          | 0x00     | RW hook | 0x00 normal, 0x80 suspend, 0x20 deep susp.                                                               |
| 0x14      | GYRO_SOFTRESET     | n/a      | WO hook | 0xB6 → regfile re-init                                                                                   |
| 0x15      | GYRO_INT_CTRL      | 0x00     | RW hook | 0x80 = drdy interrupt enabled                                                                            |
| 0x16      | INT3_INT4_IO_CONF  | **0x0F** | RW hook | int3_lvl b0, int3_od b1, int4_lvl b2, int4_od b3                                                         |
| 0x18      | INT3_INT4_IO_MAP   | 0x00     | RW hook | 0x01 drdy→INT3, 0x80 →INT4, 0x81 both                                                                    |
| 0x3C      | GYRO_SELF_TEST     | n/a      | RW hook | trig_bist b0 → set bist_rdy b1 (+rate_ok b4), bist_fail=0                                                |

## BMP390 barometer (SPI3/CS PC0, 1 dummy byte on reads)

| addr      | name                 | reset     | access  | model behavior                                                                                |
| --------- | -------------------- | --------- | ------- | --------------------------------------------------------------------------------------------- |
| 0x00      | CHIP_ID              | **0x60**  | RO      | AP's BMP388 driver accepts 0x50 and 0x60                                                      |
| 0x01      | REV_ID               | 0x01      | RO      |                                                                                               |
| 0x02      | ERR_REG              | 0x00      | RO      | fatal b0, cmd_err b1 COR, conf_err b2 COR — model 0                                           |
| 0x03      | STATUS               | —         | RO      | cmd_rdy b4 (=1), drdy_press b5 / drdy_temp b6, **reset when the respective data reg is read** |
| 0x04–0x06 | PRESS xlsb,lsb,msb   | 0x00      | RO      | 24-bit LE raw, from bmp390inv inverse                                                         |
| 0x07–0x09 | TEMP xlsb,lsb,msb    | 0x00      | RO      | idem                                                                                          |
| 0x0C–0x0E | SENSORTIME           | 0x00      | RO      | 24-bit LE counter                                                                             |
| 0x10      | EVENT                | 0x01      | RO      | por_detected b0 **COR**, itf_act_pt b1 COR                                                    |
| 0x11      | INT_STATUS           | 0x00      | RO      | fwm b0, ffull b1, drdy b3 — **whole register clears after read**                              |
| 0x12–0x16 | FIFO_LENGTH/DATA/WTM |           | RO/RW   | FIFO not modeled (AP doesn't use it); reads 0                                                 |
| 0x17–0x18 | FIFO_CONFIG_1/2      | 0x02/0x02 | RW      | stored, ignored                                                                               |
| 0x19      | INT_CTRL             | 0x02      | RW hook | drdy_en b6, int_latch b2, int_level b1, int_od b0 → INT pin                                   |
| 0x1A      | IF_CONF              | 0x00      | RW      | stored (spi3 bit0 must stay 0)                                                                |
| 0x1B      | PWR_CTRL             | 0x00      | RW hook | press_en b0, temp_en b1, mode[5:4]: 00 sleep / 01,10 forced / 11 normal                       |
| 0x1C      | OSR                  | 0x02      | RW      | osr_p[2:0], osr_t[5:3] → conversion time model (rate cap)                                     |
| 0x1D      | ODR                  | 0x00      | RW      | odr_sel[4:0]: 0x00=200 Hz, 0x01=100 Hz, … (÷2 each)                                           |
| 0x1F      | CONFIG               | 0x00      | RW      | iir_filter[3:1] — stored; filtering itself not modeled (VERIFY: AP sets IIR?)                 |
| 0x31–0x45 | NVM trim PAR_T1..P11 | baked     | RO      | Table 24 layout = bmp390inv trim_regs() byte order (T1 LSB at 0x31 … P11 at 0x45)             |
| 0x7E      | CMD                  | 0x00      | WO hook | 0xB6 softreset → re-init + EVENT.por; 0xB0 fifo_flush ignored; **reads as 0x00**              |

## RM3100 magnetometer (SPI3/CS PC1, NO dummy byte, ≤1 MHz)

| addr      | name        | reset     | access  | model behavior                                                                                                             |
| --------- | ----------- | --------- | ------- | -------------------------------------------------------------------------------------------------------------------------- |
| 0x00      | POLL        | 0x00      | RW hook | write PM bits [6:4]=XYZ → one-shot measurement (NACK1 if CMM active)                                                       |
| 0x01      | CMM         | 0x00      | RW hook | START b0, DRDM b2, CMX/Y/Z b4/5/6 → continuous mode                                                                        |
| 0x04–0x09 | CCX/CCY/CCZ | 0x00C8 ×3 | RW      | **big-endian u16 per axis (MSB at lower addr)**, default 200 → gain                                                        |
| 0x0B      | TMRC        | **0x96**  | RW      | CMM rate: 0x92≈600 Hz … 0x96≈37 Hz … halving per step; capped by CC acquisition time                                       |
| 0x24–0x2C | MX/MY/MZ    | 0         | RO      | **24-bit signed big-endian** per axis                                                                                      |
| 0x33      | BIST        | 0x00      | RW hook | self-test bits; model: store, STE→ok bits                                                                                  |
| 0x34      | STATUS      | 0x00      | RO      | bit7 DRDY                                                                                                                  |
| 0x35      | HSHAKE      | **0x1B**  | RW hook | DRC0 b0: DRDY cleared by any reg write; DRC1 b1: cleared by reading MX..MZ (both default 1); NACK0..2 b4..6 RO diagnostics |
| 0x36      | REVID       | **0x22**? | RO      | manual leaves the value open; 0x22 per deployed silicon — CONFIRM against AP driver at M6                                  |

Gain: datasheet points 50/100/200 CC → 20/38/75 LSB/µT; linear fit
gain ≈ 0.3671·CC + 1.5 LSB/µT reproduces all three (19.9/38.2/74.9) —
scale from the live CC registers.
DRDY (pin and STATUS bit): set at measurement complete; cleared per HSHAKE
DRC bits (default: any register write, or reading the measurement results).

## Cross-check notes (datasheet vs the hand-extracted headers)

- bmi08x.h agrees on every address; its own comments note the naming drift
  (INT1_IO_CONF vs datasheet INT1_IO_CTRL etc.). ACC_CONF composite values
  (0xA8 = bwp normal | odr 100 Hz) consistent with the map's reset 0xA8.
- bmp388.h NVM_PARAMS 0x31, 21 registers ✓ Table 24 (0x31..0x45) ✓
  bmp390inv byte order ✓ (round-trip already validated on the host).
- rm3100.h agrees on all addresses and the TMRC value table (0x92..0x9B).
- BMP390 map row for INT_STATUS: datasheet text says the register clears
  "after reading" as a whole — subsumes the drdy bit; modeled as whole-COR.

## Open questions for verification

1. RM3100 REVID = 0x22: not in the manual; confirm from the AP driver (or
   read a real part) before M6.
2. Accel I2C-until-CSB1-rise power-up quirk: emulate (junk until first
   deselect) or ignore? AP's probe does a dummy read first, so ignoring is
   probably safe — decide.
3. BMP390 IIR filter (CONFIG): store-only, or model the filter? (AP
   typically sets a low IIR for baro; harness physics already lags.)
4. BMI088 gyro drdy auto-clear (280–400 µs) — model via scheduler tick;
   acceptable?
5. SENSORTIME (both Bosch parts): free-running counters the DUT may read;
   feed from the 10 kHz scheduler (accel units 1/25.6 kHz — nearest tick
   ok?) or leave 0 if AP ignores them — check AP driver use.

## Addendum after review — resolutions and DUT-driver findings

Resolved by the user: no I2C emulation (SPI only, skip the accel wake
quirk); BMP390 IIR store-only; gyro drdy auto-clear via scheduler tick OK.
All interface data crosses through explicit serialize/deserialize steps
(nlib/binary.h style) — never casts.

Findings from the DUT drivers themselves:

- **ArduPilot's BMI088 driver is FIFO-based on both dies** (accel: LEN
  0x24/25, DATA 0x26, DOWNS 0x45, CONFIG 0x48/49; gyro: FIFO_STATUS 0x0E,
  FIFO_CONFIG_1 0x3E, FIFO_DATA 0x3F) — registers our datasheet rev 1.3
  still marks reserved; the FIFO sections appear in later revisions. The
  accel FIFO stream is frame-encoded and may include sensortime frames
  (the driver parses them). The direct data registers are only used by the
  in-house master. => M4 splits: **M4a register models** (covers the
  in-house mimic and all probe/config paths), **M4b BMI088 FIFO flavor**
  (streaming FIFO_DATA source in the engine + frame rendering), needed
  before the ArduPilot DUT (M6). Spec sources: the AP driver + a newer
  BMI088 datasheet revision.
- SENSORTIME registers: not read directly by either driver family — leave
  0; sensortime appears only as accel-FIFO frames (M4b).
- RM3100: ArduPilot's whoami is **reading CCX/Y/Z and comparing against
  the power-on defaults 0x00C8** — the model's reset values must be exact.
  REVID is defined but the probe relies on CC defaults; serving 0x22 is
  safe either way.
- In-house master init traffic (the M6 mimic contract): gyro CHIP_ID read,
  SELF_TEST write 0x01 / read expect (x & 0x16) == 0x12, SOFTRESET 0xB6,
  config writes **with readback verification**; accel: CHIP_ID dummy read
  (the CSB1 wake), PWR_CTRL 0x04, 50 ms, PWR_CONF 0x00, self-test with
  amplitude checks, power-on again, config + readback.

## Addendum M4b — the BMI088 FIFOs as the ArduPilot driver drives them

Implemented to the driver, cross-checked against a later datasheet rev.
Both FIFOs are served through the slave engine's streaming register (a
read burst starting at FIFO_DATA streams a side buffer instead of the
file; the address does not auto-increment, matching silicon).

**Accel** (`FIFO_LEN 0x24/25 le, FIFO_DATA 0x26, DOWNS 0x45, WTM 0x46/47,
CONFIG0 0x48, CONFIG1 0x49`):

- AP config: CONF 0x9C (1600 Hz OSR2), RANGE 0x03, PWR_CONF 0, PWR_CTRL 4,
  CONFIG0 0x02 (stop-at-full), CONFIG1 0x50 (acc_en; bit4 always 1 — reset
  value 0x10). Read-first, then write with readback, per register.
- Read cycle: LEN (2 bytes le; AP treats bit15 as "empty" — we simply
  serve the byte count), clamp to 8 frames = 56 bytes, burst from
  FIFO_DATA (accel dummy byte applies), parse headered frames by
  `byte0 & 0xFC`: 0x84 accel (7 B, xyz le, the ONLY frame this model
  emits), 0x40/0x48/0x50 skip-class (2 B), 0x44 sensortime (4 B) — parsed
  but unused by AP, so not emitted.
- Semantics: capacity 146 frames (~1 KB, like the part); full = drop
  silently (AP has no accel overrun path); ANY write touching
  CONFIG0/CONFIG1 clears the FIFO; softreset clears. LEN always a
  multiple of 7.
- Temperature stays a direct read (TEMP_MSB 0x22, 11-bit signed,
  0.125 °C/LSB, offset 23), every 100th AP cycle.

**Gyro** (`FIFO_STATUS 0x0E, CONFIG0 0x3D (wtm), CONFIG1 0x3E,
FIFO_DATA 0x3F`; plus RATE_HBW 0x13 in the config set):

- AP config after softreset, ALL as checked registers (periodic readback,
  mismatch = error + rewrite): RANGE 0, BW 0x80, LPM1 0, RATE_HBW 0,
  CONFIG1 0x40 (fifo stop-at-full). => 0x13, 0x3D, 0x3E joined the wmask.
- Read cycle: FIFO_STATUS = overrun<<7 | frame count, clamp 8 frames,
  burst count*6 headerless bytes (no dummy) from FIFO_DATA, xyz le.
- Semantics: capacity 100 frames (600 B, like the part); full = raise the
  overrun bit and drop; AP answers overrun by rewriting CONFIG1 — any
  write to CONFIG1 clears FIFO + overrun. Rate registers stay live in
  parallel.

**Pop accounting**: the engine's frame len may overcount by the TX FIFO
prefetch (≤ 4 bytes never shifted out), so the deselect hook pops
`floor(len / framesize) * framesize` bytes — exact as long as the master
reads whole-frame multiples, which both AP read cycles do (56 = 8×7,
count×6). Commits append frame + length registers in one atomic
`spidev_apply`, so a select never observes a torn FIFO.
