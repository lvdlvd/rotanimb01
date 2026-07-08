# HITL sensor simulator for ArduPlane — design & implementation plan (v2)

Status: design, topology settled. Target: STM32G474RET6 (LQFP64,
Nucleo-G474RE / 64-pin breakouts on hand), built on [N]Array. This is
the "optional test harness application (based on same STM32 MCU)" step
of the n-array workflow, and the first consumer of several planned lib
drivers (fdcan, pwm capture, exti, SPI slave).

v2 supersedes v1: the single-SPI-with-4-CS-demux topology is replaced by
a **hybrid**: dedicated SPI slaves with hardware NSS for the two BMI088
dies, one shared SPI with software CS demux for the two slow devices.
Rationale in "The two deadlines" below. SPI4 was considered and ruled
out: its pins live exclusively on port E, which PINMAP.periph confirms
exists only on the 100/128-pin packages.

## What it does

One G474 impersonates the core sensor suite of an ArduPlane flight
controller (the DUT) at the *register level*, over the DUT's own sensor
SPI bus:

- **BMI088** — two logical SPI devices on one package: accel (own CS,
  own INT1) and gyro (own CS, own INT3).
- **BMP390** barometer (CS + INT).
- **RM3100** magnetometer (CS + DRDY).

Four chip selects, four data-ready outputs, one shared SCK/MOSI/MISO on
the DUT side — the harness is the *slave* on all of them.
Simultaneously it captures up to 8 PWM servo/motor outputs of the DUT
(pulse width + period), and talks FDCAN to a host: the host commands a
kinematic state (speed, climb rate, turn rate in the earth frame); the
harness integrates that state, derives physically consistent body-frame
sensor readings, and reports the 8 PWM channels (and state readback)
upstream.

```
              host / test script
                    │ CAN (classic 1M, opt. FD)
        ┌───────────┴────────────┐
        │  G474 harness           │            ┌──────────────────┐
        │  physics ← cmd state    │ SPI1 slave │   DUT (ArduPlane │
        │  ┌────────────────────┐ │ ◄──hw NSS──┤   flight ctrl)   │
        │  │BMI088g│ = SPI1     │ │ SPI2 slave │                  │
        │  │BMI088a│ = SPI2     │ │ ◄──hw NSS──┤ one SPI bus,     │
        │  │BMP390+RM3100 = SPI3│ │ SPI3 slave │ SCK/MOSI/MISO    │
        │  └────────────────────┘ │ ◄─2×CS EXTI┤ fanned to 3 SPIs │
        │  4×DRDY ────────────────┼───────────►│                  │
        │  TIM2/TIM3 capture ◄────┼── 8×PWM ───┤ servo outputs    │
        └─────────────────────────┘            └──────────────────┘
```

Scope: IMU + baro + compass over SPI is the whole job of this board.
GPS and airspeed reach ArduPlane over its **own CAN port** (DroneCAN),
i.e. from the host or a separate feeder — less time-critical and
deliberately not on this harness. The physics state is designed so a
CAN GPS/airspeed feeder draws from the same commanded state.

## DUT-side assumptions (the other half of the ICD)

- A custom ArduPilot `hwdef.dat` for the HITL build declares exactly
  these sensors on this bus (`IMU BMI088 SPI:...`, `BARO BMP388
  SPI:...`, `COMPASS RM3100 SPI:...`) so the probe list matches what we
  emulate.
- ArduPilot's `SPIDEV` lines carry **two speeds** (lowspeed for
  init/probe, highspeed for data). HITL hwdef: **lowspeed ≤ 2 MHz for
  all four devices** (the IRQ engine's correctness domain); highspeed
  ≤ 2 MHz initially for the BMI088 pair (native speed becomes possible
  after M7 prediction), 1 MHz for baro and mag permanently (RM3100 is
  ≤ 1 MHz by datasheet; BMP390 at 50 Hz doesn't care).
- The DUT board must not have the real sensors populated on this bus
  (or they must be held in reset / CS tied off) — one slave per net.
- Everything is 3.3 V CMOS; no level shifting.

## The two deadlines (why this topology)

A slave that emulates a register-file protocol faces two very different
timing constraints, and conflating them was v1's mistake:

**Deadline 1 — CS assert → ready to serve.** The master's software gap
between asserting CS and the first SCK edge is microseconds (ChibiOS
`spiSelect` → `spiExchange` overhead), and the first byte on the wire is
the address — nothing meaningful is owed on MISO for 8 more bit-times.
An EXTI at priority 0 identifies a demuxed device with hundreds of
nanoseconds to spare. **Easy**, regardless of topology.

**Deadline 2 — address byte → first data byte, within one
transaction.** Real sensors answer a read in the same transaction that
requests it, back to back. The slave learns the address at RXNE of
byte 0; for the no-dummy-byte protocols (BMI088 **gyro**, RM3100) the
first response bit shifts out roughly **one bit-time** later. This is
protocol physics — no number of SPI peripherals or MCUs removes it.
The budget at 170 MHz:

| SPI clock | 1 bit  | 1 byte  | cycles/bit | cycles/byte |
|-----------|--------|---------|------------|-------------|
| 10 MHz    | 100 ns | 800 ns  | 17         | 136         |
| 2 MHz     | 500 ns | 4 µs    | 85         | 680         |
| 1 MHz     | 1 µs   | 8 µs    | 170        | 1360        |

An IRQ handler (12-cycle entry + read DR + decode + write DR ≈ 50–90
cycles, code in RAM, priority 0) meets an ~85-cycle deadline marginally
and a 170-cycle one comfortably — hence "IRQ engine correct at
≤ 2 MHz". BMI088 accel and BMP390 insert a **dummy byte** after the
address on reads, buying a whole extra byte-time.

The only ways past deadline 2 at native bus speed are (a) the dummy
byte, or (b) **prediction**: having the answer pre-loaded before the
address arrives. Prediction is only race-free when a device has an SPI
peripheral *to itself* — its expected periodic burst frame sits
pre-armed in that peripheral's TX DMA at all times, hardware NSS gates
it, and the address byte merely *verifies* after the fact. On a shared
SPI you can't pre-arm, because you don't know which device is next.

Rank the devices by how much they need this:

| device       | dummy byte | deadline-2 slack | rate / bandwidth | verdict |
|--------------|-----------|------------------|------------------|---------|
| BMI088 gyro  | **no**    | ~1 bit           | 2 kHz, dominant  | dedicated SPI, prediction-capable |
| BMI088 accel | yes       | ~1 byte          | 1.6 kHz, high    | dedicated SPI, prediction-capable |
| BMP390       | yes       | ~1 byte          | 50 Hz, trivial   | shared SPI, IRQ engine forever |
| RM3100       | **no**    | ~1 bit           | ≤1 MHz mandatory | shared SPI, IRQ engine forever |

Hence the hybrid: **SPI1 = gyro, SPI2 = accel** (hardware NSS, own DMA
pair each), **SPI3 = baro + mag shared** (SSM permanently selected, two
CS lines on GPIO+EXTI, the v1 demux scheme scoped down to the two
devices it is perfect for, at 1 MHz where the IRQ engine has 2×
margin).

Do we even need prediction? Bandwidth math says maybe not: gyro
2 kHz × ~8 B + accel 1.6 kHz × ~10 B ≈ 32 kB/s of payload against
250 kB/s at 2 MHz. The real DUT-side cost is bus-thread occupancy
(a 10-byte burst at 2 MHz is 40 µs; ~3.6 k transactions/s ≈ 15 % of one
bus thread) — noticeable, likely acceptable for a HITL build. So the
baseline plan runs *everything* at ≤ 2 MHz on the pure-IRQ engine, and
prediction (M7) is an upgrade on SPI1/2 only, taken if and when 2 MHz
pinches.

CPU load sanity check: ~30 k byte-IRQs/s × ~100 cycles ≈ 2 % of
170 MHz. The per-byte engine is not the bottleneck; the deadline is —
and M3 measures it before anything depends on it.

## Pinout (draft — run `narray -part STM32G474RET6 -pinfmt` on the real board.c)

| Signal        | Pin  | Function        | Notes                        |
|---------------|------|-----------------|------------------------------|
| GYRO NSS      | PA4  | SPI1_NSS AF5    | hw NSS; + EXTI4 (frame end)  |
| GYRO SCK      | PA5  | SPI1_SCK AF5    |                              |
| GYRO MISO     | PA6  | SPI1_MISO AF5   | tied w/ PB14, PC11 → DUT MISO|
| GYRO MOSI     | PA7  | SPI1_MOSI AF5   |                              |
| ACC NSS       | PB12 | SPI2_NSS AF5    | hw NSS; + EXTI12             |
| ACC SCK       | PB13 | SPI2_SCK AF5    |                              |
| ACC MISO      | PB14 | SPI2_MISO AF5   |                              |
| ACC MOSI      | PB15 | SPI2_MOSI AF5   |                              |
| BARO/MAG SCK  | PC10 | SPI3_SCK AF6    | shared slave, SSM            |
| BARO/MAG MISO | PC11 | SPI3_MISO AF6   |                              |
| BARO/MAG MOSI | PC12 | SPI3_MOSI AF6   |                              |
| CS baro       | PC0  | GPIO in, EXTI0  |                              |
| CS mag        | PC1  | GPIO in, EXTI1  |                              |
| DRDY accel    | PC4  | GPIO out        | BMI088 INT1                  |
| DRDY gyro     | PC5  | GPIO out        | BMI088 INT3                  |
| DRDY baro     | PB6  | GPIO out        | BMP390 INT                   |
| DRDY mag      | PB7  | GPIO out        | RM3100 DRDY                  |
| PWM in 1–4    | PA0 PA1 PB10 PB11 | TIM2_CH1–4 AF1 | 32-bit timer  |
| PWM in 5–8    | PC6 PC7 PC8 PC9   | TIM3_CH1–4 AF2 |               |
| CAN RX/TX     | PA11/PA12 | FDCAN1 AF9 | + external transceiver; NOT PB8/PB9 (see note) |
| Console       | PA9/PA10 | USART1 AF7  | as examples/hello            |
| LED           | PC13 | GPIO out        |                              |

**Why not PB8/PB9 for CAN:** PB8 is BOOT0 on the G4. A CAN transceiver's
RXD idles high (recessive), so a transceiver on PB8 pulls BOOT0 high
through reset and the chip boots into the system bootloader instead of
the application (unless the nBOOT0/nSWBOOT0 option bits are burned —
fragile, and wrong on every fresh chip). FDCAN1 on PA11/PA12 (AF9)
avoids this entirely; PA11/PA12's only competing role is USB, unused
here. Leave PB8 unconnected or strapped low.

~33 of 52 GPIOs, no AF conflicts. All four EXTI lines in use (0, 1, 4,
12) are distinct — each gets its own vector, no shared-line demux. EXTI
on PA4/PB12 coexists with the hardware-NSS AF (EXTI watches the pin
state regardless of mode); those two are *low-priority* frame-end
bookkeeping (predictor re-arm, stats), not on the deadline path.

**DUT harness wiring:** SCK fans out to PA5/PB13/PC10, MOSI to
PA7/PB15/PC12; the DUT's four CS lines go one each to PA4, PB12, PC0,
PC1; the three MISOs (PA6/PB14/PC11) are tied together onto the DUT's
MISO. That tie requires each *deselected* slave to tri-state MISO —
whether the G4's SPI IP releases MISO on NSS-high is a bring-up
verification item (M3 assertion). Fallback, in order: EXTI flips the
deselected MISOs to analog (the whole address byte of latency is
available, and every select line already has an EXTI); or a 74LVC125
buffer per MISO with the CS lines driving the enables.

## SPI register-file slave engine

`lib/spislave.{h,c}` — the generic engine, two **bindings** per
instance:

- **hw-NSS binding** (SPI1, SPI2): one device per peripheral. Hardware
  NSS frames the transaction; NSS-high performs the flush/resync (plus
  the SPE-toggle if the IP requires it — M3 decides). No software on
  the select path at all in prediction mode.
- **SSM binding** (SPI3): peripheral permanently selected (`SSM=1,
  SSI=0`); N CS lines on both-edge EXTI. Falling edge (priority 0):
  read IDR, set `active = &dev[i]`, reset FSM, stuff fill byte(s).
  Rising edge: disable SPE, flush both FIFOs, re-enable — the
  documented, reliable byte-resync on this IP; any mid-frame glitch
  self-heals at the next transaction.

Common per-byte path (RXNE, priority 0, handler in RAM): byte 0 =
command byte → decode R/W bit + start address per the device's rules;
reads push data into the 4-deep TX FIFO immediately and keep it topped
up, auto-incrementing per device; writes route each MOSI byte through
the device's write hook (side effects: soft reset, mode change,
status-bit clear). Nothing else in the system runs at priority 0; the
worst added jitter is one in-flight priority-0 handler (transactions on
the one physical bus can't overlap).

Each simulated device:

```c
struct SPIDev {
    uint8_t  reg[128];        // live register file (the "silicon")
    uint8_t  staging[16];     // next sample, committed under the coherency rule
    const struct SPIDevOps *ops;
};
struct SPIDevOps {
    uint8_t  read_cmd_mask;   // 0x80: MSB=read for all four devices
    uint8_t  ndummy;          // 1 for BMI088-accel & BMP390 reads, else 0
    uint8_t  fill;            // idle MISO byte
    uint8_t (*read)(struct SPIDev*, uint8_t addr);   // may clear-on-read
    void    (*write)(struct SPIDev*, uint8_t addr, uint8_t val); // side effects
};
```

Hooks execute inside the priority-0 handler: table lookups and flag ops
only; anything heavier (soft-reset re-init) defers to thread context
via a flag.

**Data coherency.** A burst must never mix sample N and N+1. The
scheduler writes into `staging`; the commit into `reg[]` (≤ 12-byte
memcpy) happens only when the device is not mid-burst — checked against
the FSM state, or deferred to the deselect edge. Mirrors the sensors'
own shadowing semantics.

**Prediction fast path (M7, SPI1/2 only).** Once a device has been put
in its continuous/normal mode by the DUT, its transactions are the
fixed periodic burst. The engine then keeps that device's TX DMA
pre-armed with the composed frame (fill, dummy-fill if any, current
sample); NSS gates it with zero software and **zero deadline** — native
10 MHz. The address byte is verified after the fact; a mismatch is
counted, answered correctly-but-late (one garbage transaction, which
these CRC-less protocols tolerate), and reported over CAN. Config-phase
random access stays on the IRQ path — which is what the hwdef's
*lowspeed* is for. Verify in M4 whether AP's BMI088 driver performs
periodic config-register health readbacks at highspeed (several ICM
drivers do); if so, either cover those templates or lower that device's
highspeed.

## The four device models (subtleties that will bite)

All register numbers to be re-verified against datasheets in M4 —
they're from memory and drive the design, not the implementation.

**BMI088 gyro** (SPI1, INT3). Separate die, separate protocol: **no
dummy byte** (the deadline-2 constraint). CHIP_ID = 0x0F. GYRO_RANGE /
GYRO_BANDWIDTH set LSB/(°/s) and ODR (2000 Hz typical for AP);
GYRO_INT_CTRL / INT3_INT4 mapping gate the DRDY pin. Soft reset
0x14 ← 0xB6.

**BMI088 accel** (SPI2, INT1). CHIP_ID 0x00 = 0x1E — ArduPilot probes
this first. Dummy byte on reads. Powers up suspended: the driver writes
ACC_PWR_CONF/ACC_PWR_CTRL and *reads back* its config writes — writable
registers must genuinely store and echo. ACC_CONF/ACC_RANGE drive both
DRDY rate and LSB/g — derive from the live registers, not constants.
ACC_SOFTRESET (0x7E ← 0xB6) → full regfile re-init. Temperature regs
feed from the physics model's ambient T (ArduPilot logs it).

**BMP390** (SPI3/CS-baro). CHIP_ID = 0x60 (AP's BMP388 driver accepts
0x50 and 0x60). Dummy byte on reads. The DUT reads the **NVM trim**
(par_t1..par_p11, regs ~0x31–0x45) once and compensates raw 24-bit ADC
words itself — so the simulator runs Bosch's polynomial **backwards**.
A real device's trim set is available and gets baked in; the inversion
(temperature first, then pressure via 2 Newton steps seeded by the
previous raw) is a **separate subproject** with a host-compiled golden
model (cordic-math emul pattern): forward-compensate the inverted raw,
assert |round-trip error| < 1 LSB across the envelope (−100 m..10 km,
−20..50 °C). It proceeds independently of everything else and plugs in
at M4. PWR_CTRL/OSR/ODR honored for rate; STATUS/INT_STATUS drdy bits
with clear-on-read.

**RM3100** (SPI3/CS-mag). No dummy byte but ≤ 1 MHz SPI by datasheet —
comfortable on the shared IRQ engine. REVID (0x36) = 0x22 is the probe.
The DUT writes cycle counts CCX/CCY/CCZ (default 200) and TMRC + CMM;
gain ≈ 0.3671·CC + 1.5 LSB/µT — scale from the live registers. Results
3 × 24-bit big-endian signed at 0x24; STATUS bit 7 = DRDY, cleared by
reading the measurement.

Common to all: reads of unimplemented addresses return 0x00 without
wedging the FSM; RO registers write-protected; per-device
`unexpected-access` counters surfaced over CAN — when a DUT driver
mis-probes, the harness should say why.

**DRDY pins vs status bits.** AP's drivers mostly *poll* on periodic
bus callbacks and check status registers; the INT pins are wired but
often unused. Emulate both: the status bit makes drivers work, the pin
serves a DRDY-driven hwdef. Pin behavior (level/edge, latched vs pulse)
follows the sensors' INT config registers as written by the DUT.

## Sample scheduler & DRDY generation

TIM7 (basic) at a 10 kHz tick runs a small table scheduler: each device
has a period derived from its live ODR config; on expiry the scheduler
(thread level, below SPI priority) pulls the current physics state,
quantizes into staging, commits under the coherency rule, sets the
status bit, drives the DRDY pin per the device's INT config. Jitter
requirement is loose (±100 µs is better than real sensors' ODR jitter);
long-term *rate* is exact because the tick is hardware-timed — and the
tick arithmetic is where the time-sync PLL (option 2 below) trims.

## PWM capture (8 ch)

TIM2 (32-bit) + TIM3, four channels each, input capture on **both
edges** (CCxP|CCxNP), 1 µs resolution (or 0.1 µs — decide in M1). Per
capture IRQ (low priority): read CCRx, read the pin to classify the
edge; rising→rising = period, rising→falling = width. 8 ch × ≤ 400 Hz ×
2 edges = 6.4 k IRQs/s — negligible. Output per channel: width µs,
period µs, age; no edge for > 100 ms → reports 0 (disconnected).
Protocol-agnostic pulse measurement: standard 50 Hz PWM and oneshot
work; DShot is out of scope (different capture strategy entirely — HITL
DUT config must use normal PWM outputs).

## Physics: commanded state → sensor truth

Commanded over CAN, earth (NED) frame: speed V, climb rate ḣ, turn
rate ψ̇. Raw commands pass through first-order lags (τ ≈ 0.3–0.5 s,
CAN-settable) so derivatives exist and the EKF sees smooth, flyable
transients instead of steps.

Integration at 2 kHz (matching the fastest ODR), CORDIC-backed trig
from `lib/cordic.h`:

- ψ ← ψ + ψ̇·dt, h ← h + ḣ·dt (heading and altitude are *internal*
  integrated state, reported back over CAN so the host can check
  drift).
- Coordinated flight: γ = asin(ḣ/V); bank tanφ = ψ̇·V·cosγ/g; pitch
  θ = γ + α₀ (fixed trim AoA).
- Body rates via the strapdown transform: p = φ̇ − ψ̇ sinθ,
  q = θ̇ cosφ + ψ̇ cosθ sinφ, r = −θ̇ sinφ + ψ̇ cosθ cosφ, with φ̇, θ̇
  analytic from the lag-filtered commands.
- Specific force: f_b = Rᵀ_nb·(a_n − g_n), a_n = d/dt (V cosγ cosψ,
  V cosγ sinψ, −ḣ) from the filtered states; reproduces the 1/cosφ
  load factor in a steady turn for free.
- Magnetometer: m_b = Rᵀ_nb·m_n; earth field (magnitude, inclination,
  declination) CAN-settable, mid-latitude default.
- Baro: ISA p(h) = p₀(1 − Lh/T₀)^5.255, CAN-settable QNH and ground
  temperature; T(h) feeds every device's temperature registers.

Quantization uses the live device configs (range/ODR/CC registers).
Optional per-sensor noise (xorshift white + settable bias) is a
CAN-switchable layer, off by default. V = 0 (bench cal) forces γ = φ =
0: 1 g down, earth field, zero rates. asin/atan domains guarded by
clamping + a CAN status flag, never NaN.

## CAN interface

Classic CAN 2.0 at 1 Mbit default (every host adapter, `candump`);
CAN-FD as an option once `lib/fdcan.h` grows it. Bit timing from
`clock_fdcan_hz()`. All headers are 29-bit bit-fields per the shared
in-house dictionary convention (ARINC825-derived: LCC / 7-bit MSGID /
FSB / SRCID-from-UID / ts_seq — layout in `src/canmsg.h`), payloads
big-endian (`lib/binary.h`). The harness owns the 0x40 MSGID block in
both channels; 0x01–0x3f (measurements) and 0x00–0x03, 0x70+ (TMC)
stay with the existing device dictionary. Scaled ints, 8-byte frames:

| LCC/MSGID | dir | payload |
|-----------|-----|---------|
| TMC 0x40  | →harness | CMD_STATE: V cm/s i16, ḣ cm/s i16, ψ̇ mrad/s i16, flags u16 |
| TMC 0x41  | →harness | CMD_ENV: QNH Pa/10 u16, T₀ 0.1 K u16, mag B 0.01 µT u16, incl 0.01° i16 |
| TMC 0x42  | →harness | CMD_NOISE: per-sensor enable mask + levels |
| MEAS 0x40/0x41 | harness→ | PWM ch1–4 / ch5–8: 4 × u16 µs, 50 Hz + on change (> 2 µs) |
| MEAS 0x42 | harness→ | STATUS 10 Hz: harness time µs u32 (truncated), ψ 0.01° u16, flags u16 |
| MEAS 0x43 | harness→ | DIAG: per-device transaction / unexpected-access / prediction-miss counters |

Stale-command watchdog: no CMD_STATE for 1 s → hold last state, flag in
STATUS. The same commanded state is what a host-side DroneCAN
GPS/airspeed feeder should derive from, so DUT sensor fusion stays
self-consistent.

## Time synchronization options

Three clock domains: DUT crystal, harness crystal, host. The sensors'
timebase *is* the harness clock — the DUT timestamps samples on
DRDY/poll arrival, exactly as with real silicon whose ODR is never
exactly nominal. Options, cheapest first:

1. **Free-run (v1 default).** ±20 ppm crystals each side → ≤ 40 ppm
   relative drift: the "2000 Hz" gyro is 1999.9–2000.1 Hz. ArduPilot
   measures and tracks actual sample rates; for functional HITL this is
   simply realistic. Log correlation to ~ms via CAN STATUS timestamps.
2. **Discipline the harness to the DUT via the PWM inputs** —
   software-only, zero extra wires, the measurement already exists. The
   DUT's servo frame rate (50 Hz default) is generated from the DUT
   clock; the capture timers measure its period to µs. Average ~10 s →
   relative frequency to ~0.1 ppm; a slow software PLL trims the sample
   scheduler (fractional period accumulator — never touch the RCC).
   The sensor timebase then tracks the DUT clock, which is the pairing
   the EKF cares about. Needs one output at a known fixed rate; AP
   output jitter is why the window is long (fine for frequency; phase
   to the DUT is irrelevant — the DUT syncs to DRDY).
3. **Shared oscillator (hardware).** DUT board's MCO or 8 MHz clock
   into the harness HSE input (one wire + solder);
   `clock_measure_hse()` auto-adopts whatever arrives. Zero drift by
   construction, deterministic replay. Board-dependent.
4. **Host↔harness sync over CAN** — only if the host runs a
   time-sensitive script (trajectory replay vs logged DUT output).
   Simplest: host streams CMD_STATE and the lag filters make phase
   moot. Fancier: DroneCAN-style two-message sync; FDCAN's RX timestamp
   counter can be clocked from TIM3 for hardware-accurate stamps;
   discipline a *reported-time offset*, not the scheduler. Only if log
   alignment < 1 ms is genuinely required.
5. **PPS wire.** 1 Hz pulse from host adapter or DUT into a spare
   capture channel → sub-µs alignment of all logs. Cheapest *precise*
   method if a wire can be run; pairs well with 1.

Recommendation: ship with 1 (+ timestamped CAN reports); implement 2 as
the M7 software upgrade — uniquely elegant here because the PWM capture
front-end doubles as the frequency reference; keep 3 documented for
deterministic-replay campaigns.

## What gets factored into n-array `lib/` (and what doesn't)

Into `lib/`:

- **`lib/fdcan.{h,c}`** — already on the README feature list. Classic
  first, FD-capable layout from day one; message-RAM helpers from the
  generated FDCAN RAM elements; TX queue + RX FIFO, app-owned IRQ
  wiring; bit timing from `clock_fdcan_hz()`. API parallel in spirit to
  `serial.h`.
- **`lib/pwmin.{h,c}`** — N-channel both-edge capture → {width, period,
  age} µs. Generic (RC input, tachometers, the sync-PLL reference).
- **`lib/exti.h`** (header-only) — SYSCFG EXTICR routing + edge config
  + pending clear. Missing today; ~60 lines of inlines.
- **`lib/spislave.{h,c}`** — the register-file slave engine with both
  bindings (hw-NSS single-device, SSM multi-device). A *different
  animal* from DESIGN.md's deferred "slave mode reusing SPIQ" (stm↔stm
  transport): this emulates a register-file protocol, generic across
  any sensor/EEPROM-style emulation. Rule-of-3-5 says 1 user — but the
  API is designed against four devices and two bindings at once, the
  device-specific parts are cleanly outside it, and the harness is the
  hardware-validation vehicle. Flagged for the review that lands it.

Stays project code: the four sensor models, physics + ISA + noise, the
CAN dictionary, board.c, the BMP390 trim-inversion subproject (host
golden + device plug-in). Proposed home: `examples/hitl-sim/` (it *is*
the flagship example: spi master+slave, fdcan, pwmin, exti, cordic,
serial, fault); evict to its own repo when a host-side toolkit grows.

Independent data task (no longer a prerequisite): extend PINMUX.periph
with the port D/E/F/G AF columns from the datasheet — PINMAP already
has their physical pin numbers; the AF tables stop at PC. Needed the
day a 100-pin board shows up (SPI4 lives on port E), useful regardless.

## Implementation plan

Every milestone compiles clean (gnu23, -Wextra) and has a test that
runs **without the DUT** — boards are in abundance, so a second
Nucleo runs a "DUT-mimic" master firmware built on the existing
`lib/spi.h` queue, replaying scripted and later *recorded ArduPilot
transaction traces* over real inter-board wiring. Rough sizes are
new-code lines.

- **M0 — skeleton (small).** board.c pinout (narray -pinfmt'd), clock,
  console, vector-table manifest, heartbeat. Clone of hello.
- **M1 — `lib/pwmin` (~250).** TIM2/TIM3 capture. Self-test: TIM4/TIM8
  generate known PWM on jumpered pins; assert width/period over the
  console. Decide 1 µs vs 0.1 µs here.
- **M2 — `lib/exti.h` + `lib/fdcan` (~600).** Bring up against a host
  adapter (`candump`/`cansend`); dictionary encode/decode; stale
  watchdog. PWM report path live end-to-end.
- **M3 — `lib/spislave` engine (~450).** One dummy 16-register device
  on each binding. DUT-mimic board drives randomized read/write bursts
  at 1/2/4 MHz into both a hw-NSS instance and the SSM pair. Assert:
  regfile contents; measured RXNE→DR margin (scope or timer) —
  converting the deadline table from analysis into fact; **MISO
  tri-state when deselected** among the tied MISOs (else enable the
  EXTI-to-analog fallback); resync after injected glitch clocks.
  Gate: clean at 2 MHz with margin.
- **M4 — sensor models (~800 + host golden).** Regfiles, ops tables,
  reset/config/status semantics, every register number re-verified
  against datasheets. BMP390 trim-inversion subproject plugs in here
  (can start any time — it's host-side until this point). DUT-mimic
  replays recorded AP driver traces against each model. Also: read the
  AP BMI088 driver source for periodic config readbacks (prediction
  risk) and the exact probe sequence.
- **M5 — physics + scheduler (~500).** Integration, quantization from
  live configs, DRDY generation, CAN state readback. Scripted CAN
  maneuvers + host plots: level flight → 1 g down; standard-rate turn
  at 20 m/s → φ ≈ 16°, load ≈ 1.04; climb → baro ramp matching ḣ.
- **M6 — DUT integration.** HITL hwdef (lowspeed ≤ 2 MHz everywhere,
  highspeed 2 MHz BMI088 / 1 MHz baro+mag), probe passes (chip IDs),
  INS/baro/compass healthy, rates as configured, EKF innovations sane
  under CAN-commanded maneuvers; DUT logs vs harness state readback.
  This is where register-map memory errors surface — budget bench time.
- **M7 — hardening & sync.** PWM-disciplined scheduler PLL (sync
  option 2); noise/bias layer; **prediction fast path on SPI1/2** if
  native-speed BMI088 is wanted — else close it out at 2 MHz and delete
  the option.

## Risks & open questions

- ~~**RXNE deadline at 2 MHz**~~ **M3 MEASURED** (loopback, gapless master,
  code in RAM, handler worst case 88–91 cycles): byte 1's FIFO load happens
  at the byte-0 IRQ itself, so it can only be served by bytes queued before
  the command. With the dummy pre-stuffed at select, **dummy protocols
  (accel, BMP390) are clean through 10.5 MHz — native speed, reactively**;
  no-dummy protocols (gyro, RM3100) are clean at 1.3 MHz, 12 % first-byte
  underrun at 2.6 MHz. Consequence: hwdef lowspeed 1 MHz for gyro + mag
  (RM3100 is ≤ 1 MHz anyway), accel/baro free choice; prediction (M7) is
  now a gyro-data-phase-only question.
- **MISO tri-state on deselect** among the three tied MISO pins:
  verify in M3; fallbacks (EXTI-to-analog, 74LVC125) are ready and
  cheap.
- **Register-map details from memory** (IDs, addresses, trim layout,
  RM3100 gain formula) re-verified in M4; the design doesn't change if
  a number does.
- **AP periodic register health-reads at highspeed** would puncture
  prediction — check the driver source in M4; mitigations: template
  coverage or per-device highspeed.
- **Bus-thread occupancy at 2 MHz** on the DUT (~15 % estimate): watch
  scheduler-overrun counters during M6; the pressure valve is M7
  prediction.
- Whether `spislave` earns its `lib/` seat now or waits per
  rule-of-3-5 — flagged for the review that lands it.
