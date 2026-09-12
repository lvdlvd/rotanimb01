# rotanimb01 — a HITL flight bench for ArduPlane and PX4

One STM32G474 (the **harness**) impersonates a flight controller's whole
sensor suite — [BMI088](doc/BMI088.pdf) gyro+accel,
[BMP390](doc/BMP390.pdf) baro, [RM3100](doc/RM3100.pdf) magnetometer — at
the register level of the **SPI** (Serial Peripheral Interface) bus of
the device under test (the **DUT**), feeds it GPS and airspeed over
DroneCAN, captures its eight servo PWM (pulse-width modulation) outputs,
and runs a six-degree-of-freedom Flight Dynamics Model (**FDM**) of a
Kitfox Model V light aircraft at 1 kHz to close the loop. A stock
ArduPlane or PX4 build on a NUCLEO-F767ZI (the DUT) boots against it,
probes "real" sensors, calibrates, arms, takes off and flies — its real
drivers, its real Extended Kalman Filter (EKF), its real control loops,
none the wiser for a hardware-in-the-loop test bench.

The same FDM also flies as an ArduPilot Software-In-The-Loop (**SITL**)
backend and as a PX4 SITL simulator on the workstation, so tuning
campaigns run at 10x real time in pure software and are then validated
through the emulated-sensor hardware path. Both rigs fly the reference
loiter legs identically to the meter ([doc/F5-TESTREPORT-2026-07-15.md](doc/F5-TESTREPORT-2026-07-15.md)).

## The loop

```
             ┌──────────────────── harness (G474) ────────────────────┐
             │                                                        │
             │   fdm.c 1 kHz ──► truth ──► sensor models (noise,     │
             │   6-DOF light aircraft      quantization, bias walk)   │
             │        ▲                     │            │            │
             │        │                 SPI slaves    FDCAN3          │
             │   controls.c             (regfiles)    DroneCAN        │
             │   cal+lag+failsafe        BMI088 x2    Fix2 5 Hz       │
             │        ▲                  BMP390       RawAirData      │
             │        │                  RM3100       NodeStatus      │
             │   PWM capture x8             │            │            │
             └────────┼───────────────────┬─┼────────────┼────────────┘
                      │                   │ │            │
                 servo PWM          SPI3 + 4xCS      CAN bus (1 Mbit)
                      │                   │ │            │
             ┌────────┴───────────────────┴─┴────────────┴────────────┐
             │       DUT: stock ArduPlane or PX4, NUCLEO-F767ZI       │
             │       real drivers → EKF → TECS/L1 → servos            │
             └─────────────────────────────────────────────────────────┘
```

In the diagram, TECS and L1 are ArduPlane's energy (speed/height) and
lateral navigation controllers, and CS the four SPI chip-select lines.

Control/observability sidechannel: the harness's USB CDC (a virtual
serial port) speaks **pseudocan** (text-framed CAN, nlib/fmtcan). Over
it run the FDM commands (mode, air-start, wind, PWM cal, parameters)
and the 20 Hz TRUTH_* telemetry. `rb01tool` is the interactive cockpit;
`tools/bench` drives unattended missions.

![rb01tool's primary flight display: an ASCII airspeed tape on the left
reading 26.7, an altitude tape on the right reading 75.6, a pitch ladder
and horizon across the middle, a heading tape along the bottom reading
316.3, and a status line with IAS, TAS, ALT, VSI, groundspeed and the
control positions.](doc/rb01tool-pfd.png)

*`rb01tool`'s PFD (`f` toggles it): the tapes, the compass and the
horizon are drawn from the TRUTH_* stream — the attitude comes straight
off the FDM's quaternion, not from the DUT's estimate, so this is what
the aircraft is actually doing rather than what the autopilot believes.*

## What you need

[![The bench wired up: a Raspberry Pi host at the top, the NUCLEO-F767ZI
DUT and the STM32G474 harness breakout on breadboards below it, joined by
two CAN transceivers and jumper wires.](doc/hitl-small.jpg)](doc/hitl.png)

*The bench. Full resolution: doc/hitl.png.*

- **Harness**: a cheap generic 64-pin STM32G474RET6 breakout board
  (the kind sold online for a few euros: LED on PC13 active-low, USB
  connector on PA11/PA12, SWD debug header) plus an ST-Link debug probe
  whose VCP (virtual COM port, a USB serial) is
  wired to USART1 PA9/PA10 as the console. That is the board this was
  developed and tested on. A NUCLEO-G474RE has the same microcontroller and pins
  but was not tested; [doc/SETUP-ARDUPLANE.md](doc/SETUP-ARDUPLANE.md) section 1 lists what
  would differ. Flashed over SWD with openocd.
- **DUT**: a NUCLEO-F767ZI. Board definitions for both autopilots ship
  in `dut/` as patch series against upstream.
- Two 5 V CAN transceivers (TJA1051 or similar) and a handful of jumper
  wires; a USB hub with per-port power control (uhubctl) is a real
  quality-of-life item, see [doc/BENCH-OPERATIONS.md](doc/BENCH-OPERATIONS.md).
- A host for the USB side: any Linux box (a Raspberry Pi works well) or
  the workstation directly. Toolchains: arm-none-eabi-gcc 15.2 for the
  harness (what it is built and tested with; `-std=gnu23`), Go 1.21+ for
  rb01tool and tools/bench (Go
  1.25+ and github.com/lvdlvd/gomavlink for tools/px4hil), the
  autopilot's own toolchain for the DUT.

## Start here

1. **[doc/SETUP-ARDUPLANE.md](doc/SETUP-ARDUPLANE.md)** — wire it, build and flash the harness,
   build stock ArduPlane for the DUT, first boot, fly the 1 km square.
2. **[doc/SETUP-PX4.md](doc/SETUP-PX4.md)** — the same for PX4.
3. **[doc/BENCH-OPERATIONS.md](doc/BENCH-OPERATIONS.md)** — the bring-up ladder and every trap the
   bench has taught: RAM-only state, power-cycle discipline, engine
   models, resets.
4. **[doc/FDM-TUNING.md](doc/FDM-TUNING.md)** — the parameter table, the golden-check
   workflow, and which observable pins which coefficient.
5. **[doc/TRUTH-TELEMETRY.md](doc/TRUTH-TELEMETRY.md)** — reading truth off the harness for your
   own consumers.

## Directory map

| dir              | what                                                                                       |
| ---------------- | ------------------------------------------------------------------------------------------ |
| `src/`           | harness firmware: SPI-slave engine users, sensor regfile models, PWM capture, FDCAN3 DroneCAN feeder, pseudocan command loop; `src/nlib/` is the bare-metal support library |
| `fdm/`           | the 6-DOF model (`fdm.c`, freestanding float32+CORDIC) + host `golden` gates + `turncheck` + **`sitljson`** (the ArduPilot SITL backend wrapper) |
| `physics/`       | mode-0 kinematic model (speed/climb/turn commands) + its golden                            |
| `bmp390inv/`     | BMP390 compensation inverter (truth pressure → raw counts for the emulated trim)           |
| `selftest/`      | the harness masters its own sensor bus and replays a real driver's init sequences (7 jumpers) |
| `rb01tool/`      | Go console cockpit: live primary flight display (PFD), single-key physics steering, host-side DroneCAN GPS feeder   |
| `tools/bench/`   | bench + SITL mission drivers (Go): param staging, departures, loiter legs, probes, autotune, harness `drive` commands, TCP↔serial bridge |
| `tools/px4hil/`  | PX4 SITL simulator bridge around the same `fdm.c` (cgo)                                    |
| `dut/ardupilot/` | patch series: the NucleoF767ZI hwdef + two bootloader fixes                                |
| `dut/px4/`       | patch series: the st/nucleo-f767zi board, BMP388 SPI driver, airframe 2110                 |
| `doc/`           | setup guides, design docs, the checkride report, the annotated DUT parameter file, example missions |

## Design documentation

- **[doc/DESIGN.md](doc/DESIGN.md)** — the harness: topology (one SPI slave, 4 CS demux),
  the two deadlines, register models, CAN dictionary, pinout, milestones
  M0-M7 with bench-measured numbers.
- **[doc/fdm-DESIGN.md](doc/fdm-DESIGN.md)** — the FDM: aero tables, power-based prop, ground
  model, servo lag, integration ladder F0-F5.
- **[doc/REGMAPS.md](doc/REGMAPS.md)** — the emulated register maps as the autopilots'
  drivers actually exercise them.
- **[doc/F5-CHECKRIDE.md](doc/F5-CHECKRIDE.md)** — the acceptance rung definition.
- **[doc/F5-TESTREPORT-2026-07-15.md](doc/F5-TESTREPORT-2026-07-15.md)** — the multi-night checkride
  report: 17+ defects found and fixed (the point of HITL), transition
  matrix, tuning campaign, final PASS numbers.
- **[doc/BMI088.pdf](doc/BMI088.pdf)**, **[doc/BMP390.pdf](doc/BMP390.pdf)**,
  **[doc/RM3100.pdf](doc/RM3100.pdf)** — the three datasheets the register
  models were built from (rev 1.3, rev 1.7, user manual V9.0). Copyright
  their manufacturers; see LICENSE.
- **doc/f5-bench.parm** — the ArduPlane parameter file, heavily
  annotated with every ArduPilot trap the bench discovered.
- **src/canmsg.h** — the as-built pseudocan dictionary (commands in,
  telemetry out).

## The two rigs

### HITL (the real bench)

All USB devices — the DUT's MAVLink CDC, the harness pseudocan CDC, the
two ST-Links — hang off one host. `tools/bench` runs on the workstation
and reaches the DUT through `bench serbridge` on that host (TCP port
5760); harness commands (`bench drive ...`) run on the host itself.
`uhubctl` power-cycles individual hub ports — the cure for ArduPilot's
watchdog latch, which survives soft resets in backup RAM.

Wiring: DUT SPI3 (PB3/4/5) + CS PD3/4/5/6 → harness SPI3 (PC10/11/12)
+ PC0-PC3; DUT PWM1-8 (PC6-9, PD12-15) → harness PA0/PA1/PB10/PB11 +
PC6-9; DUT CAN1 PD0/PD1 ↔ harness FDCAN3 PB3/PB4 via TJA1051
transceivers (5 V supply — 3.3 V cannot drive the bus, the CAN last-error code reads Bit0
forever). Full table in [doc/SETUP-ARDUPLANE.md](doc/SETUP-ARDUPLANE.md).

### SITL (the software rig)

`fdm/sitljson` wraps the **same fdm.c** as an ArduPilot SITL "JSON
backend": lockstep UDP on :9002, harness actuator model included, so
the tune sees the same servo phase the bench has. SITL's own sensor
models replace the harness path — which is exactly what makes the
pair diagnostic: anything that flies in SITL but not on the bench is
a sensor-path or configuration divergence, and vice versa.

```
cd fdm && make sitljson && ./sitljson &
arduplane --model JSON:127.0.0.1 --speedup 10 --home 52.0,5.1,0,0 -w &
```

Then drive it with `tools/bench` exactly like the real bench (same
tool, different address). Tune fast in SITL, validate through the
sensor path on hardware. See [tools/bench/README.md](tools/bench/README.md) for the campaign
tools and the session procedures. `tools/px4hil` is the PX4
equivalent (simulator-MAVLink over TCP :4560).

## Hard-won operational truths (the short list)

The long list is [doc/BENCH-OPERATIONS.md](doc/BENCH-OPERATIONS.md), [doc/F5-TESTREPORT-2026-07-15.md](doc/F5-TESTREPORT-2026-07-15.md)
and the parm file comments. The ones that cost the most:

- **The harness PWM cal is RAM-only.** Any harness reboot reverts to
  the all-positive default = inverted elevator for ArduPilot. Resend
  after every harness reboot (`bench drive cal`); note uhubctl cycles
  can reboot the harness when it shares hub power with the target.
- **A completed ArduPilot AUTOTUNE keeps its gains in RAM.** The first
  DUT reboot silently reverts them. `param_set` them explicitly and
  verify by readback at session start (`bench params` does).
- **ARSPD_RATIO 1.6327** (= 2/rho0): the harness diff pressure is
  exactly 0.5·rho0·IAS² (IAS = indicated airspeed); ArduPilot's 2.0 default reads 10.7% high.
- ArduPilot ignores RC_CHANNELS_OVERRIDE unless source_system ==
  SYSID_MYGCS (255). Silently.
- Message-interval floods before takeoff starve the climb loop —
  raise stream rates only once airborne.
- After a watchdog event, power-cycle before expecting baro cal;
  after reflashing the harness mid-session, power-cycle the DUT (its
  main loop stalls on the dead SPI bus and trips that watchdog).
- **The default engine model is a naturally aspirated 100 hp Rotax
  912.** It cannot hold altitude above ~14,000 ft. High-altitude work
  needs `bench drive engine 915` (turbo), which is RAM-only too.
- **Sensor noise scales are RAM-only as well** (`bench drive noise`): a
  harness reboot comes back at 1.0×, the datasheet default — the safe
  direction, but a scaled run that outlives a reboot is silently no longer
  scaled. The 1 Hz console heartbeat carries the live scales as
  `nz g/a/m/b/bias`, so check there rather than assuming. The baro scale
  floors at 1.0× and can only be turned up: ArduPilot's stuck-baro detector
  and PX4's `DataValidator` both declare a bit-identical stream unhealthy, so
  a noise-free baro reads as a broken one.

## Known issues

- **SPI slave desynchronisation (origin still unlocated).** The slave engine
  loses byte alignment with the DUT's bus master; the harness's `unexp`
  counter then climbs by thousands per second and ArduPlane boot-loops on the
  corrupt inertial-sensor replies. **A DUT reset is one reliable trigger** —
  reproduced three ways (SWD warm reset, SWD reset after a clean harness
  reboot, and a plain MAVLink software reboot with no debugger attached), so
  it is not a debugger artefact: any reset floats the DUT's SPI lines through
  its startup and clocks garbage into the slave. It is **not established as
  the only trigger**; the rarer unexplained cases predate that finding.
  Detector: the counters, healthy = 0 always. Recovery order matters — reset
  the **DUT first, then the harness ~2 s later**, then `drive mode 1`; the
  reverse order leaves the harness desynchronised, because the DUT's reset is
  what corrupts it.
  The harness now **detects this itself**: a 10 Hz watchdog watches the
  refused-write rate (healthy is a hard zero; a desync runs ~1800 per 100 ms),
  raises **bit 6 of the STATUS flags** so a host sees it on the CAN link
  rather than by scraping the console, and counts events as `desync N` on the
  heartbeat. **Detection only — there is no automatic recovery, and not for
  want of trying.** Re-arming the engine in place (an RCC pulse to flush the
  TXFIFO, then re-running `spislave_init` to clear the FSM, reload the DMA
  channel and re-queue the byte-0 fill, leaving the emulated register files
  intact) was implemented and **bench-tested on 2026-09-12: it does not
  work.** The resync fired once a second for 40 s while `unexp` climbed past
  1.6 M unabated, so whatever the desynchronised state is, it is not cleared
  by re-initialising the slave peripheral. That negative result is recorded
  here so the next person does not spend the day rediscovering it. The cure
  remains a DUT reset followed by a harness reset ~2 s later. See
  [doc/BENCH-OPERATIONS.md](doc/BENCH-OPERATIONS.md).
- **GPS error is correlated, not white**, so it does not average away: the
  feeder adds a first-order Gauss-Markov position error (1 m horizontal, 2 m
  vertical, 60 s time constant) on top of the transport lag, and advertises a
  covariance that matches. Velocity noise is white and small, as on a real
  doppler-derived fix. `bench drive noise` scales it, and the pitot's, along
  with the four emulated SPI sensors.
- **Airspeed is quantised to 0.1 m/s before the pitot pressure is computed.**
  The lag ring stores IAS as a u16 in 0.1 m/s units and RawAirData's
  differential pressure is `0.5*rho*ias^2` from that, so the transmitted
  pressure moves in steps of about 6 Pa at cruise — measured on the bench at
  ~45 m/s: 20 distinct values spanning 113 Pa over a 120 s climb, a 5.97 Pa
  step against 1.225*45*0.1 = 5.5 Pa predicted. Before the pitot noise existed
  this made the emitted value bit-identical for up to 24 consecutive samples
  (1.2 s at 20 Hz), which is a stuck-source pattern: PX4's `DataValidator`
  invalidates a source after 100 identical samples, so the margin was 4x, and
  a steady cruise leg holds IAS inside one bin longer than a climb does. The
  noise layer removes the *symptom* — 0.5 Pa RMS is added after the
  quantisation, so consecutive samples are never identical — but the 0.1 m/s
  step itself remains, and anything that differentiates airspeed (TECS's rate
  response, EKF airspeed fusion innovations) still sees it.
  **If you are flying a pre-1.1 harness image against PX4, read this twice:**
  PX4's `airspeed_selector` runs a data-stuck check that trips on ~3 s of
  exactly-constant indicated airspeed in fixed-wing flight (2 s
  `DATA_STUCK_TIMEOUT` plus the 1 s `ASPD_FS_T_STOP` failsafe delay), it is
  enabled by default (`ASPD_DO_CHECKS` defaults to 7), and `ASPD_FS_T_START`
  defaults to -1, which disables re-enabling in flight. So one qualifying
  interval invalidates the airspeed **for the rest of that flight**, with
  `ASPD_FALLBACK` defaulting to no fallback. We measured bin residencies up to
  2.9 s in a climb without seeing it trip, so we have not observed this
  happening — but the margin was 3% and we could not rule it out. Images from
  1.1 on do not have the mechanism.
- The spiral mode over-converges and the short period is overdamped
  with the default coefficients ([doc/FDM-TUNING.md](doc/FDM-TUNING.md) has the knobs).
- tools/bench also speaks an experimental ArduPlane mode (HDGALT,
  `bench mode hdgalt` / `bench hdgaltcmd`) that exists only in a public
  fork, branch `hdgaltmode` of https://github.com/lvdlvd/ardupilot, with
  zero warranties. Nothing in this repository needs it; on stock
  ArduPlane those two commands are rejected. See [tools/bench/README.md](tools/bench/README.md).
- Bootloader builds in ArduPilot share the board's build directory with
  the application build; build the bootloader from a clean
  `build/NucleoF767ZI` or the generated DroneCAN headers go missing.
- **Fixed wing only, and a multirotor is not just another FDM.** The bus
  side is airframe-blind — the emulated sensor register files, the
  DroneCAN GPS feed, truth telemetry, PWM capture, the 1 kHz step — but
  the FDM/harness contract is not: `struct FdmControls` is da/de/dr/dt in
  radians, `src/controls.c` hard-codes AETR channel order, calibration in
  degrees of surface deflection and a 60 ms / 300 deg/s servo lag (a
  quad's four channels are all throttle-like, with rotor spin-up lag
  instead), `fdm_trim` solves level flight rather than hover and its
  output is what the failsafe falls back to (which glides on a plane and
  drops on a quad), and the ground model is tricycle gear. Expect a new
  FDM plus a rewritten input path, trim/air-start path, parameter table
  and golden gates, and ArduCopter or a PX4 quad airframe on the DUT. The
  open question is latency: this bench was sized against an
  open-loop-stable airframe, so measure the sample→SPI→PWM round trip
  against a multirotor attitude loop before trusting the port.

## Licence

MIT, see LICENSE. The autopilot patch series in `dut/` are contributions
to ArduPilot and PX4 and carry those projects' licences (GPLv3 and
BSD-3-Clause respectively).
