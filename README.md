# rotanimb01 — a HITL flight bench for ArduPlane and PX4

One STM32G474 (the **harness**) impersonates a flight controller's whole
sensor suite — BMI088 gyro+accel, BMP390 baro, RM3100 magnetometer — at
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
loiter legs identically to the meter (doc/F5-TESTREPORT-2026-07-15.md).

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

## What you need

- **Harness**: a cheap generic 64-pin STM32G474RET6 breakout board
  (the kind sold online for a few euros: LED on PC13 active-low, USB
  connector on PA11/PA12, SWD debug header) plus an ST-Link debug probe
  whose VCP (virtual COM port, a USB serial) is
  wired to USART1 PA9/PA10 as the console. That is the board this was
  developed and tested on. A NUCLEO-G474RE has the same microcontroller and pins
  but was not tested; doc/SETUP-ARDUPLANE.md section 1 lists what
  would differ. Flashed over SWD with openocd.
- **DUT**: a NUCLEO-F767ZI. Board definitions for both autopilots ship
  in `dut/` as patch series against upstream.
- Two 5 V CAN transceivers (TJA1051 or similar) and a handful of jumper
  wires; a USB hub with per-port power control (uhubctl) is a real
  quality-of-life item, see doc/BENCH-OPERATIONS.md.
- A host for the USB side: any Linux box (a Raspberry Pi works well) or
  the workstation directly. Toolchains: arm-none-eabi-gcc 15.2 for the
  harness (what it is built and tested with; `-std=gnu23`), Go 1.21+ for
  rb01tool and tools/bench (Go
  1.25+ and github.com/lvdlvd/gomavlink for tools/px4hil), the
  autopilot's own toolchain for the DUT.

## Start here

1. **doc/SETUP-ARDUPLANE.md** — wire it, build and flash the harness,
   build stock ArduPlane for the DUT, first boot, fly the 1 km square.
2. **doc/SETUP-PX4.md** — the same for PX4.
3. **doc/BENCH-OPERATIONS.md** — the bring-up ladder and every trap the
   bench has taught: RAM-only state, power-cycle discipline, engine
   models, resets.
4. **doc/FDM-TUNING.md** — the parameter table, the golden-check
   workflow, and which observable pins which coefficient.
5. **doc/TRUTH-TELEMETRY.md** — reading truth off the harness for your
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

- **doc/DESIGN.md** — the harness: topology (one SPI slave, 4 CS demux),
  the two deadlines, register models, CAN dictionary, pinout, milestones
  M0-M7 with bench-measured numbers.
- **doc/fdm-DESIGN.md** — the FDM: aero tables, power-based prop, ground
  model, servo lag, integration ladder F0-F5.
- **doc/REGMAPS.md** — the emulated register maps as the autopilots'
  drivers actually exercise them.
- **doc/F5-CHECKRIDE.md** — the acceptance rung definition.
- **doc/F5-TESTREPORT-2026-07-15.md** — the multi-night checkride
  report: 17+ defects found and fixed (the point of HITL), transition
  matrix, tuning campaign, final PASS numbers.
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
forever). Full table in doc/SETUP-ARDUPLANE.md.

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
sensor path on hardware. See tools/bench/README.md for the campaign
tools and the session procedures. `tools/px4hil` is the PX4
equivalent (simulator-MAVLink over TCP :4560).

## Hard-won operational truths (the short list)

The long list is doc/BENCH-OPERATIONS.md, doc/F5-TESTREPORT-2026-07-15.md
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

## Known issues

- **SPI slave desynchronisation (origin unlocated).** Rarely, the
  slave engine loses byte alignment with the DUT's bus master; the
  harness's `unexp` counter then climbs by thousands per second and
  ArduPlane boot-loops on the corrupt inertial-sensor replies. Detector: the
  counters (healthy = 0, always). Cure: reset the harness, confirm the
  counters return to 0, then boot the DUT. What shifts the command byte
  has not been found; see doc/BENCH-OPERATIONS.md.
- **CMD_NOISE (0x42) is accepted but not implemented.** The harness stores
  the frame and never reads it: the sensor noise and gyro bias walk are
  compiled-in constants (src/main.c, `noise()`), with no runtime mask or
  level control. The dictionary entry is reserved for that.
- The spiral mode over-converges and the short period is overdamped
  with the default coefficients (doc/FDM-TUNING.md has the knobs).
- tools/bench also speaks an experimental ArduPlane mode (HDGALT,
  `bench mode hdgalt` / `bench hdgaltcmd`) that exists only in a public
  fork, branch `hdgaltmode` of https://github.com/lvdlvd/ardupilot, with
  zero warranties. Nothing in this repository needs it; on stock
  ArduPlane those two commands are rejected. See tools/bench/README.md.
- Bootloader builds in ArduPilot share the board's build directory with
  the application build; build the bootloader from a clean
  `build/NucleoF767ZI` or the generated DroneCAN headers go missing.

## Licence

MIT, see LICENSE. The autopilot patch series in `dut/` are contributions
to ArduPilot and PX4 and carry those projects' licences (GPLv3 and
BSD-3-Clause respectively).
