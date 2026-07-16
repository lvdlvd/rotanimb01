# rotanimb01 — a HITL flight bench for stock ArduPlane

One STM32G474 (the **harness**) impersonates a flight controller's whole
sensor suite — BMI088 gyro+accel, BMP390 baro, RM3100 magnetometer — at
the **SPI register level** on the DUT's own sensor bus, feeds DroneCAN
GPS + airspeed, captures the DUT's eight servo PWM outputs, and runs a
Kitfox V 6-DOF flight dynamics model at 1 kHz to close the loop. A stock
ArduPlane build on a NucleoF767ZI (the **DUT**) boots against it, probes
"real" sensors, calibrates, arms, takes off and flies — its real
drivers, its real EKF3, its real control loops, none the wiser.

The same FDM also flies as an ArduPilot **SITL** backend on the
workstation, so tuning campaigns run at 10x real time in pure software
and are then validated through the emulated-sensor hardware path. As of
the F5 checkride both rigs fly the owner-defined loiter legs
identically to the meter (doc/F5-TESTREPORT-2026-07-15.md).

## The loop

```
             ┌──────────────────── harness (G474) ────────────────────┐
             │                                                        │
             │   fdm.c 1 kHz ──► truth ──► sensor models (noise,     │
             │   Kitfox V 6-DOF            quantization, bias walk)   │
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
             │              DUT: stock ArduPlane, NucleoF767ZI        │
             │        real drivers → EKF3 → TECS/L1 → servos          │
             └─────────────────────────────────────────────────────────┘
```

Control/observability sidechannel: the harness's USB CDC speaks
**pseudocan** (text-framed CAN, lib/fmtcan). Over it run the FDM
commands (mode, air-start, wind, PWM cal, parameters) and the 20 Hz
TRUTH_* telemetry. `rb01tool` is the interactive cockpit; `tools/bench`
drives unattended missions.

## Directory map

| dir          | what                                                                                       |
| ------------ | ------------------------------------------------------------------------------------------ |
| `src/`       | harness firmware (n-array app): SPI-slave engine users, sensor regfile models, PWM capture, FDCAN3 DroneCAN feeder, pseudocan command loop |
| `fdm/`       | the 6-DOF model (`fdm.c`, freestanding float32+CORDIC) + host `golden` gates + `turncheck` + **`sitljson`** (the SITL backend wrapper) |
| `physics/`   | mode-0 kinematic model (speed/climb/turn commands) + its golden                            |
| `bmp390inv/` | BMP390 compensation inverter (truth pressure → raw counts for the emulated trim)           |
| `rb01tool/`  | Go console cockpit: live PFD, single-key physics steering, DroneCAN GPS host feeder        |
| `tools/bench/` | bench + SITL mission drivers (Go): param staging, departures, loiter legs, probes, autotune |
| `doc/`       | all design docs and reports (below)                                                        |

## Documentation index

- **doc/DESIGN.md** — the harness: topology (one SPI slave, 4 CS demux),
  the two deadlines, register models, CAN dictionary, pinout, milestones
  M0-M7 with bench-measured numbers.
- **doc/fdm-DESIGN.md** — the FDM: Kitfox V aero tables, power-based
  prop (owner's engine data), ground model (tricycle spec), servo lag,
  integration ladder F0-F5.
- **doc/REGMAPS.md** — the emulated register maps as ArduPilot's
  drivers actually exercise them.
- **doc/F5-CHECKRIDE.md** — the acceptance rung definition.
- **doc/F5-TESTREPORT-2026-07-15.md** — the multi-night checkride
  report: 17+ defects found and fixed (the point of HITL), transition
  matrix, tuning campaign, final PASS numbers.
- **doc/f5-bench.parm** — the DUT parameter file, heavily annotated
  with every ArduPilot trap the bench discovered.

## The two rigs

### HITL (the real bench)

All four USB devices hang off a raspberry pi (`slon.local`):
ArduPlane's MAVLink CDC, the harness pseudocan CDC, the harness ST-Link
(console + flash), the Nucleo ST-Link. `tools/bench/` runs on the
workstation and reaches the DUT through a TCP↔serial bridge on the pi
(port 5760); harness commands go over ssh. `uhubctl` on the pi
power-cycles individual ports — the cure for ArduPilot's watchdog
latch, which survives soft resets in backup RAM.

Wiring: DUT SPI3 (PB3/4/5) + CS PD3/4/5/6 → harness SPI3 + PC0-PC3;
DUT PWM1-8 (PC6-9, PD12-15) → harness PA0/PA1/PB10/PB11 + PC6-9;
DUT CAN1 PD0/PD1 ↔ harness FDCAN3 PB3/PB4 via TJA1051 transceivers
(5 V supply — 3.3 V cannot drive the bus, LEC=Bit0 forever).
The ArduPilot side is branch `nucleo-f767-hitl` in the ardupilot tree
(hwdef + two bootloader fixes); build `./waf configure --board
NucleoF767ZI && ./waf plane`.

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
tools and the session procedures.

## Hard-won operational truths (the short list)

The long list is doc/F5-TESTREPORT-2026-07-15.md and the parm file
comments. The ones that cost the most:

- **The harness PWM cal is RAM-only.** Any harness reboot reverts to
  the all-positive default = inverted elevator for ArduPilot. Resend
  after every harness reboot (`bench drive cal`); note uhubctl cycles
  can reboot the harness when it shares hub power with the target.
- **A completed ArduPilot AUTOTUNE keeps its gains in RAM.** The first
  DUT reboot silently reverts them. `param_set` them explicitly and
  verify by readback at session start (`bench params` does).
- **ARSPD_RATIO 1.6327** (= 2/rho0): the harness diff pressure is
  exactly 0.5·rho0·IAS²; ArduPilot's 2.0 default reads 10.7% high.
- ArduPilot ignores RC_CHANNELS_OVERRIDE unless source_system ==
  SYSID_MYGCS (255). Silently.
- Message-interval floods before takeoff starve the climb loop —
  raise stream rates only once airborne.
- After a watchdog event, power-cycle before expecting baro cal;
  after reflashing the harness mid-session, power-cycle the DUT (its
  main loop stalls on the dead SPI bus and trips that watchdog).
