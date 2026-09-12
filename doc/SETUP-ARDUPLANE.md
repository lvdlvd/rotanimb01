# Setup: stock ArduPlane on the bench

End to end, for someone with a NUCLEO-F767ZI, an STM32G474 board and the
parts in README.md. Every step names what "done" looks like. The traps
you will hit along the way are catalogued in BENCH-OPERATIONS.md — read
its first section before powering anything.

## 1. Wire it

Common ground FIRST. The harness is a slave on the DUT's SPI3; four
chip selects demux the four emulated devices.

| signal         | DUT NUCLEO-F767ZI         | harness G474                                           |
| -------------- | ------------------------- | ------------------------------------------------------ |
| SPI3 SCK       | PB3                       | PC10                                                   |
| SPI3 MISO      | PB4                       | PC11                                                   |
| SPI3 MOSI      | PB5                       | PC12                                                   |
| CS accel       | PD3                       | PC3                                                    |
| CS gyro        | PD4                       | PC2                                                    |
| CS baro        | PD5                       | PC0                                                    |
| CS mag         | PD6                       | PC1                                                    |
| PWM 1-4 (TIM3) | PC6 / PC7 / PC8 / PC9     | PA0 / PA1 / PB10 / PB11                                |
| PWM 5-8 (TIM4) | PD12 / PD13 / PD14 / PD15 | PC6 / PC7 / PC8 / PC9                                  |
| CAN            | PD0 RX / PD1 TX           | PB3 RX / PB4 TX — via two 5 V transceivers, STBY to GND |
| GND            | common ground             |                                                        |

DRDY outputs (harness PC4/PC5/PB6/PB7) stay unconnected: this
configuration polls. Only PWM 1-4 (aileron, elevator, throttle, rudder)
matter for flight; 5-8 are captured and reported. The CAN transceivers
need 5 V: at 3.3 V the bus never goes dominant and the DUT's CAN error
counter climbs forever (LEC = bit 0).

USB: the DUT's user USB (CN13) is SERIAL0/MAVLink; its ST-Link VCP
(USART3) is a debug console. The harness's USB (PA11/PA12) is the
pseudocan command/telemetry link; its console is USART1 PA9/PA10 at
115200 (on a breakout, through the ST-Link programmer's VCP; on a
NUCLEO-G474RE wire a USB-serial adapter or skip it — everything the
console prints is also reachable over pseudocan).

## 2. Build and flash the harness

Toolchain: arm-none-eabi-gcc 13 or newer (the sources are `-std=gnu23`),
GNU make, openocd. Nothing outside this repository is needed.

```
make -C src            # builds, runs the host-side gates (fdmgate, dccheck)
make -C src flash      # openocd over the ST-Link
```

`make flash` mass-erases the G474 and programs `rotanimb01.elf`. With
more than one ST-Link on the host:

```
make -C src flash OPENOCD_ADAPTER="-c 'adapter serial <serial>'"
```

Done looks like: the breakout LED (PC13) blinks, and the harness
enumerates as a USB CDC device named `rotanimb01 hitl-harness`. On Linux
it appears as `/dev/serial/by-id/usb-rotanimb01_hitl-harness_<uid>-if00`;
on macOS as a `/dev/cu.usbmodem*`. The console (115200) prints a
heartbeat line once a second: `fdm 0g` means the model is parked and
the sensor emulation serves rest truth.

Optional but recommended once: `selftest/` lets the harness master its
own sensor bus with seven jumpers (table at the top of selftest/main.c)
and replays a real driver's init sequences against the four register
models. `make -C selftest flash`, watch the console for PASS per device.

## 3. Build stock ArduPlane for the DUT

The board definition is two commits against upstream ArduPilot master,
shipped as patches in `dut/ardupilot/` (see the README there for what
they contain and why). Applied and built (plane + bootloader) on master as of
2026-07-27 (9bbfed9c91).

```
git clone https://github.com/ArduPilot/ardupilot.git && cd ardupilot
git submodule update --init --recursive
git am /path/to/rotanimb01/dut/ardupilot/*.patch
./waf configure --board NucleoF767ZI
./waf plane
```

Flash over SWD with openocd. The DUT needs connect-under-reset once
ArduPlane is running (a plain `halt` times out); this recipe works from
a cold board too:

```
openocd -f interface/stlink.cfg \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -f target/stm32f7x.cfg -c init -c "reset halt" \
  -c "program Tools/bootloaders/NucleoF767ZI_bl.bin verify 0x08000000" \
  -c "program build/NucleoF767ZI/bin/arduplane.bin verify 0x08018000" \
  -c "reset run" -c shutdown
```

The bootloader owns flash 0..96 K (its code in sector 0, the parameter
storage in sectors 1-2), the application starts at 0x08018000. Do NOT
`stm32f2x mass_erase` on later reflashes — it wipes the parameter
sectors; `program ... verify <addr>` erases only the sectors it writes.
Once the bootloader is on, `./waf plane --upload` over the DUT's USB
also works. The shipped `NucleoF767ZI_bl.bin` is prebuilt; to rebuild it,
`./waf configure --board NucleoF767ZI --bootloader && ./waf bootloader`
from a CLEAN `build/NucleoF767ZI` (the application build leaves
generated DroneCAN sources there that break the bootloader build).

Flash order matters when both boards are up: reset the harness first
(SPI bus quiet), then flash/reset the DUT. Flashing the DUT while the
harness SPI bus is live can wedge ArduPilot's accel driver in a reset
loop that a soft reboot does not clear.

Done looks like: the DUT enumerates as `ArduPilot NucleoF767ZI` on its
user USB and a MAVLink HEARTBEAT arrives (MAVProxy:
`mavproxy.py --master=/dev/serial/by-id/usb-ArduPilot_NucleoF767ZI_*-if00`).
"Config Error: Baro" at this point means an SPI wiring problem or a
harness that is not serving (check its heartbeat).

## 4. Host tools

```
cd rb01tool && go build          # interactive cockpit + PFD
cd tools/bench && go build       # campaign driver; GOOS=linux GOARCH=arm64 for a Pi
```

If the USB devices hang off a separate host (a Pi), run there:

```
bench serbridge -port 5760       # DUT MAVLink <-> TCP
```

and from the workstation add `-c <host-ip>:5760` to every `bench`
command. `bench drive ...` (harness commands) always runs on the host
the harness is plugged into. Device selection: `ROTANIMB01_HARNESS`,
`ROTANIMB01_CONSOLE`, `ROTANIMB01_DUT` name the devices; unset, a
`/dev/serial/by-id/` glob that matches exactly one device is used (two
ST-Links on one host make the console glob ambiguous — set the
variable).

## 5. First boot: parameters

The DUT parameters live in `doc/f5-bench.parm` (annotated — read the
comments, they are the accumulated reasons). Either

```
mavproxy> param load doc/f5-bench.parm
mavproxy> reboot
```

or, with the tool, `bench params -profile bench -reboot`, which stages
the tuning core plus the DroneCAN sensor config (GPS1_TYPE 9, ARSPD_TYPE
8 + SKIP_CAL, ARSPD_RATIO 1.6327), the synthetic INS calibration, and
verifies every write by readback. Do this before expecting any fly
check to pass.

Then sanity, all with the harness parked (`fdm 0g` or `fdm 1g`):

- RAW_IMU: accel ≈ (0, 0, -1000) mg, gyro ≈ 0.
- SCALED_PRESSURE ≈ 1013.25 hPa / 15.0 °C.
- GPS_RAW_INT: fix 3D, 12 sats, position at the feeder origin
  (45.52688 N, 1.667291 E — open farmland in central France). No fix
  with CAN wired = transceiver STBY floating, 3.3 V supply, or swapped
  PD0/PD1.
- AIRSPEED ≈ 0 and healthy (ARSPD_SKIP_CAL 1 — the harness's zero IS
  the truth, and skipping boot cal protects a DUT rebooted mid-flight).
- After ~30 s: "EKF3 IMU0 is using GPS", origin and home set. This is
  the first acceptance gate.

Calibration: the harness is level and still, so use the synthetic INS
values in the parm file (a 6-pose accel cal is impossible — the model
will not hold poses) and a fixed-yaw compass cal (`magcal yaw 0`; the
harness serves the earth field at heading 0 after a reset).

## 6. Harness init — every session, every harness reboot

```
bench drive mode 1          # six-DOF model on, parked on the gear
bench drive setpos 0 0      # at the origin (a bare mode cycle does not reset position)
bench drive cal             # PWM calibration, ArduPilot sign profile — RAM-ONLY
bench drive engine 912      # naturally aspirated 100 hp (default); 915 = turbo, for high altitude
bench drive tail 3          # read the heartbeat back: fdm 1g, cal echoed
```

All of this is RAM state: lost on every harness reboot, reflash or
power cycle. Without the cal the elevator is inverted for ArduPilot —
the aircraft never rotates, runs off the end of the runway at 50 m/s.
Then `bench reboot` the DUT and wait 45 s (clears a latched
"airspeed unhealthy").

## 7. Fly

The end-to-end check:

```
bench fly -alt 300
```

TAKEOFF-mode departure, climb, 60 s of hands-off FBWA at cruise
throttle, PASS/MARGINAL verdict with roll statistics. It passes from
clean state (roll ≤ 12°); a failure here means dirty harness state —
re-run section 6 from a harness reset.

The documented demo flight is the 1 km square at 150 m,
`doc/example-square-1km.waypoints`, laid out from the feeder origin
(south-west corner) north and east:

```
mavproxy> wp load doc/example-square-1km.waypoints
mavproxy> mode auto
mavproxy> arm throttle
```

`doc/example-straight-in.waypoints` is the same with a landing at the
origin. `bench loiter -takeoff` flies the two reference loiter legs
(standard 3°/s at 29 m/s, R 553 m; fast 6°/s at 38 m/s, R 363 m) with
demand-vs-achieved statistics and a CSV per leg; `bench probe`
cross-checks yaw vs GPS course vs EKF velocity vs airspeed.

## 8. Truth vs EKF

The harness reports the model's truth at 20 Hz over pseudocan
(TRUTH-TELEMETRY.md). `rb01tool` shows it as a PFD next to the model
state; ArduPilot's ATTITUDE / GLOBAL_POSITION_INT from MAVProxy is the
estimate. The two are independent paths through independent hardware:
on a healthy bench they agree to a degree in attitude and metres in
position (the checkride numbers are in F5-TESTREPORT-2026-07-15.md,
night 7). Do not use the DroneCAN GPS feed as truth — it is the
deliberately lagged, quantized feed the DUT consumes.

## 9. SITL first

Everything above also runs with no hardware at all, at 10x real time:

```
cd fdm && make sitljson && ./sitljson &
arduplane --model JSON:127.0.0.1 --speedup 10 --home 52.0,5.1,0,0 -w &
bench params -profile sitl -reboot
bench fly
```

Tune in SITL, then validate through the sensor path on the bench.
tools/bench/README.md has the campaign tools and the session checklist.
