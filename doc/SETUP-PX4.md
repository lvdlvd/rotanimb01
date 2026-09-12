# Setup: PX4 on the bench

The same harness, wiring and host tools as [SETUP-ARDUPLANE.md](SETUP-ARDUPLANE.md); only the
DUT firmware and a few procedures differ. Read sections 1, 2, 4 of that
guide first. PX4 flew a full GPS mission on this bench (takeoff, a 1 km
square at 150 m, landing) with all sensors emulated.

## 1. Build PX4 for the DUT

The board and the three upstream fixes it needs are five commits against
PX4 main as of 2026-08-28 (7c4bf078f4), shipped as patches in `dut/px4/`
(the README there lists them). Applied and built on that base.

```
git clone https://github.com/PX4/PX4-Autopilot.git && cd PX4-Autopilot
git checkout 7c4bf078f4            # or newer main, then resolve as needed
git submodule update --init --recursive
git am /path/to/rotanimb01/dut/px4/*.patch
make st_nucleo-f767zi_default
```

Toolchain: PX4 main does not build with arm-none-eabi-gcc 14+ at the
time of writing (its vendored Micro-XRCE-DDS client compiles a POSIX UDP
transport that calls getaddrinfo on NuttX, an error once implicit
declarations became errors). Use the gcc 10 toolchain PX4 documents,
e.g. `export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH`.
Build size ~1.35 MB of the 1792 KB application region.

## 2. Flash

There is **no PX4 bootloader** on this board: the image goes straight
to 0x08000000 over SWD. Consequence: flashing PX4 erases the ArduPilot
bootloader; going back to ArduPlane means re-flashing that bootloader
too ([SETUP-ARDUPLANE.md](SETUP-ARDUPLANE.md) section 3), not `--upload`.

```
openocd -f interface/stlink.cfg \
  -c "reset_config srst_only srst_nogate connect_assert_srst" \
  -f target/stm32f7x.cfg -c init -c "reset halt" \
  -c "program build/st_nucleo-f767zi_default/st_nucleo-f767zi_default.bin verify 0x08000000" \
  -c "reset run" -c shutdown
```

Flash sector 11 (256 KB at 0x081C0000) holds the parameters
(FLASH_BASED_PARAMS); `program ... verify` leaves it alone, a
`mass_erase` does not. Harness reset first, then the DUT (same
accel-wedge order as with ArduPilot).

Done looks like: the DUT enumerates as `PX4 Nucleo-F767ZI HITL` on the
user USB (MAVLink), NSH on the ST-Link VCP at **57600**. Set
`ROTANIMB01_DUT=/dev/serial/by-id/usb-PX4_PX4_Nucleo-F767ZI_HITL_0-if00`
for `bench serbridge` (the glob default matches both autopilots' names
only when one DUT is attached).

## 3. What the board definition already does

Board defaults (rc.board_defaults): `SYS_AUTOSTART 2110` (the bench
airframe), `SYS_DM_BACKEND 1` (mission store in RAM — there is no SD),
`BAT1_SOURCE 1` + `CBRK_SUPPLY_CHK 894281` (no power monitoring),
`COM_RC_IN_MODE 4` (no RC receiver). Sensors (rc.board_sensors): all
four devices on SPI3 in **mode 0** (`-m 0`, mandatory — a register-level
emulator cannot auto-detect the clock phase the way the real silicon
does), gyro and mag at 1 MHz, accel and baro at 4 MHz, and
`-R 8` (ROLL_180) on the BMI088 only — PX4's driver negates y and z
where ArduPilot's does not, so the harness's raw axes need one more
half-roll to land in FRD.

Airframe 2110 carries the model constants (mass 480 kg, stall 20 m/s
CAS, trim 30 m/s), the **harness's output map** (PWM 1-4 = aileron,
elevator, THROTTLE, rudder — ArduPlane's default order, which the
harness was built against; the SITL bridge's airframe uses a different
one, do not copy between them), `UAVCAN_ENABLE 2` + `UAVCAN_SUB_DPRES 1`
(the harness sends RawAirData = the differential-pressure bridge), and
two bench-specific deviations documented in the file: `ASPD_DO_CHECKS 0`
(the airspeed consistency checks latch off during the takeoff transient
and cannot be reset in flight) and `FW_LND_USETER 0` (no rangefinder;
with terrain-based flare the landing waypoint becomes an indefinite
"Holding" loiter).

## 4. First boot

On the NSH console (57600) after boot: `sensors status` — gyro, accel,
mag, baro voters all `state: OK`; `listener sensor_gps` — fix 3, 12
sats, the feeder origin; `uavcan status` — node 42 Online. `commander
status` disarmed with no failsafe.

Identity sensor calibration, once, saved to flash (there is nothing to
shake):

```
nsh> param set CAL_ACC0_ID 6946842
nsh> param set CAL_GYRO0_ID 6684698
nsh> param set CAL_MAG0_ID 458778
nsh> param set CAL_ACC0_PRIO 50
nsh> param set CAL_GYRO0_PRIO 50
nsh> param set CAL_MAG0_PRIO 50
nsh> param save
```

(Re-derive the IDs with `listener sensor_accel` etc. if the SPI
configuration ever changes.) If PX4 has learned a bias during a flight
flown with a wrong rotation, also zero `CAL_ACC0_[XYZ]OFF` and
`CAL_GYRO0_[XYZ]OFF` and save.

## 5. Harness init — every session, every harness reboot

```
bench drive mode 1
bench drive setpos 0 0
bench drive cal px4          # NOT the default profile: PX4 flips rudder too
bench drive engine 912
bench drive tail 3
```

`bench drive cal` with no argument applies the ArduPilot profile, whose
rudder sign is wrong for PX4. Then reset the DUT (after the harness)
and wait ~75 s: the EKF's GNSS position alignment can take longer than
the first preflight report — a failed preflight in the first minute is
a transient, not a fault.

## 6. Fly

Missions via QGroundControl or MAVProxy on the DUT's USB (or through
serbridge). `doc/example-straight-in.waypoints` (NAV_TAKEOFF, three
legs, NAV_LAND at the origin) completes end to end; the aircraft lands
~400 m long without terrain sensing, which is a tuning observation, not
a failure. `RWTO_TKOFF 1` + `MIS_TAKEOFF_ALT 100` are airframe defaults.

**When PX4 refuses a mode or a mission, the reason is on the NSH
console, not on MAVLink.** MAVLink only reports TEMPORARILY_REJECTED /
DENIED; the arming-check detail and the mission-feasibility verdict go
to PX4_WARN on the console. Worked example: "Mission start denied! No
valid mission" was `MIS_TKO_LAND_REQ 2` (rc.fw_defaults requires a
landing item) — invisible from the GCS side.

Truth for comparison: [TRUTH-TELEMETRY.md](TRUTH-TELEMETRY.md); PX4's estimate from
`listener vehicle_local_position` / `vehicle_attitude` or the MAVLink
stream.

## 7. SITL: tools/px4hil

`tools/px4hil` wraps the same `fdm.c` as a PX4 simulator (HIL_SENSOR +
HIL_GPS out, HIL_ACTUATOR_CONTROLS in, lockstep). It needs Go 1.25 and
depends on github.com/lvdlvd/gomavlink (fetched by `go build`). The
matching SITL airframe (output map 0 ail, 1 ele, 2 rud, 3 thr — NOT the
bench board's) is `dut/px4/sitl/0001-*.patch`, applied the same way as
the board patches:

```
git am /path/to/rotanimb01/dut/px4/sitl/*.patch
make px4_sitl_default                        # PX4 side, once
cd tools/px4hil && make px4hil && ./px4hil & # bridge listens on :4560
PX4_SIM_MODEL=px4hil_kitfox build/px4_sitl_default/bin/px4   # from the build rootfs
```

Flight-verified: runway takeoff to 100 m, loiter, EKF2 healthy.
