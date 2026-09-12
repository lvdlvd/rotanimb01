# F5 checkride — stock ArduPlane (NucleoF767ZI) vs the harness

The acceptance rung from [fdm-DESIGN.md](fdm-DESIGN.md): EKF healthy, FBWA hands-off,
LOITER in wind, TECS climb — flown entirely on the bench.

## Wiring (F767 → harness)

| signal         | F767 (DUT)                | harness (G474)                                              |
| -------------- | ------------------------- | ----------------------------------------------------------- |
| SPI3 SCK       | PB3                       | PC10                                                        |
| SPI3 MISO      | PB4                       | PC11                                                        |
| SPI3 MOSI      | PB5                       | PC12                                                        |
| CS accel       | PD3                       | PC3                                                         |
| CS gyro        | PD4                       | PC2                                                         |
| CS baro        | PD5                       | PC0                                                         |
| CS mag         | PD6                       | PC1                                                         |
| PWM 1-4 (TIM3) | PC6 / PC7 / PC8 / PC9     | PA0 / PA1 / PB10 / PB11                                     |
| PWM 5-8 (TIM4) | PD12 / PD13 / PD14 / PD15 | PC6 / PC7 / PC8 / PC9                                       |
| CAN            | PD0 RX / PD1 TX           | PB3 RX / PB4 TX — via the two 5 V transceivers, STBY to GND |
| GND            | common ground FIRST       |                                                             |

DRDY lines stay unconnected (this config polls). Only PWM 1-4 (AETR)
matter for flight.

Ports on a mac: one `/dev/cu.usbmodem*` = ArduPlane MAVLink (CN13),
another = the ST-Link VCP. Harness cockpit = rb01tool on
its own usbmodem port. Do NOT halt/resume the F767 under openocd while
watching USB — macOS drops the CDC device and it looks like a crash.

## Session script

1. Power both boards, harness first. rb01tool up, `m` into FDM mode,
   `i` air-start OFF (start on the ground: h=0, IAS=0, level).
2. `mavproxy.py --master=<the DUT MAVLink port>` — expect HEARTBEAT and
   **no** "Config Error: Baro" (that error = SPI wiring problem).
   `status` / watch STATUSTEXT.
3. First session only: `param load doc/f5-bench.parm`, `reboot`.
4. Sensors sanity before any cal:
   - `status RAW_IMU` — accel ≈ (0, 0, -1 g) in mg, gyro ≈ 0.
   - SCALED_PRESSURE ≈ 1013.25 hPa / 15.0 °C.
   - GPS: fix 3D, 12 sats, position ≈ 52 N / 5.1 E (needs CAN leg up:
     rb01tool STATUS shows the feeder, mavproxy shows GPS_RAW_INT).
   - Airspeed ≈ 0.
5. Calibration gauntlet (expect PreArm complaints until done; exact
   remedies to be confirmed on the bench):
   - Accels: harness is level and still → simple/level cal
     (`accelcalsimple`, or `ahrstrim`; full 6-pose cal is impossible —
     the FDM won't hold poses).
   - Compass: fixed-yaw cal with the FDM's known heading:
     `magcal yaw 0` (harness serves earth field at psi=0).
   - Airspeed zero: `calpress` while IAS=0.
   - If a check stays red that we can't satisfy on a bench, note it and
     exclude the single bit via ARMING_CHECK — never 0.
6. EKF: wait for "EKF3 IMU0 origin set" + green `ekf` status. This IS
   the first acceptance gate.
7. RC overrides: MAVProxy `rc 3 1000` etc. (or a joystick module).
   Channels: 1=ail 2=ele 3=thr 4=rud, FLTMODE_CH 8 default — either
   override ch8 for modes or use `mode FBWA` directly.
8. Takeoff: `mode FBWA`, arm (`arm throttle`), `rc 3 1800`, rotate with
   `rc 2` — or `i` on rb01tool for an FDM air-start at 100 m / 25 m/s
   and skip the ground roll (cleaner: no ground model in the FDM).
9. Acceptance, in order:
   - **FBWA hands-off**: center sticks, plane holds attitude, no
     divergence over 2 min. Watch rb01tool PFD vs mavproxy ATTITUDE —
     they must agree (they're independent paths).
   - **LOITER in wind**: `mode LOITER`; rb01tool `W` for 5 m/s east
     wind, `g` for gusts. Circle stays anchored.
   - **TECS climb**: `mode GUIDED`/`long MAV_CMD_DO_CHANGE_ALTITUDE` or
     FBWA + throttle; +100 m: airspeed held within ±3 m/s, sustained
     climb, no phugoid divergence.
10. Snapshot everything: rb01tool snap ring + mavproxy logs.

## Known-different-from-real-life (do not chase these as bugs)

- (Superseded, F5-TESTREPORT night 3.) Harness sensors are noise-free
  (M7' not built). The noise layer went in during the checkride and the
  mission completed with it on; it is now runtime-scalable per sensor
  (`bench drive noise`), on at datasheet levels by default.
- Spiral mode over-converges (documented deviation; PARAM_SET tuning
  session planned — TRUTH vs EKF comparison unaffected).
- (Superseded by the ground model, F5-TESTREPORT night 2.) No ground model: h<0 freezes the FDM (CRASHED on the ctl line,
  rb01tool `i`/FDM_INIT to reset). The freeze serves parked REST truth
  (level-at-frozen-attitude -1 g, zero rates), so a mode-1 "ground
  start" is really an immediate 1 cm crash-freeze that behaves like a
  parked airframe — valid for the whole cal gauntlet, GPS feed
  included. For flight, air-start (`i`) and have FBWA armed BEFORE
  pressing it: in MANUAL/disarmed nobody flies the plane (neutral
  elevator, throttle locked at idle) and 100 m lasts ~9 s.
- Both DroneCAN feeders (on-board FDCAN3 + rb01tool -gps) are node 42:
  run exactly one.

## Troubleshooting ladder

- "Config Error: Baro" → SPI3/CS wiring or harness not in FDM/idle
  serving mode; check rb01tool DIAG unexp/stray counters.
- Gyro/accel unhealthy at 0.656 MHz: bus speed is the first suspect —
  the DUT-side SPIDEV lines in hwdef.dat pin the clock; harness engine
  is clean to 1.3 MHz on dummy-protocol devices, gyro class is not.
- GPS "No GPS" with CAN wired: transceiver STBY floating? bus-off?
  rb01tool STATUS shows FDCAN3 state; harness IT0 auto-recovers.
- PWM all 900/never moving: BRD_SAFETY_DEFLT not loaded, or not armed.
