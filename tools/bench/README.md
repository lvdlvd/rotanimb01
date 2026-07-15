# bench — the F5 checkride driver

Runs on the workstation against the raspberry-pi-hosted bench (all four
USB devices on the pi; see doc/F5-TESTREPORT-2026-07-15.md):

- serbridge.py — TCP<->serial bridge, runs ON the pi (port 5760 =
  ArduPlane MAVLink on /dev/serial/by-id/usb-ArduPilot_...).
- mav.py — pymavlink connect helper. source_system MUST be 255
  (SYSID_MYGCS): ArduPilot silently ignores RC overrides from anyone
  else — a full evening was spent learning that.
- drive.py — harness pseudocan commands over the hitl-harness CDC:
  mode/airstart/wind/tail. Also runs on the pi (copy alongside).
- mission.py — the acceptance mission: FBWA takeoff (rotate at 25 m/s,
  hand the climb to TECS), LOITER calm, LOITER in 5 m/s wind + gusts,
  DO_CHANGE_ALTITUDE TECS climb; retries takeoffs (crashes re-park).

Bench facts: uhubctl on the pi power-cycles individual ports (the cure
for ArduPilot's watchdog latch, which survives soft resets in backup
RAM); after reflashing the harness mid-session, power-cycle the F767 —
its main loop stalls on the dead SPI bus and trips that watchdog.
