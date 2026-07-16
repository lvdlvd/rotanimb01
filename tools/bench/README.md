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

## SITL rig (sitl*.py) — tune fast, validate on the bench

`fdm/sitljson` wraps the SAME fdm.c the harness flies as an ArduPilot
SITL "JSON backend": tuning campaigns run at --speedup 10 with no
hardware in the loop, then the resulting gains are validated through
the real sensor path with benchloiter.py. Bring-up:

    cd fdm && make sitljson && ./sitljson &
    arduplane --model JSON:127.0.0.1 --speedup 10 --home 52.0,5.1,0,0 -w &
    python3 tools/bench/sitlparams.py      # stage params, reboot to latch
    python3 tools/bench/sitlfly.py         # end-to-end check: TAKEOFF+FBWA
    python3 tools/bench/sitltune.py        # pitch AUTOTUNE to completion
    python3 tools/bench/sitlloiter.py tag [new|old]   # loiter legs + stats

Lessons encoded in these scripts:

- TKOFF_THR_MINSPD MUST be 0 for a wheeled standing start. Nonzero
  suppresses throttle until GPS ground speed exceeds it (hand-launch
  feature) — from rest that's a deadlock, symptom "Timeout AUTO" spam.
- AUTOTUNE_AXES=2 tunes pitch without touching the validated roll gains.
- AP_AutoTune::stop() restores gains on mode exit unless BOTH the D and
  P ladders completed — read gains while still in the mode.
- Altitude-floor recovery during autotune stays IN-MODE (AUTOTUNE flies
  like FBWA): neutral stick + full throttle, resume reversals after.
- SITL re-execs itself on MAVLink reboot and strips -w: params staged
  via param_set survive the reboot that latches them.

## The RAM-only PWM cal trap (cost one bench afternoon)

The harness PWM calibration (channel deflections, incl. the elevator
SIGN FLIP an ArduPilot DUT needs — see `drive.py cal`) lives in RAM
only. Any harness reboot silently reverts to the all-positive default
= INVERTED elevator for ArduPilot. Symptom: takeoff accelerates
through rotate speed with the elevator railed and never lifts;
`ctl e` on the harness console shows +deg where the FC wants nose-up.
Note uhubctl port power cycles can reboot the harness too when it
shares hub ports with the target. Procedure: after ANY harness
reboot, run `python3 drive.py cal` (on the pi) before flying.
