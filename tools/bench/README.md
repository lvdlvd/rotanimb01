# bench — the flight-campaign driver (Go)

One dependency-light binary (stdlib + golang.org/x/sys) that drives both
rigs: the SITL rig (`fdm/sitljson` + `arduplane --model JSON`) and the
real bench (its own `serbridge` on the pi). It replaces the python
campaign scripts that flew the F5 checkride — those are checkpointed at
commit 537e750 for A/B re-verification; the mission logic, thresholds
and CSV formats here are 1:1 ports.

Build: `go build` here (and `GOOS=linux GOARCH=arm64 go build -o
bench-linux-arm64` for the pi). Test: `go test` — the MAVLink codec is
byte-pinned against pymavlink-generated reference frames, so a broken
encoder or a wrong CRC_EXTRA fails loudly instead of being silently
dropped by ArduPilot.

Validation status: codec tests + full SITL campaign (params/fly/
loiter/probe/tune) all pass with numbers identical to the python
originals; the pi-side `drive`/`serbridge` paths still await an
on-hardware run.

## Subcommands

    bench params    [-c addr] -profile bench|sitl [-reboot]
    bench fly       [-c addr] [-alt 150]
    bench loiter    [-c addr] [-tag run] [-takeoff] [-dur 5m]
    bench probe     [-c addr] [-dur 2m]
    bench tune      [-c addr] [-rev 200]
    bench disarm|reboot [-c addr]
    bench drive     <mode|airstart|wind|cal|tail> [args]     (on the pi)
    bench serbridge [-dev path] [-port 5760]                 (on the pi)

`-c` defaults to `127.0.0.1:5760` (SITL); the bench is the same port
through serbridge on the pi — pass `-c <pi-ip>:5760` (mDNS `.local`
names don't resolve through Go/python sockets on macOS).

- **params** stages the full doc/f5-bench.parm tuning core and verifies
  every write by readback; `-profile bench` adds the DroneCAN sensor
  config + INS cal shims + ARSPD_RATIO 1.6327, `-profile sitl` the SITL
  airspeed backend. `-reboot` latches the ones that need it.
- **fly** is the end-to-end check: TAKEOFF-mode departure, climb,
  60 s FBWA hands-off at cruise throttle, PASS/MARGINAL verdict.
- **loiter** flies the two reference legs (standard 3°/s: 29 m/s
  R553; fast 6°/s: 38 m/s R363) with demand-vs-achieved stats and a
  circle fit over the last 120 s; CSV per leg. `-takeoff` departs
  first, else it assumes the aircraft is already airborne.
- **probe** flies straight FBWA and cross-checks yaw vs GPS course vs
  EKF velocity vs airspeed — the estimator/sensor-consistency check
  that caught the ARSPD_RATIO defect.
- **tune** runs the pitch AUTOTUNE campaign (elevator reversals,
  in-mode altitude-floor recovery, in-mode gain readback).

## SITL rig quickstart

    cd fdm && make sitljson && ./sitljson &
    arduplane --model JSON:127.0.0.1 --speedup 10 --home 52.0,5.1,0,0 -w &
    bench params -profile sitl -reboot
    bench fly
    bench loiter -takeoff -tag mytag

## Bench session checklist

1. Power state: `bench drive tail 3` (on the pi) — harness heartbeat,
   `fdm 1g` parked. After ANY harness reboot: `bench drive cal`
   (the PWM cal is RAM-only; without it the elevator is INVERTED for
   ArduPilot — no rotation, railed elevator, ground-roll overspeed).
   uhubctl port power cycles can reboot the harness too when it shares
   hub power with the target.
2. `bench serbridge` on the pi (it reopens the device after power
   cycles), then from the workstation `bench params -profile bench`.
   This re-stages and VERIFIES the tuned gains — a completed ArduPilot
   AUTOTUNE holds its gains in RAM only and any DUT reboot silently
   reverts them (the night-5/7 "L1 mystery").
3. `bench fly -c <pi>:5760`, then the campaign.
4. DUT watchdog latch (survives soft resets, suppresses baro cal):
   `sudo uhubctl -l 1-1 -p 3,4 -a cycle` on the pi, re-run step 1.

## Lessons encoded in the code (don't relearn these)

- source_system MUST be 255 (SYSID_MYGCS) or ArduPilot silently
  ignores RC_CHANNELS_OVERRIDE.
- Message-interval floods before takeoff starve the climb loop —
  streams go up only once airborne.
- PARAM_VALUE replies cross-contaminate — always match the param id.
- WP_LOITER_RAD latches at LOITER entry (mode re-entry to change);
  DO_CHANGE_SPEED is inert in LOITER (set AIRSPEED_CRUISE live).
- TKOFF_THR_MINSPD must be 0 for a wheeled standing start.
- AP_AutoTune::stop() RESTORES gains on mode exit unless BOTH the D
  and P ladders completed — read gains while still in AUTOTUNE, then
  param_set them explicitly.

## Extending the MAVLink dictionary

mavlink.go carries only the messages this bench speaks. To add one:
wire layout is MAVLink-sorted (fields by descending type size,
extensions appended unsorted, trailing zeros truncated), add the
message's CRC_EXTRA to `crcExtra`, and pin a reference frame in
mavlink_test.go (generate it with pymavlink while it still runs, or
from a live capture).
