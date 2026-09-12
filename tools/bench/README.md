# bench — the flight-campaign driver (Go)

One dependency-light binary (stdlib + golang.org/x/sys) that drives both
rigs: the SITL rig (`fdm/sitljson` + `arduplane --model JSON`) and the
real bench (its own `serbridge` on the host the USB devices hang off).
It replaced the python campaign scripts that flew the F5 checkride; the
mission logic, thresholds and CSV formats are 1:1 ports of those and
the SITL results were identical.

Build: `go build` here (and `GOOS=linux GOARCH=arm64 go build -o
bench-linux-arm64` for a Raspberry Pi bench host). Test: `go test` —
the MAVLink codec is byte-pinned against pymavlink-generated reference
frames, so a broken encoder or a wrong CRC_EXTRA fails loudly instead
of being silently dropped by ArduPilot.

## Subcommands

Against the DUT (MAVLink over TCP, `-c addr`, default `127.0.0.1:5760`):

    bench arm       [-c addr]
    bench mode      [-c addr] manual|fbwa|autotune|rtl|loiter|takeoff
    bench param     [-c addr] NAME [VALUE]              (get, or set then read back)
    bench params    [-c addr] -profile bench|sitl [-reboot]
    bench fly       [-c addr] [-alt 150]
    bench loiter    [-c addr] [-tag run] [-takeoff] [-dur 5m]
    bench probe     [-c addr] [-dur 2m]
    bench tune      [-c addr] [-rev 200]
    bench watch     [-c addr] [-dur 30s]
    bench disarm    [-c addr]                           (force)
    bench reboot    [-c addr]
    bench mode      [-c addr] hdgalt                    (experimental fork mode, below)
    bench hdgaltcmd [-c addr] [-hdg deg] [-trate deg/s] [-alt m] [-crate m/s]

On the bench host, next to the USB devices:

    bench drive     <mode|airstart|setpos|gps|wind|engine|param|cal|tail> [args]
    bench serbridge [-dev path] [-port 5760]
    bench usbreset  [-dev path]                         (Linux, root)

The bench is reached through `serbridge` on that host — pass
`-c <host-ip>:5760` (mDNS `.local` names don't resolve through Go
sockets on macOS). Device defaults for `drive`/`serbridge`/`usbreset`:
`ROTANIMB01_HARNESS`, `ROTANIMB01_CONSOLE`, `ROTANIMB01_DUT`, else a
`/dev/serial/by-id/` glob that must match exactly one device
(drive.go).

- **params** stages the tuning core of doc/f5-bench.parm (the list in
  missions.go is authoritative) and verifies every write by readback;
  `-profile bench` adds the DroneCAN sensor config + INS cal shims +
  ARSPD_RATIO 1.6327, `-profile sitl` the SITL airspeed backend.
  `-reboot` latches the ones that need it.
- **fly** is the end-to-end check: TAKEOFF-mode departure, climb,
  60 s FBWA hands-off at cruise throttle, PASS if worst |roll| and
  |pitch| stay under 25° (a clean bench flies it under 12°), else
  MARGINAL.
- **loiter** flies the two reference legs (standard 3°/s: 29 m/s
  R553; fast 6°/s: 38 m/s R363) with demand-vs-achieved stats and a
  circle fit over the last 120 s; CSV per leg. `-takeoff` departs
  first, else it assumes the aircraft is already airborne.
- **probe** flies straight FBWA and cross-checks yaw vs GPS course vs
  EKF velocity vs airspeed — the estimator/sensor-consistency check
  that caught the ARSPD_RATIO defect. It sends a mode change; it is
  not passive.
- **tune** runs the pitch AUTOTUNE campaign (elevator reversals,
  in-mode altitude-floor recovery, in-mode gain readback).
- **watch** streams mode, altitude, climb, IAS, throttle, pitch, roll,
  altitude and speed error once a second. Passive.
- **drive** sends harness commands (pseudocan) and tails the harness
  console for the readback; reference in [doc/BENCH-OPERATIONS.md](../../doc/BENCH-OPERATIONS.md).
- **usbreset** re-enumerates a wedged CDC device without resetting the
  firmware (USBDEVFS_RESET; [doc/BENCH-OPERATIONS.md](../../doc/BENCH-OPERATIONS.md)).

## HDGALT: an experimental ArduPlane mode (not needed for anything here)

`bench mode hdgalt` and `bench hdgaltcmd` drive a heading-and-altitude
hold mode that is NOT in stock ArduPlane. It is an experimental patch
living in a public fork, branch `hdgaltmode` of
https://github.com/lvdlvd/ardupilot — flight mode number 27 and the
HDGALT_COMMAND message (id 52100) from that fork's `hdgalt_dev.xml`
dialect, hand-encoded in mavlink.go. It comes with zero warranties and
is not required to use this bench: every guide, mission and check in
this repository runs on stock ArduPlane. Against a stock build the two
commands are simply rejected (unknown mode / unknown message).

## SITL rig quickstart

    cd fdm && make sitljson && ./sitljson &
    arduplane --model JSON:127.0.0.1 --speedup 10 --home 52.0,5.1,0,0 -w &
    bench params -profile sitl -reboot
    bench fly
    bench loiter -takeoff -tag mytag

## Bench session checklist

1. Power state: `bench drive tail 3` (on the bench host) — harness
   heartbeat, `fdm 1g` parked. After ANY harness reboot: `bench drive
   cal` (the PWM cal is RAM-only; without it the elevator is INVERTED
   for ArduPilot — no rotation, railed elevator, ground-roll
   overspeed). Hub port power cycles can reboot the harness too when it
   shares hub power with the target.
2. `bench serbridge` on the bench host (it reopens the device after
   power cycles), then from the workstation `bench params -profile
   bench`. This re-stages and VERIFIES the tuned gains — a completed
   ArduPilot AUTOTUNE holds its gains in RAM only and any DUT reboot
   silently reverts them.
3. `bench fly -c <host>:5760`, then the campaign.
4. DUT watchdog latch (survives soft resets, suppresses baro cal): a
   hub port power cycle of the DUT (`uhubctl -l <hub> -p <port> -a
   cycle` on the bench host), then re-run step 1.

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
mavlink_test.go (generate it with pymavlink, or from a live capture).
