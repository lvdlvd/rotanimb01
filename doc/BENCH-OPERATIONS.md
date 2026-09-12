# Bench operations — the ladder and the traps

Everything here was learned the expensive way. The bench is unforgiving
in one specific manner: most of its state is RAM-only on two different
microcontrollers, and each of them can be reset by the other's power
rail. Read "RAM-only state" before anything else.

## RAM-only state (what a reset costs you)

On the **harness**, lost on every reboot, reflash, or power cycle:

- the PWM calibration (`bench drive cal [ardupilot|px4]`) — without it
  the elevator is inverted for both autopilots: no rotation on takeoff,
  elevator railed nose-down, ground-roll overspeed;
- the FDM mode (`bench drive mode 1`) — the harness boots in mode 0
  (kinematic, parked) and the DroneCAN GPS feed produces nothing until
  the model runs;
- the engine model (`bench drive engine 912|915`);
- the sensor noise mask (CMD_NOISE), wind, and every PARAM_SET.

On the **DUT** (ArduPilot): a completed AUTOTUNE holds its gains in RAM
until they are `param_set` explicitly; the watchdog-reset flag lives in
backup RAM and survives every soft reset (see below).

Rule: after ANY reset of anything, re-establish the harness state, read
it back (`bench drive tail`), and log it. `bench params` re-stages and
verifies the DUT side.

## Power-cycle discipline

- A hub port power cycle is the only cure for two things: ArduPilot's
  watchdog latch (backup RAM, "skipping calibration after WDG reset",
  suppresses baro cal; only a full power removal clears it), and a
  wedged DUT after an SWD reset with the harness bus live. `uhubctl -l
  <hub> -p <port> -a cycle` from the host does it without anyone
  present.
- Cycling a port that shares power with the harness reboots the harness
  too — and costs all the RAM state above. Cycle deliberately, then
  re-establish, then log.
- The DUT does not survive a warm reset while the harness SPI bus is
  live (reproduced repeatedly). Keep the harness in mode 0 or reset it
  first whenever the DUT boots or is flashed. DroneCAN traffic during
  boot is a separate bus and is harmless.
- Flash order: harness first, then DUT. Never `mass_erase` the DUT — it
  wipes the parameter sectors; `program ... verify <addr>` erases only
  what it writes.
- Harness USB CDC wedge: `serbridge` loops "serial lost (EOF)" while the
  harness console still streams — the model is alive, only its USB is
  wedged. Do not reset the harness (kills the flight): a
  USBDEVFS_RESET ioctl on the device node re-enumerates it and the
  flight survives. On a Pi, 493 "Undervoltage detected" events later
  turned out to be the cause of a whole evening of such wedges: give
  the host a proper supply before chasing USB bugs.

## Engine model vs altitude

The default engine is a naturally aspirated Rotax 912 (74.6 kW, no
critical altitude). It **cannot hold altitude above ~14,000 ft**. Staged
above that, the aircraft sinks or departs, and it looks exactly like an
attitude or control failure. It is not. `bench drive engine 915`
(105 kW, 2000 N static, critical altitude 4572 m) is the turbo model for
high-altitude work; it is RAM-only. Use 912 for takeoff realism (915's
sea-level excess power makes departures wild), switch to 915 before a
climb above ~3000 m, and read it back after every cycle.

## Resets and what they actually reset

- `bench drive mode 0` then `mode 1` zeroes ALTITUDE. It does not zero
  position (add `bench drive setpos 0 0`) and no FDM mode cycle resets
  HEADING — psi carries over from the previous flight.
- A harness SWD reboot (openocd `reset run`) zeroes psi and everything
  else. That is the recipe for repeatable-heading departures. Check psi
  in the heartbeat before judging any takeoff.
- After any such reset, REBOOT THE DUT. The model jumps in position and
  altitude under a running estimator, which cannot follow; the symptoms
  ("heading estimate invalid", "high accelerometer bias", "horizontal
  position unstable") look like a sensor regression and are not.
- Hypothesis, seen twice, not yet isolated: `bench drive mode 0` issued
  under a RUNNING, armed DUT boot-loops the DUT. Until tested: disarm
  or stop the DUT side first, or expect a power cycle afterwards.

## The bring-up ladder (verified order)

1. Host services up (serbridge on the DUT device; harness console
   reachable). Harness hard reset if in doubt:
   `openocd -f interface/stlink.cfg -c "adapter serial <harness stlink>" -f target/stm32g4x.cfg -c "init; reset run; exit"`
   (stop anything holding the harness devices first).
2. Harness init: `drive mode 1`, `drive setpos 0 0`, `drive cal
   <profile>`, `drive engine 912`, `drive tail 3` — read back `fdm 1g`
   and the cal echo.
3. Reset the DUT (after the harness). Wait 45 s (ArduPilot) / ~75 s
   (PX4) for the estimator and the airspeed binding.
4. `bench params -profile bench` (ArduPilot; PX4's equivalents are in
   the airframe) — BEFORE expecting a fly check to pass.
5. `bench fly -alt 300` → FLY CHECK PASS (roll ≤ 12°). It only fails
   with dirty harness state; go back to 1.
6. The mission / loiter legs / probe.

Altitude staging, if you need the aircraft high: **climb, do not
teleport.** An air-start teleport (`drive airstart`) under a running
autopilot is EKF-hostile: the active mode keeps its pre-teleport
altitude target, TECS commands a max-rate descent, the aircraft passes
Vne in seconds and departs. If you must teleport: set FBWA first
(attitude hold, no auto-throttle), `drive airstart <alt> <ias> <hdg>`,
wait ~3 s, then enter the altitude-holding mode fresh (a mode set to
the CURRENT mode is a silent no-op — it must be a genuine mode entry to
recapture the altitude target). The air-start IAS must be within the
trim envelope (≤ ~40 m/s) or `fdm_trim` fails silently and nothing
moves.

## The DUT boot loop: a desynchronised SPI slave

Symptom: the DUT re-enumerates on USB every ~9 s, prints the ArduPlane
banner, dies right after "BMI088: found gyro"; its next clean boot
reports a watchdog/hard-fault reset reason. Cycling the DUT does not
clear it.

Signature, on the harness console (`bench drive tail`): the `unexp`
counter (unexpected register writes) climbing by thousands per second.
Healthy is 0, permanently — the checkride report's acceptance
condition. That is the SPI-slave engine having lost byte alignment
with the master; every command byte is misread, the replies are wrong,
and the DUT's IMU driver hard-faults on them.

Recovery is on the HARNESS, not the DUT: a DUT power cycle alone cannot
clear it (do not spend an hour on the wrong device). Reset the harness
(SWD/openocd, or a power cycle), then CONFIRM on the harness console
that `unexp`/`stray`/`mid` have returned to zero and the per-device SPI
frame counters are counting up from zero again. A re-enumeration alone
is not recovery: the first reset attempt on record re-enumerated with
the counters unchanged and the DUT kept looping. Only then let the DUT
boot, re-establish the RAM-only state, and hold 30 minutes with the
counters at 0 before trusting the bench.

Two facts to keep straight, because both have misled people:

- Harness mode 0 is the kinematic flight model, NOT an SPI-quiet state.
  The register emulation runs in both modes from the moment the
  harness boots; the only way to silence the bus is to reset or unpower
  the harness.
- `bench probe` is not read-only: it sends an FBWA mode command before
  it records (so do `fly`, `loiter`, `tune`; only `watch` and `param`
  reads are passive).

What shifts the command byte in the first place has not been located.
It is an open item (README, known issues); the counters are the
detector.

## Airspeed and GPS feed facts

- The harness sends RawAirData from the first instant of boot,
  unconditionally, at 20 Hz: ArduPilot's airspeed init is one-shot and
  can only bind a DroneCAN node whose RawAirData it has ALREADY seen —
  a node that starts talking later never binds for that boot
  (ARSPD_DEVID stuck at 0). Fixed harness-side; nothing to do, but it
  explains why the harness must be up before the DUT.
- The GPS feed is Fix2 at 5 Hz with a configurable lag (default 150 ms,
  `drive gps 1 <lag_ms>`); `drive gps 0` silences it for GPS-denied
  segments. Both the on-board feeder and rb01tool's host feeder are
  node 42 — run exactly one.
- ARSPD_RATIO must be 1.6327 (= 2/ρ0): the harness's differential
  pressure is exactly ½ρ0·IAS²; at the 2.0 default IAS reads 10.7% high.

## Watching a flight

`bench watch -dur 10m` streams mode, altitude, climb, IAS, throttle,
pitch, roll, altitude error and speed error once a second — enough to
see a stale altitude target or a throttle railing before the aircraft
does. The harness heartbeat (`drive tail`) shows the truth side: mode,
altitude, psi, the post-lag surface deflections, and the four
per-channel failsafe digits (0 live, 1 holding, 2 at trim fallback).

## drive command reference

```
bench drive mode <0|1>                          kinematic / six-DOF
bench drive airstart <alt_m> <ias> [hdg] [n e]  trim & teleport (see above)
bench drive setpos <north_m> <east_m>           ground position from the origin
bench drive gps <0|1> [lag_ms]                  DroneCAN feeder on/off, lag
bench drive wind <n_mps> <e_mps> [gust_cms] [tau_s]
bench drive engine <912|915>                    engine preset (PARAM 41-43)
bench drive cal [ardupilot|px4]                 PWM sign profile — RAM-only
bench drive param <index> <value>               any FDM parameter (FDM-TUNING.md)
bench drive tail [secs]                         harness console tail
```

Wire format and the full dictionary: src/canmsg.h. rb01tool exposes the
same commands as single keys (`m` mode, `i` air-start, `w`/`W` wind,
`g` gusts, `f` PFD view) and decodes everything the harness sends.
