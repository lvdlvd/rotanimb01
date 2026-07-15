# F5 checkride — test report, bench nights of 2026-07-14/15

DUT: stock ArduPlane (master, ~4.7-dev) on NucleoF767ZI, hwdef branch
`nucleo-f767-hitl`. Harness: rotanimb01 on the G474 breakout, FDM mode 1
(Kitfox V). Bench driver: pymavlink over a Pi serial bridge (F767 side)
plus pseudocan on the harness CDC; compinator as independent bus witness.

## Verdict

**The full HITL chain is proven end to end.** ArduPilot's own drivers
consume the emulated BMI088/BMP388/RM3100 at rate; the DroneCAN GPS +
airspeed feed delivers a 3D fix (52.000000 N / 5.100000 E, 12 sats,
correctly lagged); EKF3 initialises, sets origin and home; magcal,
arming and FBWA all function; servo PWM closes the loop back into the
FDM. FBWA demonstrably stabilised the airframe mid-flight (50 s of
wings-level, roll within ±2°) — attitude control against the emulated
sensors WORKS.

**Not yet passed: the transition to flight.** Every mechanism tried for
"getting airborne" fails for a distinct, now-understood reason (matrix
below). The air-start teleport is fundamentally hostile to an EKF+
controller that is already flying the plane. This needs a small FDM
feature (proposal at the end), not more attempts.

## What was proven working (in dependency order)

| layer                                     | evidence                                                                                             |
| ----------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| SPI sensor emulation vs ArduPilot drivers | boot probes clean, gyro/accel/baro/mag stream at configured rates, unexp/stray 0                     |
| Baro/mag/accel plausibility               | RAW_IMU (0,0,-1000) mg parked; 1013.25 hPa; earth field 48.33 µT @ 65.6° incl                        |
| PWM capture of ArduPilot servo outputs    | harness heartbeat mirrors SERVO_OUTPUT_RAW incl. FBWA demands                                        |
| DroneCAN feeder wire format               | compinator witnessed complete 10-frame Fix2 + 3-frame RawAir + NodeStatus, tids/toggles/CRCs correct |
| ArduPilot GPS/airspeed ingestion          | GPS_RAW_INT fix 3 / 12 sats / origin coords; AIRSPEED healthy after ARSPD_SKIP_CAL                   |
| EKF3                                      | "EKF3 IMU0 is using GPS", origin+home set, level parked estimate (0.0°/-0.0°)                        |
| Cal + arming path                         | fixed-yaw magcal accepted; arm succeeds parked; PreArm list emptied legitimately                     |
| FBWA attitude control                     | 50 s wings-level segment (roll -0.6..3.9°) before ground contact                                     |

## Defects found and fixed by this campaign (the point of HITL)

Harness/n-array side:
1. FDCAN clock + pins + IRQ handlers + vector entries all scoped to
   TRANSPORT_CAN — the on-board feeder was stone dead in USB builds
   (three separate fixes; found via scope + NULL-vector crashdump).
2. `fdcan3_it1` never cleared IR.RF1N — first frame ever RECEIVED
   latched an interrupt storm: 100 % CPU, killed console+USB+SWD.
   Masked for days because the DUT never transmitted (dead transceiver);
   every "bizarre" harness death traced here.
3. Fix2 needs 13 frames/burst; G4 FDCAN has 3 tx buffers — silent
   truncation of every transfer. Fixed with canmsgq (now n-array lib).
4. Crash-freeze served free-fall truth (zero specific force) — poisoned
   the DUT's AHRS while "parked". Fixed: rest truth at frozen attitude.
5. ...which still parked at the CRASH attitude, poisoning every ground
   calibration after any crash. Fixed: level park, yaw kept, mag
   recomputed ("lands on its wheels").
6. Crash left impact velocity in the GPS feed (position frozen,
   velocity 25 m/s — EKF poison). Fixed: zero v/IAS when parked.
7. `fdm_publish_truth` gated behind `!fdm_crashed` — the rest-truth
   override never reached the samplers. Fixed.
8. Bit-identical pressure tripped ArduPilot's stuck-baro detector —
   a noise-free baro reads as broken. Fixed: ±1 Pa dither (first
   mandatory tenant of the M7' noise layer).
9. All CAN drain loops bounded per the aerospace rule (the RF1N storm
   vindicated it: "bounded by FIFO contents" is not a bound).

ArduPilot-side knowledge captured (nothing patched, all parm/procedure):
10. INS_ACC_ID must be SAVED to storage (only a real cal, or an explicit
    param_set, does this) — post-flash-wipe boots set it in RAM only,
    failing "3D Accel calibration needed" forever.
11. Watchdog-reset state lives in backup RAM, survives every soft
    reset, and suppresses baro calibration ("skipping calibration after
    WDG reset") — only a full power removal clears it.
12. Arming consistency checks (DCM-vs-EKF, GPS-vs-AHRS, mag field)
    assume quasi-steady flight: in-air arming during a phugoid is
    effectively impossible.
13. F7 bxCAN ESR decoding: REC climbing + LEC bit-dominant + zero rx =
    transceiver cannot drive the bus (was: dead 5 V rail, then swapped
    PD0/PD1). The compinator's ACKs masked it from the harness view.

Bench infrastructure lessons: single-TT USB hubs pass enumeration but
starve bulk data with 4 FS devices (all-silent-ports symptom); stale
`cat` processes steal serial bytes; `pkill -f` self-matches over ssh
(use `fuser -k` on the device instead); pymavlink needs param_id
matching on PARAM_VALUE and an add_message monkeypatch.

## The transition-to-flight matrix (all attempts)

| #   | mechanism                                     | outcome                   | root cause                                                   |
| --- | --------------------------------------------- | ------------------------- | ------------------------------------------------------------ |
| 1   | arm parked, air-start 100 m                   | tumble, down in 7 s       | poisoned compass (cal'd against crash attitude) + teleport   |
| 2   | air-start disarmed, settle, arm               | never flew                | FDM_INIT line lost (usb bad line), FDM stayed parked         |
| 3   | throttle into glide remnant                   | already crashed           | reconnect gap: idle glide beat me to the ground              |
| 4   | boot ArduPlane mid-glide                      | gyro cal in a tumble      | failsafe = servo NEUTRAL, not flight trim: upset during boot |
| 5   | parked boot → disarmed air-start → arm in air | arm REFUSED               | arming consistency checks vs phugoid motion                  |
| 6   | armed teleport from 1500 m                    | 98 m/s dive, down in 14 s | EKF glitch transient + firewalled surfaces into real physics |

## Night 2 addendum: the ground model flew it

The balloon-drop proposal below was superseded by the owner's call: a
minimal-fidelity tricycle ground model instead (no pitch change in the
ground roll, accelerate to Vstall, rotate, climb at Vx, no flaps). As
built: ground-speed/yaw/pitch DOFs, rolling friction with lift
unloading, nosewheel steering with speed washout (first version spun
donuts at 21 m/s — ArduPilot's steering loop vs an unwashed nosewheel),
rotation about the mains against the CG-forward weight moment (Vr
emergent), liftoff when lift + thrust vertical carries the weight;
gentle touchdowns roll out, harsh contacts re-park the wreck level with
a latched crash count. The prop became power-based per the owner's
engine data (100 hp, eta 0.50 static to 0.85 at 50%-power cruise,
momentum-theory static cap ~1600 N) — host golden: 134 m ground run vs
POH ~90 m, liftoff emergent at 26 m/s.

Night-2 finds along the way: RC_CHANNELS_OVERRIDE is silently ignored
unless the sender is SYSID_MYGCS (255) — every "throttle" of night 1
was actually idle; reflashing the harness mid-session stalls the DUT's
main loop on the dead SPI bus and trips the watchdog latch again
(uhubctl per-port USB power-cycling from the pi is the software cure);
a first tuning pass (AIRSPEED_CRUISE 25 / MIN 17 / MAX 38,
WP_LOITER_RAD 120, TRIM_THROTTLE 45) — the default 60 m loiter radius
at untuned speeds demanded 70-degree banks.

### First complete flight (night 1, FBWA manual takeoff)

Ground roll dead straight 0-26 m/s, rotation at 26, and after a
pilot-induced-oscillation settled: 90+ seconds of hands-off FBWA climb,
roll within +-1 deg, 48 -> 119 m, still climbing at script end.

### Full acceptance mission (night 2, one run, no crashes)

Takeoff (pilot-style Vy climb: full power, pitch for 27 m/s) on the
FIRST attempt to 150 m; LOITER calm 75 s (alt band 140-217 m); wind
5 m/s east + gusts injected mid-orbit, LOITER continued (132-183 m),
circle center displaced 211 m downwind but bounded; DO_CHANGE_ALTITUDE
+100 m: TECS climbed with IAS held 30-40 m/s but did not complete
+90 m within 150 s; mission ended with the aircraft in a controlled
orbit at 178 m. Total airborne time ~8 minutes.

### Acceptance verdict

| leg                                       | verdict                                                                                                      |
| ----------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| EKF healthy                               | PASS                                                                                                         |
| Takeoff (bonus: not in the original plan) | PASS — repeatable, physics-emergent Vr                                                                       |
| FBWA hands-off                            | PASS — 90 s, roll +-1 deg                                                                                    |
| LOITER in wind                            | FUNCTIONAL — anchored and bounded; 74-deg banks and a 211 m downwind offset say L1/TECS need airframe tuning |
| TECS climb                                | FUNCTIONAL — climbs with IAS in band, but slow; tuning                                                       |

The bench itself has no open defects. The remaining deltas are
ArduPlane parameter tuning against this airframe (TECS_CLMB_MAX, L1
period, pitch limits) and FDM PARAM_SET tuning — which is precisely the
work this bench exists to host. Next: the multi-MCU stage — ardu +
vnavigator/cnav on a separate microcontroller, then the visual
positioning from the parallel session; end goal, per the owner: "a pile
of microcontrollers flying a simulated kitfox around just like i would
do with my eyes and my hands and my meat brains, with some claim to
fidelity."

## Night 3 addendum: noise on, tuning holds

Full sensor noise layer added (datasheet-scale, physical units before
quantization: gyro 0.2 deg/s, accel 5 mg, mag 20 nT, baro's +-1 Pa
already in). The acceptance mission completed WITH noise at the same
envelope as without (max|roll| 72-75 vs 71-74, same TECS behavior):
the estimator and controllers are not living off the harness's
unnatural cleanliness. Found on the way: at ArduPilot's disarmed 10%
throttle the power-based prop pushes ~160 N — the aircraft taxied
itself in circles at idle; the ground model grew toe brakes below 15%
power. Post-crash the DUT's estimator can latch an inverted attitude
that never re-converges on static data (suspected railed gyro-bias
state); the bench driver's cure is an automatic DUT reboot, exercised
four times unattended in the final run. AUTOTUNE was flown (200 s of
scripted doublets) but did not modify gains — its engagement
conditions are a next-session study; rate-loop tuning for the 70-deg
bank overshoots remains open (attitude TCONST experiments made things
worse and were reverted). Takeoff reliability with noise: 1-in-4 with
the current crude scripted speed-hold climb — driver polish, not
physics. The `stray` counter read zero through every run: the old
"stray trickle" backlog item is closed by the permanently-selected
spislave redesign.

## (superseded) Proposal: the "balloon drop" (freeze-at-altitude) air-start

Add a HOLD flag to FDM_INIT: teleport to altitude but stay FROZEN,
serving rest truth at altitude (level, -1 g, zero rates, static GPS at
500 m, baro at altitude) — physically a plane hanging from a balloon.
Then:

1. The EKF absorbs the position/baro step while everything is static —
   the same benign convergence we get parked on the ground.
2. Arming consistency checks pass (quasi-steady by construction).
3. Arm FBWA, throttle up — servos move, FDM still frozen (harmless).
4. RELEASE (second command): integration starts from v=0 — a real
   balloon drop. The Kitfox builds speed nose-down for ~5 s and FBWA —
   already flying it, with a converged estimator — recovers to level.
   Every sensor stream is CONTINUOUS through the whole sequence.

Estimated cost: a flags bit in FDM_INIT, a hold state in the 1 kHz slot,
~20 lines. Alternative considered and rejected for now: ground-roll
model (bigger, better long-term — enables AUTO takeoff/landing testing).

## Session parm/procedure artifacts

- doc/f5-bench.parm grew: INS_ACC1_CALTEMP 25, ARMING_CRSDP_IGN 1,
  ARSPD_SKIP_CAL 1 + ARSPD_OFFSET 0 (reboot-in-flight-proof).
- INS_ACC_ID must be param_set once by hand after any param wipe.
- After any watchdog event: full power cycle before expecting baro cal.
- Bench driver: scratchpad bench/{mav.py,drive.py,serbridge.py};
  serial bridge on the Pi at :5760.
