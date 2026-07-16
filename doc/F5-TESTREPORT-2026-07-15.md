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

**UPDATE (nights 6-7, 2026-07-16): everything above is closed.** The
ground model made TAKEOFF mode the standard departure (night 6, after
disabling the DCM fallback the gyro bias walk corrupts); the SITL rig
completed the pitch autotune; and after three latent bench defects
fell (RAM-only PWM cal, RAM-only autotune roll gains, ARSPD_RATIO),
both owner-defined loiter legs fly on the bench IDENTICALLY to the
pure-software model: std 29.0±0.1 m/s R568, fast 38.0±0.1 m/s R374.
See "Night 7" below. F5 acceptance: takeoff, FBWA, LOITER (both
legs), TECS — all PASS through the full emulated sensor path.

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
four times unattended in the final run. AUTOTUNE initially appeared
inert; reading AP_AutoTune.cpp explained it: learning is event-based
(needs filtered rate demand > ~30 deg/s AND attitude error > 12 deg,
per-event FF fit, 4-event median, then D- and P-raise ladders to
provoked oscillation), our 4-s stick HOLDS produced almost no events,
and stop() RESTORES all gains on mode exit unless both D and P limits
were established. Re-flown with proper bank-crossing reversals and
in-mode gain readback: roll learned FF 3.00 / P 4.84 / D 0.38 (9x the
default FF — the airframe's 53 deg/s full-aileron authority), pitch
reached FF 1.62 / P 1.57 / D 0.17 before running out of sky
(candidates in f5-bench.parm). Night-4 A/B verdict: the autotuned
ROLL gains are good (mission completes, no departures); both autotuned
PITCH sets destabilize — their ladders ended in crashes and the
oscillation detector was reading the departure itself. Pitch stays at
defaults; its FF did converge (~1.65 twice), so a completed tune wants
more altitude and gentler floor recovery. Demand-vs-achieved logging
(now in the mission driver) reframes the loiter question: even at the
owner-prescribed rate-turn radius (300 m), L1 demands median 37 deg
of bank where geometry needs 19 — the orbit never settles — and
median roll tracking error is ~20 deg across all gain sets (60 ms
servo lag phase cost + flying 32-36 m/s instead of 25 because TECS
can't hold speed in turns with default pitch). Next: dataflash logs
for offline loop analysis instead of more blind runs. The takeoff
driver's proportional EMA speed-hold made first-attempt departures
routine. Takeoff reliability with noise: 1-in-4 with
the current crude scripted speed-hold climb — driver polish, not
physics. The `stray` counter read zero through every run: the old
"stray trickle" backlog item is closed by the permanently-selected
spislave redesign.

## Night 5: the instrumented loiter campaign

Owner definitions encoded as bench-standard legs: STANDARD loiter =
3 deg/s at (cruise+stall)/2; FAST loiter = 6 deg/s at cruise. The
instrumented mission (25 Hz ATTITUDE, 10 Hz NAV/HUD, CSV) flew them
and the analysis produced a chain of findings, each unblocking the
next:

1. The model's true cruise is ~38 m/s at 45% throttle — matching the
   owner's 90 kt Kitfox (the prop model IS his engine). The bench had
   been flying with AIRSPEED_CRUISE 25, an arbitrary early guess.
2. First legs commanded 22 m/s — BELOW the accelerated stall at the
   banks demanded (40 deg -> load factor 1.3 -> stall 22.6). The
   "rosette" loiters were the aircraft mushing in and out of stall
   while TECS hunted throttle 0-100%. Corrected legs: standard 29 m/s
   R553, fast 38 m/s R363.
3. Mechanism traps: WP_LOITER_RAD is LATCHED at LOITER entry (change
   requires a mode re-entry); DO_CHANGE_SPEED does not take effect in
   LOITER (set AIRSPEED_CRUISE instead — TECS reads it live).
4. The remaining L1 problem, now precisely characterized: the aircraft
   orbits a point hundreds of metres from the commanded center at
   SATURATED bank demand (median 37-40 deg where geometry wants 9-22),
   radius forced to V^2/(g tan 40) ~ 200 m regardless of command —
   loiter capture never completes. This is the open tuning target
   (NAVL1_PERIOD/damping study with the corrected speeds).
5. Fidelity milestone: the gyro bias random walk makes DCM (the backup
   estimator) drift ~18 deg while EKF3 tracks fine, tripping the
   DCM-vs-EKF arming consistency check — the bench now reproduces
   ArduPilot's real-world arming annoyances. Bench cure: reboot.
6. The scripted FBWA manual climb is the campaign's reliability
   bottleneck (entangled with FC param state). With the ground model +
   nosewheel washout now in, ArduPlane's own TAKEOFF mode (abandoned
   on night 3 for the pre-ground-model rotation bug) should be re-
   tried as the standard departure — auto-throttle, TECS-flown, no
   script fragility.

## Night 6: TAKEOFF mode, and the DCM saboteur

Re-tried ArduPlane's TAKEOFF mode per the night-5 recommendation. First
attempts reproduced the night-3 no-rotation failure — and the harness
console (which sees the servo PWM directly) showed why: +19 deg of
sustained NOSE-DOWN elevator during the ground roll. The controllers
were intermittently flying DCM's attitude, not EKF3's. The gyro bias
random walk (deliberate fidelity, see night 3) walks DCM ~18 deg off
while EKF3 tracks fine; whenever ArduPilot's AHRS elected the DCM
fallback, the elevator chased a phantom pitch-up. AHRS_OPTIONS 1
(disable fixed-wing DCM fallback) fixed TAKEOFF mode on the first
attempt after setting it. TKOFF_* values in doc/f5-bench.parm.

Two failure classes in one root cause: the same DCM drift also causes
the intermittent "DCM Roll/Pitch inconsistent" arm refusals. The bias
walk has now claimed both an estimator consumer and an arming check —
exactly the class of integration bug HITL exists to surface.

## Night 7: the SITL rig — tune at 10x, validate through the hardware

Real-bench tuning iterations cost ~10 min each (real-time flight +
power-cycle liturgy), and the remaining work was all tuning. So the
SAME fdm.c the harness flies got a second harness: fdm/sitljson wraps
it as an ArduPilot SITL "JSON backend" (lockstep UDP), and stock SITL
arduplane flies it at --speedup 10 with no hardware in the loop.
SIL under the HIL: iterate fast against identical physics, then push
the result through the real SPI/DroneCAN sensor path for validation.
Tools: tools/bench/sitl{params,fly,tune,loiter}.py; lessons encoded
there (TKOFF_THR_MINSPD=0 for wheeled starts; AUTOTUNE_AXES=2;
in-mode gain readback; in-mode altitude-floor recovery).

Results, ~30 min wall clock:

1. PITCH AUTOTUNE COMPLETED — the thing the bench never got through.
   26 elevator reversals, both ladders clean ("PitchD: 0.4981",
   "PitchP: 8.3447", "Pitch: Finished"), no floor recovery needed at
   400 m working altitude. FF converged to 1.6387 — matching the ~1.65
   the bench runs converged to twice before dying (cross-check that
   SITL physics = bench physics). Full set in doc/f5-bench.parm.
2. The night-5 open problem DISSOLVED. A/B in SITL:
   - new pitch gains: standard leg ias 29.0+-0.0 (cmd 29), roll 9.1
     (geometry wants 8.8), R 569+-0 (cmd 553); fast leg ias 38.0+-0.0,
     roll 22.8 (wants 22), R 374+-19 (cmd 363). Capture completes,
     nothing saturates.
   - default pitch gains: the ENTIRE night-5 pathology reproduces —
     ias 38 vs 29 commanded, +-25 m phugoid, throttle railing 0-80%,
     bank demand oscillating to 30+-12.
   The "L1 capture mystery" was never L1: with FF=0/P=0.345 the pitch
   loop can't track TECS demands, energy control degenerates into a
   phugoid, and L1's geometry never materializes. NAVL1_PERIOD sweeps
   were treating the symptom.
3. Bench validation was a saga of its own that closed THREE latent
   defects before it passed:
   - TKOFF_THR_MINSPD must be 0 for a wheeled standing start (nonzero
     is a hand-launch feature: throttle stays suppressed until GPS
     speed exceeds it = deadlock, "Timeout AUTO" spam).
   - The harness PWM cal is RAM-ONLY. A uhubctl power cycle had
     rebooted the harness and silently reverted the elevator to the
     all-positive default = INVERTED for ArduPilot (pwm-high must be
     nose-up). Symptom: elevator railed 18.8 deg nose-down, ground
     roll through 53 m/s, no rotation. Fix + procedure: drive.py
     'cal' (elevator defl -25 deg), resend after EVERY harness reboot.
   - The night-4 autotuned ROLL gains had silently REVERTED to
     defaults: a completed AUTOTUNE keeps its gains in RAM only, and
     the first DUT reboot after night 4 dropped them. Instrumented
     loiter (truth-vs-EKF-vs-servo cross-correlation) pinned it:
     EKF roll = truth roll (r=1.000, no estimator lag), servo tracks
     the FC's demand with no lag, but TRUTH ROLL LAGS DEMAND 4.7 s —
     the default-gain roll loop inside L1's ~17 s loop = limit cycle.
     THIS was night 5's "L1 capture mystery" all along; every L1
     period sweep was flown on default roll gains without knowing it.
     Roll gains are now param_set (persists in FC storage) and the
     parm file says to verify by readback each session.
   - Bycatch: ARSPD_RATIO must be 2/rho0 = 1.6327, not the 2.0
     default — the DUT read IAS 10.7% high (GPS 41.8 vs IAS 45.7 at
     zero wind) since night 1.
4. FINAL BENCH VALIDATION PASS (2026-07-16), identical to SITL to
   the meter, full sensor path in the loop (SPI emulation + noise +
   gyro bias walk + DroneCAN GPS/airspeed + EKF3):
   - standard leg: ias 29.0+-0.1 (cmd 29), roll 9.1+-0.6 (geometry
     8.8), R 568+-1 m (cmd 553; SITL flew 569).
   - fast leg: ias 38.0+-0.1 (cmd 38), roll 22.7+-1.2 (geometry 22),
     R 374+-2 m (cmd 363; SITL flew 374).
   The HITL bench now flies exactly like the model says it should.
   The residual +3% radius overshoot is L1's known loiter behavior,
   identical on both rigs.

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
