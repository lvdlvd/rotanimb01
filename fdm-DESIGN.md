# HITL layer 2 — Kitfox V flight dynamics model

Status: design. Companion to hitl-sim-DESIGN.md (the harness), which
this layers on top of: the kinematic speed/climb/turn command model of
that doc becomes **mode 0**; this document specifies **mode 1**, a
closed-loop 6-DOF fixed-wing model. The DUT's PWM outputs (rudder,
aileron, elevator, throttle — flaps deliberately ignored) drive forces
and torques on a Kitfox Model V; the model integrates rigid-body motion
with wind, and the *same* sensor quantization / DRDY / register layer
from the harness doc consumes the resulting truth state. Fidelity goal:
"enough to exercise ArduPlane" — trims correctly, stalls gracefully,
has plausible modes (phugoid, short period, dutch roll) and performance
numbers a Kitfox pilot recognizes. Not a certification model.

## Where it runs, and the data flow

The 6-DOF runs **on the harness G474**, in thread context, at 1 kHz
(fixed dt = 1 ms). Rationale: PWM→sensor latency is then one
integration step with zero host jitter; the alternative (host-side FDM,
state streamed back over CAN) inserts milliseconds of variable latency
inside the DUT's rate loops and requires the harness to dead-reckon
between updates anyway. The host's role shrinks to environment control
and telemetry consumption.

```
DUT servo PWM ──► pwmin (width µs, 50 Hz frames)
                    │ zero-order hold
                    ▼
   PWM calibration → δa δe δr (rad), δt (0..1)
                    │ servo lag + rate limit
                    ▼
   ┌── 6-DOF @ 1 kHz ────────────────────────────┐
   │ airdata (Va, α, β, q̄) ← wind (steady+gust)  │
   │ aero F,M ← stability derivatives             │
   │ prop T,Q ← throttle, Va                      │
   │ gravity; integrate u v w | p q r | quat | pos│
   └──────────────┬───────────────────────────────┘
                  ▼                         ▼ CAN
   sensor truth {f_b, ω_b, m_b, p_static, q̄}   TRUTH_* telemetry
                  ▼                         (host GPS/airspeed
   harness sensor layer (unchanged):         feeder → DUT's own
   quantize per live configs, DRDY, regs     CAN port, 5 Hz + lag)
```

Nothing in the harness doc's SPI/register machinery changes. The FDM
replaces the coordinated-turn kinematics as the truth source; a CAN
command selects mode 0 / mode 1 (mode 0 stays — it's the right tool for
sensor bring-up before the FDM is tuned).

## Inputs: PWM → controls

Per-channel calibration, CAN-settable and stored in the harness:
{min µs, trim µs, max µs, sign, full deflection}. Defaults 1000/1500/
2000 µs, ±20° aileron, ±25° elevator, ±25° rudder, throttle 0..1.
Mapping is piecewise-linear around trim (ArduPlane's SERVOn_TRIM need
not sit at 1500).

Actuators are not ideal: each surface passes through a first-order lag
(τ ≈ 60 ms) plus a rate limit (≈ 300 °/s) — cheap, realistic, and it
smooths both the 50 Hz zero-order hold and PWM quantization. Throttle
gets a slower lag (τ ≈ 0.3 s) standing in for engine response.

Failsafe: any channel with capture age > 100 ms holds its last value
for 0.5 s, then goes to trim (throttle to idle); flagged in STATUS.
This makes DUT reboots and cable pulls non-events.

## The model

State (13): body velocity u v w; body rates p q r; attitude quaternion
q_nb; position NED (origin-relative). Position accumulates in int32
centimetres (1 cm resolution, ±21 000 km range) to dodge float32
accumulation error; everything else float32 on the FPU.

**Airdata.** v_air = v_body − Rᵀ_nb·w_n (wind in NED). Va = |v_air|,
α = atan2(w_air, u_air), β = asin(v_air/Va), q̄ = ½ρ(h)Va². ρ from the
same ISA layer the baro already uses — one atmosphere, everywhere.

**Wind** = steady NED vector (CAN) + gusts: per-axis first-order
Gauss–Markov (Dryden-lite), σ and correlation length CAN-settable, off
by default. That is enough turbulence to exercise the EKF and TECS
without implementing full Dryden transfer functions.

**Aero**, classic component buildup, all coefficients dimensionless,
b̂ = b/2Va, ĉ = c̄/2Va:

    CL = CL0 + CLα·α            (stall-blended, below)
    CD = CD0 + k·CL²
    CY = CYβ·β + CYδr·δr
    Cl = Clβ·β + Clp·p·b̂ + Clr·r·b̂ + Clδa·δa + Clδr·δr
    Cm = Cm0 + Cmα·α + Cmq·q·ĉ + Cmδe·δe
    Cn = Cnβ·β + Cnp·p·b̂ + Cnr·r·b̂ + Cnδa·δa + Cnδr·δr

Lift/drag act in the wind frame, rotated to body by α (β small-angle);
forces F = q̄S·(...), moments M = q̄S·(b·Cl, c̄·Cm, b·Cn).

**Stall**: blend CL between the linear model and flat-plate
2·sign(α)·sin²α·cosα with a C¹ polynomial smoothstep (3t²−2t³) over
[α_stall − Δ, α_stall + Δ], α_stall ≈ 15°, Δ ≈ 3° — same shape as the
Beard–McLain sigmoid blend but with zero transcendentals (the classic
form needs two expf calls per evaluation). Cm gets a stable pitch-down bias past
the blend. This keeps ArduPlane's stall-adjacent tests (slow flight,
TECS underspeed) well-behaved instead of numerically explosive. No
asymmetric wing-drop modelled — flagged as a fidelity limit.

**Propulsion** (Rotax 912 + fixed-pitch prop), Beard–McLain quadratic:

    T = ½ρ·Sprop·Cprop·((k_m·δt)² − Va²)

with k_m and Cprop·Sprop fitted to two anchors: static thrust and
cruise (see calibration). Engine torque reaction Q = −k_Q·(k_m·δt)²
about body-x is included with a small k_Q (it gives ArduPlane a
realistic constant aileron trim demand); P-factor and slipstream over
the tail are omitted — the rudder-on-takeoff realism they'd buy isn't
worth the parameters we can't measure.

**Equations of motion**: standard rigid body,
m·(v̇_b + ω×v_b) = F_aero + F_thrust + Rᵀ_nb·(0,0,mg);
I·ω̇ + ω×Iω = M. Diagonal inertia (Ixz ≈ 0 is defensible for a
high-wing taildragger at this fidelity). Quaternion kinematics with
per-step renormalization.

**Integration**: semi-implicit Euler at 1 kHz. The fastest mode in this
model (roll subsidence, τ ≈ b²·ρ·S·|Clp|·V/8Ixx ≈ 0.1 s) is ~100×
slower than the step — RK4 buys nothing here. The sensor scheduler
(2 kHz gyro ODR) samples the latest state twice per FDM step;
irrelevant, since everything above ~50 Hz is below the DUT's own
filtering anyway. Sensor truth per step: f_b = (F_aero + F_thrust)/m
(note: gravity is *not* in specific force — this formulation hands the
accelerometer value over for free), ω_b = (p,q,r), m_b = Rᵀ_nb·m_n,
p_static = ISA(−pd), q̄ for airspeed. IMU sits at the CG in v1; the
lever-arm terms (ω×(ω×r) + ω̇×r) are a 10-line option if offset-IMU
testing is ever wanted.

## Numerics: plain C + FPU + CORDIC, no libm

Hard requirement: the FDM shares the G474 with the sensor simulation,
so the math core is plain C float32, the M4F FPU, and `lib/cordic.h` —
**no libm, no doubles**. The division of labor:

- **FPU in hardware**: + − × (1 cy), VDIV.F32 and VSQRT.F32 (14 cy).
  Every sqrt in the model (Va, quaternion renorm via 1/sqrt) is a
  hardware instruction, not a CORDIC job and not a library call.
- **CORDIC peripheral**: sincos (one op yields both) and atan2
  (phase). Convention: **CORDIC is called from the 1 kHz FDM thread
  context only**, never from ISRs — no locking, no save/restore; the
  sensor layer's mode-0 use lives in the same context.

Per-step transcendental budget — everything else is arithmetic:

| need | ops | served by |
|------|-----|-----------|
| α = atan2(w_air, u_air) | 1 | CORDIC phase |
| β = atan2(v_air, √(u²+w²)) | 1 (+VSQRT) | CORDIC phase (exact; replaces asinf) |
| sinα, cosα (wind→body rotation, flat plate) | 1 | CORDIC sincos |
| quaternion → R_nb | 0 | pure mul/add — the quaternion state exists precisely so no Euler trig appears in the loop |
| stall blend | 0 | polynomial smoothstep (above) |
| gust filters | 0 | exp(−dt/τ) precomputed at PARAM_SET time (slow path; CORDIC exp = cosh + sinh if ever needed) |
| ISA ρ(h), p(h) | 0 | the pow()-shaped ISA formula becomes a cubic Hermite table over −100..5000 m (16 knots, < 1 Pa over the Kitfox envelope), shared with the baro layer |
| trim solver | few | init-time slow path, not in the loop |

Steady state: ~3 CORDIC ops + 2–3 VSQRT/VDIV + ~400 single-cycle
FLOPs ≈ 1–2 µs of each 1000 µs step, alongside the harness's ~2 % SPI
load. Footprint: fdm.c a few KB flash, state ~100 B, param table ~60
floats — invisible on 512 K / 128 K.

*Measured at F1 (as-built):* 2972 cycles ≈ 17.7 µs per step — the
estimate above missed that the full step (airdata + aero + prop +
integrator + sensor-truth derivation) runs an order of magnitude more
FLOPs than the aero core alone. Still under 2 % of the 1 kHz budget
and invisible next to the SPI load; the ≤ 2 µs ambition is retired,
the F1 gate figure is ≤ 20 µs.

Discipline, enforced not hoped: all constants f-suffixed; build with
`-Wdouble-promotion -Werror`; the F1 gate greps the map file for libm
symbols (a stray `pow` quietly linking soft-double is exactly the
regression this catches). Position stays int32 cm as above — the one
place float32 genuinely can't serve.

The host golden build keeps working because n-array already has the
pattern: fdm.c calls the cordic float API, which the emul library
implements on the host (examples/cordic's diff-test machinery) — same
source, bit-similar results, CI without hardware.

## Kitfox V parameters

Defaults below; **⚑ = verify against the POH / actual aircraft**,
**◆ = estimated, tune via the calibration checklist**. Everything is a
CAN-settable parameter (see PARAM_SET) — tuning sessions must not
require reflashing.

| param | default | source |
|-------|---------|--------|
| m (flying weight)     | 480 kg  | ⚑ empty + pilot + fuel |
| S wing area           | 12.2 m² | ⚑ (~132 ft²) |
| b span                | 9.75 m  | ⚑ (32 ft) |
| c̄ mean chord          | 1.25 m  | S/b |
| Ixx / Iyy / Izz       | 700 / 650 / 1100 kg·m² | ◆ Roskam radii of gyration (Rx .25, Ry .38, Rz .39) |
| CL0 / CLα             | 0.30 / 5.0 rad⁻¹ | ◆ |
| α_stall / CLmax       | 15° / 1.6 | ⚑ from flaps-up Vs (worked below) |
| CD0                   | 0.050 | ◆ draggy strutted high-wing; worked below |
| k (induced)           | 0.056 | 1/(πeAR), e ≈ 0.75, AR = 7.8 |
| Cm0 / Cmα / Cmq / Cmδe| 0.02 / −0.8 / −12 / −1.2 | ◆ typical GA |
| CYβ                   | −0.3  | ◆ |
| Clβ / Clp / Clr / Clδa / Clδr | −0.08 / −0.42 / 0.03 / 0.17 / 0.01 | ◆ |
| Cnβ / Cnp / Cnr / Cnδa / Cnδr | 0.07 / −0.03 / −0.10 / −0.01 / −0.08 | ◆ |
| P engine              | 73.5 kW | confirmed: 912 ULS, 100 hp |
| k_m, Cprop·Sprop, k_Q | fitted  | calibration anchors below |

### Calibration checklist — observables → parameters

This is how "somewhat realistic" is actually achieved: each number a
Kitfox pilot knows pins one or two coefficients. Worked with the
defaults (ρ = 1.225, m = 480 kg):

1. **Flaps-up stall speed** ⚑ (say 44 mph ≈ 19.7 m/s) →
   CLmax = 2mg/(ρSVs²) = 9418/5826 ≈ **1.62**. Sets α_stall given CLα.
2. **Cruise at 75 %** ⚑ (say 100 kt ≈ 46 m/s): thrust power
   0.75·P·η ≈ 41 kW → T ≈ 890 N → CD_cruise ≈ 0.056; CL_cruise ≈ 0.30
   → **CD0 = CD_cruise − k·CL² ≈ 0.051**. (If the model cruises fast,
   CD0 is too low — this is the knob.)
3. **Static RPM / initial takeoff acceleration** ⚑ → static thrust →
   fixes **k_m·√(Cprop·Sprop)** at Va = 0; together with anchor 2 at
   cruise, both prop constants are determined.
4. **Sea-level climb rate at Vy** ⚑ (say 900 fpm ≈ 4.6 m/s) → checks
   P·η against the drag polar; adjust η (≈ 0.7) to match.
5. **Full-aileron roll rate** ⚑ (say ~65 °/s):
   p_ss = (Clδa/|Clp|)·δa_max·(2V/b) — with defaults at 40 m/s this
   gives 66 °/s. Ratio Clδa/Clp sets the rate; Clp alone sets the
   crispness.
6. **Trim elevator vs speed** (qualitative: nose-down trim as speed
   rises) → sign/magnitude of Cm0, Cmα consistency.

The host golden model (below) turns each of these into a scripted test
that prints the observable — tuning is edit-param, rerun, compare,
without hardware in the loop.

## Ground: explicitly out of scope for v1

v1 is **air-start**: CAN INIT command places the aircraft trimmed at
(altitude, IAS, heading); a small trim solver (2-variable Newton on
δe, δt for the commanded speed at γ = 0) runs at init so the EKF sees
steady flight from sample one. Takeoff/landing HITL needs a gear model
— for a taildragger, an honest one (three contact points,
spring-damper, tire side-force, tailwheel steering, ground-loop
tendency) is its own subproject and half of its realism would be fake
anyway with only 4 captured channels (no brakes). Flagged as a v2
option with the simple 3-point spring-damper sketch; until then,
ArduPlane modes exercised are everything airborne: FBWA/FBWB, LOITER,
AUTO, RTL, TECS behavior, failsafes.

## CAN dictionary extension

Adds to the harness doc's dictionary (classic 8-byte frames):

| ID    | dir | payload |
|-------|-----|---------|
| 0x120 | →harness | FDM_MODE: 0 kinematic / 1 six-dof; flags (gusts on, freeze) |
| 0x121 | →harness | FDM_INIT: alt m u16, IAS 0.1 m/s u16, heading 0.01° u16 → trim & reset |
| 0x122 | →harness | WIND: N/E/D cm/s i16 steady + gust σ cm/s u8, L/V s u8 |
| 0x123 | →harness | PWM_CAL: ch u8, min/trim/max µs u16s, sign+defl (2 frames/ch or FD) |
| 0x12F | →harness | PARAM_SET: param index u16, float32 value — every table entry above |
| 0x220 | harness→ | TRUTH_POSVEL: N/E cm i32 (frame A), D cm i32 + vN/vE/vD cm/s i16 (frame B), 20 Hz |
| 0x221 | harness→ | TRUTH_ATT: quaternion 4×i16 (×2¹⁵), 20 Hz |
| 0x222 | harness→ | TRUTH_AIR: IAS/TAS 0.1 m/s u16, α/β 0.01° i16, 20 Hz |
| 0x223 | harness→ | TRUTH_CTRL: post-lag δa/δe/δr 0.01° i16, δt 0.1 % u16, 20 Hz |

PARAM_SET gets a readback (same ID, RTR or a dump command) so a tuning
session can diff live state against the defaults. The host **GPS /
airspeed feeder** consumes 0x220–0x222 and republishes as DroneCAN
Fix2 + RawAirData on the DUT's own CAN port at 5 Hz — and should delay
by a configurable 100–200 ms: GPS latency is a first-order input to
EKF tuning, and a zero-lag GPS would make HITL kinder than reality.
(As-built the primary feeder lives on the harness itself — see F4
below; the lag knob is GPS_CFG, default 150 ms.)

## Host golden model & validation

`fdm.c` is written freestanding (no lib includes in the math core) and
compiles for the host exactly like the cordic emul: same source, a thin
main() that feeds scripted control inputs and CSVs the state for
plotting. Validation ladder:

1. **Unit invariants** (host, CI-able): trim solver converges over the
   envelope; energy bookkeeping sane in glide (Ė ≈ −D·V); quaternion
   norm; stall blend monotonic and C¹.
2. **Linearization checks** (host): numerically linearize at cruise,
   verify eigenvalues land in sane GA ranges — short period ~2–4 rad/s
   damped, phugoid ~0.1–0.2 rad/s lightly damped, dutch roll ~1.5–3
   rad/s, roll subsidence τ ≈ 0.1 s, spiral slow. Catches
   sign/magnitude errors in derivatives better than any eyeballing.
   *As-built note:* the turncheck audit (fdm/turncheck.c) proved all
   turn-coordination signs correct, but the spiral mode over-converges
   (bank washes out faster than a real Kitfox) and dutch-roll phasing
   can read as "turning against bank" over short windows. Known
   deviation, not a bug — tune Clβ/Cnβ/Clr/Cnr via PARAM_SET in the F5
   tuning session.
3. **Calibration checklist** (host): the six observables above as
   scripted tests.
4. **SITL cross-check** (host, qualitative): ArduPilot's own SITL
   plane model given equivalent step inputs — responses should look
   like cousins, not twins.
5. **HITL acceptance** (bench): DUT flies FBWA hands-off; LOITER holds
   a circle in 5 m/s commanded wind; TECS tracks a climb; radio
   failsafe and EKF remain healthy through gust injection.

## Implementation plan

Continues the harness doc's numbering; independent of its M4–M7 except
where noted.

- **F0 — fdm core, host-first (~600).** State, airdata, aero, prop,
  integrator, trim solver; host build + CSV; validation ladder 1–3.
  No hardware required; can proceed in parallel with harness M4–M5.
- **F1 — target port (~150).** Drop fdm.c onto the G474 at 1 kHz;
  measure step time (measured 17.7 µs, gate ≤ 20 µs — see the budget
  note above); gate: `-Wdouble-promotion` clean
  and no libm/soft-double symbols in the map file; wire the
  sensor-truth struct into the existing scheduler behind the mode
  switch.
- **F2 — PWM path (~150).** Calibration table + CAN messages, ZOH,
  servo lag/rate limit, failsafe ageing. Closed loop against the
  DUT-mimic board generating scripted PWM.
- **F3 — CAN extension (~200).** FDM_MODE/INIT/WIND/PARAM_SET +
  TRUTH_* telemetry; host-side plotting from candump; PARAM_SET
  round-trip.
- **F4 — feeder (host-side, ~small).** TRUTH → DroneCAN Fix2 +
  RawAirData with configurable latency, onto the DUT's CAN port.
  *As-built:* the feeder moved **on board** — the harness itself
  broadcasts DroneCAN on FDCAN3 (PB3/PB4, node 42): Fix2 + RawAirData
  at 5 Hz with configurable lag (GPS_CFG, default 150 ms) +
  NodeStatus at 1 Hz; src/dronecan.{h,c} is a freestanding UAVCAN v0
  encoder pinned byte-exact against ArduPilot's libcanard by the
  dccheck build gate. The host variant survives as rb01tool -gps
  (same node 42 — run one feeder at a time; host variant has no
  NodeStatus). Origin math is integer 1e-8 deg — float32 can't hold
  a latitude.
- **F5 — HITL acceptance.** Validation ladder 5; tune coefficients via
  PARAM_SET against the checklist; freeze the Kitfox table as defaults.

## Risks & limits (stated, not hidden)

- **Coefficient provenance**: the ◆ values are typical-GA priors, not
  Kitfox data. The calibration checklist is the mitigation; expect one
  good tuning session, not zero.
- ~~**No ground model** in v1~~ — v1.1 grew a minimal-fidelity tricycle
  ground model (owner's spec: pretend tricycle gear, so no pitch change
  in the roll; accelerate to Vstall, rotate, climb at Vx; no flaps):
  ground-speed/yaw/pitch DOFs, rolling friction with lift unloading,
  nosewheel steering from rudder with speed washout, rotation about the
  mains against the CG-forward weight moment (Vr is emergent), liftoff
  when lift + thrust vertical carries the weight; gentle touchdowns
  roll out, harsh ones re-park the wreck level with a latched crash
  count. The prop is power-based per the owner's engine data (100 hp,
  eta 0.50 static to 0.85 at cruise, momentum-theory static cap).
  Air-start teleports turned out EKF-hostile — real takeoffs are the
  transition mechanism now (full story in doc/F5-TESTREPORT).
- **Stall is symmetric and gentle** by construction; spin/ground-loop
  class behavior is out of scope.
- **Prop effects** limited to thrust + torque reaction; no P-factor /
  slipstream → rudder usage on climb-out will be unrealistically low.
- **Float32 everywhere but position**: fine at these magnitudes;
  int32-cm position keeps long LOITER sessions drift-free.
- **4 channels only**: no flaps (by request), no brakes, no separate
  steering — constrains any future ground model more than the physics
  does.
