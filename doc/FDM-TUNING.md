# Tweaking the FDM

The model is `fdm/fdm.c`: freestanding float32, 1 kHz on the harness,
the same source in every host build (golden tests, ArduPilot SITL
backend, PX4 SITL bridge). Design rationale and the equations are in
fdm-DESIGN.md; this document is the how-to.

Two ways to change it, in order of preference:

1. **Parameters at runtime** — every entry of the table below is
   settable over pseudocan without reflashing, and reverts at the next
   harness reset. Tuning sessions live here.
2. **Defaults in code** — `fdm_defaults()` in fdm.c, checked by the host
   golden before it goes anywhere near the hardware. Once a runtime
   value has proven itself, it moves here.

Things that are not parameters (constants in code, host-testable the
same way): the stall blend shape, the ground model (rolling friction,
nosewheel washout, brakes below 15% power, crash-repark), the servo lag
(τ 60 ms, 300°/s rate limit, throttle τ 0.3 s — controls.c), and the
sensor noise levels (src/main.c, `noise()`: gyro 0.2°/s white + a
per-boot bias and a random walk, accel 5 mg, mag 20 nT, baro ±1 Pa
dither; the per-sensor enable mask is CMD_NOISE 0x42).

## The parameter table (PARAM_SET index → field)

| idx   | field           | default (fdm_defaults)     | what it pins                                      |
| ----- | --------------- | -------------------------- | ------------------------------------------------- |
| 0     | m               | 480 kg                     | flying mass                                       |
| 1-3   | S, b, cbar      | 12.2 m², 9.75 m, 1.25 m    | wing area, span, mean chord                       |
| 4-6   | Ixx, Iyy, Izz   | 700 / 650 / 1100 kg·m²     | diagonal inertia                                  |
| 7-8   | CL0, CLa        | 0.30, 5.0 /rad             | lift curve                                        |
| 9-10  | a_stall, blend  | 15°, 3° (rad in the table) | stall centre and half-width of the C¹ blend        |
| 11-12 | CD0, kind       | 0.050, 0.056               | drag polar CD = CD0 + k·CL²                        |
| 13-16 | Cm0, Cma, Cmq, Cmde | 0.02, −0.8, −12, −1.2  | pitch statics, damping, elevator power             |
| 17    | CYb             | −0.3                       | side force                                        |
| 18-22 | Clb, Clp, Clr, Clda, Cldr | −0.08, −0.42, 0.03, 0.17, 0.01 | roll: dihedral, damping, aileron/rudder |
| 23-27 | Cnb, Cnp, Cnr, Cnda, Cndr | 0.07, −0.03, −0.10, −0.01, −0.08 | yaw: weathercock, damping, adverse yaw, rudder |
| 28-30 | km, CpSp, kQ    | 582, 0.00772, 3e-4         | legacy quadratic prop anchors; torque reaction kQ  |
| 31-33 | wind_n[3]       | 0                          | steady wind NED, m/s (WIND 0x46 also sets these)   |
| 34-35 | gust_sigma, gust_tau | 0, 0                  | Gauss-Markov gusts, m/s and s (0 = off)            |
| 36-38 | mag_n[3]        | 19.97, 0, 44.01 µT         | earth field NED                                    |
| 39-40 | qnh_pa, t0_k    | 101325, 288.15             | ISA anchors (rebuilds the pressure table on write) |
| 41-42 | power_w, t_static_n | 74600 W, 1601 N        | engine rated power, static thrust cap              |
| 43    | crit_alt_m      | 0                          | turbo critical altitude, m; 0 = naturally aspirated |

Indices are the float positions in `struct FdmParams` (fdm/fdm.h) — the
header is the authority if this table and it ever disagree. Angles in
the table are radians; the deflection calibration (PWM_CAL) is
separate and in degrees.

Presets: `bench drive engine 912` = 41-43 → 74600 / 1601 / 0;
`engine 915` = 105000 / 2000 / 4572 (141 hp turbo rated to FL150).

## Setting one at runtime

```
bench drive param 11 0.055          # CD0
bench drive param 43 4572           # critical altitude
```

The harness answers every PARAM_SET with a PARAM_VAL readback on the
pseudocan link (rb01tool decodes it; `bench drive` is write-only and
tails the console instead). A readback of any index is a PARAM_SET with
bit 15 of the index set. Wire format: TMC 0x47, payload u16 index +
f32 value, big-endian, zero-padded to 8 bytes (src/canmsg.h;
`bench drive` and rb01tool both encode it).

Runtime values are RAM-only. A tuning session that ends in a harness
reset ends in the defaults; write the numbers down as you go.

## Changing the defaults: the golden-check workflow

```
edit fdm/fdm.c (fdm_defaults, or the model itself)
make -C fdm golden        # builds and runs; prints the observables, PASS/FAIL
make -C src               # the F1 gate: fdm.o must stay float32-only (no soft double)
make -C src flash
```

`fdm/golden.c` is the validation ladder from fdm-DESIGN.md as code. What
it prints, and the acceptance bands it enforces:

- ISA table vs the closed form; trim solver converges across the
  envelope (20-45 m/s, 0-4000 m); glide energy bookkeeping (Ė ≈ −D·V);
  quaternion norm and stall blend monotonic/C¹.
- Modes by numerical linearisation at cruise: short period 1.5-6 rad/s,
  phugoid 0.05-0.4 rad/s, dutch roll 1-5 rad/s, roll rate 40-95°/s with
  τ 0.03-0.3 s. A sign error in a derivative fails here before it ever
  flies.
- Observables: Vs (the POH anchor ~19.7 m/s), climb at Vy 2.5-8 m/s,
  full-throttle climb at 4500 m for both engine presets, trim elevator
  at 25 and 45 m/s.
- Takeoff: ground run, liftoff time, height and speed with the ground
  model (134 m, liftoff at ~26 m/s as built).

`make -C fdm turncheck` is the direction audit the eigenvalues cannot
do: aileron → bank → yaw rate → course must all agree, pulsed and held.
Run it after touching any lateral derivative.

## Which observable pins which coefficient

The calibration checklist, worked for the defaults (ρ 1.225, m 480 kg):

1. **Flaps-up stall speed** → CLmax = 2mg/(ρ·S·Vs²) — sets `a_stall`
   given `CLa` (44 mph ≈ 19.7 m/s → 1.62).
2. **Cruise speed at a known power** → CD at cruise → `CD0 = CD −
   kind·CL²`. If the model cruises fast, CD0 is too low; this is the
   knob. The as-built model cruises ~38 m/s at 45% throttle.
3. **Static thrust / initial acceleration and rated power** →
   `t_static_n`, `power_w` (thrust is power-based: P·η/Va with η from
   0.50 static to 0.85 at cruise, capped by the momentum-theory static
   limit).
4. **Sea-level climb at Vy** → checks P·η against the polar; the golden
   prints it.
5. **Full-aileron roll rate** → p_ss = (Clda/|Clp|)·δa_max·(2V/b):
   the ratio sets the rate, `Clp` alone the crispness (53°/s as built,
   which is why the autopilot needed ~9x ArduPilot's default roll FF).
6. **Trim vs speed** (nose-down trim as speed rises) → sign and size
   of `Cm0`, `Cma`. `Cmq` sets short-period damping (−12 is
   overdamped; raise toward −8 for a livelier response).
7. **Spiral and dutch roll** → `Clb`, `Cnb`, `Clr`, `Cnr`. Known
   as-built deviation: the spiral mode over-converges (bank washes out
   faster than the real aircraft) — cosmetic for the autopilot, tune
   these four if it matters to you.

For a different aircraft altogether: mass, geometry and inertia first
(Roskam radii of gyration are fine), then the six anchors above in that
order, running the golden after each. Expect one good tuning session,
not zero.

## Sensor-side fidelity knobs

- Noise and bias walk: constants in `src/main.c` (search `noise(`);
  the per-sensor enable mask is CMD_NOISE. Turning noise OFF is a
  trap: bit-identical pressure trips ArduPilot's stuck-baro detector,
  and a noise-free bench let a DCM-drift bug hide for nights.
- GPS lag: `drive gps 1 <ms>` (default 150 — a zero-lag GPS makes HITL
  kinder than reality).
- Sensor rates and ranges follow the DUT's own configuration writes
  (the register models honour ACC_CONF/GYRO_BANDWIDTH/ODR/TMRC), so
  the autopilot's driver settings apply exactly as on real silicon.
- Wind and gusts: WIND 0x46 / parameters 31-35; rb01tool `w`/`W`/`g`.
