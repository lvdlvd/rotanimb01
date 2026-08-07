#pragma once

// fdm — 6-DOF fixed-wing flight dynamics (Kitfox V), fdm-DESIGN.md mode 1.
// Freestanding float32 core: FPU arithmetic + the cordic float API only —
// no libm, no doubles (position accumulates in int32 cm). Runs at 1 kHz on
// the harness (F1) and compiles host-side against the cordic emulation for
// the golden tests (F0).

#include <stdint.h>

// The parameter table. CAN PARAM_SET addresses these by float index (the
// config.h idiom): treat as float[FDM_NPARAMS]. Every entry is settable at
// runtime; fdm_defaults() loads the Kitfox V table from fdm-DESIGN.md.
struct FdmParams {
	float m;                    // 0  flying mass, kg
	float S, b, cbar;           // 1..3 wing area m2, span m, mean chord m
	float Ixx, Iyy, Izz;        // 4..6 kg m2 (diagonal inertia)
	float CL0, CLa;             // 7..8 lift: CL = CL0 + CLa*alpha
	float a_stall, blend;       // 9..10 stall center and half-width, rad
	float CD0, kind;            // 11..12 drag polar CD = CD0 + k*CL^2
	float Cm0, Cma, Cmq, Cmde;  // 13..16 pitch
	float CYb;                  // 17 side force
	float Clb, Clp, Clr, Clda, Cldr;  // 18..22 roll
	float Cnb, Cnp, Cnr, Cnda, Cndr;  // 23..27 yaw
	float km, CpSp, kQ;         // 28..30 prop: T = .5 rho CpSp ((km dt)^2 - Va^2)
	float wind_n[3];            // 31..33 steady wind NED, m/s
	float gust_sigma, gust_tau; // 34..35 per-axis Gauss-Markov, m/s and s (0 = off)
	float mag_n[3];             // 36..38 earth field NED, uT
	float qnh_pa, t0_k;         // 39..40 ISA anchors
	float power_w, t_static_n;  // 41..42 engine: rated power W, static thrust cap N
	float crit_alt_m;           // 43 turbo critical altitude, m: full power held to
	                            //    here, lapsing with rho above; 0 = naturally
	                            //    aspirated (lapses from sea level).
	                            //    912iS: 74600, 1601, 0
	                            //    915iS: 105000, 2000, 4572 (141 hp, FL150)
};
#define FDM_NPARAMS (sizeof(struct FdmParams) / sizeof(float))

struct FdmControls {
	float da, de, dr; // aileron, elevator, rudder, rad (post-lag; F2 owns lag)
	float dt;         // throttle 0..1
};

// truth snapshot for the sensor layer and TRUTH_* telemetry
struct FdmTruth {
	float sforce[3]; // specific force body FRD, m/s2 (gravity NOT included)
	float rate[3];   // body rates p q r, rad/s
	float mag[3];    // field in body, uT
	float p_pa, t_degc; // static pressure and OAT at altitude
	float qbar;      // dynamic pressure, Pa (airspeed truth)
	float va, alpha, beta;
	float quat[4];   // q_nb, w x y z
	int32_t pos_cm[3]; // NED from origin
	float v_ned[3];
	float h;         // altitude above origin, m (= -pos_d)
};

struct Fdm {
	struct FdmParams p;
	// state
	float v_b[3];   // u v w
	float w_b[3];   // p q r
	float quat[4];  // q_nb
	int32_t pos_cm[3];
	float pos_rem[3]; // sub-cm carry
	float gust[3];    // Gauss-Markov state, NED m/s
	uint32_t rng;     // xorshift for gusts
	uint8_t on_ground; // tricycle ground-roll mode (minimal fidelity)
	// derived per step
	struct FdmTruth truth;
};

// load the default parameter table (Kitfox V) and a benign ground state
void fdm_defaults(struct Fdm *f);

// air-start: place trimmed at altitude (m), IAS (m/s), heading (rad);
// solves alpha/de/dt for level flight, returning the trim controls in *out
// (may be NULL). Returns 0 ok, -1 out of envelope.
int fdm_trim(struct Fdm *f, float alt_m, float ias, float heading, struct FdmControls *out);

// one integration step; controls are post-lag deflections. dt seconds
// (1e-3f nominal). Updates f->truth.
void fdm_step(struct Fdm *f, const struct FdmControls *c, float dt);

// ISA from the shared Hermite table: pressure Pa and density kg/m3 at h
// metres (valid -100..5100 m). Rebuilt by fdm_defaults / PARAM_SET of qnh.
void fdm_isa(const struct Fdm *f, float h, float *p_pa, float *rho, float *t_k);

// the PARAM_SET surface: the table as float[FDM_NPARAMS] by index, with the
// side effects (ISA rebuild on the qnh/t0 anchors) applied on write.
// set returns 0, or -1 for an out-of-range index; get returns 0.0f there.
int fdm_param_set(struct Fdm *f, unsigned idx, float v);
float fdm_param_get(const struct Fdm *f, unsigned idx);
