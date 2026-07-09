// physics.c — commanded state -> sensor truth, see physics.h.
//
// Trig is plain libm (sinf/cosf/asinf/atanf/powf): the whole 2 kHz step is
// ~1% of the core; the CORDIC unit remains an option if that ever matters.

#include "physics.h"

#include <math.h>
#include <stdbool.h>

#define TAU 0.4f    // command lag (s)
#define ALPHA0 0.035f // fixed trim AoA (rad, ~2 deg)
#define VMIN 3.0f   // below this commanded V: bench mode
#define LAPSE 0.0065f
#define ISA_EXP 5.2553f
#define SIN_MAX 0.9f // asin domain guard

static struct {
	float v_t, hdot_t, psidot_t; // lag targets (commanded)
	float v, hdot, psidot;       // lag-filtered state
	float psi, h;                // integrated
	float qnh, t0, b, incl, decl;
	float phi_p, theta_p, vel_p[3]; // previous, for numerical derivatives
	bool primed;                    // previous values valid
	uint16_t flags;
} st;

static struct PhysicsTruth truth;

void physics_init(void) {
	st = (typeof(st)){0};
	// mid-latitude defaults, matching the bench-cal field (20 N + 44 D uT)
	st.qnh = 101325.0f;
	st.t0 = 288.15f;
	st.b = 48.33f;
	st.incl = 1.1449f; // 65.6 deg
	physics_step(1e-3f);
}

void physics_cmd(float v_ms, float hdot_ms, float psidot_rads) {
	st.v_t = v_ms;
	st.hdot_t = hdot_ms;
	st.psidot_t = psidot_rads;
}

void physics_env(float qnh_pa, float t0_k, float b_ut, float incl_rad) {
	st.qnh = qnh_pa;
	st.t0 = t0_k;
	st.b = b_ut;
	st.incl = incl_rad;
}

const struct PhysicsTruth *physics_truth(void) { return &truth; }
uint16_t physics_flags(void) { return st.flags; }

static inline float lag(float x, float target, float a) { return x + (target - x) * a; }

void physics_step(float dt) {
	uint16_t flags = 0;

	// first-order lags; a = dt/tau, stable for any dt by clamping
	float a = dt / TAU;
	if (a > 1.0f) {
		a = 1.0f;
	}
	st.v = lag(st.v, st.v_t, a);
	st.hdot = lag(st.hdot, st.hdot_t, a);
	st.psidot = lag(st.psidot, st.psidot_t, a);

	st.psi += st.psidot * dt;
	if (st.psi > (float)M_PI) {
		st.psi -= 2.0f * (float)M_PI;
	}
	if (st.psi < -(float)M_PI) {
		st.psi += 2.0f * (float)M_PI;
	}
	st.h += st.hdot * dt;

	// attitude from coordinated flight
	float gamma = 0, phi = 0, theta = 0;
	if (st.v < VMIN) {
		flags |= PHYSICS_FLAG_BENCH; // ground/bench: level, still
	} else {
		float sg = st.hdot / st.v;
		if (sg > SIN_MAX) {
			sg = SIN_MAX;
			flags |= PHYSICS_FLAG_CLAMP;
		}
		if (sg < -SIN_MAX) {
			sg = -SIN_MAX;
			flags |= PHYSICS_FLAG_CLAMP;
		}
		gamma = asinf(sg);
		phi = atanf(st.psidot * st.v * cosf(gamma) / PHYSICS_G);
		theta = gamma + ALPHA0;
	}

	float cps = cosf(st.psi), sps = sinf(st.psi);
	float cth = cosf(theta), sth = sinf(theta);
	float cph = cosf(phi), sph = sinf(phi);

	// NED velocity from the filtered state (bench mode: exactly zero)
	float cg = cosf(gamma);
	float vel[3] = {st.v * cg * cps, st.v * cg * sps, -st.hdot};
	if (flags & PHYSICS_FLAG_BENCH) {
		vel[0] = vel[1] = vel[2] = 0;
	}

	// numerical derivatives of the (smooth, lag-filtered) trajectory
	float phid = 0, thetad = 0, a_n[3] = {0, 0, 0};
	if (st.primed) {
		phid = (phi - st.phi_p) / dt;
		thetad = (theta - st.theta_p) / dt;
		for (int i = 0; i < 3; i++) {
			a_n[i] = (vel[i] - st.vel_p[i]) / dt;
		}
	}
	st.phi_p = phi;
	st.theta_p = theta;
	for (int i = 0; i < 3; i++) {
		st.vel_p[i] = vel[i];
	}
	st.primed = true;

	// body rates: the strapdown transform of (phid, thetad, psid)
	truth.rate[0] = phid - st.psidot * sth;
	truth.rate[1] = thetad * cph + st.psidot * cth * sph;
	truth.rate[2] = -thetad * sph + st.psidot * cth * cph;

	// specific force: f_n = a_n - g_n, rotated into the body (R = ZYX Euler,
	// body->NED; f_b = R^T f_n)
	float f_n[3] = {a_n[0], a_n[1], a_n[2] - PHYSICS_G};
	float R[3][3] = {
		{cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps},
		{cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps},
		{-sth, sph * cth, cph * cth},
	};
	for (int i = 0; i < 3; i++) {
		truth.sforce[i] = R[0][i] * f_n[0] + R[1][i] * f_n[1] + R[2][i] * f_n[2];
	}

	// magnetometer: earth field (B, inclination, declination) into the body
	float ci = cosf(st.incl), si = sinf(st.incl);
	float m_n[3] = {st.b * ci * cosf(st.decl), st.b * ci * sinf(st.decl), st.b * si};
	for (int i = 0; i < 3; i++) {
		truth.mag[i] = R[0][i] * m_n[0] + R[1][i] * m_n[1] + R[2][i] * m_n[2];
	}

	// ISA baro from QNH + integrated altitude
	float base = 1.0f - LAPSE * st.h / st.t0;
	if (base < 0.1f) {
		base = 0.1f;
		flags |= PHYSICS_FLAG_CLAMP;
	}
	truth.p_pa = st.qnh * powf(base, ISA_EXP);
	truth.t_degc = st.t0 - LAPSE * st.h - 273.15f;

	truth.psi = st.psi;
	truth.h = st.h;
	st.flags = flags;
}
