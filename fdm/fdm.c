// fdm.c — see fdm.h and ../doc/fdm-DESIGN.md. Freestanding: cordic.h only.

#include "fdm.h"

#include "cordic.h"

#include <stddef.h>

#define G 9.80665f

// minimal tricycle ground model (hard surface); tuned constants, not params
#define GND_MU 0.03f         // rolling resistance
#define GND_WHEELBASE 1.60f  // nosewheel steering arm, m
#define GND_XMG 0.30f        // mains behind the CG, m (weight's anti-rotation moment)
#define GND_THETA_MAX 0.26f  // 15 deg rotation limit (tail strike)

// ---- ISA: cubic Hermite table over -100..5100 m, 16 knots ------------------
// Built at defaults/PARAM_SET time with the (slow) cordic expf/logf; the
// 1 kHz path only evaluates the polynomial. Shared shape with the baro layer.

enum { ISA_N = 16 };
static struct {
	float h0, dh;
	float p[ISA_N], dp[ISA_N];   // pressure and d(pressure)/dh at knots
	float r[ISA_N], dr[ISA_N];   // density
	float t0k, lapse;
} isa;

static void isa_build(float qnh, float t0) {
	isa.h0 = -100.0f;
	isa.dh = 5200.0f / (ISA_N - 1);
	isa.t0k = t0;
	isa.lapse = 0.0065f;
	const float Rg = 287.05f;
	const float ep = G / (Rg * isa.lapse);	// ~5.2559
	for (int i = 0; i < ISA_N; i++) {
		float h = isa.h0 + isa.dh * (float)i;
		float base = 1.0f - isa.lapse * h / t0;
		float t = t0 - isa.lapse * h;
		float p = qnh * expf(ep * logf(base));
		float rho = p / (Rg * t);
		isa.p[i] = p;
		isa.dp[i] = -rho * G;  // hydrostatic: dp/dh = -rho g (exact)
		isa.r[i] = rho;
		// drho/dh = rho * (-g/(R T) + lapse/T)
		isa.dr[i] = rho * (isa.lapse / t - G / (Rg * t));
	}
}

static void isa_eval(float h, float *p, float *rho, float *t_k) {
	float x = (h - isa.h0) / isa.dh;
	int i = (int)x;
	if (i < 0) i = 0;
	if (i > ISA_N - 2) i = ISA_N - 2;
	float t = x - (float)i;
	float h00 = (1.0f + 2.0f * t) * (1.0f - t) * (1.0f - t);
	float h10 = t * (1.0f - t) * (1.0f - t);
	float h01 = t * t * (3.0f - 2.0f * t);
	float h11 = t * t * (t - 1.0f);
	*p = h00 * isa.p[i] + h10 * isa.dh * isa.dp[i] + h01 * isa.p[i + 1] + h11 * isa.dh * isa.dp[i + 1];
	*rho = h00 * isa.r[i] + h10 * isa.dh * isa.dr[i] + h01 * isa.r[i + 1] + h11 * isa.dh * isa.dr[i + 1];
	*t_k = isa.t0k - isa.lapse * h;
}

int fdm_param_set(struct Fdm *f, unsigned idx, float v) {
	if (idx >= FDM_NPARAMS) {
		return -1;
	}
	((float *)&f->p)[idx] = v;
	unsigned isa0 = (unsigned)(offsetof(struct FdmParams, qnh_pa) / sizeof(float));
	if (idx == isa0 || idx == isa0 + 1) {
		isa_build(f->p.qnh_pa, f->p.t0_k);
	}
	return 0;
}

float fdm_param_get(const struct Fdm *f, unsigned idx) {
	return idx < FDM_NPARAMS ? ((const float *)&f->p)[idx] : 0.0f;
}

void fdm_isa(const struct Fdm *f, float h, float *p_pa, float *rho, float *t_k) {
	(void)f;
	isa_eval(h, p_pa, rho, t_k);
}

// ---- small vector/quaternion helpers ----------------------------------------

static inline void cross3(const float a[3], const float b[3], float o[3]) {
	o[0] = a[1] * b[2] - a[2] * b[1];
	o[1] = a[2] * b[0] - a[0] * b[2];
	o[2] = a[0] * b[1] - a[1] * b[0];
}

// R_nb (body -> NED) from q_nb = {w,x,y,z}
static void quat_to_r(const float q[4], float R[3][3]) {
	float w = q[0], x = q[1], y = q[2], z = q[3];
	R[0][0] = 1 - 2 * (y * y + z * z);
	R[0][1] = 2 * (x * y - w * z);
	R[0][2] = 2 * (x * z + w * y);
	R[1][0] = 2 * (x * y + w * z);
	R[1][1] = 1 - 2 * (x * x + z * z);
	R[1][2] = 2 * (y * z - w * x);
	R[2][0] = 2 * (x * z - w * y);
	R[2][1] = 2 * (y * z + w * x);
	R[2][2] = 1 - 2 * (x * x + y * y);
}

// ---- aero ---------------------------------------------------------------------

// C1 smoothstep 0..1 over the stall blend band
static float stall_sigma(const struct FdmParams *p, float aa) {
	float lo = p->a_stall - p->blend, hi = p->a_stall + p->blend;
	if (aa <= lo) return 0.0f;
	if (aa >= hi) return 1.0f;
	float t = (aa - lo) / (hi - lo);
	return t * t * (3.0f - 2.0f * t);
}

// ---- the step -------------------------------------------------------------------

void fdm_step(struct Fdm *f, const struct FdmControls *c, float dt) {
	const struct FdmParams *p = &f->p;
	float R[3][3];
	quat_to_r(f->quat, R);

	// gusts: per-axis first-order Gauss-Markov in NED (off when sigma 0)
	if (p->gust_sigma > 0.0f && p->gust_tau > 0.0f) {
		float a = 1.0f - dt / p->gust_tau; // (1-dt/tau) ~ exp for dt<<tau
		float q = p->gust_sigma * sqrtf(2.0f * dt / p->gust_tau);
		for (int i = 0; i < 3; i++) {
			f->rng ^= f->rng << 13; f->rng ^= f->rng >> 17; f->rng ^= f->rng << 5;
			float u = (float)(int32_t)f->rng * (1.0f / 2147483648.0f); // ~U(-1,1)
			f->gust[i] = a * f->gust[i] + q * 1.732f * u;              // var-matched
		}
	} else {
		f->gust[0] = f->gust[1] = f->gust[2] = 0.0f;
	}

	// airdata: v_air = v_b - R^T (wind + gust)
	float w_n[3] = {p->wind_n[0] + f->gust[0], p->wind_n[1] + f->gust[1], p->wind_n[2] + f->gust[2]};
	float w_b[3] = {R[0][0] * w_n[0] + R[1][0] * w_n[1] + R[2][0] * w_n[2],
	                R[0][1] * w_n[0] + R[1][1] * w_n[1] + R[2][1] * w_n[2],
	                R[0][2] * w_n[0] + R[1][2] * w_n[1] + R[2][2] * w_n[2]};
	float ua = f->v_b[0] - w_b[0], va_ = f->v_b[1] - w_b[1], wa = f->v_b[2] - w_b[2];
	float va = sqrtf(ua * ua + va_ * va_ + wa * wa);
	float h = -(float)f->pos_cm[2] * 0.01f;
	float ps, rho, tk;
	isa_eval(h, &ps, &rho, &tk);
	float qbar = 0.5f * rho * va * va;

	float alpha = 0.0f, beta = 0.0f, sa = 0.0f, ca = 1.0f;
	if (va > 1.0f) {
		alpha = atan2f(wa, ua);
		beta = atan2f(va_, sqrtf(ua * ua + wa * wa));
		sincosf(alpha, &sa, &ca);
	}

	// aero coefficients (stall-blended lift, pitch-down past stall)
	float aa = fabsf(alpha);
	float sg = stall_sigma(p, aa);
	float CLlin = p->CL0 + p->CLa * alpha;
	float sgn = (alpha >= 0.0f) ? 1.0f : -1.0f;
	float CLfp = 2.0f * sgn * sa * sa * ca;
	float CL = (1.0f - sg) * CLlin + sg * CLfp;
	float CD = p->CD0 + p->kind * CL * CL;
	float bh = (va > 1.0f) ? p->b / (2.0f * va) : 0.0f;   // b-hat
	float ch = (va > 1.0f) ? p->cbar / (2.0f * va) : 0.0f; // c-hat
	float P = f->w_b[0], Q = f->w_b[1], Rr = f->w_b[2];
	float CY = p->CYb * beta + p->Cndr * 0.0f + 0.15f * c->dr; // CYdr ~ 0.15
	float Cl = p->Clb * beta + p->Clp * P * bh + p->Clr * Rr * bh + p->Clda * c->da + p->Cldr * c->dr;
	float Cm = p->Cm0 + p->Cma * alpha + p->Cmq * Q * ch + p->Cmde * c->de - 0.25f * sg * sgn;
	float Cn = p->Cnb * beta + p->Cnp * P * bh + p->Cnr * Rr * bh + p->Cnda * c->da + p->Cndr * c->dr;

	// forces: L,D in wind axes -> body (beta small-angle), Y in body y
	float L = qbar * p->S * CL, D = qbar * p->S * CD;
	float Fax = -D * ca + L * sa;
	float Faz = -D * sa - L * ca;
	float Fay = qbar * p->S * CY;

	// prop: power-based thrust along +x (the reference aircraft's engine/prop: 100 hp,
	// eta 0.50 static rising to 0.85 by cruise), capped by the momentum-
	// theory static limit; torque reaction about x
	float eta = 0.50f + 0.35f * (va > 40.0f ? 1.0f : va * (1.0f / 40.0f));
	float vden = va > 1.0f ? va : 1.0f;
	// density lapse: naturally aspirated engines lapse from sea level
	// (crit_alt 0); a turbo holds rated power to crit_alt and lapses above
	float rho_crit, ps_, tk_;
	isa_eval(p->crit_alt_m, &ps_, &rho_crit, &tk_);
	float lapse = rho < rho_crit ? rho / rho_crit : 1.0f;
	float T = eta * c->dt * p->power_w * lapse / vden;
	float Tcap = p->t_static_n * lapse * c->dt; // static momentum limit, ~linear in power lever
	if (T > Tcap) T = Tcap;
	if (T < 0.0f) T = 0.0f; // no windmilling drag model
	float kmdt = p->km * c->dt;
	float Mx_q = -p->kQ * kmdt * kmdt;

	// moments
	float Mx = qbar * p->S * p->b * Cl + Mx_q;
	float My = qbar * p->S * p->cbar * Cm;
	float Mz = qbar * p->S * p->b * Cn;

	float fsp[3];
	if (f->on_ground) {
		// ---- minimal-fidelity tricycle ground roll ------------------------
		// DOFs: ground speed along heading, yaw (nosewheel = rudder), pitch
		// rotation about the mains in [0, GND_THETA_MAX]; wings level. Vr is
		// emergent: the elevator moment must beat the CG-ahead-of-mains
		// weight moment, which needs qbar. Liftoff when lift + thrust
		// vertical carries the weight.
		float psi = atan2f(2.0f * (f->quat[0] * f->quat[3] + f->quat[1] * f->quat[2]),
		                   1.0f - 2.0f * (f->quat[2] * f->quat[2] + f->quat[3] * f->quat[3]));
		float sth = -2.0f * (f->quat[1] * f->quat[3] - f->quat[0] * f->quat[2]);
		sth = sth > 1.0f ? 1.0f : sth < -1.0f ? -1.0f : sth;
		float th = atan2f(sth, sqrtf(1.0f - sth * sth)), cth = cosf(th); // asin via atan2: no libm here
		float vgn = R[0][0] * f->v_b[0] + R[0][1] * f->v_b[1] + R[0][2] * f->v_b[2];
		float vge = R[1][0] * f->v_b[0] + R[1][1] * f->v_b[1] + R[1][2] * f->v_b[2];
		float Vg = sqrtf(vgn * vgn + vge * vge);

		float W = p->m * G;
		float N = W - L * ca - T * sinf(th); // gear normal force
		if (N < 0.0f) N = 0.0f;
		float fric = GND_MU * N;
		if (c->dt < 0.15f) fric += 400.0f; // toe brakes held below taxi power
		float Vgdot = (T * cth - D - fric) / p->m;
		Vg += Vgdot * dt;
		if (Vg < 0.0f) { Vg = 0.0f; Vgdot = 0.0f; }
		// nosewheel steering from rudder, washed out with speed (a real
		// nosewheel's authority fades as the wheel unloads; also keeps the
		// DUT's ground-steering controller from spinning us like a top)
		float k_nw = 0.30f / (1.0f + Vg * Vg * (1.0f / 64.0f));
		float psidot = (Vg > 0.5f) ? Vg * c->dr * k_nw * (1.0f / GND_WHEELBASE) : 0.0f;
		psidot = psidot > 0.5f ? 0.5f : psidot < -0.5f ? -0.5f : psidot;
		psi += psidot * dt;

		float thdot = f->w_b[1];
		float thdd = (My - W * GND_XMG * cth) / p->Iyy; // rotate about the mains
		thdot += thdd * dt;
		th += thdot * dt;
		if (th <= 0.0f) { th = 0.0f; if (thdot < 0.0f) thdot = 0.0f; }
		if (th >= GND_THETA_MAX) { th = GND_THETA_MAX; if (thdot > 0.0f) thdot = 0.0f; }

		if (L * ca + T * sinf(th) >= W) f->on_ground = 0; // liftoff

		f->w_b[0] = 0.0f;
		f->w_b[1] = thdot;
		f->w_b[2] = psidot;
		float hy = 0.5f * psi, hp2 = 0.5f * th;
		float cy = cosf(hy), sy = sinf(hy), cp = cosf(hp2), sp = sinf(hp2);
		f->quat[0] = cy * cp;
		f->quat[1] = -sy * sp;
		f->quat[2] = cy * sp;
		f->quat[3] = sy * cp;
		quat_to_r(f->quat, R);
		float cpsi = cosf(psi), spsi = sinf(psi);
		float vng[3] = {Vg * cpsi, Vg * spsi, 0.0f};
		for (int i = 0; i < 3; i++) f->v_b[i] = R[0][i] * vng[0] + R[1][i] * vng[1];
		// imu truth: horizontal accel + gravity reaction, into body axes
		float an[3] = {Vgdot * cpsi - Vg * psidot * spsi, Vgdot * spsi + Vg * psidot * cpsi, -G};
		for (int i = 0; i < 3; i++) fsp[i] = R[0][i] * an[0] + R[1][i] * an[1] + R[2][i] * an[2];
		f->pos_cm[2] = 0;
		f->pos_rem[2] = 0.0f;
	} else {
	// semi-implicit Euler: rates first
	float wdot[3] = {(Mx - (f->w_b[1] * f->w_b[2]) * (p->Izz - p->Iyy)) / p->Ixx,
	                 (My - (f->w_b[0] * f->w_b[2]) * (p->Ixx - p->Izz)) / p->Iyy,
	                 (Mz - (f->w_b[0] * f->w_b[1]) * (p->Iyy - p->Ixx)) / p->Izz};
	for (int i = 0; i < 3; i++) f->w_b[i] += wdot[i] * dt;

	// then velocity, with the NEW rates; gravity via R^T (0,0,g)
	fsp[0] = (Fax + T) / p->m; // specific force
	fsp[1] = Fay / p->m;
	fsp[2] = Faz / p->m;
	float g_b[3] = {R[2][0] * G, R[2][1] * G, R[2][2] * G};
	float wxv[3];
	cross3(f->w_b, f->v_b, wxv);
	for (int i = 0; i < 3; i++) f->v_b[i] += (fsp[i] + g_b[i] - wxv[i]) * dt;

	// quaternion kinematics + renorm
	float qw = f->quat[0], qx = f->quat[1], qy = f->quat[2], qz = f->quat[3];
	float hp = 0.5f * f->w_b[0], hq = 0.5f * f->w_b[1], hr = 0.5f * f->w_b[2];
	f->quat[0] += (-hp * qx - hq * qy - hr * qz) * dt;
	f->quat[1] += (hp * qw + hr * qy - hq * qz) * dt;
	f->quat[2] += (hq * qw - hr * qx + hp * qz) * dt;
	f->quat[3] += (hr * qw + hq * qx - hp * qy) * dt;
	float n2 = f->quat[0] * f->quat[0] + f->quat[1] * f->quat[1] + f->quat[2] * f->quat[2] + f->quat[3] * f->quat[3];
	float inv = 1.0f / sqrtf(n2);
	for (int i = 0; i < 4; i++) f->quat[i] *= inv;
	}

	// position: NED velocity, int32 cm accumulation
	quat_to_r(f->quat, R);
	float v_n[3];
	for (int i = 0; i < 3; i++) v_n[i] = R[i][0] * f->v_b[0] + R[i][1] * f->v_b[1] + R[i][2] * f->v_b[2];
	for (int i = 0; i < 3; i++) {
		f->pos_rem[i] += v_n[i] * dt * 100.0f;
		int32_t mv = (int32_t)f->pos_rem[i];
		f->pos_cm[i] += mv;
		f->pos_rem[i] -= (float)mv;
	}

	// truth
	struct FdmTruth *t = &f->truth;
	for (int i = 0; i < 3; i++) {
		t->sforce[i] = fsp[i];
		t->rate[i] = f->w_b[i];
		t->mag[i] = R[0][i] * p->mag_n[0] + R[1][i] * p->mag_n[1] + R[2][i] * p->mag_n[2];
		t->pos_cm[i] = f->pos_cm[i];
		t->v_ned[i] = v_n[i];
	}
	for (int i = 0; i < 4; i++) t->quat[i] = f->quat[i];
	t->p_pa = ps;
	t->t_degc = tk - 273.15f;
	t->qbar = qbar;
	t->va = va;
	t->alpha = alpha;
	t->beta = beta;
	t->h = f->on_ground ? 0.0f : h;

	// gentle touchdown -> rollout; anything harsher leaves h < 0 for the
	// caller's crash handling (unchanged contract)
	if (!f->on_ground && t->h <= 0.0f && v_n[2] >= 0.0f) {
		float phi = atan2f(2.0f * (f->quat[0] * f->quat[1] + f->quat[2] * f->quat[3]),
		                   1.0f - 2.0f * (f->quat[1] * f->quat[1] + f->quat[2] * f->quat[2]));
		float sth2 = -2.0f * (f->quat[1] * f->quat[3] - f->quat[0] * f->quat[2]);
		if (v_n[2] < 2.5f && fabsf(phi) < 0.21f && sth2 > -0.05f && sth2 < 0.30f) {
			f->on_ground = 1;
			f->pos_cm[2] = 0;
			f->pos_rem[2] = 0.0f;
			f->w_b[0] = 0.0f;
			t->h = 0.0f;
		}
	}
}

// ---- defaults + trim -----------------------------------------------------------

void fdm_defaults(struct Fdm *f) {
	for (unsigned i = 0; i < sizeof *f / 4; i++) ((uint32_t *)f)[i] = 0;
	struct FdmParams *p = &f->p;
	p->m = 480.0f;
	p->S = 12.2f; p->b = 9.75f; p->cbar = 1.25f;
	p->Ixx = 700.0f; p->Iyy = 650.0f; p->Izz = 1100.0f;
	p->CL0 = 0.30f; p->CLa = 5.0f;
	p->a_stall = 0.262f; p->blend = 0.052f; // 15 deg +- 3
	p->CD0 = 0.050f; p->kind = 0.056f;
	p->Cm0 = 0.02f; p->Cma = -0.8f; p->Cmq = -12.0f; p->Cmde = -1.2f;
	p->CYb = -0.3f;
	p->Clb = -0.08f; p->Clp = -0.42f; p->Clr = 0.03f; p->Clda = 0.17f; p->Cldr = 0.01f;
	p->Cnb = 0.07f; p->Cnp = -0.03f; p->Cnr = -0.10f; p->Cnda = -0.01f; p->Cndr = -0.08f;
	// prop anchors: static thrust 1600 N, 890 N at 46 m/s and 75% throttle
	p->km = 582.0f; p->CpSp = 0.00772f; p->kQ = 3.0e-4f;
	// engine: the reference aircraft's Rotax 912iS (100 hp, naturally aspirated)
	p->power_w = 74600.0f; p->t_static_n = 1601.0f; p->crit_alt_m = 0.0f;
	p->mag_n[0] = 19.97f; p->mag_n[2] = 44.01f; // the bench-cal field
	p->qnh_pa = 101325.0f; p->t0_k = 288.15f;
	f->quat[0] = 1.0f;
	f->on_ground = 1; // parked on the gear; fdm_trim (air-start) clears it
	f->rng = 0x2545F491u;
	isa_build(p->qnh_pa, p->t0_k);
}

int fdm_trim(struct Fdm *f, float alt_m, float ias, float heading, struct FdmControls *out) {
	struct FdmParams *p = &f->p;
	isa_build(p->qnh_pa, p->t0_k);
	float ps, rho, tk;
	isa_eval(alt_m, &ps, &rho, &tk);
	(void)ps; (void)tk;
	// ias is INDICATED: qbar = .5 rho0 IAS^2; the true airspeed scales up
	float qbar = 0.5f * 1.225f * ias * ias;
	float tas = ias * sqrtf(1.225f / rho);

	// sequential level trim: CL -> alpha -> de; CD -> T -> dt
	float CL = p->m * G / (qbar * p->S);
	float alpha = (CL - p->CL0) / p->CLa;
	if (alpha > p->a_stall - p->blend || alpha < -0.1f) return -1;
	float de = -(p->Cm0 + p->Cma * alpha) / p->Cmde;
	float CD = p->CD0 + p->kind * CL * CL;
	float T = qbar * p->S * CD;
	// invert the power-based prop model (whichever branch binds)
	float eta = 0.50f + 0.35f * (tas > 40.0f ? 1.0f : tas * (1.0f / 40.0f));
	float vden = tas > 1.0f ? tas : 1.0f;
	float dt_p = T * vden / (eta * 74600.0f);
	float dt_c = T * (1.0f / 1601.0f);
	float dt_ = dt_p > dt_c ? dt_p : dt_c;
	if (dt_ > 1.0f) return -1;

	// state: level flight, gamma = 0, theta = alpha, given heading;
	// q_nb = qz(psi) * qy(theta): w=ch*ct, x=-sh*st, y=ch*st, z=sh*ct
	float sa, ca, sh, ch, st, ct;
	sincosf(alpha, &sa, &ca);
	f->v_b[0] = tas * ca; f->v_b[1] = 0.0f; f->v_b[2] = tas * sa;
	f->w_b[0] = f->w_b[1] = f->w_b[2] = 0.0f;
	sincosf(0.5f * heading, &sh, &ch);
	sincosf(0.5f * alpha, &st, &ct);
	f->quat[0] = ch * ct;
	f->quat[1] = -sh * st;
	f->quat[2] = ch * st;
	f->quat[3] = sh * ct;
	f->pos_cm[0] = f->pos_cm[1] = 0;
	f->pos_cm[2] = (int32_t)(-alt_m * 100.0f);
	f->pos_rem[0] = f->pos_rem[1] = f->pos_rem[2] = 0.0f;
	f->gust[0] = f->gust[1] = f->gust[2] = 0.0f;
	f->on_ground = 0; // air-start

	if (out != 0) {
		out->da = 0.0f; out->de = de; out->dr = 0.0f; out->dt = dt_;
	}
	struct FdmControls c = {0.0f, de, 0.0f, dt_};
	fdm_step(f, &c, 1e-3f); // prime truth
	return 0;
}
