// golden.c — host validation for the fdm core (fdm-DESIGN.md ladder 1..3):
// invariants, dynamic modes by simulation probing, calibration observables.
// Links the cordic host emulation — the same math the target runs.

#include "fdm.h"

#include "cordic.h" // fabsf/sqrtf/sincosf via the same emulated backend
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...)                     \
	do {                                     \
		if (!(cond)) {                       \
			failures++;                      \
			printf("  FAIL %s: ", __func__); \
			printf(__VA_ARGS__);             \
			printf("\n");                    \
		}                                    \
	} while (0)

#define DT 1e-3f

static struct Fdm f;

static void run(const struct FdmControls *c, float sec) {
	for (int i = 0; i < (int)(sec / DT); i++) fdm_step(&f, c, DT);
}

// ---- ladder 1: invariants ----------------------------------------------------

static void test_isa(void) {
	fdm_defaults(&f);
	float p, r, t;
	fdm_isa(&f, 0.0f, &p, &r, &t);
	CHECK(p > 101324.0f && p < 101326.0f, "p0 %g", (double)p);
	CHECK(r > 1.224f && r < 1.226f, "rho0 %g", (double)r);
	fdm_isa(&f, 2000.0f, &p, &r, &t);
	CHECK(p > 79000.0f && p < 80500.0f, "p2000 %g", (double)p); // ISA 79495
	CHECK(t > 274.0f && t < 276.0f, "t2000 %g", (double)t);
}

static void test_trim_envelope(void) {
	for (float alt = 0.0f; alt <= 3000.0f; alt += 1500.0f) {
		for (float ias = 24.0f; ias <= 45.0f; ias += 3.0f) {
			fdm_defaults(&f);
			struct FdmControls c;
			int r = fdm_trim(&f, alt, ias, 0.5f, &c);
			CHECK(r == 0, "no trim at alt %g ias %g", (double)alt, (double)ias);
			if (r != 0) continue;
			CHECK(c.dt > 0.02f && c.dt < 0.95f, "dt %g at ias %g", (double)c.dt, (double)ias);
			// hold trim controls 8 s: must stay near the trim point
			run(&c, 8.0f);
			struct FdmTruth *t = &f.truth;
			float tas = ias * sqrtf(1.225f / (2.0f * t->qbar / (t->va * t->va) + 1e-6f));
			(void)tas; // va is TAS; allow generous drift band around it
			CHECK(fabsf(t->va - ias) < 0.20f * ias + 3.0f, "va drift %g -> %g", (double)ias, (double)t->va);
			CHECK(fabsf(t->rate[1]) < 0.06f, "q %g at ias %g", (double)t->rate[1], (double)ias);
			CHECK(fabsf((float)t->pos_cm[2] * 0.01f + alt) < 0.12f * ias + 20.0f,
			      "alt drift %g cm", (double)t->pos_cm[2]);
		}
	}
	printf("trim envelope ok\n");
}

static void test_glide_energy(void) {
	fdm_defaults(&f);
	struct FdmControls c;
	CHECK(fdm_trim(&f, 1000.0f, 32.0f, 0.0f, &c) == 0, "trim");
	c.dt = 0.0f; // engine cut; re-trim pitch is not needed for the bookkeeping
	run(&c, 2.0f); // let the phugoid transient develop but average over it:
	// E = .5 m V^2 + m g h ; compare dE over 10 s with integral of -D*Va
	float E0 = 0.5f * f.p.m * f.truth.va * f.truth.va + f.p.m * 9.80665f * f.truth.h;
	float dwork = 0.0f;
	for (int i = 0; i < 10000; i++) {
		fdm_step(&f, &c, DT);
		// drag from the polar at current state (recomputed as the model does)
		float CL = f.p.m * 9.80665f / (f.truth.qbar * f.p.S + 1.0f); // approx: L ~ W
		float CD = f.p.CD0 + f.p.kind * CL * CL;
		dwork += CD * f.truth.qbar * f.p.S * f.truth.va * DT;
	}
	float E1 = 0.5f * f.p.m * f.truth.va * f.truth.va + f.p.m * 9.80665f * f.truth.h;
	float lost = E0 - E1;
	CHECK(lost > 0.0f, "energy grew in glide");
	CHECK(fabsf(lost - dwork) < 0.25f * dwork, "dE %g vs D.V %g", (double)lost, (double)dwork);
	printf("glide energy ok (dE %.0f J vs %.0f J)\n", (double)lost, (double)dwork);
}

static void test_quat_norm_and_stall(void) {
	fdm_defaults(&f);
	struct FdmControls c;
	fdm_trim(&f, 500.0f, 35.0f, 1.0f, &c);
	run(&c, 60.0f);
	float n2 = 0;
	for (int i = 0; i < 4; i++) n2 += f.quat[i] * f.quat[i];
	CHECK(fabsf(n2 - 1.0f) < 1e-3f, "quat norm2 %g", (double)n2);

	// stall blend: CL(alpha) finite, C1-ish, rises to the blend then droops
	float prev = -10.0f, maxcl = 0.0f;
	int mono_ok = 1;
	for (float a = -0.5f; a <= 0.5f; a += 0.005f) {
		float sa_, ca_;
		sincosf(a, &sa_, &ca_);
		float lo = f.p.a_stall - f.p.blend, hi = f.p.a_stall + f.p.blend;
		float t = (fabsf(a) <= lo) ? 0.0f : (fabsf(a) >= hi) ? 1.0f : 0.0f;
		(void)t;
		// evaluate through the model: one step at forced alpha via v_b
		f.v_b[0] = 30.0f * ca_; f.v_b[1] = 0; f.v_b[2] = 30.0f * sa_;
		f.w_b[0] = f.w_b[1] = f.w_b[2] = 0;
		struct FdmControls cc = {0, 0, 0, 0};
		fdm_step(&f, &cc, DT);
		float CL = 0.0f;
		if (f.truth.qbar > 1.0f) {
			// reconstruct CL from specific force in wind axes
			float L = (-f.truth.sforce[2] * ca_ + f.truth.sforce[0] * sa_) * f.p.m;
			CL = L / (f.truth.qbar * f.p.S);
		}
		CHECK(isfinite(CL), "CL not finite at a %g", (double)a);
		if (a < f.p.a_stall - f.p.blend && a > -(f.p.a_stall - f.p.blend)) {
			if (CL < prev - 0.02f) mono_ok = 0;
		}
		if (CL > maxcl) maxcl = CL;
		prev = CL;
	}
	CHECK(mono_ok, "CL not monotone in the linear band");
	CHECK(maxcl > 1.3f && maxcl < 2.0f, "CLmax %g", (double)maxcl);
	printf("quat + stall ok (CLmax %.2f)\n", (double)maxcl);
}

// ---- ladder 2: dynamic modes by probing ---------------------------------------

// measure oscillation period of signal zero-crossings (same direction)
struct probe { float prev; float first_t, last_t; int crossings; };
static void probe_init(struct probe *pr) { memset(pr, 0, sizeof *pr); pr->first_t = -1.0f; }
static void probe_feed(struct probe *pr, float t, float v) {
	if (pr->prev < 0.0f && v >= 0.0f) {
		if (pr->first_t < 0.0f) pr->first_t = t;
		else { pr->last_t = t; pr->crossings++; }
	}
	pr->prev = v;
}
static float probe_omega(const struct probe *pr) {
	if (pr->crossings < 1 || pr->last_t <= pr->first_t) return 0.0f;
	return 2.0f * 3.14159265f * (float)pr->crossings / (pr->last_t - pr->first_t);
}

static void test_modes(void) {
	struct FdmControls c;

	// short period: elevator pulse, watch q
	fdm_defaults(&f);
	fdm_trim(&f, 1000.0f, 35.0f, 0.0f, &c);
	struct FdmControls cp = c;
	cp.de += 0.1f;
	run(&cp, 0.3f);
	struct probe pr;
	probe_init(&pr);
	float peak = 0.0f;
	for (int i = 0; i < 6000; i++) {
		fdm_step(&f, &c, DT);
		probe_feed(&pr, (float)i * DT, f.truth.rate[1]);
		float aq = fabsf(f.truth.rate[1]);
		if (i < 1000 && aq > peak) peak = aq;
	}
	float wsp = probe_omega(&pr);
	CHECK(fabsf(f.truth.rate[1]) < 0.15f * peak + 1e-3f, "short period undamped");
	printf("short period: omega %.2f rad/s (accept 1.5..6)\n", (double)wsp);
	CHECK(wsp == 0.0f || (wsp > 1.5f && wsp < 6.0f), "wsp %g", (double)wsp);

	// phugoid: speed kick, watch va about the TRIM speed (TAS, not IAS)
	fdm_defaults(&f);
	fdm_trim(&f, 1000.0f, 35.0f, 0.0f, &c);
	float va0 = f.truth.va;
	f.v_b[0] += 3.0f;
	probe_init(&pr);
	for (int i = 0; i < 90000; i++) {
		fdm_step(&f, &c, DT);
		probe_feed(&pr, (float)i * DT, f.truth.va - va0);
	}
	float wph = probe_omega(&pr);
	printf("phugoid: omega %.3f rad/s (accept 0.05..0.4)\n", (double)wph);
	CHECK(wph > 0.05f && wph < 0.4f, "wph %g", (double)wph);

	// dutch roll: rudder doublet, watch beta
	fdm_defaults(&f);
	fdm_trim(&f, 1000.0f, 35.0f, 0.0f, &c);
	cp = c; cp.dr = 0.15f; run(&cp, 0.4f);
	cp.dr = -0.15f; run(&cp, 0.4f);
	probe_init(&pr);
	for (int i = 0; i < 12000; i++) {
		fdm_step(&f, &c, DT);
		probe_feed(&pr, (float)i * DT, f.truth.beta);
	}
	float wdr = probe_omega(&pr);
	printf("dutch roll: omega %.2f rad/s (accept 1..5)\n", (double)wdr);
	CHECK(wdr > 1.0f && wdr < 5.0f, "wdr %g", (double)wdr);

	// roll subsidence: aileron step, p reaches steady with tau ~ 0.1 s
	fdm_defaults(&f);
	fdm_trim(&f, 1000.0f, 40.0f, 0.0f, &c);
	cp = c; cp.da = 0.35f; // full deflection 20 deg
	float p_ss = 0.0f;
	for (int i = 0; i < 1500; i++) {
		fdm_step(&f, &cp, DT);
		if (i == 1499) p_ss = f.truth.rate[0];
	}
	// time constant: find 63% crossing
	fdm_defaults(&f);
	fdm_trim(&f, 1000.0f, 40.0f, 0.0f, &c);
	cp = c; cp.da = 0.35f;
	float tau = 0.0f;
	for (int i = 0; i < 1500; i++) {
		fdm_step(&f, &cp, DT);
		if (tau == 0.0f && fabsf(f.truth.rate[0]) > 0.63f * fabsf(p_ss)) tau = (float)i * DT;
	}
	float pdeg = p_ss * 57.2958f;
	printf("roll: p_ss %.0f deg/s (accept 40..95), tau %.3f s (accept .03...3)\n", (double)pdeg, (double)tau);
	CHECK(fabsf(pdeg) > 40.0f && fabsf(pdeg) < 95.0f, "p_ss %g", (double)pdeg);
	CHECK(tau > 0.03f && tau < 0.3f, "tau %g", (double)tau);
}

// ---- ladder 3: calibration observables ------------------------------------------

static void test_observables(void) {
	fdm_defaults(&f);
	// stall speed from CLmax anchor: Vs = sqrt(2mg/(rho S CLmax))
	float CLmax = f.p.CL0 + f.p.CLa * f.p.a_stall; // ~1.61 at blend center
	float vs = sqrtf(2.0f * f.p.m * 9.80665f / (1.225f * f.p.S * CLmax));
	printf("Vs %.1f m/s (poh anchor ~19.7)\n", (double)vs);
	CHECK(vs > 17.0f && vs < 23.0f, "Vs %g", (double)vs);

	// climb at Vy, full throttle, from trim: steady climb rate after 15 s
	struct FdmControls c;
	fdm_defaults(&f);
	CHECK(fdm_trim(&f, 100.0f, 33.0f, 0.0f, &c) == 0, "trim vy");
	c.dt = 1.0f;
	run(&c, 15.0f);
	float roc = -f.truth.v_ned[2];
	printf("climb at Vy: %.1f m/s (accept 2.5..8)\n", (double)roc);
	CHECK(roc > 2.5f && roc < 8.0f, "roc %g", (double)roc);

	// trim elevator vs speed: more nose-down (de decreasing) as speed rises
	struct FdmControls c25, c45;
	fdm_defaults(&f);
	fdm_trim(&f, 500.0f, 25.0f, 0.0f, &c25);
	fdm_trim(&f, 500.0f, 45.0f, 0.0f, &c45);
	printf("trim de: %.3f rad at 25 m/s, %.3f rad at 45 m/s\n", (double)c25.de, (double)c45.de);
	// positive de = trailing edge down = nose-down moment (Cmde < 0):
	// nose-down trim demand must GROW with speed
	CHECK(c45.de > c25.de, "trim de not nose-down with speed");
}

// tricycle ground model: full-throttle takeoff from rest — rotate once the
// elevator has authority, liftoff, then climb. Vr/liftoff are emergent; the
// POH says ~90 m ground roll, accept a generous envelope.
static void test_takeoff(void) {
	struct FdmControls c = {0};
	fdm_defaults(&f);
	CHECK(f.on_ground == 1, "starts on gear");
	// parked at idle: stays put, serves rest truth (idle thrust exceeds
	// rolling friction — the toe brakes below taxi power hold it)
	c.dt = 0.10f;
	run(&c, 4.0f);
	CHECK(f.truth.va < 0.1f, "parked va %g", (double)f.truth.va);
	CHECK(fabsf(f.truth.sforce[2] + 9.80665f) < 0.05f, "parked -1g %g", (double)f.truth.sforce[2]);
	// full throttle, stick neutral until 18 m/s, then rotate
	c.dt = 1.0f;
	float t = 0.0f, run_m = 0.0f, t_liftoff = -1.0f;
	while (t < 30.0f) {
		c.de = (f.truth.va >= 18.0f) ? -0.30f : 0.0f;
		fdm_step(&f, &c, 1e-3f);
		t += 1e-3f;
		if (f.on_ground) {
			run_m = sqrtf((float)f.pos_cm[0] * (float)f.pos_cm[0] +
			              (float)f.pos_cm[1] * (float)f.pos_cm[1]) * 0.01f;
		} else if (t_liftoff < 0.0f) {
			t_liftoff = t;
		}
		if (f.truth.h > 15.0f) break;
	}
	printf("takeoff: ground run %.0f m, liftoff t=%.1f s, h %.1f m, va %.1f m/s\n",
	       (double)run_m, (double)t_liftoff, (double)f.truth.h, (double)f.truth.va);
	CHECK(t_liftoff > 0.0f, "lifted off");
	CHECK(run_m > 30.0f && run_m < 350.0f, "ground run %g", (double)run_m);
	CHECK(f.truth.h > 15.0f, "climbing, h %g", (double)f.truth.h);
	// keep climbing hands-off for a bit: no immediate stall/nose-over
	run(&c, 5.0f);
	CHECK(f.truth.h > 20.0f && f.truth.va > 15.0f, "climb-out h %g va %g",
	      (double)f.truth.h, (double)f.truth.va);
}

int main(void) {
	test_isa();
	test_trim_envelope();
	test_glide_energy();
	test_quat_norm_and_stall();
	test_modes();
	test_observables();
	test_takeoff();
	printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
	return failures != 0;
}
