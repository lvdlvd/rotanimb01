// golden.c — host-side check of the physics model's maneuver invariants
// (DESIGN.md M5): bench cal, level flight, coordinated standard turn, climb,
// heading wrap, domain clamps. Run by `make` in this directory.

#include "physics.h"

#include <math.h>
#include <stdio.h>

static int failures;

#define CHECK(cond, ...)                       \
	do {                                       \
		if (!(cond)) {                         \
			failures++;                        \
			printf("  FAIL %s: ", __func__);   \
			printf(__VA_ARGS__);               \
			printf("\n");                      \
		}                                      \
	} while (0)

#define DT 5e-4f // 2 kHz, as on the target

static void run(float seconds) {
	for (int i = 0; i < (int)(seconds / DT); i++) {
		physics_step(DT);
		const struct PhysicsTruth *t = physics_truth();
		for (int k = 0; k < 3; k++) {
			CHECK(isfinite(t->rate[k]) && isfinite(t->sforce[k]) && isfinite(t->mag[k]),
			      "non-finite output at step %d", i);
		}
		if (failures) {
			return;
		}
	}
}

static float norm3(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

static void test_bench(void) {
	physics_init();
	run(1.0f);
	const struct PhysicsTruth *t = physics_truth();
	CHECK(physics_flags() & PHYSICS_FLAG_BENCH, "bench flag");
	CHECK(norm3(t->rate) < 1e-6f, "rates %g", (double)norm3(t->rate));
	CHECK(fabsf(t->sforce[0]) < 1e-3f && fabsf(t->sforce[1]) < 1e-3f, "level f xy");
	CHECK(fabsf(t->sforce[2] + PHYSICS_G) < 1e-3f, "1 g down: fz %g", (double)t->sforce[2]);
	CHECK(fabsf(t->mag[0] - 19.97f) < 0.05f && fabsf(t->mag[1]) < 1e-3f &&
	          fabsf(t->mag[2] - 44.01f) < 0.05f,
	      "bench field %g %g %g", (double)t->mag[0], (double)t->mag[1], (double)t->mag[2]);
	CHECK(fabsf(t->p_pa - 101325.0f) < 1.0f, "QNH %g", (double)t->p_pa);
	CHECK(fabsf(t->t_degc - 15.0f) < 0.01f, "T %g", (double)t->t_degc);
	printf("bench ok\n");
}

static void test_level(void) {
	physics_init();
	physics_cmd(20, 0, 0);
	run(5.0f);
	const struct PhysicsTruth *t = physics_truth();
	float alpha0 = 0.035f;
	CHECK(norm3(t->rate) < 1e-3f, "rates %g", (double)norm3(t->rate));
	CHECK(fabsf(t->sforce[0] - PHYSICS_G * sinf(alpha0)) < 0.01f, "trim fx %g", (double)t->sforce[0]);
	CHECK(fabsf(t->sforce[1]) < 0.01f, "fy %g", (double)t->sforce[1]);
	CHECK(fabsf(t->sforce[2] + PHYSICS_G * cosf(alpha0)) < 0.01f, "fz %g", (double)t->sforce[2]);
	CHECK(fabsf(norm3(t->sforce) - PHYSICS_G) < 0.01f, "|f| %g", (double)norm3(t->sforce));
	printf("level ok\n");
}

static void test_turn(void) {
	physics_init();
	float v = 20, psidot = 0.14f; // ~8 deg/s at 20 m/s -> phi ~ 16 deg
	physics_cmd(v, 0, psidot);
	run(8.0f);
	const struct PhysicsTruth *t = physics_truth();
	float phi = atanf(psidot * v / PHYSICS_G);
	float theta = 0.035f; // gamma = 0
	// load factor 1/cos(phi), ball centered, strapdown rates
	CHECK(fabsf(norm3(t->sforce) - PHYSICS_G / cosf(phi)) < 0.02f,
	      "|f| %g want %g", (double)norm3(t->sforce), (double)(PHYSICS_G / cosf(phi)));
	CHECK(fabsf(t->sforce[1]) < 0.02f, "coordinated fy %g", (double)t->sforce[1]);
	CHECK(fabsf(t->rate[0] + psidot * sinf(theta)) < 1e-3f, "p %g", (double)t->rate[0]);
	CHECK(fabsf(t->rate[1] - psidot * cosf(theta) * sinf(phi)) < 1e-3f, "q %g", (double)t->rate[1]);
	CHECK(fabsf(t->rate[2] - psidot * cosf(theta) * cosf(phi)) < 1e-3f, "r %g", (double)t->rate[2]);
	CHECK(fabsf(norm3(t->mag) - 48.33f) < 0.02f, "|m| %g", (double)norm3(t->mag));
	// keep turning through several psi wraps: outputs must stay steady
	float f0 = norm3(t->sforce);
	for (int i = 0; i < (int)(60.0f / DT); i++) {
		physics_step(DT);
		float df = norm3(physics_truth()->sforce) - f0;
		CHECK(fabsf(df) < 0.05f, "wrap glitch step %d: d|f| %g", i, (double)df);
		if (failures) {
			return;
		}
	}
	printf("turn ok (phi %.1f deg, load %.3f)\n", (double)(phi * 180 / M_PI),
	       (double)(1 / cosf(phi)));
}

static void test_climb(void) {
	physics_init();
	physics_cmd(20, 2, 0);
	run(5.0f);
	const struct PhysicsTruth *t = physics_truth();
	float h1 = t->h, p1 = t->p_pa;
	run(5.0f);
	t = physics_truth();
	CHECK(fabsf((t->h - h1) - 2.0f * 5.0f) < 0.05f, "hdot ramp: dh %g", (double)(t->h - h1));
	float dpdh = (t->p_pa - p1) / (t->h - h1); // ~ -rho g ~ -12 Pa/m near SL
	CHECK(dpdh > -12.5f && dpdh < -11.5f, "dp/dh %g", (double)dpdh);
	CHECK(fabsf(t->t_degc - (15.0f - 0.0065f * t->h)) < 0.02f, "T(h) %g", (double)t->t_degc);
	printf("climb ok (h %.1f m, p %.0f Pa)\n", (double)t->h, (double)t->p_pa);
}

static void test_clamp(void) {
	physics_init();
	physics_cmd(5, 10, 0); // hdot/V = 2: asin domain must clamp, never NaN
	run(3.0f);
	CHECK(physics_flags() & PHYSICS_FLAG_CLAMP, "clamp flag");
	printf("clamp ok\n");
}

int main(void) {
	test_bench();
	test_level();
	test_turn();
	test_climb();
	test_clamp();
	printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
	return failures != 0;
}
