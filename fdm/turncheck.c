// turncheck — the sign audit the eigenvalue golden can't do: bank the model
// with aileron and verify the DIRECTIONS agree everywhere. Right aileron
// (da > 0) must roll right (p > 0, phi > 0); a sustained right bank must
// yaw right (psi increasing) and turn the velocity vector right.

#include "fdm.h"

#include "cordic.h"
#include <stdio.h>

#define DT 1e-3f

static struct Fdm f;

static void euler(const float q[4], float *phi, float *the, float *psi) {
	*phi = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
	              1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
	float s = 2.0f * (q[0] * q[2] - q[3] * q[1]);
	if (s > 1.0f) s = 1.0f;
	if (s < -1.0f) s = -1.0f;
	*the = atan2f(s, sqrtf(1.0f - s * s)); // asin via atan2 (cordic has no asinf)
	*psi = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
	              1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

int main(void) {
	fdm_defaults(&f);
	struct FdmControls trim;
	if (fdm_trim(&f, 100.0f, 25.0f, 0.0f, &trim) != 0) {
		printf("trim failed\n");
		return 1;
	}

	const float R2D = 180.0f / 3.14159265f;
	struct FdmControls c = trim;

	// 1 s of right aileron (+5 deg), then neutral, watching the signs
	printf(" t     da     p(deg/s)  phi     psi     course  beta\n");
	for (int i = 0; i <= 20000; i++) {
		c.da = (i < 1000) ? trim.da + 5.0f / R2D : trim.da;
		fdm_step(&f, &c, DT);
		if (i % 2000 == 0) {
			float phi, the, psi;
			euler(f.quat, &phi, &the, &psi);
			float course = atan2f(f.truth.v_ned[1], f.truth.v_ned[0]);
			printf("%5.1f  %+5.1f  %+8.2f  %+6.1f  %+6.1f  %+6.1f  %+5.2f\n",
			       (double)(i * DT), (double)(c.da * R2D),
			       (double)(f.w_b[0] * R2D), (double)(phi * R2D),
			       (double)(psi * R2D), (double)(course * R2D),
			       (double)(f.truth.beta * R2D));
		}
	}

	// phase 2: aileron HELD +2 deg for 30 s — the sustained-turn statics
	printf("held aileron +2 deg:\n");
	printf(" t     p(deg/s)  phi     psi     course  beta\n");
	c.da = trim.da + 2.0f / R2D;
	for (int i = 0; i <= 30000; i++) {
		fdm_step(&f, &c, DT);
		if (i % 5000 == 0) {
			float phi2, the2, psi2;
			euler(f.quat, &phi2, &the2, &psi2);
			float course2 = atan2f(f.truth.v_ned[1], f.truth.v_ned[0]);
			printf("%5.1f  %+8.2f  %+6.1f  %+6.1f  %+6.1f  %+5.2f\n",
			       (double)(i * DT), (double)(f.w_b[0] * R2D),
			       (double)(phi2 * R2D), (double)(psi2 * R2D),
			       (double)(course2 * R2D), (double)(f.truth.beta * R2D));
		}
	}

	float phi, the, psi;
	euler(f.quat, &phi, &the, &psi);
	int ok = 1;
	if (!(phi > 0.0f)) {
		printf("SIGN FAIL: right aileron left phi %.1f deg (expect > 0)\n", (double)(phi * R2D));
		ok = 0;
	}
	if (!(psi > 0.0f)) {
		printf("SIGN FAIL: right bank yawed psi %.1f deg (expect > 0)\n", (double)(psi * R2D));
		ok = 0;
	}
	float course = atan2f(f.truth.v_ned[1], f.truth.v_ned[0]);
	if (!(course > 0.0f)) {
		printf("SIGN FAIL: velocity turned to course %.1f deg (expect > 0)\n", (double)(course * R2D));
		ok = 0;
	}
	printf(ok ? "turncheck OK: bank, yaw and course all agree to the right\n"
	          : "turncheck FAILED\n");
	return ok ? 0 : 1;
}
