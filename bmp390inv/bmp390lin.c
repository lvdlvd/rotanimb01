// bmp390lin.c — BMP390 compensation and its inverse, see bmp390lin.h.

#include "bmp390lin.h"

#include <math.h>

void bmp390_cal_init(struct BMP390Cal *c, const struct BMP390Trim *t) {
	// datasheet 8.4 "calibration coefficients" scalings
	c->t1 = (float)t->par_t1 * 256.0f;             // / 2^-8
	c->t2 = (float)t->par_t2 / 1073741824.0f;      // / 2^30
	c->t3 = (float)t->par_t3 / 281474976710656.0f; // / 2^48

	c->p1 = (float)(t->par_p1 - 16384) / 1048576.0f;   // / 2^20
	c->p2 = (float)(t->par_p2 - 16384) / 536870912.0f; // / 2^29
	c->p3 = (float)t->par_p3 / 4294967296.0f;          // / 2^32
	c->p4 = (float)t->par_p4 / 137438953472.0f;        // / 2^37
	c->p5 = (float)t->par_p5 * 8.0f;                   // / 2^-3
	c->p6 = (float)t->par_p6 / 64.0f;                  // / 2^6
	c->p7 = (float)t->par_p7 / 256.0f;                 // / 2^8
	c->p8 = (float)t->par_p8 / 32768.0f;               // / 2^15
	c->p9 = (float)t->par_p9 / 281474976710656.0f;     // / 2^48
	c->p10 = (float)t->par_p10 / 281474976710656.0f;   // / 2^48
	c->p11 = (float)t->par_p11 / 36893488147419103232.0f; // / 2^65
}

void bmp390_trim_regs(const struct BMP390Trim *t, uint8_t regs[21]) {
	uint8_t *p = regs;
	*p++ = (uint8_t)t->par_t1;
	*p++ = (uint8_t)(t->par_t1 >> 8);
	*p++ = (uint8_t)t->par_t2;
	*p++ = (uint8_t)(t->par_t2 >> 8);
	*p++ = (uint8_t)t->par_t3;
	*p++ = (uint8_t)t->par_p1;
	*p++ = (uint8_t)(t->par_p1 >> 8);
	*p++ = (uint8_t)t->par_p2;
	*p++ = (uint8_t)(t->par_p2 >> 8);
	*p++ = (uint8_t)t->par_p3;
	*p++ = (uint8_t)t->par_p4;
	*p++ = (uint8_t)t->par_p5;
	*p++ = (uint8_t)(t->par_p5 >> 8);
	*p++ = (uint8_t)t->par_p6;
	*p++ = (uint8_t)(t->par_p6 >> 8);
	*p++ = (uint8_t)t->par_p7;
	*p++ = (uint8_t)t->par_p8;
	*p++ = (uint8_t)t->par_p9;
	*p++ = (uint8_t)(t->par_p9 >> 8);
	*p++ = (uint8_t)t->par_p10;
	*p++ = (uint8_t)t->par_p11;
}

// All internal evaluation is double: near full scale one praw count moves P
// by ~5e-8 relative — below float32 epsilon — so a float pipeline cannot
// meet the 1-LSB round trip. On the target this is soft-float, but the baro
// path runs at 50 Hz: a handful of soft-double ops is microseconds.

// the pressure polynomial's praw coefficients at a given compensated T
static void pcoef(const struct BMP390Cal *c, double T, double k[4]) {
	k[0] = c->p5 + T * (c->p6 + T * ((double)c->p7 + T * c->p8));
	k[1] = c->p1 + T * (c->p2 + T * ((double)c->p3 + T * c->p4));
	k[2] = c->p9 + T * c->p10;
	k[3] = c->p11;
}

static double tcomp(const struct BMP390Cal *c, uint32_t traw) {
	double s = (double)traw - c->t1;
	return s * (c->t2 + s * (double)c->t3);
}

void bmp390_forward(const struct BMP390Cal *c, uint32_t traw, uint32_t praw, double *t_degc, double *p_pa) {
	double T = tcomp(c, traw);
	double k[4];
	pcoef(c, T, k);
	double x = (double)praw;
	*t_degc = T;
	*p_pa = k[0] + x * (k[1] + x * (k[2] + x * k[3]));
}

static uint32_t clamp24(double v) {
	if (!(v > 0.0)) {
		return 0;
	}
	if (v > 16777215.0) {
		return 16777215;
	}
	return (uint32_t)(v + 0.5);
}

void bmp390_inverse(const struct BMP390Cal *c, float t_degc, double p_pa,
                    uint32_t *traw, uint32_t *praw, uint32_t praw_seed) {
	// temperature: t3*s^2 + t2*s - T = 0. The root near the linear solution
	// T/t2, in the cancellation-free (citardauq) form — the textbook
	// (-t2 + sqrt(.))/(2 t3) subtracts nearly equal ~1e-5 quantities.
	double T = t_degc;
	double s;
	double disc = (double)c->t2 * c->t2 + 4.0 * c->t3 * T;
	if (c->t3 != 0.0f && disc > 0.0) {
		s = 2.0 * T / (c->t2 + sqrt(disc));
	} else {
		s = T / c->t2; // linear device, or out of the parabola's range
	}
	uint32_t tr = clamp24(s + c->t1);
	// one integer refinement against the forward map
	{
		double e0 = fabs(tcomp(c, tr) - T);
		uint32_t alt = tcomp(c, tr) > T ? tr - 1 : tr + 1;
		if (alt <= 16777215 && fabs(tcomp(c, alt) - T) < e0) {
			tr = alt;
		}
	}
	*traw = tr;

	// pressure coefficients at the T the DUT will compute from the traw we
	// actually serve (quantization must not leak into P)
	double k[4];
	pcoef(c, tcomp(c, tr), k);

#define PEVAL(x) (k[0] + (x) * (k[1] + (x) * (k[2] + (x) * k[3])))
	// Newton on the cubic; the linear estimate is always a sound seed, the
	// caller's previous praw is better only when it is actually nearby
	double xlin = (p_pa - k[0]) / k[1];
	double x = xlin;
	if (praw_seed != 0 && fabs((double)praw_seed - xlin) < 1048576.0) {
		x = (double)praw_seed;
	}
	for (int i = 0; i < 8; i++) {
		double f = PEVAL(x) - p_pa;
		double df = k[1] + x * (2.0 * k[2] + x * 3.0 * k[3]);
		if (df == 0.0) {
			break;
		}
		double step = f / df;
		x -= step;
		if (fabs(step) < 0.03125) {
			break;
		}
	}
	// Newton off the rails (pathological trim): bisect — the map is
	// monotonic on real parts, and this path never runs for smooth input
	if (!(x >= 0.0 && x <= 16777215.0) || fabs(PEVAL(x) - p_pa) > 4.0 * fabs(k[1])) {
		double lo = 0.0, hi = 16777215.0;
		int rising = PEVAL(hi) > PEVAL(lo);
		for (int i = 0; i < 26; i++) {
			double mid = 0.5 * (lo + hi);
			if ((PEVAL(mid) < p_pa) == rising) {
				lo = mid;
			} else {
				hi = mid;
			}
		}
		x = 0.5 * (lo + hi);
	}
	uint32_t pr = clamp24(x);
	// one integer refinement: pick the neighbour that lands closer
	{
		double x0 = (double)pr;
		double e0 = PEVAL(x0) - p_pa;
		double x1 = e0 > 0.0 ? x0 - 1.0 : x0 + 1.0;
		if (x1 >= 0.0 && x1 <= 16777215.0 && fabs(PEVAL(x1) - p_pa) < fabs(e0)) {
			pr = (uint32_t)x1;
		}
	}
#undef PEVAL
	*praw = pr;
}
