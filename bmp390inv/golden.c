// golden — host-side round-trip validation of the BMP390 inverse model.
//
// Criterion (DESIGN M4): |round-trip error| < 1 LSB. Tested in the raw
// domain, which makes it trim-agnostic: sweep (traw, praw) over the 24-bit
// grid, forward-compensate to (T, P), invert, and require the recovered raw
// words within +-1 count of the originals. Run for a synthetic nominal trim
// and for randomized trims (rejecting non-monotonic ones, which no real
// device exhibits).
//
// Also cross-checks the float forward model against the flight-tested
// integer implementation (bmp_linearize) on the nominal trim: both implement
// the same datasheet polynomial, so they must agree to integer-truncation
// tolerance. That anchors the float model to what the DUT family computes.
//
//   cc -O2 -o golden golden.c bmp390lin.c <bminator>/src/bmp388.c -lm

#include "bmp390lin.h"

#include "bmp388.h" // the in-house integer reference

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// SYNTHETIC nominal trim: magnitudes chosen so the map covers roughly the
// physical envelope (T -40..85 degC, P 30..110 kPa over the 24-bit range).
// Replace with a real device's NVM dump when one is read out.
static const struct BMP390Trim nominal = {
	.par_t1 = 27000,
	.par_t2 = 18000,
	.par_t3 = -10,
	.par_p1 = 21400,
	.par_p2 = 16100,
	.par_p3 = 5,
	.par_p4 = -3,
	.par_p5 = 7000,
	.par_p6 = 1400,
	.par_p7 = 20,
	.par_p8 = -5,
	.par_p9 = 6000,
	.par_p10 = 3,
	.par_p11 = -2,
};

static uint32_t rngstate = 0x12345;
static uint32_t rng(void) {
	rngstate ^= rngstate << 13;
	rngstate ^= rngstate >> 17;
	rngstate ^= rngstate << 5;
	return rngstate;
}

// monotonicity check: T increasing in traw, and P increasing in praw at the
// coldest/middle/hottest temperatures of the sweep (a trim can flip the
// pressure slope at extreme T and no real device does that)
static int sane(const struct BMP390Cal *c) {
	static const uint32_t tcorner[3] = {1000, 1 << 23, 16759001};
	for (int i = 0; i < 63; i++) {
		uint32_t traw = (uint32_t)i << 18, praw = (uint32_t)i << 18;
		double t0, t1, p0, p1, dontcare;
		bmp390_forward(c, traw, 1 << 23, &t0, &dontcare);
		bmp390_forward(c, traw + (1 << 18), 1 << 23, &t1, &dontcare);
		if (t1 <= t0) {
			return 0;
		}
		for (int j = 0; j < 3; j++) {
			bmp390_forward(c, tcorner[j], praw, &t0, &p0);
			bmp390_forward(c, tcorner[j], praw + (1 << 18), &t0, &p1);
			if (p1 <= p0) {
				return 0;
			}
		}
	}
	return 1;
}

// round-trip a raw grid; returns max |delta| in counts, tracks Newton seeding
// with the previous praw as the sampler would
static uint32_t roundtrip(const struct BMP390Cal *c, uint32_t *worst_t, uint32_t *worst_p) {
	uint32_t maxdt = 0, maxdp = 0;
	uint32_t seed = 0;
	for (uint32_t ti = 0; ti < 64; ti++) {
		for (uint32_t pi = 0; pi < 64; pi++) {
			uint32_t traw = ti * 266000 + 1000; // ~24-bit coverage, off-grid steps
			uint32_t praw = pi * 266000 + 1000;
			double T, P;
			bmp390_forward(c, traw, praw, &T, &P);
			uint32_t traw2, praw2;
			bmp390_inverse(c, (float)T, P, &traw2, &praw2, seed);
			seed = praw2;
			uint32_t dt = traw2 > traw ? traw2 - traw : traw - traw2;
			uint32_t dp = praw2 > praw ? praw2 - praw : praw - praw2;
			if (dt > maxdt) {
				maxdt = dt;
			}
			if (dp > maxdp) {
				maxdp = dp;
			}
		}
	}
	*worst_t = maxdt;
	*worst_p = maxdp;
	return maxdt > maxdp ? maxdt : maxdp;
}

int main(void) {
	int fail = 0;

	// 1. nominal trim: round trip
	struct BMP390Cal cal;
	bmp390_cal_init(&cal, &nominal);
	if (!sane(&cal)) {
		printf("nominal trim not monotonic -- test bug\n");
		return 1;
	}
	uint32_t wt, wp;
	roundtrip(&cal, &wt, &wp);
	printf("nominal:  max |dtraw| %u, max |dpraw| %u counts %s\n", wt, wp,
	       (wt <= 1 && wp <= 1) ? "PASS" : "FAIL");
	fail += !(wt <= 1 && wp <= 1);

	// 2. nominal trim: float forward vs the flight-tested integer reference
	uint8_t regs[21];
	bmp390_trim_regs(&nominal, regs);
	struct LinearisationParameters par;
	bmp_decodeLinearisationParameters(&par, regs);
	float maxdT = 0, maxdP = 0;
	for (uint32_t ti = 0; ti < 32; ti++) {
		for (uint32_t pi = 0; pi < 32; pi++) {
			uint32_t traw = ti * 524287 + 1000, praw = pi * 524287 + 1000;
			double T, P;
			bmp390_forward(&cal, traw, praw, &T, &P);
			int32_t t_mdegc, p_mpa;
			bmp_linearize(&par, traw, praw, &t_mdegc, &p_mpa);
			float dT = fabsf((float)(T - t_mdegc / 1000.0));
			float dP = fabsf((float)(P - p_mpa / 1000.0));
			if (dT > maxdT) {
				maxdT = dT;
			}
			if (dP > maxdP) {
				maxdP = dP;
			}
		}
	}
	printf("vs integer reference: max |dT| %.4f degC, max |dP| %.3f Pa %s\n", (double)maxdT,
	       (double)maxdP, (maxdT < 0.01f && maxdP < 5.0f) ? "PASS" : "FAIL");
	fail += !(maxdT < 0.01f && maxdP < 5.0f);

	// 3. randomized trims (rejection-sampled for monotonicity)
	int tested = 0, rejected = 0;
	uint32_t wtmax = 0, wpmax = 0;
	while (tested < 100 && rejected < 10000) {
		struct BMP390Trim t = {
			.par_t1 = (uint16_t)(20000 + rng() % 20000),
			.par_t2 = (uint16_t)(14000 + rng() % 10000),
			.par_t3 = (int8_t)(rng() % 41) - 20,
			.par_p1 = (int16_t)(17000 + rng() % 9000),
			.par_p2 = (int16_t)(14000 + rng() % 5000),
			.par_p3 = (int8_t)(rng() % 41) - 20,
			.par_p4 = (int8_t)(rng() % 21) - 10,
			.par_p5 = (uint16_t)(3000 + rng() % 10000),
			.par_p6 = (uint16_t)(500 + rng() % 3000),
			.par_p7 = (int8_t)(rng() % 81) - 40,
			.par_p8 = (int8_t)(rng() % 21) - 10,
			.par_p9 = (int16_t)(rng() % 20000) - 10000,
			.par_p10 = (int8_t)(rng() % 21) - 10,
			.par_p11 = (int8_t)(rng() % 11) - 5,
		};
		struct BMP390Cal c;
		bmp390_cal_init(&c, &t);
		if (!sane(&c)) {
			rejected++;
			continue;
		}
		uint32_t a, b;
		roundtrip(&c, &a, &b);
		if (a > wtmax) {
			wtmax = a;
		}
		if (b > wpmax) {
			wpmax = b;
		}
		tested++;
	}
	printf("random trims: %d tested (%d rejected non-monotonic), max |dtraw| %u, max |dpraw| %u %s\n",
	       tested, rejected, wtmax, wpmax, (wtmax <= 1 && wpmax <= 1 && tested >= 50) ? "PASS" : "FAIL");
	fail += !(wtmax <= 1 && wpmax <= 1 && tested >= 50);

	printf("%s\n", fail ? "FAIL" : "PASS");
	return fail;
}
