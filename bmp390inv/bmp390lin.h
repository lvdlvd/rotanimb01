#pragma once

#include <stdint.h>

// BMP390 compensation model and its inverse.
//
// The DUT reads the sensor's NVM trim once (registers 0x31..0x45) and
// compensates the raw 24-bit ADC words itself, per the Bosch datasheet
// polynomials (BMP388/390 identical). The harness therefore runs the
// polynomial BACKWARDS: given the physics model's (T, P), produce the raw
// words to serve, such that the DUT's forward compensation reproduces
// (T, P) to within one raw LSB.
//
//   forward:  s  = traw - t1
//             T  = t2*s + t3*s^2                                    [degC]
//             P  = c0(T) + c1(T)*praw + c2(T)*praw^2 + c3*praw^3    [Pa]
//   inverse:  temperature analytically (quadratic root nearest T/t2),
//             pressure by Newton on the cubic, seeded with the previous
//             praw (samples move slowly; cold seed (P - c0)/c1).
//
// The pressure coefficients are evaluated at the T of the QUANTIZED traw,
// exactly as the DUT will, so temperature quantization does not leak into
// the served pressure.
//
// Coefficients are stored as float; all evaluation is double — near full
// scale one praw count is ~5e-8 of P, below float32 resolution. On the
// target double is soft-float, but the baro path runs at 50 Hz: microseconds.
// Shared between the host golden model and the device model plug-in.

struct BMP390Trim { // NVM layout, registers 0x31..0x45 in order
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t par_t3;
	int16_t par_p1;
	int16_t par_p2;
	int8_t par_p3;
	int8_t par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t par_p7;
	int8_t par_p8;
	int16_t par_p9;
	int8_t par_p10;
	int8_t par_p11;
};

struct BMP390Cal { // datasheet float-converted coefficients
	float t1, t2, t3;
	float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

// convert raw NVM values to the float coefficients (datasheet 8.4)
void bmp390_cal_init(struct BMP390Cal *c, const struct BMP390Trim *t);

// serialize the trim as the 21-byte register image at 0x31 (little-endian)
void bmp390_trim_regs(const struct BMP390Trim *t, uint8_t regs[21]);

// forward compensation (what the DUT computes from what we serve)
void bmp390_forward(const struct BMP390Cal *c, uint32_t traw, uint32_t praw, double *t_degc, double *p_pa);

// inverse: (t_degc, p_pa) -> 24-bit raw words, clamped to [0, 2^24-1].
// praw_seed: the previous praw for the Newton seed (pass 0 when cold).
void bmp390_inverse(const struct BMP390Cal *c, float t_degc, double p_pa,
                    uint32_t *traw, uint32_t *praw, uint32_t praw_seed);
