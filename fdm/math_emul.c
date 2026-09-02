/*
 * math_emul.c — host backend: software model of the STM32G4 CORDIC.
 *
 * Models the documented engine: a 24-bit (q1.23) datapath driven for
 * PRECISION*4 iterations, in circular or hyperbolic mode, rotation or
 * vectoring. Pure integer arithmetic throughout; no floating point.
 *
 * The function-to-engine mapping is reconstructed from RM0440's range
 * limits, which it reproduces exactly:
 *   atan : vectoring of (2^-n, x*2^-n); max x = 128 since
 *          atan(128) = 1.563 < sum(atan 2^-i) = 1.743 rad
 *   ln   : vectoring of 2^-n*(x+1, x-1); z -> ln(x)/2, and the
 *          documented x <= 9.35 is ln(9.35)/2 = 1.118 = hyperbolic
 *          convergence bound
 *   sqrt : vectoring of (x*2^-n + c, x*2^-n - c), c = 2^-(n+2);
 *          modulus = sqrt(4c * x*2^-n) = 2^-n*sqrt(x), and the
 *          documented x >= 0.027 is 0.25*e^(-2*1.118) = 0.0267
 *
 * Per RM0440's precision notes, vectoring z is accumulated with the
 * angle-table entries pre-shifted right by SCALE (this is what makes the
 * "precision reduced proportionally" behavior fall out naturally, and
 * keeps the accumulator inside q1.23).
 *
 * Undocumented micro-architecture choices live in the calibration enum
 * below.
 * True silicon bit-exactness should be confirmed once with
 * test/device_dump (see README, "Calibration").
 */

#include <stdint.h>
#include "cordic_port.h"

/* ----------------------------------------------------------------------
 * Calibration knobs — every assumption about UNDOCUMENTED silicon
 * behavior is isolated here. RM0440 documents the CORDIC's I/O behavior;
 * ST training material (and AN5325) add that the engine datapath —
 * shifters, adders and angle table — is 24-bit (q1.23). The remaining
 * micro-architectural choices below are not published. After running
 * test/device_dump on real silicon and diffing against `make vectors`,
 * bit-exact calibration is a matter of flipping these values — not
 * rewriting the engine. Defaults are the simplest plausible hardware
 * choices.
 * -------------------------------------------------------------------- */
enum {
    /* q1.31 -> q1.23 narrowing of input arguments:
     * 0 = truncate (take top 24 bits, floor), 1 = round to nearest.
     * (q1.23 -> q1.31 widening is left-shift by 8, zero fill.) */
    CM_EMUL_NARROW_ROUND = 0,

    /* Phase (atan2) result when the accumulated angle nudges past +-1.0
     * (i.e. +-pi): RM0440 notes results "close to pi may sometimes wrap
     * to -pi", so the default models 24-bit two's-complement wrap.
     * 0 = saturate, 1 = wrap. */
    CM_EMUL_PHASE_WRAP = 1,

    /* Hyperbolic iteration repeat schedule (required for convergence).
     * Standard CORDIC repeats i = 4, 13, 40, ...; with <= 24 engine
     * steps only 4 and 13 are reachable.
     * Angle tables: entries are round-to-nearest of atan(2^-i)/pi and
     * atanh(2^-i) in q1.23; if silicon turns out to truncate,
     * regenerate them truncated. Gain correction is applied as a
     * post-multiply (vectoring) resp. seeds x0 (rotation). */
    CM_EMUL_HYP_REPEAT_A = 4,
    CM_EMUL_HYP_REPEAT_B = 13,
};


enum {
    ONE23  = 1 << 23,
    HALF23 = 1 << 22,
};

/* The silicon carries a guard bit in the ANGLE accumulator z: it is q1.(23+G)
 * while x/y stay q1.23. G=1 was found by capture (it makes cosh/sinh bit-exact
 * and moves atan/atanh/ln toward exact); G=0 and G>=2 are both worse — see the
 * README "CORDIC silicon calibration". The angle tables above are q1.(23+G), so
 * changing G means regenerating them. */
enum {
    CM_EMUL_ANGLE_GUARD = 1,
    ZHALF = 1 << (22 + CM_EMUL_ANGLE_GUARD),  /* 0.5 in the z domain */
};
/* load a q1.31 angle into the q1.(23+G) accumulator, keeping G guard bits */
static inline int32_t z_from_angle(int32_t q31) { return q31 >> (8 - CM_EMUL_ANGLE_GUARD); }
/* a vectoring angle result lives in z (q1.(23+G)); drop the guard for output */
static inline int32_t z_to_q23(int32_t z) { return z >> CM_EMUL_ANGLE_GUARD; }

/* compile-time sanity: we rely on arithmetic right shift */
typedef char cm_assert_arith_shift[((-1) >> 1 == -1) ? 1 : -1];

/* ---- angle tables (round-to-nearest; see tuning header) -------------- */
/* These live in the ANGLE path (z), which carries one guard bit, so they are
 * q1.(23+CM_EMUL_ANGLE_GUARD) = q1.24. x/y never use them and stay q1.23. */

/* atan(2^-i)/pi, q1.24, i = 0..23 */
static const int32_t cm_atan_tab[24] = {
    4194304, 2476042, 1308273, 664100, 333339, 166832, 83436, 41721,
    20861, 10430, 5215, 2608, 1304, 652, 326, 163,
    81, 41, 20, 10, 5, 3, 1, 1,
};

/* atanh(2^-i), q1.24, i = 1..24 ([0] unused) */
static const int32_t cm_atanh_tab[25] = {
    0, 9215828, 4285116, 2108178, 1049945, 524459, 262165, 131075,
    65536, 32768, 16384, 8192, 4096, 2048, 1024, 512,
    256, 128, 64, 32, 16, 8, 4, 2,
    1,
};

enum {
    /* Gain reciprocals, TRUNCATED to q-format (not round-to-nearest): silicon
     * truncates the gain-correction multiply, so the calibrated capture matches
     * floor, not round. (round-to-nearest would be 5094007 / 5064610.) */
    INVK_Q23  = 5094006,  /* 1/K  = 0.6072529350088813 (circular gain),   q1.23 */
    INVKH_Q22 = 5064609,  /* 1/Kh = 1.2074970677630608 (hyperbolic gain), q2.22 */
};

/* ---- fixed-point primitives ------------------------------------------ */

static inline int32_t narrow_q31(int32_t q31)
{
    if (CM_EMUL_NARROW_ROUND)
        return (q31 + 0x80) >> 8;            /* round to nearest */
    return q31 >> 8;                         /* truncate: top 24 bits */
}

static inline int32_t widen_sat(int32_t q23)
{
    if (q23 >= ONE23)  return INT32_MAX;     /* saturate to  1 - 2^-31 */
    if (q23 < -ONE23)  return INT32_MIN;     /* saturate to -1         */
    return (int32_t)((uint32_t)q23 << 8);
}

static inline int32_t widen_wrap(int32_t q23)
{
    return (int32_t)((uint32_t)q23 << 8);    /* 24-bit two's-complement wrap */
}

static inline int32_t mul_q23(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b) >> 23);
}

static inline int32_t mul_q22(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b) >> 22);
}

/* ---- circular mode ---------------------------------------------------- */

/* Rotation: angle/pi (q1.(23+G), [-1,1)) in z, modulus m (q1.23).
 * Outputs m*cos and m*sin in q1.23. */
static void circ_rot(int32_t theta, int32_t m, int iters,
                     int32_t *xc, int32_t *ys)
{
    int32_t x = mul_q23(m, INVK_Q23);
    int32_t y = 0;
    int32_t z = theta;
    int i;

    /* quadrant pre-rotation: bring |z| <= 0.5 (converge bound 0.5549) */
    if (z > ZHALF)       { int32_t t = x; x = -y; y = t;  z -= ZHALF; }
    else if (z < -ZHALF) { int32_t t = x; x = y;  y = -t; z += ZHALF; }

    /* Direction from the sign of z, with z==0 taking the NEGATIVE branch
     * (z > 0, not z >= 0) — matches silicon, which derives the direction from a
     * subtract/borrow rather than a plain z[MSB] test. Worth ~36 fewer cos
     * mismatches. See README. */
    for (i = 0; i < iters && i < 24; i++) {
        int32_t xs = x >> i, yss = y >> i;
        if (z > 0) { x -= yss; y += xs; z -= cm_atan_tab[i]; }
        else       { x += yss; y -= xs; z += cm_atan_tab[i]; }
    }
    *xc = x;
    *ys = y;
}

/* Vectoring: inputs (x, y) q1.23; outputs angle/pi in z (q1.(23+G), may
 * transiently exceed +-1 by < 2^-22) and K*modulus (q1.23). */
static void circ_vec(int32_t x, int32_t y, int scale, int iters,
                     int32_t *zout, int32_t *mout)
{
    int32_t z = 0;
    int i;

    /* pre-rotation into the right half-plane */
    if (x < 0) {
        int32_t t = x;
        if (y >= 0) { x = y;  y = -t; z = ZHALF; }
        else        { x = -y; y = t;  z = -ZHALF; }
    }

    for (i = 0; i < iters && i < 24; i++) {
        int32_t xs = x >> i, ys = y >> i;
        if (y >= 0) { x += ys; y -= xs; z += cm_atan_tab[i] >> scale; }
        else        { x -= ys; y += xs; z -= cm_atan_tab[i] >> scale; }
    }
    *zout = z;
    *mout = mul_q23(x, INVK_Q23);
}

/* ---- hyperbolic mode --------------------------------------------------- */

/* Iterate the hyperbolic recurrence for `steps` engine steps over the
 * index sequence 1,2,3,4,4,5,...,13,13,14,... (repeats per tuning hdr).
 * mode: 0 = rotation (drive z->0), 1 = vectoring (drive y->0).
 * Table entries are applied pre-shifted right by `scale`. */
static void hyp_engine(int32_t *px, int32_t *py, int32_t *pz,
                       int mode, int scale, int steps)
{
    int32_t x = *px, y = *py, z = *pz;
    int i = 1, rep_a = 0, rep_b = 0;
    int s;

    for (s = 0; s < steps && i < 25; s++) {
        int32_t xs = x >> i, ys = y >> i;
        int32_t alpha = cm_atanh_tab[i <= 24 ? i : 24] >> scale;
        int neg = mode ? (y >= 0) : (z < 0);

        if (!neg) { x += ys; y += xs; z -= alpha; }
        else      { x -= ys; y -= xs; z += alpha; }

        if (i == CM_EMUL_HYP_REPEAT_A && !rep_a)      rep_a = 1;
        else if (i == CM_EMUL_HYP_REPEAT_B && !rep_b) rep_b = 1;
        else i++;
    }
    *px = x; *py = y; *pz = z;
}

/* ---- backend entry ----------------------------------------------------- */

void cordic_backend_run(uint32_t csr, const int32_t args[2], int32_t res[2])
{
    unsigned func  = csr & 0xF;
    int iters      = (int)((csr >> 4) & 0xF) * 4;
    int scale      = (int)((csr >> 8) & 0x7);
    int32_t a1     = narrow_q31(args[0]);
    int32_t a2     = (csr & CM_CSR_NARGS) ? narrow_q31(args[1])
                                          : (INT32_MAX >> 8); /* reset ARG2 = +1 */
    int32_t x, y, z;

    res[0] = res[1] = 0;

    switch (func) {
    case CM_FUNC_COS:
    case CM_FUNC_SIN: {
        int32_t c, s;
        circ_rot(z_from_angle(args[0]), a2, iters, &c, &s); /* z keeps the guard bit */
        if (func == CM_FUNC_COS) { res[0] = widen_sat(c); res[1] = widen_sat(s); }
        else                     { res[0] = widen_sat(s); res[1] = widen_sat(c); }
        break;
    }

    case CM_FUNC_PHASE:
    case CM_FUNC_MOD: {
        int32_t ph, m;
        circ_vec(a1, a2, 0, iters, &ph, &m);     /* ph is q1.(23+G) */
        {
            int32_t phq = z_to_q23(ph);
            int32_t phw = CM_EMUL_PHASE_WRAP ? widen_wrap(phq) : widen_sat(phq);
            int32_t mw  = widen_sat(m);      /* RM: saturates to 1 */
            if (func == CM_FUNC_PHASE) { res[0] = phw; res[1] = mw; }
            else                       { res[0] = mw;  res[1] = phw; }
        }
        break;
    }

    case CM_FUNC_ATAN:
        /* vector (2^-n, x*2^-n); z accumulates atan(x)/pi * 2^-n (q1.(23+G)) */
        x = ONE23 >> scale; y = a1; z = 0;
        {
            int i;
            for (i = 0; i < iters && i < 24; i++) {
                int32_t xs = x >> i, ys = y >> i;
                if (y >= 0) { x += ys; y -= xs; z += cm_atan_tab[i] >> scale; }
                else        { x -= ys; y += xs; z -= cm_atan_tab[i] >> scale; }
            }
        }
        res[0] = widen_sat(z_to_q23(z));
        break;

    case CM_FUNC_COSH:
    case CM_FUNC_SINH:
        x = INVKH_Q22 >> (scale >= 1 ? scale - 1 : 0);  /* (1/Kh)*2^-n, q1.23 */
        y = 0;
        /* cosh/sinh load z from the q1.23 angle lifted into the guard domain
         * with a ZERO guard bit (NOT z_from_angle's real guard) — empirically
         * this is what makes them bit-exact. The asymmetry vs sin/cos (which use
         * the real guard bit) is unexplained; see README, open question. */
        z = a1 << CM_EMUL_ANGLE_GUARD;
        hyp_engine(&x, &y, &z, 0, scale, iters);
        if (func == CM_FUNC_COSH) { res[0] = widen_sat(x); res[1] = widen_sat(y); }
        else                      { res[0] = widen_sat(y); res[1] = widen_sat(x); }
        break;

    case CM_FUNC_ATANH:
        x = ONE23 >> scale; y = a1; z = 0;
        hyp_engine(&x, &y, &z, 1, scale, iters);
        res[0] = widen_sat(z_to_q23(z));
        break;

    case CM_FUNC_LN:
        /* vector 2^-n * (x+1, x-1); z -> 2^-n * ln(x)/2 = RES1 */
        x = a1 + (ONE23 >> scale);
        y = a1 - (ONE23 >> scale);
        z = 0;
        hyp_engine(&x, &y, &z, 1, scale, iters);
        res[0] = widen_sat(z_to_q23(z));
        break;

    case CM_FUNC_SQRT:
        /* vector (a + c, a - c), c = 2^-(n+2); modulus -> 2^-n*sqrt(x) */
        x = a1 + (ONE23 >> (scale + 2));
        y = a1 - (ONE23 >> (scale + 2));
        z = 0;
        hyp_engine(&x, &y, &z, 1, scale, iters);
        res[0] = widen_sat(mul_q22(x, INVKH_Q22));
        break;

    default:
        break;
    }

    /* Honor the CSR result count: hardware writes RES2 only when NRES is set
     * (cordic_port.h). Single-result ops (e.g. phase/mod as issued by the
     * frontend) leave RES2 unread/zero — match that instead of always emitting
     * the second engine output. */
    if (!(csr & CM_CSR_NRES))
        res[1] = 0;
}
