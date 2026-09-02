/*
 * cordic_math.c — shared frontend.
 *
 * Implements the float API on top of the q1.31 CORDIC backend. This file
 * is compiled unchanged for both the device and the host test build, and
 * it uses only:
 *   - integer/bit operations,
 *   - IEEE-754 single-precision +, -, *, /, comparisons and int<->float
 *     conversions (bit-identical on Cortex-M4F and x86/ARM hosts provided
 *     -ffp-contract=off and no fast-math; see README).
 * Hence host results equal device results by construction, given a
 * backend-conformant CORDIC model.
 *
 * Function mapping (RM0440 §17.3.2):
 *   sinf/cosf  -> SINE/COSINE, angle reduced to [-pi/2, pi/2] then /pi
 *   atan2f     -> PHASE with common power-of-two prescale
 *   hypotf     -> MODULUS with common power-of-two prescale
 *   sqrtf      -> SQRT, x = m * 4^k, m in [0.25, 1)
 *   logf       -> LN,   x = m * 2^k, m in [0.5, 1), SCALE = 1
 *   expf       -> COSH+SINH (e^r = cosh r + sinh r), x = k*ln2 + r
 *   powf       -> expf(y * logf(|x|)) with sign/special-case handling
 *   log1pf     -> logf(1 + x) (see note at definition)
 *   fmodf, fabsf, roundf -> plain C, exact
 */

#include <stdint.h>
#include "cordic.h"        /* public API (was the math.h drop-in upstream) */
#include "cordic_port.h"

/* ---------------------------------------------------------------------- */
/* Bit-level float helpers (exact, identical everywhere)                  */
/* ---------------------------------------------------------------------- */

typedef union { float f; uint32_t u; int32_t i; } f32bits;

static inline uint32_t f2u(float f)    { f32bits v; v.f = f; return v.u; }
static inline float    u2f(uint32_t u) { f32bits v; v.u = u; return v.f; }

/* Kept as #defines: 0x80000000 exceeds INT_MAX, so these uint32_t masks
 * cannot portably be C11 enum constants. */
#define F32_SIGN  0x80000000u
#define F32_EXPM  0x7F800000u
#define F32_MANM  0x007FFFFFu

static inline int  is_nan_u(uint32_t u)  { return (u & ~F32_SIGN) > F32_EXPM; }
static inline int  is_inf_u(uint32_t u)  { return (u & ~F32_SIGN) == F32_EXPM; }
static inline float cm_copysign(float m, float s)
{
    return u2f((f2u(m) & ~F32_SIGN) | (f2u(s) & F32_SIGN));
}

/*
 * NaN handling note: NaNs are produced and propagated with BIT operations
 * only. Generating them arithmetically (0.0/0.0, x+y) yields
 * hardware-/optimizer-dependent payloads (x86 runtime gives 0xFFC00000,
 * constant folding and ARM give 0x7FC00000), which would break host/device
 * bit-equality. The cost is that no FE_INVALID exception is raised; the
 * project does not use fenv.
 */
static inline float cm_qnan(void)                /* canonical quiet NaN */
{
    return u2f(0x7FC00000u);
}
static inline float quiet_nan_of(float x)         /* quiet, keep payload */
{
    return u2f(f2u(x) | 0x00400000u);
}

/* 2^n as a float for n in [-126, 127]; callers split larger steps. */
static inline float pow2i(int n)
{
    return u2f((uint32_t)(n + 127) << 23);
}

/* Multiply by 2^n with correct overflow to inf / underflow through
 * subnormals to zero, for any int n. */
static float scale2(float x, int n)
{
    while (n > 127)  { x *= pow2i(127);  n -= 127; if (is_inf_u(f2u(x))) return x; }
    while (n < -126) { x *= pow2i(-126); n += 126; if (x == 0.0f)        return x; }
    return x * pow2i(n);
}

/*
 * Decompose finite nonzero |x| as f * 2^E with f in [1, 2).
 * Returns E; *frac gets f. Handles subnormals.
 */
static int frexp1(float x, float *frac)
{
    uint32_t u = f2u(x) & ~F32_SIGN;
    int e = (int)(u >> 23);
    if (e == 0) {                       /* subnormal: renormalize */
        u = f2u(u2f(u) * 0x1p64f);      /* exact: power-of-two scale */
        e = (int)(u >> 23) - 64;
    }
    *frac = u2f((u & F32_MANM) | (127u << 23));
    return e - 127;
}

/* ---------------------------------------------------------------------- */
/* q1.31 conversion (shared by all functions; saturating)                 */
/* ---------------------------------------------------------------------- */

static inline int32_t f2q31(float f)
{
    if (f >= 1.0f)  return INT32_MAX;            /* 1 - 2^-31 */
    if (f <= -1.0f) return INT32_MIN;            /* -1        */
    return (int32_t)(f * 0x1p31f);               /* C cast: truncate */
}

static inline float q31_2f(int32_t q)
{
    return (float)q * 0x1p-31f;                  /* exact for any q */
}

/* ---------------------------------------------------------------------- */
/* Trivial functions (plain C, exact)                                     */
/* ---------------------------------------------------------------------- */

float fabsf(float x)
{
    return u2f(f2u(x) & ~F32_SIGN);
}

/* C semantics: round half away from zero; -0 preserved; NaN/inf pass. */
float roundf(float x)
{
    uint32_t u = f2u(x);
    int e = (int)((u >> 23) & 0xFF) - 127;

    if (e >= 23) return x;                       /* integer, inf, or NaN */
    if (e < 0) {
        if (e == -1) return cm_copysign(1.0f, x);    /* 0.5 <= |x| < 1 */
        return cm_copysign(0.0f, x);                 /* |x| < 0.5      */
    }
    {
        uint32_t frac = F32_MANM >> e;
        if ((u & frac) == 0) return x;           /* already integral */
        u += 0x00400000u >> e;                   /* add 0.5 in fractional units */
        u &= ~frac;
        return u2f(u);
    }
}

/* Exact remainder via exponent-aligned binary long division (integer ops). */
float fmodf(float x, float y)
{
    uint32_t ux = f2u(x), uy = f2u(y);
    uint32_t sx = ux & F32_SIGN;
    int ex = (int)((ux >> 23) & 0xFF);
    int ey = (int)((uy >> 23) & 0xFF);
    uint32_t mx, my;

    if (is_nan_u(ux)) return quiet_nan_of(x);
    if (is_nan_u(uy)) return quiet_nan_of(y);
    if ((uy << 1) == 0 || ex == 0xFF)
        return cm_qnan();                          /* fmod(x,0), fmod(inf,y) */
    if ((ux << 1) <= (uy << 1)) {
        if ((ux << 1) == (uy << 1)) return cm_copysign(0.0f, x);
        return x;                                /* |x| < |y| */
    }

    /* normalize mantissas to 1.23-bit form */
    if (ex == 0) { mx = (ux & F32_MANM); while (!(mx >> 23)) { mx <<= 1; ex--; } ex++; }
    else mx = (ux & F32_MANM) | 0x00800000u;
    if (ey == 0) { my = (uy & F32_MANM); while (!(my >> 23)) { my <<= 1; ey--; } ey++; }
    else my = (uy & F32_MANM) | 0x00800000u;

    for (; ex > ey; ex--) {
        if (mx >= my) {
            mx -= my;
            if (mx == 0) return cm_copysign(0.0f, x);
        }
        mx <<= 1;
    }
    if (mx >= my) {
        mx -= my;
        if (mx == 0) return cm_copysign(0.0f, x);
    }

    while (!(mx >> 23)) { mx <<= 1; ex--; }      /* renormalize */
    if (ex > 0) {
        ux = (mx - 0x00800000u) | ((uint32_t)ex << 23);
    } else {
        ux = mx >> (1 - ex);                     /* subnormal result */
    }
    return u2f(ux | sx);
}

/* ---------------------------------------------------------------------- */
/* sinf / cosf / sincosf                                                  */
/* ---------------------------------------------------------------------- */

/* pi split for Cody-Waite reduction. P1/P2 carry 12 significant bits each
 * so n*P1 and n*P2 are EXACT float products for |n| < 2^12, keeping the
 * reduction error ~2e-7 for |x| up to ~4096*pi. */
static const float CM_PI_P1  = 3.140625f;                /* 0x1.92p+1         */
static const float CM_PI_P2  = 9.67502593994140625e-4f;  /* 0x1.fb4p-11       */
static const float CM_PI_P3  = 1.50995802528086636e-7f;  /* 0x1.4442d2p-23    */
static const float CM_INV_PI = 0.318309873342514038086f; /* 1/pi rounded      */
static const float CM_PI_F   = 3.14159274101257324219f;  /* pi rounded to f32 */

/*
 * Reduce x to r = x - n*pi with |r| <= pi/2 (+ a few ulp); returns r and
 * the parity of n in *odd. For |x| >= 2^24 single precision carries no
 * fractional angle information; we first fold by fmodf(x, 2*pi_hi+lo)
 * — deterministic, and as accurate as the format allows.
 */
static float reduce_pi(float x, int *odd)
{
    float fn, r;
    int32_t n;

    if (fabsf(x) >= 0x1p24f) {
        /* exact fmod against the f32 value of 2pi; residual phase error is
         * inherent to representing the argument in single precision */
        x = fmodf(x, 2.0f * CM_PI_F);
    }
    fn = roundf(x * CM_INV_PI);
    n  = (int32_t)fn;
    r  = x - fn * CM_PI_P1;                      /* exact for |fn| < 2^12 */
    r  = r - fn * CM_PI_P2;                      /* exact product         */
    r  = r - fn * CM_PI_P3;
    *odd = n & 1;
    return r;
}

/* Core: returns m*sin and m*cos of an angle given as (angle/pi) q1.31. */
static void cordic_sincos_q31(int32_t angle_over_pi, int32_t *s, int32_t *c)
{
    int32_t args[2], res[2];
    args[0] = angle_over_pi;
    args[1] = INT32_MAX;                          /* modulus m = 1 - 2^-31 */
    cordic_backend_run(cm_csr(CM_FUNC_SIN, CM_PREC_TRIG, 0, 1, 1), args, res);
    *s = res[0];
    *c = res[1];
}

float sinf(float x)
{
    uint32_t u = f2u(x);
    int odd;
    float r;
    int32_t s, c;

    if (is_nan_u(u) || is_inf_u(u)) return u2f(u | F32_EXPM | 0x00400000u);
    if (fabsf(x) < 0x1p-12f) return x;            /* sin x = x to < 2^-37 */

    r = reduce_pi(x, &odd);
    cordic_sincos_q31(f2q31(r * CM_INV_PI), &s, &c);
    return odd ? -q31_2f(s) : q31_2f(s);
}

float cosf(float x)
{
    uint32_t u = f2u(x);
    int odd;
    float r;
    int32_t s, c;

    if (is_nan_u(u) || is_inf_u(u)) return u2f(u | F32_EXPM | 0x00400000u);
    if (fabsf(x) < 0x1p-12f) return 1.0f;         /* cos x = 1 to < 2^-25 */

    r = reduce_pi(x, &odd);
    cordic_sincos_q31(f2q31(r * CM_INV_PI), &s, &c);
    return odd ? -q31_2f(c) : q31_2f(c);
}

void sincosf(float x, float *sp, float *cp)
{
    uint32_t u = f2u(x);
    int odd;
    float r;
    int32_t s, c;

    if (is_nan_u(u) || is_inf_u(u)) { *sp = *cp = u2f(u | F32_EXPM | 0x00400000u); return; }
    if (fabsf(x) < 0x1p-12f) { *sp = x; *cp = 1.0f; return; }

    r = reduce_pi(x, &odd);
    cordic_sincos_q31(f2q31(r * CM_INV_PI), &s, &c);
    *sp = odd ? -q31_2f(s) : q31_2f(s);
    *cp = odd ? -q31_2f(c) : q31_2f(c);
}

/* ---------------------------------------------------------------------- */
/* atan2f / hypotf                                                        */
/* ---------------------------------------------------------------------- */

/*
 * Scale the pair (a, b) by a common power of two such that the larger
 * magnitude lands in [0.25, 0.5). Power-of-two scaling is exact (down to
 * harmless sub-2^-31 underflow of the smaller member). Returns the
 * exponent k of the applied factor 2^-k.
 */
static int prescale_pair(float a, float b, float *sa, float *sb)
{
    float m = fabsf(a) >= fabsf(b) ? fabsf(a) : fabsf(b);
    float f;
    int e = frexp1(m, &f);        /* m = f*2^e, f in [1,2) */
    int k = e + 2;                /* m * 2^-k in [0.25, 0.5) */
    *sa = scale2(a, -k);
    *sb = scale2(b, -k);
    return k;
}

float atan2f(float y, float x)
{
    uint32_t ux = f2u(x), uy = f2u(y);
    float sx, sy;
    int32_t args[2], res[2];

    if (is_nan_u(uy)) return quiet_nan_of(y);
    if (is_nan_u(ux)) return quiet_nan_of(x);

    /* IEEE special cases (C99 F.10.1.4) */
    if ((uy << 1) == 0) {                                   /* y = +-0 */
        if ((ux & F32_SIGN) == 0 && (ux << 1) != 0) return y;        /* x > 0  */
        if ((ux << 1) == 0) return (ux & F32_SIGN) ? cm_copysign(CM_PI_F, y) : y;
        return cm_copysign(CM_PI_F, y);                              /* x < 0  */
    }
    if ((ux << 1) == 0)                                     /* x = +-0, y != 0 */
        return cm_copysign(0.5f * CM_PI_F, y);
    if (is_inf_u(uy)) {
        if (is_inf_u(ux))
            return cm_copysign((ux & F32_SIGN) ? 0.75f * CM_PI_F
                                               : 0.25f * CM_PI_F, y);
        return cm_copysign(0.5f * CM_PI_F, y);
    }
    if (is_inf_u(ux))
        return (ux & F32_SIGN) ? cm_copysign(CM_PI_F, y)
                               : cm_copysign(0.0f, y);

    (void)prescale_pair(x, y, &sx, &sy);          /* phase is scale-invariant */
    args[0] = f2q31(sx);
    args[1] = f2q31(sy);
    cordic_backend_run(cm_csr(CM_FUNC_PHASE, CM_PREC_TRIG, 0, 1, 0), args, res);
    /* For y != 0 the sign of atan2 is the sign of y. Enforcing it here
     * repairs two engine artifacts at once: the documented wrap of
     * results near +pi to -pi (RM0440 §17.3.2), and the loss of y's sign
     * when |y| << |x| truncates to q1.31 zero. */
    return cm_copysign(fabsf(q31_2f(res[0]) * CM_PI_F), y);
}

float hypotf(float x, float y)
{
    uint32_t ux = f2u(x), uy = f2u(y);
    float sx, sy;
    int k;
    int32_t args[2], res[2];

    /* C99 F.10.4.3: hypot(inf, anything) = inf, even NaN */
    if (is_inf_u(ux) || is_inf_u(uy)) return INFINITY;
    if (is_nan_u(ux)) return quiet_nan_of(x);
    if (is_nan_u(uy)) return quiet_nan_of(y);
    if ((ux << 1) == 0) return fabsf(y);
    if ((uy << 1) == 0) return fabsf(x);

    k = prescale_pair(x, y, &sx, &sy);            /* sum of squares <= 0.5 */
    args[0] = f2q31(sx);
    args[1] = f2q31(sy);
    cordic_backend_run(cm_csr(CM_FUNC_MOD, CM_PREC_TRIG, 0, 1, 0), args, res);
    return scale2(q31_2f(res[0]), k);
}

/* ---------------------------------------------------------------------- */
/* sqrtf                                                                  */
/* ---------------------------------------------------------------------- */

float sqrtf(float x)
{
    uint32_t u = f2u(x);
    float f, r;
    int e, k;
    int32_t args[2], res[2];

    if (is_nan_u(u)) return quiet_nan_of(x);
    if ((u << 1) == 0) return x;                  /* +-0 */
    if (u & F32_SIGN) return cm_qnan();             /* negative -> NaN */
    if (is_inf_u(u)) return x;

    e = frexp1(x, &f);                            /* x = f*2^e, f in [1,2) */
    if (e & 1) { f *= 0.5f; e += 1; }             /* x = m*4^k, m in [0.5,1) */
    else       { f *= 0.25f; e += 2; }            /*            m in [0.25,0.5) */
    k = e >> 1;

    if (f < 0.75f) {                              /* SCALE=0 window */
        args[0] = f2q31(f);
        cordic_backend_run(cm_csr(CM_FUNC_SQRT, CM_PREC_SQRT, 0, 0, 0), args, res);
        r = q31_2f(res[0]);
    } else {                                      /* SCALE=1 window */
        args[0] = f2q31(f * 0.5f);
        cordic_backend_run(cm_csr(CM_FUNC_SQRT, CM_PREC_SQRT, 1, 0, 0), args, res);
        r = q31_2f(res[0]) * 2.0f;
    }
    return scale2(r, k);
}

/* ---------------------------------------------------------------------- */
/* logf / log1pf                                                          */
/* ---------------------------------------------------------------------- */

/* ln2 split (musl float constants) */
static const float CM_LN2_HI  = 0.693145751953125f;      /* 0x1.62e4p-1    */
static const float CM_LN2_LO  = 1.42860676533018518e-6f; /* 0x1.7f7d1cp-20 */
static const float CM_INV_LN2 = 1.44269502162933349609f;

float logf(float x)
{
    uint32_t u = f2u(x);
    float f, lnm;
    int k;
    int32_t args[2], res[2];

    if (is_nan_u(u)) return quiet_nan_of(x);
    if ((u << 1) == 0) return -INFINITY;          /* log(+-0) = -inf */
    if (u & F32_SIGN) return cm_qnan();             /* negative -> NaN */
    if (is_inf_u(u)) return x;
    if (x == 1.0f) return 0.0f;                   /* exact, expected by callers */

    k = frexp1(x, &f) + 1;                        /* x = (f/2)*2^k, f/2 in [0.5,1) */

    /* LN, SCALE=1: ARG1 = m*2^-1, RES1 = 2^-2 * ln m */
    args[0] = f2q31(f * 0.25f);                   /* (f/2) * 2^-1 */
    cordic_backend_run(cm_csr(CM_FUNC_LN, CM_PREC_HYP, 1, 0, 0), args, res);
    lnm = q31_2f(res[0]) * 4.0f;                  /* ln(f/2), in [-ln2, 0) */

    return (float)k * CM_LN2_HI + (lnm + (float)k * CM_LN2_LO);
}

/*
 * log1pf(x) = logf(1 + x).
 *
 * NOTE: the accuracy-sensitive call site is logAdd-style, where the
 * argument is exp(d) with d <= 0, i.e. x in (0, 1]. On that range the
 * absolute error of this implementation is bounded by the CORDIC ln
 * error (~2^-16, see README) — verify against the caller's tolerance.
 * The 1+x == 1 guard returns x for arguments
 * below the rounding granularity of 1.0f, which is exact to within one
 * ulp there (ln(1+x) = x - x^2/2 + ..., and x^2/2 < 2^-48).
 */
float log1pf(float x)
{
    uint32_t u = f2u(x);

    if (is_nan_u(u)) return quiet_nan_of(x);
    if (x == -1.0f) return -INFINITY;
    if (x < -1.0f) return cm_qnan();
    if (1.0f + x == 1.0f) return x;               /* tiny |x|, keeps sign of 0 */
    return logf(1.0f + x);
}

/* ---------------------------------------------------------------------- */
/* expf                                                                   */
/* ---------------------------------------------------------------------- */

float expf(float x)
{
    uint32_t u = f2u(x);
    float fn, r, er;
    int32_t k;
    int32_t args[2], res[2];

    if (is_nan_u(u)) return quiet_nan_of(x);
    if (is_inf_u(u)) return (u & F32_SIGN) ? 0.0f : x;
    if (x > 88.8f)   return INFINITY;             /* overflow  */
    if (x < -104.0f) return 0.0f;                 /* underflow */

    /* x = k*ln2 + r, |r| <= ln2/2 = 0.3466 (within hyperbolic range) */
    fn = roundf(x * CM_INV_LN2);
    k  = (int32_t)fn;
    r  = x - fn * CM_LN2_HI;
    r  = r - fn * CM_LN2_LO;

    /* COSH, SCALE=1: ARG1 = r*2^-1; RES1 = cosh(r)/2, RES2 = sinh(r)/2.
     * e^r = cosh r + sinh r; the q1.31 sum is exact (<= 0.71 + headroom). */
    args[0] = f2q31(r * 0.5f);
    cordic_backend_run(cm_csr(CM_FUNC_COSH, CM_PREC_HYP, 1, 0, 1), args, res);
    er = q31_2f(res[0] + res[1]) * 2.0f;

    return scale2(er, (int)k);
}

/* ---------------------------------------------------------------------- */
/* powf                                                                   */
/* ---------------------------------------------------------------------- */

/* y integral? 0 = no, 1 = odd integer, 2 = even integer */
static int integer_kind(float y)
{
    float r = roundf(y);
    if (r != y) return 0;
    if (fabsf(y) >= 0x1p24f) return 2;            /* large floats are even */
    return ((int64_t)r & 1) ? 1 : 2;
}

/*
 * powf via exp(y * ln x). Relative error grows roughly as
 * |y*ln x| * 2^-16; fine for moderate exponents (see README).
 */
float powf(float x, float y)
{
    uint32_t ux = f2u(x), uy = f2u(y);
    int yk;
    float ax, r;

    if ((uy << 1) == 0) return 1.0f;              /* pow(x, +-0) = 1, even NaN */
    if (x == 1.0f) return 1.0f;                   /* pow(1, y) = 1, even NaN  */
    if (is_nan_u(ux)) return quiet_nan_of(x);
    if (is_nan_u(uy)) return quiet_nan_of(y);

    yk = integer_kind(y);

    if ((ux << 1) == 0) {                         /* x = +-0 */
        if (uy & F32_SIGN)
            return (yk == 1) ? cm_copysign(INFINITY, x) : INFINITY;
        return (yk == 1) ? x : 0.0f;
    }
    if (is_inf_u(uy)) {
        ax = fabsf(x);
        if (ax == 1.0f) return 1.0f;
        if ((ax > 1.0f) == !(uy & F32_SIGN)) return INFINITY;
        return 0.0f;
    }
    if (is_inf_u(ux)) {
        if (ux & F32_SIGN) {
            if (uy & F32_SIGN) return (yk == 1) ? -0.0f : 0.0f;
            return (yk == 1) ? -INFINITY : INFINITY;
        }
        return (uy & F32_SIGN) ? 0.0f : INFINITY;
    }
    if (ux & F32_SIGN) {
        if (yk == 0) return cm_qnan();              /* neg ^ non-integer: NaN */
        r = expf(y * logf(fabsf(x)));
        return (yk == 1) ? -r : r;
    }
    return expf(y * logf(x));
}
