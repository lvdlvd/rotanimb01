#pragma once

// cordic.h — single-precision math for STM32G4 backed by the CORDIC
// coprocessor. #include this and link cordic_math.c + cordic_stm32.c to get
// sinf/cosf/.../powf without -lm. Imported from the standalone cordic-math
// project (the bit-exact host emulation stays there for prototyping; only the
// backend-agnostic frontend and the device backend live here).
//
// The frontend is bit-equal to the host model only under specific build flags
// (see the cordic-math README). For correct results on-device, build with:
//   -fno-builtin       so the compiler emits real calls, not its own math
//   -ffp-contract=off  no FMA contraction (changes the low bits)
// and no -ffast-math / -funsafe-math-optimizations. Leave FPSCR at reset
// (round-to-nearest, flush-to-zero disabled).
//
// PRECONDITION: enable the CORDIC peripheral clock before the first call:
//   RCC.AHB1ENR |= RCC_AHB1ENR_CORDICEN;
// There is no init function; the library does no clock management.

#ifdef __cplusplus
extern "C" {
#endif

// ---- constants ----------------------------------------------------------

#define HUGE_VALF (__builtin_huge_valf())
#define INFINITY (__builtin_inff())
#define NAN (__builtin_nanf(""))

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef M_E
#define M_E 2.7182818284590452354
#endif

// ---- classification (minimal, macro-based) ------------------------------

#define isnan(x) (__builtin_isnan(x))
#define isinf(x) (__builtin_isinf(x))
#define isfinite(x) (__builtin_isfinite(x))
#define signbit(x) (__builtin_signbit(x))

// ---- the functions ------------------------------------------------------

float sinf(float x);
float cosf(float x);
float atan2f(float y, float x);
float hypotf(float x, float y);
float sqrtf(float x);
float expf(float x);
float logf(float x);
float log1pf(float x);
float powf(float x, float y);
float fmodf(float x, float y);
float fabsf(float x);
float roundf(float x);

// Extension: the CORDIC computes sin and cos together; this returns both for
// the price of one operation (GNU sincosf signature).
void sincosf(float x, float *s, float *c);

#ifdef __cplusplus
}
#endif
