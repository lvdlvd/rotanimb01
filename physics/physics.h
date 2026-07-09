#pragma once

// physics — commanded state -> sensor truth (DESIGN.md "Physics").
//
// Commands arrive in the earth (NED) frame: speed V, climb rate hdot, turn
// rate psidot. First-order lags (tau ~ 0.4 s) smooth them so derivatives
// exist and the DUT's EKF sees flyable transients. From the filtered state,
// coordinated flight gives the attitude (gamma = asin(hdot/V), bank from
// tan(phi) = psidot*V*cos(gamma)/g, theta = gamma + trim AoA); body rates
// come from the strapdown transform, specific force from differentiating
// the NED velocity vector — the 1/cos(phi) load factor in a steady turn
// falls out for free. V below a threshold is bench mode: gamma = phi =
// theta = 0, 1 g down, earth field, zero rates.
//
// All sensor truth is in the body FRD frame (x forward, z down): level
// flight reads specific force ~(g*sin(AoA), 0, -g). The DUT must mount all
// sensors ROTATION_NONE. Pure math, no device dependencies: host-compilable
// (golden.c asserts the maneuver invariants).

#include <stdint.h>

#define PHYSICS_G 9.80665f

struct PhysicsTruth {
	float rate[3];   // body rates p,q,r (rad/s, FRD)
	float sforce[3]; // specific force (m/s^2, FRD; level: ~{0.3, 0, -9.8})
	float mag[3];    // magnetic field (uT, FRD)
	float p_pa;      // static pressure (ISA from QNH + integrated h)
	float t_degc;    // ambient temperature at h
	float psi, h;    // integrated internal state (CAN readback, drift check)
};

enum { // physics_flags
	PHYSICS_FLAG_BENCH = 1 << 1, // V below threshold: level bench attitude
	PHYSICS_FLAG_CLAMP = 1 << 2, // asin/atan domain clamp active
};

void physics_init(void);
void physics_cmd(float v_ms, float hdot_ms, float psidot_rads); // lag targets
void physics_env(float qnh_pa, float t0_k, float b_ut, float incl_rad);
void physics_step(float dt); // integrate one step; 2 kHz nominal
const struct PhysicsTruth *physics_truth(void);
uint16_t physics_flags(void);
