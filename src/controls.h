#pragma once

// controls — the FDM's PWM input path (fdm-DESIGN.md F2): per-channel
// calibration, zero-order hold on the 50 Hz captures, first-order servo lag
// with a rate limit, and the failsafe ageing that makes DUT reboots and
// cable pulls non-events.
//
// Channel order is ArduPlane's AETR on capture channels 1..4 (cap2):
// 0 aileron, 1 elevator, 2 throttle, 3 rudder. Calibration per channel:
// {min, trim, max us, signed full deflection}; mapping is piecewise-linear
// around trim. Defaults 1000/1500/2000 us, +-20 deg aileron, +-25 deg
// elevator/rudder, throttle 0..1 (min..max, trim ignored).
//
// Failsafe: a channel that has been seen alive and then ages past 100 ms
// holds its last value for 0.5 s, then goes to calibration trim (throttle
// to idle). A channel never seen since boot stays wherever the FDM's trim
// solver put it — so an air-start without a PWM source keeps flying trimmed
// instead of idling the engine 600 ms in.

#include <stdint.h>

#include "fdm.h"
#include "pwm.h"

enum { CTL_AIL, CTL_ELE, CTL_THR, CTL_RUD, CTL_NCH };

// PWM_CAL (TMC 0x45) payload, 8 bytes: u8 ch, u8 part, then
//   part 0: u16 min us, u16 trim us, u16 max us
//   part 1: i16 full deflection, 0.01 deg signed (sign flips the channel);
//           for the throttle 10000 = full scale 1.0
void controls_cal_msg(const uint8_t p[8]);

// reset the calibration table and the lag states to the defaults above
void controls_defaults(void);

// step the input path at the FDM rate: read the captures (zero-order hold),
// age the failsafe, advance the servo lags, and write the post-lag
// deflections into *out. now_us is the harness microsecond clock, dt the
// FDM step. hold is the FDM trim solution to keep for never-seen channels.
void controls_step(struct PWMIn *cap, uint32_t now_us, float dt,
                   const struct FdmControls *hold, struct FdmControls *out);

// STATUS flag (bit 4): some channel is in failsafe hold or trim fallback
uint16_t controls_flags(void);

// post-lag deflections for the heartbeat, and per-channel failsafe state
const struct FdmControls *controls_state(void);

// per-channel failsafe as decimal digits, AETR order: 0 live (or never
// seen), 1 holding last value, 2 at the trim fallback
unsigned controls_fs_code(void);
