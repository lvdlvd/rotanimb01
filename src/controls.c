#include "controls.h"

// per-channel calibration and runtime state
struct Chan {
	// calibration (PWM_CAL-settable)
	uint16_t min_us, trim_us, max_us;
	float defl; // signed full deflection, rad (throttle: scale, 1.0)
	// input path state
	uint32_t count;     // last seen capture generation
	uint32_t change_us; // when it last advanced
	uint8_t seen;       // captured at least once since boot
	uint8_t fs;         // 0 live, 1 holding last, 2 at trim fallback
	float target;       // post-calibration, pre-lag deflection
	float y;            // lag output
};
static struct Chan ch[CTL_NCH];
static struct FdmControls post; // published post-lag deflections

// servo lag time constants (fdm-DESIGN.md): tau 60 ms surfaces, 0.3 s
// throttle; surface rate limit 300 deg/s
static const float TAU_SURF = 0.060f, TAU_THR = 0.3f;
static const float RATE_LIM = 300.0f * 3.14159265f / 180.0f; // rad/s

void controls_defaults(void) {
	static const float defl0[CTL_NCH] = {
		[CTL_AIL] = 20.0f * 3.14159265f / 180.0f,
		[CTL_ELE] = 25.0f * 3.14159265f / 180.0f,
		[CTL_THR] = 1.0f,
		[CTL_RUD] = 25.0f * 3.14159265f / 180.0f,
	};
	for (int i = 0; i < CTL_NCH; i++) {
		ch[i] = (struct Chan){.min_us = 1000, .trim_us = 1500, .max_us = 2000, .defl = defl0[i]};
	}
}

void controls_cal_msg(const uint8_t p[8]) {
	unsigned c = p[0];
	if (c >= CTL_NCH) {
		return;
	}
	if (p[1] == 0) {
		uint16_t mn = (uint16_t)(p[2] << 8 | p[3]);
		uint16_t tr = (uint16_t)(p[4] << 8 | p[5]);
		uint16_t mx = (uint16_t)(p[6] << 8 | p[7]);
		if (mn < tr && tr < mx) { // reject inverted/degenerate tables
			ch[c].min_us = mn;
			ch[c].trim_us = tr;
			ch[c].max_us = mx;
		}
	} else if (p[1] == 1) {
		int16_t d = (int16_t)(p[2] << 8 | p[3]); // 0.01 deg (throttle: 1e-4)
		ch[c].defl = (c == CTL_THR) ? (float)d * 1e-4f
		                            : (float)d * 0.01f * (3.14159265f / 180.0f);
	}
}

// piecewise-linear around trim to [-1, 1] (throttle: min..max to [0, 1])
static float normalize(const struct Chan *c, int is_thr, uint32_t w_us) {
	float w = (float)w_us;
	if (w < (float)c->min_us) {
		w = (float)c->min_us;
	}
	if (w > (float)c->max_us) {
		w = (float)c->max_us;
	}
	if (is_thr) {
		return (w - (float)c->min_us) / (float)(c->max_us - c->min_us);
	}
	if (w >= (float)c->trim_us) {
		return (w - (float)c->trim_us) / (float)(c->max_us - c->trim_us);
	}
	return (w - (float)c->trim_us) / (float)(c->trim_us - c->min_us);
}

void controls_step(struct PWMIn *cap, uint32_t now_us, float dt,
                   const struct FdmControls *hold, struct FdmControls *out) {
	const float hold_tgt[CTL_NCH] = {hold->da, hold->de, hold->dt, hold->dr};

	for (int i = 0; i < CTL_NCH; i++) {
		struct Chan *c = &ch[i];
		struct PWMInSample s = pwmin_get(cap, i);

		if (s.count != c->count) { // fresh capture: zero-order hold
			c->count = s.count;
			c->change_us = now_us;
			c->seen = 1;
			c->fs = 0;
			c->target = normalize(c, i == CTL_THR, s.width_us) * c->defl;
		} else if (!c->seen) {
			// never captured: keep flying the FDM's trim solution (a bench
			// air-start without a PWM source must not failsafe the engine)
			c->fs = 0;
			c->target = hold_tgt[i];
			c->y = hold_tgt[i]; // no lag transient when PWM does appear
		} else if (now_us - c->change_us > 600000) {
			c->fs = 2; // held 0.5 s past the 100 ms ageing: to trim
			c->target = (i == CTL_THR) ? 0.0f : 0.0f;
		} else if (now_us - c->change_us > 100000) {
			c->fs = 1; // hold last value
		}

		// first-order lag + rate limit toward target
		float tau = (i == CTL_THR) ? TAU_THR : TAU_SURF;
		float step = (c->target - c->y) * (dt / tau);
		float lim = ((i == CTL_THR) ? 3.0f : RATE_LIM) * dt; // throttle 3/s
		if (step > lim) {
			step = lim;
		}
		if (step < -lim) {
			step = -lim;
		}
		c->y += step;
	}

	post.da = ch[CTL_AIL].y;
	post.de = ch[CTL_ELE].y;
	post.dt = ch[CTL_THR].y;
	post.dr = ch[CTL_RUD].y;
	*out = post;
}

uint16_t controls_flags(void) {
	for (int i = 0; i < CTL_NCH; i++) {
		if (ch[i].fs != 0) {
			return 1 << 4;
		}
	}
	return 0;
}

const struct FdmControls *controls_state(void) { return &post; }

unsigned controls_fs_code(void) { // one decimal digit per channel, AETR order
	return (unsigned)ch[0].fs * 1000u + (unsigned)ch[1].fs * 100u +
	       (unsigned)ch[2].fs * 10u + (unsigned)ch[3].fs;
}
