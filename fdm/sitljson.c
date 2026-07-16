// sitljson — ArduPilot SITL "JSON backend" wrapper around the SAME fdm.c the
// harness flies (SIL under the HIL: tune fast against identical physics, then
// validate on the bench through the real sensor path). Lockstep UDP: SITL
// sends a binary servo packet to :9002, we integrate and reply plain JSON;
// wall-clock speed is whatever the loop achieves (use SITL --speedup).
//
//   cc -O2 -o sitljson sitljson.c fdm.c ../cordic-emul-or-mathlib...   — see
//   the Makefile target; run BEFORE starting arduplane --model JSON:127.0.0.1
//
// Includes the harness's actuator model (first-order lag + rate limit per
// controls.c) so the tune sees the same servo phase the bench has. A reset
// of SITL (frame_count going backwards) re-parks the aircraft.

#include "fdm.h"

#include <arpa/inet.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

static struct Fdm fdm;
static struct FdmControls tgt, act; // demanded (from pwm) and lagged (to fdm)
static double sim_t;

// pwm -> deflection, the harness defaults: 1000/1500/2000 us, +-20/25/25 deg,
// throttle 0..1
static float defl(uint16_t us, float full_deg) {
	float x = ((float)us - 1500.0f) * (1.0f / 500.0f);
	x = x < -1.0f ? -1.0f : x > 1.0f ? 1.0f : x;
	return x * full_deg * ((float)M_PI / 180.0f);
}

// controls.c's actuator model: tau 60 ms + 300 deg/s rate limit; throttle
// tau 0.3 s
static float lag(float y, float u, float dt, float tau, float rate) {
	float a = dt / (tau + dt);
	float ny = y + a * (u - y);
	float dmax = rate * dt;
	float d = ny - y;
	if (d > dmax) d = dmax;
	if (d < -dmax) d = -dmax;
	return y + d;
}

int main(void) {
	int s = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in addr = {0};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(9002);
	if (bind(s, (struct sockaddr *)&addr, sizeof addr) != 0) {
		perror("bind 9002");
		return 1;
	}
	fdm_defaults(&fdm);
	uint32_t last_frame = 0;
	printf("sitljson: kitfox fdm on :9002, parked on the gear\n");

	for (;;) {
		uint8_t buf[80];
		struct sockaddr_in from;
		socklen_t flen = sizeof from;
		ssize_t n = recvfrom(s, buf, sizeof buf, 0, (struct sockaddr *)&from, &flen);
		if (n < 8) continue;
		uint16_t magic = (uint16_t)(buf[0] | buf[1] << 8);
		if (magic != 18458) continue;
		uint16_t frame_rate = (uint16_t)(buf[2] | buf[3] << 8);
		uint32_t frame_count;
		memcpy(&frame_count, &buf[4], 4);
		uint16_t pwm[16];
		memcpy(pwm, &buf[8], sizeof pwm);

		if (frame_count < last_frame) { // SITL restarted: re-park
			fdm_defaults(&fdm);
			memset(&tgt, 0, sizeof tgt);
			memset(&act, 0, sizeof act);
			sim_t = 0;
			printf("sitljson: SITL reset detected, re-parked\n");
		}
		last_frame = frame_count;

		tgt.da = defl(pwm[0], 20.0f);
		tgt.de = -defl(pwm[1], 25.0f); // stick back (low pwm) = up elevator = negative de
		tgt.dt = ((float)pwm[2] - 1000.0f) * 1e-3f;
		tgt.dt = tgt.dt < 0.0f ? 0.0f : tgt.dt > 1.0f ? 1.0f : tgt.dt;
		tgt.dr = defl(pwm[3], 25.0f);

		if (frame_rate == 0) frame_rate = 50;
		float dt_frame = 1.0f / (float)frame_rate;
		int sub = (int)(dt_frame / 1e-3f + 0.5f);
		if (sub < 1) sub = 1;
		float dt = dt_frame / (float)sub;
		const float r300 = 300.0f * (float)M_PI / 180.0f;
		for (int i = 0; i < sub; i++) {
			act.da = lag(act.da, tgt.da, dt, 0.06f, r300);
			act.de = lag(act.de, tgt.de, dt, 0.06f, r300);
			act.dr = lag(act.dr, tgt.dr, dt, 0.06f, r300);
			act.dt = lag(act.dt, tgt.dt, dt, 0.30f, 10.0f);
			fdm_step(&fdm, &act, dt);
			if (fdm.truth.h < 0.0f) { // harsh contact: re-park level (bench semantics)
				const float *q = fdm.truth.quat;
				float psi = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
				                   1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
				fdm.quat[0] = cosf(0.5f * psi);
				fdm.quat[1] = fdm.quat[2] = 0.0f;
				fdm.quat[3] = sinf(0.5f * psi);
				for (int k = 0; k < 3; k++) { fdm.v_b[k] = 0.0f; fdm.w_b[k] = 0.0f; }
				fdm.pos_cm[2] = 0;
				fdm.pos_rem[2] = 0.0f;
				fdm.on_ground = 1;
				fdm_step(&fdm, &act, dt);
			}
			sim_t += (double)dt;
		}

		const struct FdmTruth *t = &fdm.truth;
		char out[512];
		int m = snprintf(out, sizeof out,
			"{\"timestamp\":%.6f,"
			"\"imu\":{\"gyro\":[%.6f,%.6f,%.6f],\"accel_body\":[%.5f,%.5f,%.5f]},"
			"\"position\":[%.4f,%.4f,%.4f],"
			"\"velocity\":[%.4f,%.4f,%.4f],"
			"\"quaternion\":[%.6f,%.6f,%.6f,%.6f],"
			"\"airspeed\":%.3f}\n",
			sim_t,
			(double)t->rate[0], (double)t->rate[1], (double)t->rate[2],
			(double)t->sforce[0], (double)t->sforce[1], (double)t->sforce[2],
			(double)t->pos_cm[0] * 0.01, (double)t->pos_cm[1] * 0.01, (double)t->pos_cm[2] * 0.01,
			(double)t->v_ned[0], (double)t->v_ned[1], (double)t->v_ned[2],
			(double)t->quat[0], (double)t->quat[1], (double)t->quat[2], (double)t->quat[3],
			(double)t->va);
		sendto(s, out, (size_t)m, 0, (struct sockaddr *)&from, flen);
	}
}
