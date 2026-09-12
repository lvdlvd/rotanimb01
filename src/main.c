// rotanimb01 — HITL sensor simulator for ArduPlane (design in ../doc/DESIGN.md).
//
// M2 state: M0 skeleton (clock, console on USART1, vector manifest, fault
// machinery, heartbeat) + the 8-channel PWM capture (lib/pwm capture on TIM2+TIM3,
// 1 us tick; TIM2's 32-bit counter doubles as the harness microsecond clock)
// + the host CAN link (lib/fdcan, FDCAN1 on PA11/PA12, classic 1 Mbit).
//
// CAN dictionary: 29-bit bit-field headers and big-endian payloads per the
// shared in-house convention — layout and the harness's MSGID allocation
// (the 0x40 block, both LCCs) in canmsg.h. The host link carries that
// dictionary over one of two transports sharing PA11/PA12 (wiring-level
// either/or, chosen at build time): TRANSPORT=usb (default) tunnels the
// messages as pseudocan lines (nlib/fmtcan, no per-line crc: USB bulk has
// link-level integrity) over the CDC-ACM virtual serial port; TRANSPORT=can
// is FDCAN1 through an external transceiver.
//
// M5: TIM7 ticks at 10 kHz; the main loop consumes ticks and runs the table
// scheduler — physics integration on every 5th tick (2 kHz), and a Bresenham
// rate accumulator per device (acc += hz; acc >= 10000 -> sample) so any
// configured ODR is served with <= 100 us jitter and an exact long-term
// rate. Samples pull the physics truth, quantize per the LIVE config
// registers and commit. CMD_STATE/CMD_ENV decode into the lag targets;
// stale watchdog: no CMD_STATE for 1 s -> hold last state, flag in STATUS.

#include "device.h" // generated for STM32G474
#include "pinmux.h"

#include "binary.h"
#include "board.h"
#include "canmsg.h"
#include "canmsgq.h"
#include "controls.h"
#include "dronecan.h"
#include "clock.h"
#include "exti.h"
#include "console.h" // pulls serial.h + tprintf.h
#include "dma_g4.h"  // G4 DMAMUX requests
#include "usart_v3.h" // usart_init for FIFO-family USARTs
#include "fault.h"
#include "fdcan.h"
#include "gpio.h"
#include "fdm.h"
#include "nvic.h"
#include "physics.h"

#include <math.h>
#include "fmtcan.h"
#include "pwm.h"
#include "sensors.h"
#include "startup.h"
#include "remap_g4.h"
#include "usb.h"

extern const isr_t __vectors[]; // the vector table, defined at the foot of the file

static uint8_t tx_buf[1024]; // power-of-two rings
static uint8_t rx_buf[512];
static struct Serial vcp = SERIAL_INITIALIZER(USART1, tx_buf);
static struct Serial vcp_rx = SERIAL_INITIALIZER(USART1, rx_buf);
static struct SerialRXCounters vcp_rxc;

static void cputc(char c) { usart_putc(&USART1, c); } // polled, for the post-mortem dump

// 8x PWM capture from the DUT (DESIGN.md pinout); TIM3 also clocks the FDCAN
// message timestamps (TSCC), so CAN ts is us mod 2^16 on the same timebase.
static struct PWMIn cap2 = PWMIN_INITIALIZER_32(TIM2, PA0, PA1, PB10, PB11);
static struct PWMIn cap3 = PWMIN_INITIALIZER(TIM3, PC6, PC7, PC8, PC9);

// The harness microsecond clock: TIM2's free-running 32-bit 1 MHz counter.
static inline uint32_t now_us(void) { return cap2.tim->CNT; }

// ---- host command mailbox (written by the CAN RX irq, read at thread level)
struct CmdBox {
	uint8_t data[8];
	volatile uint8_t len;
	volatile uint32_t seq;   // increments per received frame
	volatile uint32_t rx_us; // now_us() at reception
};
static struct CmdBox cmd_state, cmd_env, cmd_noise, cmd_fdm_mode, cmd_fdm_init, cmd_pwm_cal,
		cmd_wind, cmd_param, cmd_gps_cfg, cmd_fdm_pos;

static void cmd_store(struct CmdBox *b, const uint8_t *p, size_t len) {
	for (size_t i = 0; i < len && i < 8; i++) {
		b->data[i] = p[i];
	}
	b->len = (uint8_t)len;
	b->rx_us = now_us();
	b->seq++;
}

// ---- host link TX ------------------------------------------------------------
static uint8_t srcid; // hashed from the device UID at boot

#ifdef TRANSPORT_CAN
static struct FDCan can1 = FDCAN_INITIALIZER(FDCAN1);
#else
// pseudocan over the USB VCP: usb_recv fills usb_rx from the USB_LP irq,
// everything else runs at thread level (fmtcan's SPSC split)
static uint8_t usb_rx_bytes[512], usb_tx_bytes[2048];
static struct Fifo usb_rx = {usb_rx_bytes, sizeof usb_rx_bytes - 1, 0, 0};
static struct Fifo usb_tx = {usb_tx_bytes, sizeof usb_tx_bytes - 1, 0, 0};
static uint32_t usb_bad, usb_drop; // malformed host lines; usb_tx overflows
#endif

static void can_send(uint32_t msgid, const uint8_t *payload, size_t len) {
	static uint8_t seq[16]; // per-message ts_seq, CANMSG_PWM14..PARAM_VAL
	uint32_t id29 = canmsg_id29(CANMSG_LCC_MEAS, msgid, 0, srcid, seq[msgid - CANMSG_PWM14]++);
#ifdef TRANSPORT_CAN
	fdcan_tx(&can1, (uint8_t)msgid, can_header_from29(id29), len, payload); // tag = msgid
#else
	// no per-line crc: USB bulk already has link-level integrity
	if (can_fifo_put(&usb_tx, 1, 0, can_header_from29(id29), len, payload) == 0) {
		usb_drop++;
	}
#endif
}

// one message from the host, either transport: latch known commands
static void host_msg(uint32_t id29, const uint8_t *p, size_t len) {
	if (canmsg_lcc(id29) != CANMSG_LCC_TMC) {
		return;
	}
	switch (canmsg_msgid(id29)) {
	case CANMSG_CMD_STATE:
		cmd_store(&cmd_state, p, len);
		break;
	case CANMSG_CMD_ENV:
		cmd_store(&cmd_env, p, len);
		break;
	case CANMSG_CMD_NOISE:
		cmd_store(&cmd_noise, p, len);
		break;
	case CANMSG_FDM_MODE:
		cmd_store(&cmd_fdm_mode, p, len);
		break;
	case CANMSG_FDM_INIT:
		cmd_store(&cmd_fdm_init, p, len);
		break;
	case CANMSG_PWM_CAL:
		cmd_store(&cmd_pwm_cal, p, len);
		break;
	case CANMSG_WIND:
		cmd_store(&cmd_wind, p, len);
		break;
	case CANMSG_PARAM_SET:
		cmd_store(&cmd_param, p, len);
		break;
	case CANMSG_GPS_CFG:
		cmd_store(&cmd_gps_cfg, p, len);
		break;
	case CANMSG_FDM_POS:
		cmd_store(&cmd_fdm_pos, p, len);
		break;
	}
}

// ---- the flight dynamics model (fdm-DESIGN.md mode 1) -------------------------
//
// The 6-DOF runs in the 10 kHz scheduler at 1 kHz behind the mode switch
// (F1). Controls come from the captured servo PWM through controls_step —
// calibration, zero-order hold, servo lag, failsafe (F2); channels never
// seen since boot hold the FDM_INIT trim solution, so an air-start without
// a PWM source flies trimmed instead of failing safe to idle.
static struct Fdm fdm;
static struct FdmControls fdm_ctl;      // post-lag, fed to fdm_step (F2: from PWM)
static struct FdmControls fdm_trim_ctl; // the trim solution: hold for never-seen channels
static volatile uint16_t fdm_crashes;   // harsh-contact events (latched count for
                                        // STATUS; the wreck re-parks on its gear
                                        // and keeps simulating — FDM_INIT clears)
static volatile uint8_t fdm_mode; // 0 = mode-0 kinematics, 1 = six-dof
static struct PhysicsTruth fdm_pt; // adapter: fdm truth in the sampler's shape

static const struct PhysicsTruth *truth(void) { return fdm_mode ? &fdm_pt : physics_truth(); }

static void fdm_publish_truth(void) {
	const struct FdmTruth *t = &fdm.truth;
	for (int i = 0; i < 3; i++) {
		fdm_pt.rate[i] = t->rate[i];
		fdm_pt.sforce[i] = t->sforce[i];
		fdm_pt.mag[i] = t->mag[i];
	}
	fdm_pt.p_pa = t->p_pa;
	fdm_pt.t_degc = t->t_degc;
	// yaw from q_nb for the STATUS message
	const float *q = t->quat;
	fdm_pt.psi = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
	                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
	fdm_pt.h = t->h;
}

// ---- the on-board DroneCAN GPS/airspeed feeder (fdm-DESIGN.md F4) -------------
//
// FDCAN3 (PB3/PB4) straight onto the DUT's CAN bus, no bridge board: Fix2 at
// 5 Hz from the fdm truth held back by a configurable lag (a zero-latency GPS
// would make HITL kinder than reality), RawAirData at 20 Hz and unconditional
// (see gps_feed_air), NodeStatus at 1 Hz.
// Node id 42. GPS_CFG (TMC 0x48) sets enable and lag at runtime.
static struct FDCan can_gps = FDCAN_INITIALIZER(FDCAN3);
static volatile uint32_t gps_busoff;

// The G4 FDCAN has only 3 hardware tx buffers and a Fix2+RawAir burst is 13
// frames: queue the burst (canmsgq, the flight stacks' pattern), pump from
// the tx-complete irq. SPSC: the thread enqueues, FDCAN3_IT0 drains.
static struct CanMsgQueue gps_q;

static void gps_pump(void) { // FDCAN3_IT0 context only
	for (struct CanMsg *m; (m = canmsgq_tail(&gps_q)) != NULL;) {
		if (fdcan_tx(&can_gps, 0, m->header, m->len, m->payload) < 0) {
			return; // hw buffers full: the next tx-complete irq re-pumps
		}
		canmsgq_pop_tail(&gps_q);
	}
}

static void gps_enqueue(const struct DroneCanFrame *fr, int n) {
	for (int i = 0; i < n; i++) {
		canmsgq_enq(&gps_q, now_us(), can_header_from29(fr[i].id29), fr[i].len, fr[i].data);
	}
	nvic_set_pending(FDCAN3_IT0_IRQn); // all pumping happens at irq level
}

struct GpsSnap {
	uint32_t us; // 0 = empty slot
	int32_t n_cm, e_cm, h_cm;
	int16_t v_cms[3];
	uint16_t ias_dms; // 0.1 m/s
};
static struct GpsSnap gps_ring[8];
static uint32_t gps_head;
static uint8_t gps_enable = 1, gps_lag_10ms = 15;
static uint8_t gps_tid_fix, gps_tid_air, gps_tid_ns;

// feeder origin: 45.52688 N, 1.667291 E — open farmland in central France,
// far from any real traffic; the example missions in doc/ are laid out
// around it. Integer microdegree math — float32 cannot carry 1e-8 deg at
// these magnitudes. 1 cm north = 8.9831e-4 deg * 1e8 / 1e4; east scaled by
// 1/cos(45.52688 deg) = 1/0.70057. Changing the origin means changing all
// three constants (and the missions).
static const int64_t gps_lat0_1e8 = 4552688000LL, gps_lon0_1e8 = 166729100LL;
static const int64_t gps_ncm_num = 89831, gps_ecm_num = 128225, gps_cm_den = 10000;

// the lagged truth snapshot the feed is derived from, or NULL before the
// fdm has produced lag_us of history
static const struct GpsSnap *gps_snap(uint32_t now) {
	uint32_t lag_us = (uint32_t)gps_lag_10ms * 10000u;
	for (uint32_t i = 0; i < 8; i++) {
		const struct GpsSnap *c = &gps_ring[(gps_head - 1 - i) & 7];
		if (c->us != 0 && now - c->us >= lag_us) {
			return c;
		}
	}
	return NULL;
}

// sensor noise (M7'-lite): datasheet-scale white noise in PHYSICAL units,
// added before quantization so it tracks whatever range the DUT configures.
// Sum of two xorshift uniforms — triangular, close enough to Gaussian for
// an EKF's purposes. Sigmas: BMI088 gyro ~0.2 deg/s and accel ~5 mg at
// AP's filter settings, RM3100 ~20 nT.
static float noise(float sigma) {
	static uint32_t rng = 0x1337c0deu;
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	float u1 = (float)(int32_t)(rng & 0xffff) - 32768.0f;
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	float u2 = (float)(int32_t)(rng & 0xffff) - 32768.0f;
	return (u1 + u2) * (sigma * (1.0f / 26756.0f)); // var(u1+u2) -> sigma^2
}

// CMD_NOISE (canmsg.h 0x42) scales, in 1/16 of the nominal level beside each
// model: 16 = 1.0x = what the bench boots with, 0 = off. RAM only, like every
// other harness setting — resend after a reboot. nz_baro floors at 16
// (sample_baro): that one turns up, never off.
static uint8_t nz_gyro = 16, nz_accel = 16, nz_mag = 16, nz_baro = 16, nz_bias = 16,
               nz_pitot = 16, nz_gps = 16;
static inline float nzf(uint8_t s) { return (float)s * (1.0f / 16.0f); }

// Aiding-sensor error. Until now the DroneCAN feeder carried NO error at
// all: Fix2 reported truth position with a fixed advertised covariance, and
// the pitot reported 0.5*rho*ias^2 exactly — a parked aircraft sent a
// bit-identical 0.0 Pa forever, which is precisely the stuck-source pattern
// PX4's DataValidator rejects after 100 samples and which leaves ArduPilot's
// airspeed offset calibration nothing to chew on. Lag was the only modelled
// degradation (gps_lag_10ms).
//
// GPS position error is FIRST-ORDER GAUSS-MARKOV, not white. Real GNSS error
// is slowly correlated — ionosphere, multipath, orbit and clock all drift
// over minutes — and an EKF filters white jitter away almost for free, so a
// white model would flatter the bench and hide exactly the slow-bias
// behaviour the filter is supposed to fight. Vertical is the worse axis, as
// on real receivers. Velocity is doppler-derived and much cleaner, so it
// stays white.
#define GPS_SIGMA_H_M 1.0f    // horizontal position, stationary sigma
#define GPS_SIGMA_V_M 2.0f    // vertical position
#define GPS_SIGMA_VEL 0.05f   // velocity, white, per axis (m/s)
#define GPS_TAU_S 60.0f       // correlation time of the position error
#define PITOT_SIGMA_PA 0.5f   // differential pressure RMS

static float gps_err_m[3]; // NED position error, the Gauss-Markov state
static uint32_t gps_err_us;

// Advance the correlated position error to `now`. Euler-Maruyama on
// de = -e*dt/tau + sigma*sqrt(2*dt/tau)*w, whose stationary sigma is the
// sigma passed in.
static void gps_err_step(uint32_t now) {
	uint32_t last = gps_err_us;
	gps_err_us = now;
	if (last == 0) {
		return; // first call: no interval to integrate over
	}
	float dt = (float)(now - last) * 1e-6f;
	// BOUND THE STEP. The feeder can be switched off for minutes (GPS_CFG),
	// and integrating the whole gap in one Euler step on return would kick
	// the error somewhere absurd — the bench has been bitten by exactly that
	// shape of bug before. A capped step just means the error resumes from
	// where it was, which is what a receiver that was off would do anyway.
	if (dt > 1.0f) {
		dt = 1.0f;
	}
	float a = dt / GPS_TAU_S;
	float sd = sqrtf(2.0f * a) * nzf(nz_gps);
	for (int i = 0; i < 3; i++) {
		float sigma = i == 2 ? GPS_SIGMA_V_M : GPS_SIGMA_H_M;
		gps_err_m[i] += -a * gps_err_m[i] + noise(sigma * sd);
	}
}

// RawAirData, sent UNCONDITIONALLY from the first instant of boot — a
// parked harness reports zero airspeed rather than staying silent. The
// consumer's binding is one-shot: ArduPilot's AP_Airspeed::init() runs
// once (AP_Vehicle.cpp) and AP_Airspeed_DroneCAN::probe() can only bind
// a node whose RawAirData has ALREADY been handled, so a node that
// starts talking later never binds for that boot — measured on the
// bench 2026-09-08, ARSPD_DEVID stuck at 0 across clean boots.
// Rate is 20 Hz because the consumer's staleness windows are 250 ms for
// differential pressure and 100 ms for temperature: at the old 5 Hz the
// temperature was ALWAYS stale and the pressure had 50 ms of margin.
// Unlike Fix2 this is not a deliberately-degraded feed — real pitot
// sensors run fast, so 20 Hz is fidelity, not kindness.
static void gps_feed_air(uint32_t now) {
	const struct GpsSnap *snap = gps_snap(now);
	float ias = snap != NULL ? snap->ias_dms * 0.1f : 0.0f;
	uint8_t buf[64];
	struct DroneCanFrame fr[16];
	int n = dronecan_broadcast(DRONECAN_RAWAIR_ID, DRONECAN_RAWAIR_SIGNATURE, DRONECAN_PRIO_MEDIUM,
	                           42, &gps_tid_air, buf,
	                           dronecan_rawair(0.5f * 1.225f * ias * ias +
                                               noise(PITOT_SIGMA_PA * nzf(nz_pitot)),
                                           0.0f, 288.15f, buf), fr, 16);
	gps_enqueue(fr, n);
}

// Fix2 stays 5 Hz behind the configurable lag: a zero-latency GPS would
// make HITL kinder than reality, and it needs the fdm to have a position.
static void gps_feed(uint32_t now) {
	const struct GpsSnap *snap = gps_snap(now);
	if (snap == NULL) {
		return;
	}
	gps_err_step(now);
	float sh = GPS_SIGMA_H_M * nzf(nz_gps), sv = GPS_SIGMA_V_M * nzf(nz_gps);
	float svel = GPS_SIGMA_VEL * nzf(nz_gps);
	int32_t n_cm = snap->n_cm + (int32_t)(gps_err_m[0] * 100.0f);
	int32_t e_cm = snap->e_cm + (int32_t)(gps_err_m[1] * 100.0f);
	int32_t h_cm = snap->h_cm - (int32_t)(gps_err_m[2] * 100.0f); // D error, h is up
	// Advertise what we are actually doing, floored at the accuracy this feed
	// claimed before it had any error at all: a receiver's reported accuracy
	// is conservative, and an EKF fed a covariance smaller than the true error
	// is being lied to in the direction that makes it overconfident.
	float cov_h = sh * sh > 1.0f ? sh * sh : 1.0f;
	float cov_v = sv * sv > 1.0f ? sv * sv : 1.0f;
	float cov_vel = svel * svel > 0.25f ? svel * svel : 0.25f;
	struct DroneCanFix2 fx = {
		.usec = now,
		.lat_deg_1e8 = gps_lat0_1e8 + (int64_t)n_cm * gps_ncm_num / gps_cm_den,
		.lon_deg_1e8 = gps_lon0_1e8 + (int64_t)e_cm * gps_ecm_num / gps_cm_den,
		.h_mm = h_cm * 10,
		.vned = {snap->v_cms[0] * 0.01f + noise(svel), snap->v_cms[1] * 0.01f + noise(svel),
		         snap->v_cms[2] * 0.01f + noise(svel)},
		.sats = 12,
		.status = 3,
		.cov = {cov_h, cov_h, cov_v, cov_vel, cov_vel, cov_vel},
		.pdop = 1.5f,
	};
	uint8_t buf[64];
	struct DroneCanFrame fr[16];
	int n = dronecan_broadcast(DRONECAN_FIX2_ID, DRONECAN_FIX2_SIGNATURE, DRONECAN_PRIO_MEDIUM,
	                           42, &gps_tid_fix, buf, dronecan_fix2(&fx, buf), fr, 16);
	gps_enqueue(fr, n);
}

// IRQ priorities, 2:2 grouping: level[3:2] = preemption group, level[1:0] =
// subpriority (orders pending IRQs only). Handlers in the SAME group can
// never preempt each other — the spislave engine relies on that for SPI3 vs
// the CS EXTIs (both mutate the bus state; select/deselect vs byte-0 decode
// must serialize).
enum { IRQ_PRIORITY_GROUPING_2_2 = 5 };
#define PRIO(grp, sub) ((grp) << 2 | (sub))
static const struct {
	enum IRQn_Type irq;
	uint8_t prio;
} irqprios[] = {
	{SPI3_IRQn, PRIO(0, 0)},      // byte-0 cmd decode, ~72-cycle deadline
	{EXTI0_IRQn, PRIO(0, 1)}, // CS baro: dummy pre-stuff / deselect scrub
	{EXTI1_IRQn, PRIO(0, 1)}, // CS mag
	{EXTI2_IRQn, PRIO(0, 1)}, // CS gyro
	{EXTI3_IRQn, PRIO(0, 1)}, // CS accel

	{TIM2_IRQn, PRIO(1, 0)}, // PWM capture ch1-4 (µs timestamps)
	{TIM3_IRQn, PRIO(1, 0)}, // PWM capture ch5-8

	{FDCAN1_IT0_IRQn, PRIO(1, 1)}, // TX events, bus-off
	{FDCAN1_IT1_IRQn, PRIO(1, 1)}, // RX: command mailboxes
	{FDCAN3_IT0_IRQn, PRIO(1, 1)}, // GPS feed: TX events, bus-off recovery
	{FDCAN3_IT1_IRQn, PRIO(1, 1)}, // GPS feed: RX drain (DUT traffic, dropped)

	{TIM7_DAC2_4_IRQn, PRIO(2, 1)}, // 10 kHz scheduler tick (counter only)

	{USB_LP_IRQn, PRIO(2, 1)}, // usb: protocol + pseudocan rx bytes

	{DMA1_CH1_IRQn, PRIO(2, 0)}, // console TX DMA
	{DMA1_CH2_IRQn, PRIO(2, 0)}, // console RX DMA
	{USART1_IRQn, PRIO(2, 0)},   // console TX kick / RX idle flush
};
#undef PRIO

// ---- the 10 kHz scheduler tick ----------------------------------------------
static volatile uint32_t tim7_ticks;

// DWT cycle counter: measure the physics step on the real core (heartbeat
// reports the max per interval — the flight branch costs more than bench)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define DCB_DEMCR (*(volatile uint32_t *)0xE000EDFC)
static uint32_t phys_cycles_max;

static int16_t sat16(float x) {
	if (x > 32767.0f) {
		return 32767;
	}
	if (x < -32767.0f) {
		return -32767;
	}
	return (int16_t)x;
}

// The gyro's nastiest error is not the white noise: it is a per-boot
// turn-on bias plus an in-run bias that random-walks in 3D — the whole
// reason attitude filters carry gyro-bias states and lean on accel+mag
// as the low-bandwidth reference. Model both (BMI088-class numbers:
// turn-on sigma 0.15 deg/s, walk 0.002 deg/s per sqrt(s)). The accel
// gets a small fixed turn-on bias so those filter states work too.
static float gyro_bias[3], accel_bias[3];
static uint8_t imu_bias_init;

// pull the physics truth, quantize per the live configs, commit
static void sample_gyro(uint32_t now) {
	const struct PhysicsTruth *t = truth();
	if (!imu_bias_init) {
		imu_bias_init = 1;
		for (int i = 0; i < 3; i++) {
			gyro_bias[i] = noise(0.15f * nzf(nz_bias));  // deg/s, fixed for this boot
			accel_bias[i] = noise(0.05f * nzf(nz_bias)); // m/s^2
		}
	}
	float srw = 0.002f * sqrtf(1.0f / (float)gyro_rate_hz()); // walk step at this ODR
	float lsb = 32767.0f / gyro_fullscale_dps(); // counts per deg/s
	int16_t xyz[3];
	for (int i = 0; i < 3; i++) {
		gyro_bias[i] += noise(srw * nzf(nz_bias));
		xyz[i] = sat16((t->rate[i] * (180.0f / (float)M_PI) + gyro_bias[i] +
		                noise(0.2f * nzf(nz_gyro))) * lsb);
	}
	gyro_commit(xyz, now);
}

static void sample_accel(uint32_t now) {
	const struct PhysicsTruth *t = truth();
	float lsb = 32767.0f / (accel_fullscale_g() * PHYSICS_G); // counts per m/s^2
	int16_t xyz[3];
	for (int i = 0; i < 3; i++) {
		xyz[i] = sat16((t->sforce[i] + accel_bias[i] + noise(0.05f * nzf(nz_accel))) * lsb);
	}
	accel_commit(xyz, t->t_degc, now);
}

static void sample_mag(uint32_t now) {
	const struct PhysicsTruth *t = truth();
	float lsb = mag_lsb_per_ut();
	int32_t xyz[3];
	for (int i = 0; i < 3; i++) {
		xyz[i] = (int32_t)((t->mag[i] + noise(0.02f * nzf(nz_mag))) * lsb);
	}
	mag_commit(xyz, now);
}

static void sample_baro(uint32_t now) {
	const struct PhysicsTruth *t = truth();
	// The dither is MANDATORY, not optional: ArduPilot's stuck-baro detector
	// and PX4's sensors-module DataValidator both declare a bit-identical
	// pressure stream unhealthy — a perfectly noise-free baro reads as
	// broken. Hence the floor on nz_baro in cmd_decode: this knob turns up,
	// never off. Sigma 0.58 Pa is RMS-matched to the +-1 Pa uniform dither
	// this replaced (BMP388-class noise).
	baro_commit(t->t_degc, t->p_pa + noise(0.58f * nzf(nz_baro)), now);
}

// ---- host command decode (big-endian, canmsg.h dictionary) -------------------

static bool cmd_snapshot(struct CmdBox *b, uint8_t p[8], uint32_t *seq) {
	// seq-stable copy against the RX irq; bounded — under a command storm
	// report no-news this slot rather than spin, the next 1 kHz slot retries
	for (int try = 0; try < 8; try++) {
		uint32_t s = b->seq;
		for (int i = 0; i < 8; i++) {
			p[i] = b->data[i];
		}
		if (b->seq != s) {
			continue;
		}
		if (s == *seq || b->len < 8) {
			return false;
		}
		*seq = s;
		return true;
	}
	return false;
}

static void cmd_decode(void) {
	static uint32_t seq_state, seq_env;
	uint8_t p[8];
	if (cmd_snapshot(&cmd_state, p, &seq_state)) {
		// V cm/s i16, hdot cm/s i16, psidot mrad/s i16, flags u16
		physics_cmd((int16_t)decode_be_uint16(p) * 0.01f,
		            (int16_t)decode_be_uint16(p + 2) * 0.01f,
		            (int16_t)decode_be_uint16(p + 4) * 0.001f);
	}
	if (cmd_snapshot(&cmd_env, p, &seq_env)) {
		// QNH Pa/10 u16, T0 0.1 K u16, B 0.01 uT u16, incl 0.01 deg i16
		physics_env(decode_be_uint16(p) * 10.0f,
		            decode_be_uint16(p + 2) * 0.1f,
		            decode_be_uint16(p + 4) * 0.01f,
		            (int16_t)decode_be_uint16(p + 6) * 0.01f * ((float)M_PI / 180.0f));
	}
	static uint32_t seq_fmode, seq_finit;
	if (cmd_snapshot(&cmd_fdm_init, p, &seq_finit)) {
		// alt m u16, IAS 0.1 m/s u16, heading 0.01 deg u16: trim & reset.
		// The solver is iterative — fine here at thread level, not in an irq.
		if (fdm_trim(&fdm, (float)decode_be_uint16(p),
		             decode_be_uint16(p + 2) * 0.1f,
		             decode_be_uint16(p + 4) * 0.01f * ((float)M_PI / 180.0f),
		             &fdm_trim_ctl) == 0) {
			fdm_ctl = fdm_trim_ctl;
			fdm_crashes = 0; // a successful air-start leaves the craters behind
		}
	}
	static uint32_t seq_fpos;
	if (cmd_snapshot(&cmd_fdm_pos, p, &seq_fpos)) {
		// bench rehome: N/E teleport only — alt/attitude/energy stay; pair
		// with FDM_INIT for a full air-start at the new position
		fdm.pos_cm[0] = (int32_t)decode_be_uint32(p);
		fdm.pos_cm[1] = (int32_t)decode_be_uint32(p + 4);
		fdm.pos_rem[0] = fdm.pos_rem[1] = 0.0f;
	}
	static uint32_t seq_pcal;
	if (cmd_snapshot(&cmd_pwm_cal, p, &seq_pcal)) {
		controls_cal_msg(p);
	}
	static uint32_t seq_gps;
	if (cmd_snapshot(&cmd_gps_cfg, p, &seq_gps)) {
		gps_enable = p[0];
		gps_lag_10ms = p[1];
	}
	static uint32_t seq_noise;
	if (cmd_snapshot(&cmd_noise, p, &seq_noise)) {
		// full-state frame: u8 gyro/accel/mag/baro white-noise scale, u8 bias
		// scale, u8 flags, all in 1/16 of the datasheet default
		nz_gyro = p[0];
		nz_accel = p[1];
		nz_mag = p[2];
		nz_baro = p[3] < 16 ? 16 : p[3]; // the dither is mandatory: sample_baro
		nz_bias = p[4];
		nz_pitot = p[6];
		nz_gps = p[7];
		if (p[5] & 2) { // zero the turn-on biases and the accumulated walk
			for (int i = 0; i < 3; i++) {
				gyro_bias[i] = accel_bias[i] = 0.0f;
			}
			imu_bias_init = 1;
		} else if (p[5] & 1) { // re-draw them: sample_gyro's lazy init does it
			imu_bias_init = 0;
		}
	}
	static uint32_t seq_wind, seq_param;
	if (cmd_snapshot(&cmd_wind, p, &seq_wind)) {
		// i16 N/E/D cm/s, u8 gust sigma cm/s, u8 gust tau s
		for (int i = 0; i < 3; i++) {
			fdm.p.wind_n[i] = (int16_t)decode_be_uint16(p + 2 * i) * 0.01f;
		}
		fdm.p.gust_sigma = p[6] * 0.01f;
		fdm.p.gust_tau = (float)p[7];
	}
	if (cmd_snapshot(&cmd_param, p, &seq_param)) {
		unsigned idx = decode_be_uint16(p);
		if (idx & 0x8000) { // read request: reply with PARAM_VAL
			idx &= 0x7fff;
			uint8_t q[8] = {0};
			encode_be_uint16(q, (uint16_t)idx);
			encode_be_float32(q + 2, fdm_param_get(&fdm, idx));
			can_send(CANMSG_PARAM_VAL, q, 8);
		} else {
			union { uint32_t i; float f; } v = {.i = decode_be_uint32(p + 2)};
			fdm_param_set(&fdm, idx, v.f);
		}
	}
	if (cmd_snapshot(&cmd_fdm_mode, p, &seq_fmode)) {
		fdm_mode = p[0] & 1;
	}
}

void Reset_Handler(void) __attribute__((noreturn));
void Reset_Handler(void) {
	startup_init_memory();
	startup_remap0(); // RAM run model: SRAM1 to 0x0, fetches go zero-wait
	SCB.VTOR = (uint32_t)(uintptr_t)__vectors;
	*(volatile uint32_t *)0xE000ED88 |= 0xfu << 20; // FPU: CP10/CP11 full access
	SCB.SHCSR |= SCB_SHCSR_USGFAULTENA;             // route usage faults to our handler
	SCB.CCR |= SCB_CCR_DIV_0_TRP;                   // div-by-zero -> UsageFault

	board_init(); // clock + peripheral clocks + the whole pinout

	nvic_set_priority_grouping(IRQ_PRIORITY_GROUPING_2_2);
	for (size_t i = 0; i < sizeof irqprios / sizeof irqprios[0]; i++) {
		nvic_set_priority(irqprios[i].irq, irqprios[i].prio);
	}

	dma_set_mux(DMA1_CH1, DMA_REQ_USART1_TX);
	dma_set_mux(DMA1_CH2, DMA_REQ_USART1_RX);
	usart_init(&USART1, clock_usart_hz(1), 115200); // full duplex + RX timeout flush
	console = &vcp;
	serial_dma_rx_start(&vcp_rx, DMA1_CH2);
	nvic_enable(DMA1_CH1_IRQn);
	nvic_enable(DMA1_CH2_IRQn);
	nvic_enable(USART1_IRQn);

	pwmin_init(&cap2, clock_pclk1_timer_hz());
	pwmin_init(&cap3, clock_pclk1_timer_hz());
	nvic_enable(TIM2_IRQn);
	nvic_enable(TIM3_IRQn);

	srcid = canmsg_srcid_self();

	// the on-board DroneCAN GPS feed rides FDCAN3 regardless of transport
	fdcan_init(&can_gps, clock_fdcan_hz(), 1000000);
	nvic_enable(FDCAN3_IT0_IRQn);
	nvic_enable(FDCAN3_IT1_IRQn);

#ifdef TRANSPORT_CAN
	fdcan_init(&can1, clock_fdcan_hz(), 1000000);
	nvic_enable(FDCAN1_IT0_IRQn);
	nvic_enable(FDCAN1_IT1_IRQn);
#else
	usb_init("rotanimb01", "hitl-harness"); // board.c brought up CLK48 + CRS
	nvic_enable(USB_LP_IRQn);
#endif

	// report a crash from the previous run BEFORE re-entering the bring-up
	// that may have caused it (a boot-time assert would reset-loop silently
	// if this ran after)
	fault_report(cputc);

	// the SPI3 sensor bus: four register-file devices, CS demux on EXTI
	dma_set_mux(DMA1_CH5, DMA_REQ_SPI3_TX);
	sensors_init();
	exti_init(CS_BARO | CS_MAG | CS_GYRO | CS_ACC, true, true);
	nvic_enable(SPI3_IRQn);
	nvic_enable(EXTI0_IRQn);
	nvic_enable(EXTI1_IRQn);
	nvic_enable(EXTI2_IRQn);
	nvic_enable(EXTI3_IRQn);

	DCB_DEMCR |= 1u << 24; // TRCENA
	DWT_CTRL |= 1u;        // CYCCNTENA

	// the 10 kHz scheduler tick
	physics_init();
	fdm_defaults(&fdm); // mode 1 idles at defaults until FDM_INIT air-starts it
	controls_defaults();
	RCC.APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	TIM7.PSC = (uint16_t)(clock_pclk1_timer_hz() / 1000000 - 1); // 1 MHz
	TIM7.ARR = 100 - 1;                                          // 10 kHz
	TIM7.DIER = TIM_BASIC_INST_DIER_UIE;
	TIM7.CR1 = TIM_BASIC_INST_CR1_CEN;
	nvic_enable(TIM7_DAC2_4_IRQn);

	tprintf("\nrotanimb01 HITL harness on STM32G474, sysclk = %u Hz, hse = %u Hz, srcid %02x, transport %s\n",
	        (unsigned)clock_sysclk_hz(), (unsigned)clock_hse_hz, srcid,
#ifdef TRANSPORT_CAN
	        "can"
#else
	        "usb pseudocan"
#endif
	);

	// Report state per PWM channel: last sent width and the staleness tracker.
	struct {
		uint16_t sent_w;
		uint32_t count;
		uint32_t change_us; // last time count advanced
	} chan[8] = {0};

	uint32_t unexp_last = 0, desyncs = 0;
	bool desync = false;
	uint32_t t_pwm = now_us(), t_status = t_pwm, t_diag = t_pwm, t_tick = t_pwm, t_truth = t_pwm,
			 t_gpsfeed = t_pwm, t_airfeed = t_pwm, t_nodest = t_pwm;
	for (;;) {
		// console RX echo (link sanity, M0 heritage)
		uint8_t chunk[64];
		size_t n = fifo_read(&vcp_rx.buf, chunk, sizeof chunk);
		if (n) {
			fifo_write(&vcp.buf, chunk, n);
			serial_dma_tx_start(&vcp);
		}

		uint32_t now = now_us();
		sensors_poll(now);

#ifndef TRANSPORT_CAN
		// pseudocan lines from the host -> the command mailboxes
		for (;;) {
			unsigned pport;
			uint32_t header;
			size_t plen;
			uint8_t pbuf[8];
			int r = can_fifo_get(&usb_rx, &pport, &header, &plen, pbuf);
			if (r == 0) {
				break;
			}
			if (r < 0) {
				usb_bad++;
				continue;
			}
			if (can_header_isext(header)) {
				host_msg(can_header_to29(header), pbuf, plen);
			}
		}
		if (usb_dtr()) {
			usb_send(&usb_tx);
		} else {
			fifo_reset(&usb_tx); // nobody listening: don't accumulate
		}
#endif
		cmd_decode();

		// the table scheduler: consume 10 kHz ticks; physics every 5th
		// (2 kHz), Bresenham accumulator per device for any configured ODR
		{
			static uint32_t done;
			static uint32_t phys_div, fdm_div, acc_g, acc_a, acc_b, acc_m;
			while (done != tim7_ticks) {
				done++;
				if (++phys_div >= 5) {
					phys_div = 0;
					if (!fdm_mode) {
						uint32_t c0 = DWT_CYCCNT;
						physics_step(5 * 100e-6f);
						uint32_t c = DWT_CYCCNT - c0;
						if (c > phys_cycles_max) {
							phys_cycles_max = c;
						}
					}
				}
				if (++fdm_div >= 10) { // the 6-DOF at 1 kHz, fixed dt (fdm-DESIGN.md)
					fdm_div = 0;
					if (fdm_mode) {
						uint32_t c0 = DWT_CYCCNT;
						controls_step(&cap2, now, 1e-3f, &fdm_trim_ctl, &fdm_ctl);
						fdm_step(&fdm, &fdm_ctl, 1e-3f);
						if (fdm.truth.h < 0.0f) {
							// harsh contact: count it and re-park the wreck on
							// its gear, level, yaw kept — the sim stays alive
							// (a frozen impact state poisons every ground cal
							// the DUT runs; learned the hard way, twice)
							fdm_crashes++;
							const float *q = fdm.truth.quat;
							float psi = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
							                   1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
							float ch = cosf(0.5f * psi), sh = sinf(0.5f * psi);
							fdm.quat[0] = ch;
							fdm.quat[1] = 0.0f;
							fdm.quat[2] = 0.0f;
							fdm.quat[3] = sh;
							for (int i = 0; i < 3; i++) {
								fdm.v_b[i] = 0.0f;
								fdm.w_b[i] = 0.0f;
							}
							fdm.pos_cm[2] = 0;
							fdm.pos_rem[2] = 0.0f;
							fdm.on_ground = 1;
							fdm_step(&fdm, &fdm_ctl, 1e-3f); // rebuild truth parked
						}
						fdm_publish_truth();
						uint32_t c = DWT_CYCCNT - c0;
						if (c > phys_cycles_max) {
							phys_cycles_max = c;
						}
					}
				}
				uint32_t hz;
				if ((hz = gyro_rate_hz()) != 0 && (acc_g += hz) >= 10000) {
					acc_g -= 10000;
					sample_gyro(now);
				}
				if ((hz = accel_rate_hz()) != 0 && (acc_a += hz) >= 10000) {
					acc_a -= 10000;
					sample_accel(now);
				}
				if ((hz = baro_rate_hz()) != 0 && (acc_b += hz) >= 10000) {
					acc_b -= 10000;
					sample_baro(now);
				}
				if ((hz = mag_rate_hz()) != 0 && (acc_m += hz) >= 10000) {
					acc_m -= 10000;
					sample_mag(now);
				}
			}
		}

		// gather widths; a channel with no completed period for 100 ms reads 0
		uint16_t w[8];
		bool change = false;
		for (int i = 0; i < 8; i++) {
			struct PWMInSample s = pwmin_get(i < 4 ? &cap2 : &cap3, i & 3);
			if (s.count != chan[i].count) {
				chan[i].count = s.count;
				chan[i].change_us = now;
			}
			w[i] = (now - chan[i].change_us > 100000) ? 0 : (uint16_t)s.width_us;
			int d = (int)w[i] - (int)chan[i].sent_w;
			change |= d > 2 || d < -2;
		}
		if (change || (int32_t)(now - t_pwm) >= 20000) { // 50 Hz + on change
			t_pwm = now;
			uint8_t p[8];
			for (int i = 0; i < 4; i++) {
				encode_be_uint16(p + 2 * i, w[i]);
			}
			can_send(CANMSG_PWM14, p, 8);
			for (int i = 0; i < 4; i++) {
				encode_be_uint16(p + 2 * i, w[4 + i]);
			}
			can_send(CANMSG_PWM58, p, 8);
			for (int i = 0; i < 8; i++) {
				chan[i].sent_w = w[i];
			}
		}

		// unsigned deltas: with (int32_t), a timer first armed after ~35.8 min
		// of uptime saw now - 0 wrap negative and never fired — fdm enabled
		// late produced no TRUTH_* and no GPS feed (found 2026-08-07)
		// air data BEFORE Fix2: when both fall due in the same tick the
		// 11-frame Fix2 burst would otherwise sit ahead of it in the queue
		if (gps_enable && now - t_airfeed >= 50000) { // RawAirData 20 Hz
			t_airfeed = now;
			gps_feed_air(now);
		}
		if (gps_enable && fdm_mode && now - t_gpsfeed >= 200000) { // Fix2 5 Hz
			t_gpsfeed = now;
			gps_feed(now);
		}
		if (gps_enable && now - t_nodest >= 1000000) { // NodeStatus 1 Hz
			t_nodest = now;
			uint8_t buf[8];
			struct DroneCanFrame fr[2];
			int n = dronecan_broadcast(DRONECAN_NODESTATUS_ID, DRONECAN_NODESTATUS_SIGNATURE,
			                           DRONECAN_PRIO_LOW, 42, &gps_tid_ns, buf,
			                           dronecan_nodestatus(now / 1000000u, buf), fr, 2);
			gps_enqueue(fr, n);
		}

		if (fdm_mode && now - t_truth >= 50000) { // TRUTH_* 20 Hz
			t_truth = now;
			const struct FdmTruth *ft = &fdm.truth;
			uint8_t p[8];

			encode_be_uint32(p, (uint32_t)(int32_t)(ft->h * 100.0f));
			encode_be_uint16(p + 4, (uint16_t)sat16(-ft->v_ned[2] * 100.0f));
			encode_be_uint16(p + 6,
			                 (uint16_t)sat16(sqrtf(ft->v_ned[0] * ft->v_ned[0] +
			                                       ft->v_ned[1] * ft->v_ned[1]) * 100.0f));
			can_send(CANMSG_TRUTH_POSVEL, p, 8);

			for (int i = 0; i < 4; i++) {
				encode_be_uint16(p + 2 * i, (uint16_t)sat16(ft->quat[i] * 32767.0f));
			}
			can_send(CANMSG_TRUTH_ATT, p, 8);

			float ias = sqrtf(2.0f * ft->qbar / 1.225f); // sea-level-density IAS
			const float r2cd = 18000.0f / (float)M_PI;
			encode_be_uint16(p, (uint16_t)sat16(ias * 10.0f));
			encode_be_uint16(p + 2, (uint16_t)sat16(ft->va * 10.0f));
			encode_be_uint16(p + 4, (uint16_t)sat16(ft->alpha * r2cd));
			encode_be_uint16(p + 6, (uint16_t)sat16(ft->beta * r2cd));
			can_send(CANMSG_TRUTH_AIR, p, 8);

			const struct FdmControls *cc = controls_state();
			encode_be_uint16(p, (uint16_t)sat16(cc->da * r2cd));
			encode_be_uint16(p + 2, (uint16_t)sat16(cc->de * r2cd));
			encode_be_uint16(p + 4, (uint16_t)sat16(cc->dr * r2cd));
			encode_be_uint16(p + 6, (uint16_t)(cc->dt * 1000.0f));
			can_send(CANMSG_TRUTH_CTRL, p, 8);

			encode_be_uint32(p, (uint32_t)ft->pos_cm[0]);
			encode_be_uint32(p + 4, (uint32_t)ft->pos_cm[1]);
			can_send(CANMSG_TRUTH_POS, p, 8);

			encode_be_uint16(p, (uint16_t)sat16(ft->v_ned[0] * 100.0f));
			encode_be_uint16(p + 2, (uint16_t)sat16(ft->v_ned[1] * 100.0f));
			encode_be_uint16(p + 4, (uint16_t)sat16(ft->v_ned[2] * 100.0f));
			encode_be_uint16(p + 6, 0);
			can_send(CANMSG_TRUTH_VEL, p, 8);

			struct GpsSnap *g = &gps_ring[gps_head++ & 7];
			g->n_cm = ft->pos_cm[0];
			g->e_cm = ft->pos_cm[1];
			g->h_cm = (int32_t)(ft->h * 100.0f);
			for (int i = 0; i < 3; i++) {
				g->v_cms[i] = sat16(ft->v_ned[i] * 100.0f);
			}
			g->ias_dms = (uint16_t)sat16(sqrtf(2.0f * ft->qbar / 1.225f) * 10.0f);
			g->us = now ? now : 1;
		}

		if ((int32_t)(now - t_status) >= 100000) { // STATUS 10 Hz
			t_status = now;

			// SPI-slave desync watchdog. The counter semantics make this a
			// detector rather than a heuristic: `unexpected` counts writes
			// refused by the write mask, which is exactly what a shifted
			// command byte produces, and a healthy bus sits at a hard zero
			// indefinitely. Measured: healthy 0, desynced ~1800 per 100 ms.
			// DETECTION ONLY. An in-place re-arm of the engine was tried here
			// and DOES NOT WORK: bench-tested 2026-09-12, the resync fired
			// once a second for 40 s while unexp climbed past 1.6 M unabated.
			// The cure remains a DUT reset followed by a harness reset ~2 s
			// later. See the README known issue.
			uint32_t unexp_now = sensors_unexpected();
			uint32_t unexp_rate = unexp_now - unexp_last;
			unexp_last = unexp_now;
			if (unexp_rate > 8) { // far above healthy-zero, far below a fault
				if (!desync) {
					desyncs++; // rising edge: count events, not windows
				}
				desync = true;
			} else if (unexp_rate == 0) {
				desync = false; // quiet again
			}

			bool stale = cmd_state.seq == 0 || now - cmd_state.rx_us > 1000000;
			float psi_deg = truth()->psi * (180.0f / (float)M_PI);
			if (psi_deg < 0) {
				psi_deg += 360.0f;
			}
			uint8_t p[8];
			encode_be_uint32(p, now);
			encode_be_uint16(p + 4, (uint16_t)(psi_deg * 100.0f)); // psi 0.01 deg
			encode_be_uint16(p + 6, (uint16_t)((stale ? 1 : 0) | physics_flags() |
			                                   (fdm_mode ? controls_flags() : 0) |
			                                   (fdm_crashes ? 1 << 5 : 0) |
			                                   (desync ? 1 << 6 : 0)));
			can_send(CANMSG_STATUS, p, 8);
		}

		if ((int32_t)(now - t_diag) >= 1000000) { // DIAG 1 Hz
			t_diag = now;
			uint32_t errs = 0;
			for (int i = 0; i < 4; i++) {
				errs += cap2.ch[i].errs + cap3.ch[i].errs;
			}
			uint8_t p[8];
#ifdef TRANSPORT_CAN
			encode_be_uint16(p, (uint16_t)can1.status.tx_count);
			encode_be_uint16(p + 2, (uint16_t)can1.status.rx_count[0]);
			encode_be_uint16(p + 4, (uint16_t)(can1.status.rx_ovfl[0] + can1.status.rx_ovfl[1]));
#else
			encode_be_uint16(p, 0); // link tx/rx counts live in the usb layer
			encode_be_uint16(p + 2, (uint16_t)usb_bad);
			encode_be_uint16(p + 4, (uint16_t)usb_drop);
#endif
			encode_be_uint16(p + 6, (uint16_t)errs);
			can_send(CANMSG_DIAG, p, 8);
		}

		if ((int32_t)(now - t_tick) >= 1000000) { // console heartbeat 1 Hz
			t_tick = now;
			digitalToggle(LED);
			const struct PhysicsTruth *pt = truth();
			uint32_t pc = phys_cycles_max;
			phys_cycles_max = 0;
			tprintf("t %u us pwm %u %u %u %u %u %u %u %u psi %d cdeg h %d cm p %u Pa fdm %u%s phys %u cy spi g/a/b/m %u/%u/%u/%u unexp %u stray %u mid %u desync %u cmd seq %u nz %u/%u/%u/%u/%u/%u/%u ",
			        (unsigned)now, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
			        (int)(pt->psi * (18000.0f / (float)M_PI)), (int)(pt->h * 100.0f),
			        (unsigned)pt->p_pa, (unsigned)fdm_mode,
			        fdm.on_ground ? "g" : "a", (unsigned)pc,
			        (unsigned)gyro_dev.frames, (unsigned)accel_dev.frames,
			        (unsigned)baro_dev.frames, (unsigned)mag_dev.frames,
			        (unsigned)(gyro_dev.unexpected + accel_dev.unexpected + baro_dev.unexpected + mag_dev.unexpected),
			        (unsigned)sensor_bus.stray, (unsigned)sensor_bus.midframe, (unsigned)desyncs,
			        (unsigned)cmd_state.seq,
			        nz_gyro, nz_accel, nz_mag, nz_baro, nz_bias, nz_pitot, nz_gps);
			tprintf("gps tx %u bo %u lec %u/%u/%u/%u/%u/%u/%u ",
			        (unsigned)can_gps.status.tx_count, (unsigned)gps_busoff,
			        (unsigned)can_gps.status.lec_count[1], (unsigned)can_gps.status.lec_count[2],
			        (unsigned)can_gps.status.lec_count[3], (unsigned)can_gps.status.lec_count[4],
			        (unsigned)can_gps.status.lec_count[5], (unsigned)can_gps.status.lec_count[6],
			        (unsigned)can_gps.status.lec_count[7]);
#ifdef TRANSPORT_CAN
			tprintf("can tx %u rx %u lec %u/%u/%u/%u/%u/%u/%u\n",
			        (unsigned)can1.status.tx_count, (unsigned)can1.status.rx_count[0],
			        (unsigned)can1.status.lec_count[1], (unsigned)can1.status.lec_count[2],
			        (unsigned)can1.status.lec_count[3], (unsigned)can1.status.lec_count[4],
			        (unsigned)can1.status.lec_count[5], (unsigned)can1.status.lec_count[6],
			        (unsigned)can1.status.lec_count[7]);
#else
			tprintf("usb %s bad %u drop %u\n", usb_state_str(usb_state()),
			        (unsigned)usb_bad, (unsigned)usb_drop);
#endif
			if (fdm_mode) {
				const struct FdmControls *c = controls_state();
				const float r2cd = 18000.0f / (float)M_PI;
				tprintf("ctl a %d e %d r %d cdeg t %d%% fs %04u%s\n",
				        (int)(c->da * r2cd), (int)(c->de * r2cd), (int)(c->dr * r2cd),
				        (int)(c->dt * 100.0f), controls_fs_code(),
				        fdm_crashes ? " CRASHED" : "");
			}
		}
	}
}

// Serial interrupts (the app owns the vectors and calls the lib handlers).
static void dma1_ch1(void) { serial_dma_tx_handler(&vcp, DMA1_CH1); }
static void dma1_ch2(void) { serial_dma_rx_handler(&vcp_rx, DMA1_CH2); }
static void usart1(void) {
	serial_irq_tx_handler(&vcp, DMA1_CH1);
	serial_irq_rx_handler(&vcp_rx, DMA1_CH2, &vcp_rxc);
}

static void tim2(void) { pwmin_irq_handler(&cap2); }
static void tim3(void) { pwmin_irq_handler(&cap3); }
static void tim7(void) {
	TIM7.SR = 0; // w0c UIF
	tim7_ticks++;
}

// the sensor bus: byte-0 deadline path inlined with constant instances
static void spi3(void) { spislave_irq(&sensor_bus, &SPI3, DMA1_CH5); }
static void sensor_cs(void) { spislave_cs_handler(&sensor_bus); }

// FDCAN3, the GPS feeder — every transport. IT0: tx events + errors —
// drain the event fifo (updates counters).
static void fdcan3_it0(void) {
	uint8_t tag;
	uint32_t header;
	uint16_t ts;
	for (int n = 0; n < 8 && fdcan_tx_done(&can_gps, &tag, &header, &ts) >= 0; n++) {
	}
	gps_pump(); // hw buffers just freed (or nvic_set_pending kick): drain the queue
	uint32_t ir = FDCAN3.IR;
	if (ir & FDCAN_IR_BO) {
		FDCAN3.CCCR &= ~FDCAN_CCCR_INIT; // bus-off recovery: rejoin after 129 idles
		gps_busoff++;
	}
	FDCAN3.IR = ir & ~(FDCAN_IR_RF0N | FDCAN_IR_RF1N);
}

static void fdcan3_it1(void) { // the DUT's own traffic: drain and drop
	FDCAN3.IR = FDCAN_IR_RF0N | FDCAN_IR_RF1N; // w1c FIRST — a latched flag
	// left set re-pends this irq forever (the 100%-CPU storm of 2026-07-15)
	uint8_t fmi, p[8];
	uint32_t header;
	size_t len;
	uint16_t ts;
	// bounded per the aerospace rule: a sick fifo must not own the CPU;
	// leftovers wait for the next frame's RF1N
	for (int n = 0; n < 8 && fdcan_rx(&can_gps, &fmi, &header, &len, p, &ts) >= 0; n++) {
	}
}

#ifdef TRANSPORT_CAN
static void fdcan1_it0(void) {
	uint8_t tag;
	uint32_t header;
	uint16_t ts;
	for (int n = 0; n < 8 && fdcan_tx_done(&can1, &tag, &header, &ts) >= 0; n++) {
	}
	FDCAN1.IR = FDCAN1.IR & ~(FDCAN_IR_RF0N | FDCAN_IR_RF1N); // w1c all but the RX flags
}

// FDCAN1 IT1: rx fifos — pop everything, latch known commands.
static void fdcan1_it1(void) {
	FDCAN1.IR = FDCAN_IR_RF0N | FDCAN_IR_RF1N; // w1c
	uint8_t fmi, p[8];
	uint32_t header;
	uint16_t ts;
	size_t len = sizeof p;
	for (int n = 0; n < 8 && fdcan_rx(&can1, &fmi, &header, &len, p, &ts) >= 0; n++) {
		if (can_header_isext(header)) {
			host_msg(can_header_to29(header), p, len);
		}
		len = sizeof p;
	}
}
#else
static void usb_lp(void) { usb_recv(&usb_rx); }
#endif

extern void _estack(void); // top of stack (linker)

// The vector table IS the program: slot 0 = SP, 1 = Reset (positional), 3-6 the
// core fault handlers, device IRQs by VECTOR(IRQn). Unwired slots stay NULL and
// fault loudly if ever taken.
__attribute__((section(".isr_vector"))) const isr_t __vectors[NVIC_VECTORS] = {
	(isr_t)&_estack, // [0] SP    — positional
	Reset_Handler,   // [1] Reset — positional, no +16
	[VECTOR(HardFault_IRQn)] = HardFault_Handler,
	[VECTOR(MemManage_IRQn)] = MemManage_Handler,
	[VECTOR(BusFault_IRQn)] = BusFault_Handler,
	[VECTOR(UsageFault_IRQn)] = UsageFault_Handler,
	[VECTOR(DMA1_CH1_IRQn)] = dma1_ch1,
	[VECTOR(DMA1_CH2_IRQn)] = dma1_ch2,
	[VECTOR(USART1_IRQn)] = usart1,
	[VECTOR(TIM2_IRQn)] = tim2,
	[VECTOR(TIM3_IRQn)] = tim3,
	[VECTOR(TIM7_DAC2_4_IRQn)] = tim7,
	[VECTOR(SPI3_IRQn)] = spi3,
	[VECTOR(EXTI0_IRQn)] = sensor_cs,
	[VECTOR(EXTI1_IRQn)] = sensor_cs,
	[VECTOR(EXTI2_IRQn)] = sensor_cs,
	[VECTOR(EXTI3_IRQn)] = sensor_cs,
	// the FDCAN3 GPS feeder runs in every transport
	[VECTOR(FDCAN3_IT0_IRQn)] = fdcan3_it0,
	[VECTOR(FDCAN3_IT1_IRQn)] = fdcan3_it1,
#ifdef TRANSPORT_CAN
	[VECTOR(FDCAN1_IT0_IRQn)] = fdcan1_it0,
	[VECTOR(FDCAN1_IT1_IRQn)] = fdcan1_it1,
#else
	[VECTOR(USB_LP_IRQn)] = usb_lp,
#endif
};
