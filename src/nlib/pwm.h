#pragma once

// pwm — PWM on general-purpose timers, both directions, at a fixed 1 µs
// resolution: N-channel input measurement (pulse width + period; struct
// PWMIn) and N-channel output generation (struct PWMOut).
//
// ---- input capture ----------------------------------------------------------
//
// Each channel captures on BOTH edges (CCxP|CCxNP); the IRQ handler reads
// CCRx and classifies the edge by the pin's live level: rising->rising is the
// period, rising->falling the width. Protocol-agnostic pulse measurement:
// standard 50 Hz servo PWM and oneshot work; DShot needs a different capture
// strategy and is out of scope.
//
// Resolution is fixed at 1 µs (the timer ticks at 1 MHz):
//  - a 16-bit timer (TIM3/TIM4) then wraps at 65.5 ms, so a 20 ms servo frame
//    is a single unsigned diff; at 0.1 µs it would wrap mid-frame.
//  - 1 µs on a 1-2 ms servo pulse is 0.05-0.1 %, an order finer than the
//    consumers (ArduPilot u16-µs reports; the sync-PLL averages ~10 s of
//    frames, so its frequency resolution is set by the window, not the tick).
// Measurable range per channel: pulse and period from ~5 µs (IRQ latency
// bound, plus a ~100 ns input glitch filter) up to the counter wrap
// (16-bit: 65.5 ms; 32-bit TIM2/TIM5: 71.6 min).
//
// The application owns the interrupt vectors (the lib model) and the
// staleness policy: `count` advances once per completed period measurement,
// so a reader polling at its own rate detects a disconnected channel as
// `count` standing still (e.g. unchanged for 100 ms -> report width 0).
//
// Bring-up sketch (eight RC-servo channels on TIM2 + TIM3):
//   static struct PWMIn cap2 = PWMIN_INITIALIZER_32(TIM2, PA0, PA1, PB10, PB11);
//   static struct PWMIn cap3 = PWMIN_INITIALIZER(TIM3, PC6, PC7, PC8, PC9);
//   ...
//   RCC.APB1ENR1 |= RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM3EN;   // app owns clocks
//   gpioConfigAll(board, COUNT(board));    // PA0_TIM2_CH1 | PIN_PULLDOWN, ...
//   pwmin_init(&cap2, clock_pclk1_timer_hz());
//   pwmin_init(&cap3, clock_pclk1_timer_hz());
//   // in vectors: [VECTOR(TIM2_IRQn)] = tim2;  with
//   //             static void tim2(void) { pwmin_irq_handler(&cap2); }
//   nvic_enable(TIM2_IRQn); nvic_enable(TIM3_IRQn);
//   ...
//   struct PWMInSample s = pwmin_get(&cap2, 0);  // coherent {width, period, count}

#include "device.h"
#include "pinmux.h"

struct PWMInChan {
	const enum GPIO_Pin pin; // capture input, read to classify the edge; 0 = channel unused
	uint32_t rise;           // counter at the last rising edge (internal)
	uint8_t have_rise;       // a rising edge has been seen (internal)
	uint8_t prev_level;      // level at the previous capture, for edge-slip detection (internal)
	volatile uint32_t width_us;  // last completed high pulse
	volatile uint32_t period_us; // last completed rising-to-rising interval
	volatile uint32_t count;     // completed period measurements; stalls when disconnected
	volatile uint32_t errs;      // overcaptures + missed-edge slips (pulse shorter than IRQ latency)
};

struct PWMIn {
	struct TIM_GP16_Type *const tim;
	uint32_t mask; // counter width mask, auto-detected by pwmin_init (0xffff / 0xffffffff)
	struct PWMInChan ch[4];
};

#define PWMIN_INITIALIZER(tim, p1, p2, p3, p4) \
	{&tim, 0, {{.pin = p1}, {.pin = p2}, {.pin = p3}, {.pin = p4}}}
// TIM2/TIM5 (32-bit, struct TIM_GP32_Type) share the GP16 register layout for
// everything this driver touches; only the usable counter width differs (detected
// at init).
#define PWMIN_INITIALIZER_32(tim, p1, p2, p3, p4) \
	{(struct TIM_GP16_Type *)&tim, 0, {{.pin = p1}, {.pin = p2}, {.pin = p3}, {.pin = p4}}}

// pwmin_init: program a 1 MHz tick from the timer's kernel clock (pass
// clock_pclk1_timer_hz() / clock_pclk2_timer_hz() from clock.h), free-running
// full-range counter, both-edge capture + glitch filter + IRQ enable on every
// channel whose pin is nonzero, and start the counter. The application
// enables the RCC clock before, and the NVIC line after.
void pwmin_init(struct PWMIn *p, uint32_t kernel_hz);

// pwmin_irq_handler: service all pending capture channels of this timer.
// Wire it to the timer's vector. Not on any latency-critical path: capture
// values are latched by hardware, the handler only has to keep up with edge
// rate (8 servo channels x 400 Hz x 2 edges = 6.4 k IRQ/s worst case).
void pwmin_irq_handler(struct PWMIn *p);

// pwmin_get: a torn-read-safe snapshot of one channel. width/period are the
// last COMPLETED measurements (never partial), count is their generation
// number: unchanged count means no new period completed since the last call.
struct PWMInSample {
	uint32_t width_us, period_us, count;
};
struct PWMInSample pwmin_get(const struct PWMIn *p, int chan);

// ---- output generation --------------------------------------------------------
//
// One struct PWMOut per generating timer: all its channels share the period
// (the timer's ARR), each channel's pulse width is its CCR, both programmed
// directly in µs at the 1 MHz tick. Width updates are preloaded (OCxPE) and
// take effect at the next period boundary.
//
//   static const struct PWMOut servo = PWMOUT_INITIALIZER(TIM4);
//   RCC.APB1ENR1 |= RCC_APB1ENR1_TIM4EN;                   // app owns clocks
//   pwmout_init(&servo, clock_pclk1_timer_hz(), 20000, 0xf); // 50 Hz, CH1-4
//   pwmout_set(&servo, 0, 1500);                           // 1.5 ms on CH1

struct PWMOut {
	struct TIM_GP16_Type *const tim;
	const uint8_t moe; // timer has break circuitry: BDTR.MOE must be raised
};

// GP16/GP32 timers (TIM2..5); the layout-compatibility note above applies.
#define PWMOUT_INITIALIZER(tim) {(struct TIM_GP16_Type *)&tim, 0}
// break-circuitry timers (TIM1/8/15/16/17/20); TIM15 has 2 channels, TIM16/17 one
#define PWMOUT_INITIALIZER_BRK(tim) {(struct TIM_GP16_Type *)&tim, 1}

// pwmout_init: 1 MHz tick from the timer's kernel clock, period_us on ARR,
// PWM mode 1 + preload on every channel in chmask (bit 0 = CH1), outputs
// enabled, counter running. Widths start at 0 (constant low): pwmout_set().
// The pins' AF and the RCC clock are the app's, as ever.
void pwmout_init(const struct PWMOut *p, uint32_t kernel_hz, uint32_t period_us, uint8_t chmask);

// set one channel's pulse width; takes effect at the next period boundary
static inline void pwmout_set(const struct PWMOut *p, int chan, uint32_t width_us) {
	(&p->tim->CCR1)[chan] = width_us; // CCR1..CCR4 are contiguous
}
