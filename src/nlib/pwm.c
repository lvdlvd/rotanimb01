// pwm.c — PWM input measurement and output generation, see pwm.h.

#include "pwm.h"

#include "gpio.h"

void pwmin_init(struct PWMIn *p, uint32_t kernel_hz) {
	struct TIM_GP16_Type *t = p->tim;
	t->CR1 = 0;
	t->DIER = 0;
	t->CCER = 0; // capture units off while configuring
	t->PSC = kernel_hz / 1000000u - 1;
	t->ARR = 0xffffffffu; // free-running full range;
	p->mask = t->ARR;     // a 16-bit timer reads back 0xffff
	t->CNT = 0;

	// Per used channel: CCxS = 01 (input, mapped on TIx), ICxPSC = 0 (capture
	// every edge), ICxF = 0b1001 (f_DTS/8, 8 samples): a ~0.4 us glitch
	// filter at a 168 MHz kernel clock (CR1.CKD = 0 so f_DTS = kernel).
	uint32_t ccmr[2] = {0, 0}, ccer = 0, dier = 0;
	for (int i = 0; i < 4; i++) {
		if (!p->ch[i].pin) {
			continue;
		}
		ccmr[i / 2] |= (1u | (9u << 4)) << (8 * (i & 1));
		ccer |= (TIM_GP16_CCER_CC1E | TIM_GP16_CCER_CC1P | TIM_GP16_CCER_CC1NP) << (4 * i);
		dier |= TIM_GP16_DIER_CC1IE << i;
	}
	t->CCMR1 = ccmr[0];
	t->CCMR2 = ccmr[1];
	t->EGR = TIM_GP16_EGR_UG; // latch PSC into the running prescaler
	t->SR = 0;                // discard flags raised by configuration
	t->CCER = ccer;
	t->DIER = dier;
	t->CR1 = TIM_GP16_CR1_CEN;
}

void pwmin_irq_handler(struct PWMIn *p) {
	struct TIM_GP16_Type *t = p->tim;
	uint32_t sr = t->SR;
	volatile uint32_t *const ccr = &t->CCR1; // CCR1..CCR4 are contiguous
	for (int i = 0; i < 4; i++) {
		struct PWMInChan *c = &p->ch[i];
		if (!c->pin) {
			continue;
		}
		if (sr & (TIM_GP16_SR_CC1OF << i)) {
			t->SR = ~(TIM_GP16_SR_CC1OF << i); // rc_w0: clears only this flag
			c->errs++;
		}
		if (!(sr & (TIM_GP16_SR_CC1IF << i))) {
			continue;
		}
		uint32_t cap = ccr[i] & p->mask; // reading CCRx clears CCxIF
		// The capture is latched by hardware; the pin level classifies which
		// edge it was. A pulse shorter than the IRQ latency reads as two
		// same-level edges: the lost half-measurement is counted in errs,
		// period measurement survives.
		int level = digitalIn(c->pin) != 0;
		if (level) { // rising: close the period, open the next pulse
			if (c->have_rise) {
				if (c->prev_level) {
					c->errs++; // missed the falling edge; width is stale
				}
				c->period_us = (cap - c->rise) & p->mask;
				c->count++;
			}
			c->rise = cap;
			c->have_rise = 1;
		} else { // falling: close the width
			if (!c->prev_level) {
				c->errs++; // missed the rising edge; rise is stale, skip
			} else if (c->have_rise) {
				c->width_us = (cap - c->rise) & p->mask;
			}
		}
		c->prev_level = (uint8_t)level;
	}
}

struct PWMInSample pwmin_get(const struct PWMIn *p, int chan) {
	const struct PWMInChan *c = &p->ch[chan];
	struct PWMInSample s;
	do {
		s.count = c->count;
		s.width_us = c->width_us;
		s.period_us = c->period_us;
	} while (s.count != c->count); // re-read on a mid-snapshot update
	return s;
}

void pwmout_init(const struct PWMOut *p, uint32_t kernel_hz, uint32_t period_us, uint8_t chmask) {
	struct TIM_GP16_Type *t = p->tim;
	t->CR1 = 0;
	t->CCER = 0; // CCxS is writable only while CCxE = 0 (the timer may be second-hand)
	t->PSC = kernel_hz / 1000000u - 1;
	t->ARR = period_us - 1;

	// PWM mode 1 (0b0110) + preload per selected channel; widths start 0
	uint32_t ccer = 0;
	uint32_t mode1 = (6u << 4) | TIM_GP16_CCMR1_OC1PE, mode2 = (6u << 12) | TIM_GP16_CCMR1_OC2PE;
	t->CCMR1 = (chmask & 1 ? mode1 : 0) | (chmask & 2 ? mode2 : 0);
	if (chmask & 0xc) { // don't touch CCMR2 on the 1/2-channel timers
		t->CCMR2 = (chmask & 4 ? mode1 : 0) | (chmask & 8 ? mode2 : 0);
	}
	for (int i = 0; i < 4; i++) {
		if (chmask & (1u << i)) {
			(&t->CCR1)[i] = 0;
			ccer |= TIM_GP16_CCER_CC1E << (4 * i);
		}
	}
	t->CCER = ccer;
	if (p->moe) {
		// BDTR lives at 0x44 on the break-circuitry timers (TIM1/8/15/16/17/20),
		// a reserved slot in the GP16 layout this struct is cast from.
		*(volatile uint32_t *)((uintptr_t)t + 0x44) = TIM_ADV_BDTR_MOE;
	}
	t->EGR = TIM_GP16_EGR_UG; // latch PSC/ARR
	t->CR1 = TIM_GP16_CR1_CEN;
}
