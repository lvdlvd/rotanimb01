#pragma once

// exti — EXTI line routing/config for GPIO pins (header-only).
//
// EXTI lines 0..15 are shared by all GPIO ports: SYSCFG_EXTICR selects which
// port drives each line, so two pins with the same pin NUMBER cannot both
// have an EXTI, regardless of port. The functions take an enum GPIO_Pin
// (one port + pin mask, multi-pin masks applied per set bit) and follow the
// [N]Array division of labor: the application enables the SYSCFG clock
// (RCC_APB2ENR_SYSCFGEN) before routing, owns the NVIC lines and the vector
// slots (EXTI0..EXTI4 have their own vectors; 5-9 and 10-15 share
// EXTI9_5_IRQn / EXTI15_10_IRQn), and clears pending bits in its handler.
//
// Bring-up sketch (SPI chip-select demux, both-edge on PC0/PC1):
//   RCC.APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//   exti_init(PC0 | PC1, true, true);          // route + both edges + unmask
//   nvic_enable(EXTI0_IRQn); nvic_enable(EXTI1_IRQn);
//   // in the handler: if (exti_pending(PC0)) { exti_clear(PC0); ... }

#include "device.h"
#include "gpio.h"

// exti_route: point the pins' EXTI line(s) at the pins' port.
static inline void exti_route(enum GPIO_Pin pins) {
	assert(gpio_one_port(pins));
	uint32_t port = gpio_index(pins);
	for (uint32_t m = pins & Pin_All; m != 0; m &= m - 1) {
		int line = __builtin_ctz(m);
		volatile uint32_t *cr = &SYSCFG.EXTICR1 + line / 4; // EXTICR1..4 are contiguous
		int sh = 4 * (line % 4);
		*cr = (*cr & ~(0xFu << sh)) | (port << sh);
	}
}

// edge selection and interrupt mask, per pin mask
static inline void exti_rising(enum GPIO_Pin pins, bool on) {
	uint32_t m = pins & Pin_All;
	EXTI.RTSR1 = on ? (EXTI.RTSR1 | m) : (EXTI.RTSR1 & ~m);
}
static inline void exti_falling(enum GPIO_Pin pins, bool on) {
	uint32_t m = pins & Pin_All;
	EXTI.FTSR1 = on ? (EXTI.FTSR1 | m) : (EXTI.FTSR1 & ~m);
}
static inline void exti_enable(enum GPIO_Pin pins) { EXTI.IMR1 |= pins & Pin_All; }
static inline void exti_disable(enum GPIO_Pin pins) { EXTI.IMR1 &= ~(pins & Pin_All); }

// pending: PR1 is rc_w1 — exti_clear writes 1s to clear exactly these lines.
static inline uint32_t exti_pending(enum GPIO_Pin pins) { return EXTI.PR1 & (pins & Pin_All); }
static inline void exti_clear(enum GPIO_Pin pins) { EXTI.PR1 = pins & Pin_All; }

// exti_trigger: software-raise the lines (testing; fires like a real edge).
static inline void exti_trigger(enum GPIO_Pin pins) { EXTI.SWIER1 = pins & Pin_All; }

// exti_init: route + edge config + clear stale pendings + unmask, one call.
static inline void exti_init(enum GPIO_Pin pins, bool rising, bool falling) {
	exti_route(pins);
	exti_rising(pins, rising);
	exti_falling(pins, falling);
	exti_clear(pins);
	exti_enable(pins);
}
