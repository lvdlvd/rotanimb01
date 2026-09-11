#pragma once

// Clock setup (STM32G4). The model: detect whether an HSE crystal
// is fitted and measure its frequency at runtime (RM0440 §7.2.16 — TIM16 input-
// capture of HSE/32 against the 16 MHz HSI), then a handful of preset targets
// bring SYSCLK up using the PLL (off HSE if present, else HSI16), taking care of
// the PWR voltage range / boost and the flash wait-states in the RM-mandated
// order. For anything fancier, configure the RCC tree by hand.
//
// The rate queries read the *live* RCC config, so they work after a preset or a
// hand-rolled setup; use them to compute UART BRR, CAN bit timing, etc. The
// measured HSE is cached so PLL-from-HSE rates can be reported.
//
// Peripheral clock *enables* are the application's job (RCC.APBxENR |= ... at
// the top of main), as in the reference projects — this library only reads.
//
// Requires the generated device header as "device.h".

#include "device.h"
#include <stdint.h>

enum { HSI16_HZ = 16000000 };

// clock_hse_hz caches the measured crystal frequency (0 = no HSE / not measured).
extern uint32_t clock_hse_hz;

// ---- HSE detection & measurement (§7.2.16) --------------------------------

// clock_measure_hse turns on HSE, and if it stabilises, measures it by input-
// capturing HSE/32 on TIM16_CH1 against the (reset-default 16 MHz) timer clock.
// Returns the frequency rounded to the nearest MHz, or 0 if no crystal. Must run
// while SYSCLK is still HSI16 (i.e. early in Reset_Handler). Caches clock_hse_hz.
static inline uint32_t clock_measure_hse(void) {
	RCC.CR |= RCC_CR_HSEON;
	for (int i = 0; i < 20000 && !(RCC.CR & RCC_CR_HSERDY); i++) {
		__asm volatile("");
	}
	if (!(RCC.CR & RCC_CR_HSERDY)) {
		clock_hse_hz = 0;
		return 0;
	}

	RCC.APB2ENR |= RCC_APB2ENR_TIM16EN;
	TIM16.OR1 = TIM16_17_OR1_HSE32EN;                              // route HSE/32 to tim_ti1_in3
	TIM16.TISEL = TIM16_17_TISEL_TI1SEL_HSE_32;                    // TI1 <- HSE/32
	TIM16.CCMR1 = (1u << 0) | TIM16_17_CCMR1_IC1PSC;              // CC1S=TI1, capture every 8th edge
	TIM16.CCER = TIM16_17_CCER_CC1E;                              // enable capture (rising)
	TIM16.ARR = 0xFFFF;
	TIM16.PSC = 0;
	TIM16.EGR = TIM16_17_EGR_UG;
	TIM16.SR = 0;
	TIM16.CR1 = TIM16_17_CR1_CEN;

	// HSE is ready, so a capture is due within 256 HSE cycles; the bound is
	// a generous multiple (all waits here are bounded + post-checked: a
	// hung clock bit must degrade, never hang the boot)
	uint32_t cap[2] = {0, 0};
	int captured = 0;
	for (int k = 0; k < 2; k++) {
		for (int i = 0; i < 100000 && !(TIM16.SR & TIM16_17_SR_CC1IF); i++) {
			__asm volatile("");
		}
		if (!(TIM16.SR & TIM16_17_SR_CC1IF)) {
			break;
		}
		cap[k] = TIM16.CCR1; // reading CCR1 clears CC1IF
		captured++;
	}
	// leave TIM16 as found (reset state): a stale CC1E would silently veto a
	// later user's CC1S write (channel mode bits are locked while CCxE = 1)
	TIM16.CR1 = 0;
	TIM16.CCER = 0;
	TIM16.CCMR1 = 0;
	TIM16.TISEL = 0;
	TIM16.OR1 = 0;
	TIM16.ARR = 0xFFFF;
	RCC.APB2ENR &= ~RCC_APB2ENR_TIM16EN;

	uint32_t ticks = (cap[1] - cap[0]) & 0xFFFF;       // 8 HSE/32 periods at 16 MHz
	if (captured != 2 || ticks == 0) {
		clock_hse_hz = 0;
		return 0;
	}
	// HSE = (8 periods) * 32 * 16 MHz / ticks
	uint32_t hse = (uint32_t)(((uint64_t)8 * 32 * HSI16_HZ) / ticks);
	hse = ((hse + 500000) / 1000000) * 1000000;        // round to nearest MHz
	clock_hse_hz = hse;
	return hse;
}

// ---- preset system-clock targets ------------------------------------------

// clock_set runs SYSCLK at target_hz via the PLL, sourced from HSE if present
// (else HSI16), choosing PLLM to land the VCO input near 8 MHz and PLLN for the
// target with PLLR=2. Sets the flash wait-states and PWR boost as required, in
// the RM-mandated order (voltage/boost before speed-up, flash WS before switch,
// intermediate AHB prescaler while the voltage settles). Returns achieved SYSCLK,
// or 0 if an RCC/FLASH handshake never acknowledged (all waits bounded; on 0 the
// core is still on its previous clock and the caller decides how to degrade).
// Internal — use the clock_init_* presets.
static inline uint32_t clock_pll_set(uint32_t target_hz) {
	uint32_t src = clock_hse_hz ? clock_hse_hz : HSI16_HZ;

	// PLLM (1..8): bring VCO input as close to 8 MHz as possible.
	uint32_t m = (src + 4000000) / 8000000;
	if (m < 1) {
		m = 1;
	}
	if (m > 8) {
		m = 8;
	}
	uint32_t vco_in = src / m;
	// PLLR fixed at 2; PLLN for the target, clamped and kept in VCO range.
	uint32_t n = (target_hz * 2 + vco_in / 2) / vco_in;
	if (n < 8) {
		n = 8;
	}
	if (n > 127) {
		n = 127;
	}
	uint32_t vco = vco_in * n;
	uint32_t sysclk = vco / 2;

	// PWR: range 1; boost (R1MODE=0) only above 150 MHz.
	RCC.APB1ENR1 |= RCC_APB1ENR1_PWREN;
	pwr_cr1_vos_set(1);
	if (sysclk > 150000000) {
		PWR.CR5 &= ~PWR_CR5_R1MODE;
	} else {
		PWR.CR5 |= PWR_CR5_R1MODE;
	}

	rcc_cfgr_hpre_set(8); // AHB/2 while voltage/boost settle (RM0440 7.2.7)
	rcc_cfgr_ppre1_set(0);
	rcc_cfgr_ppre2_set(0);

	// Configure and start the PLL (must be off to change it).
	RCC.CR &= ~RCC_CR_PLLON;
	for (int i = 0; i < 1000000 && (RCC.CR & RCC_CR_PLLRDY); i++) {
		__asm volatile("");
	}
	if (RCC.CR & RCC_CR_PLLRDY) {
		return 0;
	}
	rcc_pllcfgr_pllsrc_set(clock_hse_hz ? 3 : 2); // 3=HSE, 2=HSI16
	rcc_pllcfgr_pllm_set(m - 1);                  // field = divider-1
	rcc_pllcfgr_plln_set(n);
	rcc_pllcfgr_pllr_set(0);                      // 0 -> /2
	RCC.PLLCFGR |= RCC_PLLCFGR_PLLREN;
	RCC.CR |= RCC_CR_PLLON;

	// Flash wait-states for the target HCLK (range 1 boost, RM0440 Table 19).
	uint32_t ws = sysclk <= 34000000 ? 0 : sysclk <= 68000000 ? 1 : sysclk <= 102000000 ? 2 : sysclk <= 136000000 ? 3 : 4;
	FLASH.ACR |= FLASH_ACR_PRFTEN;
	flash_acr_latency_set(ws);
	for (int i = 0; i < 100000 && flash_acr_latency_get() != ws; i++) {
		__asm volatile("");
	}
	if (flash_acr_latency_get() != ws) {
		return 0;
	}

	for (int i = 0; i < 1000000 && !(RCC.CR & RCC_CR_PLLRDY); i++) {
		__asm volatile("");
	}
	if (!(RCC.CR & RCC_CR_PLLRDY)) {
		return 0;
	}
	rcc_cfgr_sw_set(3); // PLL as system clock
	for (int i = 0; i < 100000 && rcc_cfgr_sws_get() != 3; i++) {
		__asm volatile("");
	}
	if (rcc_cfgr_sws_get() != 3) {
		return 0;
	}
	rcc_cfgr_hpre_set(0); // AHB at full speed now that voltage is stable
	return sysclk;
}

// Run from HSI16 (or HSE if a 16 MHz crystal) directly, no PLL: 16 MHz, 0 WS,
// no boost — the low-power / simplest option.
static inline uint32_t clock_init_16(void) {
	clock_measure_hse();
	RCC.APB1ENR1 |= RCC_APB1ENR1_PWREN;
	pwr_cr1_vos_set(1);
	flash_acr_latency_set(0);
	rcc_cfgr_hpre_set(0);
	rcc_cfgr_ppre1_set(0);
	rcc_cfgr_ppre2_set(0);
	if (clock_hse_hz == 16000000) {
		rcc_cfgr_sw_set(2); // HSE
		for (int i = 0; i < 100000 && rcc_cfgr_sws_get() != 2; i++) {
			__asm volatile("");
		}
		return rcc_cfgr_sws_get() == 2 ? 16000000 : 0;
	}
	rcc_cfgr_sw_set(1); // HSI16
	for (int i = 0; i < 100000 && rcc_cfgr_sws_get() != 1; i++) {
		__asm volatile("");
	}
	return rcc_cfgr_sws_get() == 1 ? HSI16_HZ : 0;
}

static inline uint32_t clock_init_64(void) {
	clock_measure_hse();
	return clock_pll_set(64000000);
}
static inline uint32_t clock_init_144(void) {
	clock_measure_hse();
	return clock_pll_set(144000000);
}
static inline uint32_t clock_init_168(void) {
	clock_measure_hse();
	return clock_pll_set(168000000);
}

// ---- live rate queries ----------------------------------------------------

// ahb_div / apb_div decode the HPRE / PPRE prescaler fields to a divisor.
static inline uint32_t clock_ahb_div(void) {
	static const uint16_t d[] = {1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
	return d[rcc_cfgr_hpre_get()];
}
static inline uint32_t clock_apb_div(uint32_t ppre) {
	static const uint8_t d[] = {1, 1, 1, 1, 2, 4, 8, 16};
	return d[ppre & 7];
}

// clock_sysclk_hz computes the current SYSCLK from the live RCC config.
static inline uint32_t clock_sysclk_hz(void) {
	switch (rcc_cfgr_sws_get()) {
	case 1:
		return HSI16_HZ;
	case 2:
		return clock_hse_hz;
	case 3: { // PLL
		uint32_t src = rcc_pllcfgr_pllsrc_get() == 3 ? clock_hse_hz : HSI16_HZ;
		uint32_t vco = (src / (rcc_pllcfgr_pllm_get() + 1)) * rcc_pllcfgr_plln_get();
		return vco / ((rcc_pllcfgr_pllr_get() + 1) * 2); // 0,1,2,3 -> /2,/4,/6,/8
	}
	default:
		return HSI16_HZ; // HSI16 after reset
	}
}

static inline uint32_t clock_hclk_hz(void) { return clock_sysclk_hz() / clock_ahb_div(); }
static inline uint32_t clock_pclk1_hz(void) { return clock_hclk_hz() / clock_apb_div(rcc_cfgr_ppre1_get()); }
static inline uint32_t clock_pclk2_hz(void) { return clock_hclk_hz() / clock_apb_div(rcc_cfgr_ppre2_get()); }

// APBx timer clock: equal to PCLKx if the APB prescaler is /1, else 2*PCLKx.
static inline uint32_t clock_pclk1_timer_hz(void) {
	return clock_apb_div(rcc_cfgr_ppre1_get()) == 1 ? clock_pclk1_hz() : 2 * clock_pclk1_hz();
}
static inline uint32_t clock_pclk2_timer_hz(void) {
	return clock_apb_div(rcc_cfgr_ppre2_get()) == 1 ? clock_pclk2_hz() : 2 * clock_pclk2_hz();
}

// ---- peripheral kernel-clock rates (CCIPR muxes) --------------------------
// For BRR / bit-timing math. Each reads the peripheral's clock-source field.

// USART1..3, UART4/5 on PCLK: USART1 is on APB2, the rest on APB1.
// sel: 0=PCLK, 1=SYSCLK, 2=HSI16, 3=LSE (LSE not tracked; returns 0).
static inline uint32_t clock_usart_hz(int usart) {
	uint32_t sel;
	switch (usart) {
	case 1:
		sel = rcc_ccipr_usart1sel_get();
		break;
	case 2:
		sel = rcc_ccipr_usart2sel_get();
		break;
	case 3:
		sel = rcc_ccipr_usart3sel_get();
		break;
	case 4:
		sel = rcc_ccipr_uart4sel_get();
		break;
	default:
		sel = rcc_ccipr_uart5sel_get();
		break;
	}
	switch (sel) {
	case 0:
		return usart == 1 ? clock_pclk2_hz() : clock_pclk1_hz();
	case 1:
		return clock_sysclk_hz();
	case 2:
		return HSI16_HZ;
	default:
		return 0; // LSE
	}
}

// LPUART1: 0=PCLK1, 1=SYSCLK, 2=HSI16, 3=LSE.
static inline uint32_t clock_lpuart_hz(void) {
	switch (rcc_ccipr_lpuart1sel_get()) {
	case 0:
		return clock_pclk1_hz();
	case 1:
		return clock_sysclk_hz();
	case 2:
		return HSI16_HZ;
	default:
		return 0;
	}
}

// I2C1..3: 0=PCLK1, 1=SYSCLK, 2=HSI16.
static inline uint32_t clock_i2c_hz(int i2c) {
	uint32_t sel = i2c == 1 ? rcc_ccipr_i2c1sel_get() : i2c == 2 ? rcc_ccipr_i2c2sel_get() : rcc_ccipr_i2c3sel_get();
	switch (sel) {
	case 1:
		return clock_sysclk_hz();
	case 2:
		return HSI16_HZ;
	default:
		return clock_pclk1_hz();
	}
}

// FDCAN: 0=HSE, 1=PLLQ, 2=PCLK1.
static inline uint32_t clock_fdcan_hz(void) {
	switch (rcc_ccipr_fdcansel_get()) {
	case 0:
		return clock_hse_hz;
	case 1: { // PLLQ
		uint32_t src = rcc_pllcfgr_pllsrc_get() == 3 ? clock_hse_hz : HSI16_HZ;
		uint32_t vco = (src / (rcc_pllcfgr_pllm_get() + 1)) * rcc_pllcfgr_plln_get();
		return vco / ((rcc_pllcfgr_pllq_get() + 1) * 2);
	}
	default:
		return clock_pclk1_hz();
	}
}
