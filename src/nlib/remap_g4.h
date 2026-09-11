#pragma once

// SRAM1-at-zero remap for the G4 RAM run model (lib/sections_ram.ld).
// startup_init_memory() has already copied the code image to physical SRAM1;
// this maps SRAM1 at 0x00000000, so the very next fetches execute from SRAM
// at zero wait states with every address unchanged. Call it right after
// startup_init_memory() in a RAM-model Reset_Handler; ROM builds simply
// don't. G4-only by unit convention — no other family has this remap.

#include "device.h"

static inline void startup_remap0(void) {
	RCC.APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// MEM_MODE 3: SRAM1 at 0x00000000 (RM0440 SYSCFG_MEMRMP)
	SYSCFG.MEMRMP = (SYSCFG.MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE) | (3 & SYSCFG_MEMRMP_MEM_MODE);
	__asm volatile("dsb 0xf; isb 0xf" ::: "memory"); // flush the fetch stream onto the new mapping
}
