#pragma once

// Startup memory init. Call startup_init_memory() first thing in
// Reset_Handler: it copies the .data initialiser image to RAM and zeroes .bss
// using the symbols from nlib/sections.ld. Run-model-independent — for a RAM
// build _sidata == _sdata so the copy loop is a no-op.
//
// The application owns Reset_Handler (the vector table is the program); a
// minimal one is:
//
//   void Reset_Handler(void) {
//       startup_init_memory();
//       startup_remap0();   // RAM-model builds only (nlib/remap_g4.h)
//       SCB.VTOR = (uint32_t)(uintptr_t)__vectors;  // point at our table
//       ... clock, fpu ...
//       main();
//   }
//
// The G4 RAM run model finishes with the SRAM1-at-zero remap — an explicit
// call to startup_remap0() from nlib/remap_g4.h, a G4-only unit: the app
// states its run model in code, no conditional compilation.

#include "device.h"

#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;    // provided by sections[_ram].ld
extern uint32_t _sitext, _stextcpy, _etextcpy;            // RAM model: code image copy span (empty for ROM)
extern uint32_t _sifast, _sfast, _efast;                  // FASTCODE copy span (VMA == LMA no-op where FAST is flash)

// FASTCODE places a function in the FAST region — ITCM RAM on F7/H7
// (copied at boot by startup_init_memory), plain flash on G4. For the
// deadline-handler class: IRQ bodies whose worst case must not depend on
// flash wait states or cache warmth.
#define FASTCODE __attribute__((section(".fast")))

// DMABUF places a buffer in the DMARAM region — an SRAM every DMA
// master can reach. Required on H7, where the general DMA cannot address
// DTCM (the .data/.bss home); elsewhere it lands after .bss. NOLOAD and not
// zeroed at boot: initialise before starting the controller.
#define DMABUF __attribute__((section(".dmabuf")))

static inline void startup_init_memory(void) {
	// RAM run model: code is linked at 0x00000000 (the boot alias, currently
	// the flash) and copied to its physical SRAM1 home; the remap below then
	// puts SRAM1 at 0x00000000, so the very next fetches execute from SRAM
	// at zero wait states. In the ROM model this span is empty.
	for (uint32_t *src = &_sitext, *dst = &_stextcpy; dst < &_etextcpy;) {
		*dst++ = *src++;
	}
	// FASTCODE code to its run home (ITCM); empty or VMA == LMA elsewhere
	for (uint32_t *src = &_sifast, *dst = &_sfast; dst < &_efast && dst != src;) {
		*dst++ = *src++;
	}
	for (uint32_t *src = &_sidata, *dst = &_sdata; dst < &_edata;) {
		*dst++ = *src++;
	}
	for (uint32_t *dst = &_sbss; dst < &_ebss; dst++) {
		*dst = 0;
	}
}
