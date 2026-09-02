#pragma once

// NVIC — the core of the [N]Array non-operating-system.
//
// In [N]Array the interrupt vector table is the program: a designated-
// initializer array the application owns, with handlers placed by the
// generated IRQn_Type enum. This header is the API for that table and for the
// controller that dispatches through it. It owns three things:
//
//   1. The table's location (SCB.VTOR) and runtime handler installation.
//   2. Per-interrupt enable / pending / active state.
//   3. Interrupt priority (device IRQs and core exceptions alike).
//
// Slot index = IRQn + 16: slots 0..15 are the Cortex core exceptions (SP,
// Reset, faults, SVCall, PendSV, SysTick), device IRQ n is at slot n+16.
// VECTOR() expresses that for the table literal; the core slots are positional.
//
// Requires the narray-generated device header (NVIC, SCB, IRQn_Type,
// NVIC_VECTORS) to be included first. This layout is Cortex-M3/M4/M7; M0+ has
// a single ISER and no IABR and needs its own version.

#ifndef NARRAY_DEVICE
#error "include the narray-generated device header before nvic.h"
#endif

typedef void (*isr_t)(void);

// VECTOR(irq) is the table slot for a device interrupt. Core exceptions sit at
// fixed low slots and are written positionally, not through this macro.
#define VECTOR(irq) ((int)(irq) + 16)

// STM32 Cortex-M4 implements the top 4 of the 8 priority bits: 16 levels.
enum { NVIC_PRIO_BITS = 4 };

// Memory barriers. Guarded so they coexist with a core header that defines them.
#ifndef __DSB
#define __DSB() __asm volatile("dsb 0xf" ::: "memory")
#endif
#ifndef __ISB
#define __ISB() __asm volatile("isb 0xf" ::: "memory")
#endif
#ifndef __DMB
#define __DMB() __asm volatile("dmb 0xf" ::: "memory")
#endif

// ---- per-interrupt state (device IRQs, irq >= 0) --------------------------

static inline void nvic_enable(enum IRQn_Type irq) { NVIC.ISER[irq >> 5] = 1u << (irq & 31); }

static inline void nvic_disable(enum IRQn_Type irq) {
	NVIC.ICER[irq >> 5] = 1u << (irq & 31);
	__DSB(); // ensure the disable takes effect before returning
	__ISB();
}

static inline int nvic_is_enabled(enum IRQn_Type irq) { return (NVIC.ISER[irq >> 5] >> (irq & 31)) & 1u; }

static inline void nvic_set_pending(enum IRQn_Type irq) { NVIC.ISPR[irq >> 5] = 1u << (irq & 31); }
static inline void nvic_clear_pending(enum IRQn_Type irq) { NVIC.ICPR[irq >> 5] = 1u << (irq & 31); }
static inline int nvic_is_active(enum IRQn_Type irq) { return (NVIC.IABR[irq >> 5] >> (irq & 31)) & 1u; }

// ---- priority -------------------------------------------------------------

// nvic_set_priority sets level (0 = highest, 15 = lowest) for any interrupt:
// device IRQs land in NVIC.IPR, core exceptions (irq < 0) in SCB.SHPR. The
// level occupies the implemented high bits of the priority byte.
static inline void nvic_set_priority(enum IRQn_Type irq, unsigned level) {
	uint8_t b = (uint8_t)(level << (8 - NVIC_PRIO_BITS));
	if (irq < 0) {
		((volatile uint8_t *)&SCB.SHPR1)[(int)irq + 12] = b; // SHPR covers handlers 4..15
	} else {
		((volatile uint8_t *)NVIC.IPR)[irq] = b;
	}
}

// nvic_set_priority_grouping selects the priority/subpriority split via the
// unlock-keyed AIRCR write. 0..7, see PM0214 4.4.5.
static inline void nvic_set_priority_grouping(unsigned group) {
	uint32_t r = SCB.AIRCR & ~(SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP); // clear key + group bits
	SCB.AIRCR = r | (0x5FAu << 16) | ((group & 7u) << 8);
}

// ---- vector table relocation & runtime install ----------------------------

// nvic_relocate copies the current (flash) vector table into ram and points
// VTOR at it, enabling runtime handler installation. ram must hold NVIC_VECTORS
// entries and be aligned to NVIC_VTOR_ALIGN. After this, nvic_install works.
static inline void nvic_relocate(isr_t *ram) {
	const isr_t *cur = (const isr_t *)(uintptr_t)SCB.VTOR;
	for (int i = 0; i < NVIC_VECTORS; i++) {
		ram[i] = cur[i];
	}
	__DSB();
	SCB.VTOR = (uint32_t)(uintptr_t)ram;
	__DSB();
	__ISB();
}

// nvic_install points an interrupt at fn in the live (RAM) vector table. For a
// device IRQ it brackets the swap with disable/enable so an interrupt can't
// fire on a half-written slot; a previously-disabled IRQ stays disabled.
// Requires nvic_relocate() to have moved the table to writable RAM.
static inline void nvic_install(enum IRQn_Type irq, isr_t fn) {
	isr_t *vt = (isr_t *)(uintptr_t)SCB.VTOR;
	if (irq < 0) { // core exception: no NVIC enable bit to manage
		vt[VECTOR(irq)] = fn;
		__DSB();
		__ISB();
		return;
	}
	int was_on = nvic_is_enabled(irq);
	nvic_disable(irq);
	vt[VECTOR(irq)] = fn;
	__DSB();
	__ISB();
	if (was_on) {
		nvic_enable(irq);
	}
}

// ---- system ---------------------------------------------------------------

// nvic_system_reset requests a full system reset and does not return.
static inline void nvic_system_reset(void) {
	__DSB();
	SCB.AIRCR = (0x5FAu << 16) | (SCB.AIRCR & SCB_AIRCR_PRIGROUP) | SCB_AIRCR_SYSRESETREQ;
	__DSB();
	for (;;) {
	}
}
