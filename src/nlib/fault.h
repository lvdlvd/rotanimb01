#pragma once

// Fault, assert and stray-IRQ trapping for [N]Array (Cortex-M4).
//
// Faults capture a minimal post-mortem into a no-init crash record at the low
// end of CCRAM (survives reset): the causal PC/LR + fault cause, and a call-
// chain harvested by scanning the stack for plausible Thumb return addresses
// (no unwind tables in ROM). After capture: BKPT if a debugger is attached,
// else reset. On the next boot, fault_report() prints the record via the (now
// working) console — including the backtrace addresses, which you feed to an
// offline symbolizer (arm-none-eabi-addr2line -fie fw.elf <addrs>) to recover
// source:line of each call site.
//
// Stray IRQs: with the [N]Array NULL-vector model, an unwired IRQ branches to 0
// and faults; the fault handler recognises pc≈0 and records the IRQ number from
// the stacked xPSR. No default IRQ handler is needed.
//
// The app wires the four fault handlers into its vector table (slots 3-6):
//   HardFault_Handler, MemManage_Handler, BusFault_Handler, UsageFault_Handler.
//
// Requires the narray device header as "device.h" and the linker symbols
// _estack, _stack_limit (lib/sections.ld), _stext, _etext.

#include <stdint.h>

enum crash_kind { CRASH_NONE = 0, CRASH_FAULT, CRASH_ASSERT, CRASH_STRAYIRQ };

enum { CRASH_BT_MAX = 24, CRASH_MAGIC = 0x0FEEDBAC };

struct crash {
	uint32_t magic; // CRASH_MAGIC when the record is valid
	uint8_t kind;   // enum crash_kind
	uint8_t overflow;
	uint16_t irq;                  // stray-IRQ exception number (CRASH_STRAYIRQ)
	uint32_t pc, lr, psr, sp;      // causal context
	uint32_t cfsr, hfsr, faultaddr; // fault cause (CRASH_FAULT)
	const char *file, *expr;       // assert site (CRASH_ASSERT)
	int32_t line;
	uint32_t nbt;            // backtrace entry count
	uint32_t bt[CRASH_BT_MAX]; // call-chain return addresses
};

// The crash record, in no-init CCRAM — valid across reset when magic is set.
extern struct crash crashdump;

// The fault entry points (wire into the vector table). Defined in fault.c.
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

// Print and clear the crash record (if valid) via putc. Call early in boot once
// a console is available. Prints the backtrace addresses for offline addr2line.
void fault_report(void (*putc)(char));
