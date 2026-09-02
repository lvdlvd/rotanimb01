#include "fault.h"

#include "device.h"

// Linker-provided: stack top / low limit (lib/sections.ld) and the code span.
extern uint32_t _estack, _stack_limit, _stext, _etext;

// The crash record lives at the low end of CCRAM (.crashdump, NOLOAD) so it is
// not zeroed at boot and survives a reset. A guard gap separates it from the
// stack growing down from _estack (see sections.ld); _stack_limit is the lowest
// the stack may reach.
struct crash crashdump __attribute__((section(".crashdump")));

// looks_like_return: a plausible Thumb call-return address — odd, within the
// code span, and the instruction it returns to is preceded by a BL or BLX. The
// code-span bound keeps the probe reads safe (no faulting on wild addresses).
static int looks_like_return(uint32_t a) {
	if (!(a & 1)) {
		return 0;
	}
	uint32_t insn = a & ~1u;
	if (insn < (uint32_t)&_stext + 4 || insn > (uint32_t)&_etext) {
		return 0;
	}
	uint16_t h2 = *(const uint16_t *)(insn - 2);
	if ((h2 & 0xff80) == 0x4780) {
		return 1; // BLX Rm (16-bit)
	}
	uint16_t h1 = *(const uint16_t *)(insn - 4);
	return (h1 & 0xf800) == 0xf000 && (h2 & 0xd000) == 0xd000; // BL (32-bit)
}

// Scan the stack from sp up to _estack, recording return addresses. Order: this
// runs BEFORE the record is written, so harvesting can't read clobbered stack.
static uint32_t harvest_backtrace(uint32_t sp) {
	uint32_t top = (uint32_t)&_estack;
	uint32_t n = 0;
	for (uint32_t p = sp & ~3u; p < top && n < CRASH_BT_MAX; p += 4) {
		uint32_t w = *(const uint32_t *)p;
		if (looks_like_return(w)) {
			crashdump.bt[n++] = w;
		}
	}
	return n;
}

static void crash_finish(void) __attribute__((noreturn));
static void crash_finish(void) {
	crashdump.magic = CRASH_MAGIC;
	__asm volatile("dsb");
	// BKPT to stop live if a debugger is attached (CoreDebug DHCSR C_DEBUGEN).
	if (*(volatile uint32_t *)0xE000EDF0u & 1u) {
		__asm volatile("bkpt 0");
	}
	SCB.AIRCR = (0x5FAu << 16) | (SCB.AIRCR & SCB_AIRCR_PRIGROUP) | SCB_AIRCR_SYSRESETREQ;
	__asm volatile("dsb");
	for (;;) {
	}
}

// crash_from_fault is tail-called by the naked fault handlers with r0 = the
// exception frame (the active stack), r1 = EXC_RETURN.
void crash_from_fault(uint32_t *frame, uint32_t exc_return) __attribute__((used, noreturn));
void crash_from_fault(uint32_t *frame, uint32_t exc_return) {
	crashdump.lr = frame[5];
	crashdump.pc = frame[6];
	crashdump.psr = frame[7];

	uint32_t framelen = 32;            // r0-r3, r12, lr, pc, xpsr
	if (!(exc_return & 0x10)) {         // bit 4 clear -> extended (FP) frame
		framelen += 18 * 4;
	}
	if (frame[7] & (1u << 9)) {         // xPSR bit 9 -> 4-byte stack alignment pad
		framelen += 4;
	}
	uint32_t sp = (uint32_t)frame + framelen;
	crashdump.sp = sp;
	crashdump.overflow = sp < (uint32_t)&_stack_limit;

	if (crashdump.pc < 0x100) {        // branched to a NULL vector -> stray IRQ
		crashdump.kind = CRASH_STRAYIRQ;
		crashdump.irq = frame[7] & 0x1ff;
	} else {
		crashdump.kind = CRASH_FAULT;
		crashdump.cfsr = SCB.CFSR;
		crashdump.hfsr = SCB.HFSR;
		crashdump.faultaddr = (crashdump.cfsr & SCB_CFSR_MMARVALID) ? SCB.MMFAR
		                      : (crashdump.cfsr & SCB_CFSR_BFARVALID) ? SCB.BFAR
		                                                              : 0;
	}
	crashdump.nbt = crashdump.overflow ? 0 : harvest_backtrace(sp);
	crash_finish();
}

#define FAULT_HANDLER(name)                                                       \
	__attribute__((naked)) void name(void) {                                      \
		__asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n"     \
		               "mov r1, lr\n b crash_from_fault\n");                       \
	}
FAULT_HANDLER(HardFault_Handler)
FAULT_HANDLER(MemManage_Handler)
FAULT_HANDLER(BusFault_Handler)
FAULT_HANDLER(UsageFault_Handler)

// __assert_func: the newlib assert() hook, so <assert.h> works. No exception
// frame — capture the call site and walk the current stack.
void __assert_func(const char *file, int line, const char *func, const char *expr) __attribute__((noreturn));
void __assert_func(const char *file, int line, const char *func, const char *expr) {
	(void)func;
	__asm volatile("cpsid i");
	uint32_t sp;
	__asm volatile("mov %0, sp" : "=r"(sp));
	crashdump.kind = CRASH_ASSERT;
	crashdump.file = file;
	crashdump.expr = expr;
	crashdump.line = line;
	crashdump.pc = (uint32_t)__builtin_return_address(0);
	crashdump.sp = sp;
	crashdump.overflow = sp < (uint32_t)&_stack_limit;
	crashdump.nbt = crashdump.overflow ? 0 : harvest_backtrace(sp);
	crash_finish();
}

// ---- reporting -------------------------------------------------------------

static void puts_(void (*putc)(char), const char *s) {
	while (*s) {
		putc(*s++);
	}
}
static void hex_(void (*putc)(char), uint32_t v) {
	for (int i = 7; i >= 0; i--) {
		putc("0123456789abcdef"[(v >> (4 * i)) & 0xf]);
	}
}

void fault_report(void (*putc)(char)) {
	if (crashdump.magic != CRASH_MAGIC) {
		return;
	}
	static const char *const kinds[] = {"none", "fault", "assert", "stray-irq"};
	puts_(putc, "\nCRASH ");
	puts_(putc, crashdump.kind <= CRASH_STRAYIRQ ? kinds[crashdump.kind] : "?");
	puts_(putc, " pc=");
	hex_(putc, crashdump.pc);
	puts_(putc, " lr=");
	hex_(putc, crashdump.lr);
	if (crashdump.kind == CRASH_FAULT) {
		puts_(putc, " cfsr=");
		hex_(putc, crashdump.cfsr);
		puts_(putc, " hfsr=");
		hex_(putc, crashdump.hfsr);
		puts_(putc, " addr=");
		hex_(putc, crashdump.faultaddr);
	}
	if (crashdump.kind == CRASH_STRAYIRQ) {
		puts_(putc, " irq=");
		hex_(putc, crashdump.irq);
	}
	if (crashdump.kind == CRASH_ASSERT && crashdump.file) {
		puts_(putc, " ");
		puts_(putc, crashdump.file);
		puts_(putc, ":");
		hex_(putc, (uint32_t)crashdump.line);
		puts_(putc, " (");
		puts_(putc, crashdump.expr ? crashdump.expr : "");
		puts_(putc, ")");
	}
	if (crashdump.overflow) {
		puts_(putc, " STACK-OVERFLOW");
	}
	// The address sequence after a clean NARRAY-BT...END marker, so the offline
	// symbolizer can find it in arbitrary pasted console text. pc and lr lead
	// (most precise), then the harvested call chain.
	puts_(putc, "\nNARRAY-BT ");
	hex_(putc, crashdump.pc);
	putc(' ');
	hex_(putc, crashdump.lr);
	for (uint32_t i = 0; i < crashdump.nbt; i++) {
		putc(' ');
		hex_(putc, crashdump.bt[i]);
	}
	puts_(putc, " END\n");
	crashdump.magic = 0; // consumed
}
