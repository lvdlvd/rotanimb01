/*
 * RAM-run section layout: a standalone flash image whose code
 * executes from SRAM1 at zero wait states.
 *
 * Code and vectors are linked at 0x00000000 — the boot alias, where the main
 * flash appears out of reset and where SRAM1 appears after the SYSCFG remap.
 * The image is stored in flash (LOAD); startup_init_memory(), still executing
 * from the flash alias, copies the code image to its physical SRAM1 home
 * (the .textimage reservation at the bottom of DATA), copies .data, zeroes
 * .bss; the Reset_Handler then calls startup_remap0() (lib/remap_g4.h) to map
 * SRAM1 at 0x00000000: every address stays valid while fetches move to SRAM.
 * Nothing here depends on a debugger.
 *
 * The per-part memory script binds TEXT (= CODE at 0x00000000), DATA (the
 * SRAM region) and LOAD (= FLASH) before INCLUDE-ing this file.
 */

ENTRY(Reset_Handler)
EXTERN(__vectors)

SECTIONS
{
  .isr_vector : {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > TEXT AT> LOAD

  .text : {
    . = ALIGN(4);
    _stext = .;                  /* code span, for the fault backtrace scanner */
    *(.text) *(.text.*)
    *(.fast) *(.fast.*)          /* FASTCODE: whole image runs from SRAM here */
    *(.rodata) *(.rodata.*)
    . = ALIGN(4);
    _etext = .;
  } > TEXT AT> LOAD

  /* the physical SRAM the 0x00000000-linked code occupies after the remap:
     reserve it at the bottom of DATA so .data/.bss land above the code */
  .textimage (NOLOAD) : {
    . += SIZEOF(.isr_vector) + SIZEOF(.text);
    . = ALIGN(4);
  } > DATA
  _sitext = LOADADDR(.isr_vector); /* code image in flash...            */
  _stextcpy = ORIGIN(DATA);        /* ...copied to physical SRAM1,      */
  _etextcpy = _stextcpy + SIZEOF(.isr_vector) + SIZEOF(.text); /* which the remap aliases at 0 */

  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(SORT_BY_ALIGNMENT(.data.*))
    . = ALIGN(4);
    _edata = .;
  } > DATA AT> LOAD
  _sidata = LOADADDR(.data);

  .bss : {
    . = ALIGN(4);
    _sbss = .;
    *(.bss) *(SORT_BY_ALIGNMENT(.bss.*)) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > DATA

  /* DMABUF buffers: NOLOAD, not zeroed — the owner initialises them */
  .dmabuf (NOLOAD) : {
    . = ALIGN(4);
    *(.dmabuf) *(.dmabuf.*)
    . = ALIGN(4);
  } > DMARAM

  /* fast-copy hooks, inert: FASTCODE code is in the image copied above */
  _sifast = 0;
  _sfast = 0;
  _efast = 0;


  /* lib/fault.c crash record at the low end of SCRATCH (NOLOAD, survives
     reset); stack grows down from _estack (top of SCRATCH). */
  .crashdump (NOLOAD) : {
    . = ALIGN(8);
    KEEP(*(.crashdump))
    . = ALIGN(8);
  } > SCRATCH
  _stack_limit = ORIGIN(SCRATCH) + SIZEOF(.crashdump) + 256; /* + guard gap */

  /DISCARD/ : {
    *(.ARM.exidx) *(.ARM.exidx.*)
  }
}
