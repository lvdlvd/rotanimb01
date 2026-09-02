// cordic_stm32.c — device backend: drives the STM32G4 CORDIC coprocessor via
// the narray-generated CORDIC singleton.
//
// Zero-overhead mode (RM0440 §17.3.6): write CSR, push argument(s) to WDATA,
// then read RDATA — the read inserts AHB wait states until the result is
// ready, so there is no polling and no interrupt.
//
// PRECONDITION: the application must enable the CORDIC peripheral clock
//   RCC.AHB1ENR |= RCC_AHB1ENR_CORDICEN;
// before calling any cordic.h function. This file does no clock management.
//
// Concurrency: the CORDIC is a single global resource. Per the runtime
// guarantee (no floating point in IRQ handlers) no locking is done here; if
// that ever changes, wrap cordic_backend_run in a critical section.
//
// This is the only backend that differs from the standalone cordic-math
// project: upstream hand-rolls the register overlay, here we use the generated
// CORDIC struct from device.h. The frontend (cordic_math.c) and the backend
// ABI (cordic_port.h) are imported verbatim.

#include "device.h"
#include "cordic_port.h"

#include <stdint.h>

void cordic_backend_run(uint32_t csr, const int32_t args[2], int32_t res[2]) {
	CORDIC.CSR = csr;
	CORDIC.WDATA = (uint32_t)args[0];
	if (csr & CM_CSR_NARGS) {
		CORDIC.WDATA = (uint32_t)args[1];
	}

	// the RDATA read stalls until the calculation completes
	res[0] = (int32_t)CORDIC.RDATA;
	if (csr & CM_CSR_NRES) {
		res[1] = (int32_t)CORDIC.RDATA;
	}
}
