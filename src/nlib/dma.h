#pragma once

// DMA channel helpers, family-neutral half: the channel-based controllers of
// the G4 and L4 share the DMA_Type.CH[] register layout, so everything here
// works on both. What differs is how a peripheral request reaches a channel —
// G4: DMAMUX, any request to any channel; L4: a per-channel CSELR code with
// fixed channel assignments. The app includes the matching request-routing
// header, lib/dma_g4.h or lib/dma_l4.h (each pulls this file in); the drivers
// (serial, spi) include only this neutral half.
//
// Channels are numbered 1..16 across the two controllers via enum DMA_CHAN;
// parts with fewer channels (G4 cat2: 6+6, L4: 7+7) simply lack the higher
// ones — using an absent channel is the caller's responsibility, like an
// unexposed pin.
//
// Requires the generated device header as "device.h".

#include "device.h"
#include <stddef.h>
#include <stdint.h>

// DMA_CHAN n: controller = n/8 (0=DMA1, 1=DMA2), channel index = n%8.
enum DMA_CHAN {
	DMA1_CH1 = 0, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, DMA1_CH8,
	DMA2_CH1 = 8, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, DMA2_CH6, DMA2_CH7, DMA2_CH8,
};

// Per-channel interrupt flags within the controller's ISR/IFCR (each channel
// occupies a 4-bit group; these are the bit positions within a group).
enum {
	DMA_TEIF = 1u << 3, // transfer error
	DMA_HTIF = 1u << 2, // half transfer
	DMA_TCIF = 1u << 1, // transfer complete
	DMA_GIF = 1u << 0,  // global (errata: do not clear)
};

static inline struct DMA_Type *dma_unit(enum DMA_CHAN ch) {
	struct DMA_Type *const u[2] = {&DMA1, &DMA2};
	return u[ch / 8];
}

// read and clear the channel's interrupt flags (lower bits per the enum above)
static inline uint8_t dma_isr(enum DMA_CHAN ch) {
	struct DMA_Type *u = dma_unit(ch);
	unsigned shift = 4 * (ch % 8);
	uint8_t isr = (u->ISR >> shift) & 0xe; // errata: do not touch GIF
	u->IFCR = (uint32_t)isr << shift;
	return isr;
}

// remaining transfer count (CNDTR) — used to finalise a partial RX
static inline uint16_t dma_remaining(enum DMA_CHAN ch) { return dma_unit(ch)->CH[ch % 8].NDTR; }

static inline void dma_setprio(enum DMA_CHAN ch, uint32_t prio) {
	volatile uint32_t *cr = &dma_unit(ch)->CH[ch % 8].CR;
	*cr = (*cr & ~DMA_CH_CR_PL) | ((prio << 12) & DMA_CH_CR_PL);
}

// stop the channel, preserving its priority field
static inline void dma_stop(enum DMA_CHAN ch) {
	volatile uint32_t *cr = &dma_unit(ch)->CH[ch % 8].CR;
	*cr &= DMA_CH_CR_PL;
}

// start a transfer with transfer-complete and error interrupts enabled
static inline void dma_start(enum DMA_CHAN ch, const volatile uint32_t *dr, void *mem, size_t n, uint32_t flags) {
	struct DMA_Type *u = dma_unit(ch);
	unsigned i = ch % 8;
	u->CH[i].CR &= DMA_CH_CR_PL; // disable, keep priority
	u->CH[i].NDTR = n;
	u->CH[i].PAR = (uintptr_t)dr;
	u->CH[i].MAR = (uintptr_t)mem;
	u->CH[i].CR |= (flags | DMA_CH_CR_TEIE | DMA_CH_CR_TCIE | DMA_CH_CR_EN) & ~DMA_CH_CR_PL;
}

static inline void dma_start_rx(enum DMA_CHAN ch, const volatile uint32_t *dr, void *mem, size_t n) {
	dma_start(ch, dr, mem, n, DMA_CH_CR_MINC); // periph->mem, increment memory
}
static inline void dma_start_tx(enum DMA_CHAN ch, const volatile uint32_t *dr, void *mem, size_t n) {
	dma_start(ch, dr, mem, n, DMA_CH_CR_MINC | DMA_CH_CR_DIR); // mem->periph
}
