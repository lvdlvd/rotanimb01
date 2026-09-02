#pragma once

// DMA request routing for STM32G4 (and other DMAMUX families): any request
// line can reach any channel through the DMAMUX. The family-neutral channel
// helpers live in lib/dma.h (pulled in here); the L4 counterpart with its
// fixed-channel CSELR scheme is lib/dma_l4.h.

#include "dma.h"

// DMAMUX request line ids (RM0440 Table 91). Extend as needed.
enum DMA_REQ {
	DMA_REQ_NONE = 0,
	DMA_REQ_SPI1_RX = 10, DMA_REQ_SPI1_TX, DMA_REQ_SPI2_RX, DMA_REQ_SPI2_TX, DMA_REQ_SPI3_RX, DMA_REQ_SPI3_TX,
	DMA_REQ_I2C1_RX = 16, DMA_REQ_I2C1_TX, DMA_REQ_I2C2_RX, DMA_REQ_I2C2_TX, DMA_REQ_I2C3_RX, DMA_REQ_I2C3_TX, DMA_REQ_I2C4_RX, DMA_REQ_I2C4_TX,
	DMA_REQ_USART1_RX = 24, DMA_REQ_USART1_TX, DMA_REQ_USART2_RX, DMA_REQ_USART2_TX, DMA_REQ_USART3_RX, DMA_REQ_USART3_TX,
	DMA_REQ_UART4_RX = 30, DMA_REQ_UART4_TX, DMA_REQ_UART5_RX, DMA_REQ_UART5_TX, DMA_REQ_LPUART1_RX, DMA_REQ_LPUART1_TX,
};

// route a DMAMUX request line onto a channel
static inline void dma_set_mux(enum DMA_CHAN ch, enum DMA_REQ req) {
	DMAMUX.C[ch] = (DMAMUX.C[ch] & ~DMAMUX_C_DMAREQ_ID) | ((uint32_t)req & DMAMUX_C_DMAREQ_ID);
}
