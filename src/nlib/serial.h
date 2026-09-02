#pragma once

// DMA- and IRQ-driven buffered serial (U[S]ART) for STM32G4, over the lib/fifo
// ring buffer. One struct Serial per direction (RX or TX) of a U[S]ART.
//
// The application owns the interrupt vectors (the [N]Array model): wire the
// channel and USART handlers to call the serial_*_handler() routines below.
//
// Bring-up sketch:
//   static uint8_t txbuf[2048];                 // size must be a power of two
//   static struct Serial u1tx = SERIAL_INITIALIZER(USART1, txbuf);
//   ...
//   RCC.AHB1ENR  |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN;  // enable clocks (app)
//   RCC.APB2ENR  |= RCC_APB2ENR_USART1EN;
//   gpioConfigAll(board, COUNT(board));         // TX/RX/DE pins (see pinmux)
//   dma_set_mux(DMA1_CH1, DMA_REQ_USART1_TX);
//   usart_init_tx(&USART1, clock_usart_hz(1), 115200);   // kernel rate from clock.h
//   // in vectors: void DMA1_CH1_Handler(void){ serial_dma_tx_handler(&u1tx, DMA1_CH1); }
//   //             void USART1_Handler(void){ serial_irq_tx_handler(&u1tx, DMA1_CH1); }
//   nvic_enable(DMA1_CH1_IRQn); nvic_enable(USART1_IRQn);
//   ... write to u1tx.buf, then serial_dma_tx_start(&u1tx);

#include "device.h"
#include "dma.h"
#include "fifo.h"

struct Serial {
	struct USART_Type *const usart;
	struct Fifo buf;
	volatile uint32_t ovfl_count;
	volatile uint32_t dmaerr_count;
	volatile uint16_t xfersize; // bytes in the in-flight DMA transfer
};

#define SERIAL_INITIALIZER(usart, buf) {&usart, FIFO_INITIALIZER(buf), 0, 0, 0}
// LPUART1 shares the USART base register layout for the subset serial touches.
#define SERIAL_INITIALIZER_LP(buf) {(struct USART_Type *)&LPUART1, FIFO_INITIALIZER(buf), 0, 0, 0}

// kernel_hz is the peripheral's clock-source rate (clock_usart_hz(n) /
// clock_lpuart_hz() from clock.h). USART/UART: BRR = kernel/baud (oversample
// 16). LPUART: a 256x prescaler factor, BRR = 256*kernel/baud (20-bit).
static inline uint32_t usart_brr(uint32_t kernel_hz, uint32_t baud) { return kernel_hz / baud; }
static inline uint32_t lpuart_brr(uint32_t kernel_hz, uint32_t baud) { return (uint32_t)(((uint64_t)kernel_hz * 256) / baud); }

// The usart_init* functions live in the per-IP-version headers — the app
// includes the one matching its family: lib/usart_v3.h (G4/H7, FIFO IP) or
// lib/usart_v2.h (L4/F7, no FIFO). Everything in this file is neutral: the
// v3 register header keeps the legacy TXE/RXNE flag names, and the
// receive-timeout machinery exists on both.

// ---- RX: DMA into the fifo head, flushed on receive timeout ----------------

// Start a DMA RX of up to a quarter of the buffer into the fifo head. Used by
// both the DMA and the USART timeout handlers.
static inline void serial_dma_rx_start(struct Serial *s, enum DMA_CHAN ch) {
	if (fifo_full(&s->buf)) {
		fifo_reset(&s->buf); // consumer too slow — drop the buffer
		++s->ovfl_count;
	}
	dma_stop(ch);
	size_t n = fifo_head_size(&s->buf);
	size_t cap = (s->buf.szmsk + 1) / 4;
	if (n > cap) {
		n = cap;
	}
	s->xfersize = n;
	dma_start_rx(ch, &s->usart->RDR, fifo_head(&s->buf), n);
}

// Call from the RX channel's DMA handler.
static inline void serial_dma_rx_handler(struct Serial *s, enum DMA_CHAN ch) {
	uint8_t isr = dma_isr(ch);
	if (isr & DMA_TEIF) {
		++s->dmaerr_count;
	}
	if (isr & DMA_TCIF) {
		fifo_push_head(&s->buf, s->xfersize);
		serial_dma_rx_start(s, ch);
	}
}

struct SerialRXCounters {
	uint32_t stall, parityerr, noiseerr, overrunerr;
};

// Call from the USART handler when this Serial is the RX side: on receive
// timeout, flush the partial DMA burst and drain the FIFO-mode bytes, then
// re-arm DMA. counters may be NULL.
static inline void serial_irq_rx_handler(struct Serial *s, enum DMA_CHAN ch, struct SerialRXCounters *c) {
	uint32_t isr = s->usart->ISR;
	if (isr & USART_ISR_RTOF) {
		s->usart->CR3 &= ~USART_CR3_DMAR;
		dma_stop(ch);
		s->usart->ICR |= USART_ICR_RTOCF;
		fifo_push_head(&s->buf, s->xfersize - dma_remaining(ch)); // partial burst
		while (s->usart->ISR & USART_ISR_RXNE) {                  // drain the rest
			if (fifo_full(&s->buf)) {
				fifo_reset(&s->buf);
				++s->ovfl_count;
			}
			fifo_put_head(&s->buf, s->usart->RDR);
		}
		serial_dma_rx_start(s, ch);
		s->usart->CR3 |= USART_CR3_DMAR;
	}
	if (c) {
		if (isr & USART_ISR_PE) ++c->parityerr;
		if (isr & USART_ISR_NF) ++c->noiseerr;
		if (isr & USART_ISR_ORE) ++c->overrunerr;
		if (isr & USART_ISR_RTOF) ++c->stall;
	}
}

// ---- TX: DMA from the fifo tail, chained by the TC interrupt ---------------

// Polled, blocking single-byte transmit: spin on TXE, then push one byte direct
// to TDR. The synchronous counterpart to the DMA path below. Use it for output
// that must NOT depend on the async DMA+IRQ engine — above all the post-mortem
// crash report (fault_report): it runs moments before the program continues or
// re-faults/resets, so a DMA-queued report would sit in the FIFO and never reach
// the wire. Bypasses the Serial fifo, so don't interleave with an active DMA TX.
// Bounded: a dead USART (clock off, misconfig) drops the byte rather than hang
// the caller — which here is above all fault_report itself.
static inline void usart_putc(struct USART_Type *u, char c) {
	for (int i = 0; i < 1000000 && !(u->ISR & USART_ISR_TXE); i++) {
		__asm volatile("");
	}
	if (u->ISR & USART_ISR_TXE) {
		u->TDR = (uint8_t)c;
	}
}

// Kick the TX engine after filling the fifo (idempotent — safe if running).
static inline void serial_dma_tx_start(struct Serial *s) { s->usart->CR1 |= USART_CR1_TCIE; }

// Call from the TX channel's DMA handler.
static inline void serial_dma_tx_handler(struct Serial *s, enum DMA_CHAN ch) {
	uint8_t isr = dma_isr(ch);
	if (isr & DMA_TEIF) {
		++s->dmaerr_count;
	}
	if (isr & DMA_TCIF) {
		fifo_pop_tail(&s->buf, s->xfersize);
		s->xfersize = 0;
	}
}

// Call from the USART handler when this Serial is the TX side: on transmission
// complete, start the next burst from the fifo tail, or stop if the fifo drained.
static inline void serial_irq_tx_handler(struct Serial *s, enum DMA_CHAN ch) {
	if ((s->usart->ISR & USART_ISR_TC) && s->xfersize == 0) {
		if (fifo_empty(&s->buf)) {
			s->usart->CR1 &= ~USART_CR1_TCIE; // done; stop chaining
		} else {
			s->usart->ICR |= USART_ICR_TCCF;
			s->xfersize = fifo_tail_size(&s->buf);
			dma_start_tx(ch, &s->usart->TDR, fifo_tail(&s->buf), s->xfersize);
		}
	}
}
