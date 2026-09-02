#pragma once

// Master-mode SPI transaction queue for STM32G4, generalised from a flight-tested
// STM32L4 driver. One ring of fixed-size transaction records serves as both the
// input and output FIFO, with three cursors instead of two:
//
//     elem[]:  [ done ][ done ][  IN FLIGHT  ][ pending ][ free ]
//              ^tail                ^curr                 ^head
//              `- deq results       `- DMA exchanging     `- enq here
//
//   [tail, curr)  completed transactions, results sitting in buf, awaiting deq
//   elem[curr]    the one DMA is exchanging right now (the element "in the middle")
//   [curr, head)  enqueued, waiting their turn
//
// Each transaction is a full-duplex exchange in place: the TX-DMA clocks buf out
// while the RX-DMA writes the received bytes back into the same buf. Enqueue with
// spiq_head()/spiq_enq_head(), pick up results with spiq_tail()/spiq_deq_tail(),
// or use the synchronous spiq_xmit() helper.
//
// Slave (NSS-driven) operation is a future variant that reuses this same queue.
//
// Built on the generated SPI_Type registers and lib/dma.h (DMAMUX). The app owns
// the vector table: wire the RX DMA channel's IRQ slot to a handler that calls
// spi_rx_dma_handler(), and nvic_enable() that one IRQ. The TX channel needs no
// NVIC line — its transfer-complete flag is cleared inside the handler.

#include "device.h"
#include "dma.h"

#include <stddef.h>
#include <stdint.h>

#ifndef SPI_QUEUE_LEN
#define SPI_QUEUE_LEN 16 // ring depth; must be a power of two for cheap % indexing
#endif
#ifndef SPI_XMIT_BUF
#define SPI_XMIT_BUF 32 // bytes exchanged per transaction (in place: tx out, rx in)
#endif

// A SPIXmit is one transaction on a SPI device.
struct SPIXmit {
	uint64_t ts;  // timestamp (app's units; the driver does not touch it)
	uint32_t tag; // app cookie to remember what this transaction was for
	uint16_t addr;
	uint16_t status; // SPI SR error bits captured at completion
	size_t len;
	uint8_t buf[SPI_XMIT_BUF];
};

// Software slave-select hook used instead of hardware NSS. If passed to
// spiq_init, the driver calls it with on=1 just before enabling the SPI and on=0
// just after disabling it, for each transaction. The addr is whatever the app put
// in the SPIXmit; the address space is entirely the app's. Because the SPI is
// passed in, the hook may also retune speed/polarity per slave.
typedef void spi_slave_select_func_t(struct SPI_Type *spi, uint16_t addr, int on);

// A SPIQ manages background transactions on one SPI peripheral.
struct SPIQ {
	struct SPI_Type *spi;
	enum DMA_CHAN rx_chan;
	enum DMA_CHAN tx_chan;
	spi_slave_select_func_t *ss_func;

	// head >= curr >= tail; elem[curr] is the transaction in flight.
	volatile uint32_t head;
	volatile uint32_t curr;
	volatile uint32_t tail;
	volatile uint32_t dropped; // transactions refused because the queue was full
	volatile uint32_t busy;    // transaction in flight (SPE stays set at idle so SCK is driven, not floating)
	struct SPIXmit elem[SPI_QUEUE_LEN];
};

// Configure spi as an 8-bit master (mode 0) at the given baud divisor and reset
// the queue. clock_div is one of the generated SPI_CR1_BR_Div* values. If
// ss_func is non-NULL, hardware NSS is disabled and the hook drives
// slave-select; otherwise NSS is driven by hardware. The app routes the two
// DMA channels first (dma_set_mux from dma_g4.h/dma_l4.h — family-specific,
// same division of labor as the serial driver) and wires + enables the RX
// channel's NVIC IRQ.
void spiq_init(
	struct SPIQ *q,
	struct SPI_Type *spi,
	enum SPI_CR1_BR clock_div,
	enum DMA_CHAN rx_chan,
	enum DMA_CHAN tx_chan,
	spi_slave_select_func_t *ss_func);

// Call from the RX DMA channel's IRQ handler:
//
//     static void spi1_rx(void) { spi_rx_dma_handler(&spiq); }
//     ... [VECTOR(DMA1_CH2_IRQn)] = spi1_rx ...
//
// It clears both channels' DMA flags, records the status, completes the current
// transaction and starts the next one queued.
void spi_rx_dma_handler(struct SPIQ *q);

// spiq_head returns the slot to fill for enqueuing, or NULL (and bumps dropped) if
// the queue is full.
static inline struct SPIXmit *spiq_head(struct SPIQ *q) {
	if (q->head == q->tail + SPI_QUEUE_LEN) {
		++q->dropped;
		return NULL;
	}
	return &q->elem[q->head % SPI_QUEUE_LEN];
}

// spiq_tail returns the oldest completed transaction to read, or NULL if none has
// finished yet.
static inline struct SPIXmit *spiq_tail(struct SPIQ *q) {
	return (q->curr == q->tail) ? NULL : &q->elem[q->tail % SPI_QUEUE_LEN];
}

// Enqueue the head slot for transmission. Only call after spiq_head() returned
// non-NULL. The buffer must not be touched until the transaction reappears at the
// tail. Starts the SPI immediately if it was idle, else the ISR chains to it.
void spiq_enq_head(struct SPIQ *q);

// Release the tail slot for reuse after its result has been read.
static inline void spiq_deq_tail(struct SPIQ *q) { ++q->tail; }

// Block (via WFI) until at least one transaction has completed and is ready to
// deq, or the queue has gone fully idle.
void spi_wait(struct SPIQ *q);

// Synchronous convenience: exchange len bytes with the addressed device and return
// the status (0xffff if the queue was full). Do NOT mix with async use of the same
// queue. buf is overwritten with the received bytes in place.
uint16_t spiq_xmit(struct SPIQ *q, uint16_t addr, size_t len, uint8_t *buf);
