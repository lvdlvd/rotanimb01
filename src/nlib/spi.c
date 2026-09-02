#include "spi.h"

void spiq_init(
	struct SPIQ *q,
	struct SPI_Type *spi,
	enum SPI_CR1_BR clock_div,
	enum DMA_CHAN rx_chan,
	enum DMA_CHAN tx_chan,
	spi_slave_select_func_t *ss_func) {
	q->spi = spi;
	q->rx_chan = rx_chan;
	q->tx_chan = tx_chan;
	q->ss_func = ss_func;
	q->head = q->curr = q->tail = q->dropped = 0;
	q->busy = 0;

	// 8-bit master, mode 0 (CPOL=CPHA=0), at the requested baud divisor.
	spi->CR1 = 0;
	spi->CR1 = SPI_CR1_MSTR | clock_div;
	spi->CR2 = SPI_CR2_DS_Bits8 | SPI_CR2_FRXTH_Quarter | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

	if (ss_func != NULL) {
		spi->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // software-managed NSS held high
	} else {
		spi->CR2 |= SPI_CR2_SSOE; // hardware drives NSS
	}
	// DMA request routing is the app's job before calling this (family-
	// specific: dma_set_mux from dma_g4.h/dma_l4.h) — same division of
	// labor as the serial driver.

	// Enable now and stay enabled: a disabled master tri-states SCK, and a
	// floating clock parked at a slave's input threshold (breakout-board
	// pull-ups put it mid-rail) generates spurious edges that bit-slip
	// edge-counting slaves. Enabled with an empty TX FIFO, SCK idles driven
	// at the CPOL level.
	spi->CR1 |= SPI_CR1_SPE;
}

static void startxmit(struct SPIQ *q) {
	struct SPIXmit *x = &q->elem[q->curr % SPI_QUEUE_LEN];

	q->busy = 1;

	if (q->ss_func != NULL) {
		q->ss_func(q->spi, x->addr, 1);
	}

	// RX armed before TX so no byte can arrive unclaimed once the clock runs.
	dma_start_rx(q->rx_chan, &q->spi->DR, x->buf, x->len);
	dma_start_tx(q->tx_chan, &q->spi->DR, x->buf, x->len);
}

// Idle == no transaction in flight. startxmit sets busy; the RX DMA handler
// clears it. (SPE deliberately stays set throughout — see spiq_init.)
static inline int spi_idle(struct SPIQ *q) { return !q->busy; }

void spiq_enq_head(struct SPIQ *q) {
	q->head++;
	if (spi_idle(q)) {
		startxmit(q);
	}
}

void spi_rx_dma_handler(struct SPIQ *q) {
	struct SPIXmit *x = &q->elem[q->curr % SPI_QUEUE_LEN];

	// RX completion ends the transaction; clear both channels' flags (the TX
	// channel has no NVIC line of its own, only this shared completion point).
	dma_isr(q->rx_chan);
	dma_isr(q->tx_chan);

	x->status = q->spi->SR & 0xf0; // error/overrun bits
	q->busy = 0;

	if (q->ss_func != NULL) {
		q->ss_func(q->spi, x->addr, 0);
	}

	q->curr++;
	if (q->head != q->curr) {
		startxmit(q);
	}
}

void spi_wait(struct SPIQ *q) {
	// Condition check and WFI must be atomic against the completion irq: a
	// TC that lands between them would otherwise be consumed with nothing
	// left pending, and WFI sleeps forever (a lost wakeup). WFI with PRIMASK
	// set still wakes on a pending-enabled interrupt; the handler then runs
	// at cpsie, and the loop re-checks.
	for (;;) {
		__asm volatile("cpsid i" ::: "memory");
		if (q->curr != q->tail || (spi_idle(q) && q->head == q->curr)) {
			__asm volatile("cpsie i" ::: "memory");
			return;
		}
		__asm volatile("wfi; cpsie i" ::: "memory");
	}
}

uint16_t spiq_xmit(struct SPIQ *q, uint16_t addr, size_t len, uint8_t *buf) {
	struct SPIXmit *x = spiq_head(q);
	if (x == NULL) {
		return 0xffff;
	}
	x->addr = addr;
	x->len = len;
	for (size_t i = 0; i < len; ++i) {
		x->buf[i] = buf[i];
	}
	spiq_enq_head(q);
	spi_wait(q);
	if (x != spiq_tail(q)) {
		return 0xfffe; // someone else drained our result; queue is misused
	}
	uint16_t r = x->status;
	for (size_t i = 0; i < len; ++i) {
		buf[i] = x->buf[i];
	}
	spiq_deq_tail(q);
	return r;
}
