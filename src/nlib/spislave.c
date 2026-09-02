// spislave.c — register-file SPI slave engine, see spislave.h for the model.

#include "spislave.h"

#include "exti.h"
#include "gpio.h"

#include <assert.h>

static inline uint32_t irq_lock(void) {
	uint32_t primask;
	__asm volatile("mrs %0, primask; cpsid i" : "=r"(primask)::"memory");
	return primask;
}
static inline void irq_unlock(uint32_t primask) {
	__asm volatile("msr primask, %0" ::"r"(primask) : "memory");
}

void spislave_init(struct SPISlave *s, int spi_mode, bool hw_nss,
                   volatile uint32_t *rstr, uint32_t rstmask) {
	assert(spi_mode == 0 || spi_mode == 3);
	s->rstr = rstr;
	s->rstmask = rstmask;

	// all cs pins on one port, devs[] in ascending pin order
	uint32_t all = 0;
	for (int i = 0; i < 16; i++) {
		s->by_pin[i] = NULL;
	}
	for (int i = 0; i < s->ndev; i++) {
		assert(i == 0 || (s->devs[i]->cs & Pin_All) > (s->devs[i - 1]->cs & Pin_All));
		all |= s->devs[i]->cs;
		s->by_pin[__builtin_ctz(s->devs[i]->cs & Pin_All)] = s->devs[i];
	}
	s->cs_all = all;
	assert(gpio_one_port(s->cs_all));
	assert(!hw_nss || s->ndev == 1); // hardware NSS gates exactly one device

	s->cr2 = SPI_CR2_DS_Bits8 | SPI_CR2_FRXTH_Quarter | SPI_CR2_TXDMAEN | SPI_CR2_RXNEIE;
	s->cr1 = SPI_CR1_SPE | (spi_mode == 3 ? SPI_CR1_CPOL | SPI_CR1_CPHA : 0);
	if (!hw_nss) {
		// SSM with SSI held LOW permanently: the engine is the only MISO
		// driver on its bus and the master idles SCK between frames, so
		// there is nothing to release and nothing to ignore. CS edges are
		// frame delimiters and device demux only; the per-frame bit-sync
		// SSI used to provide comes from the end-of-frame reset instead.
		// This removes select from the deadline entirely: the byte-0 fill
		// is queued into an already-selected SPI a whole CS-gap before the
		// master's first edge, instead of racing it (see spislave.h).
		s->cr1 |= SPI_CR1_SSM;
	}

	// TX DMA: peripheral address is fixed; the memory side is steered per
	// frame. Resolve the channel registers once — the byte-0 handler is on
	// the protocol deadline.
	typeof(&DMA1.CH[0]) ch = &dma_unit(s->tx_chan)->CH[s->tx_chan % 8];
	ch->CR &= DMA_CH_CR_PL;
	ch->PAR = (uintptr_t)&s->dev->DR;
	s->tx_cr = &ch->CR;
	s->tx_ndtr = &ch->NDTR;
	s->tx_mar = &ch->MAR;

	s->active = NULL;
	s->phase = SPISLAVE_PHASE_CMD;

	s->dev->CR1 = 0;
	s->dev->CR2 = s->cr2;
	s->dev->CR1 = s->cr1;
	*spislave_dr8(s->dev) = s->fill; // cover the first command byte-time
}

void spislave_irq_handler(struct SPISlave *s) {
	spislave_irq(s, s->dev, s->tx_chan); // generic: instance resolved at runtime
}

// scrub the SPI at end of frame (an RCC reset pulse — the only way to flush
// the TXFIFO on this IP) and return the frame's length for accounting. Ends
// reconfigured, selected again (the slave is selected permanently, see the
// header), with exactly the fill queued: it preloads into the idle shifter
// right here, a whole CS-gap before the next frame's first edge, so no
// queue write ever races a slot's first bit (AN5543's late-write
// displacement).
static int spislave_close_hw(struct SPISlave *s) {
	int len = 0;
	switch (s->phase) {
	case SPISLAVE_PHASE_READ:
		len = s->ndtr0 - *s->tx_ndtr; // overcounts the TXFIFO prefetch, see spislave.h
		break;
	case SPISLAVE_PHASE_WRITE:
		len = (s->addr - s->addr0) & (SPIDEV_REGS - 1);
		break;
	}
	*s->tx_cr &= DMA_CH_CR_PL;

	*s->rstr |= s->rstmask; // scrub: both FIFOs, shift logic, stale flags
	*s->rstr &= ~s->rstmask;
	s->dev->CR2 = s->cr2;
	s->dev->CR1 = s->cr1; // reconfigured and immediately selected again (SSM, SSI low):
	                      // the reset just resynced the bit counter for the next frame
	*spislave_dr8(s->dev) = s->fill; // byte-0 cover: preloads into the idle,
	                                 // already-selected shifter right here — a
	                                 // whole CS-gap ahead of the next frame
	s->phase = SPISLAVE_PHASE_CMD;
	return len;
}

// account the closed frame and run the device's hook — off the select
// deadline: call only after any reselect has been armed.
static void spislave_close_account(struct SPISlave *s, struct SPIDev *d, int len) {
	d->frames++;
	if (d->frame != NULL) {
		d->frame(d, s->cmd, s->addr0, len);
	}
}

// select the device: pure bookkeeping — the SPI is permanently selected
// (see spislave_init); only the dummy slots are device-specific, and they
// are not due until byte 1, more than a byte-time away.
static inline void spislave_select(struct SPISlave *s, struct SPIDev *sel) {
	for (int i = 0; i < sel->ndummy; i++) {
		*spislave_dr8(s->dev) = s->fill;
	}
	s->active = sel;
	s->phase = SPISLAVE_PHASE_CMD;
}

void spislave_resolve(struct SPISlave *s) {
	uint32_t low = (s->cs_all & Pin_All) & ~digitalIn(s->cs_all);

	// fast path: idle to a single select — bookkeeping and the dummy slots
	// only; the SPI is permanently selected and the fill is long preloaded.
	if (s->active == NULL && low != 0 && (low & (low - 1)) == 0) {
		struct SPIDev *sel = s->by_pin[__builtin_ctz(low)];
		if (sel != NULL) {
			spislave_select(s, sel);
			exti_clear(s->cs_all);
			return;
		}
	}

	uint32_t pend = exti_pending(s->cs_all);
	exti_clear(s->cs_all);

	// arbitrate: lowest selected (low) cs pin wins; devs[] is pin-ascending
	if (low & (low - 1)) {
		s->overlap++; // ill-behaved master: more than one select asserted
	}
	struct SPIDev *sel = NULL;
	for (int i = 0; i < s->ndev; i++) {
		if (s->devs[i]->cs & low) {
			sel = s->devs[i];
			break;
		}
	}

	if (sel == s->active) {
		// same device still selected — but a pending edge on its own line
		// means it bounced high and back (back-to-back transactions faster
		// than this handler): close that frame and open the next, hook last.
		if (sel != NULL && (pend & sel->cs & Pin_All)) {
			int len = spislave_close_hw(s);
			spislave_select(s, sel);
			spislave_close_account(s, sel, len);
		}
		return;
	}
	if (s->active != NULL) {
		struct SPIDev *old = s->active;
		int len = spislave_close_hw(s);
		s->active = NULL;
		if (sel != NULL) {
			spislave_select(s, sel);
		}
		spislave_close_account(s, old, len);
		return;
	}
	if (sel != NULL) {
		spislave_select(s, sel);
	}
}

void spislave_cs_handler(struct SPISlave *s) { spislave_resolve(s); }

bool spidev_commit(struct SPISlave *s, struct SPIDev *d, uint8_t addr, const void *src, size_t len) {
	assert(addr + len <= SPIDEV_REGS);
	const uint8_t *p = src;
	uint32_t primask = irq_lock();
	if (s->active == d) {
		irq_unlock(primask);
		return false;
	}
	for (size_t i = 0; i < len; i++) {
		d->reg[addr + i] = p[i];
	}
	irq_unlock(primask);
	return true;
}

bool spidev_apply(struct SPISlave *s, struct SPIDev *d, void (*fn)(struct SPIDev *d, void *ctx), void *ctx) {
	uint32_t primask = irq_lock();
	if (s->active == d) {
		irq_unlock(primask);
		return false;
	}
	fn(d, ctx);
	irq_unlock(primask);
	return true;
}
