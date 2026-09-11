#pragma once

// spislave — a register-file SPI slave engine: emulates the address+data
// protocol family shared by sensor/EEPROM-style devices (BMI08x, BMP3xx,
// RM3100, 25xx, ...) against live register files, at the register level.
//
// Protocol model. A transaction is framed by the device's chip select. Byte 0
// is the command: a read/write flag plus a 7-bit start address; the address
// increments implicitly for every following byte. Reads may owe the master a
// fixed number of dummy bytes after the command (BMI08x accel, BMP3xx: 1;
// BMI08x gyro, RM3100: 0) and then stream the register file; writes carry
// data bytes the master stores through the file's write mask.
//
// Engine. The register file IS the TX DMA buffer:
//  - byte 0 arrives by RXNE interrupt: decode and point the TX DMA at
//    reg[addr] — every subsequent byte of a read burst is then served by
//    hardware (RXNE is muted until deselect; the RXFIFO overrun this causes
//    is scrubbed by the end-of-frame reset). TX slot structure: byte 0 (the
//    command time) ships the fill, queued by the previous frame's close
//    into the idle, already-selected shifter — a whole CS-gap before the
//    master's first edge; the device's dummy slots are stuffed at select
//    and are not due until byte 1. A byte queued later than its slot's
//    first bit does not corrupt but DISPLACES — it ships one slot late,
//    with the underrun pattern (oldest FIFO byte per AN5543, 0x00 from a
//    scrubbed FIFO) covering the miss; the permanently-selected design
//    (see Selection) exists to keep every queue write a whole CS-gap or
//    byte-time clear of its slot. A dummy protocol's first data byte is
//    owed a whole byte-time after the command; a no-dummy protocol owes
//    byte 1 itself, straight off this interrupt, and is only reliable to
//    ~1.3 MHz against a gapless master (168 MHz, code in RAM) — which fits
//    this device class: the no-dummy parts are 1 MHz-limited by datasheet
//    or served at 1 MHz lowspeed. A burst reading past the top of the file
//    shifts stale bytes — real parts leave that undefined too.
//  - write bytes stay per-RXNE software: masked store into the file, address
//    increment. Side effects (soft reset, mode changes) do NOT run here.
//  - deselect closes the frame: the SPI is scrubbed by an RCC reset pulse
//    (RM0440 39.5.9: RXFIFO survives SPE=0 and TXFIFO cannot be flushed any
//    other way), reconfigured (two register writes, ending selected again
//    with the next fill queued), and the device's frame() hook runs with
//    {cmd, addr, len}: clear-on-read semantics, write side effects,
//    protocol diagnostics — all off the deadline path. The reset doubles
//    as the per-frame bit-counter resync that NSS toggling would provide.
//
// Selection. The slave is SELECTED PERMANENTLY (SSM with SSI held low):
// this engine is by construction the only MISO driver on its bus — every
// emulated device lives in this one SPI — and the master idles SCK between
// frames, so there is nothing to release and nothing to ignore. A
// deselect-reselect gap on a master that chains transactions is a few
// hundred ns; gating SSI per frame put the shifter preload in a race with
// the master's first SCK edge (lost by a hair: responses shifted one slot,
// or bit-shifted commands when SSI lost outright). Held low, the race does
// not exist. The CS lines are plain GPIO inputs on one port, watched by
// both-edge EXTI (lib/exti.h) purely as FRAME DELIMITERS and device demux:
// the shared spislave_cs_handler() reads them in one IDR access and picks
// the device — devs[] in ascending pin order, lowest selected pin wins.
// With hardware NSS (single device) the NSS pin gates in silicon as usual;
// the same EXTI handler still closes/opens frames and runs the hook.
//
// Register-file integrity: the sampler updates the file with
// spidev_commit(), an atomic attempt that refuses (returns false) while the
// device is selected — retry on the next tick. The copy runs with interrupts
// masked for its ~few-dozen cycles so a select cannot sneak a read burst
// into a half-written file; if bring-up shows that jitter matters at the
// byte-0 deadline, the fallbacks are a write queue drained by the frame hook
// or commits scheduled off the TX DMA half-transfer interrupt.
//
// The application owns clocks, pins, EXTI routing, DMAMUX and all vectors
// (the lib model): route the TX channel (dma_set_mux SPIn_TX), wire the
// SPI vector to spislave_irq_handler and every CS EXTI vector to
// spislave_cs_handler, all at the highest device priority — nothing else in
// the system may share it.

#include "device.h"
#include "dma.h"
#include "exti.h"
#include "pinmux.h"

#include <stdbool.h>
#include <stddef.h>

#ifndef SPIDEV_REGS
#define SPIDEV_REGS 128 // register file size == the 7-bit protocol address space
#endif

struct SPIDev {
	// configuration
	const enum GPIO_Pin cs;      // the device's select line (GPIO input, EXTI both edges)
	const uint8_t read_cmd_mask; // command-byte flag meaning "read" (0x80 for the sensor family)
	const uint8_t ndummy;        // dummy bytes owed after a read command (0 or 1)
	const uint8_t *wmask;        // SPIDEV_REGS/8 bytes, LSB-first bitmap of master-writable
	                             // registers; NULL = whole file read-only
	// frame hook, called at deselect (priority 0: keep it to register flips):
	// cmd is the raw command byte, addr its start address, len the data bytes
	// exchanged after command+dummies (reads: may overcount by the few bytes
	// prefetched into the TX FIFO but never shifted out).
	void (*frame)(struct SPIDev *d, uint8_t cmd, uint8_t addr, int len);

	// optional streaming register (FIFO_DATA-style drain ports): a read burst
	// that REACHES stream_addr is served from stream[] instead of the file —
	// matching real parts, where such a register does not auto-increment
	// either. The frame hook sees the usual {cmd, addr, len} and pops what
	// the master consumed; since len can overcount by the TX FIFO prefetch,
	// pop in whole frames (len rounded down to the frame size) — exact
	// whenever the master reads whole-frame multiples, which framed FIFOs
	// impose anyway. A burst reading PAST stream_size shifts stale bytes,
	// like reading past the top of the file. NULL = no streaming register.
	//
	// stream_prefix (usually 0) says how many registers BELOW stream_addr a
	// burst may be entered at and still land in the port: drivers exist that
	// read a length/status register and the drain port in one transaction,
	// counting on the address increment to carry them across. Those k
	// registers must be MIRRORED into the head of stream[] (stream[0..k-1] =
	// reg[stream_addr-k .. stream_addr-1], the device's job, wherever it
	// maintains them) so the hardware can serve the whole burst from one
	// linear source; stream[k] is then the first byte of the port itself.
	// A burst entered at stream_addr-j (0 <= j <= k) starts at stream[k-j].
	// The frame hook must subtract the same (stream_addr - addr) from len
	// before popping. k = 0 means "only a burst starting exactly at
	// stream_addr", which is what a device whose driver splits the two
	// transactions needs.
	uint8_t *stream;
	uint16_t stream_size;
	uint8_t stream_addr;
	uint8_t stream_prefix;

	uint8_t reg[SPIDEV_REGS]; // the live register file (the "silicon")

	// diagnostics
	volatile uint32_t frames;     // completed frames
	volatile uint32_t writes;     // master write bytes accepted
	volatile uint32_t unexpected; // write bytes refused by wmask
};

struct SPISlave {
	struct SPI_Type *const dev;
	const enum DMA_CHAN tx_chan;
	struct SPIDev *const *const devs; // ascending cs pin number = descending priority
	const int ndev;
	const uint8_t fill; // MISO byte during command/dummy time and for RO gaps

	// set by spislave_init
	volatile uint32_t *rstr; // RCC reset register + mask for this SPI: the
	uint32_t rstmask;        // end-of-frame scrub (see header comment)
	uint32_t cr1, cr2;       // configuration reapplied after the scrub
	enum GPIO_Pin cs_all;    // all cs pins OR'd (one port)
	struct SPIDev *by_pin[16]; // pin number -> device, for the select fast path
	volatile uint32_t *tx_cr, *tx_ndtr, *tx_mar; // the TX DMA channel registers, resolved once

	// frame state (internal)
	struct SPIDev *active; // selected device, NULL = none
	uint8_t phase;         // 0 = expect command, 1 = write data, 2 = read streaming
	uint8_t cmd, addr0, addr;
	uint16_t ndtr0;            // NDTR armed at stream start, for len accounting
	volatile uint32_t stray;    // bytes clocked with no device selected
	volatile uint32_t midframe; // frame boundaries resolved from the RXNE path: the
	                            // master deselected and reselected while this handler
	                            // ran or was pended (same preemption group as the CS
	                            // EXTIs), so the EXTI would have lost the race to the
	                            // next command byte — healed, but a nonzero count means
	                            // the master chains transactions back-to-back
	volatile uint32_t overlap;  // CS edges seen with >1 select asserted (ill-behaved
	                           // master): the lowest pin wins, the loser is ignored —
	                           // or, if the newcomer outranks the running transfer,
	                           // preempts it (partial frame() with the truncated len);
	                           // the loser's eventual frame is garbage until its next
	                           // deselect. Real silicon would fight over MISO instead.
};

#define SPISLAVE_INITIALIZER(spi_, tx_chan_, devtab_, ndev_, fill_) \
	{.dev = &spi_, .tx_chan = tx_chan_, .devs = devtab_, .ndev = ndev_, .fill = fill_}

// spislave_init: configure the SPI as an 8-bit slave (mode 0 or 3 — real
// sensors auto-detect per frame, a hardware slave cannot: use what the
// master's driver uses), hardware NSS when hw_nss (single device, its cs is
// the NSS pin) else SSM with SSI held low permanently (see Selection above).
// rstr/rstmask name
// the SPI's RCC reset bit (e.g. &RCC.APB2RSTR + RCC_APB2RSTR_SPI1RST).
// The app enables the RCC clocks and DMAMUX routing before, and the NVIC
// lines after.
void spislave_init(struct SPISlave *s, int spi_mode, bool hw_nss,
                   volatile uint32_t *rstr, uint32_t rstmask);

// ---- the RXNE handler ---------------------------------------------------------
//
// Two forms of the same body. The inline form takes the SPI instance and the
// TX DMA channel as parameters so that a vector shim passing compile-time
// constants gets literal addresses and no call overhead — worth 15-25 cycles
// on the byte-0 deadline path:
//
//   static void spi3_irq(void) { spislave_irq(&slave, &SPI3, DMA1_CH5); }
//
// The plain spislave_irq_handler(s) wrapper serves everything less critical.

enum { SPISLAVE_PHASE_CMD, SPISLAVE_PHASE_WRITE, SPISLAVE_PHASE_READ };

// spislave_resolve: the CS arbitration/close/reopen body, shared between the
// CS EXTI handler and the RXNE handler's end-of-drain boundary check below.
void spislave_resolve(struct SPISlave *s);

// the FIFO'd SPI wants byte-size DR accesses for 8-bit frames (data packing)
static inline volatile uint8_t *spislave_dr8(struct SPI_Type *spi) {
	return (volatile uint8_t *)&spi->DR;
}

__attribute__((always_inline)) static inline void spislave_irq(struct SPISlave *s, struct SPI_Type *spi, enum DMA_CHAN tx_chan) {
	typeof(&DMA1.CH[0]) ch = &dma_unit(tx_chan)->CH[tx_chan % 8];
	while (spi->SR & SPI_SR_RXNE) {
		uint8_t b = *spislave_dr8(spi);
		struct SPIDev *d = s->active;
		if (d == NULL) {
			s->stray++;
			continue;
		}
		if (s->phase == SPISLAVE_PHASE_CMD) {
			uint8_t addr = b & (SPIDEV_REGS - 1) & 0x7f;
			if (b & d->read_cmd_mask) {
				// the deadline path: steer the TX DMA at reg[addr] (linear to
				// the top of the file), bookkeeping after. The device's dummy
				// byte, if any, was pre-stuffed at select: byte 1's FIFO load
				// happens essentially AT this interrupt, no software can feed
				// it — pre-stuffing moves the dummy protocol's deadline to
				// byte 2, a whole byte-time away. The streaming-register test
				// costs ~5 cycles here also for devices without one.
				uint16_t n = (uint16_t)(SPIDEV_REGS - addr);
				const uint8_t *src = &d->reg[addr];
				// off wraps to >= 129 when addr is above stream_addr, which
				// never compares <= a prefix, so one subtract and one compare
				// decide both the exact-hit and the enter-early cases.
				uint8_t off = (uint8_t)(d->stream_addr - addr);
				if (d->stream != NULL && off <= d->stream_prefix) {
					uint8_t skip = (uint8_t)(d->stream_prefix - off);
					n = (uint16_t)(d->stream_size - skip);
					src = d->stream + skip;
				}
				ch->NDTR = n;
				ch->MAR = (uintptr_t)src;
				ch->CR |= DMA_CH_CR_MINC | DMA_CH_CR_DIR | DMA_CH_CR_EN;
				// hardware streams from here; mute RXNE (the RXFIFO overruns
				// harmlessly, the end-of-frame reset scrubs it)
				spi->CR2 = s->cr2 & ~SPI_CR2_RXNEIE;
				s->cmd = b;
				s->addr0 = addr;
				s->ndtr0 = n;
				s->phase = SPISLAVE_PHASE_READ;
				break; // loop ends; a wrapping shim's epilogue still runs
			}
			s->cmd = b;
			s->addr0 = s->addr = addr;
			s->phase = SPISLAVE_PHASE_WRITE;
			continue;
		}
		// SPISLAVE_PHASE_WRITE: masked store, implicit address increment
		if (d->wmask != NULL && ((d->wmask[s->addr / 8] >> (s->addr % 8)) & 1)) {
			d->reg[s->addr] = b;
			d->writes++;
		} else {
			d->unexpected++;
		}
		s->addr = (s->addr + 1) & (SPIDEV_REGS - 1);
	}
	// A CS edge that arrived while we ran (or while we were pended) would
	// reach the CS EXTI — same preemption group — only after this exit, and
	// a master that deselects and reselects back-to-back (both edges inside
	// its own transfer-done interrupt) starts clocking the next command
	// within a microsecond of that: the EXTI's close/reopen would lose that
	// race and the next read would shift out one stale TXFIFO byte. Resolve
	// the boundary here, synchronously, while byte 0 of the next frame is
	// still a whole byte-time away. (The EXTI still fires afterwards; it
	// finds no pending edge and the same device selected, and does nothing.)
	if (exti_pending(s->cs_all)) {
		s->midframe++;
		spislave_resolve(s);
	}
}

void spislave_irq_handler(struct SPISlave *s);


// CS handler — wire to every cs pin's EXTI vector (both edges, priority 0).
// Reads all cs pins, arbitrates (lowest pin wins), closes/opens the frame.
void spislave_cs_handler(struct SPISlave *s);

// spidev_commit: atomically copy len bytes over reg[addr..] unless the device
// is currently selected (returns false: retry on the caller's next tick).
// A burst can never observe a half-committed sample.
bool spidev_commit(struct SPISlave *s, struct SPIDev *d, uint8_t addr, const void *src, size_t len);

// spidev_apply: the general form — run fn(d, ctx) with interrupts masked,
// unless the device is currently selected (returns false, fn not run: retry
// on the caller's next tick). For updates a plain copy can't express: multi-
// region register writes that must not tear against a frame, stream[] appends
// with their bookkeeping registers. Keep fn to a few dozen cycles.
bool spidev_apply(struct SPISlave *s, struct SPIDev *d, void (*fn)(struct SPIDev *d, void *ctx), void *ctx);
