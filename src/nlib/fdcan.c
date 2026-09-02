// fdcan.c — classic CAN 2.0B on the STM32G4 FDCAN controllers (see fdcan.h).

#include "fdcan.h"

#include <assert.h>

// the fdcan hardware headers are in 32-bit packed (ESI,XTD,RTR,id-a[11],
// id-b[18]) format as defined in the FDCAN RxTx buffer elements
enum { CAN_EXTID = 1 << 30 }; // XTD bit in hardware headers (see below)

// convert between can.h and the STM32G4 fdcan hardware: the hardware has the
// XTD(IDE) bit in position 30 and the 29-bit address contiguous in the lower
// bits, with ID-A in position 28..18.
static inline uint32_t stmfd2can(uint32_t fdheader) {
	return (fdheader & CAN_EXTID) ? can_header_from29(fdheader) : can_header((fdheader >> 18) & 0x7ff, 0);
}

static inline uint32_t can2stmfd(uint32_t header) {
	return can_header_isext(header) ? (CAN_EXTID | can_header_to29(header)) : (can_header_id_a(header) << 18);
}

static void update_can_status(struct FDCan *c) {
	uint32_t isr = c->dev->IR;
	if ((isr & FDCAN_IR_ARA) != 0) {
		c->dev->IR = FDCAN_IR_ARA; // w1c: clear only this flag
		c->status.ara_err++;
	}
	if ((isr & FDCAN_IR_MRAF) != 0) {
		// cf. RM0440 44.4.15
		c->dev->CCCR &= ~FDCAN_CCCR_ASM;
		c->dev->IR = FDCAN_IR_MRAF;
		c->status.mraf_err++;
	}
	uint32_t psr = c->dev->PSR;
	c->status.esr = psr;
	uint8_t lec = psr & 0x7; // last error code
	c->status.lec_count[lec]++;
}

uint32_t fdcan_ecr(const struct FDCan *c) { return c->dev->ECR; }

// The G4 message RAM has a fixed layout per controller (RM0440 44.3.3),
// 0x350 bytes each, FDCAN1/2/3 consecutive from the start of SRAMCAN
// (see FDCAN_INITIALIZER in fdcan.h).
struct FifoElem {
	volatile uint32_t fdheader;
	volatile uint32_t control;
	volatile uint32_t buf[16];
};

struct FDCAN_MsgRAM {
	uint32_t StandardMessageIDFilter[28]; // 44.3.8
	struct ExtendedMessageIDFilter_t {    // 44.3.9
		uint32_t filter;
		uint32_t mask;
	} ExtendedMessageIDFilter[8];
	struct FifoElem RX0FIFO[3]; // 44.3.5
	struct FifoElem RX1FIFO[3];
	struct TXEvent_t { // 44.3.7
		volatile uint32_t fdheader;
		volatile uint32_t control;
	} TXEvent[3];
	struct FifoElem TXFIFO[3]; // 44.3.6
};
static_assert(sizeof(struct FDCAN_MsgRAM) == 0x350, "RM0440 44.3.3 message RAM element size");

// nbrp for a 14-quanta bit (sync + 9 + 4) at bitrate from the kernel clock.
static uint32_t fdcan_nbrp(uint32_t kernel_hz, uint32_t bitrate) {
	uint32_t q = 14 * bitrate;
	assert(bitrate > 0 && kernel_hz % q == 0); // pick a kernel/bitrate that divides
	uint32_t presc = kernel_hz / q;
	assert(presc >= 1 && presc <= 512); // NBRP is 9 bits
	return presc - 1;
}

// fdcan_cccr_wait: CCCR writes are acknowledged across the kernel-clock
// domain; bounded so a missing kernel clock (fdcansel/APB enable) degrades
// to a dead port instead of a hung boot. Returns 1 on ack.
static int fdcan_cccr_wait(struct FDCAN_Type *dev, uint32_t want) {
	for (int i = 0; i < 1000000 && dev->CCCR != want; i++) {
		__asm volatile("");
	}
	return dev->CCCR == want;
}

// fdcan_config_enter: set INIT (stops the port) and wait for the kernel-clock
// domain to acknowledge it, then set CCE to unlock the protected config
// registers. Preserves the other CCCR config bits (DAR, MON, ...), which are
// themselves write-protected outside an INIT+CCE window and so survive the
// reconfiguration. Bounded; returns 1 on ack.
static int fdcan_config_enter(struct FDCAN_Type *dev) {
	dev->CCCR |= FDCAN_CCCR_INIT;
	for (int i = 0; i < 1000000 && !(dev->CCCR & FDCAN_CCCR_INIT); i++) {
		__asm volatile("");
	}
	if (!(dev->CCCR & FDCAN_CCCR_INIT)) {
		return 0;
	}
	dev->CCCR |= FDCAN_CCCR_CCE;
	return 1;
}

void fdcan_setspeed(struct FDCan *c, uint32_t kernel_hz, uint32_t bitrate) {
	uint32_t nbrp = fdcan_nbrp(kernel_hz, bitrate);

	if (!fdcan_config_enter(c->dev)) {
		return;
	}

	fdcan_nbtp_nbrp_set(c->dev, nbrp);

	c->dev->CCCR &= ~FDCAN_CCCR_INIT; // done configuring (clears CCE as well)
}

void fdcan_monitor(struct FDCan *c, int on) {
	if (!fdcan_config_enter(c->dev)) {
		return;
	}

	if (on) {
		c->dev->CCCR |= FDCAN_CCCR_MON;
	} else {
		c->dev->CCCR &= ~FDCAN_CCCR_MON;
	}

	c->dev->CCCR &= ~FDCAN_CCCR_INIT; // done configuring (clears CCE as well)
}

uint32_t fdcan_getspeed(const struct FDCan *c, uint32_t kernel_hz) {
	return kernel_hz / 14 / (1 + fdcan_nbtp_nbrp_get(c->dev));
}

void fdcan_init(struct FDCan *c, uint32_t kernel_hz, uint32_t bitrate) {
	struct FDCAN_Type *dev = c->dev;

	// enter init mode
	dev->CCCR = FDCAN_CCCR_INIT;
	if (!fdcan_cccr_wait(dev, FDCAN_CCCR_INIT)) {
		return;
	}
	// zero all other flags: standard CAN2.0B operation
	dev->CCCR = FDCAN_CCCR_INIT | FDCAN_CCCR_CCE;
	if (!fdcan_cccr_wait(dev, FDCAN_CCCR_INIT | FDCAN_CCCR_CCE)) {
		return;
	}

	dev->CCCR |= FDCAN_CCCR_DAR; // disable automatic retransmit
	// enable this when the CAN headers on the bus have not been chosen with
	// proper prioritization in mind
	// dev->CCCR |= FDCAN_CCCR_TXP;  // pause 2 bits between transmissions

	// bit timing: 14 tq per bit, 1 (sync) + 9 + 4, SJW 3
	fdcan_nbtp_nsjw_set(dev, 3);
	fdcan_nbtp_nbrp_set(dev, fdcan_nbrp(kernel_hz, bitrate));
	fdcan_nbtp_ntseg1_set(dev, 9 - 1);
	fdcan_nbtp_ntseg2_set(dev, 4 - 1);

	// timestamp in units of TIM3's counter (the app owns TIM3)
	fdcan_tscc_tcp_set(dev, 0); // no divisor
	fdcan_tscc_tss_set(dev, 2); // tim3

	// Only RX0 and RX1 go to FDCANx_IT1, TX and everything else to IT0
	dev->ILS = FDCAN_ILS_RXFIFO1 | FDCAN_ILS_RXFIFO0;
	dev->ILE = FDCAN_ILE_EINT0 | FDCAN_ILE_EINT1;
	fdcan_txbtie_tie_set(dev, 7); // enable tx irq on all 3 txbuffers

	dev->IE = FDCAN_IE_ARAE | FDCAN_IE_MRAFE;                    // fatal error, reset CCCR.ASM
	dev->IE |= FDCAN_IE_PEDE | FDCAN_IE_PEAE | FDCAN_IE_BOE;     // protocol errors on the bus, bus off
	dev->IE |= FDCAN_IE_TEFNE | FDCAN_IE_RF1NE | FDCAN_IE_RF0NE; // transmit or receive events available

	dev->RXGFC = FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE; // reject remote frames (RTR set)
	fdcan_rxgfc_anfe_set(dev, 0);                     // all 29-bit messages to fifo0
	fdcan_rxgfc_anfs_set(dev, 1);                     // all 11-bit messages to fifo1

	dev->CCCR &= ~FDCAN_CCCR_INIT; // done configuring (clears CCE as well)
}

void fdcan_set_filter_extended(struct FDCan *c, int idx, enum CANFilterAction action, uint32_t value, uint32_t mask) {
	assert(can_header_isext(value) && can_header_isext(mask));
	c->msgram->ExtendedMessageIDFilter[idx].filter = (action & 0x7) << 29 | can_header_to29(value);
	c->msgram->ExtendedMessageIDFilter[idx].mask = 0x2 << 30 | can_header_to29(mask); // 0x2<<30: classic filter/mask
}

void fdcan_get_filter_extended(const struct FDCan *c, int idx, enum CANFilterAction *action, uint32_t *value, uint32_t *mask) {
	*action = (c->msgram->ExtendedMessageIDFilter[idx].filter >> 29) & 0x7;
	*value = can_header_from29(c->msgram->ExtendedMessageIDFilter[idx].filter);
	*mask = can_header_from29(c->msgram->ExtendedMessageIDFilter[idx].mask);
}

int fdcan_rx(struct FDCan *c, uint8_t *fmi, uint32_t *header, size_t *len, uint8_t *payload, uint16_t *ts) {
	struct FDCAN_Type *dev = c->dev;

	int fifo = 0;
	int idx = 0;

	volatile struct FifoElem *rxbuf = NULL;

	if (fdcan_rxf0s_f0fl_get(dev) != 0) {
		if (dev->RXF0S & FDCAN_RXF0S_RF0L) {
			++(c->status.rx_ovfl[0]);
		}
		fifo = 0;
		idx = fdcan_rxf0s_f0gi_get(dev);
		rxbuf = &c->msgram->RX0FIFO[idx];
	} else if (fdcan_rxf1s_f1fl_get(dev) != 0) {
		if (dev->RXF1S & FDCAN_RXF1S_RF1L) {
			++(c->status.rx_ovfl[1]);
		}
		fifo = 1;
		idx = fdcan_rxf1s_f1gi_get(dev);
		rxbuf = &c->msgram->RX1FIFO[idx];
	} else {
		return -1;
	}

	*ts = rxbuf->control & 0xffff; // RXTS, raw (see fdcan.h)

	// top bit: no match, if 0 then index of matching filter
	int match = (rxbuf->control & 0x80000000) ? 0x80 : (int)((rxbuf->control & 0x1f000000) >> 24);
	if (rxbuf->fdheader & CAN_EXTID) {
		match |= 0x40;
	}
	*fmi = (uint8_t)match;

	*header = stmfd2can(rxbuf->fdheader);
	size_t plen = *len;
	*len = (rxbuf->control >> 16) & 0xf;

	if (*len < plen) {
		plen = *len;
	}

	switch (plen) {
	default:
		payload[7] = rxbuf->buf[1] >> 24;
		// fallthrough
	case 7:
		payload[6] = rxbuf->buf[1] >> 16;
		// fallthrough
	case 6:
		payload[5] = rxbuf->buf[1] >> 8;
		// fallthrough
	case 5:
		payload[4] = rxbuf->buf[1];
		// fallthrough
	case 4:
		payload[3] = rxbuf->buf[0] >> 24;
		// fallthrough
	case 3:
		payload[2] = rxbuf->buf[0] >> 16;
		// fallthrough
	case 2:
		payload[1] = rxbuf->buf[0] >> 8;
		// fallthrough
	case 1:
		payload[0] = rxbuf->buf[0];
		// fallthrough
	case 0:;
	}

	if (fifo == 0) {
		fdcan_rxf0a_f0ai_set(dev, idx);
	} else {
		fdcan_rxf1a_f1ai_set(dev, idx);
	}

	c->status.rx_count[fifo]++;
	update_can_status(c);

	return fifo;
}

int fdcan_tx(struct FDCan *c, uint8_t tag, uint32_t header, size_t len, const uint8_t *payload) {
	assert(len <= 8);

	struct FDCAN_Type *dev = c->dev;
	if ((dev->TXFQS & FDCAN_TXFQS_TFQF) != 0) { // tx fifo full
		c->status.tx_ovfl++;
		return -1;
	}

	uint8_t idx = fdcan_txfqs_tfqpi_get(dev);
	volatile struct FifoElem *m = &c->msgram->TXFIFO[idx];

	m->fdheader = can2stmfd(header);
	m->control = (((uint32_t)tag) << 24) | ((len & 0xf) << 16) | (1 << 23); // MM=tag, DLC, EFC: store tx event

	uint64_t v = 0;
	switch (len) {
	case 8:
		v = ((uint64_t)payload[7]) << 56;
		// fallthrough
	case 7:
		v |= ((uint64_t)payload[6]) << 48;
		// fallthrough
	case 6:
		v |= ((uint64_t)payload[5]) << 40;
		// fallthrough
	case 5:
		v |= ((uint64_t)payload[4]) << 32;
		// fallthrough
	case 4:
		v |= ((uint64_t)payload[3]) << 24;
		// fallthrough
	case 3:
		v |= ((uint64_t)payload[2]) << 16;
		// fallthrough
	case 2:
		v |= ((uint64_t)payload[1]) << 8;
		// fallthrough
	case 1:
		v |= ((uint64_t)payload[0]) << 0;
		// fallthrough
	case 0:;
	}
	m->buf[0] = (uint32_t)v;
	m->buf[1] = (uint32_t)(v >> 32);

	fdcan_txbar_ar_set(dev, 1u << idx);

	return idx;
}

int fdcan_tx_done(struct FDCan *c, uint8_t *tag, uint32_t *header, uint16_t *ts) {
	struct FDCAN_Type *dev = c->dev;

	if (fdcan_txefs_effl_get(dev) == 0) { // tx event fifo fill level
		return -1;
	}

	uint8_t idx = fdcan_txefs_efgi_get(dev);
	volatile struct TXEvent_t *evbuf = &c->msgram->TXEvent[idx];

	*ts = evbuf->control & 0xffff; // TXTS, raw (see fdcan.h)
	*header = stmfd2can(evbuf->fdheader);
	*tag = evbuf->control >> 24;

	fdcan_txefa_efai_set(dev, idx);

	c->status.tx_count++;
	update_can_status(c);

	return idx;
}
