#pragma once

// usbfs — low-level access helpers for the STM32 USB_FS device (RM0440
// section 45; the same IP as the F103's, RM0008 section 23, modulo the
// packet-memory stride). The register block and bit definitions come from
// the generated device.h (struct USB_Type, USB_EPR_*/USB_CNTR_*/USB_ISTR_*);
// this header adds what the generator cannot express: the packet memory
// layout, and the endpoint-register write discipline — USB_EPR is a mix of
// rw, write-1-to-toggle and write-0-to-clear bits, so none of the accesses
// are straightforward read-modify-writes.
//
// Lineage: the STM32F103 driver at github.com/daedaleanai/stm32f103_usb,
// ported to the G4 registers.

#include "device.h"

#include <stdint.h>

enum {
	// values for the STAT_TX/STAT_RX fields (pre-shift)
	USB_EP_STAT_DISABLED = 0,
	USB_EP_STAT_STALL = 1,
	USB_EP_STAT_NAK = 2,
	USB_EP_STAT_VALID = 3,

	// values for the TYPE|KIND field pair (pre-shift)
	USB_EP_TYPE_BULK = 0,
	USB_EP_TYPE_BULK_DBLBUF = 1,
	USB_EP_TYPE_CONTROL = 2,
	USB_EP_TYPE_CONTROL_STSOUT = 3,
	USB_EP_TYPE_ISO = 4,
	USB_EP_TYPE_INTERRUPT = 6,

	USB_EPR_EP_TYPEKIND = USB_EPR_EP_TYPE | USB_EPR_EP_KIND,
	USB_EP_CFG = USB_EPR_EP_TYPEKIND | USB_EPR_EA,  // bits to preserve when toggling or clearing
	USB_EP_CTR = USB_EPR_CTR_RX | USB_EPR_CTR_TX,   // bits to write as 1 to preserve
};

// Clear CTR_xX, set type, kind and endpoint address. Does not touch the toggle/stat bits.
static inline void usb_ep_config(uint8_t ep, uint16_t typekind, uint8_t ea) {
	USB.EPR[ep & 0x7] = ((typekind << 8) & USB_EPR_EP_TYPEKIND) | (ea & USB_EPR_EA);
}

static inline void usb_ep_setkind(uint8_t ep) {
	USB.EPR[ep & 0x7] = (USB.EPR[ep & 0x7] & (USB_EPR_EA | USB_EPR_EP_TYPE)) | USB_EP_CTR | USB_EPR_EP_KIND;
}

// Clear endpoint address and reset typekind to zero (BULK), clear DTOG_xX and STAT_xX to DISABLED.
static inline void usb_ep_reset(uint8_t ep) {
	USB.EPR[ep & 0x7] = USB.EPR[ep & 0x7] & (USB_EPR_DTOG_RX | USB_EPR_STAT_RX | USB_EPR_DTOG_TX | USB_EPR_STAT_TX);
}

static inline void usb_ep_set_stat_rx(uint8_t ep, uint16_t val) {
	USB.EPR[ep & 0x7] = USB_EP_CTR | ((USB.EPR[ep & 0x7] & (USB_EP_CFG | USB_EPR_STAT_RX)) ^ ((val << 12) & USB_EPR_STAT_RX));
}
static inline void usb_ep_set_stat_tx(uint8_t ep, uint16_t val) {
	USB.EPR[ep & 0x7] = USB_EP_CTR | ((USB.EPR[ep & 0x7] & (USB_EP_CFG | USB_EPR_STAT_TX)) ^ ((val << 4) & USB_EPR_STAT_TX));
}
static inline void usb_ep_clr_ctr_rx(uint8_t ep) { USB.EPR[ep & 0x7] = USB_EPR_CTR_TX | (USB.EPR[ep & 0x7] & USB_EP_CFG); }
static inline void usb_ep_clr_ctr_tx(uint8_t ep) { USB.EPR[ep & 0x7] = USB_EPR_CTR_RX | (USB.EPR[ep & 0x7] & USB_EP_CFG); }
static inline void usb_ep_clr_dtog_rx(uint8_t ep) {
	USB.EPR[ep & 0x7] = USB_EP_CTR | (USB.EPR[ep & 0x7] & (USB_EP_CFG | USB_EPR_DTOG_RX));
}
static inline void usb_ep_set_dtog_rx(uint8_t ep) {
	USB.EPR[ep & 0x7] = USB_EP_CTR | ((USB.EPR[ep & 0x7] & (USB_EP_CFG | USB_EPR_DTOG_RX)) ^ USB_EPR_DTOG_RX);
}
static inline void usb_ep_clr_dtog_tx(uint8_t ep) {
	USB.EPR[ep & 0x7] = USB_EP_CTR | (USB.EPR[ep & 0x7] & (USB_EP_CFG | USB_EPR_DTOG_TX));
}
static inline uint16_t usb_ep_get_stat_rx(uint8_t ep) { return (USB.EPR[ep & 0x7] & USB_EPR_STAT_RX) >> 12; }
static inline uint16_t usb_ep_get_stat_tx(uint8_t ep) { return (USB.EPR[ep & 0x7] & USB_EPR_STAT_TX) >> 4; }
static inline uint16_t usb_ep_get_ep_typekind(uint8_t ep) { return (USB.EPR[ep & 0x7] & USB_EPR_EP_TYPEKIND) >> 8; }
static inline uint16_t usb_ep_get_ea(uint8_t ep) { return USB.EPR[ep & 0x7] & USB_EPR_EA; }

static inline int usb_ep_is_tx(uint8_t ep) { return (ep & 0x80) != 0; } // true: IN/TX, false: OUT/RX

// ---- packet memory (RM0440 45.6.2) ------------------------------------------
//
// 1024 bytes at the USB_RAM linker symbol (generated devs.ld). The buffer
// descriptor table sits at its base by virtue of USB.BTABLE = 0. Only byte
// and half-word accesses are allowed — never 32-bit.

extern union USB_PMA_Type {
	struct {
		volatile uint16_t ADDR_TX; // in bytes, always even
		volatile uint16_t COUNT_TX;
		volatile uint16_t ADDR_RX;
		volatile uint16_t COUNT_RX;
	} btable[8];
	volatile uint16_t buf[512];
} USB_RAM;

enum {
	USB_PMA_COUNT_BLSIZE = 1u << 15,
	USB_PMA_COUNT_NUMBLOCKS = ((1u << 5) - 1) << 10,
	USB_PMA_COUNT_COUNT = (1u << 10) - 1,
};

// tx/rx packet buffers, only accessible as uint16_t or as bytes
static inline volatile uint16_t *usb_ep_tx_buf(int ep) { return USB_RAM.buf + USB_RAM.btable[ep].ADDR_TX / 2; }
static inline volatile uint16_t *usb_ep_rx_buf(int ep) { return USB_RAM.buf + USB_RAM.btable[ep].ADDR_RX / 2; }

// these are in units of bytes
static inline void usb_ep_set_tx_count(int ep, uint16_t len) { USB_RAM.btable[ep].COUNT_TX = len & USB_PMA_COUNT_COUNT; }
static inline uint16_t usb_ep_get_tx_count(int ep) { return USB_RAM.btable[ep].COUNT_TX & USB_PMA_COUNT_COUNT; }
static inline uint16_t usb_ep_get_rx_count(int ep) { return USB_RAM.btable[ep].COUNT_RX & USB_PMA_COUNT_COUNT; }

static inline uint16_t usb_ep_get_rx_size(int ep) {
	uint16_t crx = USB_RAM.btable[ep].COUNT_RX;
	uint16_t v = (crx & USB_PMA_COUNT_NUMBLOCKS) >> 10;
	if (crx & USB_PMA_COUNT_BLSIZE) {
		return (v + 1) * 32;
	}
	return v * 2;
}

static inline void usb_ep_set_rx_size(int ep, uint16_t size) {
	if (size < 62) {
		USB_RAM.btable[ep].COUNT_RX = (size / 2) << 10;
	} else {
		USB_RAM.btable[ep].COUNT_RX = (((size / 32) - 1) << 10) | USB_PMA_COUNT_BLSIZE;
	}
}

static inline uint16_t usb_istr_get_ep_id(void) { return USB.ISTR & USB_ISTR_EP_ID; }
