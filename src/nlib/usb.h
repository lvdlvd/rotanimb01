#pragma once

// usb — a minimal CDC-ACM device (virtual serial port) on the STM32 USB_FS
// macrocell: 1 device, 1 configuration; endpoint 0x00/0x80 for control, the
// 0x01/0x81 BULK pair carrying the serial payload (64-byte buffers both
// ways), and a notification endpoint that never speaks. Enumerates as
// 0483:5740 with the manufacturer/product strings given at init and a
// serial number derived from the die's unique id — /dev/cu.usbmodem<serial>
// on mac, /dev/ttyACM* on linux, no special host client or driver.
//
// The descriptor set is the minimal ACM (the optional call-management
// functional descriptor is dropped), which at 62 bytes fits a single
// 64-byte control packet — so the control flow never needs multi-packet
// transfers. The line coding the host sets is stored and served back but
// drives nothing: there is no physical UART behind it.
//
// Lineage: the STM32F103 bulk driver at github.com/daedaleanai/stm32f103_usb,
// ported to the G4 registers and grown CDC control semantics.
//
// The app owns clocks and vectors (the lib model). Bring-up:
//
//     PWR.CR3 |= PWR_CR3_UCPD1_DBDIS;                     // dead-battery pull-downs off!
//     RCC.CRRCR |= RCC_CRRCR_HSI48ON;                     // 48 MHz kernel clock
//     while (!(RCC.CRRCR & RCC_CRRCR_HSI48RDY))
//         ;
//     rcc_ccipr_clk48sel_set(0);                          // CLK48 = HSI48
//     RCC.APB1ENR1 |= RCC_APB1ENR1_USBEN | RCC_APB1ENR1_CRSEN;
//     CRS.CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;           // trim HSI48 on USB SOF
//     usb_init("acme", "widget");
//     nvic_enable(USB_LP_IRQn);                           // wire vector to a usb_recv() caller
//
// USB DP/DM are PA12/PA11 — leave them in analog; the macrocell drives the
// pads directly. This implementation uses only the USB_LP interrupt; calling
// usb_recv() from the main loop instead also works, it drives the whole
// protocol stack.

#include <stddef.h>
#include <stdint.h>

#include "fifo.h"

// usb_init (re-)initializes the usb device (enable RCC USBEN + CLK48
// first). The strings appear as the manufacturer and product descriptors,
// ASCII, at most 31 chars used; NULL is allowed. Only the pointers are
// kept — the UTF-16 wire form is produced on the fly in packet memory.
void usb_init(const char *manufacturer, const char *product);

// usb_shutdown puts the USB transceiver in power-down mode (disable the
// USB_LP interrupt first; RCC USBEN may be cleared after).
void usb_shutdown(void);

// Cf. the visible device states, USB 2.0 section 9.1.1. In DEFAULT the
// address is zero and only endpoint 0 is configured. A resume returns
// SUSPENDED to whatever the state was before.
enum usb_state_t {
	USB_UNATTACHED, // not yet communicating with the host
	USB_DEFAULT,    // the host has issued a bus reset
	USB_ADDRESS,    // the host has assigned a device address
	USB_CONFIGURED, // the host has accepted the configuration
	USB_SUSPENDED,  // suspended by bus inactivity
};

enum usb_state_t usb_state(void);
const char *usb_state_str(enum usb_state_t s);

// enumeration diagnostics: event counters and the most recent SETUP
// request, for bring-up heartbeats
extern struct UsbDebug {
	volatile uint32_t resets, setups, ep0_tx, stalls;
	volatile uint16_t last_req, last_val, last_len;
} usb_debug;

// usb_dtr: the host has the port open (DTR asserted by
// SET_CONTROL_LINE_STATE, dropped on close and bus reset). Gate
// transmissions on this — bytes sent into a closed port pile up
// against NAK.
int usb_dtr(void);

// usb_recv drives the protocol stack: handles reset/control traffic and
// copies a received bulk message (max 64 bytes) from endpoint 1 into the
// fifo. If the fifo has less than 64 bytes free the message may be
// discarded — call with at least 64 free. Call from the USB_LP handler or
// the main loop. Returns the number of bytes copied.
size_t usb_recv(struct Fifo *fifo);

// usb_send moves up to 64 bytes from the fifo into a free transmit buffer
// on endpoint 1 and schedules it. Returns the number of bytes copied, 0 if
// the previous packet is still in flight (retry later) or unconfigured.
size_t usb_send(struct Fifo *fifo);
