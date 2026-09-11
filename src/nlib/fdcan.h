#pragma once

// fdcan — classic CAN 2.0B driver for the STM32G4 FDCAN controllers.
//  - bit timing derives from the kernel clock passed by the app (use
//    clock_fdcan_hz() from clock.h);
//  - message timestamps are returned raw: the 16-bit CAN timestamp counter,
//    clocked per TSCC from TIM3's counter — the app owns TIM3 and any
//    reconstruction into a wider timebase. With TIM3 ticking at 1 MHz
//    (lib/pwm), ts is in µs mod 2^16.
//
// One struct FDCan per controller bundles the register block, the
// controller's slice of the message RAM, and the status counters (the
// serial.h pattern). The application owns the clocks, pins, NVIC and
// vectors (the lib model). Each controller has two interrupt lines:
// the driver routes RX FIFO events to IT1 and everything else (TX events,
// errors) to IT0.
//
// Bring-up sketch (FDCAN1 on PA11/PA12, classic 1 Mbit):
//   static struct FDCan can1 = FDCAN_INITIALIZER(FDCAN1);
//   ...
//   RCC.APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
//   rcc_ccipr_fdcansel_set(2);                     // kernel = PCLK1
//   fdcan_init(&can1, clock_fdcan_hz(), 1000000);  // 14 tq/bit: 1+9+4, SJW 3
//   // vectors: [VECTOR(FDCAN1_IT0_IRQn)] -> fdcan_tx_done() + status
//   //          [VECTOR(FDCAN1_IT1_IRQn)] -> fdcan_rx() loop
//   nvic_enable(FDCAN1_IT0_IRQn); nvic_enable(FDCAN1_IT1_IRQn);
//
// Headers on the API are in the portable can.h format.

#include <stddef.h>

#include "can.h"
#include "device.h"

// updated by the transmit and receive functions
struct CANStatus {
	volatile uint32_t tx_ovfl;
	volatile uint32_t tx_count;
	volatile uint32_t esr; // last error status register, only gets updated on error
	volatile uint32_t lec_count[8];
	volatile uint32_t rx_count[2];
	volatile uint32_t rx_ovfl[2];
	volatile uint32_t ara_err;
	volatile uint32_t mraf_err;
};

// The controller's message RAM slice (layout in fdcan.c); the FDCANn_RAM
// addresses come from the generated devs.ld, like the register blocks.
struct FDCAN_MsgRAM;
extern volatile struct FDCAN_MsgRAM FDCAN1_RAM, FDCAN2_RAM, FDCAN3_RAM;

struct FDCan {
	struct FDCAN_Type *const dev;
	volatile struct FDCAN_MsgRAM *const msgram;
	struct CANStatus status;
};

// One per controller, by device name: FDCAN_INITIALIZER(FDCAN1) binds the
// register block and its message RAM slice.
#define FDCAN_INITIALIZER(dev) {&dev, &dev##_RAM, {}}

// fdcan_init initializes the controller for classic CAN 2.0B at the given
// bit rate. kernel_hz is the FDCAN kernel clock (clock_fdcan_hz()); the bit
// is 14 quanta (sync + 9 + 4, SJW 3), so kernel_hz must be an integer
// multiple of 14 * bitrate (and the prescaler <= 512): 168 MHz PCLK1 works
// for 1 Mbit (prescaler 12), 500k, 250k, ... down to 24 kbit.
void fdcan_init(struct FDCan *c, uint32_t kernel_hz, uint32_t bitrate);

// attempt to reset an already configured controller to this bit rate
void fdcan_setspeed(struct FDCan *c, uint32_t kernel_hz, uint32_t bitrate);
uint32_t fdcan_getspeed(const struct FDCan *c, uint32_t kernel_hz);

// fdcan_monitor sets/clears bus-monitoring mode (CCCR.MON, ISO 11898-1): the
// controller receives everything but never drives a dominant bit — no ACKs,
// no error frames — invisible to the bus. A passive listener can combine this
// with fdcan_setspeed to probe for the bus bit rate: at the wrong rate
// nothing is received, at the right one messages arrive. fdcan_tx while
// monitoring goes nowhere.
void fdcan_monitor(struct FDCan *c, int on);

// Error Counter Register for diagnostics
uint32_t fdcan_ecr(const struct FDCan *c);

// fdcan_tx schedules a message for transmission cf RM0440 44.3.5 and 44.3.6.
// the ESI and RTR bits are forced to 0 (RTR not supported by this stack).
// returns 0,1,2, the tx buffer used, on success, -1 if no buffer is free.
// tag identifies the message in the corresponding fdcan_tx_done() event.
int fdcan_tx(struct FDCan *c, uint8_t tag, uint32_t header, size_t len, const uint8_t *payload);

// fdcan_tx_done pops the first tx event from the tx event fifo and updates
// the status error counters; call from the IT0 irq handler until it returns
// -1. ts is the raw CAN timestamp of the transmission (see header comment).
int fdcan_tx_done(struct FDCan *c, uint8_t *tag, uint32_t *header, uint16_t *ts);

enum CANFilterAction {
	CAN_SKIP = 0, // 000: Disable filter element
	CAN_FIFO0,    // 001: Store in Rx FIFO 0 if filter matches
	CAN_FIFO1,    // 010: Store in Rx FIFO 1 if filter matches
	CAN_REJECT,   // 011: Reject ID if filter matches
	CAN_PRIO,     // 100: Set priority if filter matches, can be ORed with FIFO0/1
	              // 101/110: priority + store in FIFO 0/1;  111: not used
};

// NOTE: the filter LIST is only consulted up to RXGFC.LSE entries, and
// RXGFC is writable only in configuration mode (CCCR INIT+CCE). This call
// writes the message-RAM element only: the app must also set LSE (and the
// ANFE/ANFS non-matching policy) inside its own INIT+CCE window, after
// fdcan_init. With LSE = 0 the list is ignored entirely.
void fdcan_set_filter_extended(struct FDCan *c, int idx, enum CANFilterAction action, uint32_t value, uint32_t mask);
void fdcan_get_filter_extended(const struct FDCan *c, int idx, enum CANFilterAction *action, uint32_t *value, uint32_t *mask);

// fdcan_rx pops the first message available in rx fifo 0, else rx fifo 1;
// call from the IT1 irq handler until it returns -1 (else the index 0/1 of
// the fifo that returned the message). On entry len holds the capacity of
// payload[], on return the actual message size. fmi is the index of the
// matching filter, bit 7 set on fallthrough, bit 6 on an extended filter.
// ts is the raw CAN timestamp of reception (see header comment).
int fdcan_rx(struct FDCan *c, uint8_t *fmi, uint32_t *header, size_t *len, uint8_t *payload, uint16_t *ts);
