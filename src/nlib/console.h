#pragma once

// Glue between the Paland tprintf (lib/tprintf.h) and the buffered serial
// (lib/serial.h): printf-family output that lands in a serial port's FIFO and
// kicks the DMA TX engine. tprintf talks straight to fifo.h through this.
//
// One writer per console FIFO — do not printf to the same port from different
// interrupt priorities.

#include "serial.h"
#include "tprintf.h"

// The default console for tprintf()/printf(). Set it after creating the TX
// Serial:  console = &uart1tx;
extern struct Serial *console;

// fctprintf output hook: push one char to a serial port's FIFO (drop if full).
static inline void serial_putc(char c, void *s) {
	struct Serial *ser = s;
	if (!fifo_full(&ser->buf)) {
		fifo_put_head(&ser->buf, (uint8_t)c);
	}
}

// printf to a specific serial port (any port), then start the TX DMA.
#define serial_printf(s, ...) (fctprintf(serial_putc, (s), __VA_ARGS__), serial_dma_tx_start(s))
