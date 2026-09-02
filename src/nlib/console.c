#include "console.h"

// The default console (NULL until the app sets it). Single definition so
// tprintf()/printf() output goes to one place across translation units.
struct Serial *console = 0;

// tprintf()'s output hook (declared in tprintf.h): route to the default console.
void _putchar(char c) {
	if (console) {
		serial_putc(c, console);
		serial_dma_tx_start(console);
	}
}
