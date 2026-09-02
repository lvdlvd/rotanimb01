#pragma once

// GPIO runtime for [N]Array. Consumes the generated pinmux.h (the GPIO_Pin /
// GPIO_Conf / pinconf_t constants) and the device register header. A board's
// whole pinout is an array of pinconf_t — a pin OR'd with its configuration,
// e.g. PA9_USART1_TX | PIN_HIGH — applied with gpioConfigAll(); digitalHi/Lo/
// toggle/in drive already-configured pins.
//
// The narray-generated register header is included as "device.h" (struct
// GPIO_Type, GPIOA); pinmux.h supplies the pin/mux/flag constants.
// The application owns the clock: enable the GPIO port clocks in RCC before
// calling gpioConfig (this library touches only the GPIO registers).

#include "device.h"
#include "pinmux.h"
#include <assert.h>

// STM32 GPIO ports are 0x400 apart; GPIO_ALL overlays them as an array so a
// port index selects the port. It is bound to GPIOA's address by devs.ld.
union GPIO_Page {
	struct GPIO_Type gpio;
	uint8_t page[0x400];
};
extern union GPIO_Page GPIO_ALL[8];

// A GPIO_Pin carries a one-hot port bit in [31:24]; more than one set means
// pins from different ports were OR'd together, which these functions reject.
static inline int gpio_one_port(enum GPIO_Pin pins) { return !((pins >> 24) & ((pins >> 24) - 1)); }
static inline unsigned gpio_index(enum GPIO_Pin pins) { return (pins >> 16) & 7; }

// gpio_assert_one_port rejects a multi-port pin mask at COMPILE time when the
// mask is a compile-time constant (the usual case: OR'd Pxn literals), and at
// runtime via assert otherwise — a computed pin value compiles fine. The
// error-attribute call only survives into the object when gcc can prove the
// condition true, so it needs optimization (-O1+, always on here) to fold;
// without it the check quietly degrades to the runtime assert alone.
// A failing assert would constant-fold too, silently dead-coding everything
// after the call site — this turns that into a build failure instead.
extern void gpio_error_pins_span_ports(void) __attribute__((error("pin mask spans multiple GPIO ports — one digital*()/gpioConfig call per port")));
// always_inline: partial inlining would outline the cold error branch and
// lose the constantness of pins, silently degrading the check to runtime
__attribute__((always_inline)) static inline void gpio_assert_one_port(enum GPIO_Pin pins) {
	// gpio_one_port() open-coded: __builtin_constant_p must see the value,
	// not a not-yet-inlined function call, to fold before the error check
	if (__builtin_constant_p(pins) && ((pins >> 24) & ((pins >> 24) - 1))) {
		gpio_error_pins_span_ports();
	}
	assert(gpio_one_port(pins));
}

// gpioConfig applies one pinconf_t: low 32 bits select the pin(s), high 32 the
// configuration (mode/af/otype/pupd/speed). Pins of one port may be OR'd.
void gpioConfig(pinconf_t pc);

static inline void gpioConfigAll(const pinconf_t *board, int n) {
	for (int i = 0; i < n; i++) {
		gpioConfig(board[i]);
	}
}

// gpioLock freezes a pin's configuration until the next reset.
uint32_t gpioLock(enum GPIO_Pin pins);

static inline void digitalHi(enum GPIO_Pin hi) {
	gpio_assert_one_port(hi);
	GPIO_ALL[gpio_index(hi)].gpio.BSRR = hi & Pin_All;
}

static inline void digitalLo(enum GPIO_Pin lo) {
	gpio_assert_one_port(lo);
	GPIO_ALL[gpio_index(lo)].gpio.BSRR = (uint32_t)(lo & Pin_All) << 16; // reset in BSRR high half
}

static inline void digitalToggle(enum GPIO_Pin pins) {
	gpio_assert_one_port(pins);
	GPIO_ALL[gpio_index(pins)].gpio.ODR ^= pins & Pin_All;
}

// digitalIn returns the masked input bits (a Pin_n mask of the high inputs).
static inline uint32_t digitalIn(enum GPIO_Pin pins) {
	gpio_assert_one_port(pins);
	return GPIO_ALL[gpio_index(pins)].gpio.IDR & (pins & Pin_All);
}
