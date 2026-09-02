#include "gpio.h"

// Spread a per-pin value across the bit positions selected by `pins`: for each
// set pin bit, OR `val` shifted to that pin's field. `bits` is the field width
// (2 for MODER/PUPDR/OSPEEDR, 4 for AFRL/AFRH).
static uint32_t maskn(uint16_t pins, uint8_t bits, uint32_t val) {
	uint32_t mask = 0;
	while (pins) {
		if (pins & 1) {
			mask |= val;
		}
		pins >>= 1;
		val <<= bits;
	}
	return mask;
}

void gpioConfig(pinconf_t pc) {
	enum GPIO_Pin pins = (enum GPIO_Pin)(uint32_t)pc; // low 32: port + pin mask
	uint32_t conf = (uint32_t)(pc >> 32);             // high 32: GPIO_Conf bits
	assert(gpio_one_port(pins));

	struct GPIO_Type *gpio = &GPIO_ALL[gpio_index(pins)].gpio;
	uint16_t mask = pins & Pin_All;

	if ((conf & 0x3000) == GPIO_ANALOG) {
		conf = GPIO_ANALOG; // analog overrides any other flags
	}
	if ((conf & 0x0030) == 0x0030) {
		conf &= ~0x0030; // both pull-up and pull-down requested -> none (reserved)
	}

	uint32_t mode = (conf & 0x3000) >> 12;
	uint32_t af = (conf & 0x0f00) >> 8;
	uint32_t od = (conf & 0x0080) >> 7;
	uint32_t pupd = (conf & 0x0030) >> 4;
	uint32_t spd = (conf & 0x0003);

	uint32_t clr2 = maskn(mask, 2, 0x3);
	gpio->MODER &= ~clr2;
	gpio->PUPDR &= ~clr2;
	gpio->OSPEEDR &= ~clr2;

	if (od) {
		gpio->OTYPER |= mask;
	} else {
		gpio->OTYPER &= ~mask;
	}

	gpio->OSPEEDR |= maskn(mask, 2, spd);
	gpio->PUPDR |= maskn(mask, 2, pupd);
	gpio->MODER |= maskn(mask, 2, mode);

	if (mode == 2) { // alternate function: program AFRL/AFRH nibbles
		gpio->AFRL &= ~maskn(mask & 0xff, 4, 0xf);
		gpio->AFRL |= maskn(mask & 0xff, 4, af);
		gpio->AFRH &= ~maskn((mask >> 8) & 0xff, 4, 0xf);
		gpio->AFRH |= maskn((mask >> 8) & 0xff, 4, af);
	}
}

uint32_t gpioLock(enum GPIO_Pin pins) {
	assert(gpio_one_port(pins));
	struct GPIO_Type *gpio = &GPIO_ALL[gpio_index(pins)].gpio;
	uint16_t mask = pins & Pin_All;
	// LCKR lock sequence: write 1+key, write 0+key wait, write 1+key, then read.
	gpio->LCKR = mask | (1u << 16);
	gpio->LCKR = mask;
	gpio->LCKR = mask | (1u << 16);
	return gpio->LCKR;
}
