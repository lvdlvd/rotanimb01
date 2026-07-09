// rotanimb01 board pinout and bring-up (roles in board.h, rationale in
// ../DESIGN.md, v3 topology). The harness serves all four devices on ONE
// SPI3 slave (SSM), demuxed by four CS inputs on PC0..PC3 (both-edge EXTI).
// SPI1/SPI2 pins stay reserved (analog) as the hybrid fallback. TIM2/TIM3
// capture 8 PWM outputs of the DUT; FDCAN1 talks to the host.

#include "board.h"

#include "clock.h"
#include "gpio.h"

// The whole harness pinout (run `make pinfmt` to re-annotate).
static const pinconf_t board[] = {
	// Unused -> analog (low power). PAMost = port A minus SWDIO/SWCLK, so the
	// debugger survives on the running chip.
	PAMost | PIN_ANALOG,
	PBAll | PIN_ANALOG,
	PCAll | PIN_ANALOG,

	// Console + LED.
	PC13 | PIN_OUTPUT,              //%  2  PC13  GPIO       -    out   | breakout LED (active low)
	PA9_USART1_TX | PIN_HIGH,       //% 43  PA9   USART1_TX  AF7  af,hi | ST-Link VCP TX
	PA10_USART1_RX | PIN_PULLUP,    //% 44  PA10  USART1_RX  AF7  af,pu | ST-Link VCP RX

	// SPI3 slave = the whole sensor bus (v3): all four devices demuxed by
	// their CS inputs, one port, both-edge EXTI. SPI1/SPI2 pins (PA4-PA7,
	// PB12-PB15) stay in the analog blanket: reserved hybrid fallback.
	PC10_SPI3_SCK | PIN_PULLDOWN,   //% 52  PC10  SPI3_SCK   AF6  af,pd | from DUT SCK
	PC11_SPI3_MISO | PIN_HIGH,      //% 53  PC11  SPI3_MISO  AF6  af,hi | to DUT MISO
	PC12_SPI3_MOSI | PIN_PULLDOWN,  //% 54  PC12  SPI3_MOSI  AF6  af,pd | from DUT MOSI
	PC0 | PIN_INPUT | PIN_PULLUP,   //%  8  PC0   GPIO       -    in,pu | DUT CS baro = CS_BARO (EXTI0)
	PC1 | PIN_INPUT | PIN_PULLUP,   //%  9  PC1   GPIO       -    in,pu | DUT CS mag = CS_MAG (EXTI1)
	PC2 | PIN_INPUT | PIN_PULLUP,   //% 10  PC2   GPIO       -    in,pu | DUT CS gyro = CS_GYRO (EXTI2)
	PC3 | PIN_INPUT | PIN_PULLUP,   //% 11  PC3   GPIO       -    in,pu | DUT CS accel = CS_ACC (EXTI3)

	// Data-ready outputs to the DUT (reset low = inactive for the default
	// active-high INT configs; polarity follows the DUT's INT config writes).
	PC4 | PIN_OUTPUT,               //% 22  PC4   GPIO       -    out   | DRDY_ACC: BMI088 INT1
	PC5 | PIN_OUTPUT,               //% 23  PC5   GPIO       -    out   | DRDY_GYRO: BMI088 INT3
	PB6 | PIN_OUTPUT,               //% 59  PB6   GPIO       -    out   | DRDY_BARO: BMP390 INT
	PB7 | PIN_OUTPUT,               //% 60  PB7   GPIO       -    out   | DRDY_MAG: RM3100 DRDY

	// 8x PWM capture from the DUT's servo outputs. Pulled down so a
	// disconnected channel reads no edges (reported as width 0).
	PA0_TIM2_CH1 | PIN_PULLDOWN,    //% 12  PA0   TIM2_CH1   AF1  af,pd | PWM in 1
	PA1_TIM2_CH2 | PIN_PULLDOWN,    //% 13  PA1   TIM2_CH2   AF1  af,pd | PWM in 2
	PB10_TIM2_CH3 | PIN_PULLDOWN,   //% 30  PB10  TIM2_CH3   AF1  af,pd | PWM in 3
	PB11_TIM2_CH4 | PIN_PULLDOWN,   //% 33  PB11  TIM2_CH4   AF1  af,pd | PWM in 4
	PC6_TIM3_CH1 | PIN_PULLDOWN,    //% 38  PC6   TIM3_CH1   AF2  af,pd | PWM in 5
	PC7_TIM3_CH2 | PIN_PULLDOWN,    //% 39  PC7   TIM3_CH2   AF2  af,pd | PWM in 6
	PC8_TIM3_CH3 | PIN_PULLDOWN,    //% 40  PC8   TIM3_CH3   AF2  af,pd | PWM in 7
	PC9_TIM3_CH4 | PIN_PULLDOWN,    //% 41  PC9   TIM3_CH4   AF2  af,pd | PWM in 8

	// FDCAN1 to the host, external transceiver. NOT PB8/PB9: PB8 is BOOT0 and
	// a transceiver's recessive-high RXD would boot the system loader.
	PA11_FDCAN1_RX | PIN_PULLUP,    //% 45  PA11  FDCAN1_RX  AF9  af,pu | CAN RX
	PA12_FDCAN1_TX | PIN_HIGH,      //% 46  PA12  FDCAN1_TX  AF9  af,hi | CAN TX
};

void board_init(void) {
	clock_init_168(); // HSE if present (auto-measured), else HSI16

	// Peripheral clocks: console, PWM capture on TIM2/3, FDCAN1, the SPI3
	// sensor bus + SYSCFG for its CS EXTIs; TIM7 comes with M5's scheduler.
	RCC.AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
	RCC.AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_FDCANEN | RCC_APB1ENR1_SPI3EN;
	RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;
	rcc_ccipr_fdcansel_set(2); // FDCAN kernel clock = PCLK1 (168 MHz: /12 -> 14 tq at 1 Mbit)

	gpioConfigAll(board, sizeof board / sizeof board[0]);
}
