// rotanimb01 board pinout and bring-up (roles in board.h, rationale in
// ../DESIGN.md). The harness is the SPI *slave* on all three sensor buses:
// SPI1 = BMI088 gyro, SPI2 = BMI088 accel (hardware NSS each), SPI3 = BMP390 +
// RM3100 shared (SSM, CS demux on PC0/PC1 EXTI). TIM2/TIM3 capture 8 PWM
// outputs of the DUT; FDCAN1 talks to the host.

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

	// SPI1 slave = BMI088 gyro die. Hardware NSS from the DUT's gyro CS.
	PA4_SPI1_NSS | PIN_PULLUP,      //% 18  PA4   SPI1_NSS   AF5  af,pu | DUT CS gyro (idle deselected)
	PA5_SPI1_SCK | PIN_PULLDOWN,    //% 19  PA5   SPI1_SCK   AF5  af,pd | from DUT SCK
	PA6_SPI1_MISO | PIN_HIGH,       //% 20  PA6   SPI1_MISO  AF5  af,hi | tied w/ PB14, PC11 onto DUT MISO
	PA7_SPI1_MOSI | PIN_PULLDOWN,   //% 21  PA7   SPI1_MOSI  AF5  af,pd | from DUT MOSI

	// SPI2 slave = BMI088 accel die. Hardware NSS from the DUT's accel CS.
	PB12_SPI2_NSS | PIN_PULLUP,     //% 34  PB12  SPI2_NSS   AF5  af,pu | DUT CS accel (idle deselected)
	PB13_SPI2_SCK | PIN_PULLDOWN,   //% 35  PB13  SPI2_SCK   AF5  af,pd | from DUT SCK
	PB14_SPI2_MISO | PIN_HIGH,      //% 36  PB14  SPI2_MISO  AF5  af,hi | tied w/ PA6, PC11 onto DUT MISO
	PB15_SPI2_MOSI | PIN_PULLDOWN,  //% 37  PB15  SPI2_MOSI  AF5  af,pd | from DUT MOSI

	// SPI3 slave = BMP390 + RM3100 shared (SSM permanently selected; the two
	// CS lines are plain inputs here, both-edge EXTI demux from M3 on).
	PC10_SPI3_SCK | PIN_PULLDOWN,   //% 52  PC10  SPI3_SCK   AF6  af,pd | from DUT SCK
	PC11_SPI3_MISO | PIN_HIGH,      //% 53  PC11  SPI3_MISO  AF6  af,hi | tied w/ PA6, PB14 onto DUT MISO
	PC12_SPI3_MOSI | PIN_PULLDOWN,  //% 54  PC12  SPI3_MOSI  AF6  af,pd | from DUT MOSI
	PC0 | PIN_INPUT | PIN_PULLUP,   //%  8  PC0   GPIO       -    in,pu | DUT CS baro = CS_BARO (EXTI0)
	PC1 | PIN_INPUT | PIN_PULLUP,   //%  9  PC1   GPIO       -    in,pu | DUT CS mag = CS_MAG (EXTI1)

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

	// Peripheral clocks for everything in use through M2 (console, PWM
	// capture on TIM2/3, FDCAN1); SPI1/2/3 + SYSCFG/EXTI and TIM7 come in
	// with M3/M5.
	RCC.AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
	RCC.AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
	RCC.APB1ENR1 |= RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_FDCANEN;
	RCC.APB2ENR |= RCC_APB2ENR_USART1EN;
	rcc_ccipr_fdcansel_set(2); // FDCAN kernel clock = PCLK1 (168 MHz: /12 -> 14 tq at 1 Mbit)

	gpioConfigAll(board, sizeof board / sizeof board[0]);
}
