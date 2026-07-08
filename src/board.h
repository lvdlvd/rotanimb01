#pragma once

// rotanimb01 board definition — STM32G474RET6 64-pin breakout wired as the
// HITL sensor-simulator harness (../DESIGN.md "Pinout"). Role names for every
// signal the application drives or reads; the pinconf table and board_init()
// live in board.c.

#include "device.h"
#include "pinmux.h"

// Status LED (breakout board, active low).
#define LED PC13

// DUT-driven chip selects, all four demuxed on the shared SPI3 slave
// (v3 topology): GPIO inputs on distinct EXTI lines.
#define CS_BARO PC0  // EXTI0
#define CS_MAG	PC1  // EXTI1
#define CS_GYRO PA4  // EXTI4
#define CS_ACC	PB12 // EXTI12 (shared EXTI15_10 vector)

// Data-ready outputs to the DUT, driven per each device's INT config.
#define DRDY_ACC  PC4 // BMI088 INT1
#define DRDY_GYRO PC5 // BMI088 INT3
#define DRDY_BARO PB6 // BMP390 INT
#define DRDY_MAG  PB7 // RM3100 DRDY

// board_init: memory/clock/GPIO/console bring-up common to every milestone.
// Enables the peripheral clocks the pinout needs; does NOT touch the NVIC —
// the application owns its IRQs and vector table.
void board_init(void);
