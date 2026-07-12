#pragma once

// sensors — the four emulated register-file devices on the SPI3 sensor bus
// (v3 single-SPI topology): BMI088 gyro + accel, BMP390, RM3100. Register
// maps and semantics per doc/REGMAPS.md; all multi-byte register content
// goes through explicit binary.h codecs (Bosch parts little-endian, RM3100
// big-endian) — never casts.
//
// Split of labor:
//  - the spislave engine serves the wire (main.c owns the vectors);
//  - deselect frame hooks (here) do clear-on-read, DRDY pin release and
//    write side-effect flags;
//  - the sampler (M5 scheduler; a static stand-in until then) pushes
//    samples via the *_commit functions, which quantize per the LIVE config
//    registers, set status bits and drive DRDY pins per the LIVE INT config;
//  - sensors_poll (thread level) executes deferred soft resets and the
//    gyro drdy auto-clear.

#include "bmp390lin.h"
#include "spislave.h"

#include <stdbool.h>
#include <stdint.h>

extern struct SPISlave sensor_bus; // SPI3 + the four devices, CS-demuxed
extern struct SPIDev gyro_dev, accel_dev, baro_dev, mag_dev;
extern const struct BMP390Trim baro_trim; // the trim served as the baro's NVM

// regfile resets + spislave_init. The app enables RCC clocks (SPI3, SYSCFG),
// routes DMAMUX and EXTI, and owns the vectors.
void sensors_init(void);

// bring-up frame trace: formats the next undrained frame as
// " <dev><cmd>@<addr>+<len>" (e.g. " a80@00+02") into buf and returns its
// length, 0 when drained. Thread level only.
int sensors_trace_next(char buf[static 16]);

// thread-level upkeep: deferred soft resets, gyro drdy auto-clear (~300 µs)
void sensors_poll(uint32_t now_us);

// sample rates the DUT has configured (Hz), for the sampler's schedule;
// 0 = device not sampling (suspended / mode off)
uint32_t gyro_rate_hz(void);
uint32_t accel_rate_hz(void);
uint32_t baro_rate_hz(void);
uint32_t mag_rate_hz(void); // CMM rate; also nonzero once while a POLL is pending

// full-scale ranges per the LIVE config registers, for the sampler's
// quantization (int16 full scale = +-range)
float gyro_fullscale_dps(void);  // 125 .. 2000
float accel_fullscale_g(void);   // 3 .. 24
float mag_lsb_per_ut(void);      // 0.3671 * CC + 1.5, from the live CC regs

// commit one sample (already quantized to device LSBs). Returns false if
// the device was selected (retry next tick) or not sampling.
bool gyro_commit(const int16_t xyz[3], uint32_t now_us);
bool accel_commit(const int16_t xyz[3], float t_degc, uint32_t now_us);
bool baro_commit(float t_degc, double p_pa, uint32_t now_us); // raw words via bmp390inv
bool mag_commit(const int32_t xyz[3], uint32_t now_us);       // 24-bit signed per axis
