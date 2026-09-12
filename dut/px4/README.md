# PX4: the st/nucleo-f767zi bench board

Five commits against PX4 main as of 2026-08-28 (7c4bf078f4), applied and built
on that base (1348357 B, 73 % of flash). Three are upstreamable fixes the board
needs, two are the board itself.

1. **bmp388: split the bus interface out, add SPI** — the in-tree
   driver was I2C-only; this moves it to the interface-factory pattern
   (as rm3100/dps310) with `bmp388_spi.cpp` (read = 0x80|reg + one
   dummy byte) and `bmp388_i2c.cpp`.
2. **uavcan: add battery to module DEPENDS** — the DroneCAN battery
   bridge links `lib/battery`, which only `battery_status` pulled in,
   and `battery_status` cannot build with `BOARD_NUMBER_BRICKS 0`.
3. **param: treat an empty PARAM_BACKUP_FILE as absent** — on an
   SD-less board `open("")` fails with ENXIO, not ENOENT, and `param
   load-or-init` treated it as a broken backup.
4. **boards: add st/nucleo-f767zi** — pin-for-pin the ArduPilot
   NucleoF767ZI hwdef: SPI3 PB3/4/5, CS PD3 accel / PD4 gyro / PD5 baro
   / PD6 mag, PWM 1-8 on TIM3/TIM4 (PC6-9, PD12-15), CAN1 PD0/PD1,
   USART3 = ST-Link VCP console (57600), USB CDC = MAVLink, HRT on
   TIM5. No bootloader (image at 0x08000000, parameters in flash sector
   11), no SD, no FRAM, no ADC, no safety switch, no RC.
   `rc.board_sensors` starts all four drivers in SPI mode 0 (`-m 0`,
   mandatory for the emulator) with ROLL_180 on the BMI088 only.
5. **init.d: add 2110_rotanimb01_kitfox airframe** — the model's
   constants, the harness's PWM order (1 ail, 2 ele, 3 THR, 4 rud),
   `UAVCAN_ENABLE 2` + `UAVCAN_SUB_DPRES 1`, and two documented
   bench-specific deviations (`ASPD_DO_CHECKS 0`, `FW_LND_USETER 0`).

`sitl/0001-*.patch` is separate: the `5100_px4hil_kitfox` POSIX SITL
airframe for `tools/px4hil` ([doc/SETUP-PX4.md](../../doc/SETUP-PX4.md) section 7). Its output
map is the bridge's (0 ail, 1 ele, 2 rud, 3 thr), not the board's.

Apply and build (gcc 10 toolchain — see [doc/SETUP-PX4.md](../../doc/SETUP-PX4.md)):

```
git checkout 7c4bf078f4
git am /path/to/rotanimb01/dut/px4/*.patch
make st_nucleo-f767zi_default
```
