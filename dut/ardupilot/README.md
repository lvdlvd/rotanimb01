# ArduPilot: the NucleoF767ZI bench board

Two commits against upstream ArduPilot master, verified with `git am` on
master as of 2026-07-27 (9bbfed9c91):

1. **NucleoF767ZI: HITL bench board** — `libraries/AP_HAL_ChibiOS/hwdef/
   NucleoF767ZI/hwdef.dat` + `hwdef-bl.dat` + the prebuilt bootloader
   `Tools/bootloaders/NucleoF767ZI_bl.bin`. Full ArduPlane on the
   2 MB F767: BMI088 + BMP388/390 + RM3100 on SPI3 with chip selects on
   port D, **MODE0 on every SPIDEV** (real parts auto-detect the clock
   phase per transaction; a register-level emulator cannot), gyro and
   mag capped at 1 MHz, PWM 1-8 on TIM3/TIM4, CAN1 on PD0/PD1 for the
   DroneCAN GPS/airspeed feed, SERIAL0 on the user USB, ADC fully
   disabled, 8 MHz MCO HSE bypass. Bootloader owns flash 0..96 K, the
   application starts at 0x08018000, parameters in flash sectors 1-2.
   The same hwdef drives real sensor breakouts on SPI3 (that is how the
   emulation was differentially tested).
2. **Bootloader: two first-boot bugs** — `FLASH_RESERVE_START_KB 0` in
   hwdef-bl (the generator default of 16 linked the bootloader 16 K past
   where it was flashed: reset landed in erased flash), and empty asm
   barriers in AP_Bootloader's own strlen/memset loops (gcc 15's
   loop-idiom recognition compiled strlen into a tail call to itself).
   The second one applies to any board built with a newer toolchain.

Apply and build:

```
git am /path/to/rotanimb01/dut/ardupilot/*.patch
./waf configure --board NucleoF767ZI && ./waf plane
```

Flashing and first boot: doc/SETUP-ARDUPLANE.md. Runtime parameters
the board expects: `doc/f5-bench.parm` (CAN_P1_DRIVER 1, GPS1_TYPE 9,
ARSPD_TYPE 8, and the rest).
