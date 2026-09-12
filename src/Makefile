# rotanimb01 — HITL sensor simulator for ArduPlane / PX4 (see ../doc/DESIGN.md).
# STM32G474RET6 (LQFP64, 64-pin breakout), bare metal.
#
# SELF-CONTAINED BUILD. Everything this firmware compiles lives in this
# repository: the bare-metal support library is in nlib/ (clocks, GPIO, DMA,
# SPI slave engine, FDCAN, USB CDC, console, CORDIC math, fault trapping),
# and the device register header, pin map and linker scripts sit beside this
# Makefile (device.h, pinmux.h, devs.ld, STM32G474Exx_RAM.ld — generated from
# the reference manual, marked DO NOT EDIT; sections_ram.ld is the section
# layout for the run-from-RAM image). Nothing on the build path points
# outside the repository. Toolchain: arm-none-eabi-gcc 15.2 (built and tested with)
# (-std=gnu23 — older compilers untested); openocd for flashing.

# TRANSPORT=usb (default): host link = pseudocan lines over the USB VCP
# (PA11/PA12 = USB DP/DM). TRANSPORT=can: FDCAN1 on the same pins, for a
# bench with a transceiver — the two are a wiring-level either/or.
TRANSPORT ?= usb

PART      = STM32G474
PARTFULL  = STM32G474RET6
MEMTARGET = STM32G474Exx_RAM

NLIB   = nlib

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

ARCH    = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS  = -std=gnu23 -O2 -g3 -DPRINTF_ENABLE_FLOAT -ffreestanding -ffunction-sections -fdata-sections \
          $(ARCH) -Wall -Wextra -Wno-pedantic -I. -I../physics -I../bmp390inv -I../fdm -I$(NLIB)
LDFLAGS = -nostartfiles -lm -lnosys $(ARCH) -T $(MEMTARGET).ld -Wl,-L,. \
          -Wl,--gc-sections,-Map,rotanimb01.map

# cordic_math defines sinf & co as object symbols, so they beat libm at link
# time: the whole image's float transcendentals run on the CORDIC.
CSRC = main.c board.c sensors.c controls.c dronecan.c ../physics/physics.c ../bmp390inv/bmp390lin.c ../fdm/fdm.c \
       $(NLIB)/cordic_math.c $(NLIB)/cordic_stm32.c \
       $(NLIB)/pwm.c $(NLIB)/fdcan.c $(NLIB)/spislave.c $(NLIB)/gpio.c $(NLIB)/clock.c $(NLIB)/console.c \
       $(NLIB)/usb.c $(NLIB)/fmtcan.c $(NLIB)/fault.c $(NLIB)/tprintf.c

ifeq ($(TRANSPORT),can)
CFLAGS += -DTRANSPORT_CAN
endif

# everything the image depends on besides CSRC: the lib headers, the
# generated device files and the linker scripts
DEPS = $(wildcard $(NLIB)/*.h) board.h controls.h sensors.h dronecan.h canmsg.h \
       ../fdm/fdm.h ../physics/physics.h ../bmp390inv/bmp390lin.h \
       device.h pinmux.h devs.ld $(MEMTARGET).ld sections_ram.ld

# F1 gate (fdm-DESIGN.md): fdm.o alone must compile -Wdouble-promotion clean
# and reference no soft-double helpers — its math is FPU float32 + cordic only
fdmgate: ../fdm/fdm.c ../fdm/fdm.h
	$(CC) $(CFLAGS) -Wdouble-promotion -Werror -c ../fdm/fdm.c -o fdmgate.o
	@if $(PREFIX)nm -u fdmgate.o | grep -qE '__aeabi_d|__extendsfdf2|__truncdfsf2'; then \
		echo "F1 gate FAIL: soft-double helpers in fdm.o"; rm -f fdmgate.o; exit 1; \
	else echo "F1 gate: fdm.o promotion-clean, no soft-double"; rm -f fdmgate.o; fi

# the DroneCAN encoder must stay byte-exact vs frames generated with
# ArduPilot's own libcanard (vectors in dccheck.c) — host-checked every build
dccheck: dronecan.c dronecan.h dccheck.c
	cc -O2 -std=c2x -Wall -Wextra -Werror -o dccheck dccheck.c dronecan.c
	./dccheck

rotanimb01.elf: $(CSRC) $(DEPS) fdmgate dccheck
	$(CC) $(CFLAGS) $(CSRC) $(LDFLAGS) -o $@
	@$(SIZE) $@ | awk 'BEGIN {printf("%6s %6s %6s  %6s %6s %s\n", "text", "data", "bss", "flash", "ram", "file")} $$1*1 == $$1 {printf("%6d %6d %6d  %6d %6d %s\n", $$1, $$2, $$3-$$1, $$1+$$2, $$2+$$3, $$6)}'

rotanimb01.bin: rotanimb01.elf
	$(OBJCOPY) -O binary $< $@

.DEFAULT_GOAL := rotanimb01.bin

# Flash over ST-Link with OpenOCD (G4 flash is driven by the stm32l4x cmd set).
# With more than one ST-Link attached, select one: make flash OPENOCD_ADAPTER="-c 'adapter serial <serial>'"
flash: rotanimb01.elf
	openocd \
		-f interface/stlink.cfg $(OPENOCD_ADAPTER) \
		-f target/stm32g4x.cfg \
		-c init \
		-c halt \
		-c "flash probe 0" \
		-c "stm32l4x mass_erase 0" \
		-c "sleep 100" \
		-c "program $< verify" \
		-c reset \
		-c shutdown

clean:
	rm -f *.o *.elf *.bin *.hex *.map dccheck

.PHONY: flash clean fdmgate dccheck

.DELETE_ON_ERROR:
