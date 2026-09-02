# rotanimb01 — HITL sensor simulator for ArduPlane (see ../doc/DESIGN.md).
# STM32G474RET6 (LQFP64, 64-pin breakout), built on [N]Array.
#
# SELF-CONTAINED [N]ARRAY BUILD. Everything this firmware compiles lives in
# this repository: the lib sources it uses are copied into nlib/ and the
# narray-generated files sit beside this Makefile, all of them tracked. The
# rules below carry NO prerequisite on the n-array checkout, so a file that
# is already here is never refreshed, whatever upstream does — n-array can
# evolve freely without breaking this build, and these copies can be edited
# for this project's needs without asking anyone.
#
# A file is fetched or regenerated ONLY when it is missing. The one gesture
# that reaches upstream is therefore:
#
#	make n-array-clean && make	# take the current n-array, see what breaks
#
# and that breakage lands at a moment you chose. `make clean` deliberately
# does NOT remove these files, so clean-and-rebuild works with no checkout
# in sight. NARRAY_DIR locates that checkout when it is needed: a peer of
# this repo by default, overridable in the environment or on the command
# line (`make NARRAY_DIR=/abs/path n-array-clean all`). RULE OF THREE: a
# local edit to nlib/ that you want for the third time belongs upstream in
# n-array instead — anonymized, since n-array is public and this is not.

# TRANSPORT=usb (default): host link = pseudocan lines over the USB VCP
# (PA11/PA12 = USB DP/DM). TRANSPORT=can: FDCAN1 on the same pins, for the
# lab bench with a transceiver — the two are a wiring-level either/or.
TRANSPORT ?= usb

NARRAY_DIR ?= ../../n-array

PART      = STM32G474
PARTFULL  = STM32G474RET6
MEMTARGET = STM32G474Exx_RAM

NLIB   = nlib
PERIPH = $(NARRAY_DIR)/periph/stm32g4
NARRAY = $(NARRAY_DIR)/gen/narray

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

# The lib files this project uses — the transitive include closure of the
# sources above, plus ../selftest's spi.c and ../fdm's cordic frontend,
# which build against this same nlib. Unused lib files are not copied.
NLIB_C  = clock.c console.c cordic_math.c cordic_stm32.c fault.c fdcan.c fmtcan.c \
          gpio.c pwm.c spi.c spislave.c tprintf.c usb.c
NLIB_H  = binary.h can.h canmsgq.h clock.h console.h cordic.h cordic_port.h dma.h \
          dma_g4.h exti.h fault.h fdcan.h fifo.h fmtcan.h gpio.h nvic.h pwm.h \
          remap_g4.h serial.h spi.h spislave.h startup.h tprintf.h usart_v3.h \
          usb.h usbfs.h
NLIB_FILES = $(addprefix $(NLIB)/, $(NLIB_C) $(NLIB_H))

# The section layout is copied from the lib too, but it is linker input, so
# it belongs beside the generated linker scripts rather than among the C
# sources in nlib/ — $(MEMTARGET).ld INCLUDEs it by bare name.
LIBLD = sections_ram.ld

# generated here by narray from the periph descriptions in the checkout
GEN = device.h pinmux.h devs.ld $(MEMTARGET).ld

# --- the only rules that reach NARRAY_DIR, and only when a file is gone ---

$(NLIB_FILES) $(LIBLD):
	@mkdir -p $(@D)
	cp $(NARRAY_DIR)/lib/$(@F) $@

# The generator is built in place in the checkout, on demand. It is bootstrapped
# from INSIDE the recipes below rather than named as their prerequisite: make
# remakes a prerequisite even when the target is present and up to date — even
# an order-only one — so naming it here would demand the checkout on every
# build and hand back the dependency this whole arrangement removes. Reached
# from a recipe, it is needed only when a generated file is actually missing.
$(NARRAY): $(wildcard $(NARRAY_DIR)/gen/*.go)
	cd $(NARRAY_DIR)/gen && go build -o narray .

device.h:
	@$(MAKE) --no-print-directory $(NARRAY)
	$(NARRAY) -part $(PART) -header $(PERIPH) > $@

pinmux.h:
	@$(MAKE) --no-print-directory $(NARRAY)
	$(NARRAY) -pinmux $(PERIPH) > $@

devs.ld:
	@$(MAKE) --no-print-directory $(NARRAY)
	$(NARRAY) -part $(PART) -devs $(PERIPH) > $@

$(MEMTARGET).ld:
	@$(MAKE) --no-print-directory $(NARRAY)
	$(NARRAY) -memory $(MEMTARGET) $(PERIPH) > $@

# --------------------------------------------------------------------------

# F1 gate (fdm-DESIGN.md): fdm.o alone must compile -Wdouble-promotion clean
# and reference no soft-double helpers — its math is FPU float32 + cordic only
fdmgate: $(GEN) $(NLIB_FILES)
	$(CC) $(CFLAGS) -Wdouble-promotion -Werror -c ../fdm/fdm.c -o fdmgate.o
	@if $(PREFIX)nm -u fdmgate.o | grep -qE '__aeabi_d|__extendsfdf2|__truncdfsf2'; then \
		echo "F1 gate FAIL: soft-double helpers in fdm.o"; rm -f fdmgate.o; exit 1; \
	else echo "F1 gate: fdm.o promotion-clean, no soft-double"; rm -f fdmgate.o; fi

# the DroneCAN encoder must stay byte-exact vs frames generated with
# ArduPilot's own libcanard (vectors in dccheck.c) — host-checked every build
dccheck: dronecan.c dronecan.h dccheck.c
	cc -O2 -std=c2x -Wall -Wextra -Werror -o dccheck dccheck.c dronecan.c
	./dccheck

rotanimb01.elf: $(GEN) $(LIBLD) $(NLIB_FILES) $(CSRC) board.h fdmgate dccheck
	$(CC) $(CFLAGS) $(CSRC) $(LDFLAGS) -o $@
	@$(SIZE) $@ | awk 'BEGIN {printf("%6s %6s %6s  %6s %6s %s\n", "text", "data", "bss", "flash", "ram", "file")} $$1*1 == $$1 {printf("%6d %6d %6d  %6d %6d %s\n", $$1, $$2, $$3-$$1, $$1+$$2, $$2+$$3, $$6)}'

rotanimb01.bin: rotanimb01.elf
	$(OBJCOPY) -O binary $< $@

.DEFAULT_GOAL := rotanimb01.bin

# regenerate the in-file pinout annotations from the datasheet
pinfmt: pinmux.h | $(NARRAY)
	$(NARRAY) -part $(PARTFULL) -pinfmt board.c $(PERIPH)

# Flash over ST-Link with OpenOCD (G4 flash is driven by the stm32l4x cmd set).
flash: rotanimb01.elf
	openocd \
		-f interface/stlink.cfg \
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

# the deliberate upgrade gesture: drop everything that came from n-array,
# then `make` to take whatever NARRAY_DIR holds today
n-array-clean:
	rm -f $(GEN) $(LIBLD) $(NLIB_FILES)

.PHONY: flash clean n-array-clean pinfmt fdmgate dccheck

# a failed "narray ... > $@" must not leave a truncated target behind
.DELETE_ON_ERROR:
