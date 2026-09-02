# M4a self-test — SPI1 masters the board's own SPI3 sensor bus (see main.c
# for the jumper table). Reuses ../src/{board,sensors}.c against the real
# models; RAM memory model like the harness proper.

PART      = STM32G474
PARTFULL  = STM32G474RET6
MEMTARGET = STM32G474Exx_RAM

# The n-array files this build needs are the ones ../src owns: one copy of
# nlib/ and one set of generated headers serve both firmwares. Everything
# here is delegated there, so NARRAY_DIR only ever matters in ../src.
NLIB   = ../src/nlib

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

ARCH    = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS  = -std=gnu23 -O2 -g3 -DPRINTF_ENABLE_FLOAT -ffreestanding -ffunction-sections -fdata-sections \
          $(ARCH) -Wall -Wextra -Wno-pedantic -I../src -I../bmp390inv -I$(NLIB)
LDFLAGS = -nostartfiles -lm -lnosys $(ARCH) -T ../src/$(MEMTARGET).ld -Wl,-L,../src \
          -Wl,--gc-sections,-Map,selftest.map

CSRC = main.c ../src/board.c ../src/sensors.c ../bmp390inv/bmp390lin.c \
       $(NLIB)/spi.c $(NLIB)/spislave.c $(NLIB)/gpio.c $(NLIB)/clock.c $(NLIB)/console.c \
       $(NLIB)/fault.c $(NLIB)/tprintf.c

# generated files and lib copies are shared with the harness build in ../src.
# sections_ram.ld is a lib copy but linker input, so it lives beside the
# generated scripts there rather than in nlib/.
GEN = ../src/device.h ../src/pinmux.h ../src/devs.ld ../src/$(MEMTARGET).ld \
      ../src/sections_ram.ld

# this build's own slice of the shared nlib — listed so that a bare
# `make` here restores exactly what it needs after a `make -C ../src
# n-array-clean`, without having to build the harness first
NLIB_FILES = $(addprefix $(NLIB)/, clock.c console.c fault.c gpio.c spi.c \
             spislave.c tprintf.c binary.h clock.h console.h dma.h dma_g4.h \
             exti.h fault.h fifo.h gpio.h nvic.h serial.h spi.h spislave.h \
             startup.h tprintf.h usart_v3.h)

$(GEN) $(NLIB_FILES):
	$(MAKE) -C ../src $(patsubst ../src/%,%,$@)

selftest.elf: $(GEN) $(NLIB_FILES) $(CSRC) ../src/board.h ../src/sensors.h
	$(CC) $(CFLAGS) $(CSRC) $(LDFLAGS) -o $@
	@$(SIZE) $@ | awk 'BEGIN {printf("%6s %6s %6s  %6s %6s %s\n", "text", "data", "bss", "flash", "ram", "file")} $$1*1 == $$1 {printf("%6d %6d %6d  %6d %6d %s\n", $$1, $$2, $$3-$$1, $$1+$$2, $$2+$$3, $$6)}'

selftest.bin: selftest.elf
	$(OBJCOPY) -O binary $< $@

.DEFAULT_GOAL := selftest.bin

# Flash over ST-Link with OpenOCD (G4 flash is driven by the stm32l4x cmd set).
flash: selftest.elf
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
	rm -f *.o *.elf *.bin *.hex *.map

.PHONY: flash clean

# a failed "narray ... > $@" must not leave a truncated target behind
.DELETE_ON_ERROR:
