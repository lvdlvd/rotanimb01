# M4a self-test — SPI1 masters the board's own SPI3 sensor bus (see main.c
# for the jumper table). Reuses ../src/{board,sensors}.c against the real
# models; RAM memory model like the harness proper.

PART      = STM32G474
PARTFULL  = STM32G474RET6
MEMTARGET = STM32G474Exx_RAM

LIB    = ../../n-array/lib
PERIPH = ../../n-array/periph/stm32g4
NARRAY = ../../n-array/gen/narray

PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

ARCH    = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS  = -std=gnu23 -O2 -g3 -DPRINTF_ENABLE_FLOAT -ffreestanding -ffunction-sections -fdata-sections \
          $(ARCH) -Wall -Wextra -Wno-pedantic -I../src -I../bmp390inv -I$(LIB)
LDFLAGS = -nostartfiles -lm -lnosys $(ARCH) -T ../src/$(MEMTARGET).ld -Wl,-L,../src -Wl,-L,$(LIB) \
          -Wl,--gc-sections,-Map,selftest.map

CSRC = main.c ../src/board.c ../src/sensors.c ../bmp390inv/bmp390lin.c \
       $(LIB)/spi.c $(LIB)/spislave.c $(LIB)/gpio.c $(LIB)/clock.c $(LIB)/console.c \
       $(LIB)/fault.c $(LIB)/tprintf.c

# generated files are shared with the harness build in ../src
GEN = ../src/device.h ../src/pinmux.h ../src/devs.ld ../src/$(MEMTARGET).ld

$(GEN):
	$(MAKE) -C ../src $(notdir $@)

selftest.elf: $(GEN) $(CSRC) ../src/board.h ../src/sensors.h $(LIB)/sections.ld
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
