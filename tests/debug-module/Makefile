.PHONY: all check clean

# Any RISC-V gcc will do, the test doesn't use a libc.
TOOLCHAIN ?= riscv64-unknown-linux-gnu-
CC = $(TOOLCHAIN)gcc
CFLAGS = -nostdlib -T test.ld
# Silence the RWX segment warning, if the linker is new enough to give it.
CFLAGS += $(shell $(CC) -Wl,--no-warn-rwx-segments -Wl,--version >/dev/null 2>&1 && echo -Wl,--no-warn-rwx-segments)
WHISPER ?= ../../build-$(shell uname -s)/whisper
PYTHON ?= python3

all: loop64.elf loop32.elf

loop64.elf: loop.S test.ld
	$(CC) $(CFLAGS) -march=rv64g -mabi=lp64 -o $@ loop.S

loop32.elf: loop.S test.ld
	$(CC) $(CFLAGS) -march=rv32g -mabi=ilp32 -o $@ loop.S

check: all
	$(PYTHON) dmtest.py $(abspath $(WHISPER)) loop64.elf loop32.elf

clean:
	rm -f *.elf
