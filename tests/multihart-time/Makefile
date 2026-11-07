.PHONY: all run check clean

# Any RISC-V gcc will do, the test doesn't use a libc.
TOOLCHAIN ?= riscv64-unknown-linux-gnu-
CC = $(TOOLCHAIN)gcc
OBJDUMP = $(TOOLCHAIN)objdump
WHISPER ?= ../../build-$(shell uname -s)/whisper
HARTS ?= 4
ROUNDS ?= 10000

all: test-$(HARTS).elf

test-$(HARTS).elf: test.S test.ld
	$(CC) -march=rv64g -mabi=lp64 -nostdlib -T test.ld -DNHARTS=$(HARTS) -DROUNDS=$(ROUNDS) -o $@ test.S
	$(OBJDUMP) -d $@ > test-$(HARTS).dis

# Whisper's default configuration doesn't implement the time CSR, Zicntr adds it.
ISA = rv64imafd_zicsr_zicntr

# --quitany: the run ends when any hart writes tohost (hart 0 on success, the
# failing hart on failure), without waiting for the others, which are parked.
run: test-$(HARTS).elf
	$(WHISPER) --isa $(ISA) --hart $(HARTS) --quitany $<

clean:
	rm -f *.elf *.log *.dis

check: run
