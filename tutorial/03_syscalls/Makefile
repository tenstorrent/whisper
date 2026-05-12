.PHONY: all run clean

TOOLCHAIN=riscv-none-elf-
CC=$(TOOLCHAIN)gcc
OBJDUMP=$(TOOLCHAIN)objdump
WHISPER=whisper

all: test.elf

test.elf: test.S test.ld
	$(CC) -march=rv64g -mabi=lp64 -nostdlib -T test.ld -o test.elf test.S
	$(OBJDUMP) -d test.elf > test.dis

run: test.elf
	$(WHISPER) --newlib test.elf

clean:
	rm -f *.elf *.log *.dis
