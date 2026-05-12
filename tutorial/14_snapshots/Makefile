.PHONY: all run clean

TOOLCHAIN=riscv-none-elf-
CC=$(TOOLCHAIN)gcc
OBJDUMP=$(TOOLCHAIN)objdump
WHISPER=whisper

all: test.elf

test.elf: start.S main.c test.ld
	$(CC) -o test.elf main.c
	$(OBJDUMP) -d test.elf > test.dis

run: test.elf
	$(WHISPER) --snapshotperiod 10000 -- test.elf

load:
	$(WHISPER) --loadfrom snapshot2 test.elf

clean:
	rm -f *.elf *.log *.dis
