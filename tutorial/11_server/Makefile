.PHONY: all run clean

TOOLCHAIN=riscv-none-elf-
CC=$(TOOLCHAIN)gcc
OBJDUMP=$(TOOLCHAIN)objdump
WHISPER=whisper

all: test.elf

test.elf: test.S test.ld
	$(CC) -march=rv64g -mabi=lp64 -nostdlib -T test.ld -o test.elf test.S
	$(OBJDUMP) -d test.elf > test.dis

client: client.cpp ../../WhisperMessage.cpp
	$(CXX) -std=c++20 -I../../ -o client client.cpp ../../WhisperMessage.cpp

run: test.elf
	WHISPER=$(WHISPER) ./run.sh

clean:
	rm -f *.elf *.log *.dis
