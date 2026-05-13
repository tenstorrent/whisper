.PHONY: all run perf clean

SRC=test.S
ELF=$(SRC:%.S=%.elf)
DIS=$(SRC:%.S=%.dis)
LOG=$(SRC:%.S=%.log)
CMD=$(SRC:%.S=%.cmd)

SEED ?= -1
R ?= 5
M ?= 0
P ?= 1.0
WHISPER ?= whisper
WHISPER_CONFIG ?= whisper.json

all: $(ELF) $(DIS)

run: $(LOG)

%.elf: %.S test.ld
	riscv64-elf-gcc -g -static -mcmodel=medany -march=rv64imafdc -mabi=lp64d -nostdlib -T test.ld -o $(ELF) $(SRC)

%.dis: %.elf
	riscv64-elf-objdump -dl $< > $@

%.log: %.elf
	$(WHISPER) --configfile $(WHISPER_CONFIG) --csv -f $@ $< 2> /dev/null

clean:
	rm -f $(ELF) $(LOG) $(DIS)
