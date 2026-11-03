# Two-hart CSR race test, run under whisper.

# Any RISC-V gcc will do, the test doesn't use a libc.
TOOLCHAIN ?= riscv64-unknown-linux-gnu-
CC        = $(TOOLCHAIN)gcc
CFLAGS    = -march=rv64gc -mabi=lp64 -nostdlib -T csr-race.ld
# Silence the RWX segment warning, if the linker is new enough to give it.
CFLAGS   += $(shell $(CC) -Wl,--no-warn-rwx-segments -Wl,--version >/dev/null 2>&1 && echo -Wl,--no-warn-rwx-segments)
WHISPER  ?= ../../build-$(shell uname -s)/whisper

TESTS = csr-race-topei.elf csr-race-ireg.elf

.PHONY: all check clean

all: $(TESTS)

csr-race-topei.elf: csr-race.S csr-race.ld
	$(CC) $(CFLAGS) csr-race.S -o $@

csr-race-ireg.elf: csr-race.S csr-race.ld
	$(CC) $(CFLAGS) -DIREG_RMW csr-race.S -o $@

check: $(TESTS)
	for t in $(TESTS); do \
	  echo "== $$t"; $(WHISPER) --configfile csr-race.json $$t || exit 1; \
	done

clean:
	rm -f $(TESTS)
