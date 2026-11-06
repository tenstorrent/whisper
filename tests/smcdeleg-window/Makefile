# Smcdeleg counter-window test, run under whisper.

# Any RISC-V gcc will do, the test doesn't use a libc.
TOOLCHAIN ?= riscv64-unknown-linux-gnu-
CC        = $(TOOLCHAIN)gcc
CFLAGS    = -nostdlib -T test.ld
# Silence the RWX segment warning, if the linker is new enough to give it.
CFLAGS   += $(shell $(CC) -Wl,--no-warn-rwx-segments -Wl,--version >/dev/null 2>&1 && echo -Wl,--no-warn-rwx-segments)
WHISPER  ?= ../../build-$(shell uname -s)/whisper

EXTS  = zicsr_zicntr_zihpm_smaia_ssaia_smcdeleg_sscsrind_sscofpmf
TESTS = test-rv64.elf test-rv32.elf

.PHONY: all check clean

all: $(TESTS)

test-rv64.elf: test.S test.ld
	$(CC) $(CFLAGS) -march=rv64ima_zicsr -mabi=lp64 test.S -o $@

test-rv32.elf: test.S test.ld
	$(CC) $(CFLAGS) -march=rv32ima_zicsr -mabi=ilp32 test.S -o $@

# Each runs without an IMSIC, where sireg* outside the CTR and Smcdeleg ranges is
# illegal, and with one.
check: $(TESTS)
	for xlen in 64 32; do \
	  for cfg in counters.json counters.json,imsic.json; do \
	    echo "== rv$$xlen $$cfg"; \
	    $(WHISPER) --isa rv$${xlen}imacsu_$(EXTS) --counters --configfile $$cfg test-rv$$xlen.elf || exit 1; \
	  done; \
	done

clean:
	rm -f $(TESTS)
