CXX      = g++
OFLAGS   = -g -O2
CPPFLAGS = -I..
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic $(CPPFLAGS) $(OFLAGS)

TARGET   = claim-race

.PHONY:  all clean test tsan check-csr

all:     $(TARGET)

claim-race: claim-race.cpp ../Imsic.cpp ../Imsic.hpp
	$(CXX) $(CXXFLAGS) claim-race.cpp ../Imsic.cpp -o $@ -pthread

test: claim-race
	./claim-race

# Same test built with ThreadSanitizer, reports the data race directly.
claim-race-tsan: claim-race.cpp ../Imsic.cpp ../Imsic.hpp
	$(CXX) $(CXXFLAGS) -fsanitize=thread claim-race.cpp ../Imsic.cpp -o $@ -pthread

tsan: claim-race-tsan
	setarch `uname -m` -R ./claim-race-tsan

# Two-hart CSR race test, run under whisper.
RISCV_CC = riscv64-unknown-elf-gcc
RVFLAGS  = -march=rv64gc -mabi=lp64 -nostdlib -T csr-race.ld -Wl,--no-warn-rwx-segments
WHISPER  = ../../build-Linux/whisper

CSR_RACE = csr-race-topei.elf csr-race-ireg.elf

csr-race-topei.elf: csr-race.S csr-race.ld
	$(RISCV_CC) $(RVFLAGS) csr-race.S -o $@

csr-race-ireg.elf: csr-race.S csr-race.ld
	$(RISCV_CC) $(RVFLAGS) -DIREG_RMW csr-race.S -o $@

check-csr: $(CSR_RACE)
	for t in $(CSR_RACE); do \
	  echo "== $$t"; $(WHISPER) --configfile csr-race.json $$t || exit 1; \
	done

clean:
	rm -f *.o $(TARGET) claim-race-tsan $(CSR_RACE)
