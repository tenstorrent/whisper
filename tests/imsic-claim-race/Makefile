CXX      = g++
OFLAGS   = -g -O2
CPPFLAGS = -I../../imsic
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic $(CPPFLAGS) $(OFLAGS)
SRCS     = claim-race.cpp ../../imsic/Imsic.cpp
DEPS     = $(SRCS) ../../imsic/Imsic.hpp

.PHONY:  all check tsan clean

all:     claim-race

claim-race: $(DEPS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ -pthread

check: claim-race
	./claim-race

# Same test built with ThreadSanitizer, reports the data race directly.
claim-race-tsan: $(DEPS)
	$(CXX) $(CXXFLAGS) -fsanitize=thread $(SRCS) -o $@ -pthread

tsan: claim-race-tsan
	setarch `uname -m` -R ./claim-race-tsan

clean:
	rm -f claim-race claim-race-tsan
