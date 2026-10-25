CXX      = g++
OFLAGS   = -g -O2
CPPFLAGS = -I..
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic $(CPPFLAGS) $(OFLAGS)

TARGET   = claim-race

.PHONY:  all clean test tsan

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

clean:
	rm -f *.o $(TARGET) claim-race-tsan
