CXX      = /opt/rh/devtoolset-11/root/bin/g++
OFLAGS   = -g
CPPFLAGS = -I../.. -I..
CXXFLAGS = -std=c++20 -Wall -Wextra -pedantic $(CPPFLAGS) $(OFLAGS)

TARGET   = device-table-walk process-table-walk fault-queue msi-translation ats-unified-test iofence-test test_iotinval_commands test_iodir

.PHONY:  all clean

all:     $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

device-table-walk: device-table-walk.o
	$(CXX) $< -o $@ ../libiommu.a

process-table-walk: process-table-walk.o
	$(CXX) $< -o $@ ../libiommu.a

fault-queue: fault-queue.o
	$(CXX) $< -o $@ ../libiommu.a

msi-translation: msi-translation.o
	$(CXX) $< -o $@ ../libiommu.a

ats-unified-test: ats-unified-test.o
	$(CXX) $< -o $@ ../libiommu.a

# Legacy ATS tests (kept for reference)
ats-command-test: ats-command-test.o
	$(CXX) $< -o $@ ../libiommu.a

ats-test: ats-test.o
	$(CXX) $< -o $@ ../libiommu.a

clean:
	rm -f *.o $(TARGET)


iofence-test: iofence-test.o
	$(CXX) $< -o $@ ../libiommu.a

test_iotinval_commands: test_iotinval_commands.o
	$(CXX) $< -o $@ ../libiommu.a

test_iodir: test_iodir.o
	$(CXX) $< -o $@ ../libiommu.a


