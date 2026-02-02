CXX      = g++
OFLAGS   = -O3
CXXFLAGS = -MMD -MP -std=c++20 -Wall -pedantic -Wunused-parameter -I. $(OFLAGS) -fPIC
SRCS    = VirtMem.cpp Tlb.cpp
OBJS    = $(SRCS:.cpp=.o)
DEPS    = $(OBJS:.o=.d)
TARGET  = libvirtual_memory.a

# Needed to link against boost libraries which were compiled using older ABI
ifeq ($(BOOST_ROOT), /tools_vendor/FOSS/boost/1.82)
  CXXFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0
endif

.PHONY: all clean
all: $(TARGET)
$(TARGET): $(OBJS)
	$(AR) rcs $(TARGET) $(OBJS)
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)

.PHONY: clean

-include $(DEPS)
