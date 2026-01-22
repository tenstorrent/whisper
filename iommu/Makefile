CXX      = /opt/rh/devtoolset-11/root/bin/g++
OFLAGS   = -g
CXXFLAGS = -MMD -MP -std=c++20 -Wall -Wextra -pedantic $(OFLAGS)
SRCS     = Iommu.cpp
OBJS     = $(SRCS:.cpp=.o)
DEPS     = $(OBJS:.o=.d)
TARGET   = libiommu.a

.PHONY:  all clean tests

all:     $(TARGET)

tests:   $(TARGET)
	$(MAKE) -C tests

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(AR) rcs $(TARGET) $(OBJS)

clean:
	$(RM) $(OBJS) $(DEPS) $(TARGET)
	$(MAKE) -C tests clean

-include $(DEPS)
