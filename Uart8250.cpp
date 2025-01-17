#include <cassert>
#include <iostream>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include "Uart8250.hpp"


using namespace WdRiscv;


Uart8250::Uart8250(uint64_t addr, uint64_t size)
  : IoDevice(addr, size)
{
  auto func = [this]() { this->monitorStdin(); };
  stdinThread_ = std::thread(func);

  struct termios term;
  tcgetattr(fileno(stdin), &term);
  cfmakeraw(&term);
  term.c_lflag &= ~ECHO;
  tcsetattr(fileno(stdin), 0, &term);
}


Uart8250::~Uart8250()
{
  terminate_ = true;
  stdinThread_.join();
}


uint32_t
Uart8250::read(uint64_t addr)
{
  uint64_t offset = (addr - address()) / 4;
  bool dlab = lcr_ & 0x80;

  if (dlab == 0)
    {
      switch (offset)
	{
	case 0:
	  {
	    std::lock_guard<std::mutex> lock(mutex_);
	    uint32_t res = 0;
	    if (!rx_fifo.empty()) {
	      res = rx_fifo.front();
	      rx_fifo.pop();
	    }
	    if (rx_fifo.empty()) {
	      lsr_ &= ~1;  // Clear least sig bit
	      iir_ |= 1;   // Set least sig bit indicating no interrupt.
	      setInterruptPending(false);
	    }
	    return res;
	  }

	case 1: return ier_;
	case 2: return iir_;
	case 3: return lcr_;
	case 4: return mcr_;
	case 5: return lsr_;
	case 6: return msr_;
	case 7: return scr_;
	}
    }
  else
    {
      switch (offset)
	{
	case 0: return dll_;
	case 1: return dlm_;
	}
    }

  assert(0);
  return 0;
}


void
Uart8250::write(uint64_t addr, uint32_t value)
{
  uint64_t offset = (addr - address()) / 4;
  bool dlab = lcr_ & 0x80;

  if (dlab == 0)
    {
      switch (offset)
	{
	case 0:
	    {
	      int c = static_cast<int>(value & 0xff);
	      if (c)
		{
		  putchar(c);
		  fflush(stdout);
		}
	    }
	  break;

	case 1: ier_ = value; break;
	case 2: fcr_ = value; break;
	case 3: lcr_ = value; break;
	case 4: mcr_ = value; break;
	case 5:
	case 6: break;
	case 7: scr_ = value; break;
	default:
	  std::cerr << "Uart writing addr 0x" << std::hex << addr << std::dec << '\n';
	  assert(0);
	}
    }
  else
    {
      switch (offset)
	{
	case 0: dll_ = value; break;
	case 1: dlm_ = value; break;
	case 3: lcr_ = value; break;
	case 5: psd_ = value; break;
	default:
	  std::cerr << "Uart writing addr 0x" << std::hex << addr << std::dec << '\n';
	  assert(0);
	}
    }
}


void
Uart8250::monitorStdin()
{
  struct pollfd inPollfd;

  int fd = fileno(stdin);        // stdin file descriptor
  inPollfd.fd = fd;
  inPollfd.events = POLLIN;

  while (true)
    {
      if (terminate_)
	return;

      int code = poll(&inPollfd, 1, -1);
      if (code == 0)
	continue;   // Timed out.

      if (code == 1)
	{
	  if ((inPollfd.revents & POLLIN) != 0)
	    {
	      std::lock_guard<std::mutex> lock(mutex_);
	      char c;
	      if (::read(fd, &c, sizeof(c)) != 1)
		std::cerr << "Uart8250::monitorStdin: unexpected fail on read\n";
	      if (isatty(fd))
		{
		  static char prev = 0;

		  // Force a stop if control-a x is seen.
		  if (prev == 1 and c == 'x')
		    throw std::runtime_error("Keyboard stop");
		  prev = c;
		}
	      rx_fifo.push(c);
	      lsr_ |= 1;  // Set least sig bit of line status.
	      iir_ &= ~1;  // Clear bit 0 indicating interrupt is pending.
	      // setInterruptPending(true);
	    }
	}

      // TODO: handle error return codes from poll.
    }
}
