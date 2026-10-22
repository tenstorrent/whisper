#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <utility>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif
#include <sys/socket.h>
#include <netinet/in.h>
#include <span>
#include "Uart8250.hpp"


using namespace WdRiscv;

FDChannel::FDChannel(int in_fd, int out_fd)
  : in_fd_(in_fd), out_fd_(out_fd), is_tty_(isatty(in_fd_))
{
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  if (pipe(terminate_pipe_.data()))
    throw std::runtime_error("FDChannel: Failed to get termination pipe\n");

  pollfds_[0].fd = in_fd_;
  pollfds_[0].events = POLLIN;
  pollfds_[1].fd = terminate_pipe_[0];
  pollfds_[1].events = POLLIN;

  if (is_tty_) {
    original_termios_ = std::make_unique<struct termios>();
    tcgetattr(in_fd_, original_termios_.get());
    struct termios term = *original_termios_;
    cfmakeraw(&term);
    term.c_lflag &= ~ECHO;
    term.c_oflag |= OPOST | ONLCR;
    tcsetattr(in_fd_, 0, &term);
  }
}

size_t FDChannel::read(std::span<uint8_t> arr, size_t n) {
  while (true) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    int code = poll(pollfds_.data(), 2, 10);

    if (code > 0)
    {
      // Terminated
      if ((pollfds_[1].revents & POLLIN))
        {
          char buf = 0;
          (void) ::read(terminate_pipe_[0], &buf, 1);
          return 0;
        }

      if ((pollfds_[0].revents & POLLIN) != 0)
      {
        ssize_t count = ::read(in_fd_, arr.data(), n);
        // std::cout << "Read " << count << " bytes from in_fd_\n";
        if (count < 0)
          throw std::runtime_error("FDChannel: Failed to read from in_fd_\n");

        if (is_tty_)
          for (size_t i = 0; i < static_cast<size_t>(count); i++) {
            const uint8_t c = arr[i];

            // Force a stop if control-a x is seen.
            if (prev_ == 1 and c == 'x') {
              // It is implementation defined whether destructors get called
              // before the program exits due to an unhandled exception, so we
              // restore termios before throwing
              restoreTermios();
              throw std::runtime_error("Keyboard stop");
            }
            prev_ = c;
          }

        return count;
      }
    }

    if (code == 0)
      // Timeout
      continue;

    // TODO: handle error return codes from poll.
    return 0;
  }
}

void FDChannel::write(uint8_t byte) {
  long written = 0;
  do {
    written = ::write(out_fd_, &byte, 1);
  } while (written != 1 && written != -1);

  if (written == -1)
    throw std::runtime_error("FDChannel error writing to output\n");
}


void FDChannel::terminate() {
  const uint8_t byte = 0;
  if (::write(terminate_pipe_[1], &byte, 1) != 1)
    std::cerr << "Info: FDChannel::terminate: write failed\n";
}

void FDChannel::restoreTermios() {
  if (is_tty_ && original_termios_) {
    tcsetattr(in_fd_, TCSANOW, original_termios_.get());
  }
}

FDChannel::~FDChannel() {
  restoreTermios();

  for (int i : terminate_pipe_) {
    if (i != -1)
      close(i);
  }
}

PTYChannelBase::PTYChannelBase() {
  std::array<char, 256> name{};
  if (openpty(&master_, &slave_, name.data(), nullptr, nullptr) < 0)
    throw std::runtime_error("Failed to open a PTY\n");

  std::cerr << "Info: Got PTY " << name.data() << "\n";
}

PTYChannelBase::~PTYChannelBase()
{
  if (master_ != -1)
    close(master_);

  if (slave_ != -1)
    close(slave_);
}


PTYChannel::PTYChannel() :  FDChannel(master_, master_)
{ }


SocketChannelBase::SocketChannelBase(int server_fd) {
  struct sockaddr client_addr{};
  socklen_t client_len = sizeof(struct sockaddr_in);
  conn_fd_ = accept(server_fd, &client_addr, &client_len);
  if (conn_fd_ < 0)
    throw std::runtime_error("Failed to accept socket connection\n");
}

SocketChannelBase::~SocketChannelBase() {
  if (conn_fd_ != -1) {
    shutdown(conn_fd_, SHUT_RDWR);
    close(conn_fd_);
  }
}

SocketChannel::SocketChannel(int server_fd) : SocketChannelBase(server_fd), FDChannel(conn_fd_, conn_fd_)
{ }


ForkChannel::ForkChannel(std::unique_ptr<UartChannel> readWriteChannel, std::unique_ptr<UartChannel> writeOnlyChannel)
  : readWriteChannel_(std::move(readWriteChannel)), writeOnlyChannel_(std::move(writeOnlyChannel)) {}

size_t ForkChannel::read(std::span<uint8_t> buf, size_t size) {
  return readWriteChannel_->read(buf, size);
}

void ForkChannel::write(uint8_t byte) {
  readWriteChannel_->write(byte);
  writeOnlyChannel_->write(byte);
}

void ForkChannel::terminate() {
  readWriteChannel_->terminate();
  writeOnlyChannel_->terminate();
}

Uart8250::Uart8250(uint64_t addr, uint64_t size,
    std::shared_ptr<TT_APLIC::Aplic> aplic, uint32_t iid,
    std::unique_ptr<UartChannel> channel, bool enableInput, unsigned regShift)
  : IoDevice("uart8250", addr, size, std::move(aplic), iid), channel_(std::move(channel)), regShift_(regShift)
{
  if (enableInput)
    Uart8250::enable();
}

void Uart8250::disable()
{
  terminate_ = true;
  channel_->terminate();
  if (inThread_.joinable())
    inThread_.join();
}

Uart8250::~Uart8250()
{
  if (not terminate_)
    Uart8250::disable();
}

void Uart8250::enable() {
  terminate_ = false;
  auto func = [this]() { this->monitorInput(); };
  inThread_ = std::thread(func);
}

void Uart8250::interruptUpdate() {
  uint32_t initialIir = iir_;
  iir_ &= ~0xf;
  if ((ier_ & 0x01) && (lsr_ & 0x01)) {
    iir_ |= 0x04; // Receive interrupt pending
  } else if ((ier_ & 0x02) && (lsr_ & 0x20)) {
    iir_ |= 0x02; // Transmitter Holding Register Empty Interrupt
  } else {
    iir_ |= 0x01; // No interrupt pending
  }

  if ((initialIir & 1) != (iir_ & 1)) {
    setInterruptPending(!(iir_ & 1));
  }
}

uint32_t Uart8250::read(uint64_t addr) {
  uint64_t offset = (addr - address()) >> regShift_;
  bool dlab = lcr_ & 0x80;

  if (dlab == 0) {
    switch (offset) {
      case 0: {
        std::unique_lock<std::mutex> lock(mutex_);
        uint32_t res = 0;
        if (!rx_fifo.empty()) {
          res = rx_fifo.front();
          rx_fifo.pop();
        }
        if (rx_fifo.empty())
          lsr_ &= ~1; // Clear least sig bit of line status.
        lock.unlock();
        cv_.notify_all();
        return res;
      }
      case 1: return ier_;
      case 2: {
	uint32_t iir = iir_;
	interruptUpdate();
	return iir;
      }
      case 3: return lcr_;
      case 4: return mcr_;
      case 5: return lsr_;
      case 6: return msr_;
      case 7: return scr_;
      default: assert(false);
    }
  } else {
    switch (offset) {
      case 0: return dll_;
      case 1: return dlm_;
      default: assert(false);
    }
  }

  assert(0 && "Error: Assertion failed");
  return 0;
}

void Uart8250::write(uint64_t addr, uint32_t value) {
  uint64_t offset = (addr - address()) >> regShift_;
  bool dlab = lcr_ & 0x80;

  if (dlab == 0) {
    switch (offset) {
      case 0: {
        uint8_t byte = value;
        if (byte) {
          if (mcr_ & (1 << 4))
            rx_fifo.push(byte);
          else
            channel_->write(byte);
        }
        interruptUpdate();
      }
      break;
      case 1: {
        ier_ = value;
        interruptUpdate();
      } break;
      case 2: fcr_ = value; break;
      case 3: lcr_ = value; break;
      case 4: mcr_ = value; break;
      case 5:
      case 6: break;
      case 7: scr_ = value; break;
      default:
        /* std::cerr << "Error: Uart writing addr 0x" << std::hex << addr << std::dec << '\n'; */
        assert(0 && "Error: Assertion failed");
    }
  } else {
    switch (offset) {
      case 0: dll_ = value; break;
      case 1: dlm_ = value; break;
      case 3: lcr_ = value; break;
      case 5: psd_ = value; break;
      default:
        /* std::cerr << "Error: Uart writing addr 0x" << std::hex << addr << std::dec << '\n'; */
        assert(0 && "Error: Assertion failed");
    }
  }
}

bool Uart8250::saveSnapshot(const std::string& filename) const
{
  std::ofstream ofs(filename);
  if (not ofs)
    {
      std::cerr << "Error: failed to open snapshot file for writing: " << filename << "\n";
      return false;
    }
  ofs << std::hex;
  ofs << "ier 0x" << (int) ier_ << "\n";
  ofs << "iir 0x" << (int) iir_ << "\n";
  ofs << "lcr 0x" << (int) lcr_ << "\n";
  ofs << "mcr 0x" << (int) mcr_ << "\n";
  ofs << "lsr 0x" << (int) lsr_ << "\n";
  ofs << "msr 0x" << (int) msr_ << "\n";
  ofs << "scr 0x" << (int) scr_ << "\n";
  ofs << "fcr 0x" << (int) fcr_ << "\n";
  ofs << "dll 0x" << (int) dll_ << "\n";
  ofs << "dlm 0x" << (int) dlm_ << "\n";
  ofs << "psd 0x" << (int) psd_ << "\n";

  auto rx_fifo_copy = rx_fifo;
  while (!rx_fifo_copy.empty()) {
      ofs << "rx_fifo 0x" << (int) rx_fifo_copy.front() << "\n";
      rx_fifo_copy.pop();
  }

  ofs << "reg_shift " << regShift_ << "\n";

  return true;
}


static std::string
stripComment(const std::string& line)
{
    size_t pos = line.find('#');
    if (pos != std::string::npos)
        return line.substr(0, pos);
    return line;
}


bool Uart8250::loadSnapshot(const std::string& filename)
{
  std::ifstream ifs(filename);
  if (not ifs)
    std::cerr << "Warning: failed to open snapshot file " << filename << "\n";

  std::string line;
  int lineno = 0;
  while (std::getline(ifs, line))
    {
      lineno++;
      std::string data = stripComment(line);
      boost::algorithm::trim(data);
      if (data.empty())
        continue;
      std::istringstream iss(data);
      iss >> std::hex;
      std::string regName;
      unsigned value = 0;
      if (not (iss >> regName >> value))
        {
          std::cerr << "Error: failed to parse UART snapshot file " << filename << " line " << lineno << ": \n" << line << '\n';
          return false;
        }
      std::string dummy;
      if (iss >> dummy)
        {
          std::cerr << "Error: failed to parse UART snapshot file " << filename << " line " << lineno << ": "
                    << "unexpected tokens\n";
          return false;
        }
      if      (regName == "ier") ier_ = value;
      else if (regName == "iir") iir_ = value;
      else if (regName == "lcr") lcr_ = value;
      else if (regName == "mcr") mcr_ = value;
      else if (regName == "lsr") lsr_ = value;
      else if (regName == "msr") msr_ = value;
      else if (regName == "scr") scr_ = value;
      else if (regName == "fcr") fcr_ = value;
      else if (regName == "dll") dll_ = value;
      else if (regName == "dlm") dlm_ = value;
      else if (regName == "psd") psd_ = value;
      else if (regName == "rx_fifo") rx_fifo.push(value);
      else if (regName == "rx_fifo") rx_fifo.push(value);
      else if (regName == "reg_shift") regShift_ = value;
      else
        {
          std::cerr << "Error: failed to parse UART snapshot file " << filename << " line " << lineno << ": '"
                  << regName << "' is not a valid UART register name\n";
          return false;
        }
    }
  return true;
}

void Uart8250::monitorInput() {
  while (true) {
    if (terminate_)
      return;

    std::array<uint8_t, FIFO_SIZE> arr{};
    size_t count = channel_->read(arr, FIFO_SIZE);
    if (count == 0)
      // EOF
      return;

    std::unique_lock<std::mutex> lock(mutex_);

    size_t i = 0;
    do {
      if (terminate_)
        return;

      for (; i < count && rx_fifo.size() < FIFO_SIZE; i++) {
        rx_fifo.push(arr.at(i));
      }

      lsr_ |= 1;  // Set receiver data ready
      interruptUpdate();

      if (rx_fifo.size() >= FIFO_SIZE)
        // Block until rx_fifo has space
        cv_.wait(lock);
    } while (i != count);
  }
}
