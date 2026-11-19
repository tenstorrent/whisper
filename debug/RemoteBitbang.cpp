// Copyright 2026 David_Harris@hmc.edu.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string_view>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include "JtagDtm.hpp"
#include "RemoteBitbang.hpp"


using namespace WdRiscv;


static bool
setNonBlocking(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 and fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}


RemoteBitbang::RemoteBitbang(JtagDtm& dtm)
  : dtm_(dtm)
{
}


RemoteBitbang::~RemoteBitbang()
{
  if (watcher_.joinable())
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
      }
      cv_.notify_all();
      char byte = 0;
      if (write(wakePipe_.at(1), &byte, 1) < 0)
        std::cerr << "Warning: remote bitbang: failed to wake helper thread\n";
      watcher_.join();
    }

  disconnect();
  for (int fd : { listenFd_, wakePipe_.at(0), wakePipe_.at(1) })
    if (fd >= 0)
      close(fd);
}


bool
RemoteBitbang::open(unsigned port)
{
  listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listenFd_ < 0)
    {
      std::cerr << "Error: remote bitbang: failed to create socket: " << strerror(errno) << '\n';
      return false;
    }

  int one = 1;
  setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(static_cast<uint16_t>(port));

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  auto* saddr = reinterpret_cast<sockaddr*>(&addr);
  if (bind(listenFd_, saddr, sizeof(addr)) < 0)
    {
      std::cerr << "Error: remote bitbang: failed to bind to port " << port << ": "
                << strerror(errno) << '\n';
      return false;
    }

  if (listen(listenFd_, 1) < 0 or not setNonBlocking(listenFd_))
    {
      std::cerr << "Error: remote bitbang: failed to listen: " << strerror(errno) << '\n';
      return false;
    }

  socklen_t len = sizeof(addr);
  if (getsockname(listenFd_, saddr, &len) < 0)
    {
      std::cerr << "Error: remote bitbang: getsockname failed: " << strerror(errno) << '\n';
      return false;
    }

  if (pipe(wakePipe_.data()) < 0)
    {
      std::cerr << "Error: remote bitbang: failed to create pipe: " << strerror(errno) << '\n';
      return false;
    }

  // Debuggers and test scripts parse this line to find the port.
  std::cout << "Listening for remote bitbang connection on port " << ntohs(addr.sin_port)
            << ".\n" << std::flush;

  armed_ = true;
  watcher_ = std::thread(&RemoteBitbang::watch, this);
  return true;
}


void
RemoteBitbang::watch()
{
  while (true)
    {
      int fd = -1;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return armed_ or stop_; });
        if (stop_)
          return;
        fd = clientFd_ >= 0 ? clientFd_ : listenFd_;
      }

      std::array<pollfd, 2> fds = { pollfd{ .fd = fd, .events = POLLIN, .revents = 0 },
                                    pollfd{ .fd = wakePipe_.at(0), .events = POLLIN, .revents = 0 } };
      int rc = poll(fds.data(), fds.size(), -1);
      if (rc < 0 and errno == EINTR)
        continue;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_)
          return;
        armed_ = false;
        ready_.store(true, std::memory_order_release);
      }
      cv_.notify_all();
    }
}


void
RemoteBitbang::arm()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    armed_ = true;
  }
  cv_.notify_all();
}


void
RemoteBitbang::wait(unsigned timeoutMs)
{
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
               [this]() { return ready_.load(std::memory_order_acquire); });
}


void
RemoteBitbang::disconnect()
{
  if (clientFd_ >= 0)
    close(clientFd_);
  clientFd_ = -1;
}


bool
RemoteBitbang::execute(std::string_view commands, std::string& out)
{
  for (char c : commands)
    {
      switch (c)
        {
        case '0': case '1': case '2': case '3':
        case '4': case '5': case '6': case '7':
          {
            auto bits = static_cast<unsigned>(c - '0');
            dtm_.setPins((bits & 4) != 0, (bits & 2) != 0, (bits & 1) != 0);
          }
          break;

        case 'R':
          out.push_back(dtm_.tdo() ? '1' : '0');
          break;

        case 't': case 'u':
          // Reset with TRST asserted resets the TAP. SRST (s, u) is not connected.
          dtm_.reset();
          break;

        case 'r': case 's': case 'B': case 'b': case '\n': case '\r':
          // TRST deasserted, LED blink, and line ends: nothing to do.
          break;

        case 'Q':
          return false;

        default:
          std::cerr << "Warning: remote bitbang: ignoring unsupported command '" << c << "'\n";
          break;
        }
    }
  return true;
}


void
RemoteBitbang::service()
{
  ready_.store(false, std::memory_order_release);

  if (clientFd_ < 0)
    {
      int fd = accept(listenFd_, nullptr, nullptr);
      if (fd >= 0)
        {
          int one = 1;
          setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
          if (setNonBlocking(fd))
            clientFd_ = fd;
          else
            close(fd);
        }
      arm();
      return;
    }

  std::array<char, 8192> buf{};
  std::string out;
  bool open = true;

  while (open)
    {
      ssize_t count = recv(clientFd_, buf.data(), buf.size(), 0);
      if (count < 0 and errno == EINTR)
        continue;
      if (count < 0 and (errno == EAGAIN or errno == EWOULDBLOCK))
        break;
      if (count <= 0)
        {
          open = false;  // Client closed the connection (or an error).
          break;
        }

      out.clear();
      open = execute(std::string_view(buf.data(), static_cast<size_t>(count)), out);

      // Send the TDO replies before reading more: the client may be waiting for them.
      size_t sent = 0;
      while (sent < out.size())
        {
          auto rest = std::string_view(out).substr(sent);
          ssize_t n = send(clientFd_, rest.data(), rest.size(), MSG_NOSIGNAL);
          if (n > 0)
            sent += static_cast<size_t>(n);
          else if (n < 0 and (errno == EAGAIN or errno == EWOULDBLOCK or errno == EINTR))
            {
              pollfd pfd{ .fd = clientFd_, .events = POLLOUT, .revents = 0 };
              poll(&pfd, 1, 100);
            }
          else
            {
              open = false;
              break;
            }
        }
    }

  if (not open)
    disconnect();

  arm();
}
