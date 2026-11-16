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

#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>


namespace WdRiscv
{

  class JtagDtm;

  /// Server for the remote_bitbang protocol used by OpenOCD's remote_bitbang
  /// adapter driver (and by Spike's --rbb-port): the client drives the JTAG
  /// pins with one ASCII character per change and asks for TDO with 'R'.
  ///
  /// The server listens on 127.0.0.1 and serves one client at a time. A helper
  /// thread waits for socket activity and raises a flag, so that the
  /// simulation loop can check for debugger input with one memory read
  /// between instruction slices; all socket reads and writes and all JTAG
  /// activity take place in the thread that calls service.
  class RemoteBitbang
  {
  public:

    explicit RemoteBitbang(JtagDtm& dtm);

    ~RemoteBitbang();

    RemoteBitbang(const RemoteBitbang&) = delete;
    RemoteBitbang& operator=(const RemoteBitbang&) = delete;

    /// Listen on the given TCP port (0: let the OS choose) and print
    /// "Listening for remote bitbang connection on port N." on the standard
    /// output. Return true on success.
    bool open(unsigned port);

    /// Return true if the socket has (or may have) input: a pending connection,
    /// commands from the client, or a closed connection. Cheap.
    bool hasInput() const
    { return ready_.load(std::memory_order_acquire); }

    /// Accept a pending connection and execute every command the client has
    /// sent, replying to each 'R'. Does not block.
    void service();

    /// Block until the socket has input or the given time (in milliseconds)
    /// elapses.
    void wait(unsigned timeoutMs);

  private:

    /// Body of the helper thread: wait for input on the listening socket or
    /// on the client connection, then raise ready_ and wait to be re-armed.
    void watch();

    /// Let the helper thread wait for the next input.
    void arm();

    /// Execute the given command characters, appending TDO replies to out.
    /// Return false if the client asked to quit.
    bool execute(std::string_view commands, std::string& out);

    /// Close the client connection.
    void disconnect();

    JtagDtm& dtm_;
    int listenFd_ = -1;
    int clientFd_ = -1;
    std::array<int, 2> wakePipe_ = { -1, -1 };  // Wakes the helper thread to stop it.

    std::thread watcher_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool armed_ = false;          // Helper thread may poll (guarded by mutex_).
    bool stop_ = false;           // Helper thread must exit (guarded by mutex_).
    std::atomic<bool> ready_ = false;
  };

}
