#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include "IoDevice.hpp"


namespace WdRiscv
{

  class Uartsf : public IoDevice
  {
  public:

    Uartsf(uint64_t addr, uint64_t size);

    ~Uartsf() override;

    void enable() override {}

    void disable() override {}

    uint32_t read(uint64_t addr) override;

    void write(uint64_t addr, uint32_t value) override;

    bool saveSnapshot([[maybe_unused]] const std::string& filename) const override
    {
      std::cerr << "Uartsf::saveSnapshot() not implemented\n";
      return false;
    }

    bool loadSnapshot([[maybe_unused]]const std::string& filename) override
    {
      std::cerr << "Uartsf::loadSnapshot() not implemented\n";
      return false;
    }

  private:

    /// This runs in its own thread. It monitors the standard input and
    /// marks interrupt pending when input is possible placing the input
    /// character in byte_ for the Uart to consume.
    void monitorInput();

    enum RegId { TX_FIFO, RX_FIFO, TX_CTRL, RX_CTRL, IE, IP, DIV, N };
    const uint32_t RX_EMPTY = 0x80000000;
    const uint32_t TX_EN = 1;
    const uint32_t RX_EN = 1;

    std::vector<uint32_t> regs_;  // Indexed by RegId

    std::thread stdinThread_;
    std::atomic<bool> terminate_ = false;
    std::mutex mutex_;   // Synchronize access to byte_ with stdinThread_.
  };
}
