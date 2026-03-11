#pragma once

#include <cassert>
#include <unordered_map>
#include <cmath>
#include <vector>
#include <cstdint>
#include <functional>
#include <iostream>

namespace TT_CACHE
{

  /// Model a generic cache. This is for MCM purposes.
  class Cache
  {
  public:

    Cache(unsigned lineSize = 64)
      : lineSize_(lineSize)
    {
      assert(lineSize > 0 and (lineSize % 8) == 0);
      lineShift_ = unsigned(std::log2(lineSize));
    }

    /// Add a line to the cache. Data is obtained by calling fetchMem
    /// for the words of the line. Return true on success. Return false
    /// if any of the memory reads fails. If the line already exists,
    /// we don't do anything.
    bool addLine(uint64_t addr)
    {
      if (not memRead_)
        return false;

      uint64_t lineNum = addr >> lineShift_;
      if (data_.contains(lineNum))
        return true;
      auto& vec = data_[lineNum];
      vec.resize(lineSize_);
      bool ok = true;
      unsigned dwords = lineSize_ / sizeof(uint64_t);
      addr = lineNum << lineShift_;
      for (unsigned i = 0; i < dwords; ++i, addr += sizeof(uint64_t))
	{
	  uint64_t val = 0;
	  ok = memRead_(addr, val) and ok;
	  unsigned j = i * sizeof(uint64_t);
	  vec.at(j) = val;
	  vec.at(j + 1) = val >> 8;
	  vec.at(j + 2) = val >> 16;
	  vec.at(j + 3) = val >> 24;
	  vec.at(j + 4) = val >> 32;
	  vec.at(j + 5) = val >> 40;
	  vec.at(j + 6) = val >> 48;
	  vec.at(j + 7) = val >> 56;
	}
      return ok;
    }

    bool writebackLine(uint64_t addr, const std::vector<uint8_t>& rtlData)
    {
      if (not memWrite_)
        return false;

      if (!rtlData.empty() and rtlData.size() != lineSize_)
        {
          std::cerr << "Error: writeback line size " << rtlData.size() << " does"
                       " not match cache line size " << lineSize_ << '\n';
          return false;
        }

      bool skipCheck = rtlData.empty();
      uint64_t lineNum = addr >> lineShift_;

      if (not data_.contains(lineNum))
        return false;
      auto& vec = data_.at(lineNum);
      bool ok = true;
      unsigned dwords = lineSize_ / sizeof(uint64_t);
      addr = lineNum << lineShift_;
      for (unsigned i = 0; i < dwords; ++i, addr += sizeof(uint64_t))
	{
          unsigned j = i * sizeof(uint64_t);
          auto val = uint64_t(vec.at(j));
          val |= uint64_t(vec.at(j + 1)) << 8;
          val |= uint64_t(vec.at(j + 2)) << 16;
          val |= uint64_t(vec.at(j + 3)) << 24;
          val |= uint64_t(vec.at(j + 4)) << 32;
          val |= uint64_t(vec.at(j + 5)) << 40;
          val |= uint64_t(vec.at(j + 6)) << 48;
          val |= uint64_t(vec.at(j + 7)) << 56;

          if (not skipCheck)
            {
              auto rtlVal = uint64_t(rtlData.at(j));
              rtlVal |= uint64_t(rtlData.at(j + 1)) << 8;
              rtlVal |= uint64_t(rtlData.at(j + 2)) << 16;
              rtlVal |= uint64_t(rtlData.at(j + 3)) << 24;
              rtlVal |= uint64_t(rtlData.at(j + 4)) << 32;
              rtlVal |= uint64_t(rtlData.at(j + 5)) << 40;
              rtlVal |= uint64_t(rtlData.at(j + 6)) << 48;
              rtlVal |= uint64_t(rtlData.at(j + 7)) << 56;

              if (val != rtlVal)
                {
                  std::cerr << "Error: Failed writeback comparison for dword " << i
                            << " Whisper: " << std::hex << val << " RTL: " << rtlVal << '\n' << std::dec;
                  ok = false;
                }
            }

	  ok = memWrite_(addr, val) and ok;
        }
      return ok;
    }

    /// Remove from this cashe the line contining the given address.
    /// No-op if line is not in cache.
    void removeLine(uint64_t addr)
    {
      uint64_t lineNum = addr >> lineShift_;
      data_.erase(lineNum);
    }

    /// Return true if the line contaning the given address is in the cache.
    bool isLineResident(uint64_t addr) const
    {
      uint64_t lineNum = addr >> lineShift_;
      return data_.contains(lineNum);
    }

    /// Read into inst the 2-bytes at the given address. Return true on success. Return
    /// false if addr + size would cross cacheline or does not exist.
    template <typename SZ>
    bool read(uint64_t addr, SZ& data) const
    {
      uint64_t lineNum = addr >> lineShift_;
      uint64_t lineNum2 = (addr + sizeof(SZ) - 1) >> lineShift_;

      if (lineNum != lineNum2)
        return false;
      if (not data_.contains(lineNum))
        return false;
      const auto& vec = data_.at(lineNum);
      unsigned byteIx = addr % lineSize_;
      if constexpr (sizeof(SZ) >= sizeof(uint8_t))
        data = vec.at(byteIx);
      if constexpr (sizeof(SZ) >= sizeof(uint16_t))
        data |= (uint16_t(vec.at(byteIx + 1)) << 8);
      if constexpr (sizeof(SZ) >= sizeof(uint32_t))
        {
          data |= (uint32_t(vec.at(byteIx + 2)) << 16);
          data |= (uint32_t(vec.at(byteIx + 3)) << 24);
        }
      if constexpr (sizeof(SZ) >= sizeof(uint64_t))
        {
          data |= (uint64_t(vec.at(byteIx + 4)) << 32);
          data |= (uint64_t(vec.at(byteIx + 5)) << 40);
          data |= (uint64_t(vec.at(byteIx + 6)) << 48);
          data |= (uint64_t(vec.at(byteIx + 7)) << 56);
        }
      return true;
    }

    /// Poke byte if given address is in the cache. Return false
    /// otherwise.
    bool poke(uint64_t addr, uint8_t byte)
    {
      uint64_t lineNum = addr >> lineShift_;
      if (not data_.contains(lineNum))
        return false;
      unsigned byteIx = addr % lineSize_;
      data_.at(lineNum)[byteIx] = byte;
      return true;
    }

    /// Callback to read from memory.
    void addMemReadCallback(const std::function<bool(uint64_t, uint64_t&)>& memRead)
    { memRead_ = memRead; }

    /// Callback to write tomemory.
    void addMemWriteCallback(const std::function<bool(uint64_t, uint64_t)>& memWrite)
    { memWrite_ = memWrite; }

    /// Empty cache.
    void clear()
    { data_.clear(); }

  private:

    unsigned lineSize_ = 64;
    unsigned lineShift_ = 6;
    std::unordered_map< uint64_t, std::vector<uint8_t>> data_{};
    std::function<bool(uint64_t, uint64_t&)> memRead_;
    std::function<bool(uint64_t, uint64_t)> memWrite_;
  };

}


