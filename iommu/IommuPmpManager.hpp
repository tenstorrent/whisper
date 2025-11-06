// Copyright 2020 Western Digital Corporation or its affiliates.
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

// Copyright 2024 Tenstorrent Corporation.
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

#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <iostream>
#include <bit>
#include <cassert>
#include "riscv_enums.hpp"

namespace TT_IOMMU
{

  /// Physical memory protection. An instance of this is associated with a region of the
  /// address space in PmpManager.
  class Pmp
  {
  public:

    friend class PmpManager;

    /// Type of region: off, top-of-range, naturally aligned of size 4,
    /// naturally aligned power of 2.
    enum Type : uint8_t { Off = 0, Tor = 1, Na4 = 2, Napot = 3, _Count = 4 };

    /// Region access modes.
    enum Mode : uint8_t
      {
        None = 0, Read = 1, Write = 2, Exec = 4, ReadWrite = Read | Write,
        Default = Read | Write | Exec
      };

    /// Default constructor: No access allowed.
    Pmp(Mode m = None, unsigned pmpIx = 0, bool locked = false,
        Type type = Type::Off)
      : mode_(m), type_(type), locked_(locked), pmpIx_(pmpIx)
    { }

    /// Return true if read (i.e. load instructions) access allowed
    bool isRead(PrivilegeMode mode) const
    {
      bool check = (mode != PrivilegeMode::Machine) or locked_;
      return check ? mode_ & Read : true;
    }

    /// Return true if write (i.e. store instructions) access allowed.
    bool isWrite(PrivilegeMode mode) const
    {
      bool check = (mode != PrivilegeMode::Machine or locked_);
      return check ? mode_ & Write : true;
    }

    /// Return true if instruction fecth is allowed.
    bool isExec(PrivilegeMode mode) const
    {
      bool check = (mode != PrivilegeMode::Machine or locked_);
      return check ? mode_ & Exec : true;
    }

    /// Return true if this object has the mode attributes as the
    /// given object.
    bool operator== (const Pmp& other) const
    { return mode_ == other.mode_ and pmpIx_ == other.pmpIx_; }

    /// Return true if this object has different attributes from those
    /// of the given object.
    bool operator!= (const Pmp& other) const
    { return mode_ != other.pmpIx_ or pmpIx_ != other.pmpIx_; }

    /// Return string representation of the given PMP type.
    static std::string toString(Type type)
    {
      switch (type)
        {
        case Pmp::Type::Off:   return "off";
        case Pmp::Type::Tor:   return "tor";
        case Pmp::Type::Na4:   return "na4";
        case Pmp::Type::Napot: return "napot";
        default:               return "?";
        }
      return "";
    }

    /// Return string representation of the given PMP mode.
    static std::string toString(Mode mode)
    {
      std::string result;

      result += (mode & Mode::Read)  ? "r" : "-";
      result += (mode & Mode::Write) ? "w" : "-";
      result += (mode & Mode::Exec)  ? "x" : "-";

      return result;
    }

    /// Return integer representation of the given PMP configuration.
    uint8_t val() const
    { return (locked_ << 7) | (0 << 5) | ((uint8_t(type_) & 3) << 3) | (mode_ & 7); }

    /// Return the index of the PMP entry from which this object was
    /// created.
    unsigned pmpIndex() const
    { return pmpIx_; }

  private:

    uint8_t mode_ = 0;
    Type type_      : 8;
    bool locked_    : 1;
    unsigned pmpIx_ : 5;  // Index of corresponding pmp register.
  } __attribute__((packed));
  static_assert(sizeof(Pmp) <= 3);


  /// Physical memory protection manager. One per hart.  Protection applies to
  /// word-aligned regions as small as 1 word but are expected to be applied to a small
  /// number (less than or equal 64) of regions.
  class PmpManager
  {
  public:

    friend class Memory;

    enum AccessReason { None, Fetch, LdSt };

    /// Constructor: Mark all memory as no access to user/supervisor.
    PmpManager() = default;

    /// Destructor.
    ~PmpManager() = default;

    /// Reset: Mark all memory as no access to user/supervisor.
    void reset()
    {
      regions_.clear();
      fastRegion_.reset();
    }

    /// Return the physical memory protection object (pmp) associated
    /// with the word-aligned word designated by the given
    /// address. Return a no-access object if the given address is out
    /// of memory range.
    Pmp getPmp(uint64_t addr) const
    {
      addr = (addr >> 2) << 2;
      if (fastRegion_)
        if (addr >= fastRegion_->firstAddr_ and addr <= fastRegion_->lastAddr_)
          return fastRegion_->region_.pmp_;
      for (unsigned ix = 0; ix < regions_.size(); ++ix)
        {
          const auto& region = regions_.at(ix);
          if (addr >= region.firstAddr_ and addr <= region.lastAddr_)
            {
              updateCachedRegion(region, ix);
              return region.pmp_;
            }
        }
      return {};
    }

    /// Return the physical memory protection object (pmp) associated
    /// with a given index. Return a no-access object if the given index
    /// is out of range.
    Pmp peekPmp(size_t ix) const
    {
      for (const auto& region : regions_)
        {
          auto pmp = region.pmp_;
          if (pmp.pmpIndex() == ix)
            return pmp;
        }
      return {};
    }

    struct PmpTrace
    {
      uint32_t ix_;
      uint64_t addr_;
      uint8_t val_;
      AccessReason reason_;
    };

    /// Similar to getPmp but it also updates the access count associated with
    /// each PMP entry.
    inline const Pmp& accessPmp(uint64_t addr) const
    {
      addr = (addr >> 2) << 2;
      if (fastRegion_)
        {
          if (addr >= fastRegion_->firstAddr_ and addr <= fastRegion_->lastAddr_)
            {
              if (trace_)
                {
                  const auto& pmp = fastRegion_->region_.pmp_;
                  auto ix = pmp.pmpIndex();
                  auto val = pmp.val();
                  pmpTrace_.push_back({ix, addr, val, reason_});
                }
              return fastRegion_->region_.pmp_;
            }
        }
      for (unsigned ix = 0; ix < regions_.size(); ++ix)
        {
          const auto& region = regions_.at(ix);
          if (addr >= region.firstAddr_ and addr <= region.lastAddr_)
            {
              if (trace_)
                {
                  const auto& pmp = region.pmp_;
                  auto ix = pmp.pmpIndex();
                  auto val = pmp.val();
                  pmpTrace_.push_back({ix, addr, val, reason_});
                }
              updateCachedRegion(region, ix);
              return region.pmp_;
            }
        }
      return defaultPmp_;
    }

    /// Used for tracing to determine if an address matches multiple PMPs.
    bool matchMultiplePmp(uint64_t addr) const
    {
      bool hit = false;
      for (const auto& region : regions_)
        if (addr >= region.firstAddr_ and addr <= region.lastAddr_)
          {
            if (hit)
              return true;
            hit = true;
          }
      return false;
    }

    /// Enable/disable physical memory protection.
    void enable(bool flag)
    { enabled_ = flag; }

    /// Return true if physical memory protection is enabled.
    bool isEnabled() const
    { return enabled_; }

    /// Set access mode of word-aligned words overlapping given region
    /// for user/supervisor.
    void defineRegion(uint64_t addr0, uint64_t addr1, Pmp::Type type,
                      Pmp::Mode mode, unsigned pmpIx, bool locked)
    {
      addr0 = (addr0 >> 2) << 2;   // Make word aligned.
      addr1 = (addr1 >> 2) << 2;   // Make word aligned.

      Pmp pmp(mode, pmpIx, locked, type);
      Region region{addr0, addr1, pmp};
      regions_.push_back(region);
    }

    /// Print statistics on the given stream.
    bool printStats(std::ostream& out) const;

    /// Print statistics on the given file.
    bool printStats(const std::string& path) const;

    /// Print current pmp map matching a particular address.
    void printPmps(std::ostream& os, uint64_t addr) const
    {
      auto region = getRegion(addr);
      printRegion(os, region);
    }

    /// Print current pmp map.
    void printPmps(std::ostream& os) const
    {
      for (const auto& region : regions_)
        printRegion(os, region);
    }

    /// Return the access count of PMPs used in most recent instruction.
    const std::vector<PmpTrace>& getPmpTrace() const
    { return pmpTrace_; }

    void clearPmpTrace()
    { pmpTrace_.clear(); }

    /// Collect stats if flag is true.
    void enableTrace(bool flag)
    { trace_ = flag; }

    /// This is to differentiate fetch from ld/st accesses.
    void setAccReason(AccessReason reason)
    { reason_ = reason; }

    /// Given the internal value of a PMPADDR register and the corresponding byte in the
    /// PMPCFG register, return the read value of PMPADDR. This is done on a CSR read
    /// since the read value of PMPADDR may be different than its internal value.
    uint64_t adjustPmpValue(uint64_t value, uint8_t pmpcfgByte, bool rv32) const
    {
      if (pmpG_ == 0)
        return value;

      unsigned aField =(pmpcfgByte >> 3) & 3;
      if (aField < 2)
        {
          // A field is OFF or TOR
          if (pmpG_ >= 1)
            value = (value >> pmpG_) << pmpG_; // Clear least sig G bits.
        }
      else
        {
          // A field is NAPOT
          if (pmpG_ >= 2)
            {
              uint64_t mask = ~uint64_t(0);
              unsigned width = rv32 ? 32 : 64;
              if (width >= pmpG_ - 1)
                mask >>= (width - pmpG_ + 1);
              value = value | mask; // Set to 1 least sig G-1 bits
            }
        }

      return value;
    }

    /// Set the physical memory protection G parameter. The grain size is 2 to the power
    /// G+2. The values returned by a read operation of the PMPADDR registers are adjusted
    /// according to G.
    void setPmpG(unsigned value)
    { pmpG_ = value; }

    /// Return the physical memory protection G parameter. See setPmpG.
    unsigned getPmpG() const
    { return pmpG_; }

    /// Unpack the mode (read/write/exec), type, and locked fields encoded in the given
    /// byte of a PMPCFG CSR.
    static void unpackPmpconfigByte(uint8_t byte, Pmp::Mode& mode, Pmp::Type& type,
                                    bool& locked)
    {
      unsigned mm = 0;

      if (byte & 1) mm |= Pmp::Read;
      if (byte & 2) mm |= Pmp::Write;
      if (byte & 4) mm |= Pmp::Exec;

      mode = Pmp::Mode(mm);

      type = Pmp::Type((byte >> 3) & 3);
      locked = byte & 0x80;
    }

    /// Given the PMPCFG byte corresponding to a PMPADDR CSR, the value of that CSR,
    /// and the value of the preceding CSR (for TOR), return the mode, and type of that
    /// PMPADDR CSR, whether or not it is locked, and the range of addresses it covers.
    bool unpackMemoryProtection(unsigned config, uint64_t pmpVal, uint64_t prevPmpVal,
                                bool rv32, Pmp::Mode& mode, Pmp::Type& type,
                                bool& locked, uint64_t& low, uint64_t& high) const
    {
      unpackPmpconfigByte(config, mode, type, locked);

      if (type == Pmp::Type::Off)
        return true;   // Entry is off.

      if (type == Pmp::Type::Tor)    // Top of range
        {
          low = prevPmpVal;
          low = (low >> pmpG_) << pmpG_;  // Clear least sig G bits.
          low = low << 2;

          high = pmpVal;
          high = (high >> pmpG_) << pmpG_;
          high = high << 2;
          if (high == 0)
            {
              type = Pmp::Type::Off;  // Empty range.
              return true;
            }

          high = high - 1;
          return true;
        }

      uint64_t sizeM1 = 3;     // Size minus 1
      uint64_t napot = pmpVal;  // Naturally aligned power of 2.
      if (type == Pmp::Type::Napot)  // Naturally algined power of 2.
        {
          unsigned rzi = 0;  // Righmost-zero-bit index in pmpval.
          if ((rv32 and pmpVal == ~uint32_t(0)) or (not rv32 and pmpVal == ~uint64_t(0)))
            {
              // Handle special case where pmpVal is set to maximum value
              napot = 0;
              rzi = rv32? 32 : 64;
            }
          else
            {
              rzi = std::countr_zero(~pmpVal); // rightmost-zero-bit ix.
              napot = (napot >> rzi) << rzi; // Clear bits below rightmost zero bit.
            }

          // Avoid overflow when computing 2 to the power 64 or higher. This is incorrect
          // but should work in practice where the physical address space is 64-bit wide
          // or less.
          if (rzi + 3 >= 64)
            sizeM1 = -1L;
          else
            sizeM1 = (uint64_t(1) << (rzi + 3)) - 1;
        }
      else
        assert(type == Pmp::Type::Na4);

      low = napot;
      low = (low >> pmpG_) << pmpG_;
      low = low << 2;
      high = low + sizeM1;
      return true;
    }

    /// Enable/disable top-of-range mode in pmp configurations.
    void enableTor(bool flag)
    { torEnabled_ = flag; }

    /// Enable/disable NA4 mode in pmp configurations.
    void enableNa4(bool flag)
    { na4Enabled_ = flag; }

    /// Return true if top-of-range mode in pmp config is enabled.
    bool torEnabled() const
    { return torEnabled_; }

    /// Return true if naturally aligned size 4 mode in pmp config is enabled.
    bool na4Enabled() const
    { return na4Enabled_; }

    /// Legalize the PMPCFG value (next) before updating such a register: If the grain
    /// factor G is greater than or equal to 1, then the NA4 mode is not selectable in the
    /// A field. If a field is locked it is replaced by the prev value. Return the
    /// legalized value. The typename T is either uint32_t (rv32) or uint64_t (rv64).
    template <typename T>
    T legalizePmpcfg(T prev, T next) const
    {
      T legal = 0;
      for (unsigned i = 0; i < sizeof(next); ++i)
        {
          uint8_t pb = (prev >> (i*8)) & 0xff;  // Prev byte.
          uint8_t nb = (next >> (i*8)) & 0xff;    // New byte.

          if (pb >> 7)
            nb = pb; // Field is locked. Use byte from prev value.
          else
            {
              unsigned aField = (nb >> 3) & 3;
              if (aField == 2)   // NA4
                {
                  // If G is >= 1 then NA4 is not selectable in the A field.
                  if (not na4Enabled() or (getPmpG() != 0 and aField == 2))
                    nb = (pb & 0x18) | (nb & ~0x18);  // Preserve A field.
                }
              else if (aField == 1)  // TOR
                {
                  if (not torEnabled())    // TOR not supported
                    nb = (pb & 0x18) | (nb & ~0x18);  // Preserve A field.
                }

              // w=1 r=0 is not allowed: Preserve the xwr field.
              if ((nb & 3) == 2)
                {
                  nb = (pb & 7) | (nb & ~7);   // Preserve xwr field.
                }
            }

          legal = legal | (T(nb) << i*8);
        }

      return legal;
    }

  private:

    struct Region
    {
      uint64_t firstAddr_ = 0;
      uint64_t lastAddr_ = 0;
      Pmp pmp_;
    };

    struct FastRegion
    {
      uint64_t firstAddr_ = 0;
      uint64_t lastAddr_ = 0;
      const Region& region_;
    };

    /// Return the Region object associated with the word-aligned word designed by the
    /// given address. Return a no-access object if the given address is out of memory
    /// range.
    Region getRegion(uint64_t addr) const
    {
      addr = (addr >> 2) << 2;
      for (const auto& region : regions_)
        if (addr >= region.firstAddr_ and addr <= region.lastAddr_)
          return region;
      return {};
    }

    /// Print current pmp map matching a particular address.
    static void printRegion(std::ostream& os, Region region)
    {
      const auto& pmp = region.pmp_;
      os << "pmp ix: " << std::dec << pmp.pmpIndex() << "\n";
      os << "base addr: " << std::hex << region.firstAddr_ << "\n";
      os << "last addr: " << std::hex << region.lastAddr_ << "\n";

      os << "rwx: " << Pmp::toString(Pmp::Mode(pmp.mode_)) << "\n";
      os << "matching: " << Pmp::toString(Pmp::Type(pmp.type_)) << "\n";
    }

    /// Update cached last region, finding largest non-overlapping
    /// region with priority.
    void updateCachedRegion(const auto& region, unsigned ix) const
    {
      uint64_t firstAddr = region.firstAddr_;
      uint64_t lastAddr = region.lastAddr_;
      for (unsigned i = 0; i < ix; ++i)
        {
          // By common use case, shrink lower bound address instead
          // of computing maximum size.
          auto a2 = regions_.at(i).lastAddr_;
          if (firstAddr <= a2)
            {
              firstAddr = a2 + 4;
              continue;
            }
        }
      if (firstAddr <= lastAddr)
        fastRegion_.emplace(firstAddr, lastAddr, region);
    }

    std::vector<Region> regions_;
    mutable std::optional<FastRegion> fastRegion_;

    bool enabled_ = false;
    bool trace_ = false;        // Collect stats if true.
    bool torEnabled_ = true;    // True if top-of-range type is enabled.
    bool na4Enabled_ = true;    // True if naturally-aligned size 4 type is enabled

    Pmp defaultPmp_;

    unsigned pmpG_ = 0;     // PMP G value: ln2(pmpGrain) - 2

    // PMPs used in most recent instruction
    mutable std::vector<PmpTrace> pmpTrace_;
    AccessReason reason_{};
  };
}
