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

#pragma once

#include <cstdint>
#include <string_view>
#include <vector>
#include <algorithm>
#include <unordered_map>

namespace WdRiscv
{

  /// Physical memory attribute. An instance of this is typically
  /// associated with a word-aligned section of the address space.
  class Pma
  {
  public:

    friend class PmaManager;

    enum Attrib
      {
       None = 0, Read = 1, Write = 2, Exec = 4, Idempotent = 8,
       AmoOther = 0x10,  // for amo add/min/max
       AmoSwap = 0x20, AmoLogical = 0x40,
       MemMapped = 0x200, Rsrv = 0x400,
       Io = 0x800, Cacheable = 0x1000,
       MisalOk = 0x2000, // True if misaligned access supported.
       MisalAccFault = 0x4000, // Set if misaligned generates access fault.
       Mapped = Exec | Read | Write,
       AmoArith = AmoSwap | AmoOther | AmoLogical,
       Amo = AmoArith,
       Default = Read | Write | Exec | Idempotent | Amo | Rsrv | MisalOk
      };

    /// Default constructor: No access allowed, no-mmr, no-atomic.
    Pma(Attrib a = None)
      : attrib_(a)
    { }

    /// Return true if associated address region is mapped (accessible
    /// for read, write, or execute).
    bool isMapped() const
    { return attrib_ & Mapped; }

    /// Return true if region has memory mapped register(s).
    bool hasMemMappedReg() const
    { return attrib_ & MemMapped; }

    /// Return true if idempotent region (non-IO region).
    bool isIdempotent() const
    { return attrib_ & Idempotent; }

    /// Return true if cacheable region.
    bool isCacheable() const
    { return attrib_ & Cacheable; }

    /// Return true if readable (load instructions allowed) region.
    bool isRead() const
    { return attrib_ & Read; }

    /// Return true if writeable (store instructions allowed) region.
    bool isWrite() const
    { return attrib_ & Write; }

    /// Return true if executable (fetch allowed) region.
    bool isExec() const
    { return attrib_ & Exec; }

    /// Return true if atomic instructions are allowed.
    bool isAmo() const
    { return attrib_ & Amo; }

    /// Return true if lr/sc instructions are allowed.
    bool isRsrv() const
    { return attrib_ & Rsrv; }

    /// Return true if IO region.
    bool isIo() const
    { return attrib_ & Io; }

    /// Return true if misaligned data access is supported in this region.
    bool isMisalignedOk() const
    { return attrib_& MisalOk; }

    /// Return true if misaligned access generates a misaligned
    /// exception in this region.
    bool misalOnMisal() const
    { return not (attrib_ & MisalAccFault); }

    /// Return true if misaligned access generates an access fault
    /// exception in this region.
    bool accessFaultOnMisal() const
    { return (attrib_ & MisalAccFault); }

    /// Return true if this object has the same attributes as the
    /// given object.
    bool operator== (const Pma& other) const
    { return attrib_ == other.attrib_; }

    /// Return true if this object has different attributes from those
    /// of the given object.
    bool operator!= (const Pma& other) const
    { return attrib_ != other.attrib_; }

    /// Enable given attribute in this PMA. Enabling None has no effect.
    void enable(Attrib a)
    { attrib_ |= a; }

    /// Disable given attribute in this PMA. Disabling None has no effect.
    void disable(Attrib a)
    { attrib_ &= ~a; }

    /// Return true if this PMA has the given attribute. If given value
    /// is the or of multiple attributes, then all attributes must be
    /// present in this PMA.
    bool hasAttrib(Attrib a) const
    { return (attrib_ & a) == a; }

    /// Return an integer represenation of the attributes. For now,
    /// just return as-is, could modify later.
    uint32_t attributesToInt() const
    { return attrib_; }

    /// Convert given string to a Pma object. Return true on success return false if
    /// string does not contain a valid attribute names.  Valid names: none, read, write,
    /// execute, idempotent, amo, mem_mapped, rsrv, io.
    static bool stringToAttrib(std::string_view str, Attrib& attrib)
    {
      static const std::unordered_map<std::string_view, Attrib> stringToAttrib = {
        { "none", Pma::None },
        { "read", Pma::Read },
        { "write", Pma::Write },
        { "exec", Pma::Exec },
        { "idempotent", Pma::Idempotent },
        { "amoswap", Pma::AmoSwap },
        { "amological", Pma::AmoLogical },
        { "amoother", Pma::AmoOther },
        { "amoarithmetic", Pma::AmoArith },
        { "amo", Pma::AmoArith },
        { "mem_mapped", Pma::MemMapped },
        { "rsrv", Pma::Rsrv },
        { "io", Pma::Io },
        { "cacheable", Pma::Cacheable },
        { "misal_ok", Pma::MisalOk },
        { "misal_acc_fault", Pma::MisalAccFault }
      };

      auto iter = stringToAttrib.find(str);
      if (iter != stringToAttrib.end())
        {
          attrib = iter->second;
          return true;
        }

      attrib = Pma::None;
      return false;
    }

    static std::string attributesToString(uint32_t attrib)
    {
      std::string result;

      result += (attrib & Pma::None)? "none," : "";
      result += (attrib & Pma::Read)? "read," : "";
      result += (attrib & Pma::Write)? "write," : "";
      result += (attrib & Pma::Exec)? "exec," : "";
      result += (attrib & Pma::Idempotent)? "idempotent," : "";
      result += (attrib & Pma::AmoOther)? "amoother," : "";
      result += (attrib & Pma::AmoSwap)? "amoswap," : "";
      result += (attrib & Pma::AmoLogical)? "amological," : "";
      result += (attrib & Pma::MemMapped)? "memmapped," : "";
      result += (attrib & Pma::Rsrv)? "rsrv," : "";
      result += (attrib & Pma::Io)? "io," : "";
      result += (attrib & Pma::Cacheable)? "cacheable," : "";
      result += (attrib & Pma::MisalOk)? "misalok," : "";
      result += (attrib & Pma::MisalAccFault)? "misalaccfault," : "";
      return result;
    }

  private:

    uint32_t attrib_ = 0;
  };


  /// Physical memory attribute manager. One per memory. Shared
  /// among cores and harts. Physical memory attributes apply to
  /// word-aligned regions as small as 1 word (but are expected to be
  /// applied to a few number of large regions).
  class PmaManager
  {
  public:

    friend class Memory;

    /// For architecture coverage.
    enum AccessReason { None, Fetch, LdSt };

    /// Constructor.
    PmaManager(uint64_t memorySize)
      : memSize_(memorySize)
    {
      noAccessPma_.enable(Pma::Attrib::MisalOk);
      regions_.reserve(32);
    }


    /// Destructor.
    ~PmaManager() = default;

    /// Return the physical memory attribute associated with the
    /// word-aligned address covering the given address. Return
    /// an unmapped attribute if the given address is out of memory
    /// range.
    inline Pma getPma(uint64_t addr) const
    {
      addr = (addr >> 2) << 2; // Make word aligned.

      // Search regions in order. Return first matching.
      for (const auto& region : regions_)
        if (region.valid_ and region.overlaps(addr))
          {
            if (not region.pma_.hasMemMappedReg())
              return region.pma_;
            return memMappedPma(region.pma_, addr);
          }

      if (addr >= memSize_)
        return noAccessPma_;
      return defaultPma_;  // rwx amo rsrv idempotent misalok
    }

    struct PmaTrace
    {
      unsigned ix_;
      uint64_t addr_;
      uint64_t baseAddr_;
      uint64_t lastAddr_;
      AccessReason reason_;
    };

    /// Similar to getPma but updates trace associated with each PMA entry
    inline Pma accessPma(uint64_t addr) const
    {
      addr = (addr >> 2) << 2; // Make word aligned.

#ifndef FAST_SLOPPY
      // Search regions in order. Return first matching.
      auto it = std::find_if(regions_.begin(), regions_.end(),
          [addr] (const auto& region) {
            return region.valid_ and region.overlaps(addr);
          });

      if (it != regions_.end())
        {
          if (trace_)
            pmaTrace_.push_back({unsigned(std::distance(regions_.begin(), it)),
                                  addr, it->firstAddr_, it->lastAddr_, reason_});
          const auto& region = *it;
          if (not region.pma_.hasMemMappedReg())
            return region.pma_;
          return memMappedPma(region.pma_, addr);
        }
#endif

      if (addr >= memSize_)
        return noAccessPma_;
      return defaultPma_;  // rwx amo rsrv idempotent misalok
    }

    /// Used for tracing to determine if an address matches multiple PMAs.
    bool matchMultiplePma(uint64_t addr) const
    {
      bool hit = false;
      for (const auto& region : regions_)
        if (region.valid_ and region.overlaps(addr))
          {
            if (hit)
              return true;
            hit = true;
          }
      return false;
    }

    /// Define/re-define a physical memory attribute region at given index ix (indices are
    /// 0 to n-1 where n is the region count). Regions are checked in order order (if an
    /// address is covered by multiple regions, then the first defined region applies. The
    /// defined region consists of the word-aligned words with addresses between fistAddr
    /// and lastAddr inclusive. For example, if firstAddr is 5 and lastAddr is 13, then
    /// the defined region consists of the words at 8 and 12 (bytes 8 to 15). Return true
    /// on success.
    bool defineRegion(unsigned ix, uint64_t firstAddr, uint64_t lastAddr, Pma pma)
    {
      Region region{firstAddr, lastAddr, pma, true};
      if (ix >= 128)
        return false;  // Arbitrary limit.

      if (ix >= regions_.size())
        regions_.resize(ix + 1);
      regions_.at(ix) = region;

      // If definition comes from config file, remember memory mapped address range.
      if (pma.hasMemMappedReg())
        {
          if (ix >= memMappedRanges_.size())
            memMappedRanges_.resize(ix + 1);
          memMappedRanges_.at(ix) = std::make_pair(firstAddr, lastAddr);
        }
      return true;
    }

    /// Mark entry at given index as invalid.
    void invalidateEntry(unsigned ix)
    {
      if (ix >= 128)
        return;  // Arbitrary limit.
      if (ix >= regions_.size())
        regions_.resize(ix + 1);
      regions_.at(ix).valid_ = false;
    }

    /// Define a memory mapped register. Return true on success and false if size is not 4
    /// or 8 or if the address is not word/double-word aligned.
    bool defineMemMappedReg(uint64_t addr, uint64_t mask, unsigned size, Pma pma)
    {
      if (size != 4 and size != 8)
        return false;

      if ((addr & (size - 1)) != 0)
        return false;   // Not aligned.

      MemMappedReg mmr {.mask_ = mask, .size_ = size, .pma_ = pma};
      memMappedRegs_[addr] = mmr;
      return true;
    }

    /// Return mask associated with the word-aligned word at the given
    /// address.  Return 0xffffffff if no mask was ever associated
    /// with given address.
    uint64_t getMemMappedMask(uint64_t addr) const
    {
      auto iter = memMappedRegs_.find(addr);
      if (iter == memMappedRegs_.end())
        return ~uint64_t(0);
      return iter->second.mask_;
    }

    /// Return true if given address is whitin a memory mapped register.
    bool isMemMappedReg(size_t addr) const
    {
      addr = (addr >> 2) << 2;   // Make a multiple of 4.
      if (memMappedRegs_.find(addr) != memMappedRegs_.end())
        return true;
      addr = (addr >> 3) << 3;   // Make a multiple of 8.
      return memMappedRegs_.find(addr) != memMappedRegs_.end();
    }

    /// Enable misaligned data access in default PMA.
    void enableMisalignedData(bool flag)
    {
      if (flag)
        {
          defaultPma_.enable(Pma::Attrib::MisalOk);
          noAccessPma_.enable(Pma::Attrib::MisalOk);
        }
      else
        {
          defaultPma_.disable(Pma::Attrib::MisalOk);
          noAccessPma_.disable(Pma::Attrib::MisalOk);
        }
    }

    /// Clear the default PMA (no access).
    void clearDefaultPma()
    { defaultPma_.attrib_ = Pma::Attrib::None; }

    /// Enable given attributes in the default PMA.
    void enableInDefaultPma(Pma::Attrib a)
    { defaultPma_.enable(a); }

    /// Return true if the given range [start,end] overlaps a memory mapped register
    /// region.
    bool overlapsMemMappedRegs(uint64_t start, uint64_t end) const
    {
      return std::any_of(memMappedRanges_.begin(), memMappedRanges_.end(),
          [start, end] (const auto& region) {
            auto [low, high] = region;
            if (end >= low && start <= high)
              return true;
            return false;
          });
    }

    const std::vector<PmaTrace>& getPmaTrace() const
    { return pmaTrace_; }

    void clearPmaTrace()
    { pmaTrace_.clear(); }

    void enableTrace(bool flag)
    { trace_ = flag; }

    /// This is to differentiate fetch from ld/st accesses.
    void setAccReason(AccessReason reason)
    { reason_ = reason; }

    /// Print current pma map matching a particular address.
    void printPmas(std::ostream& os, uint64_t address) const
    {
      auto region = getRegion(address);
      printRegion(os, region);
    }

    /// Print current pma map.
    void printPmas(std::ostream& os) const
    {
      for (size_t i = 0; i < regions_.size(); ++i)
        {
          os << "Region " << i << '\n';
          const auto& region = regions_.at(i);
          printRegion(os, region);
          os << '\n';
        }
    }
    /// Mark region as having memory mapped registers if it overlapps such registers.
    void updateMemMappedAttrib(unsigned ix)
    {
      auto& region = regions_.at(ix);

      for (auto& range : memMappedRanges_)
        if (region.overlaps(range.first, range.second))
          region.pma_.enable(Pma::Attrib::MemMapped);
    }

    /// Unpack the value of a PMACFG CSR.
    static void unpackPmacfg(uint64_t value, bool& valid, uint64_t& low, uint64_t& high,
                             Pma& pma)
    {
      // Recover n = log2 of size.
      uint64_t n = value >> 58;   // Bits 63:58
      valid = n != 0;
      if (not valid)
        return;

      if (n < 12)
        n = 12;

      // Default: misaligned load/store allowed everywhere. This does not apply to AMO/LR/SC.
      unsigned attrib = Pma::Attrib::MisalOk;

      if (value & 1)  attrib |= Pma::Attrib::Read;
      if (value & 2)  attrib |= Pma::Attrib::Write;
      if (value & 4)  attrib |= Pma::Attrib::Exec;

      // FIX : Support io channel0 and channel1
      unsigned memType = (value >> 3) & 3;   // Bits 4:3
      if (memType != 0)
        {    // IO
          attrib |= Pma::Attrib::Io;
          attrib &= ~Pma::Attrib::MisalOk;       // No misaligned access in IO region.
          attrib |= Pma::Attrib::MisalAccFault;  // Misal access triggers access fault.
        }
      else
        {   // Regular memory.
          bool cacheable = value & 0x80;  // Bit 7
          if (cacheable)
            {
              attrib |= Pma::Attrib::Cacheable;
              attrib |= Pma::Attrib::Rsrv;

              unsigned amoType = (value >> 5) & 3;   // Bits 6:5
              if      (amoType == 1)  attrib |= Pma::Attrib::AmoSwap;
              else if (amoType == 2)  attrib |= Pma::Attrib::AmoLogical;
              else if (amoType == 3)  attrib |= Pma::Attrib::AmoArith;
            }
        }

      pma = Pma(Pma::Attrib(attrib));

      // Recover base address: Bits 55:12
      uint64_t addr = (value << 8) >> 8;  // Clear most sig 8 bits of value.
      low = (addr >> n) << n;  // Clear least sig n bits.
      high = ~uint64_t(0);
      if (n < 56)
        {
          high = low;
          high |= (uint64_t(1) << n) - 1; // Set bits 0 to n-1
        }
    }

    /// Legalize the value of a PMACFG CSR: Modify next to make it legal. Use prev to
    /// retain fields that are illegal in next.
    static uint64_t legalizePmacfg(uint64_t prev, uint64_t next)
    {
      // If any of the fields of next are illegal, keep prev value.
      uint64_t val = next;

      uint64_t n = val >> 58;
      if (n > 0 and n < 12)
        return prev;

      bool read = (val & 1);       // bit 0
      bool write = (val & 2);      // bit 1
      bool exec = (val & 4);       // bit 2
      bool cacheable = val & 0x80; // Bit 7
      bool coherent = val & 0x100; // Bit 8, routing for IO.

      unsigned memType = (val >> 3) & 3;   // Bits 4:3
      bool io = memType != 0;

      unsigned amo = (val >> 5) & 3;   // Bits 6:5

      if (io)
        {
          if (write and !read and !exec)
            return prev;
          if (amo != 0)
            return prev;  // IO must be amo-none.
          if (write and not read)
            return prev;  // Cannot have write without read.
          if (coherent)
            return prev;  // IO routing constraint.
        }
      else
        {
          // Either RWX or no access.
          unsigned count = read + write + exec;
          if (count != 0 and count != 3)
            return prev;

          if (cacheable and amo != 3)
            return prev;   // Cacheable must be amo-arithmetic.
          if (not cacheable and amo != 0)
            return prev;   // Non-cacheable must be amo-none.
          if (cacheable and not coherent)
            return prev;
        }

      return next;
    }

  protected:

    /// Reset (to zero) all memory mapped registers.
    void resetMemMapped()
    { for (auto& kv  : memMappedRegs_) kv.second.value_ = 0; }

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr does not fall in a memory-mapped register.
    bool readRegister(uint64_t addr, uint8_t& value) const
    {
#if 0
      return false;  // Only word or double-word allowed.
#endif

      uint64_t aa = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(aa);
      if (iter == memMappedRegs_.end())
        {
          aa = (addr >> 3) << 3;  // Make double-word aligned.
          iter = memMappedRegs_.find(aa);
          if (iter == memMappedRegs_.end())
            return false;
        }

      uint64_t mmrv = iter->second.value_;  // MMR value
      uint64_t offset = addr - aa;
      value = mmrv >> (offset*8);
      return true;
    }

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr is not half-word aligned or does not fall in a
    /// memory-mapped register.
    bool readRegister(uint64_t addr, uint16_t& value) const
    {
#if 0
      return false;  // Only word or double-word allowed.
#endif

      if ((addr & 1) != 0)
        return false;  // Not half-word aligned.

      uint64_t aa = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(aa);
      if (iter == memMappedRegs_.end())
        {
          aa = (addr >> 3) << 3;  // Make double-word aligned.
          iter = memMappedRegs_.find(aa);
          if (iter == memMappedRegs_.end())
            return false;
        }

      uint64_t mmrv = iter->second.value_;  // MMR value
      uint64_t offset = addr - aa;
      value = mmrv >> (offset*8);
      return true;
    }

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr is not word aligned or does not fall in a
    /// memory-mapped register.
    bool readRegister(uint64_t addr, uint32_t& value) const
    {
      if ((addr & 3) != 0)
        return false;  // Not word aligned.

      uint64_t aa = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(addr);
      if (iter == memMappedRegs_.end())
        {
          aa = (addr >> 3) << 3;  // Make double-word aligned.
          iter = memMappedRegs_.find(aa);
          if (iter == memMappedRegs_.end())
            return false;
        }

      uint64_t mmrv = iter->second.value_;  // MMR value
      uint64_t offset = addr - aa;
      value = mmrv >> (offset*8);
      return true;
    }

    /// Set value to the value of the memory mapped register at addr returning true if
    /// addr is valid. Return false if addr is not double-word aligned or is not that of a
    /// memory-mapped register.
    bool readRegister(uint64_t addr, uint64_t& value) const
    {
      if ((addr & 7) != 0)
        return false;   // Not double-word aligned.

      auto iter = memMappedRegs_.find(addr);
      if (iter == memMappedRegs_.end())
        return false;

      value = iter->second.value_;

      if (iter->second.size_ == 4)
        {
          // Loaded least sig 4 bytes from a word MMR, see if we can load most sig 4 bytes.
          addr += 4;
          iter = memMappedRegs_.find(addr);
          if (iter != memMappedRegs_.end())
            value |= iter->second.value_ << 32;
        }

      return true;
    }

    /// Set the value of the byte of the memory mapped regiser at addr to the given value
    /// returning true if addr is valid. Return false if addr does not fall in a memory
    /// mapped reg.
    bool writeRegister(uint64_t addr, uint8_t value)
    {
#if 0
      return false;  // Only word or double-word allowed.
#endif

      uint64_t aa = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(aa);
      if (iter == memMappedRegs_.end())
        {
          aa = (addr >> 3) << 3;  // Make double-word aligned.
          iter = memMappedRegs_.find(aa);
          if (iter == memMappedRegs_.end())
            return false;
        }

      uint64_t shift = (addr - aa) * 8;
      uint64_t mask = uint64_t(0xff) << shift;  // Byte mask
      uint64_t shiftedValue = uint64_t(value) << shift;

      uint64_t& mmrv = iter->second.value_;  // MMR value
      uint64_t mmrm = iter->second.mask_;    // MMR mask

      mmrv = (mmrv & ~mask & ~mmrm) | (shiftedValue & mmrm);
      return true;
    }

    /// Set the value of the half-word of the memory mapped regiser at addr to the given
    /// value returning true if addr is valid. Return false if addr does not fall in a
    /// memory mapped reg or if addr is not half-word aligned.
    bool writeRegister(uint64_t addr, uint16_t value)
    {
#if 0
      return false;  // Only word or double-word allowed.
#endif

      if ((addr & 1) != 0)
        return false;  // Not half-word aligned.

      uint64_t aa = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(aa);
      if (iter == memMappedRegs_.end())
        {
          aa = (addr >> 3) << 3;  // Make double-word aligned.
          iter = memMappedRegs_.find(aa);
          if (iter == memMappedRegs_.end())
            return false;
        }

      uint64_t shift = (addr - aa) * 8;
      uint64_t mask = uint64_t(0xffff) << shift;  // Half-word mask
      uint64_t shiftedValue = uint64_t(value) << shift;

      uint64_t& mmrv = iter->second.value_;  // MMR value
      uint64_t mmrm = iter->second.mask_;    // MMR mask

      mmrv = (mmrv & ~mask & ~mmrm) | (shiftedValue & mmrm);
      return true;
    }

    /// Set the value of the word of the memory mapped regiser at addr to the given
    /// value returning true if addr is valid. Return false if addr does not fall in a
    /// memory mapped reg or if addr is not word aligned.
    bool writeRegister(uint64_t addr, uint32_t value)
    {
      if ((addr & 3) != 0)
        return false;  // Not word aligned.

      uint64_t aa = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(aa);
      if (iter == memMappedRegs_.end())
        {
          aa = (addr >> 3) << 3;  // Make double-word aligned.
          iter = memMappedRegs_.find(aa);
          if (iter == memMappedRegs_.end())
            return false;
        }

      uint64_t shift = (addr - aa) * 8;
      uint64_t mask = uint64_t(0xffffffff) << shift;  // Word mask
      uint64_t shiftedValue = uint64_t(value) << shift;

      uint64_t& mmrv = iter->second.value_;  // MMR value
      uint64_t mmrm = iter->second.mask_;    // MMR mask

      mmrv = (mmrv & ~mask & ~mmrm) | (shiftedValue & mmrm);
      return true;
    }
    /// Set the value of the the memory mapped regiser(s) overlapping addr to the given
    /// value returning true if addr is valid. Return false if addr does not fall in a
    /// memory mapped reg or if addr is not double-word aligned.
    bool writeRegister(uint64_t addr, uint64_t value)
    {
      if ((addr & 7) != 0)
        return false;   // Not double-word aligned.

      auto iter = memMappedRegs_.find(addr);
      if (iter == memMappedRegs_.end())
        return false;

      uint64_t& mmrv = iter->second.value_;  // MMR value
      uint64_t mmrm = iter->second.mask_;    // MMR mask

      mmrv = value & mmrm;

      if (iter->second.size_ == 4)
        {
          // Wrote least sig 4 bytes into a word MMR, see if we can write most sig 4 bytes.
          addr += 4;
          iter = memMappedRegs_.find(addr);
          if (iter != memMappedRegs_.end())
            {
              value >>= 32;
              uint64_t& mmrv2 = iter->second.value_;
              uint64_t mmrm2 = iter->second.mask_;
              auto mask = uint64_t(0xffffffff);
              mmrv2 = (mmrv2 & ~mask & mmrm2) | (value & mmrm2);
            }
        }

      return true;
    }
    /// Return true if write is allowed.
    bool checkRegisterWrite(uint64_t addr, unsigned size) const
    {
#if 0
      if (size != 4 and size != 8)
        return false;
#endif

      unsigned mask = size - 1;
      if ((addr & mask) != 0)
        return false;   // Not aligned.

      addr = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(addr);
      if (iter != memMappedRegs_.end())
        return true;

      addr = (addr >> 3) << 3;  // Make double-word aligned.
      return memMappedRegs_.find(addr) != memMappedRegs_.end();
    }

    /// Return true if read is allowed.
    bool checkRegisterRead(uint64_t addr, unsigned size) const
    {
#if 0
      if (size != 4 and size != 8)
        return false;
#endif

      unsigned mask = size - 1;
      if ((addr & mask) != 0)
        return false;   // Not aligned.

      addr = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(addr);
      if (iter != memMappedRegs_.end())
        return true;

      addr = (addr >> 3) << 3;  // Make double-word aligned.
      return memMappedRegs_.find(addr) != memMappedRegs_.end();
    }
    /// Similar to writeRgister but no masking is applied to value.
    bool pokeRegister(uint64_t addr, uint64_t value);

    /// Similar to writeRgister but no masking is applied to value.
    bool pokeRegisterByte(uint64_t addr, uint8_t value)
    {
      uint64_t aa = (addr >> 2) << 2;  // Make word aligned.
      auto iter = memMappedRegs_.find(aa);
      if (iter == memMappedRegs_.end())
        {
          aa = (addr >> 3) << 3;  // Make double-word aligned.
          iter = memMappedRegs_.find(aa);
          if (iter == memMappedRegs_.end())
            return false;
        }

      uint64_t shift = (addr - aa) * 8;
      uint64_t mask = uint64_t(0xff) << shift;  // Byte mask
      uint64_t shiftedValue = uint64_t(value) << shift;

      uint64_t& mmrv = iter->second.value_;  // MMR value

      mmrv = (mmrv & ~mask) | shiftedValue;
      return true;
    }

    /// Return the memory mapped register PMA associated with the given address or the
    /// given PMA if address does not correspond to a memory mapped register.  Address is
    /// expected to be word aligned.
    Pma memMappedPma(Pma pma, uint64_t addr) const
    {
      auto iter = memMappedRegs_.find(addr);
      if (iter == memMappedRegs_.end())
        iter = memMappedRegs_.find((addr >> 3) << 3);  // Check double word aligned address.
      return iter == memMappedRegs_.end() ? pma : iter->second.pma_;
    }

  private:

    struct Region
    {
      bool overlaps(uint64_t addr) const
      { return addr >= firstAddr_ and addr <= lastAddr_; }

      bool overlaps(uint64_t low, uint64_t high) const
      { return high >= firstAddr_ && low <= lastAddr_; }

      uint64_t firstAddr_ = 0;
      uint64_t lastAddr_ = 0;
      Pma pma_;
      bool valid_ = false;
    };

    struct MemMappedReg
    {
      uint64_t value_ = 0;
      uint64_t mask_ = ~uint64_t(0);
      unsigned size_ = 4;
      Pma pma_;
    };

    /// Return the Region object associated with the word-aligned word containing the
    /// given address. Return a no-access object if the given address is out of memory
    /// range.
    Region getRegion(uint64_t addr) const
    {
      addr = (addr >> 2) << 2;
      for (const auto& region : regions_)
        if (region.valid_ and region.overlaps(addr))
          return region;

      if (addr >= memSize_)
        return { .pma_ = noAccessPma_ };
      return { .pma_ = defaultPma_ };  // rwx amo rsrv idempotent misalok
    }

    /// Print current pmp map matching a particular address.
    static void printRegion(std::ostream& os, Region region)
    {
      const auto& pma = region.pma_;
      os << "valid: " << std::hex << region.valid_ << "\n";

      if (not region.valid_)
        return;

      os << std::hex;
      os << "base addr: 0x" << region.firstAddr_ << "\n";
      os << "last addr: 0x" << region.lastAddr_ << "\n";
      os << std::dec;

      os << "attributes: " << Pma::attributesToString(pma.attrib_) << "\n";
    }

    std::vector<Region> regions_;
    uint64_t memSize_ = 0;
    Pma defaultPma_{Pma::Attrib::Default};
    Pma noAccessPma_{Pma::Attrib::None};

    std::unordered_map<uint64_t, MemMappedReg> memMappedRegs_;
    std::vector<std::pair<uint64_t, uint64_t>> memMappedRanges_;

    bool trace_ = false;  // Collect stats if true.
    mutable std::vector<PmaTrace> pmaTrace_;
    AccessReason reason_{};
  };
}
