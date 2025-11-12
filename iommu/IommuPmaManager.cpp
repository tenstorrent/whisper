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

#include <iostream>
#include <cmath>
#include <cassert>
#include "IommuPmaManager.hpp"

using namespace TT_IOMMU;


bool
Pma::stringToAttrib(std::string_view str, Pma::Attrib& attrib)
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


std::string
Pma::attributesToString(uint32_t attrib)
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


PmaManager::PmaManager(uint64_t memSize)
  : memSize_(memSize)
{
  noAccessPma_.enable(Pma::Attrib::MisalOk);
  regions_.reserve(32);
}


bool
PmaManager::defineMemMappedReg(uint64_t addr, uint64_t mask, unsigned size, Pma pma)
{
  if (size != 4 and size != 8)
    return false;

  if ((addr & (size - 1)) != 0)
    return false;   // Not aligned.

  MemMappedReg mmr {.mask_ = mask, .size_ = size, .pma_ = pma};
  memMappedRegs_[addr] = mmr;
  return true;
}


bool
PmaManager::defineRegion(unsigned ix, uint64_t firstAddr, uint64_t lastAddr, Pma pma)
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


uint64_t
PmaManager::getMemMappedMask(uint64_t addr) const
{
  auto iter = memMappedRegs_.find(addr);
  if (iter == memMappedRegs_.end())
    return ~uint64_t(0);
  return iter->second.mask_;
}


bool
PmaManager::readRegister(uint64_t addr, uint8_t& value) const
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


bool
PmaManager::readRegister(uint64_t addr, uint16_t& value) const
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


bool
PmaManager::readRegister(uint64_t addr, uint32_t& value) const
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


bool
PmaManager::readRegister(uint64_t addr, uint64_t& value) const
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


bool
PmaManager::writeRegister(uint64_t addr, uint8_t value)
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


bool
PmaManager::writeRegister(uint64_t addr, uint16_t value)
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


bool
PmaManager::writeRegister(uint64_t addr, uint32_t value)
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


bool
PmaManager::writeRegister(uint64_t addr, uint64_t value)
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


bool
PmaManager::checkRegisterRead(uint64_t addr, unsigned size) const
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



bool
PmaManager::checkRegisterWrite(uint64_t addr, unsigned size) const
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


bool
PmaManager::pokeRegisterByte(uint64_t addr, uint8_t value)
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


void
PmaManager::printRegion(std::ostream& os, Region region)
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


void
PmaManager::printPmas(std::ostream& os, uint64_t address) const
{
  auto region = getRegion(address);
  printRegion(os, region);
}


void
PmaManager::printPmas(std::ostream& os) const
{
  for (size_t i = 0; i < regions_.size(); ++i)
    {
      os << "Region " << i << '\n';
      const auto& region = regions_.at(i);
      printRegion(os, region);
      os << '\n';
    }
}


void
PmaManager::updateMemMappedAttrib(unsigned ix)
{
  auto& region = regions_.at(ix);

  for (auto& range : memMappedRanges_)
    if (region.overlaps(range.first, range.second))
      region.pma_.enable(Pma::Attrib::MemMapped);
}


void
PmaManager::unpackPmacfg(uint64_t val, bool& valid, uint64_t& low, uint64_t& high,
                         Pma& pma)
{
  // Recover n = log2 of size.
  uint64_t n = val >> 58;   // Bits 63:58
  valid = n != 0;
  if (not valid)
    return;

  if (n < 12)
    n = 12;

  // Default: misaligned load/store allowed everywhere. This does not apply to AMO/LR/SC.
  unsigned attrib = Pma::Attrib::MisalOk;

  if (val & 1)  attrib |= Pma::Attrib::Read;
  if (val & 2)  attrib |= Pma::Attrib::Write;
  if (val & 4)  attrib |= Pma::Attrib::Exec;

  // FIX : Support io channel0 and channel1
  unsigned memType = (val >> 3) & 3;   // Bits 4:3
  if (memType != 0)
    {    // IO
      attrib |= Pma::Attrib::Io;
      attrib &= ~Pma::Attrib::MisalOk;       // No misaligned access in IO region.
      attrib |= Pma::Attrib::MisalAccFault;  // Misal access triggers access fault.
    }
  else
    {   // Regular memory.
      bool cacheable = val & 0x80;  // Bit 7
      if (cacheable)
        {
          attrib |= Pma::Attrib::Cacheable;
          attrib |= Pma::Attrib::Rsrv;

          unsigned amoType = (val >> 5) & 3;   // Bits 6:5
          if      (amoType == 1)  attrib |= Pma::Attrib::AmoSwap;
          else if (amoType == 2)  attrib |= Pma::Attrib::AmoLogical;
          else if (amoType == 3)  attrib |= Pma::Attrib::AmoArith;
        }
    }

  pma = Pma(Pma::Attrib(attrib));

  // Recover base address: Bits 55:12
  uint64_t addr = (val << 8) >> 8;  // Clear most sig 8 bits of val.
  low = (addr >> n) << n;  // Clear least sig n bits.
  high = ~uint64_t(0);
  if (n < 56)
    {
      high = low;
      high |= (uint64_t(1) << n) - 1; // Set bits 0 to n-1
    }
}
