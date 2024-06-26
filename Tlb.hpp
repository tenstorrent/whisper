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
#include <ostream>
#include <vector>
#include <algorithm>

namespace WdRiscv
{

  /// Translation lookaside buffer entry.
  struct TlbEntry
  {
    uint64_t virtPageNum_ = 0;
    uint64_t physPageNum_ = 0;
    uint64_t counter_ = 0;   // 2-bit counter for replacement.
    uint32_t asid_ = 0;      // Address space identifier.
    uint32_t vmid_ = 0;      // Virtual machine identifier.
    bool valid_ = false;
    bool global_ = false;    //
    bool user_ = false;      // User-mode entry if true.
    bool read_ = false;      // Has read access.
    bool write_ = false;     // Write access.
    bool exec_ = false;      // Execute Access.
    bool accessed_ = false;
    bool dirty_ = false;
    uint8_t levels_ = 3;
    uint8_t pbmt_ = 0;
  };


  /// Translation lookaside buffer.
  class Tlb
  {
  public:

    /// Define a a TLB with the given size (number of entries).
    Tlb(unsigned size);

    /// Return pointer to TLB entry associated with given virtual page
    /// number and address space identifier.
    /// Return nullptr if no such entry.
    TlbEntry* findEntry(uint64_t pageNum, uint32_t asid)
    {
      auto* entry = getEntry(pageNum);

      if (entry and entry->valid_ and entry->virtPageNum_ == pageNum)
        if (entry->global_ or entry->asid_ == asid)
            return entry;
      return nullptr;
    }

    /// Return pointer to TLB entry associated with given virtual page
    /// number, address space identifier, and virtual machine identifier.
    /// Return nullptr if no such entry.
    TlbEntry* findEntry(uint64_t pageNum, uint32_t asid, uint32_t vmid)
    {
      auto* entry = getEntry(pageNum);

      if (entry and entry->valid_ and entry->virtPageNum_ == pageNum)
        if (entry->global_ or (entry->asid_ == asid and entry->vmid_ == vmid))
            return entry;
      return nullptr;
    }

    /// Return pointer to TLB entry associated with given virtual page
    /// number and address space identifier. Update entry time of
    /// access and increment time if entry is found. Return nullptr if
    /// no such entry.
    TlbEntry* findEntryUpdateTime(uint64_t pageNum, uint32_t asid)
    {
      auto* entry = findEntry(pageNum, asid);
      if (entry)
        {
          ++entry->counter_;
          entry->counter_ &= 3;
        }
      return entry;
    }

    /// Return pointer to TLB entry associated with given virtual page
    /// number and address space identifier. Update entry time of
    /// access and increment time if entry is found. Return nullptr if
    /// no such entry.
    TlbEntry* findEntryUpdateTime(uint64_t pageNum, uint32_t asid, uint32_t vmid)
    {
      auto* entry = findEntry(pageNum, asid, vmid);
      if (entry)
        {
          ++entry->counter_;
          entry->counter_ &= 3;
        }
      return entry;
    }

    /// print TLB content
    void printTlb(std::ostream& ost) const;

    /// print TLB entry
    static void printEntry(std::ostream& ost, const TlbEntry& te);

    /// Set number of TLB entries.
    void setTlbSize(unsigned size)
    { entries_.resize(size); }

    /// Insert a TLB entry for the given translation parameters. If TLB is full
    /// the contents of the  least recently accessed slot are replaced by the
    /// given parameters. Return true on success and false otherwise.
    bool insertEntry(uint64_t virtPageNum, uint64_t physPageNum,
                     uint32_t asid, bool global, bool isUser, bool read,
                     bool write, bool exec);

    /// Insert copy of given entry. Return true on success and false otherwise.
    bool insertEntry(const TlbEntry& entry);

    /// Invalidate every entry matching given address space identifier
    /// unless it is global.
    void invalidateAsid(uint32_t asid)
    {
      for (auto& entry : entries_)
	if (entry.asid_ == asid and not entry.global_)
          {
            entry.valid_ = false;
            entry.counter_ = 0;
          }
    }

    /// Invalidate every entry matching given virtual mahine identifier.
    void invalidateVmid(uint32_t vmid)
    {
      for (auto& entry : entries_)
	if (entry.vmid_ == vmid)
          {
            entry.valid_ = false;
            entry.counter_ = 0;
          }
    }

    /// Invalidate every entry matching given virtual page number.
    void invalidateVirtualPage(uint64_t vpn)
    {
      for (auto& entry : entries_)
	if (entry.virtPageNum_ == vpn)
          {
            entry.valid_ = false;
            entry.counter_ = 0;
          }
    }

    /// Invalidate every entry matching given virtual page number and
    /// address space identifer except for global entries.
    void invalidateVirtualPageAsid(uint64_t vpn, uint32_t asid)
    {
      for (auto& entry : entries_)
        if (entry.virtPageNum_ == vpn and entry.asid_ == asid and not entry.global_)
          {
            entry.valid_ = false;
            entry.counter_ = 0;
          }
    }

    /// Invalidate every entry matching given virtual page number and
    /// virtual machine identifer except for global entries.
    void invalidateVirtualPageVmid(uint64_t vpn, uint32_t vmid)
    {
      for (auto& entry : entries_)
        if (entry.virtPageNum_ == vpn and entry.vmid_ == vmid and not entry.global_)
          {
            entry.valid_ = false;
            entry.counter_ = 0;
          }
    }

    /// Invalidate all entries.
    void invalidate()
    {
      for (auto& entry : entries_)
        {
          entry.valid_ = false;
          entry.counter_ = 0;
        }
    }

  protected:

  private:

    /// Return reference to TLB entry associated with given virtual page
    /// number.
    inline TlbEntry* getEntry(uint64_t pageNum)
    {
      unsigned ix = pageNum & (entries_.size() - 1);
      return (entries_.size())? &entries_.at(ix) : nullptr;
    }

    std::vector<TlbEntry> entries_;
  };
}


