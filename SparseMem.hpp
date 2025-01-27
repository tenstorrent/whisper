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
#include <cstring>
#include <memory>
#include <vector>
#include <span>
#include <unordered_map>
#include <mutex>
#include "Lock.hpp"


namespace WdRiscv
{

  /// Memory model. Host machine memory is conserved by allocating
  /// pages only for addresses refereneced.
  class SparseMem
  {
  public:

    SparseMem() : pageMapCache_(cacheSize_),
        cacheSpinLock_(std::make_unique<SpinLock[]>(cacheSize_)) {}

    ~SparseMem();

    /// Read an unsigned item of the given size (1, 2, 4 or 8 bytes)
    /// from the given target-machine address placing the item bits
    /// in value. Return true on success and false on failure (out of
    /// bounds address or size).
    bool read(uint64_t addr, unsigned size, uint64_t& value);

    /// Write an unsigned item of the given size (1, 2, 4 or 8 bytes)
    /// to the given target-machine address gettng the item bits from
    /// the least sig bits of value. Return true on success and false
    /// on failure (out of bounds address or size).
    bool write(uint64_t addr, unsigned size, uint64_t value);

    /// Write the contents of the memory to a verilog hex file. Return
    /// true on success and false on failure.
    bool writeHexFile(const std::string& path) const;

    /// Fill the given vector with the addresses/sizes of the
    /// used memory areas (pages) sorted in ascending order.
    void getUsedBlocks(std::vector<std::pair<uint64_t, uint64_t>>& vec) const;

    /// Initialize page at the given address with the contents of given buffer. Buffer
    /// size must be greater than or equal to the page size.
    bool initializePage(uint64_t addr, const std::span<uint8_t> buffer);

  protected:

    /// Read from given target-machine address an item of type U
    /// (uint8_t, uint16_t, uint32_t or uin64_t) and place it in
    /// value. Address must be aligned.
    template <typename U>
    bool
    read(uint64_t addr, uint64_t& value)
    {
      uint64_t pageRank = getPageRank(addr);
      std::vector<uint8_t>& page = findOrCreatePage(pageRank);
      unsigned offset = addr & pageMask_;
      value = *( reinterpret_cast<U*>(page.data() + offset) );
      return true;
    }

    /// Write to the given target-machine address an item of type U
    /// (uint8_t, uint16_t, uint32_t or uin64_t). Address must be
    /// aligned. Item bits are in the least significant bits of
    /// value.
    template <typename U>
    bool
    write(uint64_t addr, uint64_t value)
    {
      uint64_t pageRank = getPageRank(addr);
      std::vector<uint8_t>& page = findOrCreatePage(pageRank);
      unsigned offset = addr & pageMask_;
      *( reinterpret_cast<U*>(page.data() + offset) ) = value;
      return true;
    }

    /// Return the page number of the page containing the byte at the
    /// given address.
    uint64_t getPageRank(uint64_t addr) const
    { return addr >> pageShift_; }

    /// Return host-machine address of the target-machine page with
    /// the given page number creating such a page (and zeroing it) if
    /// it has never been accessed before.
    inline std::vector<uint8_t>& findOrCreatePage(uint64_t pageNum)
    {
      uint64_t idx = pageMapCache_.idx(pageNum);
      std::lock_guard<SpinLock> cacheLock(cacheSpinLock_[idx]);

      std::vector<uint8_t>* p = pageMapCache_.find(pageNum, idx);
      if (p != nullptr) {
        return *p;
      }

      std::lock_guard<SpinLock> mapLock(mapSpinLock_);

      auto end = pageMap_.end();
      auto iter = pageMap_.find(pageNum);
      if (iter != end) {
        std::vector<uint8_t>& page = iter->second;
        pageMapCache_.update(pageNum, page, idx);
    	return page;
      }

      return createPage(pageNum);
    }

  private:

    class PageMapCache {
        struct entry_t {
            uint64_t pageNum = 0;
            std::vector<uint8_t>* page = nullptr;
        };
        uint64_t mask_;
        std::vector<entry_t> cache_;

        inline std::vector<uint8_t>* get_page(uint64_t pageNum, entry_t& e) {
            if (e.page == nullptr or e.pageNum != pageNum) {
                return nullptr;
            }
            return e.page;
        }

        public:
        PageMapCache(uint64_t n) : mask_(n - 1) {
            assert((n & (n - 1)) == 0 && "Cache size must be a power of 2");
            cache_.resize(n);
        }

        inline uint64_t idx(uint64_t pageNum) { return pageNum & mask_; }

        inline std::vector<uint8_t>* find(uint64_t pageNum) {
            entry_t& e = cache_[idx(pageNum)];
            return get_page(pageNum, e);
        }

        inline std::vector<uint8_t>* find(uint64_t pageNum, uint64_t idx) {
            entry_t& e = cache_[idx];
            return get_page(pageNum, e);
        }

        inline void update(uint64_t pageNum, std::vector<uint8_t>& page) {
            uint64_t idx = this->idx(pageNum);
            cache_[idx].pageNum = pageNum;
            cache_[idx].page = &page;
        }

        inline void update(uint64_t pageNum, std::vector<uint8_t>& page, uint64_t idx) {
            cache_[idx].pageNum = pageNum;
            cache_[idx].page = &page;
        }

        size_t size() { return cache_.size(); }
    };

    std::vector<uint8_t>& createPage(uint64_t pageNum)
    {
	  auto iter = pageMap_.try_emplace(pageNum, pageSize_, 0).first;
      auto& page = iter->second;
	  pageMapCache_.update(pageNum, page);
      return page;
    }

    size_t pageSize_ = UINT64_C(4)*1024;
    unsigned pageShift_ = 12;
    unsigned pageMask_ = 0xfff;
    size_t cacheSize_ = 16384;

    std::unordered_map<uint64_t, std::vector<uint8_t>> pageMap_;  // Map address to page
    SpinLock mapSpinLock_;
    PageMapCache pageMapCache_;
    std::unique_ptr<SpinLock[]> cacheSpinLock_;
  };
}
