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

#include <vector>
#include "InstId.hpp"
#include "VecRegs.hpp"

namespace WdRiscv
{

  struct InstProfile
  {
    InstId id_ = InstId::illegal;
    uintmax_t freq_ = 0;       // Number of times instruction was executed
    uintmax_t user_ = 0;       // Number of times exeuted in user mode.
    uintmax_t supervisor_ = 0; // Number of times exeuted in supervisor mode.
    uintmax_t machine_ = 0;    // Number of times exeuted in machine mode.
    ElementWidth elemWidth_ = ElementWidth::Byte;  // For vector instructions.
  };


  class InstProfiles
  {
  public:

    InstProfiles() = default;

    void configure();

    InstProfile* find(InstId id)
    { return size_t(id) < vec_.size()? &vec_.at(size_t(id)) : nullptr; }

    InstProfile* find(InstId id, ElementWidth width)
    {
      size_t base = unsigned(InstId::endId_);
      size_t multiplier = unsigned(width);
      size_t ix = base*multiplier + size_t(id);
      return ix < vec_.size()? &vec_.at(ix) : nullptr;
    }

    /// Set the elements of the given vector to the indices of the instruction records
    /// sorted by frequency.
    void sort(std::vector<size_t>& indices) const;

    /// Return number of records in this container.
    size_t size();

    /// Return the ith entry in this container or null if i is out of bounds.
    const InstProfile* ithEntry(size_t i) const
    { return i < vec_.size() ? &vec_.at(i) : nullptr; }

  private:

    std::vector<InstProfile> vec_;
  };
}
