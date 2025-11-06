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
#include <array>

namespace TT_IOMMU
{

  /// Inbound transaction type.
  enum class Ttype : uint32_t
    {
      None          = 0,
      UntransExec   = 1,    // Read for exec (fetch).
      UntransRead   = 2,
      UntransWrite  = 3,
      Reserved      = 4,
      TransExec     = 5,    // Read for exec (fetch).
      TransRead     = 6,
      TransWrite    = 7,
      PcieAts       = 8,    // PCIE address translation service.
      PcieMessage   = 9
    };


  /// Iommu fault-queue record. Section 4.2 of spec
  struct FaultRecord
  {
    uint32_t cause    : 12 = 0;
    uint32_t pid      : 20 = 0;
    uint32_t pv       : 1  = 0;
    uint32_t priv     : 1  = 0;
    uint32_t ttyp     : 6  = 0;
    uint32_t did      : 24 = 0;
    uint32_t custom        = 0;
    uint32_t reserved      = 0;
    uint64_t iotval        = 0;
    uint64_t iotval2       = 0;
  };
  static_assert(sizeof(FaultRecord) == 32, "FaultRecord not 32 bytes in size");

  /// Interpret FaultRecord as a an array of double words.
  union FaultRecDwords
  {
    FaultRecDwords() : dwords{}
    { }

    FaultRecord rec;
    std::array<uint64_t, sizeof(FaultRecord)/8> dwords;
  };
}
