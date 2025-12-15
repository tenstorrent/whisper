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


namespace TT_IOMMU
{

  /// Union to pack/unpack first double word of basic MSI PTE.
  union MsiPte0
  {
    MsiPte0(uint64_t value)
      : value_(value)
    { }

    uint64_t value_;   // First variant of union

    struct    // Second variant
    {
      unsigned v_      : 1;   // Bit  0     Valid
      unsigned m_      : 2;   // Bits 2:1   Mode
      unsigned rsrv0_  : 7;   // Bits 9:3
      uint64_t ppn_    : 44;  // Bits 53:10 Root page number
      unsigned rsrv1_  : 9;   // Bits 62:54
      unsigned c_      : 1;   // Bit  63
    } bits_;
  };


  // Union to pack/unpack first double-word of MRIF MSI PTE.
  union MsiMrifPte0
  {
    MsiMrifPte0(uint64_t value)
    : value_(value)
    { }

    uint64_t value_;  // First variant of union

    struct   // Second variant
    {
      unsigned v_         : 1;   // Bit  0       Valid
      unsigned m_         : 2;   // Bits 2:1     Mode
      unsigned reserved0_ : 4;   // Bits 3:6
      uint64_t addr_      : 47;  // Bits 53:7    MRIF address
      unsigned reserved1_ : 9;   // Bits 62:54
      unsigned c_         : 1;   // Bit  63
    } bits_;
  };


  // Union to pack/unpack second double-word of MRIF MSI PTE.
  union MsiMrifPte1
  {
    MsiMrifPte1(uint64_t value)
    : value_(value)
    { }

    uint64_t value_;    // First variant of union

    struct   // Second variant
    {
      unsigned nidl_      : 10;  // Bits 9:0    NID[9:0]
      uint64_t nppn_      : 44;  // Bits 53:10  NPPN
      unsigned reserved0_ : 6;   // Bits 59:54
      unsigned nidh_      : 1;   // Bit  60     NID[10]
      unsigned reserved1_ : 3;
    } bits_;
  };

}
