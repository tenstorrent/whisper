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
#include <cassert>

namespace TT_IOMMU
{

  /// Union to pack/unpack process id bits.
  union Procid
  {
    Procid(uint32_t value)
      : value_(value)
    { }

    unsigned ithPdi(unsigned i) const
    {
      if (i == 0)
        return bits_.pdi0_;
      if (i == 1)
        return bits_.pdi1_;
      if (i == 2)
        return bits_.pdi2_;
      assert(0);
      return 0;
    }

    uint32_t value_;  // First variant of union

    struct   // Second variant
    {
      unsigned pdi0_   : 8;
      unsigned pdi1_   : 9;
      unsigned pdi2_   : 3;
      unsigned unused_ : 12;
    } bits_;
  };


  /// Address translation mode for first stage.
  enum class IosatpMode : uint32_t
    {
      Bare = 0, Sv32 = 1, Sv39 = 8, Sv48 = 9, Sv57 = 10
    };


  /// Union to pack/unpack the Iosatp register.
  union Iosatp
  {
    Iosatp(uint64_t value)
      : value_(value)
    { }

    uint64_t value_;   // First variant of union

    struct     // Second variant
    {
      uint64_t ppn_      : 44;
      unsigned reserved_ : 16;
      IosatpMode mode_   : 4;
    } bits_;
  };


  /// Union to pack/unpack the PDTE (non-leaf process diretory table entry).
  union Pdte
  {
    Pdte(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;   // First variant of union

    struct     // Second variant
    {
      unsigned v_      : 1;
      unsigned res0_   : 9;   // Reserved.
      uint64_t ppn_    : 44;
      unsigned res1_   : 10;
    } bits_;
  };


  /// Union to pack/unpack the translation attribute of the process context.
  union ProcTransAttrib
  {
    ProcTransAttrib(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;

    struct
    {
      uint32_t v_     : 1;
      uint32_t ens_   : 1;
      uint32_t sum_   : 1;
      uint32_t res0_  : 9;  // Reserved
      uint32_t pscid_ : 20;
      uint32_t res1_  : 32; // Reserved
    } bits_;
  };


  /// Union to pack/unpack first stage context.
  union Fsc
  {
    Fsc(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_ = 0;   // First variant

    struct Bits    // Second variant
    {
      uint64_t ppn_  : 44;
      uint32_t res_  : 16;
      uint32_t mode_ : 4;
    } bits_;
  };


  /// Model a process context. Section 2.2.2, 2.2.3, and 2.2.4 of the IOMMU spec.
  class ProcessContext
  {
  public:

    ProcessContext() = default;

    ProcessContext(uint64_t procTransAttrib, uint64_t fsc)
      : ta_(procTransAttrib), fsc_(fsc)
    { }

    /// Return true if this context is valid (bit V of TA is 1).
    bool valid() const
    { return ta_.bits_.v_; }

    /// Return true if supervisor access is enabled in this context
    /// (bit ENS of TA is 1).
    bool ens() const
    { return ta_.bits_.ens_; }

    /// Return true if supervisor access of user pages is enabled in
    /// this context (bit SUM of TA is 1).
    bool sum() const
    { return ens() and (ta_.bits_.sum_); }

    /// Return the process soft context id (bits PSCID of TA).
    unsigned pscid() const
    { return ta_.bits_.pscid_; }

    /// Return the first stage address translation mode of this context
    /// (bits MODE of FSC).
    IosatpMode iosatpMode() const
    { return IosatpMode{fsc_.bits_.mode_}; }

    /// Return the first stage address translation root page table number
    /// of this context (bits PPN of FSC).
    uint64_t iosatpPpn() const
    { return fsc_.bits_.ppn_; }

    /// Return true if any of the reserved bits in this context are non zero.
    bool nonZeroReservedBits() const
    { return (ta_.value_ & taResMask()) != 0 or (fsc_.value_ & fscResMask()) != 0; }

    /// Set the TA and FSC fields of this context to the given values.
    void set(uint64_t ta, uint64_t fsc)
    { ta_ = ta; fsc_ = fsc; }

    /// Return the ta (translation attribute) field of this object.
    uint64_t ta() const
    { return ta_.value_; }

    /// Return the ta (first stage context) field of this object.
    uint64_t fsc() const
    { return fsc_.value_; }

    /// Comparison operator: compare all the fields.
    bool operator==(const ProcessContext& other) const
    { return ta_.value_ == other.ta_.value_ and fsc_.value_ == other.fsc_.value_; }

  protected:

    /// Return mask of reserved bits in TA field.
    static uint64_t taResMask()
    { return 0xffff'ffff'0000'0ff8; }

    /// Return mask of reserved bits in FSC field.
    static uint64_t fscResMask()
    { return 0x0fff'f000'0000'0000; }

  private:

    ProcTransAttrib ta_ = 0;  // Translation attributes.
    Fsc fsc_ = 0;             // First stage context.
  };

}
