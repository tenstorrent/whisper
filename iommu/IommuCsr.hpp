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
#include <iostream>
#include "riscv_enums.hpp"

namespace TT_IOMMU
{

  /// IOMMU control and status register enumeration.
  enum class CsrNumber : uint32_t
    {
      Capabilities,
      Fctl,
      Custom0,
      Ddtp,
      Cqb,
      Cqh,
      Cqt,
      Fqb,
      Fqh,
      Fqt,
      Pqb,
      Pqh,
      Pqt,
      Cqcsr,
      Fqcsr,
      Pqcsr,
      Ipsr,
      Iocntovf,
      Iocntinh,
      Iohpmcycles,
      Iohpmctr1,
      Iohmpctr31 = Iohpmctr1 + 30,
      Iohpmevt1,
      Iohpmevt31 = Iohpmevt1 + 30,
      TrReqIova,
      TrReqCtl,
      TrResponse,
      IommuQosid,
      Reserved0,
      Reserved1,
      Reserved2,
      Reserved3,
      Reserved4,
      Reserved5,
      Reserved6,
      Reserved7,
      Custom1,
      Custom9 = Custom1 + 8,
      Icvec,
      MsiAddr0,
      MsiData0,
      MsiVecCtl0,
      MsiAddr15 = MsiAddr0 + 3*15,
      MsiData15,
      MsiVecCtl15,
    };


  // Bits that are RW1S (writing 0 has no effect, writing 1 will set):
  //   tr_req_ctl bit 0
  //
  // Bits that are RW1C (writing 0 has no effect, writing 1 will clear):
  //   cqcsr  bits 8, 9, 10, 11
  //   fqcsr  bits 8, 9
  //   pqcsr  bits 8, 9
  //   ipsr   bits 0, 1, 2, 3


  /// Union to Pack/unpack device directory table pointer.
  union Ddtp
  {
    enum class Mode : uint32_t
    {
      Off = 0, Bare = 1, Level1 = 2, Level2 = 3, Level3 = 4
    };

    Ddtp(uint64_t value = 0)
      : value_(value)
    { }

    /// Return the number of levels encoded in this DDTP or 0 if no valid number of
    /// levels.
    unsigned levels() const
    {
      switch (bits_.mode_)
      {
      case Mode::Level1: return 1;
      case Mode::Level2: return 2;
      case Mode::Level3: return 3;
      default: return 0;
      }
      return 0;
    }

    Mode mode() const
    { return bits_.mode_; }

    uint64_t ppn() const
    { return bits_.ppn_; }

    uint64_t value_;  // First variant of union

    // Second variant: Bit fields of DDTP.
    struct
    {
      Mode mode_          : 4;
      unsigned busy_      : 1;
      unsigned reserved0_ : 5;
      uint64_t ppn_       : 44;
      unsigned reserved1_ : 10;
    } bits_;
  };


  /// Values fo interrupt generation support (IGS) field of Capabilities.
  enum class IgsMode : uint32_t
    {
      Msi, Wsi, Both, Reserved
    };


  /// Union to pack/unpack capabilities register.
  union Capabilities
  {
    Capabilities(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;   // First variant of union

    /// Bit fields of capabilities register.
    struct
    {
      unsigned version_ : 8;    // Bits 0-7
      unsigned sv32_    : 1;    // Bit  8
      unsigned sv39_    : 1;    // Bit  9
      unsigned sv48_    : 1;    // bit  10
      unsigned sv57_    : 1;    // bit  11
      unsigned resrv0_  : 3;    // bits 12-14
      unsigned svpbmt_  : 1;    // bit  15
      unsigned sv32x4_  : 1;    // bit  16
      unsigned sv39x4_  : 1;    // bit  17
      unsigned sv48x4_  : 1;    // bit  18
      unsigned sv57x4_  : 1;    // bit  19
      unsigned resrv1_  : 1;    // bit  20
      unsigned amoMrif_ : 1;    // bit  21
      unsigned msiFlat_ : 1;    // bit  22
      unsigned msiMrif_ : 1;    // bit  23
      unsigned amoHwad_ : 1;    // bit  24
      unsigned ats_     : 1;    // bit  25
      unsigned t2gpa_   : 1;    // bit  26
      unsigned end_     : 1;    // bit  27
      unsigned igs_     : 2;    // bit  28-29
      unsigned hmp_     : 1;    // bit  30
      unsigned debug_   : 1;    // bit  31
      unsigned pas_     : 6;    // bit  32-37
      unsigned pd8_     : 1;    // bit  38
      unsigned pd17_    : 1;    // bit  39
      unsigned pd20_    : 1;    // bit  40
      unsigned qosid_   : 1;    // bit  41
      unsigned resrv2_  : 14;   // bit  42-55
      unsigned custom_  : 8;    // Bits 56-63
    } bits_;  // Second variant of union
  };


  /// Union to pack/unpack the features control register.
  union
  Fctl
  {
    Fctl(uint32_t value = 0)
      : value_(value)
    { }

    uint32_t value_;  // First variant of union

    struct   // Second variant of union
    {
      unsigned be_       : 1;    // Big Endian
      unsigned wsi_      : 1;    // Wire signaled interrupts
      unsigned gxl_      : 1;    // G stage translation is 32-bits when 1
      unsigned reserved_ : 13;
      unsigned custom_   : 16;
    } bits_;
  };


  union Cqcsr
  {
    Cqcsr(uint32_t value = 0)
      : value_(value)
    { }

    // First varian of union.
    uint32_t value_;

    // Second variant: Bit fields of Cqcsr.
    struct
    {
      unsigned cqen_      : 1;  // Command queue enable.
      unsigned cie_       : 1;  // Command interrupt enable.
      unsigned reserved0_ : 6;
      unsigned cqmf_      : 1;  // Command while storing to queue. No more stores till 0. RW1C.
      unsigned cmd_to_    : 1;  // Timeout
      unsigned cmd_ill_   : 1;  // Illegal
      unsigned fence_w_ip_: 1;  // Completion of IOFENCE.C (for IOMMUs which support wire-signaled-interrupts)
      unsigned reserved1_ : 4;
      unsigned cqon_      : 1;  // Command queue active.
      unsigned busy_      : 1;  // Command queue busy.
      unsigned reserved3_ : 10;
      unsigned custom_    : 4;
    } bits_;
  };


  /// Union to Pack/unpack the fault queue CSR.
  union Fqcsr
  {

    Fqcsr(uint32_t value = 0)
      : value_(value)
    { }

    // First varian of union.
    uint32_t value_;

    // Second variant: Bit fields of Fqcsr.
    struct
    {
      unsigned fqen_      : 1;  // Fault queue enable.
      unsigned fie_       : 1;  // Fault interrupt enable.
      unsigned reserved0_ : 6;
      unsigned fqmf_      : 1;  // Fault while storing to queue. No more stores till 0. RW1C.
      unsigned fqof_      : 1;  // Fault queue full. No more stores till 0. RW1C.
      unsigned reserved1_ : 6;
      unsigned fqon_      : 1;  // Fault queue active.
      unsigned busy_      : 1;  // Fault queue busy.
      unsigned reserved3_ : 10;
      unsigned custom_    : 4;
    } bits_;
  };


  /// Union to Pack/unpack the page-request-queue CSR.
  union Pqcsr
  {

    Pqcsr(uint32_t value = 0)
      : value_(value)
    { }
    uint32_t value_;

    struct
    {
      unsigned pqen_      : 1;  // Page request queue enable.
      unsigned pie_       : 1;  // Page request interrupt enable.
      unsigned reserved0_ : 6;
      unsigned pqmf_      : 1;  // Memory fault while storing to queue. No more stores till 0. RW1C.
      unsigned pqof_      : 1;  // Page request queue full. No more stores till 0. RW1C.
      unsigned reserved1_ : 6;
      unsigned pqon_      : 1;  // Page request queue active.
      unsigned busy_      : 1;  // Page request queue busy.
      unsigned reserved2_ : 10;
      unsigned custom_    : 4;
    } bits_;
  };


  /// Union to Pack/unpack the interrupt pending status register.
  union Ipsr
  {

    Ipsr(uint32_t value = 0)
      : value_(value)
    { }

    // First variant of union.
    uint32_t value_;

    // Second variant: Bit fields of Ipsr.
    struct
    {
      unsigned cip_       : 1;  // Command queue interrupt pending. RW1C.
      unsigned fip_       : 1;  // Fault queue interrupt pending. RW1C.
      unsigned pmip_      : 1;  // Perf monitors interrupt pending. RW1C.
      unsigned pip_       : 1;  // Page request quehe interrupt pending. RW1C.
      unsigned reserved0_ : 4;
      unsigned custom_    : 8;
      unsigned reserved1_ : 16;
    } bits_;
  };


  /// Union to Pack/unpack the queue base CSRs (cqb, fqb, and pqb).
  union Qbase
  {

    Qbase(uint64_t value = 0)
      : value_(value)
    { }

    // First variant of union.
    uint64_t value_;

    // Second variant: Bit fields of Fqb.
    struct
    {
      unsigned logszm1_   : 5;  // Log of size minus 1.
      unsigned reserved0_ : 5;  // Fault queue interrupt pending. RW1C.
      uint64_t ppn_       : 44; // Physical page number of memory buffer used for queue.
      unsigned reserved1_ : 10;
    } bits_;
  };


  /// Access fields of the Icvec CSR.
  union Icvec
  {
    uint64_t value_ = 0;

    struct Bits
    {
      unsigned civ_   : 4;    // Command queue interrupt vector
      unsigned fiv_   : 4;    // Fault queue interrupt vector
      unsigned pmiv_  : 4;    // Performance monitoring interrupt vector
      unsigned piv_   : 4;    // Page request interrupt vector
      unsigned res0_  : 16;
      uint32_t custom;
    } bits_;
  };


  /// Model of an IOMMU constrol and status register.
  class IommuCsr
  {
  public:

    friend class Iommu;

    /// Default constructor: All fields are zero.
    IommuCsr() = default;

    /// Constructor: Define a CSR with the given name, offset, size, reset value and write
    /// mask. Size is the size of the CSR in bytes. Offset is the offset of the CSR in the
    /// memory region associated with the IOMMU: offset zero corresponds to the first CSR.
    IommuCsr(const std::string& name, unsigned offset, unsigned size, uint64_t reset,
             uint64_t mask, uint64_t rw1cMask = 0, uint64_t rw1sMask = 0)
    { define(name, offset, size, reset, mask, rw1cMask, rw1sMask); }

    /// Read current value of CSR.
    uint64_t read() const
    { return value_; }

    /// Write into CSR given value masked by the CSR mask. This honors the RW1C and RW1S
    /// masks.
    void write(uint64_t newVal)
    {
      assert((rw1cMask_ & rw1sMask_) == 0);

      // Where RW1C is 0, effective val is the new val.
      uint64_t eff = (newVal & ~rw1cMask_);

      // Where RW1C is 1 and newVal is 1, the effective val is 0.
      // Where RW1C is 1 and newVal is 0, the effective val is the original.
      eff |= rw1cMask_ & value_ & ~newVal;

      // Wehre RW1S is 0, effective val is the new val.
      eff = eff & ~rw1sMask_;

      // Where RW1S is 1 and newVal is 1, the effective val is 1.
      // Where RW1S is 1 and newVal is 0, the effective val is the original.
      eff |= rw1sMask_ & (newVal | (value_ & ~newVal));

      value_ = (value_ & ~mask_) | (eff & mask_);
    }

    /// Similar to the write method but is not affected by RW1C and RW1S field attributes.
    void poke(uint64_t newVal)
    {
      value_ = (value_ & ~mask_) | (newVal & mask_);
    }

    /// Name of this CSR.
    std::string name() const
    { return name_; }

    /// Number of this CSR.
    CsrNumber number() const
    { return number_; }

    /// Size of this CSR in bytes.
    unsigned size() const
    { return size_; }

    /// Offset of this CSR in the memory mapped region associated with
    /// the IOMMU.
    unsigned offset() const
    { return offset_; }

    /// Write mask of this CSR. Values passed to the write method are anded with this mask
    /// before updating the CSR value.
    uint64_t mask() const
    { return mask_; }

    /// Configure the write mask of this CSR.
    void configureMask(uint64_t mask)
    { mask_ = mask; }

    /// Change the reset value.
    void configureReset(uint64_t value) {
      reset_ = value_ = value;
  }

    /// Reset the CSR: Set current value to the reset value.
    void reset()
    { value_ = reset_; }

  protected:

    void define(const std::string& name, unsigned offset, unsigned size, uint64_t reset,
                uint64_t mask, uint64_t rw1cMask, uint64_t rw1sMask)
    {
      name_ = name;
      offset_ = offset;
      size_ = size;
      reset_ = reset;
      value_ = reset;
      mask_ = mask;
      rw1cMask_ = rw1cMask;
      rw1sMask_ = rw1sMask;
    }

    void setNumber(CsrNumber n)
    { number_ = n; }

  private:

    std::string name_;
    CsrNumber number_ = CsrNumber{0};
    unsigned offset_ = 0;
    unsigned size_ = 0;
    uint64_t reset_ = 0;
    uint64_t mask_ = 0;
    uint64_t rw1cMask_ = 0;   // Where this is 1, writing 1 clears, writing 0 has no effect.
    uint64_t rw1sMask_ = 0;   // Where this is 1, writing 1 sets, writing 0 has no effect.
    uint64_t value_ = 0;
  };

}
