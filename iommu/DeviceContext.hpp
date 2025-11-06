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

#include <functional>
#include <cassert>
#include "ProcessContext.hpp"  // To get IosatpMode

namespace TT_IOMMU
{

  /// Bit fields of a base device id.
  struct BaseDevid
  {
    unsigned ddi0_   : 7;
    unsigned ddi1_   : 9;
    unsigned ddi2_   : 8;
    unsigned unused_ : 8;
  };


  /// Bit fields of an extended device id.
  struct ExtendedDevid
  {
    unsigned ddi0_   : 6;
    unsigned ddi1_   : 9;
    unsigned ddi2_   : 9;
    unsigned unused_ : 8;
  };


  /// Union to pack/unpack device id.
  union Devid
  {
    Devid(uint32_t value)
      : value_(value)
    { }

    unsigned ithDdi(unsigned i, bool extended) const
    {
      assert(i <= 2);
      if (extended)
        {
          if (i == 0)
            return extendedBits_.ddi0_;
          if (i == 1)
            return extendedBits_.ddi1_;
          if (i == 2)
            return extendedBits_.ddi2_;
          return 0;
        }
      if (i == 0)
        return baseBits_.ddi0_;
      if (i == 1)
        return baseBits_.ddi1_;
      if (i == 2)
        return baseBits_.ddi2_;
      return 0;
    }

    uint32_t value_;
    BaseDevid baseBits_;
    ExtendedDevid extendedBits_;
  };


  /// Non leaf device directory tree entry
  union Ddte
  {
    Ddte(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;

    struct
    {
      unsigned v_        : 1;
      unsigned reserved_ : 9;
      uint64_t ppn_      : 44; // Correct width - bits 10-53
      unsigned reserved2_: 10; // Additional reserved bits
    } bits_;
  };


  enum class PdtpMode : uint32_t
    {
      Bare = 0, Pd8 = 1, Pd17 = 2, Pd20 = 3
    };


  /// Process directory table pointer fields.
  struct Pdtp_
  {
    uint64_t ppn_      : 44;
    unsigned reserved_ : 16;
    PdtpMode mode_     : 4;
  };


  union Pdtp
  {
    Pdtp(uint64_t value = 0)
    : value_(value)
    { }

    PdtpMode mode() const
    { return bits_.mode_; }

    uint64_t value_ = 0;
    Pdtp_ bits_;
  };


  enum class MsiptpMode : uint32_t
    {
      Off = 0, Flat = 1
    };


  enum class IohgatpMode : uint32_t
    {
      Bare = 0, Sv32x4 = 1, Sv39x4 = 8, Sv48x4 = 9, Sv57x4 = 10
    };


  /// Union to pack/unpack the Iohgatp register.
  union Iohgatp
  {
    Iohgatp(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_;   // First variant of union

    struct     // Second variant
    {
      uint64_t ppn_     : 44;
      unsigned gcsid_   : 16;
      IohgatpMode mode_ : 4;
    } bits_;
  };


  /// Union to pack/unpack device context translation control
  union TransControl
  {
    TransControl(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_ = 0; // First variant.

    struct Bits  // Second variant.
    {
      unsigned v_       : 1;    // Valid
      unsigned ats_     : 1;    // Enable ATS
      unsigned pri_     : 1;    // Enable page request
      unsigned t2gpa_   : 1;    // Stage 2 translation returns GPA
      unsigned dtf_     : 1;    // Disable translation fault reporting
      unsigned pdtv_    : 1;    // FSC field holds a process directory tree addr
      unsigned prpr_    : 1;    // PRG responce PASID required
      unsigned gade_    : 1;    // G (2nd) stage translation updates A/D bits of PTE
      unsigned sade_    : 1;    // VS (1st) stage translation updates A/D bits of PTE
      unsigned dpe_     : 1;    // Enable use of 0 as default process id
      unsigned sbe_     : 1;    // Implicit 1st stage access is big endian
      unsigned sxl_     : 1;    // True if translating 32-bit addresses (RV32)
      unsigned res0_    : 12;
      unsigned custom_  : 8;
      uint32_t res1_    : 32;
    } bits_;
  };


  /// Union to pack/unpack device context translation attribute.
  union DevTransAttrib
  {
    DevTransAttrib(uint64_t value = 0)
      : value_(value)
    { }

    uint64_t value_ = 0;

    struct Bits
    {
      unsigned res0_     : 12;
      unsigned pscid_    : 20;
      unsigned res1_     : 8;
      unsigned rcid_     : 12;
      unsigned mcid_     : 12;
    } bits_;
  };


  /// This is used to return a copy of the base device context in DeviceContext.
  struct BaseDeviceContext
  {
    uint64_t tc_ = 0;       // Translation control.
    uint64_t iohgatp_ = 0;  // Hypervisor guest address translation.
    uint64_t ta_ = 0;       // Translation attributes.
    uint64_t fsc_ = 0;      // First stage context.
  };


  /// This is used to return a copy of the extended device context in DeviceContext.
  struct ExtendedDeviceContext
  {
    uint64_t tc_ = 0;       // Translation control.
    uint64_t iohgatp_ = 0;  // Hypervisor guest address translation.
    uint64_t ta_ = 0;       // Translation attributes.
    uint64_t fsc_ = 0;      // First stage context.
    uint64_t msiptp_ = 0;   // MSI page table pointer.
    uint64_t msimask_ = 0;  // MSI address mask.
    uint64_t msipat_ = 0;   // MSI address pattern.
    uint64_t reserved_ = 0;
  };


  /// Models both base and extended device contexts. For base, the MSI
  /// fields are all zero.
  class DeviceContext
  {
  public:

    /// Default constructor: Zero all bits.
    DeviceContext() = default;

    /// Constructor for a base device conext.
    DeviceContext(uint64_t transControl, uint64_t iohgatp, uint64_t devTransAttribs,
                  uint64_t firstStage)
      : tc_(transControl), iohgatp_(iohgatp), ta_(devTransAttribs), fsc_(firstStage)
    { }

    /// Constructor for an extended device conext.
    DeviceContext(uint64_t transControl, uint64_t iohgatp, uint64_t devTransAttribs,
                  uint64_t firstStage, uint64_t msiPtp, uint64_t msiMask,
                  uint64_t msiPattern, uint64_t reserved = 0)
      : tc_(transControl), iohgatp_(iohgatp), ta_(devTransAttribs), fsc_(firstStage),
        msiptp_(msiPtp), msimask_(msiMask), msipat_(msiPattern),
        reserved_(reserved)
    { }

    /// Return true if given address matches the MSI address range.
    /// Return false if this is not an extended context.
    bool isMsiAddress(uint64_t gpa) const
    {
      // Extract the upper bits that need to match
      uint64_t shiftedGpa = gpa >> 12;
      uint64_t pattern = msiPattern();
      uint64_t mask = msiMask();

      // The matching criteria should be: (shifted_gpa & ~shifted_mask) == (shifted_pattern & ~shifted_mask)
      return (shiftedGpa & ~mask) == (pattern & ~mask);
    }

    /// Extract the interrupt file number from given shifted address
    /// and MSI mask (see section 2.3.3 of IOMMU spec).
    static uint64_t extractMsiBits(uint64_t addr, uint64_t mask)
    {
      uint64_t res = 0;
      unsigned n = 0;  // Count of extracted bits
      for (unsigned i = 0; i < 64; ++i)
        if ((mask >> i) & 1)
          {
            res = res | (((addr >> i) & 1) << n);
            ++n;
          }
      return res;
    }

    /// Return true if any of the reserved bits in this context are
    /// non zero. Check base fields if extended is false; otherwise,
    /// check all fields.
    bool nonZeroReservedBits(bool extended) const
    {
      if ((tcResMask() & tc_) or (taResMask() & ta_) or (fscResMask() & fsc_))
        return true;

      if (extended)
        if((msiptpResMask() & msiptp_) or (msiAddrResMask() & msimask_) or
           (msiPatternResMask() & msipat_) or reserved_)
          return true;

      return false;
    }

    /// Return true if this context is valid (bit V of TC).
    bool valid() const
    { return TransControl{tc_}.bits_.v_; }

    /// Return true if address translation services are enabled in
    /// this context (bit EN_ATS of TC)
    bool ats() const
    { return TransControl{tc_}.bits_.ats_; }

    /// Return true if page request is enabled in this context (bit
    /// EN_PRI of TC).
    bool pri() const
    { return TransControl{tc_}.bits_.pri_; }

    /// Return true if 2-stage translation should returns guest physical
    /// address (bit T2GPA of TA).
    bool t2gpa() const
    { return TransControl{tc_}.bits_.t2gpa_; }

    /// Return true if translation fault reporting is disabled (bit
    /// DTF of TA).
    bool dtf() const
    { return TransControl{tc_}.bits_.dtf_; }

    /// Return true if the FSC field holds a process directory
    /// tree address (bit PDTV of TA).
    bool pdtv() const
    { return TransControl{tc_}.bits_.pdtv_; }

    bool prpr() const
    { return TransControl{tc_}.bits_.prpr_; }

    /// Return true if the IOMMU second stage translation updates the
    /// A/D bits of the PTE automatically.  Return false if the IOMMU
    /// takes a page fault when A/D bits need to be updated in second
    /// stage.
    bool gade() const
    { return TransControl{tc_}.bits_.gade_; }

    /// Return true if the IOMMU first stage translation updates the
    /// A/D bits of the PTE automatically.  Return false if the IOMMU
    /// takes a page fault when A/D bits need to be updated in first
    /// stage.
    bool sade() const
    { return TransControl{tc_}.bits_.sade_; }

    /// Return true if zero is used as default process id.
    bool dpe() const
    { return TransControl{tc_}.bits_.dpe_; }

    /// Return true if implicit memory access for 1st stage and for process table should
    /// use big endian mode.
    bool sbe() const
    { return TransControl{tc_}.bits_.sbe_; }

    /// Return true if translating 32-bit addresses (RV32).
    bool sxl() const
    { return TransControl{tc_}.bits_.sxl_; }

    /// Return the mode bits of IOHGATP field.
    IohgatpMode iohgatpMode() const
    { return Iohgatp{iohgatp_}.bits_.mode_; }

    /// Guest soft-context id.
    unsigned iohgatpGscid() const
    { return Iohgatp{iohgatp_}.bits_.gcsid_; }

    /// Iohgatp root address translation page number.
    uint64_t iohgatpPpn() const
    { return Iohgatp{iohgatp_}.bits_.ppn_; }

    /// Iohgatp field.
    uint64_t iohgatp() const
    { return iohgatp_; }

    /// Process soft-context id.
    unsigned pscid() const
    { return DevTransAttrib{ta_}.bits_.pscid_; }

    /// First stage address translation mode.
    IosatpMode iosatpMode() const
    { assert(not pdtv()); return Iosatp{fsc_}.bits_.mode_; }

    /// First stage translation root page number.
    uint64_t iosatpPpn() const
    { assert(not pdtv()); return Iosatp{fsc_}.bits_.ppn_; }

    /// Iosatp field.
    uint64_t iosatp() const
    { assert(not pdtv()); return fsc_; }

    /// Process directory tree mode.
    PdtpMode pdtpMode() const
    { assert(pdtv()); return Pdtp{fsc_}.bits_.mode_; }

    /// Process directory tree root page number.
    uint64_t pdtpPpn() const
    { assert(pdtv()); return Pdtp{fsc_}.bits_.ppn_; }

    /// Process directory tree pointer register.
    uint64_t pdtp() const
    { assert(pdtv()); return fsc_; }

    /// MSI translation mode.
    MsiptpMode msiMode() const
    { return MsiptpMode{unsigned(msiptp_ >> 60)}; }

    /// MSI translation root page number.
    uint64_t msiPpn() const
    { return (msiptp_ << 20) >> 20; }

    /// MSI address mask. This zeros out the reserved bits of the mask.
    uint64_t msiMask() const
    { return (msimask_ << 12) >> 12; }

    /// MSI address pattern.  This zeros out the reserved bits of the pattern.
    uint64_t msiPattern() const
    { return (msipat_ << 12) >> 12; }

    /// Return mask of reserved bits in TC field.
    static uint64_t tcResMask()
    { return 0xffff'ffff'00ff'f000; }

    /// Return mask of reserved bits in TA field.
    static uint64_t taResMask()
    { return 0xffff'ffff'0000'0fff; }

    /// Return mask of reserved bits in FSC field.
    static uint64_t fscResMask()
    { return 0x0fff'f000'0000'0000; }

    /// Return mask of reserved bits in msitp field.
    static uint64_t msiptpResMask()
    { return 0x0fff'f000'0000'0000; }

    /// Return mask of reserved bits in msi addr field.
    static uint64_t msiAddrResMask()
    { return 0xfff0'0000'0000'0000; }

    /// Return mask of reserved bits in msi pattern field.
    static uint64_t msiPatternResMask()
    { return 0xfff0'0000'0000'0000; }

    /// Comparison operator. Compare all the fields.
    bool operator==(const DeviceContext& other) const = default;

    /// Return a copy of the base device context part of this object. This is used for
    /// testing.
    BaseDeviceContext basePart() const
    { return BaseDeviceContext{ tc_, iohgatp_, ta_, fsc_ }; }

    /// Return a copy of the extended device context part of this object. This is used for
    /// testing.
    ExtendedDeviceContext extendedPart() const
    { return ExtendedDeviceContext{ tc_, iohgatp_, ta_, fsc_, msiptp_, msimask_, msipat_,
      reserved_}; }

    /// Return the number of levels in the process table pointed to by this device
    /// context. This is valid only if this device context holds a process directory tree
    /// address (pdtv() returns true).
    unsigned processTableLevels() const
    {
      assert(pdtv());

      switch(pdtpMode())
        {
        case PdtpMode::Pd20: return 3;
        case PdtpMode::Pd17: return 2;
        case PdtpMode::Pd8:  return 1;
        default:             return 0;
        }
      return 0;
    }

    /// Return translation control field of this object.
    TransControl transControl() const
    { return tc_; }

    /// Return translation attribute field of this object.
    DevTransAttrib transAttrib() const
    { return ta_; }

    /// Return first stage context field of this object.
    uint64_t firstStageContext() const
    { return fsc_; }

    /// Return the MSIP page table pointer field of this object. The root page number is a
    /// subset of this (use msiPpn to get root page number).
    uint64_t msiTablePointer() const
    { return msiptp_; }

    /// Reutrn the full MSI address mask (does not zero out reserved bits).
    uint64_t fullMsiMask() const
    { return msimask_; }

    /// Reutrn the full MSI pattern (does not zero out reserved bits).
    uint64_t fullMsiPattern() const
    { return msipat_; }

  private:

    uint64_t tc_ = 0;       // Translation control.
    uint64_t iohgatp_ = 0;  // Hypervisor guest address translation.
    uint64_t ta_ = 0;       // Translation attributes.
    uint64_t fsc_ = 0;      // First stage context.
    uint64_t msiptp_ = 0;   // MSI page table pointer.
    uint64_t msimask_ = 0;  // MSI address mask.
    uint64_t msipat_ = 0;   // MSI address pattern.
    uint64_t reserved_ = 0;
  };
}
