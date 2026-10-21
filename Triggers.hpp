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
#include <vector>
#include <array>
#include <string>
#include <bit>
#include "virtual_memory/trapEnums.hpp"

namespace WdRiscv
{

  /// Trigger timing control: Before instruction or after.
  enum class TriggerTiming { Before, After };

  /// Trigger type.
  enum class TriggerType : uint32_t {
    None = 0, Legacy = 1, AddrData = 2, Mcontrol = AddrData, Icount = 3, Itrigger = 4,
    Etrigger = 5, Mcontrol6 = 6, Tmext = 7, Reserved0 = 8, Reserved1 = 9, Reserved2 = 10,
    Reserved3 = 11, Custom0 = 12, Custom1 = 13, Custom2 = 14, Disabled = 15
  };

  /// Trigger action.
  enum class TriggerAction : uint32_t { RaiseBreak, EnterDebug, StartTrace, StopTrace,
		                        EmitTrace, Reserved, External0, External1, Limit = 15 };

  enum class TriggerOffset { Tdata1 = 0, Tdata2 = 1, Tdata3 = 2, Tinfo = 3 };

  /// The hart state that the conditions in tdata3 (textra32/textra64) of a trigger
  /// depend on.
  template <typename URV>
  struct TextraContext
  {
    URV mcontext = 0;
    URV scontext = 0;
    URV asid = 0;       // ASID of vsatp in VS and VU modes, ASID of satp otherwise.
    URV vmid = 0;
    bool hyperEnabled = false;  // Selects the legal mhselect values (legalData3).
  };


  template <typename URV>
  struct Mcontrol;


  template <typename URV>
  struct Mcontrol6;


  /// Bit fields of tinfo trigger register component.
  struct Tinfo
  {
    unsigned info_     : 16;   // Bits 15-0 : Supported types in type field of TDATA1.
    unsigned           : 8;    // Bits 23-16: Reserved -- hardwired to zero.
    unsigned version_  : 8;    // Bits 24-31: Version
  };


  /// Bit fields of mcontrol trigger register component. 32-bit version.
  template <>
  struct Mcontrol<uint32_t>
    {
      unsigned load_    : 1;   // Bit  0    : trigger on load
      unsigned store_   : 1;   // Bit  1    : trigger on store
      unsigned execute_ : 1;   // Bit  2    : trigger on instruction
      unsigned u_       : 1;   // Bit  3    : enable in user mode
      unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
      unsigned          : 1;   // Bit  5    : reserved: hardwired to zero
      unsigned m_       : 1;   // Bit  6    : enable in machine mode
      unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
      unsigned chain_   : 1;   // Bit  11   :
      unsigned action_  : 4;   // Bits 15-12:
      unsigned sizelo_  : 2;   // Bits 17-16:
      unsigned timing_  : 1;   // Bit  18   :
      unsigned select_  : 1;   // Bit  19   :
      unsigned hit_     : 1;   // Bit  20   :
      unsigned maskMax_ : 6;   // Bit  26-21:
      unsigned dmode_   : 1;   // Bit  27   : trigger writable only in debug mode.
      unsigned type_    : 4;   // Bits 31-28:
  } __attribute__((packed));


  /// Bit fields of mcontrol trigger register component. 64-bit version.
  template <>
  struct Mcontrol<uint64_t>
  {
    unsigned load_    : 1;   // Bit  0    : trigger on load
    unsigned store_   : 1;   // Bit  1    : trigger on store
    unsigned execute_ : 1;   // Bit  2    : trigger on instruction
    unsigned u_       : 1;   // Bit  3    : enable in user mode
    unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
    unsigned          : 1;   // Bit  5    : reserved: hardwired to zero
    unsigned m_       : 1;   // Bit  6    : enable in machine mode
    unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
    unsigned chain_   : 1;   // Bit  11   :
    unsigned action_  : 4;   // Bits 15-12:
    unsigned sizelo_  : 2;   // Bits 17-16:
    unsigned timing_  : 1;   // Bit  18   :
    unsigned select_  : 1;   // Bit  19   :
    unsigned hit_     : 1;   // Bit  20   :
    unsigned sizehi_  : 2;   // Bits 22-21:
    unsigned          : 30;  // Bits 52-23:
    unsigned maskMax_ : 6;   // Bits 58-53:
    unsigned dmode_   : 1;   // Bit  59   : trigger writable only in debug mode.
    unsigned type_    : 4;   // Bits 63-60:
  } __attribute__((packed));


  /// Bit fields of mcontrol6 trigger register component. 32-bit version.
  template <>
  struct Mcontrol6<uint32_t>
  {
    unsigned load_    : 1;   // Bit  0    : trigger on load
    unsigned store_   : 1;   // Bit  1    : trigger on store
    unsigned execute_ : 1;   // Bit  2    : trigger on instruction
    unsigned u_       : 1;   // Bit  3    : enable in user mode
    unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
    unsigned unceren_ : 1;   // Bit  5    : uncertain
    unsigned m_       : 1;   // Bit  6    : enable in machine mode
    unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
    unsigned chain_   : 1;   // Bit  11   :
    unsigned action_  : 4;   // Bits 15-12:
    unsigned size_    : 3;   // Bits 18-16:
    unsigned          : 2;   // Bits 20-19: reserved
    unsigned select_  : 1;   // Bit  21   :
    unsigned hit0_    : 1;   // Bit  22   :
    unsigned vu_      : 1;   // Bit  23   :
    unsigned vs_      : 1;   // Bit  24   :
    unsigned hit1_    : 1;   // Bit  25   :
    unsigned uncer_   : 2;   // Bits 26   :
    unsigned dmode_   : 1;   // Bit  27   : trigger writable only in debug mode.
    unsigned type_    : 4;   // Bits 31-28:
  } __attribute__((packed));


  /// Bit fields of mcontrol6 trigger register component. 64-bit version.
  template <>
  struct Mcontrol6<uint64_t>
  {
    unsigned load_    : 1;   // Bit  0    : trigger on load
    unsigned store_   : 1;   // Bit  1    : trigger on store
    unsigned execute_ : 1;   // Bit  2    : trigger on instruction
    unsigned u_       : 1;   // Bit  3    : enable in user mode
    unsigned s_       : 1;   // Bit  4    : enable in supervisor mode
    unsigned unceren_ : 1;   // Bit  5    : uncertain
    unsigned m_       : 1;   // Bit  6    : enable in machine mode
    unsigned match_   : 4;   // Bits 10-7 : controls what is considered to be a match
    unsigned chain_   : 1;   // Bit  11   :
    unsigned action_  : 4;   // Bits 15-12:
    unsigned size_    : 3;   // Bits 18-16:
    unsigned          : 2;   // Bits 20-19: reserved
    unsigned select_  : 1;   // Bit  21   :
    unsigned hit0_    : 1;   // Bit  22   :
    unsigned vu_      : 1;   // Bit  23   :
    unsigned vs_      : 1;   // Bit  24   :
    unsigned hit1_    : 1;   // Bit  25   :
    unsigned uncer_   : 2;   // Bits 26   :
    unsigned          : 32;  // Bits 58-27:
    unsigned dmode_   : 1;   // Bit  59   : trigger writable only in debug mode.
    unsigned type_    : 4;   // Bits 63-60:
  } __attribute__((packed));


  /// Bit fields for Icount trigger register component.
  template <typename URV>
  struct Icount
  {
    unsigned action_  : 6;
    unsigned u_       : 1;
    unsigned s_       : 1;
    unsigned pending_ : 1;
    unsigned m_       : 1;
    unsigned count_   : 14;
    unsigned hit_     : 1;
    unsigned vu_      : 1;
    unsigned vs_      : 1;
    URV               : 8*sizeof(URV) - 32;  // Zero bits
    unsigned dmode_   : 1;   // Trigger writable only in debug mode.
    unsigned type_    : 4;
  } __attribute__((packed));


  /// Bif tields for Etrigger trigger register component.
  template <typename URV>
  struct Etrigger
  {
    unsigned action_  : 6;
    unsigned u_       : 1;
    unsigned s_       : 1;
    unsigned          : 1;                  // Reserved -- zero.
    unsigned m_       : 1;
    unsigned          : 1;                  // Reserved -- zero.
    unsigned vu_      : 1;
    unsigned vs_      : 1;
    uint64_t          : 8*sizeof(URV) - 19; // Reserved -- zero.
    unsigned hit_     : 1;
    unsigned dmode_   : 1;
    unsigned type_    : 4;
  } __attribute((packed));


  /// Bif tields for Itrigger trigger register component.
  template <typename URV>
  struct Itrigger
  {
    unsigned action_  : 6;
    unsigned u_       : 1;
    unsigned s_       : 1;
    unsigned          : 1;                  // Reserved -- zero.
    unsigned m_       : 1;
    unsigned nmi_     : 1;
    unsigned vu_      : 1;
    unsigned vs_      : 1;
    uint64_t          : sizeof(URV)*8 - 19; // Reserved -- zero.
    unsigned hit_     : 1;
    unsigned dmode_   : 1;
    unsigned type_    : 4;
  } __attribute((packed));


  /// Union to pack/unpack TINFO trigger register value
  union TinfoBits
  {
    TinfoBits(uint64_t value) :
      value_(value)
    { }

    unsigned info() const      { return tinfo_.info_; }
    unsigned version() const   { return tinfo_.version_; }

    uint64_t value_ = 0;
    Tinfo tinfo_;
  };


  /// Union to pack/unpack TDATA1 trigger register value
  template <typename URV>
  union Data1Bits
  {
    Data1Bits(URV value) :
      value_(value)
    { }

    /// Return trigger type field of Tdata1.
    TriggerType type() const { return TriggerType(mcontrol_.type_); }

    /// Return the dmode (debug mode only) field of Tdata1.
    bool dmode() const   { return mcontrol_.dmode_; }

    /// Return the action field of Tdata1. The field location depends on the type.
    TriggerAction action() const
    {
      if (isAddrData())
        return TriggerAction(mcontrol_.action_);
      return TriggerAction(icount_.action_);
    }

    /// Return true if type is None or Disabled.
    bool isDisabled() const
    { return type() == TriggerType::None or type() == TriggerType::Disabled; }

    bool isMcontrol()  const { return type() == TriggerType::Mcontrol; }
    bool isMcontrol6() const { return type() == TriggerType::Mcontrol6; }
    bool isAddrData()  const { return isMcontrol() or isMcontrol6(); }
    bool isInstCount() const { return type() == TriggerType::Icount; }
    bool isEtrigger()  const { return type() == TriggerType::Etrigger; }
    bool isItrigger()  const { return type() == TriggerType::Itrigger; }
    bool isTmext()     const { return type() == TriggerType::Tmext; }

    /// Return true if the type has an action field.
    bool hasAction() const
    { return isAddrData() or isInstCount() or isEtrigger() or isItrigger() or isTmext(); }

    /// Configuration dmode==0 and action==1 (enter debug) is not allowed.
    void legalizeAction()
    {
      if (hasAction() and not dmode() and action() == TriggerAction::EnterDebug)
        setAction(TriggerAction::RaiseBreak);
    }

    /// Set the type field of tdata1.
    void setType(TriggerType type)
    { mcontrol_.type_ = unsigned(type); }

    /// Set the dmode field of tdata1.
    void setDmode(bool flag)
    { mcontrol_.dmode_ = flag; }

    /// Set the action field of tdata1.
    void setAction(TriggerAction action)
    {
      if (isAddrData())
        mcontrol_.action_ = unsigned(action);
      else
        icount_.action_ = unsigned(action);
    }

    template <typename T>
    const T& mcontrol() const
    {
      if constexpr (std::is_same_v<T, decltype(mcontrol_)>)
        return mcontrol_;
      else
        return mcontrol6_;
    }

    URV value_ = 0;
    Mcontrol<URV>  mcontrol_;
    Mcontrol6<URV> mcontrol6_;
    Icount<URV> icount_;
    Etrigger<URV> etrigger_;
    Itrigger<URV> itrigger_;
  };


  template <typename URV>
  class Triggers;

  /// Model a RISCV trigger.
  template <typename URV>
  class Trigger
  {
  public:

    friend class Triggers<URV>;

    enum class Select { MatchAddress, MatchData };

    enum class Chain { No, Yes };

    enum class Match { Equal = 0, Masked = 1, GE = 2, LT = 3, MaskHighEqualLow = 4,
		       MaskLowEqualHigh = 5, NotEqual = 8, NotMasked = 9,
                       NotMaskHighEqualLow = 12, NotMaskLowEqualHigh = 13 };

    /// Constructor.
    Trigger(URV data1 = 0, URV data2 = 0, URV /*data3*/ = 0,
	    URV mask1 = ~URV(0), URV mask2 = ~URV(0), URV mask3 = 0);

    /// Return the type of this trigger.
    TriggerType type() const
    { return data1_.type(); }

    /// Read the tdata1 register of the trigger. This is typically the control register of
    /// the trigger. A CSR instruction that reads TDATA1 may trip a trigger and that will
    /// modify data1. If that happens, we return the valye of TDATA1 before the tripping.
    URV readData1() const
    { return modifiedT1_? prevData1_ : data1_.value_; }

    /// Similar to readData1 except for always returning the final value of TDATA1.
    URV peekData1() const
    { return data1_.value_; }

    /// Read the tdata2 register of the trigger. This is typically the
    /// target value of the trigger.
    URV readData2() const
    { return data2_; }

    /// Read the tdata3 register of the trigger.
    URV readData3(bool hyperEnabled) const
    { return legalData3(data3_, hyperEnabled); }

    /// Read the tinfo register ot the trigger.
    URV readInfo() const
    { return info_.value_; }

    /// Return true if this trigger supports the given type: its bit is set in tinfo. A
    /// trigger configured without tinfo (info 0) supports only the type of its tdata1
    /// reset value: without tinfo, the debug spec has a debugger read the one supported
    /// type from tdata1.
    bool supportsType(TriggerType type) const
    {
      unsigned info = info_.info();
      if (info == 0)
        return type == Data1Bits<URV>(data1Reset_).type();
      return (info >> unsigned(type)) & 1;
    }

    /// Return the value that writeData1(debugMode, val) leaves in tdata1, without
    /// writing it.
    URV writtenData1(bool debugMode, URV val) const
    {
      if (isDebugModeOnly() and not debugMode)
	return data1_.value_;
      URV mask = data1WriteMask_;
      if (not debugMode)  // dmode bit writable only in debug mode
	mask &= ~(URV(1) << (8*sizeof(URV) - 5));

      // Writing 0 (None) into type is changed to 15 (Disabled). Section 5.7.2 of spec.
      Data1Bits<URV> valBits{val};
      if (valBits.type() == TriggerType::None)
	valBits.setType(TriggerType::Disabled);
      val = valBits.value_;

      Data1Bits<URV> next{(val & mask) | (data1_.value_ & ~mask)};

      next.legalizeAction();
      if (next.isAddrData())
	{
          if (isReservedMatch(Match(next.mcontrol_.match_)))
            next.mcontrol_.match_ = unsigned(Match::Equal);
          if (next.isMcontrol())
            next.mcontrol_.maskMax_ = napotMaskMax_;
        }

      return next.value_;
    }

    /// Write the tdata1 register of the trigger. This is the interface for CSR
    /// instructions.
    void writeData1(bool debugMode, URV val)
    {
      if (isDebugModeOnly() and not debugMode)
	return;

      if (not modifiedT1_)
        prevData1_ = data1_.value_;

      data1_.value_ = writtenData1(debugMode, val);
      modifiedT1_ = true;
    }

    /// Write the tdata2 register of the trigger. This is the interface for CSR
    /// instructions.
    void writeData2(bool debugMode, URV value)
    {
      if (isDebugModeOnly() and not debugMode)
	return;

      data2_ = (value & data2WriteMask_) | (data2_ & ~data2WriteMask_);
      modifiedT2_ = true;

      legalizeNapotData2();
      updateCompareMask();
    }

    /// Write the tdata3 register of the trigger. This is the interface for CSR
    /// instructions.
    void writeData3(bool debugMode, URV value, bool hyperEnabled)
    {
      if (isDebugModeOnly() and not debugMode)
	return;

      data3_ = legalData3((value & data3WriteMask_) | (data3_ & ~data3WriteMask_), hyperEnabled);
      modifiedT3_ = true;
    }

    /// Write the tinfo register of the trigger. This is the interface for CSR
    /// instructions.
    void writeInfo(bool debugMode, URV value)
    {
      if (isDebugModeOnly() and not debugMode)
	return;

      info_.value_ = (value & infoWriteMask_) | (info_.value_ & ~infoWriteMask_);
      modifiedInfo_ = true;
    }

    /// Return the value that pokeData1(x) leaves in tdata1, without writing it.
    URV pokedData1(URV x) const
    {
      Data1Bits<URV> next{(x & data1PokeMask_) | (data1_.value_ & ~data1PokeMask_)};
      next.legalizeAction();
      return next.value_;
    }

    /// Poke tdata1. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeData1(URV x)
    { data1_.value_ = pokedData1(x); }

    /// Poke tdata2. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeData2(URV x)
    {
      data2_ = (x & data2PokeMask_) | (data2_ & ~data2PokeMask_);

      legalizeNapotData2();
      updateCompareMask();
    }

    /// Poke tdata3. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeData3(URV x, bool hyperEnabled)
    { data3_ = legalData3((x & data3PokeMask_) | (data3_ & ~data3PokeMask_), hyperEnabled); }

    /// Return the given tdata3 (textra32/textra64) value with its WARL select fields made
    /// legal: 0 in place of a reserved mhselect (3 or 7) or sselect (3), and in place of
    /// an mhselect other than 0 and 4 while hyperEnabled is false (hypervisor disabled).
    static URV legalData3(URV value, bool hyperEnabled)
    {
      constexpr unsigned mhShift = sizeof(URV) == 8 ? 48 : 23;
      unsigned mhselect = unsigned((value >> mhShift) & URV(0x7));
      bool mhLegal = hyperEnabled ? (mhselect & 3) != 3 : (mhselect & 3) == 0;
      if (not mhLegal)
        value &= ~(URV(0x7) << mhShift);

      if ((value & URV(0x3)) == 3)
        value &= ~URV(0x3);
      return value;
    }

    /// Poke tinfo. This allows writing of modifiable bits that are read-only to the CSR
    /// instructions.
    void pokeInfo(URV x)
    { info_.value_ = (x & infoPokeMask_) | (info_.value_ & ~infoPokeMask_); }

    void configData1(URV reset, URV mask, URV pokeMask)
    { data1Reset_ = reset; data1_.value_ = reset; data1WriteMask_ = mask; data1PokeMask_ = pokeMask;}

    void configData2(URV reset, URV mask, URV pokeMask)
    { data2Reset_ = reset; data2_ = reset; data2WriteMask_ = mask; data2PokeMask_ = pokeMask;}

    void configData3(URV reset, URV mask, URV pokeMask)
    { data3Reset_ = reset; data3_ = reset; data3WriteMask_ = mask; data3PokeMask_ = pokeMask;}

    void configInfo(URV reset, URV mask, URV pokeMask)
    { infoReset_ = reset; info_ = reset; infoWriteMask_ = mask; infoPokeMask_ = pokeMask;}

    /// Reset trigger.
    void reset(bool hyperEnabled)
    {
      data1_.value_ = data1Reset_; data2_ = data2Reset_;
      data3_ = legalData3(data3Reset_, hyperEnabled);
      if (data1_.isMcontrol())
        data1_.mcontrol_.maskMax_ = napotMaskMax_;
      writeData2(true, data2Reset_); // Define compare mask
    }

    /// Return true if this trigger is enabled.
    bool isEnabled() const
    {
      if (data1_.isAddrData())
        {
          if (data1_.isMcontrol())
            {
              auto& ctl = data1_.mcontrol_;
              return ctl.m_ or ctl.s_ or ctl.u_;
            }
          auto& ctl6 = data1_.mcontrol6_;
          return ctl6.m_ or ctl6.s_ or ctl6.u_ or ctl6.vs_ or ctl6.vu_;
        }

      if (data1_.isInstCount())
        {
          auto& ctl = data1_.icount_;
          return ctl.m_ or ctl.s_ or ctl.u_ or ctl.vs_ or ctl.vu_;
        }

      if (data1_.isItrigger())
        {
          auto& ctl = data1_.itrigger_;
          return ctl.m_ or ctl.s_ or ctl.u_ or ctl.vs_ or ctl.vu_ or ctl.nmi_;
        }

      if (data1_.isEtrigger())
        {
          auto& ctl = data1_.etrigger_;
          return ctl.m_ or ctl.s_ or ctl.u_ or ctl.vs_ or ctl.vu_;
        }

      return false;
    }

    /// Return true if trigger is writable only in debug mode.
    bool isDebugModeOnly() const
    { return data1_.dmode(); }

    /// Return true if this is an instruction (execute) trigger.
    bool isInst() const
    { return data1_.isAddrData() and data1_.mcontrol_.execute_; }

    /// Return true if this trigger will cause the processor to enter debug mode on a hit.
    bool isEnterDebugOnHit() const
    { return getAction() == TriggerAction::EnterDebug; }

    /// Return true if this trigger is enabled for loads (if isLoad is true) or for stores
    /// (if isStore is true), for addresses, for the given timing, if its tdata3 conditions
    /// hold on the given context and if it matches the given data address.  Return false
    /// otherwise.
    bool matchLdStAddr(URV address, unsigned size, TriggerTiming timing, bool isLoad,
                       bool isStore, PrivilegeMode mode, bool virtMode,
                       const TextraContext<URV>& context, URV& hitAddr) const;

    /// Return true if this trigger is enabled for loads (or stores if
    /// isLoad is false), for data, for the given timing, if its tdata3
    /// conditions hold on the given context and if it matches the given
    /// value address.  Return false otherwise.
    bool matchLdStData(URV value, TriggerTiming timing, bool isLoad,
                       PrivilegeMode mode, bool virtMode,
                       const TextraContext<URV>& context) const;

    /// Return true if this trigger is enabled for instruction
    /// addresses (execution), for the given timing, if its tdata3
    /// conditions hold on the given context and if it matches the given
    /// address. Return false otherwise.
    bool matchInstAddr(URV address, unsigned size, TriggerTiming timing,
                       PrivilegeMode mode, bool virtMode,
                       const TextraContext<URV>& context, URV& hitAddr) const;

    /// Return true if this trigger is enabled for instruction opcodes
    /// (execution), for the given timing, if its tdata3 conditions hold
    /// on the given context and if it matches the given opcode.  Return
    /// false otherwise.
    bool matchInstOpcode(URV opcode, TriggerTiming timing,
                         PrivilegeMode mode, bool virtMode,
                         const TextraContext<URV>& context) const;

    /// Return true if the given hart state satisfies the mhselect and sselect conditions
    /// of tdata3: textra64 when URV is 64 bits, textra32 when it is 32 bits.
    bool matchTextra(const TextraContext<URV>& context) const
    {
      constexpr bool rv64 = sizeof(URV) == 8;
      constexpr unsigned mhselectShift = rv64 ? 48 : 23;
      constexpr unsigned mhvalueShift = rv64 ? 51 : 26;
      constexpr URV mhvalueMask = rv64 ? 0x1fff : 0x3f;
      constexpr URV mhvalueExtMask = (mhvalueMask << 1) | 1;
      constexpr unsigned sbytemaskShift = rv64 ? 36 : 18;
      constexpr unsigned svalueBytes = rv64 ? 4 : 2;
      constexpr URV svalueMask = rv64 ? 0xffffffff : 0xffff;
      constexpr URV asidMask = rv64 ? 0xffff : 0x1ff;       // ASIDMAX bits.

      // The conditions are those of the value a CSR read of tdata3 returns.
      URV data3 = readData3(context.hyperEnabled);

      unsigned mhselect = unsigned((data3 >> mhselectShift) & 7);
      URV mhvalue = (data3 >> mhvalueShift) & mhvalueMask;
      // {mhvalue, mhselect[2]}, as wide as VMIDMAX.
      URV mhvalueExt = (mhvalue << 1) | (mhselect >> 2);

      switch (mhselect)
        {
        case 4:  // mcontext
          if ((context.mcontext & mhvalueMask) != mhvalue)
            return false;
          break;

        case 1:  // mcontext_select
        case 5:
          if ((context.mcontext & mhvalueExtMask) != mhvalueExt)
            return false;
          break;

        case 2:  // vmid_select
        case 6:
          if (context.vmid != mhvalueExt)
            return false;
          break;

        default:  // 0: ignore.
          break;
        }

      URV svalue = (data3 >> 2) & svalueMask;

      switch (unsigned(data3 & 3))
        {
        case 1:  // scontext: a byte whose sbytemask bit is set is not compared.
          {
            URV mask = svalueMask;
            for (unsigned i = 0; i < svalueBytes; ++i)
              if ((data3 >> (sbytemaskShift + i)) & 1)
                mask &= ~(URV(0xff) << (8*i));
            return ((context.scontext ^ svalue) & mask) == 0;
          }

        case 2:  // asid
          return context.asid == (svalue & asidMask);

        default:  // 0: ignore.
          return true;
        }
    }

    /// Return true if this trigger is enabled for given mode and its
    /// tdata3 conditions hold on the given context. Return false
    /// otherwise. This is called for both instruction retire and trap
    /// scenarios.

    bool matchInstCount(PrivilegeMode mode, bool virtMode, const TextraContext<URV>& context)
    {
      if (not data1_.isInstCount())
	return false;  // Not an icount trigger.
      return matchModeAndTextra(data1_.icount_, mode, virtMode, context);
    }

    /// If this trigger is enabled and is of type icount, then make it
    /// count down returning true if its value becomes zero. Return
    /// false otherwise.
    bool instCountdown()
    {
      Icount<URV>& icount = data1_.icount_;

      if (icount.count_)
        {
          icount.count_ = icount.count_ - 1;
          icount.pending_ = not icount.count_;
        }
      return icount.pending_;
    }

    /// Perform a match on the given item (maybe an address or a value) and the data2
    /// component of this trigger according to the match variable.
    bool doMatch(URV item, Match match) const;

    /// Set the hit bit of this trigger. For a chained trigger, this is to be called only
    /// if all the triggers in the chain have tripped.
    void setHit(bool flag)
    {
      if (data1_.isAddrData())
	{
          if (not modifiedT1_)
            prevData1_ = data1_.value_;
          if (data1_.isMcontrol())
            data1_.mcontrol_.hit_ = flag;
          else
            data1_.mcontrol6_.hit0_ = flag; // only implement before
	  modifiedT1_ = true;
	}
      if (data1_.isInstCount())
	{
          if (not modifiedT1_)
            prevData1_ = data1_.value_;
	  data1_.icount_.hit_ = flag;
	  modifiedT1_ = true;
	}
      if (data1_.isItrigger())
        {
          if (not modifiedT1_)
            prevData1_ = data1_.value_;
          data1_.itrigger_.hit_ = flag;
          modifiedT1_ = true;
        }
      if (data1_.isEtrigger())
        {
          if (not modifiedT1_)
            prevData1_ = data1_.value_;
          data1_.etrigger_.hit_ = flag;
          modifiedT1_ = true;
        }
    }

    /// Return the hit bit of this trigger.
    bool getHit() const
    {
      if (data1_.isAddrData())
        return data1_.isMcontrol()? data1_.mcontrol_.hit_ : data1_.mcontrol6_.hit0_;
      if (data1_.isInstCount())
	return data1_.icount_.hit_;
      if (data1_.isItrigger())
        return data1_.itrigger_.hit_;
      if (data1_.isEtrigger())
        return data1_.etrigger_.hit_;
      return false;
    }

    /// Return the chain bit of this trigger or false if this trigger has
    /// no chain bit.
    bool getChain() const
    {
      if (data1_.isAddrData())
	return data1_.mcontrol_.chain_;
      return false;
    }

    /// Return the timing of this trigger.
    TriggerTiming getTiming() const
    {
      if (data1_.isAddrData())
	return data1_.isMcontrol()? TriggerTiming(data1_.mcontrol_.timing_) : TriggerTiming::Before;
      return TriggerTiming::After;  // icount has "after" timing.
    }

    /// Return true if the chain of this trigger has tripped.
    bool hasTripped() const
    { return chainHit_; }

    /// Mark this trigger as tripped.
    void setTripped(bool flag)
    { chainHit_ = flag; }

    /// Return the action fields of the trigger.
    TriggerAction getAction() const
    {
      if (data1_.hasAction())
	return data1_.action();
      return TriggerAction::RaiseBreak;
    }

    /// Enable/disable all ld/st address matching [address, address+size-1].
    void enableAllDataAddrMatch(bool flag)
    { matchAllDataAddr_ = flag ? ~uint32_t(0) : 0; }

    /// Enable/disable all inst address matching [address, address+size-1].
    void enableAllInstrAddrMatch(bool flag)
    { matchAllInstrAddr_ = flag ? uint32_t(0) : 0; }

    /// Enable/disable all ld/st address maching for given match type.
    void enableAllDataAddrMatch(unsigned matchType, bool flag)
    {
      uint32_t mask = uint32_t(1) << matchType;
      if (flag)
        matchAllDataAddr_ |= mask;
      else
        matchAllDataAddr_ &= ~mask;
    }

    /// Enable/disable all instr address (fetch) maching for given match type.
    void enableAllInstrAddrMatch(unsigned matchType, bool flag)
    {
      uint32_t mask = uint32_t(1) << matchType;
      if (flag)
        matchAllInstrAddr_ |= mask;
      else
        matchAllInstrAddr_ &= ~mask;
    }

    /// Config maskmax: the largest NAPOT range is 2^maskMax bytes.
    void configNapotMaskMax(unsigned maskMax)
    { napotMaskMax_ = maskMax; }

  protected:

    /// Return true if the given match type is negated: NotEqual, NotMasked, ...
    static bool isNegatedMatch(Match m)
    {
      return m == Match::NotEqual or m == Match::NotMasked or
             m == Match::NotMaskHighEqualLow or m == Match::NotMaskLowEqualHigh;
    }

    /// Return true if the given match type is reserved (6, 7, 10, 11, 14 or 15).
    static bool isReservedMatch(Match m)
    { return m > Match::MaskLowEqualHigh and not isNegatedMatch(m); }

    /// Return the non-negated match corresponding to the given negated match m. For
    /// example, if m is NotEqual, this method returns Equal.
    static Match negateNegatedMatch(Match m)
    { assert(isNegatedMatch(m)); return Match(unsigned(m) - unsigned(Match::NotEqual)); }

    /// For an mcontrol6 trigger with match 1 (NAPOT) or 9 (not NAPOT), make tdata2 a
    /// legal NAPOT value: if bits maskmax6-1:0 are all ones, clear bit maskmax6-1.
    void legalizeNapotData2()
    {
      if (not data1_.isMcontrol6())
        return;
      auto match = Match(data1_.mcontrol6_.match_);
      if (match != Match::Masked and match != Match::NotMasked)
        return;
      URV topBit = URV(1) << (napotMaskMax_ - 1);
      URV lowMask = topBit | (topBit - 1);
      if ((data2_ & lowMask) == lowMask)
        data2_ &= ~topBit;
    }

    void updateCompareMask()
    {
      // Pre-compute mask for a NAPOT compare.
      data2CompareMask_ = ~URV(0);
      unsigned leastSigZeroBit = std::countr_one(data2_); // Index of least sig zero bit
      if (leastSigZeroBit >= 8*sizeof(URV) - 1)
        data2CompareMask_ = 0;
      else
        data2CompareMask_ = data2CompareMask_ << (leastSigZeroBit + 1);
    }

    bool isModified() const
    { return modifiedT1_ or modifiedT2_ or modifiedT3_ or modifiedInfo_ or modifiedControl_; }

    void clearModified()
    { modifiedT1_ = modifiedT2_ = modifiedT3_ = modifiedInfo_ = modifiedControl_ = false; }

    bool getLocalHit() const
    { return localHit_; }

    void setLocalHit(bool flag)
    { localHit_ = flag; }

    void setChainHit(bool flag)
    { chainHit_ = flag; }

    void setChainBounds(size_t begin, size_t end)
    {
      chainBegin_ = begin;
      chainEnd_ = end;
    }

    void getChainBounds(size_t& begin, size_t& end) const
    {
      begin = chainBegin_;
      end = chainEnd_;
    }

    bool peek(uint64_t& data1, uint64_t& data2, uint64_t& data3) const
    {
      data1 = data1_.value_; data2 = data2_; data3 = data3_;
      return true;
    }

    bool peek(uint64_t& data1, uint64_t& data2, uint64_t& data3,
	      uint64_t& wm1, uint64_t& wm2, uint64_t& wm3,
	      uint64_t& pm1, uint64_t& pm2, uint64_t& pm3) const
    {
      bool ok = peek(data1, data2, data3);
      wm1 = data1WriteMask_; wm2 = data2WriteMask_; wm3 = data3WriteMask_;
      pm1 = data1PokeMask_; pm2 = data2PokeMask_; pm3 = data3PokeMask_;
      return ok;
    }

    /// Return true if the control ctl (the mcontrol, mcontrol6, icount, itrigger or
    /// etrigger view of tdata1) enables this trigger in the given mode and the tdata3
    /// conditions hold on the given context.
    template <typename C>
    bool matchModeAndTextra(const C& ctl, PrivilegeMode mode, bool virtMode,
                            const TextraContext<URV>& context) const;

    // Helper to public matchLdStAddr.
    template <typename M>
    bool matchLdStAddr(URV address, unsigned size, TriggerTiming timing, bool isLoad,
                       bool isStore, PrivilegeMode mode, bool virtMode,
                       const TextraContext<URV>& context, URV& hitAddr) const;

    template <typename M>
    bool matchLdStData(URV value, TriggerTiming timing, bool isLoad,
                       PrivilegeMode mode, bool virtMode,
                       const TextraContext<URV>& context) const;

    template <typename M>
    bool matchInstAddr(URV address, unsigned size, TriggerTiming timing,
                       PrivilegeMode mode, bool virtMode,
                       const TextraContext<URV>& context, URV& hitAddr) const;

    template <typename M>
    bool matchInstOpcode(URV opcode, TriggerTiming timing,
                         PrivilegeMode mode, bool virtMode,
                         const TextraContext<URV>& context) const;

    /// Return true if given match type compares against all data addresses of an
    /// instruction (2 address of lh, 4 for lw, ...), return false otherwise indicating
    /// that the match type compares against the smallest data address of an instruction.
    bool matchAllDataAddresses(Match match) const
    { return (matchAllDataAddr_ >> unsigned(match)) & 1; }
      
    /// Same as above for instruction (fetch) addresses.
    bool matchAllInstrAddresses(Match match) const
    { return matchAllInstrAddr_ >> unsigned(match) & 1; }

    /// Perform a match on the size addresses starting at addr. On a hit, set hitAddr to
    /// the lowest address that matches, or to addr if the match is negated.
    bool doMatchAllAddresses(URV addr, unsigned size, Match match, URV& hitAddr) const;


  private:

    Data1Bits<URV> data1_ = Data1Bits<URV>(0);
    URV data2_            = 0;
    URV data3_            = 0;
    TinfoBits info_       = TinfoBits(0);

    URV data1Reset_   = 0;
    URV data2Reset_   = 0;
    URV data3Reset_   = 0;
    URV infoReset_    = 0;

    URV data1WriteMask_   = ~URV(0);
    URV data2WriteMask_   = ~URV(0);
    URV data3WriteMask_   = 0;
    URV infoWriteMask_    = 0;              // Fields of tinfo are read-only.

    URV data1PokeMask_   = ~URV(0);
    URV data2PokeMask_   = ~URV(0);
    URV data3PokeMask_   = 0;
    URV infoPokeMask_    = ~URV(0);

    URV data2CompareMask_ = ~URV(0);
    unsigned napotMaskMax_ = 8*sizeof(URV) - 1;

    URV prevData1_ = 0;

    bool localHit_ = false;   // Trigger tripped in isolation.
    bool chainHit_ = false;   // All entries in chain tripped.
    bool modifiedT1_ = false;
    bool modifiedT2_ = false;
    bool modifiedT3_ = false;
    bool modifiedInfo_ = false;
    bool modifiedControl_ = false;

    size_t chainBegin_ = 0, chainEnd_ = 0;

    // One bit for each match type (Match). If set for a match type, then the match will
    // consider all the data addresses of an instruction; otherwise, it will only consider
    // the first address. By default all match types match all data addresses.
    uint32_t matchAllDataAddr_ = ~uint32_t(0);
    uint32_t matchAllInstrAddr_ = ~uint32_t(0); // Same as above but for instruction fetch.
  };


  template <typename URV>
  class Triggers
  {
  public:

    Triggers(unsigned count = 0);

    /// Return the number of defined triggers.
    size_t size() const
    { return triggers_.size(); }

    /// Set value to the tdata1 register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds.
    bool readData1(URV trigger, URV& value) const;

    /// Similar to readData1 except for always returning the final value of TDATA1.
    /// See Trigger::peekData1.
    bool peekData1(URV trigger, URV& value) const;

    /// Set value to the tdata2 register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds or if data2 is not
    /// implemented.
    bool readData2(URV trigger, URV& value) const;

    /// Set value to the tdata3 register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds of if data3 is not
    /// implemented.
    bool readData3(URV trigger, URV& value) const;

    /// Set value to the tinfo register of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds of if tinfo is not
    /// implemented.
    bool readInfo(URV trigger, URV& value) const;

    /// Set value to the tcontroinfo of the given trigger. Return true on success and
    /// false (leaving value unmodified) if trigger is out of bounds of if tcontrol is not
    /// implemented.
    bool readControl(URV trigger, URV& value) const;

    /// Set the tdata1 register of the given trigger to the given value. Return true on
    /// success and false if trigger is out of bounds.
    bool writeData1(URV trigger, bool debugMode, URV value);

    /// Set the tdata2 register of the given trigger to the given value. Return true on
    /// success and false if trigger is out of bounds.
    bool writeData2(URV trigger, bool debugMode, URV value);

    /// Set the tdata3 register of the given trigger to the given value. Return true on
    /// success and false if trigger is out of bounds.
    bool writeData3(URV trigger, bool debugMode, URV value);

    /// Set the tinfo register of the given trigger to the given value. Return true on
    /// success and false if trigger is out of bounds.
    bool writeInfo(URV trigger, bool debugMode, URV value);

    /// Set the tcontrol register of the given trigger to the given value. Return true on
    /// success and false (leaving value unmodified) if trigger is out of bounds or if
    /// tcontrol is not implemented.
    bool writeControl(URV trigger, bool debugMode, URV value);

    /// Return true if given trigger is enabled. Return false if trigger is not enabled or
    /// if it is out of bounds.
    bool isEnabled(URV trigger) const
    {
      if (trigger >= triggers_.size())
	return false;
      return triggers_.at(trigger).isEnabled();
    }

    /// Return true if one or more triggers are enabled.
    bool hasActiveTrigger() const
    {
      for (const auto& trigger : triggers_)
	if (trigger.isEnabled())
	  return true;
      return false;
    }

    /// Return true if one or more instruction (execute) triggers are
    /// enabled.
    bool hasActiveInstTrigger() const
    {
      for (const auto& trigger : triggers_)
	if (trigger.isEnabled() and trigger.isInst())
	  return true;
      return false;
    }

    /// Return true if any of the load (if isLoad) or store (if isStore) triggers trips. A
    /// load/store trigger trips if it matches the given address and timing and if all the
    /// remaining triggers in its chain have tripped. Set the local-hit bit of any
    /// load/store trigger that matches. If the trigger action is contingent on interrupts
    /// being enabled (ie == true), then the trigger will not trip even if its condition
    /// is satisfied.
    bool ldStAddrTriggerHit(URV address, unsigned size, TriggerTiming, bool isLoad,
                            bool isStore, PrivilegeMode mode, bool virtMode, bool ie,
                            const TextraContext<URV>& context, URV& hitAddr);

    /// Similar to ldStAddrTriggerHit but for data match.
    bool ldStDataTriggerHit(URV value, TriggerTiming, bool isLoad,
                            PrivilegeMode mode, bool virtMode, bool ie,
                            const TextraContext<URV>& context);

    /// Similar to ldStAddrTriggerHit but for instruction address.
    bool instAddrTriggerHit(URV address, unsigned size, TriggerTiming timing,
                            PrivilegeMode mode, bool virtMode, bool ie,
                            const TextraContext<URV>& context, URV& hitAddr);

    /// Similar to instAddrTriggerHit but for instruction opcode.
    bool instOpcodeTriggerHit(URV opcode, TriggerTiming timing,
                              PrivilegeMode mode, bool virtMode, bool ie,
                              const TextraContext<URV>& context);

    /// Make every active icount trigger count down unless the current instruction wrote
    /// its tdata1, or its action is suppressed (e.g. action is ebreak exception and
    /// interrupts are disabled). A trigger whose count reaches zero becomes pending.
    void evaluateIcount(PrivilegeMode mode, bool virtMode, bool ie, bool skipModified,
                        const TextraContext<URV>& context);

    bool icountTriggerFired(PrivilegeMode mode, bool virtMode, bool interruptEnabled,
                            const TextraContext<URV>& context);

    /// Set the hit bit of every exception-trigger (etrigger) that matches. Return true if
    /// one of them has the "raise breakpoint" or "enter debug mode" action, and set
    /// enterDebug if one has the latter. The mode/virtMode are those the trap was taken
    /// from; handlerMode is the mode it was taken into, where the trigger fires.
    bool expTriggerHit(URV cause, PrivilegeMode mode, bool virtMode,
                       PrivilegeMode handlerMode, bool interruptEnabled,
                       const TextraContext<URV>& context, bool& enterDebug);

    /// Same as expTriggerHit but for the interrupt-triggers (itrigger).
    bool intTriggerHit(URV cause, PrivilegeMode mode, bool virtMode,
                       PrivilegeMode handlerMode, bool interruptEnabled,
                       const TextraContext<URV>& context, bool& enterDebug, bool isNmi);

    /// Return the tigger at the given index. Reurn None if index is out of bounds.
    TriggerType triggerType(URV trigger) const
    {
      if (trigger >= triggers_.size())
	return TriggerType::None;
      return triggers_.at(trigger).type();
    }

    /// Reset the given trigger with the given data1, data2, and data3
    /// values and corresponding write and poke masks. Values are applied
    /// without masking. Subsequent writes will be masked.
    bool reset(URV trigger, URV data1, URV data2, URV data3,
	       URV writeMask1, URV writeMask2, URV writeMask3,
	       URV pokeMask1, URV pokeMask2, URV pokeMask3);

    /// Configure given trigger with given reset values, write masks and
    /// and poke masks.
    bool config(unsigned trigger, 
		const std::vector<uint64_t>& resets,
		const std::vector<uint64_t>& masks,
		const std::vector<uint64_t>& pokeMasks);

    /// Get the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool peek(unsigned trigger, uint64_t& data1, uint64_t& data2,
              uint64_t& data3) const;

    /// Get the values of the three components of the given debug
    /// trigger as well as the components write and poke masks. Return
    /// true on success and false if trigger is out of bounds.
    bool peek(unsigned trigger,
              uint64_t& data1, uint64_t& data2, uint64_t& data3,
	      uint64_t& wm1, uint64_t& wm2, uint64_t& wm3,
	      uint64_t& pm1, uint64_t& pm2, uint64_t& pm3) const;

    /// Set the values of the three components of the given debug
    /// trigger. Return true on success and false if trigger is out of
    /// bounds.
    bool poke(URV trigger, URV v1, URV v2, URV v3);

    bool pokeData1(URV trigger, URV val);
    bool pokeData2(URV trigger, URV val);
    bool pokeData3(URV trigger, URV val);
    bool pokeInfo(URV trigger, URV val);

    /// Clear the remembered indices of the triggers written by the
    /// last instruction.
    void clearLastWrittenTriggers()
    {
      for (auto& trig : triggers_)
	{
	  trig.setLocalHit(false);
	  trig.setChainHit(false);
	  trig.clearModified();
	}
    }

    /// Fill the trigs vector with the indices of the triggers written
    /// by the last instruction.
    void getLastWrittenTriggers(std::vector<unsigned>& trigs) const
    {
      trigs.clear();
      for (unsigned i = 0; i < triggers_.size(); ++i)
	if (triggers_.at(i).isModified())
	  trigs.push_back(i);
    }

    /// Set before/after to the count of tripped triggers with
    /// before/after timing.
    void countTrippedTriggers(unsigned& before, unsigned& after) const
    {
      before = after = 0;
      for (const auto& trig : triggers_)
	if (trig.hasTripped())
	  (trig.getTiming() == TriggerTiming::Before)? before++ : after++;
    }

    /// Return true if there is one or more tripped trigger action is set to "enter debug
    /// mode".
    bool hasEnterDebugModeTripped() const
    {
      for (const auto& trig : triggers_)
	if (trig.hasTripped())
          {
            // If chained, use action of last trigger in chain.
            size_t start = 0, end = 0;
            trig.getChainBounds(start, end);
            auto& last = triggers_.at(end - 1);
            if (last.getAction() == TriggerAction::EnterDebug)
              return true;
          }
      return false;
    }

    /// Return true if there is one or more tripped trigger action is set to "take
    /// breakpoint".
    bool hasBreakpTripped() const
    {
      for (const auto& trig : triggers_)
	if (trig.hasTripped())
          {
            // If chained, use action of last trigger in chain.
            size_t start = 0, end = 0;
            trig.getChainBounds(start, end);
            auto& last = triggers_.at(end - 1);
            if (last.getAction() == TriggerAction::RaiseBreak)
              return true;
          }
      return false;
    }

    /// Return true if there is one or more tripped trigger action is set to
    /// "take breakpoint" or to "enter debug mode".
    bool breakpOrEnterDebugTripped() const
    {
      for (const auto& trig : triggers_)
	if (trig.hasTripped())
          {
            // If chained, use action of last trigger in chain.
            size_t start = 0, end = 0;
            trig.getChainBounds(start, end);
            auto& last = triggers_.at(end - 1);
            auto action = last.getAction();
            if (action == TriggerAction::RaiseBreak or action == TriggerAction::EnterDebug)
              return true;
          }
      return false;
    }

    /// Enable all ld/st address matching [address, address+size-1].
    void enableAllDataAddrMatch(bool flag)
    { for ( auto& trig : triggers_) trig.enableAllDataAddrMatch(flag); }

    /// Enable all inst address matching [address, address+size-1].
    void enableAllInstrAddrMatch(bool flag)
    { for ( auto& trig : triggers_) trig.enableAllInstrAddrMatch(flag); }

    /// Enable all ld/st address matching [address, address+size-1] for a particular match
    /// type.
    void enableAllDataAddrMatch(unsigned matchType, bool flag)
    { for ( auto& trig : triggers_) trig.enableAllDataAddrMatch(matchType, flag); }

    /// Enable all inst address matching [address, address+size-1] for a particular match
    /// tpye.
    void enableAllInstrAddrMatch(unsigned matchType, bool flag)
    { for ( auto& trig : triggers_) trig.enableAllInstrAddrMatch(matchType, flag); }

    /// Set the maximum NAPOT range with maskmax. Return false, changing nothing, if
    /// maskMax is not in 1 to XLEN-1.
    bool configNapotMaskMax(unsigned maskMax)
    {
      if (maskMax == 0 or maskMax > (8*sizeof(URV) - 1))
        return false;

      for ( auto& trig : triggers_) trig.configNapotMaskMax(maskMax);
      return true;
    }

    /// Reset all triggers.
    void reset();

    /// Return true if given trigger has a local hit.
    bool getLocalHit(URV ix) const
    { return ix < triggers_.size()? triggers_[ix].getLocalHit() : false; }

    /// Enable/disable firing of triggers in machine mode. This supports MTE bit of the
    /// TCONTROL CSR. Applicable only if TCONTROL is enabled (see enableTcontrol).
    void enableMachineMode(bool flag)
    { mmodeEnabled_ = flag; }

    /// Enable use of TCONTROL to control firing of triggers in machine mode.
    void enableTcontrol(bool flag)
    { tcontrolEnabled_ = flag; }

    /// Return true if TCONTROL controls firing of triggers in machine mode.
    bool isTcontrolEnabled() const
    { return tcontrolEnabled_; }

    /// Return true if given trigger type is supported.
    bool isSupportedType(TriggerType type) const
    {
      auto ix = unsigned(type);
      return ix < supportedTypes_.size() ? supportedTypes_.at(ix) : false;
    }

    /// Return true if given action is supported.
    bool isSupportedAction(TriggerAction action) const
    {
      auto ix = unsigned(action);
      return ix < supportedActions_.size() ? supportedActions_.at(ix) : false;
    }

    /// Set the supported trigger types to the types in the given vector. Items not in the
    /// vector are not supported. Return true on success. Return false if
    /// "none"/"disabled" is not in types vector or if a string in the vector does not correspond
    /// to a trigger type. Valid strings:
    ///   "none", "legacy", "mcontrol", "icount", "itrigger", "etrigger", "mcontrol6",
    //    "tmexttrigger", "disabled"
    bool setSupportedTypes(const std::vector<std::string>& types);

    /// Set the supported trigger types to the types in the given vector. Items not in the
    /// vector are not supported. Return true on success. Return false if None/Disabled is
    /// not in types vector.
    bool setSupportedTypes(const std::vector<TriggerType>& types);

    /// Same as above but for trigger actions.
    bool setSupportedActions(const std::vector<std::string>& types);

    bool setSupportedActions(const std::vector<TriggerAction>& actions);

    /// Enable hypervisor mode.
    void enableHypervisor(bool flag);

    /// Enable supervisor or user mode: The s or u bits of tdata1 are read-only-zero if
    /// that mode is disabled.
    void enableMode(PrivilegeMode mode, bool flag);

    /// Set the read mask of TDATA1 when the type is disabled (15): internal value is
    /// anded with this mask on CSR read. Default value makes most significant 5 bits of
    /// TDATA1 visible and the remaining bits 0.
    void setDisabledReadMask(URV mask)
    {
      disabledReadMask_ = mask;
      data1ReadMasks_.at(unsigned(TriggerType::Disabled)) = mask;
    }

    /// If flag is true, then clear the bits of tdata1 (except for type and dmode) whenever
    /// a CSR instruction attempts to write it and the incoming type field is "disabled".
    void clearTdata1OnDisabled(bool flag)
    { clearData1OnDisabled_ = flag; }

    /// When enabled, writing an unsupported action value will clear the action field
    /// to 0 instead of preserving the old value.
    void configClearUnsupportedAction(bool flag)
    { clearUnsupportedAction_ = flag; }

    void getTriggerChange(URV ix, std::vector<std::pair<TriggerOffset, uint64_t>>& changes) const
    {
      changes.clear();
      if (ix >= triggers_.size())
        return;

      using TO = TriggerOffset;

      const auto& trig = triggers_.at(ix);
      if (trig.modifiedT1_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tdata1, trig.data1_.value_));

      if (trig.modifiedT2_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tdata2, trig.data2_));

      if (trig.modifiedT3_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tdata3, trig.data3_));

      if (trig.modifiedInfo_)
	changes.push_back(std::pair<TO, uint64_t>(TO::Tinfo, trig.info_.value_));
    }

    bool isTdata3Modified(URV ix) const
    { return ix < triggers_.size()? triggers_[ix].modifiedT3_ : false; }

  protected:

    /// Return pointer to preceeding trigger in chain or nullptr
    /// if no such trigger.
    Trigger<URV>* getPreceedingTrigger(const Trigger<URV>& trig)
    {
      size_t beginChain = 0, endChain = 0;
      trig.getChainBounds(beginChain, endChain);
      for (unsigned i = beginChain; i < endChain; ++i)
        if (&triggers_.at(i) == &trig)
          return i == beginChain? nullptr : &triggers_.at(i-1);
      return nullptr;
    }

    /// If all the triggers in the chain of the given trigger have tripped (in isolation
    /// using local-hit), then return true setting the hit bit of these
    /// triggers. Otherwise, return false.
    bool updateChainHitBit(Trigger<URV>& trigger);

    /// Define the chain bounds of each trigger.
    void defineChainBounds();

  private:

    std::vector<bool> supportedTypes_;   // Indexed by a TriggerMode.
    std::vector<bool> supportedActions_; // Indexed by an Action.

    std::vector< Trigger<URV> > triggers_;
    bool mmodeEnabled_ = true;  // Triggers trip in Machine mode when true.
    bool tcontrolEnabled_ = true;
    bool hyperEnabled_ = false;
    bool clearData1OnDisabled_ = false;
    bool clearUnsupportedAction_ = false;

    // Read mask for TDATA1 when type is disabled (15).
    URV disabledReadMask_ = URV(0x1f) << (sizeof(URV)*8 - 5);  // Most sig 5 bits visible.

    // Set a read/write mask for each type.
    constexpr static unsigned typeLimit_ = unsigned(TriggerType::Disabled) + 1;
    std::array<URV, typeLimit_> data1ReadMasks_;
    std::array<URV, typeLimit_> data1WriteMasks_;
  };
}

