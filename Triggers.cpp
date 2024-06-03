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

#include <cassert>

#include "Triggers.hpp"


using namespace WdRiscv;


template <typename URV>
Triggers<URV>::Triggers(unsigned count)
  : triggers_(count)
{
  // Define each triggers as a single-element chain.
  for (unsigned i = 0; i < count; ++i)
    triggers_.at(i).setChainBounds(i, i+1);
}


template <typename URV>
bool
Triggers<URV>::readData1(URV trigger, URV& value) const
{
  if (trigger >= triggers_.size())
    return false;

  value = triggers_.at(trigger).readData1();
  return true;
}


template <typename URV>
bool
Triggers<URV>::readData2(URV trigger, URV& value) const
{
  if (trigger >= triggers_.size())
    return false;

  value = triggers_.at(trigger).readData2();
  return true;
}


template <typename URV>
bool
Triggers<URV>::readData3(URV trigger, URV& value) const
{
  if (trigger >= triggers_.size())
    return false;

  value = triggers_.at(trigger).readData3();
  return true;
}


template <typename URV>
bool
Triggers<URV>::readInfo(URV trigger, URV& value) const
{
  if (trigger >= triggers_.size())
    {
      value = 1;  // Per spec.
      return true;
    }

  value = triggers_.at(trigger).readInfo();
  return true;
}


template <typename URV>
bool
Triggers<URV>::writeData1(URV trigIx, bool debugMode, URV value)
{
  if (trigIx >= triggers_.size())
    return false;

  auto& trig = triggers_.at(trigIx);

  Data1Bits d1bits(value); // Unpack value of attempted write.

  // If next trigger has a dmode of 1 (debugger only), then clear
  // chain bit in attempted wirte if write would also set dmode to 0.
  // Otherwise we would have a chain with different dmodes.
  if (trigIx + 1 < triggers_.size())
    {
      auto& nextTrig = triggers_.at(trigIx + 1);
      if (nextTrig.isDebugModeOnly() and not d1bits.dmodeOnly())
        {
          d1bits.mcontrol_.chain_ = 0;
          value = d1bits.value_;
        }
    }

  // Write is ignored if it would set dmode and previous trigger has
  // both dmode=0 and chain=1. Otherwise, we would have a chain with
  // different dmodes.
  if (d1bits.dmodeOnly() and trigIx > 0)
    {
      auto& prevTrig = triggers_.at(trigIx - 1);
      if (prevTrig.getChain() and not prevTrig.isDebugModeOnly())
        return false;
    }

  bool oldChain = trig.getChain();

  if (not trig.writeData1(debugMode, value))
    return false;

  bool newChain = trig.getChain();
  if (oldChain != newChain)
    defineChainBounds();

  return true;
}


template <typename URV>
bool
Triggers<URV>::writeData2(URV trigger, bool debugMode, URV value)
{
  if (trigger >= triggers_.size())
    return false;

  return triggers_.at(trigger).writeData2(debugMode, value);
}


template <typename URV>
bool
Triggers<URV>::writeData3(URV trigger, bool debugMode, URV value)
{
  if (trigger >= triggers_.size())
    return false;

  return triggers_.at(trigger).writeData3(debugMode, value);
}


template <typename URV>
bool
Triggers<URV>::writeInfo(URV trigger, bool debugMode, URV value)
{
  if (trigger >= triggers_.size())
    return false;

  return triggers_.at(trigger).writeInfo(debugMode, value);
}


template <typename URV>
bool
Triggers<URV>::updateChainHitBit(Trigger<URV>& trigger)
{
  bool chainHit = true;  // True if all items in chain hit
  TriggerTiming  timing = trigger.getTiming();
  bool uniformTiming = true;

  size_t beginChain = 0, endChain = 0;
  trigger.getChainBounds(beginChain, endChain);

  for (size_t i = beginChain; i < endChain; ++i)
    {
      auto& trig = triggers_.at(i);
      chainHit = chainHit and trig.getLocalHit();
      uniformTiming = uniformTiming and (timing == trig.getTiming());
      if (chainHit)
        trig.setHit(true);
    }

  if (not chainHit or not uniformTiming)
    return false;

  for (size_t i = beginChain; i < endChain; ++i)
    triggers_.at(i).setTripped(true);

  return true;
}


template <typename URV>
bool
Triggers<URV>::ldStAddrTriggerHit(URV address, TriggerTiming timing,
				  bool isLoad, PrivilegeMode mode,
                                  bool virtMode,
                                  bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are enabled.
  bool skip = mode == PrivilegeMode::Machine and interruptEnabled and not mmodeWithIe_;

  bool chainHit = false;  // Chain hit.
  for (auto& trigger : triggers_)
    {
      if (not trigger.isEnterDebugOnHit())
	{
	  if (skip)
	    continue;
	  if (mode == PrivilegeMode::Machine and not mmodeEnabled_)
	    continue;  // Cannot fire in machine mode.
	}

      if (not trigger.matchLdStAddr(address, timing, isLoad, mode, virtMode))
	continue;

      trigger.setLocalHit(true);

      if (updateChainHitBit(trigger))
	chainHit = true;
    }
  return chainHit;
}


template <typename URV>
bool
Triggers<URV>::ldStDataTriggerHit(URV value, TriggerTiming timing, bool isLoad,
				  PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are enabled.
  bool skip = mode == PrivilegeMode::Machine and interruptEnabled and not mmodeWithIe_;

  bool chainHit = false;  // Chain hit.
  for (auto& trigger : triggers_)
    {
      if (not trigger.isEnterDebugOnHit())
	{
	  if (skip)
	    continue;
	  if (mode == PrivilegeMode::Machine and not mmodeEnabled_)
	    continue;  // Cannot fire in machine mode.
	}

      if (not trigger.matchLdStData(value, timing, isLoad, mode, virtMode))
	continue;

      trigger.setLocalHit(true);

      if (updateChainHitBit(trigger))
	chainHit = true;
    }

  return chainHit;
}


template <typename URV>
bool
Triggers<URV>::instAddrTriggerHit(URV address, TriggerTiming timing,
                                  PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are enabled.
  bool skip = mode == PrivilegeMode::Machine and interruptEnabled and not mmodeWithIe_;

  bool chainHit = false;  // Chain hit.
  for (auto& trigger : triggers_)
    {
      if (not trigger.isEnterDebugOnHit())
	{
	  if (skip)
	    continue;
	  if (mode == PrivilegeMode::Machine and not mmodeEnabled_)
	    continue;  // Cannot fire in machine mode.
	}

      if (not trigger.matchInstAddr(address, timing, mode, virtMode))
	continue;

      trigger.setLocalHit(true);

      if (updateChainHitBit(trigger))
	chainHit = true;
    }

  return chainHit;
}


template <typename URV>
bool
Triggers<URV>::instOpcodeTriggerHit(URV opcode, TriggerTiming timing,
                                    PrivilegeMode mode, bool virtMode,
				    bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are enabled.
  bool skip = mode == PrivilegeMode::Machine and interruptEnabled and not mmodeWithIe_;

  bool hit = false;
  for (auto& trigger : triggers_)
    {
      if (not trigger.isEnterDebugOnHit())
	{
	  if (skip)
	    continue;
	  if (mode == PrivilegeMode::Machine and not mmodeEnabled_)
	    continue;  // Cannot fire in machine mode.
	}

      if (not trigger.matchInstOpcode(opcode, timing, mode, virtMode))
	continue;

      trigger.setLocalHit(true);

      if (updateChainHitBit(trigger))
	hit = true;
    }

  return hit;
}


template <typename URV>
bool
Triggers<URV>::icountTriggerHit(PrivilegeMode prevPrivMode, bool prevVirtMode, PrivilegeMode mode,
				bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are enabled.
  bool skip = mode == PrivilegeMode::Machine and interruptEnabled and not mmodeWithIe_;

  bool hit = false;

  for (auto& trig : triggers_)
    {
      if (trig.isModified())
	continue; // Trigger was written by current instruction.

      if (not trig.instCountdown(prevPrivMode, prevVirtMode))
        continue;

      if (not trig.isEnterDebugOnHit())
	{
	  if (skip)
	    continue;
	  if (mode == PrivilegeMode::Machine and not mmodeEnabled_)
	    continue;  // Cannot fire in machine mode.
	}

      if (not trig.matchInstCount(mode, virtMode) or
          not trig.data1_.icount_.pending_)
        continue; // Next mode is non-matching.

      hit = true;
      trig.setHit(true);
      trig.setLocalHit(true);
      trig.data1_.icount_.pending_ = false;
    }
  return hit;
}


template <typename URV>
bool
Triggers<URV>::expTriggerHit(URV cause, PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are enabled.
  bool skip = mode == PrivilegeMode::Machine and interruptEnabled and not mmodeWithIe_;

  URV mask = URV(1) << cause;

  bool hit = false;

  for (auto& trigger : triggers_)
    {
      using PM = PrivilegeMode;

      if (not trigger.isEnterDebugOnHit())
	{
	  if (skip)
	    continue;

	  if (mode == PM::Machine and not mmodeEnabled_)
	    continue;  // Cannot fire in machine mode.
	}
	  
      if (not trigger.data1_.isEtrigger())
	continue;

      auto& etrig = trigger.data1_.etrigger_;

      if (mode == PM::Machine and not etrig.m_)
	continue;

      if (mode == PM::Supervisor and not virtMode and not etrig.s_)
	continue;

      if (mode == PrivilegeMode::User and not virtMode and not etrig.u_)
	continue;

      if (mode == PrivilegeMode::Reserved)
	continue;

      URV data2 = trigger.data2_;
      if (data2 & mask)
	{
	  trigger.setLocalHit(true);
	  hit = true;
	}
    }

  return hit;
}


template <typename URV>
bool
Triggers<URV>::intTriggerHit(URV cause, PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are enabled.
  bool skip = mode == PrivilegeMode::Machine and interruptEnabled and not mmodeWithIe_;

  cause = (cause << 1) >> 1;  // Clear most sig bit.

  URV mask = URV(1) << cause;

  bool hit = false;

  for (auto& trigger : triggers_)
    {
      using PM = PrivilegeMode;

      if (not trigger.isEnterDebugOnHit())
	{
	  if (skip)
	    continue;

	  if (mode == PM::Machine and not mmodeEnabled_)
	    continue;  // Cannot fire in machine mode.
	}
	  
      if (not trigger.data1_.isItrigger())
	continue;

      auto& itrig = trigger.data1_.itrigger_;

      if (mode == PM::Machine and not itrig.m_)
	continue;

      if (mode == PM::Supervisor and not virtMode and not itrig.s_)
	continue;

      if (mode == PrivilegeMode::User and not virtMode and not itrig.u_)
	continue;

      if (mode == PrivilegeMode::Reserved)
	continue;

      URV data2 = trigger.data2_;
      if (data2 & mask)
	{
	  trigger.setLocalHit(true);
	  hit = true;
	}
    }

  return hit;
}


template <typename URV>
bool
Triggers<URV>::config(unsigned triggerIx,
		      const std::vector<uint64_t>& resets,
		      const std::vector<uint64_t>& masks,
		      const std::vector<uint64_t>& pokeMasks)
{
  if (triggerIx <= triggers_.size())
    triggers_.resize(triggerIx + 1);

  if (resets.size() !=masks.size() or resets.size() != pokeMasks.size())
    return false;

  auto& trigger = triggers_.at(triggerIx);

  if (resets.size() > 0)
    trigger.configData1(resets.at(0), masks.at(0), pokeMasks.at(0));

  if (resets.size() > 1)
    {
      trigger.configData2(resets.at(1), masks.at(1), pokeMasks.at(1));
      trigger.writeData2(true, resets.at(1));  // Define compare mask.
    }

  if (resets.size() > 2)
    trigger.configData3(resets.at(2), masks.at(2), pokeMasks.at(2));

  if (resets.size() > 3)
    trigger.configInfo(resets.at(3), masks.at(3), pokeMasks.at(3));

  defineChainBounds();

  return true;
}


template <typename URV>
void
Triggers<URV>::reset()
{
  for (auto& trigger : triggers_)
    trigger.reset();
  defineChainBounds();
}



template <typename URV>
bool
Triggers<URV>::peek(unsigned trigger, uint64_t& data1, uint64_t& data2,
                    uint64_t& data3) const
{
  if (trigger >= triggers_.size())
    return false;

  return triggers_.at(trigger).peek(data1, data2, data3);
}


template <typename URV>
bool
Triggers<URV>::peek(unsigned trigger,
                    uint64_t& data1, uint64_t& data2, uint64_t& data3,
		    uint64_t& wm1, uint64_t& wm2, uint64_t& wm3,
		    uint64_t& pm1, uint64_t& pm2, uint64_t& pm3) const
{
  if (trigger >= triggers_.size())
    return false;

  const Trigger<URV>& trig = triggers_.at(trigger);
  return trig.peek(data1, data2, data3, wm1, wm2, wm3, pm1, pm2, pm3);
}


template <typename URV>
bool
Triggers<URV>::poke(URV trigger, URV v1, URV v2, URV v3)
{
  if (trigger >= triggers_.size())
    return false;

  Trigger<URV>& trig = triggers_.at(trigger);

  trig.pokeData1(v1);
  trig.pokeData2(v2);
  trig.pokeData3(v3);

  return true;
}


template <typename URV>
bool
Triggers<URV>::pokeData1(URV trigIx, URV value)
{
  if (trigIx >= triggers_.size())
    return false;

  auto& trig = triggers_.at(trigIx);

  Data1Bits d1bits(value); // Unpack value of attempted write.

  // If next trigger has a dmode of 1 (debugger only), then clear
  // chain bit in attempted wirte if write would also set dmode to 0.
  // Otherwise we would have a chain with different dmodes.
  if (trigIx + 1 < triggers_.size())
    {
      auto& nextTrig = triggers_.at(trigIx + 1);
      if (nextTrig.isDebugModeOnly() and not d1bits.dmodeOnly())
        {
          d1bits.mcontrol_.chain_ = 0;
          value = d1bits.value_;
        }
    }

  // Write is ignored if it would set dmode and previous trigger has
  // both dmode=0 and chain=1. Otherwise, we would have a chain with
  // different dmodes.
  if (d1bits.dmodeOnly() and trigIx > 0)
    {
      auto& prevTrig = triggers_.at(trigIx - 1);
      if (prevTrig.getChain() and not prevTrig.isDebugModeOnly())
        return false;
    }

  bool oldChain = trig.getChain();

  trig.pokeData1(value);

  bool newChain = trig.getChain();
  if (oldChain != newChain)
    defineChainBounds();

  return true;
}


template <typename URV>
bool
Triggers<URV>::pokeData2(URV trigger, URV val)
{
  if (trigger >= triggers_.size())
    return false;

  Trigger<URV>& trig = triggers_.at(trigger);

  trig.pokeData2(val);
  return true;
}


template <typename URV>
bool
Triggers<URV>::pokeData3(URV trigger, URV val)
{
  if (trigger >= triggers_.size())
    return false;

  Trigger<URV>& trig = triggers_.at(trigger);

  trig.pokeData3(val);
  return true;
}


template <typename URV>
bool
Triggers<URV>::pokeInfo(URV trigger, URV val)
{
  if (trigger >= triggers_.size())
    return false;

  Trigger<URV>& trig = triggers_.at(trigger);

  trig.pokeInfo(val);
  return true;
}


template <typename URV>
void
Triggers<URV>::defineChainBounds()
{
  size_t begin = 0, end = 0;

  for (size_t i = 0; i < triggers_.size(); ++i)
    {
      if (not triggers_.at(i).getChain())
	{
	  end = i + 1;
	  for (size_t j = begin; j < end; j++)
	    triggers_.at(j).setChainBounds(begin, end);
	  begin = end;
	}
    }

  end = triggers_.size();
  for  (size_t i = begin; i < end; ++i)
    triggers_.at(i).setChainBounds(begin, end);
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchLdStAddr(URV address, TriggerTiming timing, bool isLoad,
                            PrivilegeMode mode, bool virtMode) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (mode == PrivilegeMode::Machine and not ctl.m_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Supervisor and not virtMode and not ctl.s_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::User and not virtMode and not ctl.u_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Reserved)
    return false;  // Not enabled;

  if constexpr (std::is_same_v<M, decltype(data1_.mcontrol6_)>)
    {
      if (mode == PrivilegeMode::Supervisor and virtMode and not ctl.vs_)
        return false;  // Not enabled;

      if (mode == PrivilegeMode::User and virtMode and not ctl.vu_)
        return false;  // Not enabled;
    }
  else if (virtMode)
    return false;

  bool isStore = not isLoad;

  if (getTiming() == timing and Select(ctl.select_) == Select::MatchAddress and
      ((isLoad and ctl.load_) or (isStore and ctl.store_)))
    return doMatch(address);

  return false;
}


template <typename URV>
bool
Trigger<URV>::matchLdStAddr(URV address, TriggerTiming timing, bool isLoad,
                            PrivilegeMode mode, bool virtMode) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchLdStAddr<decltype(data1_.mcontrol_)>(address, timing, isLoad, mode, virtMode);
  else
    return matchLdStAddr<decltype(data1_.mcontrol6_)>(address, timing, isLoad, mode, virtMode);
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchLdStData(URV value, TriggerTiming timing, bool isLoad,
                            PrivilegeMode mode, bool virtMode) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (mode == PrivilegeMode::Machine and not ctl.m_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Supervisor and not virtMode and not ctl.s_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::User and not virtMode and not ctl.u_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Reserved)
    return false;  // Not enabled;

  if constexpr (std::is_same_v<M, decltype(data1_.mcontrol6_)>)
    {
      if (mode == PrivilegeMode::Supervisor and virtMode and not ctl.vs_)
        return false;  // Not enabled;

      if (mode == PrivilegeMode::User and virtMode and not ctl.vu_)
        return false;  // Not enabled;
    }
  else if (virtMode)
    return false;

  bool isStore = not isLoad;

  if (getTiming() == timing and Select(ctl.select_) == Select::MatchData and
      ((isLoad and ctl.load_) or (isStore and ctl.store_)))
    return doMatch(value);
  return false;
}


template <typename URV>
bool
Trigger<URV>::matchLdStData(URV value, TriggerTiming timing, bool isLoad,
                            PrivilegeMode mode, bool virtMode) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchLdStData<decltype(data1_.mcontrol_)>(value, timing, isLoad, mode, virtMode);
  else
    return matchLdStData<decltype(data1_.mcontrol6_)>(value, timing, isLoad, mode, virtMode);
}


template <typename URV>
bool
Trigger<URV>::doMatch(URV item) const
{
  URV data2 = data2_;

  auto doMatch = [this] (URV item, URV compare, Match match) -> bool {
    switch (match)
      {
      case Match::Equal:
        return item == compare;

      case Match::Masked:
        return (item & data2CompareMask_) == (compare & data2CompareMask_);

      case Match::GE:
        return item >= compare;

      case Match::LT:
        return item < compare;

      case Match::MaskHighEqualLow:
        {
          unsigned halfBitCount = 4*sizeof(URV);
          // Mask low half of item with tdata2 high half
          item = item & (compare >> halfBitCount);
          // Compare low half
          return (item << halfBitCount) == (compare << halfBitCount);
        }

      case Match::MaskLowEqualHigh:
        {
          unsigned halfBitCount = 4*sizeof(URV);
          // Mask high half of item with high half of tdata2
          item = item & compare;
          // Compare high half of item with low half of tdata2
	  compare = (compare << halfBitCount) >> halfBitCount;  // Clear high half of tdata2
          return (item >> halfBitCount) == compare;
        }

      default:
        assert(0 and "Unhandled match case.");
      }

    return false;
  };

  Match match = Match(data1_.mcontrol_.match_);
  if (match >= Match::Equal and match <= Match::MaskLowEqualHigh)
    return doMatch(item, data2, match);
  else
    return not doMatch(item, data2, Match(uint32_t(match) - 8));
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchInstAddr(URV address, TriggerTiming timing, PrivilegeMode mode, bool virtMode) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (mode == PrivilegeMode::Machine and not ctl.m_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Supervisor and not virtMode and not ctl.s_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::User and not virtMode and not ctl.u_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Reserved)
    return false;  // Not enabled;

  if constexpr (std::is_same_v<M, decltype(data1_.mcontrol6_)>)
    {
      if (mode == PrivilegeMode::Supervisor and virtMode and not ctl.vs_)
        return false;  // Not enabled;

      if (mode == PrivilegeMode::User and virtMode and not ctl.vu_)
        return false;  // Not enabled;
    }
  else if (virtMode)
    return false;

  if (getTiming() == timing and Select(ctl.select_) == Select::MatchAddress and ctl.execute_)
    return doMatch(address);

  return false;
}


template <typename URV>
bool
Trigger<URV>::matchInstAddr(URV address, TriggerTiming timing,
                            PrivilegeMode mode, bool virtMode) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchInstAddr<decltype(data1_.mcontrol_)>(address, timing, mode, virtMode);
  else
    return matchInstAddr<decltype(data1_.mcontrol6_)>(address, timing, mode, virtMode);
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchInstOpcode(URV opcode, TriggerTiming timing,
                              PrivilegeMode mode, bool virtMode) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (mode == PrivilegeMode::Machine and not ctl.m_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Supervisor and not virtMode and not ctl.s_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::User and not virtMode and not ctl.u_)
    return false;  // Not enabled;

  if (mode == PrivilegeMode::Reserved)
    return false;  // Not enabled;

  if constexpr (std::is_same_v<M, decltype(data1_.mcontrol6_)>)
    {
      if (mode == PrivilegeMode::Supervisor and virtMode and not ctl.vs_)
        return false;  // Not enabled;

      if (mode == PrivilegeMode::User and virtMode and not ctl.vu_)
        return false;  // Not enabled;
    }
  else if (virtMode)
    return false;

  if (getTiming() == timing and Select(ctl.select_) == Select::MatchData and ctl.execute_)
    return doMatch(opcode);
  return false;
}


template <typename URV>
bool
Trigger<URV>::matchInstOpcode(URV opcode, TriggerTiming timing,
                              PrivilegeMode mode, bool virtMode) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchInstOpcode<decltype(data1_.mcontrol_)>(opcode, timing, mode, virtMode);
  else
    return matchInstOpcode<decltype(data1_.mcontrol6_)>(opcode, timing, mode, virtMode);
}


template class WdRiscv::Trigger<uint32_t>;
template class WdRiscv::Trigger<uint64_t>;

template class WdRiscv::Triggers<uint32_t>;
template class WdRiscv::Triggers<uint64_t>;
