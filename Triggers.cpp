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
#include <iostream>

#include "Triggers.hpp"


using namespace WdRiscv;



template <typename URV>
Triggers<URV>::Triggers(unsigned count)
  : triggers_(count)
{
  // Define each triggers as a single-element chain.
  for (unsigned i = 0; i < count; ++i)
    triggers_.at(i).setChainBounds(i, i+1);

  // Define default enabled types.
  unsigned typeLimit = unsigned(TriggerType::Disabled) + 1;
  supportedTypes_.resize(typeLimit, true);

  using TT = TriggerType;
  supportedTypes_.at(unsigned(TT::Reserved0)) = false;
  supportedTypes_.at(unsigned(TT::Reserved1)) = false;
  supportedTypes_.at(unsigned(TT::Reserved2)) = false;
  supportedTypes_.at(unsigned(TT::Custom0)) = false;
  supportedTypes_.at(unsigned(TT::Custom1)) = false;
  supportedTypes_.at(unsigned(TT::Custom2)) = false;


  using TA = TriggerAction;
  supportedActions_.resize(unsigned(TA::Limit) + 1, false);
  supportedActions_.at(unsigned(TA::RaiseBreak)) = true;
  supportedActions_.at(unsigned(TA::EnterDebug)) = true;
  supportedActions_.at(unsigned(TA::StartTrace)) = true;
  supportedActions_.at(unsigned(TA::StopTrace)) = true;
  supportedActions_.at(unsigned(TA::EmitTrace)) = true;
  supportedActions_.at(unsigned(TA::External0)) = true;
  supportedActions_.at(unsigned(TA::External1)) = true;

  // By default read masks allow reading of all bits.
  for (auto& mask : data1ReadMasks_)
    mask = ~URV(0);

  // Same for write masks.
  for (auto& mask : data1WriteMasks_)
    mask = ~URV(0);

  // Setup read mask of tdata1 when type is "disabled": Only top 5 bits
  // readable. Remaining bits are read-only-zero.
  data1ReadMasks_.at(unsigned(TriggerType::Disabled)) = disabledReadMask_;

  // Setup read mask of tdata1 when type is "None": All bits are read-only-zero.
  data1ReadMasks_.at(unsigned(TriggerType::None)) = 0;

  // Update read/write masks to make hyervisor realted bits read-only-zero. That may
  // change later when/if hypervisor is enabled.
  enableHypervisor(false);
}


template <typename URV>
bool
Triggers<URV>::readData1(URV trigIx, URV& value) const
{
  if (trigIx >= triggers_.size())
    return false;

  auto& trigger = triggers_.at(trigIx);
  auto typeIx = unsigned(trigger.type());

  URV readMask = data1ReadMasks_.at(typeIx);

  value = trigger.readData1() & readMask;
  return true;
}


template <typename URV>
bool
Triggers<URV>::peekData1(URV trigIx, URV& value) const
{
  if (trigIx >= triggers_.size())
    return false;

  auto& trigger = triggers_.at(trigIx);
  auto typeIx = unsigned(trigger.type());

  URV readMask = data1ReadMasks_.at(typeIx);

  value = trigger.peekData1() & readMask;
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
      value = 0x1000001;  // Per spec.
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
      if (nextTrig.isDebugModeOnly() and not d1bits.dmode())
        {
          d1bits.mcontrol_.chain_ = 0;
          value = d1bits.value_;
        }
    }

  // Write is ignored if it would set dmode and previous trigger has
  // both dmode=0 and chain=1. Otherwise, we would have a chain with
  // different dmodes.
  if (d1bits.dmode() and trigIx > 0)
    {
      auto& prevTrig = triggers_.at(trigIx - 1);
      if (prevTrig.getChain() and not prevTrig.isDebugModeOnly())
        return false;
    }

  bool oldChain = trig.getChain();

  // If new type is not supported, preserve old type.
  Data1Bits<URV> valBits{value};
  bool preserveType = not isSupportedType(valBits.type());

  if (valBits.type() != TriggerType::None and not preserveType)
    {
      // If new type is not supported by this trigger, preserve old type.
      auto typeNumber = unsigned(valBits.type());
      unsigned mask = 1 << typeNumber;
      TinfoBits tinfo(trig.readInfo());
      preserveType = not (tinfo.info() & mask);
    }

  if (preserveType)
     {
       valBits.setType(trig.data1_.type());
       value = valBits.value_;
     }

  // If new action is not supported, preserve old action or clear field.
  if (not isSupportedAction(valBits.action()))
    {
      auto legal = clearUnsupportedAction_? TriggerAction::RaiseBreak :
                                            trig.data1_.action();
      valBits.setAction(legal);
      value = valBits.value_;
    }

  // If incmoming type is "disabled" clear all other bits except type and dmode (most sig
  // 5 bits).
  if (clearData1OnDisabled_ and valBits.isDisabled())
    {
      auto copy = valBits;
      valBits = Data1Bits<URV>{0};  // Clear
      valBits.setType(copy.type());    // Keep type
      valBits.setDmode(copy.dmode());  // Keep dmode
      value = valBits.value_;
    }

  auto typeIx = unsigned(valBits.type());
  URV mask = data1WriteMasks_.at(typeIx);

  URV prev = trig.readData1();
  URV next = (prev & ~mask) | (value & mask);
  if (not trig.writeData1(debugMode, next))
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
Triggers<URV>::ldStAddrTriggerHit(URV address, unsigned size, TriggerTiming timing,
				  bool isLoad, PrivilegeMode mode,
                                  bool virtMode,
                                  bool interruptEnabled)
{
  // Check if we should skip tripping because we are running in machine mode and
  // interrupts are disabled.
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool chainHit = false;  // Chain hit.
  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;  // Cannot fire in machine mode.

      if (not trigger.matchLdStAddr(address, size, timing, isLoad, mode, virtMode))
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
  // Check if we should skip tripping because of reentrant behavior.
  bool skip = false;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;
  else
    skip = not interruptEnabled and mode == PrivilegeMode::Machine;

  bool chainHit = false;  // Chain hit.
  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;  // Cannot fire in machine mode.

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
Triggers<URV>::instAddrTriggerHit(URV address, unsigned size, TriggerTiming timing,
                                  PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because of reentrant behavior
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool chainHit = false;  // Chain hit.
  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;  // Cannot fire in machine mode.

      if (not trigger.matchInstAddr(address, size, timing, mode, virtMode))
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
  // Check if we should skip tripping because of reentrant behavior. 
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool hit = false;
  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;  // Cannot fire in machine mode.

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
Triggers<URV>::icountTriggerFired(PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because of reentrant behavior. 
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool fire = false;

  for (auto& trig : triggers_)
    {
      if (not trig.matchInstCount(mode, virtMode))
        continue;

      if (unsigned(trig.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trig.isEnterDebugOnHit() and skip)
          continue;  // Cannot fire in machine mode.

      if (trig.data1_.icount_.pending_)
        {
          fire = true;
          // Icount doesn't have a chain bit, we just use it to indicate
          // this trigger is tripped.
          trig.setTripped(fire);
          trig.setHit(true);
          trig.setLocalHit(true);
        }

      trig.data1_.icount_.pending_ = false;
    }

  return fire;
}


template <typename URV>
void
Triggers<URV>::evaluateIcount(PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because of reentrant behavior. 
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  for (auto& trig : triggers_)
    {
      if (not trig.matchInstCount(mode, virtMode))
        continue;

      if (trig.isModified())
	continue; // Trigger was written by current instruction.

      if (unsigned(trig.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trig.isEnterDebugOnHit() and skip)
          continue;  // Cannot hit in machine mode.

      if (not trig.instCountdown())
        continue;

      assert(trig.data1_.icount_.pending_);
    }
}


template <typename URV>
bool
Triggers<URV>::expTriggerHit(URV cause, PrivilegeMode mode, bool virtMode, bool interruptEnabled)
{
  // Check if we should skip tripping because of reentrant behavior. 
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  URV mask = URV(1) << cause;

  bool hit = false;

  for (auto& trigger : triggers_)
    {
      using PM = PrivilegeMode;

      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;  // Cannot fire in machine mode.
	  
      if (not trigger.data1_.isEtrigger())
	continue;

      auto& etrig = trigger.data1_.etrigger_;

      if (mode == PM::Machine and not etrig.m_)
	continue;

      if (not virtMode)
        {
          if (mode == PM::Supervisor and not etrig.s_)
            continue;
          if (mode == PrivilegeMode::User and not etrig.u_)
            continue;
        }
      else
        {
          if (mode == PM::Supervisor and not etrig.vs_)
            continue;
          if (mode == PrivilegeMode::User and not etrig.vu_)
            continue;
        }

      if (mode == PrivilegeMode::Reserved)
	continue;

      URV data2 = trigger.data2_;
      if (data2 & mask)
	{
	  trigger.setLocalHit(true);
          trigger.setHit(true);
	  hit = true;
	}
    }

  return hit;
}


template <typename URV>
bool
Triggers<URV>::intTriggerHit(URV cause, PrivilegeMode mode, bool virtMode, bool interruptEnabled,
                             bool isNmi)
{
  // Check if we should skip tripping because of reentrant behavior. 
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  cause = (cause << 1) >> 1;  // Clear most sig bit.

  URV mask = URV(1) << cause;

  bool hit = false;

  for (auto& trigger : triggers_)
    {
      using PM = PrivilegeMode;

      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;  // Cannot fire in machine mode.
	  
      if (not trigger.data1_.isItrigger())
	continue;

      auto& itrig = trigger.data1_.itrigger_;

      if (itrig.nmi_ != isNmi)
        continue;

      if (mode == PM::Machine and not itrig.m_)
	continue;

      if (not virtMode)
        {
          if (mode == PM::Supervisor and not itrig.s_)
            continue;
          if (mode == PrivilegeMode::User and not itrig.u_)
            continue;
        }
      else
        {
          if (mode == PM::Supervisor and not itrig.vs_)
            continue;
          if (mode == PrivilegeMode::User and not itrig.vu_)
            continue;
        }

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

  if (!resets.empty())
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
Triggers<URV>::setSupportedTypes(const std::vector<TriggerType>& types)
{
  using TT = TriggerType;

  std::fill(supportedTypes_.begin(), supportedTypes_.end(), false);

  supportedTypes_.at(unsigned(TT::None)) = true;
  supportedTypes_.at(unsigned(TT::Disabled)) = true;

  bool hasNone = false, hasDisabled = false;

  for (auto type : types)
    {
      auto ix = unsigned(type);
      if (type == TT::None)
	hasNone = true;
      else if (type == TT::Disabled)
	hasDisabled = true;
      supportedTypes_.at(ix) = true;
    }

  if (not hasNone)
    std::cerr << "Error: Triggers::SetSupportedTypes: Missing triger-type \"none\"\n";

  if (not hasDisabled)
    std::cerr << "Error: Triggers::SetSupportedTypes: Missing triger-type \"disabled\"\n";

  return hasNone and hasDisabled;
}


template <typename URV>
bool
Triggers<URV>::setSupportedTypes(const std::vector<std::string>& strings)
{
  using TT = TriggerType;
  std::vector<TT> types;

  unsigned errors = 0;

  for (const auto& str : strings)
    {
      if (str == "none")
	types.push_back(TT::None);
      else if (str == "legacy")
	types.push_back(TT::Legacy);
      else if (str == "mcontrol")
	types.push_back(TT::Mcontrol);
      else if (str == "icount")
	types.push_back(TT::Icount);
      else if (str == "itrigger")
	types.push_back(TT::Itrigger);
      else if (str == "etrigger")
	types.push_back(TT::Etrigger);
      else if (str == "mcontrol6")
	types.push_back(TT::Mcontrol6);
      else if (str == "tmexttrigger")
	types.push_back(TT::Tmext);
      else if (str == "disabled")
	types.push_back(TT::Disabled);
      else
	{
	  std::cerr << "Error: No such trigger type: " << str << '\n';
	  errors++;
	}
    }

  if (not setSupportedTypes(types))
    errors++;

  return errors == 0;
}


template <typename URV>
bool
Triggers<URV>::setSupportedActions(const std::vector<TriggerAction>& actions)
{
  std::fill(supportedActions_.begin(), supportedActions_.end(), false);

  supportedActions_.at(unsigned(TriggerAction::RaiseBreak)) = true;

  bool hasRaiseBreak = false;

  for (auto action : actions)
    {
      auto ix = unsigned(action);
      if (action == TriggerAction::RaiseBreak)
        hasRaiseBreak = true;
      supportedActions_.at(ix) = true;
    }

  if (not hasRaiseBreak)
    std::cerr << "Error: Triggers::SetSupportedActions: Missing action \"raisebreak\"\n";

  return hasRaiseBreak;
}


template <typename URV>
bool
Triggers<URV>::setSupportedActions(const std::vector<std::string>& strings)
{
  using TA = TriggerAction;
  std::vector<TA> actions;

  unsigned errors = 0;

  for (const auto& str : strings)
    {
      if (str == "raisebreak")
	actions.push_back(TA::RaiseBreak);
      else if (str == "enterdebug")
	actions.push_back(TA::EnterDebug);
      else if (str == "starttrace")
	actions.push_back(TA::StartTrace);
      else if (str == "stoptrace")
	actions.push_back(TA::StopTrace);
      else if (str == "emittrace")
	actions.push_back(TA::EmitTrace);
      else if (str == "external0")
	actions.push_back(TA::External0);
      else if (str == "external1")
	actions.push_back(TA::External1);
      else
	{
	  std::cerr << "Error: No such trigger action: " << str << '\n';
	  errors++;
	}
    }

  if (not setSupportedActions(actions))
    errors++;

  return errors == 0;
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
      if (nextTrig.isDebugModeOnly() and not d1bits.dmode())
        {
          d1bits.mcontrol_.chain_ = 0;
          value = d1bits.value_;
        }
    }

  // Write is ignored if it would set dmode and previous trigger has
  // both dmode=0 and chain=1. Otherwise, we would have a chain with
  // different dmodes.
  if (d1bits.dmode() and trigIx > 0)
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
void
Triggers<URV>::enableHypervisor(bool flag)
{
  // Setup read masks. The vs/vu bits are read-only-zero if hyervisor disabled.


  // Setup mcontrol6.
  URV rmask = data1ReadMasks_.at(unsigned(TriggerType::Mcontrol6));
  Data1Bits<URV> d1Bits(0);
  d1Bits.mcontrol6_.vs_ = 1;
  d1Bits.mcontrol6_.vu_ = 1;
  rmask = flag? rmask | d1Bits.value_ : rmask & ~ d1Bits.value_;
  data1ReadMasks_.at(unsigned(TriggerType::Mcontrol6)) = rmask;

  URV wmask = data1WriteMasks_.at(unsigned(TriggerType::Mcontrol6));
  wmask = flag? wmask | d1Bits.value_ : wmask & ~ d1Bits.value_;
  data1WriteMasks_.at(unsigned(TriggerType::Mcontrol6)) = wmask;

  // Setup icount.
  rmask = data1ReadMasks_.at(unsigned(TriggerType::Mcontrol6));
  d1Bits.value_ = 0;
  d1Bits.icount_.vs_ = 1;
  d1Bits.icount_.vu_ = 1;
  rmask = flag? rmask | d1Bits.value_ : rmask & ~ d1Bits.value_;
  data1ReadMasks_.at(unsigned(TriggerType::Icount)) = rmask;
  data1WriteMasks_.at(unsigned(TriggerType::Icount)) = rmask;

  wmask = data1WriteMasks_.at(unsigned(TriggerType::Icount));
  wmask = flag? wmask | d1Bits.value_ : wmask & ~ d1Bits.value_;
  data1WriteMasks_.at(unsigned(TriggerType::Icount)) = wmask;


  // Setup itrigger.
  rmask = data1ReadMasks_.at(unsigned(TriggerType::Itrigger));
  d1Bits.value_ = 0;
  d1Bits.itrigger_.vs_ = 1;
  d1Bits.itrigger_.vu_ = 1;
  rmask = flag? rmask | d1Bits.value_ : rmask & ~ d1Bits.value_;
  data1WriteMasks_.at(unsigned(TriggerType::Itrigger)) = rmask;

  wmask = data1WriteMasks_.at(unsigned(TriggerType::Itrigger));
  wmask = flag? wmask | d1Bits.value_ : wmask & ~ d1Bits.value_;
  data1WriteMasks_.at(unsigned(TriggerType::Itrigger)) = wmask;

  // Setup etrigger.
  rmask = data1ReadMasks_.at(unsigned(TriggerType::Etrigger));
  d1Bits.value_ = 0;
  d1Bits.etrigger_.vs_ = 1;
  d1Bits.etrigger_.vu_ = 1;
  rmask = flag? rmask | d1Bits.value_ : rmask & ~ d1Bits.value_;
  data1ReadMasks_.at(unsigned(TriggerType::Etrigger)) = rmask;

  wmask = data1WriteMasks_.at(unsigned(TriggerType::Etrigger));
  wmask = flag? wmask | d1Bits.value_ : wmask & ~ d1Bits.value_;
  data1WriteMasks_.at(unsigned(TriggerType::Etrigger)) = wmask;
}


template <typename URV>
Trigger<URV>::Trigger(URV data1, URV data2, URV /*data3*/,
                      URV mask1, URV mask2, URV mask3)
  : data1_(data1), data2_(data2), data1WriteMask_(mask1),
    data2WriteMask_(mask2), data3WriteMask_(mask3)
{
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchLdStAddr(URV address, unsigned size, TriggerTiming timing, bool isLoad,
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
    {
      if constexpr (std::is_same_v<M, decltype(data1_.mcontrol6_)>)
	{
	  switch (data1_.mcontrol6_.size_)
	    {
	    case 0:   // Match any access size
              break;

	    case 1:  // Byte access
	      if (size != 1)
		return false;
	      break;

	    case 2:  // half word access
	      if (size != 2)
		return false;
	      break;

	    case 3:  // word access
	      if (size != 4)
		return false;
	      break;

	    case 4:  // 6-byte access
	      assert(0 && "Error: Assertion failed");  // Not supported
	      break;

	    case 5:  // double word access
	      if (size != 8)
		return false;
	      break;

	    default:
	      assert(0 && "Error: Assertion failed");
	      break;
	    }
	}

      auto match = Match(data1_.mcontrol_.match_);
      if (matchAllDataAddresses(match))
	{
	  // Match all addresses covered by size.
	  bool negated = isNegatedMatch(match);
	  if (negated)
	    match = negateNegatedMatch(match);

	  bool hit = false;
          for (unsigned i = 0; i < size and not hit; ++i)
            hit = hit or doMatch(address + i, match);
	  if (negated)
	    hit = not hit;
          return hit;
	}
      
      // Match first ld/st data address.
      return doMatch(address, match);
    }

  return false;
}


template <typename URV>
bool
Trigger<URV>::matchLdStAddr(URV address, unsigned size, TriggerTiming timing, bool isLoad,
                            PrivilegeMode mode, bool virtMode) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchLdStAddr<decltype(data1_.mcontrol_)>(address, size, timing, isLoad, mode, virtMode);
      return matchLdStAddr<decltype(data1_.mcontrol6_)>(address, size, timing, isLoad, mode, virtMode);
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
    {
      auto match = Match(data1_.mcontrol_.match_);
      return doMatch(value, match);
    }
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
      return matchLdStData<decltype(data1_.mcontrol6_)>(value, timing, isLoad, mode, virtMode);
}


template <typename URV>
bool
Trigger<URV>::doMatch(URV item, Match match) const
{
  URV data2 = data2_;

  auto helper = [this] (URV item, URV compare, Match match) -> bool {
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

  if (match >= Match::Equal and match <= Match::MaskLowEqualHigh)
    return helper(item, data2, match);
      return not helper(item, data2, Match(uint32_t(match) - 8));
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchInstAddr(URV address, unsigned size, TriggerTiming timing, PrivilegeMode mode,
			    bool virtMode) const
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
    {
      if constexpr (std::is_same_v<M, decltype(data1_.mcontrol6_)>)
	{
	  switch (data1_.mcontrol6_.size_)
	    {
	    case 0:   // Match any access size
              break;

	    case 1:  // Byte access
	      if (size != 1)
		return false;
	      break;

	    case 2:  // half word access
	      if (size != 2)
		return false;
	      break;

	    case 3:  // word access
	      if (size != 4)
		return false;
	      break;

	    case 4:  // 6-byte access
	      assert(0 && "Error: Assertion failed");  // Not supported
	      break;

	    case 5:  // double word access
	      if (size != 8)
		return false;
	      break;

	    default:
	      assert(0 && "Error: Assertion failed");
	      break;
	    }
	}

      auto match = Match(data1_.mcontrol_.match_);
      if (matchAllInstrAddresses(match))
        {
	  // Match all addresses covered by size.
	  bool negated = isNegatedMatch(match);
	  if (negated)
	    match = negateNegatedMatch(match);

	  bool hit = false;
          for (unsigned i = 0; i < size and not hit; ++i)
            hit = hit or doMatch(address + i, match);
	  if (negated)
	    hit = not hit;
          return hit;
        }

      return doMatch(address, match);
    }

  return false;
}


template <typename URV>
bool
Trigger<URV>::matchInstAddr(URV address, unsigned size, TriggerTiming timing,
                            PrivilegeMode mode, bool virtMode) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchInstAddr<decltype(data1_.mcontrol_)>(address, size, timing, mode, virtMode);
      return matchInstAddr<decltype(data1_.mcontrol6_)>(address, size, timing, mode, virtMode);
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
    {
      auto match = Match(data1_.mcontrol_.match_);
      return doMatch(opcode, match);
    }

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
      return matchInstOpcode<decltype(data1_.mcontrol6_)>(opcode, timing, mode, virtMode);
}


template class WdRiscv::Trigger<uint32_t>;
template class WdRiscv::Trigger<uint64_t>;

template class WdRiscv::Triggers<uint32_t>;
template class WdRiscv::Triggers<uint64_t>;
