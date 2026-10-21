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

  value = triggers_.at(trigger).readData3(hyperEnabled_);
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

  Data1Bits<URV> valBits{value};
  bool unsupportedType = not isSupportedType(valBits.type());

  if (valBits.type() != TriggerType::None and not unsupportedType)
    unsupportedType = not trig.supportsType(valBits.type());

  if (unsupportedType)
    {
      valBits.setType(TriggerType::Disabled);
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

  if (unsupportedType or (clearData1OnDisabled_ and valBits.isDisabled()))
    {
      auto copy = valBits;
      valBits = Data1Bits<URV>{0};
      valBits.setType(copy.type());
      valBits.setDmode(copy.dmode());
      value = valBits.value_;
    }

  auto typeIx = unsigned(valBits.type());
  URV mask = data1WriteMasks_.at(typeIx);

  URV prev = trig.readData1();
  URV next = (prev & ~mask) | (value & mask);

  // The chain rules below go by the value the write leaves in tdata1, after type
  // resolution and both write masks.
  Data1Bits<URV> written{trig.writtenData1(debugMode, next)};

  // If next trigger has a dmode of 1 (debugger only), then clear
  // chain bit in attempted write if write would also set dmode to 0.
  // Otherwise we would have a chain with different dmodes.
  if (written.isAddrData() and not written.dmode() and trigIx + 1 < triggers_.size())
    {
      auto& nextTrig = triggers_.at(trigIx + 1);
      if (nextTrig.isDebugModeOnly())
        {
          // Zeroed in the value to write, so the trigger's write mask applies to it.
          Data1Bits<URV> bits{next};
          bits.mcontrol_.chain_ = 0;  // Same bit as the chain of mcontrol6.
          next = bits.value_;
        }
    }

  // Write is ignored if it would set dmode and previous trigger has
  // both dmode=0 and chain=1. Otherwise, we would have a chain with
  // different dmodes.
  if (written.dmode() and trigIx > 0)
    {
      auto& prevTrig = triggers_.at(trigIx - 1);
      if (prevTrig.getChain() and not prevTrig.isDebugModeOnly())
        return true;
    }

  bool oldChain = trig.getChain();

  trig.writeData1(debugMode, next);

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

  triggers_.at(trigger).writeData2(debugMode, value);
  return true;
}


template <typename URV>
bool
Triggers<URV>::writeData3(URV trigger, bool debugMode, URV value)
{
  if (trigger >= triggers_.size())
    return false;

  triggers_.at(trigger).writeData3(debugMode, value, hyperEnabled_);
  return true;
}


template <typename URV>
bool
Triggers<URV>::writeInfo(URV trigger, bool debugMode, URV value)
{
  if (trigger >= triggers_.size())
    return false;

  triggers_.at(trigger).writeInfo(debugMode, value);
  return true;
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
Triggers<URV>::ldStAddrTriggerHit(URV addr, unsigned size, TriggerTiming timing,
                                  bool isLoad, bool isStore, PrivilegeMode mode,
                                  bool virtMode, bool interruptEnabled,
                                  const TextraContext<URV>& context, URV& hitAddr)
{
  // Check if we should skip tripping because of reentrant behavior.
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool chainHit = false;
  bool breakHit = false;  // A chain raising the breakpoint hit.
  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;

      URV ha = addr;   // Hit address.
      if (not trigger.matchLdStAddr(addr, size, timing, isLoad, isStore, mode, virtMode,
                                    context, ha))
	continue;

      trigger.setLocalHit(true);

      if (not updateChainHitBit(trigger))
        continue;

      chainHit = true;
      if (not breakHit and chainAction(trigger) == TriggerAction::RaiseBreak)
        {
          breakHit = true;
          hitAddr = ha;
        }
    }

  return chainHit;
}


template <typename URV>
bool
Triggers<URV>::ldStDataTriggerHit(URV value, TriggerTiming timing, bool isLoad,
				                          PrivilegeMode mode, bool virtMode,
                                  bool interruptEnabled, const TextraContext<URV>& context)
{
  // Check if we should skip tripping because of reentrant behavior.
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool chainHit = false;
  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;

      if (not trigger.matchLdStData(value, timing, isLoad, mode, virtMode, context))
	continue;

      trigger.setLocalHit(true);

      if (updateChainHitBit(trigger))
	chainHit = true;
    }

  return chainHit;
}


template <typename URV>
bool
Triggers<URV>::instAddrTriggerHit(URV addr, unsigned size, TriggerTiming timing,
                                  PrivilegeMode mode, bool virtMode, bool interruptEnabled,
                                  const TextraContext<URV>& context, URV& hitAddr)
{
  // Check if we should skip tripping because of reentrant behavior
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool chainHit = false;
  bool breakHit = false;  // A chain raising the breakpoint hit.
  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;

      URV ha = addr;  // Hit address
      if (not trigger.matchInstAddr(addr, size, timing, mode, virtMode, context, ha))
	continue;

      trigger.setLocalHit(true);

      if (not updateChainHitBit(trigger))
        continue;

      chainHit = true;
      if (not breakHit and chainAction(trigger) == TriggerAction::RaiseBreak)
        {
          breakHit = true;
          hitAddr = ha;
        }
    }

  return chainHit;
}


template <typename URV>
bool
Triggers<URV>::instOpcodeTriggerHit(URV opcode, TriggerTiming timing,
                                    PrivilegeMode mode, bool virtMode,
				    bool interruptEnabled, const TextraContext<URV>& context)
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
          continue;

      if (not trigger.matchInstOpcode(opcode, timing, mode, virtMode, context))
	continue;

      trigger.setLocalHit(true);

      if (updateChainHitBit(trigger))
	hit = true;
    }

  return hit;
}


template <typename URV>
bool
Triggers<URV>::icountTriggerFired(PrivilegeMode mode, bool virtMode, bool interruptEnabled,
                                  const TextraContext<URV>& context)
{
  // Check if we should skip tripping because of reentrant behavior. 
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  bool fire = false;

  for (auto& trig : triggers_)
    {
      if (not trig.matchInstCount(mode, virtMode, context))
        continue;

      if (unsigned(trig.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trig.isEnterDebugOnHit() and skip)
          continue;

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
Triggers<URV>::evaluateIcount(PrivilegeMode mode, bool virtMode, bool interruptEnabled,
                              bool skipModified, const TextraContext<URV>& context)
{
  // Check if we should skip tripping because of reentrant behavior. 
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = mode == PrivilegeMode::Machine and not mmodeEnabled_;

  for (auto& trig : triggers_)
    {
      if (not trig.matchInstCount(mode, virtMode, context))
        continue;

      if (trig.modifiedT1_ and skipModified)
        continue; // Current instruction wrote tdata1: count holds the written value.

      if (unsigned(trig.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trig.isEnterDebugOnHit() and skip)
          continue;

      if (not trig.instCountdown())
        continue;

      assert(trig.data1_.icount_.pending_);
    }
}


template <typename URV>
bool
Triggers<URV>::expTriggerHit(URV cause, PrivilegeMode mode, bool virtMode,
                             PrivilegeMode handlerMode, bool interruptEnabled,
                             const TextraContext<URV>& context, bool& enterDebug)
{
  // Check if we should skip tripping because of reentrant behavior. The trigger fires
  // just before the first instruction of the trap handler, so tcontrol.mte applies when
  // the trap is taken into machine mode.
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = handlerMode == PrivilegeMode::Machine and not mmodeEnabled_;

  URV mask = URV(1) << cause;

  bool fire = false;
  enterDebug = false;

  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;

      if (not trigger.data1_.isEtrigger())
	continue;

      if (not trigger.matchModeAndTextra(trigger.data1_.etrigger_, mode, virtMode, context))
        continue;

      URV data2 = trigger.data2_;
      if (data2 & mask)
	{
	  trigger.setLocalHit(true);
          trigger.setHit(true);
          auto action = trigger.getAction();
          fire = fire or action == TriggerAction::RaiseBreak or
                 action == TriggerAction::EnterDebug;
          enterDebug = enterDebug or action == TriggerAction::EnterDebug;
	}
    }

  return fire;
}


template <typename URV>
bool
Triggers<URV>::intTriggerHit(URV cause, PrivilegeMode mode, bool virtMode,
                             PrivilegeMode handlerMode, bool interruptEnabled,
                             const TextraContext<URV>& context, bool& enterDebug,
                             bool isNmi)
{
  // Check if we should skip tripping because of reentrant behavior. The trigger fires
  // just before the first instruction of the trap handler, so tcontrol.mte applies when
  // the trap is taken into machine mode.
  bool skip = not interruptEnabled;
  if (tcontrolEnabled_)
    skip = handlerMode == PrivilegeMode::Machine and not mmodeEnabled_;

  cause = (cause << 1) >> 1;  // Clear most sig bit.

  URV mask = URV(1) << cause;

  bool fire = false;
  enterDebug = false;

  for (auto& trigger : triggers_)
    {
      if (unsigned(trigger.getAction()) <= unsigned(TriggerAction::EnterDebug))
        if (not trigger.isEnterDebugOnHit() and skip)
          continue;

      if (not trigger.data1_.isItrigger())
	continue;

      auto& itrig = trigger.data1_.itrigger_;

      if (not trigger.matchModeAndTextra(itrig, mode, virtMode, context))
        continue;

      bool selected = isNmi ? itrig.nmi_ : (trigger.data2_ & mask) != 0;
      if (selected)
	{
	  trigger.setLocalHit(true);
          trigger.setHit(true);
          auto action = trigger.getAction();
          fire = fire or action == TriggerAction::RaiseBreak or
                 action == TriggerAction::EnterDebug;
          enterDebug = enterDebug or action == TriggerAction::EnterDebug;
	}
    }

  return fire;
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
    trigger.reset(hyperEnabled_);
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
  trig.pokeData3(v3, hyperEnabled_);

  return true;
}


template <typename URV>
bool
Triggers<URV>::pokeData1(URV trigIx, URV value)
{
  if (trigIx >= triggers_.size())
    return false;

  auto& trig = triggers_.at(trigIx);

  // The chain rules below go by the value the poke leaves in tdata1, as in writeData1.
  Data1Bits<URV> poked{trig.pokedData1(value)};

  // If next trigger has a dmode of 1 (debugger only), then clear
  // chain bit in attempted poke if poke would also set dmode to 0.
  // Otherwise we would have a chain with different dmodes.
  if (poked.isAddrData() and not poked.dmode() and trigIx + 1 < triggers_.size())
    {
      auto& nextTrig = triggers_.at(trigIx + 1);
      if (nextTrig.isDebugModeOnly())
        {
          Data1Bits<URV> bits{value};
          bits.mcontrol_.chain_ = 0;  // Same bit as the chain of mcontrol6.
          value = bits.value_;
        }
    }

  // Poke fails if it would set dmode and previous trigger has
  // both dmode=0 and chain=1. Otherwise, we would have a chain with
  // different dmodes.
  if (poked.dmode() and trigIx > 0)
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

  trig.pokeData3(val, hyperEnabled_);
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
  // Setup read masks. The vs/vu bits are read-only-zero if hypervisor disabled.

  Data1Bits<URV> d1Bits(0);
  d1Bits.mcontrol6_.vs_ = 1;
  d1Bits.mcontrol6_.vu_ = 1;
  URV mask = d1Bits.value_; // Set vs/vu in mask for mcontrol6.

  // Setup mcontrol6.
  URV rmask = data1ReadMasks_.at(unsigned(TriggerType::Mcontrol6));
  rmask = flag? rmask | mask : rmask & ~ mask;
  data1ReadMasks_.at(unsigned(TriggerType::Mcontrol6)) = rmask;

  URV wmask = data1WriteMasks_.at(unsigned(TriggerType::Mcontrol6));
  wmask = flag? wmask | mask : wmask & ~ mask;
  data1WriteMasks_.at(unsigned(TriggerType::Mcontrol6)) = wmask;

  // Setup icount.
  d1Bits.value_ = 0;
  d1Bits.icount_.vs_ = 1;
  d1Bits.icount_.vu_ = 1;
  mask = d1Bits.value_; // Set vs/vu in mask for icount.

  rmask = data1ReadMasks_.at(unsigned(TriggerType::Icount));
  rmask = flag? rmask | mask : rmask & ~ mask;
  data1ReadMasks_.at(unsigned(TriggerType::Icount)) = rmask;

  wmask = data1WriteMasks_.at(unsigned(TriggerType::Icount));
  wmask = flag? wmask | mask : wmask & ~ mask;
  data1WriteMasks_.at(unsigned(TriggerType::Icount)) = wmask;


  // Setup itrigger.
  d1Bits.value_ = 0;
  d1Bits.itrigger_.vs_ = 1;
  d1Bits.itrigger_.vu_ = 1;
  mask = d1Bits.value_; // Set vs/vu in mask for itrigger.

  rmask = data1ReadMasks_.at(unsigned(TriggerType::Itrigger));
  rmask = flag? rmask | mask : rmask & ~ mask;
  data1ReadMasks_.at(unsigned(TriggerType::Itrigger)) = rmask;

  wmask = data1WriteMasks_.at(unsigned(TriggerType::Itrigger));
  wmask = flag? wmask | mask : wmask & ~ mask;
  data1WriteMasks_.at(unsigned(TriggerType::Itrigger)) = wmask;

  // Setup etrigger.
  d1Bits.value_ = 0;
  d1Bits.etrigger_.vs_ = 1;
  d1Bits.etrigger_.vu_ = 1;
  mask = d1Bits.value_; // Set vs/vu in mask for etrigger.

  rmask = data1ReadMasks_.at(unsigned(TriggerType::Etrigger));
  rmask = flag? rmask | mask : rmask & ~ mask;
  data1ReadMasks_.at(unsigned(TriggerType::Etrigger)) = rmask;

  wmask = data1WriteMasks_.at(unsigned(TriggerType::Etrigger));
  wmask = flag? wmask | mask : wmask & ~ mask;
  data1WriteMasks_.at(unsigned(TriggerType::Etrigger)) = wmask;

  hyperEnabled_ = flag;
}


template <typename URV>
void
Triggers<URV>::enableMode(PrivilegeMode mode, bool flag)
{
  bool super = mode == PrivilegeMode::Supervisor;

  Data1Bits<URV> mc(0), ic(0);
  mc.mcontrol6_.s_ = super;      // Same bits in mcontrol.
  mc.mcontrol6_.u_ = not super;
  ic.icount_.s_ = super;         // Same bits in itrigger and etrigger.
  ic.icount_.u_ = not super;

  using TT = TriggerType;
  for (auto type : { TT::Mcontrol, TT::Mcontrol6, TT::Icount, TT::Itrigger, TT::Etrigger })
    {
      URV bits = (type == TT::Mcontrol or type == TT::Mcontrol6) ? mc.value_ : ic.value_;
      for (auto masks : { &data1ReadMasks_, &data1WriteMasks_ })
        {
          URV& mask = masks->at(unsigned(type));
          mask = flag ? mask | bits : mask & ~bits;
        }
    }
}


template <typename URV>
Trigger<URV>::Trigger(URV data1, URV data2, URV /*data3*/,
                      URV mask1, URV mask2, URV mask3)
  : data1_(data1), data2_(data2), data1WriteMask_(mask1),
    data2WriteMask_(mask2), data3WriteMask_(mask3)
{
}


template <typename URV>
template <typename C>
bool
Trigger<URV>::matchModeAndTextra(const C& ctl, PrivilegeMode mode, bool virtMode,
                                 const TextraContext<URV>& context) const
{
  if (mode == PrivilegeMode::Machine and not ctl.m_)
    return false;

  if (mode == PrivilegeMode::Supervisor and not virtMode and not ctl.s_)
    return false;

  if (mode == PrivilegeMode::User and not virtMode and not ctl.u_)
    return false;

  if (mode == PrivilegeMode::Reserved)
    return false;

  if constexpr (not std::is_same_v<C, decltype(data1_.mcontrol_)>)
    {
      if (mode == PrivilegeMode::Supervisor and virtMode and not ctl.vs_)
        return false;

      if (mode == PrivilegeMode::User and virtMode and not ctl.vu_)
        return false;
    }
  else if (virtMode)
    return false;

  return matchTextra(context);
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchLdStAddr(URV addr, unsigned size, TriggerTiming timing, bool isLoad,
                            bool isStore, PrivilegeMode mode, bool virtMode,
                            const TextraContext<URV>& context, URV& hitAddr) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (not matchModeAndTextra(ctl, mode, virtMode, context))
    return false;

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

	    case 4:  // 48-bit: only matches 48-bit instructions, which Whisper does not have
	      return false;

	    case 5:  // double word access
	      if (size != 8)
		return false;
	      break;

	    case 6:  // quad word access
	      if (size != 16)
		return false;
	      break;

	    default:  // Reserved
	      return false;
	    }
	}

      auto match = Match(data1_.mcontrol_.match_);
      if (matchAllDataAddresses(match))
        return doMatchAllAddresses(addr, size, match, hitAddr);

      // Match first ld/st data address.
      auto hit = doMatch(addr, match);
      if (hit)
        hitAddr = addr;
      return hit;
    }

  return false;
}


template <typename URV>
bool
Trigger<URV>::matchLdStAddr(URV addr, unsigned size, TriggerTiming timing, bool isLoad,
                            bool isStore, PrivilegeMode mode, bool virtMode,
                            const TextraContext<URV>& context, URV& hitAddr) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.

  if (data1_.isMcontrol())
    return matchLdStAddr<decltype(data1_.mcontrol_)>(addr, size, timing, isLoad, isStore, mode, virtMode, context, hitAddr);

  return matchLdStAddr<decltype(data1_.mcontrol6_)>(addr, size, timing, isLoad, isStore, mode, virtMode, context, hitAddr);
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchLdStData(URV value, TriggerTiming timing, bool isLoad,
                            PrivilegeMode mode, bool virtMode,
                            const TextraContext<URV>& context) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (not matchModeAndTextra(ctl, mode, virtMode, context))
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
                            PrivilegeMode mode, bool virtMode,
                            const TextraContext<URV>& context) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchLdStData<decltype(data1_.mcontrol_)>(
        value, timing, isLoad, mode, virtMode, context);
  return matchLdStData<decltype(data1_.mcontrol6_)>(
      value, timing, isLoad, mode, virtMode, context);
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

  if (isReservedMatch(match))
    return false;
  if (isNegatedMatch(match))
    return not helper(item, data2, negateNegatedMatch(match));
  return helper(item, data2, match);
}


template <typename URV>
bool
Trigger<URV>::doMatchAllAddresses(URV addr, unsigned size, Match match, URV& hitAddr) const
{
  bool negated = isNegatedMatch(match);
  if (negated)
    match = negateNegatedMatch(match);

  for (unsigned i = 0; i < size; ++i)
    if (doMatch(addr + i, match))
      {
        if (not negated)
          hitAddr = addr + i;
        return not negated;
      }

  if (negated)
    hitAddr = addr;
  return negated;
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchInstAddr(URV addr, unsigned size, TriggerTiming timing, PrivilegeMode mode,
			    bool virtMode, const TextraContext<URV>& context, URV& hitAddr) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (not matchModeAndTextra(ctl, mode, virtMode, context))
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

	    case 4:  // 48-bit: only matches 48-bit instructions, which Whisper does not have
	      return false;

	    case 5:  // double word access
	      if (size != 8)
		return false;
	      break;

	    case 6:  // quad word access
	      if (size != 16)
		return false;
	      break;

	    default:  // Reserved
	      return false;
	    }
	}

      auto match = Match(data1_.mcontrol_.match_);
      if (matchAllInstrAddresses(match))
        return doMatchAllAddresses(addr, size, match, hitAddr);

      // Match first instruction address.
      auto hit = doMatch(addr, match);
      if (hit)
        hitAddr = addr;
      return hit;
    }

  return false;
}


template <typename URV>
bool
Trigger<URV>::matchInstAddr(URV addr, unsigned size, TriggerTiming timing,
                            PrivilegeMode mode, bool virtMode,
                            const TextraContext<URV>& context, URV& hitAddr) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.

  if (data1_.isMcontrol())
    return matchInstAddr<decltype(data1_.mcontrol_)>(addr, size, timing, mode, virtMode, context, hitAddr);

  return matchInstAddr<decltype(data1_.mcontrol6_)>(addr, size, timing, mode, virtMode, context, hitAddr);
}


template <typename URV>
template <typename M>
bool
Trigger<URV>::matchInstOpcode(URV opcode, TriggerTiming timing,
                              PrivilegeMode mode, bool virtMode,
                              const TextraContext<URV>& context) const
{
  const M& ctl = data1_.template mcontrol<M>();

  if (not matchModeAndTextra(ctl, mode, virtMode, context))
    return false;

  if (getTiming() == timing and Select(ctl.select_) == Select::MatchData and ctl.execute_)
    {
      // A non-zero mcontrol6 size only matches instructions of that size: 2 for 16-bit
      // and 3 for 32-bit.
      if constexpr (std::is_same_v<M, decltype(data1_.mcontrol6_)>)
        if (unsigned size = data1_.mcontrol6_.size_; size != 0 and size != ((opcode & 3) == 3 ? 3u : 2u))
          return false;

      auto match = Match(data1_.mcontrol_.match_);
      return doMatch(opcode, match);
    }

  return false;
}


template <typename URV>
bool
Trigger<URV>::matchInstOpcode(URV opcode, TriggerTiming timing,
                              PrivilegeMode mode, bool virtMode,
                              const TextraContext<URV>& context) const
{
  if (not data1_.isAddrData())
    return false;  // Not an address trigger.
  if (data1_.isMcontrol())
    return matchInstOpcode<decltype(data1_.mcontrol_)>(opcode, timing, mode, virtMode, context);
  return matchInstOpcode<decltype(data1_.mcontrol6_)>(opcode, timing, mode, virtMode, context);
}


template class WdRiscv::Trigger<uint32_t>;
template class WdRiscv::Trigger<uint64_t>;

template class WdRiscv::Triggers<uint32_t>;
template class WdRiscv::Triggers<uint64_t>;
