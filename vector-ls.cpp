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

#include <iostream>
#include <cfenv>
#include <cmath>
#include <climits>
#include <limits>
#include <cassert>
#include <optional>
#include "functors.hpp"
#include "wideint.hpp"
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


template <typename URV>
bool
Hart<URV>::isLegalVecLdSt(const DecodedInst* di, ElementWidth eew, GroupMultiplier emul)
{
  if (not preVecExec())
    return false;

  if (not vecRegs_.legalConfig(eew, emul) or not vecRegs_.legalConfig())
    return false;

  // Dest register (vd) cannot overlap mask register v0 and data source (vs3)
  // cannot overlap mask register v0.
  if (di->isMasked() and di->op0() == 0)
    return false;

  // None of the vector source registers can overlap mask regiser v0.
  // This is only applicable to vector indexed ld/st.
  // Section 5.2 of vector spec version 1.1.
  if (di->isMasked())
    {
      for (unsigned i = 1; i < di->operandCount(); ++i)
	if (di->ithOperand(i) == 0 and di->ithOperandType(i) == OperandType::VecReg)
          return false;
    }

  // Use of vstart values greater than vlmax is reserved (section 32.3.7 of spec).
  if (trapOobVstart_ and csRegs_.peekVstart() >= vecRegs_.vlmax(eew, emul))
    return false;

  return true;
}


template <typename URV>
bool
Hart<URV>::checkVecLdStIndexedInst(const DecodedInst* di, unsigned vd, unsigned vi,
                                   unsigned offsetWidth, unsigned offsetGroupX8,
                                   unsigned fieldCount)
{
  if (not isLegalVecLdSt(di, vecRegs_.elemWidth(), vecRegs_.groupMultiplier()))
    {
      postVecFail(di);
      return false;
    }

  unsigned sew = vecRegs_.elemWidthInBits();
  uint32_t groupX8 = vecRegs_.groupMultiplierX8();

  // For segment load: Normalize fractional groups to 1 and account for field count.
  unsigned offsetGroup = offsetGroupX8 >= 8 ? offsetGroupX8/8 : 1;
  unsigned group = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned segGroup = group * fieldCount;

  if (fieldCount > 1)   // If segment load.
    {
      groupX8 = segGroup * 8;
      offsetGroupX8 = offsetGroup * 8;
    }

  bool ok = true;
  if (di->ithOperandMode(0) == OperandMode::Write)
    {
      // From 7.8.3 of spec, for indexed segment loads, no overlap between destination
      // (vd) and source (vi) is allowed.
      if (fieldCount > 1)   // If segment load.
        ok = vi >= vd + segGroup  or  vd >= vi + offsetGroup;
      else
        ok = checkDestSourceOverlap(vd, sew, groupX8, vi, offsetWidth, offsetGroupX8);
    }
  else
    {
      ok = checkSourceOverlap(vd, sew, groupX8, vi, offsetWidth, offsetGroupX8);
    }

  if (not ok)
    postVecFail(di);
  return ok;
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoad(const DecodedInst* di, ElementWidth eew, bool faultFirst)
{
  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * VecRegs::elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier emul = GroupMultiplier::One;
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(groupX8, emul);
  badConfig = badConfig or not isLegalVecLdSt(di, eew, emul);

  if (badConfig)
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1();

  if (not checkVecOpsVsEmul(di, groupX8, {vd}))
    return false;

  unsigned elemSize = sizeof(ELEM_TYPE);
  unsigned elemMax = vecRegs_.elemMax(eew); // Includes tail elems.
  unsigned elemCount = vecRegs_.elemCount();  // Does not include tail elems.
  unsigned start = csRegs_.peekVstart();
  uint64_t addr = intRegs_.read(rs1) + start*elemSize;

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.init(elemCount, elemSize, vd, group, true /*isLoad*/);

  if (start >= elemCount)
    return true;

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = true;

  for (unsigned ix = start; ix < elemMax; ++ix, addr += elemSize)
    {
      ELEM_TYPE elem = 0;
      bool skip = not vecRegs_.isDestActive(vd, ix, groupX8, masked, elem);

      ldStInfo.addElem(VecLdStElem{addr, addr, addr, elem, ix, skip});

      if (skip)
	{
          vecRegs_.write(vd, ix, groupX8, elem);
	  continue;
	}

      auto cause = ExceptionCause::NONE;
      uint64_t pa1 = addr, pa2 = addr; // Phys addrs or faulting virtual address.
      uint64_t gpa1 = addr, gpa2 = addr;

#ifndef FAST_SLOPPY
      bool hyper = false, amo = false;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize, hyper, amo, ix);

      if (hasTrig)
        {
          uint64_t pmva = applyPointerMask(addr, isLd);
          if (ldStAddrTriggerHit(pmva, elemSize, timing, isLd))
            {
              ldStInfo.removeLastElem();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              return false;
            }
        }
#else
      if (faultFirst)
	cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize, false, false, ix);
#endif

      if (cause == ExceptionCause::NONE)
        {
	  uint64_t data = 0;
          if (not readForLoad<ELEM_TYPE>(di, addr, pa1, pa2, data, ix))
	    assert(0 && "Error: Assertion failed");
	  elem = data;
          ldStInfo.setLastElem(pa1, pa2, elem);

#ifndef FAST_SLOPPY
          if (hasTrig)
            {
              ldStDataTriggerHit(elem, timing, isLd);
              if (breakpOrEnterDebugTripped())
                {
                  ldStInfo.removeLastElem();
                  return false;
                }
            }
#endif
        }
      else
        {
	  ldStInfo.removeLastElem();
          markVsDirty();
	  if (faultFirst)
	    {
	      if (ix == 0)
                initiateLoadException(di, cause, ldStFaultAddr_, gpa1);
	      else
                {
		  // We reduce VL before processing tail elements. This is allowed
		  // by the spec (items between old VL and new VL can be updated
		  // with arbitrary values according to the spec).
                  pokeCsr(CsrNumber::VL, ix);
                  recordCsrWrite(CsrNumber::VL);
                  vecRegs_.elemCount(ix);  // Update cached value of VL.

		  // Fill tail elements with all-ones if so configured.
		  ELEM_TYPE ones = ~ ELEM_TYPE{0};
		  if (vecRegs_.isTailAgnostic() and vecRegs_.isTailAgnosticOnes())
		    for (unsigned ti = vecRegs_.elemCount(); ti < elemMax; ti++)
		      vecRegs_.write(vd, ti, groupX8, ones);
                  return true;
                }
	    }
	  else
	    {
	      csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	      initiateLoadException(di, cause, ldStFaultAddr_, gpa1);
	    }
          return false;
        }

      vecRegs_.write(vd, ix, groupX8, elem);
    }
  return true;
}


template <typename URV>
void
Hart<URV>::execVle8_v(const DecodedInst* di)
{
  if (not vectorLoad<uint8_t>(di, ElementWidth::Byte, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle16_v(const DecodedInst* di)
{
  if (not vectorLoad<uint16_t>(di, ElementWidth::Half, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle32_v(const DecodedInst* di)
{
  if (not vectorLoad<uint32_t>(di, ElementWidth::Word, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle64_v(const DecodedInst* di)
{
  if (not vectorLoad<uint64_t>(di, ElementWidth::Word2, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStore(const DecodedInst* di, ElementWidth eew)
{
  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * VecRegs::elemWidthInBits(eew) / vecRegs_.elemWidthInBits();

  GroupMultiplier emul = GroupMultiplier::One;
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(groupX8, emul);
  badConfig = badConfig or not isLegalVecLdSt(di, eew, emul);

  if (badConfig)
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1();

  if (not checkVecOpsVsEmul(di, groupX8, {vd}))
    return false;

  unsigned elemCount = vecRegs_.elemCount();
  unsigned elemSize = sizeof(ELEM_TYPE);
  unsigned start = csRegs_.peekVstart();
  uint64_t addr = intRegs_.read(rs1) + start*elemSize;

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.init(elemCount, elemSize, vd, group, false /*isLoad*/);

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.

  for (unsigned ix = start; ix < elemCount; ++ix, addr += elemSize)
    {
      bool skip = masked and not vecRegs_.isActive(0, ix);

      ldStInfo.addElem(VecLdStElem{addr, addr, addr, 0, ix, skip});

      if (skip)
	continue;

      ELEM_TYPE elem = 0;
      vecRegs_.read(vd, ix, groupX8, elem);

      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;

      auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, elemSize, false /*hyper*/);

      if (hasTrig)
        {
          uint64_t pmva = applyPointerMask(addr, isLd);
          ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
          ldStDataTriggerHit(elem, timing, isLd);
        }

      if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
	{
	  if (not writeForStore(addr, pa1, pa2, elem))
	    assert(0 && "Error: Assertion failed");
	  ldStInfo.setLastElem(pa1, pa2, elem);
	}
      else
        {
	  ldStInfo.removeLastElem();
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	  if (not breakpOrEnterDebugTripped())
	    initiateStoreException(di, cause, ldStFaultAddr_, gpa1);
          return false;
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVse8_v(const DecodedInst* di)
{
  if (not vectorStore<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVse16_v(const DecodedInst* di)
{
  if (not vectorStore<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVse32_v(const DecodedInst* di)
{
  if (not vectorStore<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVse64_v(const DecodedInst* di)
{
  if (not vectorStore<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVse128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVse256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVse512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVse1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlm_v(const DecodedInst* di)
{
  if (not preVecExec() or not vecRegs_.legalConfig() or di->isMasked())
    {
      postVecFail(di);
      return;
    }

  // Record element count, element width, group multiplier, and tail-agnostic
  // setting.
  ElementWidth ew = vecRegs_.elemWidth();
  uint32_t elems = vecRegs_.elemCount();
  GroupMultiplier gm = vecRegs_.groupMultiplier();
  bool tailAgnostic = vecRegs_.isTailAgnostic();

  // Change element count to byte count, element width to byte, and emul to 1.
  // Change tail-agnostic to true.
  uint32_t bytes = (elems + 7) / 8;
  vecRegs_.elemCount(bytes);
  vecRegs_.elemWidth(ElementWidth::Byte);
  vecRegs_.groupMultiplier(GroupMultiplier::One);
  vecRegs_.setTailAgnostic(true);

  // Do load bytes.
  bool ok = vectorLoad<uint8_t>(di, ElementWidth::Byte, false);

  // Restore elem count, elem width, emul, and tail-agnostic.
  vecRegs_.elemCount(elems);
  vecRegs_.elemWidth(ew);
  vecRegs_.groupMultiplier(gm);
  vecRegs_.setTailAgnostic(tailAgnostic);

  if (not ok)
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsm_v(const DecodedInst* di)
{
  if (not preVecExec() or not vecRegs_.legalConfig() or di->isMasked())
    {
      postVecFail(di);
      return;
    }

  // Record element count, element width, group multiplier, and tail-agnostic
  // setting.
  ElementWidth ew = vecRegs_.elemWidth();
  uint32_t elems = vecRegs_.elemCount();
  GroupMultiplier gm = vecRegs_.groupMultiplier();
  bool tailAgnostic = vecRegs_.isTailAgnostic();

  // Change element count to byte count, element width to byte, and emul to 1.
  // Change tail-agnostic to true.
  uint32_t bytes = (elems + 7) / 8;
  vecRegs_.elemCount(bytes);
  vecRegs_.elemWidth(ElementWidth::Byte);
  vecRegs_.groupMultiplier(GroupMultiplier::One);
  vecRegs_.setTailAgnostic(true);

  // Do store bytes.
  bool ok = vectorStore<uint8_t>(di, ElementWidth::Byte);

  // Restore elem count, elem width, emul, and tail-agnostic.
  vecRegs_.elemCount(elems);
  vecRegs_.elemWidth(ew);
  vecRegs_.groupMultiplier(gm);
  vecRegs_.setTailAgnostic(tailAgnostic);

  if (not ok)
    return;
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadWholeReg(const DecodedInst* di, ElementWidth eew)
{
  unsigned start = csRegs_.peekVstart();
  unsigned fieldCount = di->vecFieldCount();
  unsigned group = 1, groupX8 = 8, effGroupX8 = fieldCount*8;

  // Field count must be a power of 2.
  bool ok = (fieldCount & (fieldCount - 1)) == 0;
  if (ok)
    {
      GroupMultiplier egm = GroupMultiplier::One;
      ok = VecRegs::groupNumberX8ToSymbol(effGroupX8, egm);
      ok = ok and preVecExec() and vecRegs_.legalConfig(eew, egm) and not di->isMasked();
    }

  if (not ok)
    {
      postVecFail(di);
      return false;
    }

  unsigned vd = di->op0(), rs1 = di->op1();

  // This should never fail, we call it to record the operand (vd) effective group.
  if (not checkVecOpsVsEmul(di, effGroupX8, {vd}))
    return false;

  unsigned elemBytes = VecRegs::elemWidthInBytes(eew);
  assert(elemBytes == sizeof(ELEM_TYPE));
  unsigned elemCount = (group*vecRegs_.bytesPerRegister()*fieldCount) / elemBytes;
  URV addr = intRegs_.read(rs1) + start*elemBytes;

  // We don't set the field count for whole register load, we scale group instead.
  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.init(elemCount, elemBytes, vd, group*fieldCount, true /*isLoad*/);

  if (start >= elemCount)
    return true;

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = true;

  bool result = true;

  for (unsigned ix = start; ix < elemCount; ++ix, addr += elemBytes)
    {
      auto cause = ExceptionCause::NONE;
      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr;

#ifndef FAST_SLOPPY
      uint64_t gpa2 = addr;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, sizeof(ELEM_TYPE), false,
                                     false, ix);

      if (hasTrig)
	{
          uint64_t pmva = applyPointerMask(addr, isLd);
          if (ldStAddrTriggerHit(pmva, elemBytes, timing, isLd))
            {
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              result = false;
              break;
            }
        }
#endif

      if (cause == ExceptionCause::NONE)
	{
	  uint64_t data = 0;
	  if (not readForLoad<ELEM_TYPE>(di, addr, pa1, pa2, data, ix))
	    assert(0 && "Error: Assertion failed");
	  ELEM_TYPE elem = data;

#ifndef FAST_SLOPPY
          if (hasTrig)
            {
              ldStDataTriggerHit(elem, timing, isLd);
              if (breakpOrEnterDebugTripped())
                {
                  markVsDirty();
                  csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
                  result = false;
                  break;
                }
            }
#endif
          ldStInfo.addElem(VecLdStElem{addr, pa1, pa2, elem, ix, false /*skip*/});
          vecRegs_.write(vd, ix, effGroupX8, elem);
	}
      else
        {
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateLoadException(di, cause, ldStFaultAddr_, gpa1);
          result = false;
          break;
        }
    }

  vecRegs_.touchReg(vd, groupX8);  // We want the group and not the effective group.

  return result;
}


template <typename URV>
void
Hart<URV>::execVlre8_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlre16_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlre32_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlre64_v(const DecodedInst* di)
{
  if (not vectorLoadWholeReg<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlre128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlre256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlre512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlre1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
bool
Hart<URV>::vectorStoreWholeReg(const DecodedInst* di)
{
  unsigned start = csRegs_.peekVstart();
  unsigned fieldCount = di->vecFieldCount();
  unsigned group = 1, effGroupX8 = fieldCount*8;

  // Field count must be a multiple of 8.
  bool ok = (fieldCount & (fieldCount - 1)) == 0;
  if (ok)
    {
      GroupMultiplier egm = GroupMultiplier::One;
      ElementWidth eew = ElementWidth::Byte;
      ok = VecRegs::groupNumberX8ToSymbol(effGroupX8, egm);
      ok = ok and preVecExec() and vecRegs_.legalConfig(eew, egm) and not di->isMasked();
    }

  if (not ok)
    {
      postVecFail(di);
      return false;
    }

  unsigned vd = di->op0(), rs1 = di->op1();

  // This should never fail, we call it to record the operand (vd) group.
  if (not checkVecOpsVsEmul(di, effGroupX8, {vd}))
    return false;

  const unsigned elemBytes = 1;
  unsigned elemCount = (group*vecRegs_.bytesPerRegister()*fieldCount) / elemBytes;
  URV addr = intRegs_.read(rs1) + start*elemBytes;

  // We don't set the field count for whole register store, we scale group instead.
  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.init(elemCount, elemBytes, vd, group*fieldCount, false /*isLoad*/);

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.

  for (unsigned ix = start; ix < elemCount; ++ix, addr += elemBytes)
    {
      uint8_t val = 0; // Element value.
      vecRegs_.read(vd, ix, effGroupX8, val);

      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;
      auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, elemBytes, false /*hyper*/);

      if (hasTrig)
        {
          uint64_t pmva = applyPointerMask(addr, isLd);
          ldStAddrTriggerHit(pmva, elemBytes, timing, isLd);
          ldStDataTriggerHit(val, timing, isLd);
        }

      if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
	{
	  if (not writeForStore(addr, pa1, pa2, val))
	    assert(0 && "Error: Assertion failed");
	  bool skip = false; // Not masked off.
	  ldStInfo.addElem(VecLdStElem{addr, pa1, pa2, val, ix, skip});
	}
      else
        {
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	  if (not breakpOrEnterDebugTripped())
	    initiateStoreException(di, cause, ldStFaultAddr_, gpa1);
          return false;
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVs1r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVs2r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVs4r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVs8r_v(const DecodedInst* di)
{
  if (not vectorStoreWholeReg(di))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle8ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint8_t>(di, ElementWidth::Byte, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle16ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint16_t>(di, ElementWidth::Half, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle32ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint32_t>(di, ElementWidth::Word, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle64ff_v(const DecodedInst* di)
{
  if (not vectorLoad<uint64_t>(di, ElementWidth::Word2, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVle128ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle256ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle512ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVle1024ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadStrided(const DecodedInst* di, ElementWidth eew)
{
  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * VecRegs::elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier emul = GroupMultiplier::One;
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(groupX8, emul);
  badConfig = badConfig or not isLegalVecLdSt(di, eew, emul);

  if (badConfig)
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1(), rs2 = di->op2();

  if (not checkVecOpsVsEmul(di, groupX8, {vd}))
    return false;

  uint64_t stride = intRegs_.read(rs2);
  unsigned start = csRegs_.peekVstart();
  unsigned elemMax = vecRegs_.elemMax(eew);   // Includes tail elements.
  unsigned elemCount = vecRegs_.elemCount();  // Does not include tail elements.
  uint64_t addr = intRegs_.read(rs1) + start*stride;

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  unsigned elemSize = sizeof(ELEM_TYPE);
  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.initStrided(elemCount, elemSize, vd, group, stride, true /*isLoad*/);

  if (start >= elemCount)
    return true;

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = true;

  for (unsigned ix = start; ix < elemMax; ++ix, addr += stride)
    {
      ELEM_TYPE elem = 0;
      bool skip = not vecRegs_.isDestActive(vd, ix, groupX8, masked, elem);
      ldStInfo.addElem(VecLdStElem{addr, addr, addr, elem, ix, skip});

      if (skip)
	{
	  vecRegs_.write(vd, ix, groupX8, elem);
	  continue;
	}

      auto cause = ExceptionCause::NONE;
      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr;

#ifndef FAST_SLOPPY
      uint64_t gpa2 = addr;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, sizeof(elem), false, false, ix);

      if (hasTrig)
	{
          uint64_t pmva = applyPointerMask(addr, isLd);
          if (ldStAddrTriggerHit(pmva, elemSize, timing, isLd))
            {
              ldStInfo.removeLastElem();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              return false;
            }
	}
#endif

      if (cause == ExceptionCause::NONE)
        {
	  uint64_t data = 0;
	  if (not readForLoad<ELEM_TYPE>(di, addr, pa1, pa2, data, ix))
	    assert(0 && "Error: Assertion failed");
	  elem = data;
          ldStInfo.setLastElem(pa1, pa2, elem);
        }
      else
        {
	  ldStInfo.removeLastElem();
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateLoadException(di, cause, ldStFaultAddr_, gpa1);
          return false;
        }

      vecRegs_.write(vd, ix, groupX8, elem);
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVlse8_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlse16_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlse32_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlse64_v(const DecodedInst* di)
{
  if (not vectorLoadStrided<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlse128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlse256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlse512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlse1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreStrided(const DecodedInst* di, ElementWidth eew)
{
  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * VecRegs::elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier emul = GroupMultiplier::One;
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(groupX8, emul);
  badConfig = badConfig or not isLegalVecLdSt(di, eew, emul);

  if (badConfig)
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1(), rs2 = di->op2();

  if (not checkVecOpsVsEmul(di, groupX8, {vd}))
    return false;

  uint64_t stride = intRegs_.read(rs2);
  unsigned elemCount = vecRegs_.elemCount();
  unsigned elemSize = sizeof(ELEM_TYPE);
  unsigned start = csRegs_.peekVstart();
  uint64_t addr = intRegs_.read(rs1) + start*stride;

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.initStrided(elemCount, elemSize, vd, group, stride, false /*isLoad*/);

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.

  for (unsigned ix = start; ix < elemCount; ++ix, addr += stride)
    {
      bool skip = masked and not vecRegs_.isActive(0, ix);
      ldStInfo.addElem(VecLdStElem{addr, addr, addr, 0, ix, skip});

      if (skip)
	continue;

      ELEM_TYPE val = 0;
      vecRegs_.read(vd, ix, groupX8, val);

      uint64_t pa1 = addr, pa2 = addr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = addr, gpa2 = addr;

      auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, elemSize, false /*hyper*/);

      if (hasTrig)
        {
          uint64_t pmva = applyPointerMask(addr, isLd);
          ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
          ldStDataTriggerHit(val, timing, isLd);
        }

      if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
	{
	  if (not writeForStore(addr, pa1, pa2, val))
	    assert(0 && "Error: Assertion failed");
	  ldStInfo.setLastElem(pa1, pa2, val);
	}
      else
        {
	  ldStInfo.removeLastElem();
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	  if (not breakpOrEnterDebugTripped())
	    initiateStoreException(di, cause, ldStFaultAddr_, gpa1);
          return false;
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVsse8_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint8_t>(di, ElementWidth::Byte))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsse16_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint16_t>(di, ElementWidth::Half))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsse32_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint32_t>(di, ElementWidth::Word))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsse64_v(const DecodedInst* di)
{
  if (not vectorStoreStrided<uint64_t>(di, ElementWidth::Word2))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsse128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsse256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsse512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsse1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadIndexed(const DecodedInst* di, ElementWidth offsetEew)
{
  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = VecRegs::elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);

  if (not preVecExec() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkIndexedOpsVsEmul(di, vd, vi, groupX8, offsetGroupX8))
    return false;

  if (not checkVecLdStIndexedInst(di, vd, vi, offsetWidth, offsetGroupX8, 1 /* fieldCount */))
    return false;

  uint64_t addr = intRegs_.read(rs1);

  unsigned start = csRegs_.peekVstart();
  unsigned elemMax = vecRegs_.elemMax();  // Includes tail elements.
  unsigned elemCount = vecRegs_.elemCount();  // Does not include tail elements.
  unsigned elemSize = elemWidth / 8;

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  // Effective index reg group. If group is fractional, snap to 1.
  offsetGroupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), offsetGroupX8);
  unsigned ixGroup = offsetGroupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.initIndexed(elemCount, elemSize, vd, vi, group, ixGroup, true /*isLoad*/);

  if (start >= elemCount)
    return true;

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = true;

  for (unsigned ix = start; ix < elemMax; ++ix)
    {
      uint64_t vaddr = 0;
      ELEM_TYPE elem = 0;
      bool skip = not vecRegs_.isDestActive(vd, ix, groupX8, masked, elem);
      if (ix < vecRegs_.elemCount())
        {
          uint64_t offset = vecRegs_.readIndexReg(vi, ix, offsetEew, offsetGroupX8);
          vaddr = addr + offset;
        }

      ldStInfo.addElem(VecLdStElem{vaddr, vaddr, vaddr, elem, ix, skip});

      if (skip)
	{
          vecRegs_.write(vd, ix, groupX8, elem);
	  continue;
	}

      uint64_t pa1 = vaddr, pa2 = vaddr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = vaddr;

      auto cause = ExceptionCause::NONE;

#ifndef FAST_SLOPPY
      uint64_t gpa2 = vaddr;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize, false, false, ix);

      if (hasTrig)
        {
          uint64_t pmva = applyPointerMask(vaddr, isLd);
          if (ldStAddrTriggerHit(pmva, elemSize, timing, isLd))
            {
              ldStInfo.removeLastElem();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              return false;
            }
        }
#endif

      if (cause == ExceptionCause::NONE)
	{
	  uint64_t data = 0;
	  if (not readForLoad<ELEM_TYPE>(di, vaddr, pa1, pa2, data, ix))
	    assert(0 && "Error: Assertion failed");
	  elem = data;
          ldStInfo.setLastElem(pa1, pa2, elem);
	  vecRegs_.write(vd, ix, groupX8, elem);
	}
      else
        {
	  ldStInfo.removeLastElem();
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateLoadException(di, cause, ldStFaultAddr_, gpa1);
          return false;
        }
    }
  return true;
}


template <typename URV>
void
Hart<URV>::execVloxei8_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Byte); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Byte); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Byte); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Byte); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxei16_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Half); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Half); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Half); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Half); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxei32_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Word); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Word); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Word); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Word); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxei64_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadIndexed<uint8_t>(di,  EW::Word2); break;
    case EW::Half:   ok = vectorLoadIndexed<uint16_t>(di, EW::Word2); break;
    case EW::Word:   ok = vectorLoadIndexed<uint32_t>(di, EW::Word2); break;
    case EW::Word2:  ok = vectorLoadIndexed<uint64_t>(di, EW::Word2); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxei8_v(const DecodedInst* di)
{
  execVloxei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxei16_v(const DecodedInst* di)
{
  execVloxei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxei32_v(const DecodedInst* di)
{
  execVloxei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxei64_v(const DecodedInst* di)
{
  execVloxei64_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreIndexed(const DecodedInst* di, ElementWidth offsetEew)
{
  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = VecRegs::elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);

  if (not preVecExec() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkIndexedOpsVsEmul(di, vd, vi, groupX8, offsetGroupX8))
    return false;

  if (not checkVecLdStIndexedInst(di, vd, vi, offsetWidth, offsetGroupX8, 1 /* fieldCount */))
    return false;

  uint64_t addr = intRegs_.read(rs1);
  unsigned start = csRegs_.peekVstart();
  unsigned elemCount = vecRegs_.elemCount(), elemSize = elemWidth / 8;

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  // Effective index reg group. If group is fractional, snap to 1.
  offsetGroupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), offsetGroupX8);
  unsigned ixGroup = offsetGroupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.initIndexed(elemCount, elemSize, vd, vi, group, ixGroup, false /*isLoad*/);

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.

  for (unsigned ix = start; ix < elemCount; ++ix)
    {
      uint64_t offset = vecRegs_.readIndexReg(vi, ix, offsetEew, offsetGroupX8);

      uint64_t vaddr = addr + offset, data = 0;
      bool skip = masked and not vecRegs_.isActive(0, ix);
      ldStInfo.addElem(VecLdStElem{vaddr, vaddr, vaddr, 0, ix, skip});
      if (skip)
	continue;

      uint64_t pa1 = vaddr, pa2 = vaddr; // Physical addresses or faulting virtual addresses.
      uint64_t gpa1 = vaddr, gpa2 = vaddr;

      auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, elemSize, false /*hyper*/);

      if (elemSize == 1)
	{
	  uint8_t x = 0;
	  vecRegs_.read(vd, ix, groupX8, x);

	  if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(vaddr, isLd);
              ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
              ldStDataTriggerHit(x, timing, isLd);
            }

	  if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
	    if (not writeForStore(vaddr, pa1, pa2, x))
	      assert(0 && "Error: Assertion failed");
	  data = x;
	}
      else if (elemSize == 2)
	{
	  uint16_t x = 0;
	  vecRegs_.read(vd, ix, groupX8, x);

	  if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(vaddr, isLd);
              ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
              ldStDataTriggerHit(x, timing, isLd);
            }

	  if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
	    if (not writeForStore(vaddr, pa1, pa2, x))
	      assert(0 && "Error: Assertion failed");
	  data = x;
	}
      else if (elemSize == 4)
	{
	  uint32_t x = 0;
	  vecRegs_.read(vd, ix, groupX8, x);

	  if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(vaddr, isLd);
              ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
              ldStDataTriggerHit(x, timing, isLd);
            }

	  if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
	    if (not writeForStore(vaddr, pa1, pa2, x))
	      assert(0 && "Error: Assertion failed");
	  data = x;
	}
      else if (elemSize == 8)
	{
	  uint64_t x = 0;
	  vecRegs_.read(vd, ix, groupX8, x);

	  if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(vaddr, isLd);
              ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
              ldStDataTriggerHit(x, timing, isLd);
            }

	  if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
	    if (not writeForStore(vaddr, pa1, pa2, x))
	      assert(0 && "Error: Assertion failed");
	  data = x;
	}
      else
	assert(0 && "Error: Assertion failed");

      if (cause != ExceptionCause::NONE or breakpOrEnterDebugTripped())
        {
	  ldStInfo.removeLastElem();
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
	  if (not breakpOrEnterDebugTripped())
	    initiateStoreException(di, cause, ldStFaultAddr_, gpa1);
          return false;
        }

      ldStInfo.setLastElem(pa1, pa2, data);
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVsoxei8_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Byte); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Byte); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Byte); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Byte); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxei16_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Half); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Half); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Half); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Half); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxei32_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Word); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Word); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Word); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Word); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxei64_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreIndexed<uint8_t>(di,  EW::Word2); break;
    case EW::Half:   ok = vectorStoreIndexed<uint16_t>(di, EW::Word2); break;
    case EW::Word:   ok = vectorStoreIndexed<uint32_t>(di, EW::Word2); break;
    case EW::Word2:  ok = vectorStoreIndexed<uint64_t>(di, EW::Word2); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei8_v(const DecodedInst* di)
{
  execVsoxei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei16_v(const DecodedInst* di)
{
  execVsoxei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei32_v(const DecodedInst* di)
{
  execVsoxei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei64_v(const DecodedInst* di)
{
  execVsoxei64_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadSeg(const DecodedInst* di, ElementWidth eew,
			 unsigned fieldCount, uint64_t stride, bool faultFirst)
{
  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * VecRegs::elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier emul = GroupMultiplier::One;
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(groupX8, emul);
  badConfig = badConfig or not isLegalVecLdSt(di, eew, emul);
  badConfig = badConfig or (groupX8*fieldCount > 64);

  if (badConfig)
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1();

  if (not checkVecOpsVsEmul(di, groupX8, {vd}))
    return false;

  unsigned start = csRegs_.peekVstart();
  uint64_t addr = intRegs_.read(rs1) + start*stride;
  unsigned elemMax = vecRegs_.elemMax(eew);  // Includes tail elements.
  unsigned elemCount = vecRegs_.elemCount();  // Does not include tail elements.

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  // Used registers must not exceed register count (32).
  if (vd + fieldCount*group > vecRegs_.registerCount())
    {
      postVecFail(di);
      return false;
    }

  unsigned elemSize = sizeof(ELEM_TYPE);

  auto& ldStInfo = vecRegs_.ldStInfo_;
  
  if (di->isVectorLoadStrided())
    ldStInfo.initStrided(elemCount, elemSize, vd, group, stride, true /*isLoad*/);
  else
    ldStInfo.init(elemCount, elemSize, vd, group, true /*isLoad*/);
  ldStInfo.setFieldCount(fieldCount, true /*isSeg*/);

  if (start >= elemCount)
    return true;

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = true;

  for (unsigned ix = start; ix < elemMax; ++ix, addr += stride)
    {
      uint64_t faddr = addr;  // Field address

      for (unsigned field = 0; field < fieldCount; ++field, faddr += elemSize)
	{
	  unsigned fdv = vd + field*group;   // Field destination register
	  ELEM_TYPE elem(0);
	  bool skip = not vecRegs_.isDestActive(fdv, ix, groupX8, masked, elem);

	  ldStInfo.addElem(VecLdStElem{faddr, faddr, faddr, elem, ix, skip, field});

	  if (skip)
	    {
              if (vecRegs_.partialSegUpdate_)
                vecRegs_.write(fdv, ix, groupX8, elem);
	      continue;
	    }

	  uint64_t pa1 = faddr, pa2 = faddr; // Physical addrs or faulting virtual addrs.
          uint64_t gpa1 = faddr;

	  auto cause = ExceptionCause::NONE;

#ifndef FAST_SLOPPY
	  uint64_t gpa2 = faddr;
          cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize, false, false, ix);

	  if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(faddr, isLd);
              if (ldStAddrTriggerHit(pmva, elemSize, timing, isLd))
                {
                  ldStInfo.removeLastElem();
                  if (not vecRegs_.partialSegUpdate_)
                    while (not ldStInfo.elems_.empty() and ldStInfo.elems_.back().ix_ == ix)
                      ldStInfo.removeLastElem();
                  markVsDirty();
                  csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
                  return false;
                }
            }
#endif

	  if (cause == ExceptionCause::NONE)
            {
	      uint64_t data = 0;
	      if (not readForLoad<ELEM_TYPE>(di, faddr, pa1, pa2, data, ix, field))
		assert(0 && "Error: Assertion failed");
	      elem = data;
              ldStInfo.setLastElem(pa1, pa2, elem);
            }
	  else
	    {
	      ldStInfo.removeLastElem();
              if (not vecRegs_.partialSegUpdate_)
                while (not ldStInfo.elems_.empty() and ldStInfo.elems_.back().ix_ == ix)
                  ldStInfo.removeLastElem();
              markVsDirty();
	      if (ix == 0 or not faultFirst)
                {
                  csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
                  initiateLoadException(di, cause, ldStFaultAddr_, gpa1);
                  return false;
                }
	      if (vecRegs_.isTailAgnostic() and vecRegs_.isTailAgnosticOnes())
		{
		  // We reduce VL before processing tail elements. This is allowed
		  // by the spec (items between old VL and new VL can be updated
		  // with arbitrary values according to the spec).
                  pokeCsr(CsrNumber::VL, ix);
                  recordCsrWrite(CsrNumber::VL);
                  vecRegs_.elemCount(ix);  // Update cached value of VL.

		  // Fill tail elements with all-ones if so configured.
		  ELEM_TYPE ones = ~ ELEM_TYPE{0};
		  for (unsigned ti = vecRegs_.elemCount(); ti < elemMax; ti++)
		    for (unsigned fi = 0; fi < fieldCount; ++fi)
		      {
			unsigned fdv = vd + fi*group;   // Field destination vector.
			vecRegs_.write(fdv, ti, groupX8, ones);
		      }
		}
	      return true;
	    }

          if (vecRegs_.partialSegUpdate_)
            vecRegs_.write(fdv, ix, groupX8, elem);
	}

      // If we get here then no excpections were encoutered. Commit all the fields if
      // partial update is not on.
      if (not vecRegs_.partialSegUpdate_)
        {
          unsigned nelems = ldStInfo.elems_.size();
          assert(nelems >= fieldCount);
          for (unsigned field = 0; field < fieldCount; ++field)
            {
              const auto& elem = ldStInfo.elems_.at(nelems - fieldCount + field);
              unsigned fdv = vd + field*group;   // Field destination vector.
              vecRegs_.write(fdv, ix, groupX8, ELEM_TYPE(elem.data_));
            }
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVlsege8_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint8_t);
  if (not vectorLoadSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege16_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint16_t);
  if (not vectorLoadSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege32_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint32_t);
  if (not vectorLoadSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege64_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint64_t);
  if (not vectorLoadSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreSeg(const DecodedInst* di, ElementWidth eew,
			  unsigned fieldCount, uint64_t stride)
{
  // Compute emul: lmul*eew/sew
  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  groupX8 = groupX8 * VecRegs::elemWidthInBits(eew) / vecRegs_.elemWidthInBits();
  GroupMultiplier emul = GroupMultiplier::One;
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(groupX8, emul);
  badConfig = badConfig or not isLegalVecLdSt(di, eew, emul);

  // emul*fieldcount cannot be larger than 8 registers.
  badConfig = badConfig or (groupX8*fieldCount > 64);

  if (badConfig)
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(), rs1 = di->op1();

  if (not checkVecOpsVsEmul(di, groupX8, {vd}))
    return false;

  unsigned start = csRegs_.peekVstart();
  uint64_t addr = intRegs_.read(rs1) + start*stride;
  unsigned elemCount = vecRegs_.elemCount(), elemSize = sizeof(ELEM_TYPE);
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;

  // Used registers must not exceed 32.
  if (vd + fieldCount*eg > 32)
    {
      postVecFail(di);
      return false;
    }

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  if (di->isVectorStoreStrided())
    ldStInfo.initStrided(elemCount, elemSize, vd, group, stride, false /*isLoad*/);
  else
    ldStInfo.init(elemCount, elemSize, vd, group, false /*isLoad*/);
  ldStInfo.setFieldCount(fieldCount, true /*isSeg*/);

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.

  for (unsigned ix = start; ix < elemCount; ++ix, addr += stride)
    {
      uint64_t faddr = addr;   // Field address

      for (unsigned field = 0; field < fieldCount; ++field, faddr += elemSize)
	{
	  uint64_t pa1 = faddr, pa2 = faddr; // Physical addrs or faulting virtual addrs.
          uint64_t gpa1 = faddr, gpa2 = faddr;
	  unsigned dvg = vd + field*eg;   // Source vector group.

	  bool skip = masked and not vecRegs_.isActive(0, ix);
	  ldStInfo.addElem(VecLdStElem{faddr, faddr, faddr, 0, ix, skip, field});
	  if (skip)
	    continue;

	  ELEM_TYPE val = 0;
	  vecRegs_.read(dvg, ix, groupX8, val);

          auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, elemSize, false /*hyper*/);

          if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(faddr, isLd);
              ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
              ldStDataTriggerHit(val, timing, isLd);
            }

          if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
            {
              if (vecRegs_.partialSegUpdate_)
                if (not writeForStore(faddr, pa1, pa2, val))
                  assert(0 && "Error: Assertion failed");
              ldStInfo.setLastElem(pa1, pa2, val);
            }
          else
            {
              ldStInfo.removeLastElem();
              if (not vecRegs_.partialSegUpdate_)
                while (not ldStInfo.elems_.empty() and ldStInfo.elems_.back().ix_ == ix)
                  ldStInfo.removeLastElem();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              if (not breakpOrEnterDebugTripped())
                initiateStoreException(di, cause, ldStFaultAddr_, gpa1);
              return false;
            }
        }

      // If we get here, no exception was encoutered, update all the fields if not in
      // partial-update.
      if (not vecRegs_.partialSegUpdate_)
        {
          for (const auto& elem : ldStInfo.elems_)
            {
              if (elem.skip_)
                continue;

              auto val = ELEM_TYPE(elem.data_);
              if (not writeForStore(elem.va_, elem.pa_, elem.pa2_, val))
                assert(0 && "Error: Assertion failed");
            }
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVssege8_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint8_t);
  if (not vectorStoreSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssege16_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint16_t);
  if (not vectorStoreSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssege32_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint32_t);
  if (not vectorStoreSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssege64_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint64_t);
  if (not vectorStoreSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVssege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVssege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVssege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege8_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlssege16_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlssege32_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlssege64_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorLoadSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride, false))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlssege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlssege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege8_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsssege16_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsssege32_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsssege64_v(const DecodedInst* di)
{
  uint64_t stride = intRegs_.read(di->op2());
  unsigned fieldCount = di->vecFieldCount();
  if (not vectorStoreSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsssege128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsssege1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorLoadSegIndexed(const DecodedInst* di, ElementWidth offsetEew,
                                unsigned fieldCount)
{
  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = VecRegs::elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);
  badConfig = badConfig or (groupX8*fieldCount > 64);

  if (not preVecExec() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkIndexedOpsVsEmul(di, vd, vi, groupX8, offsetGroupX8))
    return false;

  if (not checkVecLdStIndexedInst(di, vd, vi, offsetWidth, offsetGroupX8, fieldCount))
    return false;

  uint64_t addr = intRegs_.read(rs1);
  unsigned start = csRegs_.peekVstart(), elemSize = elemWidth / 8;
  unsigned elemMax = vecRegs_.elemMax();  // Includes tail elements.
  unsigned elemCount = vecRegs_.elemCount();  // Does not include tail elements.

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  // Used registers must not exceed register count (32).
  if (vd + fieldCount*group > vecRegs_.registerCount())
    {
      postVecFail(di);
      return false;
    }

  // Effective index reg group. If group is fractional, snap to 1.
  offsetGroupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), offsetGroupX8);
  unsigned ixGroup = offsetGroupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.initIndexed(elemCount, elemSize, vd, vi, group, ixGroup, true /*isLoad*/);
  ldStInfo.setFieldCount(fieldCount, true /*isSeg*/);

  if (start >= elemCount)
    return true;

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = true;

  for (unsigned ix = start; ix < elemMax; ++ix)
    {
      for (unsigned field = 0; field < fieldCount; ++field)
        {
          uint64_t faddr = 0;
          unsigned fdv = vd + (uint64_t(field) * group);  // Field destination register
          ELEM_TYPE elem(0);
          bool skip = not vecRegs_.isDestActive(fdv, ix, groupX8, masked, elem);
          if (ix < vecRegs_.elemCount())
            {
              uint64_t offset = vecRegs_.readIndexReg(vi, ix, offsetEew, offsetGroupX8);
              faddr = addr + offset + (uint64_t(field) * elemSize);
            }

          ldStInfo.addElem(VecLdStElem{faddr, faddr, faddr, elem, ix, skip, field});

          if (skip)
            {
              if (vecRegs_.partialSegUpdate_)
                vecRegs_.write(fdv, ix, groupX8, elem);
              continue;
            }

          uint64_t pa1 = faddr, pa2 = faddr; // Physical addrs or faulting virtual addrs.
          uint64_t gpa1 = faddr;

          auto cause = ExceptionCause::NONE;

#ifndef FAST_SLOPPY
          uint64_t gpa2 = faddr;
          cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize, false, false, ix);

          if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(faddr, isLd);
              if (ldStAddrTriggerHit(pmva, elemSize, timing, isLd))
                {
                  ldStInfo.removeLastElem();
                  if (not vecRegs_.partialSegUpdate_)
                    while (not ldStInfo.elems_.empty() and ldStInfo.elems_.back().ix_ == ix)
                  ldStInfo.removeLastElem();
                  markVsDirty();
                  csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
                  return false;
                }
            }
#endif

          if (cause == ExceptionCause::NONE)
            {
              uint64_t data = 0;
              if (not readForLoad<ELEM_TYPE>(di, faddr, pa1, pa2, data, ix, field))
                assert(0 && "Error: Assertion failed");
              elem = data;
              ldStInfo.setLastElem(pa1, pa2, elem);

              if (vecRegs_.partialSegUpdate_)
                vecRegs_.write(fdv, ix, groupX8, elem);
            }
          else
            {
              ldStInfo.removeLastElem();
              if (not vecRegs_.partialSegUpdate_)
                while (not ldStInfo.elems_.empty() and ldStInfo.elems_.back().ix_ == ix)
                  ldStInfo.removeLastElem();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              initiateLoadException(di, cause, ldStFaultAddr_, gpa1);
              return false;
            }
        }

      // If we get here then no excpections were encoutered. Commit all the fields if
      // partial update is not on.
      if (not vecRegs_.partialSegUpdate_)
        {
          unsigned nelems = ldStInfo.elems_.size();
          assert(nelems >= fieldCount);
          for (unsigned field = 0; field < fieldCount; ++field)
            {
              const auto& elem = ldStInfo.elems_.at(nelems - fieldCount + field);
              unsigned fdv = vd + field*group;   // Field destination vector.
              vecRegs_.write(fdv, ix, groupX8, ELEM_TYPE(elem.data_));
            }
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVluxsegei8_v(const DecodedInst* di)
{
  execVloxsegei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei16_v(const DecodedInst* di)
{
  execVloxsegei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei32_v(const DecodedInst* di)
{
  execVloxsegei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei64_v(const DecodedInst* di)
{
  execVloxsegei64_v(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVluxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vectorStoreSegIndexed(const DecodedInst* di, ElementWidth offsetEew,
                                 unsigned fieldCount)
{
  uint32_t elemWidth = vecRegs_.elemWidthInBits();
  uint32_t offsetWidth = VecRegs::elemWidthInBits(offsetEew);

  uint32_t groupX8 = vecRegs_.groupMultiplierX8();
  uint32_t offsetGroupX8 = (offsetWidth*groupX8)/elemWidth;

  GroupMultiplier offsetGroup{GroupMultiplier::One};
  bool badConfig = not VecRegs::groupNumberX8ToSymbol(offsetGroupX8, offsetGroup);
  badConfig = badConfig or not vecRegs_.legalConfig(offsetEew, offsetGroup);
  badConfig = badConfig or (groupX8*fieldCount > 64);

  if (not preVecExec() or badConfig or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  bool masked = di->isMasked();
  uint32_t vd = di->op0(), rs1 = di->op1(), vi = di->op2();

  if (not checkIndexedOpsVsEmul(di, vd, vi, groupX8, offsetGroupX8))
    return false;

  if (not checkVecLdStIndexedInst(di, vd, vi, offsetWidth, offsetGroupX8, fieldCount))
    return false;

  uint64_t addr = intRegs_.read(rs1);
  unsigned start = csRegs_.peekVstart(), elemSize = elemWidth / 8;
  unsigned elemCount = vecRegs_.elemCount();
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;

  // Used registers must not exceed 32.
  if (vd + fieldCount*eg > 32)
    {
      postVecFail(di);
      return false;
    }

  // Effective group. If group is fractional, snap to 1.
  groupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupX8);
  unsigned group = groupX8 / 8;

  // Effective index reg group. If group is fractional, snap to 1.
  offsetGroupX8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), offsetGroupX8);
  unsigned ixGroup = offsetGroupX8 / 8;

  auto& ldStInfo = vecRegs_.ldStInfo_;
  ldStInfo.initIndexed(elemCount, elemSize, vd, vi, group, ixGroup, false /*isLoad*/);
  ldStInfo.setFieldCount(fieldCount, true /*isSeg*/);

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.

  for (unsigned ix = start; ix < elemCount; ++ix)
    {
      uint64_t offset = vecRegs_.readIndexReg(vi, ix, offsetEew, offsetGroupX8);
      uint64_t faddr = addr + offset;

      for (unsigned field = 0; field < fieldCount; ++field, faddr += elemSize)
        {
          uint64_t pa1 = faddr, pa2 = faddr; // Physical addrs or faulting virtual addrs.
          uint64_t gpa1 = faddr, gpa2 = faddr;
          unsigned dvg = vd + field*eg;   // Source vector group.

          bool skip = masked and not vecRegs_.isActive(0, ix);
          ldStInfo.addElem(VecLdStElem{faddr, faddr, faddr, 0, ix, skip, field});
          if (skip)
            continue;

          ELEM_TYPE val = 0;
          vecRegs_.read(dvg, ix, groupX8, val);

          auto cause = determineStoreException(pa1, pa2, gpa1, gpa2, elemSize, false /*hyper*/);

          if (hasTrig)
            {
              uint64_t pmva = applyPointerMask(faddr, isLd);
              ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
              ldStDataTriggerHit(val, timing, isLd);
            }

          if (cause == ExceptionCause::NONE and not breakpOrEnterDebugTripped())
            {
              if (vecRegs_.partialSegUpdate_)
                if (not writeForStore(faddr, pa1, pa2, val))
                  assert(0 && "Error: Assertion failed");
              ldStInfo.setLastElem(pa1, pa2, val);
            }
          else
            {
              ldStInfo.removeLastElem();
              if (not vecRegs_.partialSegUpdate_)
                while (not ldStInfo.elems_.empty() and ldStInfo.elems_.back().ix_ == ix)
                  ldStInfo.removeLastElem();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              if (not breakpOrEnterDebugTripped())
                initiateStoreException(di, cause, ldStFaultAddr_, gpa1);
              return false;
            }
        }

      // If we get here, no exception was encoutered, update all the fields if not in
      // partial-update.
      if (not vecRegs_.partialSegUpdate_)
        {
          for (const auto& elem : ldStInfo.elems_)
            {
              if (elem.skip_)
                continue;

              auto val = ELEM_TYPE(elem.data_);
              if (not writeForStore(elem.va_, elem.pa_, elem.pa2_, val))
                assert(0 && "Error: Assertion failed");
            }
        }
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVsuxsegei8_v(const DecodedInst* di)
{
  execVsoxsegei8_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei16_v(const DecodedInst* di)
{
  execVsoxsegei16_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei32_v(const DecodedInst* di)
{
  execVsoxsegei32_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei64_v(const DecodedInst* di)
{
  execVsoxsegei64_v(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsuxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei8_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Byte, fieldCount); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Byte, fieldCount); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Byte, fieldCount); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Byte, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei16_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Half, fieldCount); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Half, fieldCount); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Half, fieldCount); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Half, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei32_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Word, fieldCount); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Word, fieldCount); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Word, fieldCount); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Word, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei64_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorLoadSegIndexed<uint8_t>(di,  EW::Word2, fieldCount); break;
    case EW::Half:   ok = vectorLoadSegIndexed<uint16_t>(di, EW::Word2, fieldCount); break;
    case EW::Word:   ok = vectorLoadSegIndexed<uint32_t>(di, EW::Word2, fieldCount); break;
    case EW::Word2:  ok = vectorLoadSegIndexed<uint64_t>(di, EW::Word2, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVloxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei8_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Byte, fieldCount); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Byte, fieldCount); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Byte, fieldCount); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Byte, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei16_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Half, fieldCount); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Half, fieldCount); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Half, fieldCount); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Half, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei32_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Word, fieldCount); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Word, fieldCount); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Word, fieldCount); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Word, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei64_v(const DecodedInst* di)
{
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned fieldCount = di->vecFieldCount();
  bool ok = true;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   ok = vectorStoreSegIndexed<uint8_t>(di,  EW::Word2, fieldCount); break;
    case EW::Half:   ok = vectorStoreSegIndexed<uint16_t>(di, EW::Word2, fieldCount); break;
    case EW::Word:   ok = vectorStoreSegIndexed<uint32_t>(di, EW::Word2, fieldCount); break;
    case EW::Word2:  ok = vectorStoreSegIndexed<uint64_t>(di, EW::Word2, fieldCount); break;
    default:         postVecFail(di); return;
    }

  if (ok)
    postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei128_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei256_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei512_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsoxsegei1024_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege8ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint8_t);
  if (not vectorLoadSeg<uint8_t>(di, ElementWidth::Byte, fieldCount, stride, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege16ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint16_t);
  if (not vectorLoadSeg<uint16_t>(di, ElementWidth::Half, fieldCount, stride, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege32ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint32_t);
  if (not vectorLoadSeg<uint32_t>(di, ElementWidth::Word, fieldCount, stride, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege64ff_v(const DecodedInst* di)
{
  unsigned fieldCount = di->vecFieldCount();
  unsigned stride = fieldCount*sizeof(uint64_t);
  if (not vectorLoadSeg<uint64_t>(di, ElementWidth::Word2, fieldCount, stride, true))
    return;
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVlsege128ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege256ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege512ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVlsege1024ff_v(const DecodedInst* di)
{
  postVecFail(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
