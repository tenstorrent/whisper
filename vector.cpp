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


// Checking for conversions of signed char to int is a useful check,
// but in many cases, a signed char is used as a signed byte, and as
// such, conversion to int is allowed.  These conversions are handled
// frequently in templated code below, and clang-tidy isn't able to
// properly ignore these warnings in templates, so just disable them
// entirely in the majority of this file.  The checks can still occur
// in other files, and an incorrect use of a signed char to int
// conversion is unlikely in this file.
//NOLINTBEGIN(bugprone-signed-char-misuse)


namespace WdRiscv
{
  /// Set result to the upper half of a*b computed in double width
  /// intermediate.
  template <typename T>
  void mulh(const T& a, const T& b, T& result)
  {
    using T2 = makeDoubleWide_t<T>; // Double wide type

    unsigned tbits = integerWidth<T> (); // Number of bits in T

    T2 temp = a;
    temp *= b;
    temp >>= tbits;
    result = T(temp);
  }


  /// Set result to the upper half of a*b computed in double width
  /// intermediate. TS is a signed integer type (e.g. int8_t).
  /// TU is the corresponding unsigned integer type (e.g. uint8_t).
  template <typename TS, typename TU>
  void mulhsu(const TS& a, const TU& b, TS& result)
  {
    using TS2 = makeDoubleWide_t<TS>; // Double wide signed type

    unsigned bits = integerWidth<TS> (); // Number of bits in TS and TU

    TS2 temp = a;
    temp *= b;
    temp >>= bits;
    result = TS(temp);
  }


  /// Set result to the product of a and b where a is signed and b
  /// is unsigned and where a and b have the same width.
  template <typename TS, typename TU>
  void mulsu(const TS& a, const TU& b, TS& result)
  {
    bool neg = a < 0;
    TU aa = neg? TU(-a) : TU(a);
    aa *= b;
    result = TS(aa);
    if (neg)
      result = - result;
  }
}


using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::enableVectorExtension(bool flag)
{
  enableExtension(RvExtension::V, flag);
  csRegs_.enableVector(flag);

  if (not flag and not isRvs())
    setVecStatus(VecStatus::Off);
}


template <typename URV>
inline
void
Hart<URV>::setVecStatus(FpStatus value)
{
  if (mstatus_.bits_.VS != unsigned(value))
    {
      mstatus_.bits_.VS = unsigned(value);
      writeMstatus();
    }

  if (virtMode_ and vsstatus_.bits_.VS != unsigned(value))
    {
      vsstatus_.bits_.VS = unsigned(value);
      pokeCsr(CsrNumber::VSSTATUS, vsstatus_.value_);
      recordCsrWrite(CsrNumber::VSSTATUS);
      updateCachedVsstatus();
    }
}


template <typename URV>
bool
Hart<URV>::checkVecIntInst(const DecodedInst* di)
{
  return checkVecIntInst(di, vecRegs_.elemWidth(), vecRegs_.groupMultiplier());
}


template <typename URV>
bool
Hart<URV>::checkVecIntInst(const DecodedInst* di, ElementWidth eew, GroupMultiplier gm)
{
  if (not checkSewLmulVstart(di))
    return false;

  // Dest register cannot overlap mask register v0.
  if (di->isMasked() and di->op0() == 0)
    {
      postVecFail(di);
      return false;
    }

  // None of the vector source registers can overlap mask regiser v0.
  // Section 5.2 of vector spec version 1.1.
  if (di->isMasked())
    {
      for (unsigned i = 1; i < di->operandCount(); ++i)
	if (di->ithOperand(i) == 0 and di->ithOperandType(i) == OperandType::VecReg)
	  {
	    postVecFail(di);
	    return false;
	  }
    }

  // Use of vstart values greater than vlmax is reserved (section 32.3.7 of spec).
  if (trapOobVstart_ and csRegs_.peekVstart() >= vecRegs_.vlmax(eew, gm))
    {
      postVecFail(di);
      return false;
    }

  return true;
}


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
Hart<URV>::checkSewLmulVstart(const DecodedInst* di)
{
  // Vector extension must be enabled, MSTATUS.FS must not be off, sew/lmul must
  // be legal, vtype.vill must not be set.
  auto ok = preVecExec();

  using enum InstId;
  auto id = di->instId();

  // Vill must not be set, unless vmv*r.v instruction and vtype=1 is allowed for it.
  bool isVmv = (id == vmv1r_v or id == vmv2r_v or id == vmv4r_v or id == vmv8r_v);
  if (not isVmv or not vecRegs_.vmvrIgnoreVill_)
    ok = ok and vecRegs_.legalConfig();

  // Trap on use of non-zero vstart for arithmetic vector ops.
  URV vstart = csRegs_.peekVstart();
  ok = ok and not (trapNonZeroVstart_ and vstart > 0);

  if (not ok)
    postVecFail(di);

  return ok;
}  


template <typename URV>
bool
Hart<URV>::checkVecOpsVsEmul(const DecodedInst* di, unsigned groupX8,
                             std::initializer_list<unsigned> opList)
{
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8

  bool ok = true;

  // Each vector operand number must be a multiple of the effective grop.
  unsigned i = 0;
  for (auto op : opList)
    {
      vecRegs_.setIthOpEmul(i++, eg);  // For reporting: record group of operand.
      ok = ok and ((op & mask) == 0);
    }

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
inline
bool
Hart<URV>::checkRedOpVsEmul(const DecodedInst* di)
{
  // Reduction ops must have zero vstart.
  unsigned start = csRegs_.peekVstart();
  if (start > 0)
    {
      postVecFail(di);
      return false;
    }

  if (di->isMasked() and (di->op1() == 0 or di->op2() == 0))
    {
      postVecFail(di);    // V0 used as source with differing EEWs.
      return false;
    }

  unsigned groupX8 = vecRegs_.groupMultiplierX8();
  unsigned vs1 = di->op1();

  // Vector register (vs1) must be a multiple of lmul.

  unsigned lmul = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = lmul - 1;   // Assumes lmul is 1, 2, 4, or 8

  if ((vs1 & mask) != 0)
    {
      postVecFail(di);
      return false;
    }

  // Track operand group for logging (vd and vs2 have an lmul of 1).
  vecRegs_.setOpEmul(1, lmul, 1);
  return true;
}


template <typename URV>
inline
bool
Hart<URV>::checkWideRedOpVsEmul(const DecodedInst* di)
{
  // Reduction ops must have zero vstart.
  unsigned start = csRegs_.peekVstart();
  if (start > 0)
    {
      postVecFail(di);
      return false;
    }

  unsigned gX8 = vecRegs_.groupMultiplierX8();
  unsigned lmul = gX8 >= 8 ? gX8 / 8 : 1;
  unsigned vs1 = di->op1();

  // Vector register (vs1) must be a multiple of lmul.
  unsigned mask = lmul - 1;   // Assumes lmul is 1, 2, 4, or 8

  if ((vs1 & mask) != 0)
    {
      postVecFail(di);
      return false;
    }

  // Check that source registers are not used with different EEWs
  unsigned vs2 = di->op2();
  if ( (di->isMasked() and (vs1 == 0 or vs2 == 0)) or (vs2 >= vs1 and vs2 < vs1 + lmul) )
    {
      postVecFail(di);  
      return false;
    }

  // Track operand group for logging (vd and vs2 have an lmul of 1).
  vecRegs_.setOpEmul(1, lmul, 1);
  return true;
}


template <typename URV>
bool
Hart<URV>::checkIndexedOpsVsEmul(const DecodedInst* di, unsigned op0, unsigned op2,
                                 unsigned groupX8, unsigned offsetGroupX8)
{
  unsigned eg0 = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask0 = eg0 - 1;   // Assumes eg is 1, 2, 4, or 8

  unsigned eg2 = offsetGroupX8 >= 8 ? offsetGroupX8 / 8 : 1;
  unsigned mask2 = eg2 - 1;

  if (((op0 & mask0) == 0) and
      ((op2 & mask2) == 0))
    {
      vecRegs_.setOpEmul(eg0, 1, eg2);  // Track operand group for logging
      return true;
    }
  postVecFail(di);
  return false;
}


template <typename URV>
bool
Hart<URV>::checkDestSourceOverlap(unsigned dest, unsigned destWidth, unsigned destGroupX8,
                                  unsigned src, unsigned srcWidth, unsigned srcGroupX8)
{
  if (srcWidth == destWidth)
    return true;   // Source eew == dest eew

  unsigned srcGroup = srcGroupX8 >= 8 ? srcGroupX8/8 : 1;
  unsigned destGroup = destGroupX8 >= 8 ? destGroupX8/8 : 1;

  if (src >= dest + destGroup or dest >= src + srcGroup)
    return true;  // No overlap.

  // Destination eew > source eew: Overlap ok if source group is >=
  // 1 and overlap is at last # of <srcGroup> registers in dest.
  if (destWidth > srcWidth)
    return srcGroupX8 >= 8 and src == dest + destGroup - srcGroup;

  // Destination eew < source eew: Overlap ok if overlap is at
  // first register in source.
  return src == dest;
}


template <typename URV>
inline
bool
Hart<URV>::checkSourceOverlap(unsigned s1, unsigned eew1, unsigned group1X8,
                              unsigned s2, unsigned eew2, unsigned group2X8)
{
  if (eew1 == eew2)
    return true;

  unsigned g1 = group1X8 >= 8 ? group1X8/8 : 1;
  unsigned g2 = group2X8 >= 8 ? group2X8/8 : 1;

  if (s1 >= s2 + g2 or s2 >= s1 + g1)
    return true;  // No overlap.
  return false;   // Overlap and different EEWs.
}


template <typename URV>
inline
bool
Hart<URV>::checkVecMaskInst(const DecodedInst* di, unsigned dest,
			    unsigned src, unsigned groupX8)
{
  if (not checkSewLmulVstart(di))
    return false;

  // Use of vstart values greater than vlmax is reserved (section 32.3.7 of spec).
  if (trapOobVstart_ and csRegs_.peekVstart() >= vecRegs_.vlmax())
    {
      postVecFail(di);
      return false;
    }

  // Source register (eew != 1) cannot overlap v0 (eew == 1) if instruction
  // is masked.
  if (di->isMasked() and src == 0)
    {
      postVecFail(di);
      return false;
    }

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8

  // Destination eew is 1 which is less than source, overlap ok only
  // if it is in the 1st source register (dest == src).
  if (hasDestSourceOverlap(dest, 8, src, groupX8) and dest != src)
    {
      postVecFail(di);
      return false;
    }

  if ((src & mask) == 0)
    {
      vecRegs_.setOpEmul(1, eg);  // Track operand group for logging. 1 for mask.
      return true;
    }

  // Vector operand not a multiple of emul: illegal.
  postVecFail(di);
  return false;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecMaskInst(const DecodedInst* di, unsigned op0, unsigned op1,
			    unsigned op2, unsigned groupX8)
{
  if (not checkSewLmulVstart(di))
    return false;

  // Use of vstart values greater than vlmax is reserved (section 32.3.7 of spec).
  if (trapOobVstart_ and csRegs_.peekVstart() >= vecRegs_.vlmax())
    {
      postVecFail(di);
      return false;
    }

  // Source registers (eew != 1) cannot overlap v0 (eew == 1) if instruction
  // is masked.
  if (di->isMasked() and (op1 == 0 or op2 == 0))
    {
      postVecFail(di);
      return false;
    }

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  unsigned mask = eg - 1;   // Assumes eg is 1, 2, 4, or 8

  unsigned destEew = 1, destGroupX8 = 8, srcEew = vecRegs_.elemWidthInBits();
  if (not checkDestSourceOverlap(op0, destEew, destGroupX8, op1, srcEew, groupX8) or
      not checkDestSourceOverlap(op0, destEew, destGroupX8, op2, srcEew, groupX8))
    {
      postVecFail(di);
      return false;
    }

  unsigned op = op1 | op2;
  if ((op & mask) == 0)
    {
      vecRegs_.setOpEmul(1, eg, eg);  // Track operand group for logging. 1 for mask.
      return true;
    }

  // Vector operand not a multiple of emul: illegal.
  postVecFail(di);
  return false;
}


template <typename URV>
bool
Hart<URV>::checkVecOpsVsEmul(const DecodedInst* di, unsigned groupX8,
                             std::initializer_list<std::pair<unsigned,bool>> opList,
                             bool destAlsoSrc)
{
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;  // Effective group.

  // Mask of bits that must be 0 in vector register number. Assumes eg is 1, 2, 4, or 8
  unsigned mask = eg - 1;

  unsigned wgX8 = 2*groupX8;
  unsigned ewg = wgX8 >= 8 ? wgX8 / 8 : 1;  // Effective wide group.
  unsigned wmask = ewg - 1;

  unsigned sew = vecRegs_.elemWidthInBits();
  unsigned wsew = sew * 2;  // Wide sew

  bool ok = true;

  // Each vector operand number must be a multiple of the effective grop.
  unsigned nn = 0;  // Operand count.
  for (auto item : opList)
    {
      auto [op, wide] = item;
      auto ig = wide ? ewg : eg;  // Item group.
      vecRegs_.setIthOpEmul(nn++, ig);  // For reporting: record group of operand.
      unsigned im = wide ? wmask : mask;  // Item mask.
      ok = ok and ((op & im) == 0);
    }

  assert(nn <= 3);  // At most 3 vector operands.

  if (nn > 1)
    {
      auto iter = opList.begin();
      auto [dest, dw] = *iter++;
      unsigned dg = dw ? wgX8 : groupX8;  // Dest group.
      unsigned dsew = dw ? wsew : sew;

      auto [op1, op1w] = *iter++;
      unsigned op1g = op1w ? wgX8 : groupX8;
      unsigned op1sew = op1w ? wsew : sew;

      ok = ok and checkDestSourceOverlap(dest, dsew, dg, op1, op1sew, op1g);
      if (destAlsoSrc)
        ok = ok and checkSourceOverlap(dest, dsew, dg, op1, op1sew, op1g);

      if (nn > 2)
        {
          auto [op2, op2w] = *iter++;
          unsigned op2g = op2w ? wgX8 : groupX8;
          unsigned op2sew = op2w ? wsew : sew;

          ok = ok and checkDestSourceOverlap(dest, dsew, dg, op2, op2sew, op2g);
          if (destAlsoSrc)
            ok = ok and checkSourceOverlap(dest, dsew, dg, op2, op2sew, op2g);

          ok = ok and checkSourceOverlap(op1, op1sew, op1g, op2, op2sew, op2g);
        }
    }

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecFpMaskInst(const DecodedInst* di, unsigned dest,
				   unsigned src, unsigned groupX8)
{
  if (not checkVecMaskInst(di, dest, src, groupX8))
    return false;

  bool ok = false;
  if (checkRoundingModeCommon(di))
    {
      using EW = ElementWidth;
      EW sew = vecRegs_.elemWidth();
      switch (sew)
        {
        case EW::Half:  ok = isHalfFpLegal(); break;
        case EW::Word:  ok = isFpLegal();     break;
        case EW::Word2: ok = isDpLegal();     break;
        default:        ok = false;           break;
        }
    }

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecFpMaskInst(const DecodedInst* di, unsigned dest,
				   unsigned src1, unsigned src2, unsigned groupX8)
{
  if (not checkVecMaskInst(di, dest, src1, src2, groupX8))
    return false;

  bool ok = false;
  if (checkRoundingModeCommon(di))
    {
      using EW = ElementWidth;
      EW sew = vecRegs_.elemWidth();
      switch (sew)
        {
        case EW::Half:  ok = isHalfFpLegal(); break;
        case EW::Word:  ok = isFpLegal();     break;
        case EW::Word2: ok = isDpLegal();     break;
        default:        ok = false;           break;
        }
    }

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  if (not ok)
    postVecFail(di);

  return ok;
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
bool
Hart<URV>::vsetvl(unsigned rd, unsigned rs1, URV vtypeVal, bool vli /* vsetvli instruction*/)
{
  bool ma = (vtypeVal >> 7) & 1;  // Mask agnostic
  bool ta = (vtypeVal >> 6) & 1;  // Tail agnostic
  bool altfmt = (isRvzvfofp8min() or isRvzvfbfa() or isRvzvfwbdota16bf() or isRvzvfqwbdota8f()) and ((vtypeVal >> 8) & 1);
  auto gm = GroupMultiplier(vtypeVal & 7);
  auto ew = ElementWidth((vtypeVal >> 3) & 7);

  bool vill = (vtypeVal >> (8*sizeof(URV) - 1)) & 1;
  vill = vill or not vecRegs_.legalConfig(ew, gm);
  vill = vill or (altfmt and ew >= ElementWidth::Word);

  // Only least sig 8 bits can be non-zero unless Zvfbfa/Zvfofp8min/Zvfwbdota16bf/Zvfqwbdota8f
  // enables VTYPE.ALTFMT at bit 8. All other bits are reserved.
  URV reservedBits = vtypeVal >> (altfmt ? 9 : 8);
  vill = vill or (reservedBits != 0);

  // Determine vl
  bool legalizedAvl = false;
  URV elems = 0;

  if (gm == GroupMultiplier::Reserved)
    vill = true;
  else
    {
      uint32_t gm8 = VecRegs::groupMultiplierX8(gm);
      unsigned bitsPerElem = VecRegs::elemWidthInBits(ew);
      unsigned vlmax = (gm8*vecRegs_.bitsPerRegister()/bitsPerElem) / 8;
      if (vlmax == 0)
        vill = true;
      else
        {
          if (rd != 0 and rs1 == 0)
            elems = vlmax;
          else if (rd == 0 and rs1 == 0)
	    {
	      // Section 6.2 of vec spec. Cannot change VLMAX. This should apply uniformly
	      // to vsetvl & vsetvli. The instructions are implemented by different
	      // tribes. One tribe takes an exception, the other legalizes (trims) VL.
	      unsigned prevVlmax = vecRegs_.vlmax();
	      if (vlmax != prevVlmax and not vill)
		{
		  auto trim = vli? vecRegs_.legalizeVsetvliAvl_ : vecRegs_.legalizeVsetvlAvl_;
		  vill = not trim;
		}
	      elems = peekCsr(CsrNumber::VL);  // Current value of VL (maybe trimmed below).
	    }
          else  // strip mining
            {
              URV avl = intRegs_.read(rs1);  // Application vector length.
              if (avl <= vlmax)
                elems = avl;
              else
		// For avl >= 2*vlmax, spec mandates setting vl to vlmax.
		// For avl > vlmax and < 2*vlmax, spec allows anything between 
		// ceil(avl/2) and vlmax inclusive. We choose vlmax.
		elems = vlmax;
            }
        }

      if (elems > vlmax)
        {
          legalizedAvl = vli? vecRegs_.legalizeVsetvliAvl_ : vecRegs_.legalizeVsetvlAvl_;
          if (legalizedAvl and vlmax != 0)
            elems = vlmax;
          else
            vill = true;
        }
    }

  if (vill)
    {
      // Spec gives us a choice: Trap on illegal config or set vtype.vill.
      if (vecRegs_.trapVtype_)
	return false;  // Caller must trap.
      ma = false; ta = false; gm = GroupMultiplier(0); ew = ElementWidth(0);
      altfmt = false;
      elems = 0;
    }

  if (vill or (rd != 0 or rs1 != 0) or legalizedAvl)
    {
      // VL is not writeable: Poke it.
      pokeCsr(CsrNumber::VL, elems);
      recordCsrWrite(CsrNumber::VL);
    }

  elems = peekCsr(CsrNumber::VL);
  intRegs_.write(rd, elems);
  vecRegs_.elemCount(elems);  // Update cached value of VL.

  // Pack vtype values and update vtype
  URV vtype = 0;
  vtype |= URV(gm) | (URV(ew) << 3) | (URV(ta) << 6) | (URV(ma) << 7);
  vtype |= URV(altfmt) << 8;
  vtype |= (URV(vill) << (8*sizeof(URV) - 1));
  pokeCsr(CsrNumber::VTYPE, vtype);  // This updates cached vtype values in vecRegs_.
  recordCsrWrite(CsrNumber::VTYPE);

  markVsDirty();
  return true;
}


template <typename URV>
void
Hart<URV>::postVecSuccess(const DecodedInst* di)
{
  bool dirty = false;  // True if MSTATUS.VS should be marked dirty.

  if (vecRegs_.getLastWrittenReg() >= 0)
    dirty = true;    // A vector register was written.

  if (not dirty and vecRegs_.alwaysMarkDirty_)
    {
      auto type = di->ithOperandType(0);  // Dest register type
      auto mode = di->ithOperandMode(0);  // Dest register mode
      bool vecTarget =  type == OperandType::VecReg and mode == OperandMode::Write;
      if (vecTarget)
        {
          if (di->isVectorLoad())
            dirty = vecRegs_.amdCoversLoad_;
          else
            dirty = true;
        }
    }

  if (csRegs_.peekVstart() != 0)
    {
      csRegs_.clearVstart();
      recordCsrWrite(CsrNumber::VSTART);
      dirty = true;
    }

  if (dirty)
    markVsDirty();
}


template <typename URV>
void
Hart<URV>::postVecFail(const DecodedInst* di)
{
  illegalInst(di);

  if (vecRegs_.getLastWrittenReg() >= 0)
    markVsDirty();
}


template <typename URV>
void
Hart<URV>::execVsetvli(const DecodedInst* di)
{
  if (not preVecExec())
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0();
  unsigned rs1 = di->op1();
  unsigned imm = di->op2();
  
  URV vtypeVal = imm;
  if (vsetvl(rd, rs1, vtypeVal, true /* isVtypeImm */))
    postVecSuccess(di);
  else
    postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVsetivli(const DecodedInst* di)
{
  if (not preVecExec())
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0();
  unsigned avl = di->op1();
  unsigned imm = di->op2();
  
  bool ma = (imm >> 7) & 1;  // Mask agnostic
  bool ta = (imm >> 6) & 1;  // Tail agnostic
  bool altfmt = (isRvzvfofp8min() or isRvzvfbfa() or isRvzvfwbdota16bf() or isRvzvfqwbdota8f()) and ((imm >> 8) & 1);
  auto gm = GroupMultiplier(imm & 7);
  auto ew = ElementWidth((imm >> 3) & 7);

  // Only least sig 8 bits can be non-zero unless Zvfbfa/Zvfofp8min/Zvfwbdota16bf/Zvfqwbdota8f
  // enables VTYPE.ALTFMT at bit 8. All other bits are reserved.
  bool vill = (imm >> (altfmt? 9 : 8)) != 0;
  vill = vill or not vecRegs_.legalConfig(ew, gm);

  // Determine vl
  URV elems = avl;
  if (gm == GroupMultiplier::Reserved)
    vill = true;
  else
    {
      uint32_t gm8 = VecRegs::groupMultiplierX8(gm);
      unsigned bitsPerElem = VecRegs::elemWidthInBits(ew);
      unsigned vlmax = (gm8*vecRegs_.bitsPerRegister()/bitsPerElem) / 8;
      if (vlmax == 0)
        vill = true;
      else if (elems > vlmax)
	elems = vlmax;
    }

  if (vill)
    {
      // Spec gives us a choice: Trap on illegal config or set vtype.vill.
      if (vecRegs_.trapVtype_)
	{
	  postVecFail(di);  // Trap.
	  return;
	}
      ma = false; ta = false; gm = GroupMultiplier(0); ew = ElementWidth(0);
      elems = 0;
    }

  // VL is not writeable: Poke it.
  pokeCsr(CsrNumber::VL, elems);
  recordCsrWrite(CsrNumber::VL);

  vecRegs_.elemCount(elems);  // Update cached value of VL.
  intRegs_.write(rd, elems);

  // Pack vtype values and update vtype. Vtype is read-only, poke it.
  VtypeFields<URV> vtf{0};
  vtf.bits_.LMUL = unsigned(gm);
  vtf.bits_.SEW = unsigned(ew);
  vtf.bits_.VTA = ta;
  vtf.bits_.VMA = ma;
  vtf.bits_.ALTFMT = altfmt;
  vtf.bits_.VILL = vill;
  pokeCsr(CsrNumber::VTYPE, vtf.value_);
  recordCsrWrite(CsrNumber::VTYPE);

  // Update cached vtype fields in vecRegs_.
  vecRegs_.updateConfig(ew, gm, ma, ta, vill);
  postVecSuccess(di);

  markVsDirty();
}


template <typename URV>
void
Hart<URV>::execVsetvl(const DecodedInst* di)
{
  if (not preVecExec())
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0();
  unsigned rs1 = di->op1();
  URV vtypeVal = intRegs_.read(di->op2());

  if (vsetvl(rd, rs1, vtypeVal, false /* isVtypeImm */))
    postVecSuccess(di);
  else
    postVecFail(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked,
		  std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = op(e1, e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vop_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		  unsigned start, unsigned elems, bool masked,
		  std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op)
{
  ELEM_TYPE e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = op(e1, e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVop_vv(const DecodedInst* di, OP op)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVopu_vv(const DecodedInst* di, OP op)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();

  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVop_vx(const DecodedInst* di, OP op)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVopu_vx(const DecodedInst* di, OP op)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVop_vi(const DecodedInst* di, OP op)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int64_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVopu_vi(const DecodedInst* di, OP op)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  auto imm = uint64_t(int64_t(di->op2As<int32_t>()));
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint8_t> (vd, vs1, imm, group, start, elems, masked, op);
      break;
    case EW::Half:
      vop_vx<uint16_t>(vd, vs1, imm, group, start, elems, masked, op);
      break;
    case EW::Word:
      vop_vx<uint32_t>(vd, vs1, imm, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vop_vx<uint64_t>(vd, vs1, imm, group, start, elems, masked, op);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVadd_vv(const DecodedInst* di)
{
  execVop_vv(di, std::plus());
}


template <typename URV>
void
Hart<URV>::execVadd_vx(const DecodedInst* di)
{
  execVop_vx(di, std::plus());
}


template <typename URV>
void
Hart<URV>::execVadd_vi(const DecodedInst* di)
{
  execVop_vi(di, std::plus());
}


template <typename URV>
void
Hart<URV>::execVsub_vv(const DecodedInst* di)
{
  execVop_vv(di, std::minus());
}


template <typename URV>
void
Hart<URV>::execVsub_vx(const DecodedInst* di)
{
  execVop_vx(di, std::minus());
}


template <typename URV>
void
Hart<URV>::execVrsub_vx(const DecodedInst* di)
{
  execVop_vx(di, MyRsub());
}


template <typename URV>
void
Hart<URV>::execVrsub_vi(const DecodedInst* di)
{
  execVop_vi(di, MyRsub());
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0;

  // We take the max of lmul == 1 to compensate for tail elements.
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = DWT(e1);
	  dest += DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwaddu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwadd_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwadd_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwadd_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwadd_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwadd_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type

  ELEM_TYPE e1 = 0;
  DWT dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = DWT(e1);
	  dest += DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwaddu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  // Here SEW is at most 32-bits.
  auto e2 = uint32_t(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwadd_vx<uint8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwadd_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwadd_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: // Fall-through to invalid case.
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwadd_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  // Here sew is at most 32-bits.
  SRV e2 = SRV(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwadd_vx<int8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwadd_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwadd_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwadd_vx<int64_t>(vd, vs1, e2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwsub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type

  ELEM_TYPE e1 = 0;
  DWT dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = DWT(e1);
	  dest -= DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwsubu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  // Here SEW is at most 32-bits.
  auto e2 = uint32_t(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwsub_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwsub_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwsub_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwsub_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  // Here SEW is at most 32-bits.
  auto e2 = int32_t(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwsub_vx<int8_t> (vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwsub_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwsub_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template<typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = DWT(e1);
	  dest -= DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}

                        
template <typename URV>
void
Hart<URV>::execVwsubu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwsub_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwsub_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwsub_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwsub_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwsub_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwsub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwsub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwadd_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type
  unsigned wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, wideGroup, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = e1;
	  dest += DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwaddu_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwadd_wv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwadd_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwadd_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwadd_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwadd_wv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwadd_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwadd_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwadd_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwaddu_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}}))
    return;

  // Scalar value is sign extended and then interpreted as unsigned if XLEN < SEW.
  // Here SEW is at most a word,
  URV e2 = intRegs_.read(di->op2());

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint16_t>(vd, vs1, uint8_t(e2), group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<uint32_t>(vd, vs1, uint16_t(e2), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<uint64_t>(vd, vs1, uint32_t(e2), group*2, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwadd_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}}))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int16_t>(vd, vs1, int8_t(e2),  group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<int32_t>(vd, vs1, int16_t(e2), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<int64_t>(vd, vs1, int32_t(e2), group*2, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwsubu_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}}))
    return;

  URV e2 = intRegs_.read(di->op2());

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<uint16_t>(vd, vs1, -uint16_t(uint8_t(e2)),  group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<uint32_t>(vd, vs1, -uint32_t(uint16_t(e2)), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<uint64_t>(vd, vs1, -uint64_t(uint32_t(e2)), group*2, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwsub_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}}))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vop_vx<int16_t>(vd, vs1, -int16_t(int8_t(e2)),  group*2, start, elems, masked, std::plus());
      break;
    case EW::Half:
      vop_vx<int32_t>(vd, vs1, -int32_t(int16_t(e2)), group*2, start, elems, masked, std::plus());
      break;
    case EW::Word:
      vop_vx<int64_t>(vd, vs1, -int64_t(int32_t(e2)), group*2, start, elems, masked, std::plus());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwsub_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type
  unsigned wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, wideGroup, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = e1;
	  dest -= DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwsubu_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwsub_wv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwsub_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwsub_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwsub_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwsub_wv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwsub_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwsub_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwsub_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked,
		   std::function<bool(ELEM_TYPE, ELEM_TYPE)> op)
{
  ELEM_TYPE e1{}, e2{};

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;
      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  flag = op(e1, e2);
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmop_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		   unsigned start, unsigned elems, bool masked,
		   std::function<bool(ELEM_TYPE, ELEM_TYPE)> op)
{
  ELEM_TYPE e1 = 0;

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;
      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  flag = op(e1, e2);
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmseq_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmseq_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmseq_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int64_t imm = di->op2As<int32_t>();

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsne_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::not_equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsne_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::not_equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsne_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int64_t imm = di->op2As<int32_t>();

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::not_equal_to());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsltu_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsltu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmslt_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmslt_vx(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Sign extend scalar operand per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::less());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsleu_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsleu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsleu_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Immediate is sign etxended and then treated as unsigned. Per spec.
  auto imm = uint64_t(int64_t(di->op2As<int32_t>()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsle_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsle_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsle_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int64_t imm = di->op2As<int32_t>();

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::less_equal());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsgtu_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsgtu_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Immediate is sign extended and then treated as unsigned.
  auto imm = uint64_t(int64_t(di->op2As<int32_t>()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<uint8_t> (vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<uint16_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<uint32_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<uint64_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsgt_vx(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, e2, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsgt_vi(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vd, vs1, group))
    return;

  int64_t imm = di->op2As<int32_t>();

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vmop_vx<int8_t> (vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Half:
      vmop_vx<int16_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word:
      vmop_vx<int32_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    case EW::Word2:
      vmop_vx<int64_t>(vd, vs1, imm, group, start, elems, masked, std::greater());
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVminu_vv(const DecodedInst* di)
{
  execVopu_vv(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVminu_vx(const DecodedInst* di)
{
  execVopu_vx(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVmin_vv(const DecodedInst* di)
{
  execVop_vv(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVmin_vx(const DecodedInst* di)
{
  execVop_vx(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVmaxu_vv(const DecodedInst* di)
{
  execVopu_vv(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVmaxu_vx(const DecodedInst* di)
{
  execVopu_vx(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVmax_vv(const DecodedInst* di)
{
  execVop_vv(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVmax_vx(const DecodedInst* di)
{
  execVop_vx(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVand_vv(const DecodedInst* di)
{
  execVopu_vv(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVand_vx(const DecodedInst* di)
{
  execVop_vx(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVand_vi(const DecodedInst* di)
{
  execVop_vi(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVor_vv(const DecodedInst* di)
{
  execVopu_vv(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVor_vx(const DecodedInst* di)
{
  execVop_vx(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVor_vi(const DecodedInst* di)
{
  execVop_vi(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVxor_vv(const DecodedInst* di)
{
  execVopu_vv(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVxor_vx(const DecodedInst* di)
{
  execVop_vx(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVxor_vi(const DecodedInst* di)
{
  execVop_vi(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVsll_vv(const DecodedInst* di)
{
  execVopu_vv(di, MySll());
}


template <typename URV>
void
Hart<URV>::execVsll_vx(const DecodedInst* di)
{
  execVopu_vx(di, MySll());
}


template <typename URV>
void
Hart<URV>::execVsll_vi(const DecodedInst* di)
{
  execVopu_vi(di, MySll());
}


template <typename URV>
void
Hart<URV>::execVsrl_vv(const DecodedInst* di)
{
  execVopu_vv(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsrl_vx(const DecodedInst* di)
{
  execVopu_vx(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsrl_vi(const DecodedInst* di)
{
  execVopu_vi(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsra_vv(const DecodedInst* di)
{
  execVop_vv(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsra_vx(const DecodedInst* di)
{
  execVop_vx(di, MySr());
}


template <typename URV>
void
Hart<URV>::execVsra_vi(const DecodedInst* di)
{
  execVop_vi(di, MySr());
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnsr_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		  unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE2X e1 = 0;
  ELEM_TYPE e2 = 0, dest = 0;

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned group2x = group*2;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = ELEM_TYPE(e1 >> (unsigned(e2) & mask));
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVnsrl_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnsr_wv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnsr_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnsr_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnsr_wx(unsigned vd, unsigned vs1, URV e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE2X e1 = 0;
  ELEM_TYPE dest = 0;

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned amount = unsigned(e2) & mask;
  unsigned group2x = group*2;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  dest = ELEM_TYPE(e1 >> amount);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVnsrl_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  // Spec says sign extend scalar register. We comply.
  URV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<uint8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vnsr_wx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vnsr_wx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wx<uint64_t>(vd, vs1, e2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnsrl_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool msk = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  URV imm = di->op2();   // Unsigned -- zero extended.

  unsigned gp = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, gp))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, gp, {{vd, false}, {vs1, true}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<uint8_t> (vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Half:  vnsr_wx<uint16_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word:  vnsr_wx<uint32_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word2: vnsr_wx<uint64_t>(vd, vs1, imm, gp, start, elems, msk); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnsra_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnsr_wv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnsr_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnsr_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnsra_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  // Spec says sign extend scalar register. We comply.
  URV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<int8_t> (vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vnsr_wx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vnsr_wx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vnsr_wx<int64_t>(vd, vs1, e2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnsra_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool msk = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  URV imm = di->op2();   // Unsigned -- zero extended

  unsigned gp = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, gp))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, gp, {{vd, false}, {vs1, true}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnsr_wx<int8_t> (vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Half:  vnsr_wx<int16_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word:  vnsr_wx<int32_t>(vd, vs1, imm, gp, start, elems, msk); break;
    case EW::Word2: vnsr_wx<int64_t>(vd, vs1, imm, gp, start, elems, msk); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgather_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = 0;
	  if (e2 < vecRegs_.bytesPerRegister() * 8)
	    {
	      auto vs1Ix = unsigned(e2);
	      if (vecRegs_.isValidIndex(vs1, vs1Ix, group, sizeof(e1)))
		{
		  vecRegs_.read(vs1, vs1Ix, group, e1);
		  dest = e1;
		}
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVrgather_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (hasDestSourceOverlap(vd, group, vs1, group) or
      hasDestSourceOverlap(vd, group, vs2, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vrgather_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vrgather_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vrgather_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vrgather_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgather_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  unsigned bytesPerElem = sizeof(ELEM_TYPE);
  unsigned vlmax = group*vecRegs_.bitsPerRegister()/bytesPerElem;

  URV rv2 = intRegs_.read(rs2);
  URV vs1Ix = rv2 < vlmax ? rv2 : vlmax;

  ELEM_TYPE e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  dest = 0;
	  if (vecRegs_.isValidIndex(vs1, vs1Ix, group, sizeof(e1)))
	    {
	      vecRegs_.read(vs1, vs1Ix, group, e1);
	      dest = e1;
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVrgather_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vrgather_vx<uint8_t> (vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vrgather_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vrgather_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vrgather_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgather_vi(unsigned vd, unsigned vs1, uint32_t imm, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  uint32_t vs1Ix = imm;
  ELEM_TYPE e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  dest = 0;
	  if (vecRegs_.isValidIndex(vs1, vs1Ix, group, sizeof(e1)))
	    {
	      vecRegs_.read(vs1, vs1Ix, group, e1);
	      dest = e1;
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVrgather_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  uint32_t vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vrgather_vi<uint8_t> (vd, vs1, imm, group, start, elems, masked); break;
    case EW::Half:  vrgather_vi<uint16_t>(vd, vs1, imm, group, start, elems, masked); break;
    case EW::Word:  vrgather_vi<uint32_t>(vd, vs1, imm, group, start, elems, masked); break;
    case EW::Word2: vrgather_vi<uint64_t>(vd, vs1, imm, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrgatherei16_vv(unsigned vd, unsigned vs1, unsigned vs2,
                           unsigned group, unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;
  uint16_t e2 = 0;

  unsigned e2Group = (16UL*group)/(8*sizeof(ELEM_TYPE));
  e2Group = std::max(e2Group, 1u);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, e2Group, e2);

	  unsigned vs1Ix = e2;
	  dest = 0;
	  if (vecRegs_.isValidIndex(vs1, vs1Ix, group, sizeof(e1)))
	    {
	      vecRegs_.read(vs1, vs1Ix, group, e1);
	      dest = e1;
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVrgatherei16_vv(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned g8 = vecRegs_.groupMultiplierX8();  // Group of VD and V1 times 8.
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  unsigned widthInBytes = VecRegs::elemWidthInBytes(sew);
  bool masked = di->isMasked();

  unsigned v2g8 = (2*g8) / widthInBytes;  // Group of V2 times 8.

  GroupMultiplier v2gm = GroupMultiplier::One;
  if (not VecRegs::groupNumberX8ToSymbol(v2g8, v2gm) or
      not vecRegs_.legalConfig(ElementWidth::Half, v2gm) or
      (masked and (vd == 0 or vs1 == 0 or vs2 == 0)))
    {
      postVecFail(di);
      return;
    }

  unsigned eg = g8 >= 8 ? g8 / 8 : 1;
  unsigned v2g = v2g8 >= 8 ? v2g8 / 8 : 1;

  if ((vd % eg) or (vs1 % eg) or (vs2 % v2g))
    {
      postVecFail(di);
      return;
    }

  unsigned ew1 = vecRegs_.elemWidthInBits();
  unsigned ew2 = 16;

  if (hasDestSourceOverlap(vd, g8, vs1, g8) or
      hasDestSourceOverlap(vd, g8, vs2, v2g8) or
      not checkSourceOverlap(vs1, ew1, g8, vs2, ew2, v2g8))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  vecRegs_.setOpEmul(eg, eg, v2g);  // Track operand group for logging

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vrgatherei16_vv<uint8_t>(vd, vs1, vs2, g8, start, elems, masked); break;
    case EW::Half:  vrgatherei16_vv<uint16_t>(vd, vs1, vs2, g8, start, elems, masked); break;
    case EW::Word:  vrgatherei16_vv<uint32_t>(vd, vs1, vs2, g8, start, elems, masked); break;
    case EW::Word2: vrgatherei16_vv<uint64_t>(vd, vs1, vs2, g8, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vcompress_vm(unsigned vd, unsigned vs1, unsigned vs2,
                        unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, dest = 0;
  unsigned destIx = 0;

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isActive(vs2, ix))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = e1;
	  vecRegs_.write(vd, destIx++, group, dest);
	}
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  // Remaining elements are treated as tail elements.
  bool setTail = vecRegs_.isTailAgnostic() and vecRegs_.isTailAgnosticOnes();
  if (setTail)
    {
      unsigned elemMax = vecRegs_.elemMax();
      dest = ~ELEM_TYPE{0};
      for (unsigned ix = destIx; ix < elemMax; ++ix)
	vecRegs_.write(vd, ix, destGroup, dest);  // Either copy of original or all ones.
    }

  vecRegs_.touchReg(vd, group);  // For logging: in case no element was written.
}


template <typename URV>
void
Hart<URV>::execVcompress_vm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  // vs2 is a mask register: elmul2 is 1

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;
  vecRegs_.setIthOpEmul(2, 1);  // EMUL of vs2 is 1.

  if (hasDestSourceOverlap(vd, group, vs1, group) or
      hasDestSourceOverlap(vd, group, vs2, 1) or di->isMasked() or start > 0)
    {
      postVecFail(di);  // Source/dest cannot overlap, must not be masked, 0 vstart.
      return;
    }

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vcompress_vm<uint8_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Half:  vcompress_vm<uint16_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word:  vcompress_vm<uint32_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word2: vcompress_vm<uint64_t>(vd, vs1, vs2, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vredop_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked,
		     std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> op)
{
  ELEM_TYPE e1 = 0, result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      vecRegs_.read(vs1, ix, group, e1);
      result = op(result, e1);
    }

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  unsigned destElems = vecRegs_.singleMax(vecRegs_.elemWidth());
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVredop_vs(const DecodedInst* di, OP op)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di))
    return;

  if (elems == 0)
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vredop_vs<int8_t> (vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vredop_vs<int16_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vredop_vs<int32_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vredop_vs<int64_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    default:  postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVredopu_vs(const DecodedInst* di, OP op)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRedOpVsEmul(di))
    return;

  if (elems == 0)
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vredop_vs<uint8_t> (vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Half:
      vredop_vs<uint16_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word:
      vredop_vs<uint32_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    case EW::Word2:
      vredop_vs<uint64_t>(vd, vs1, vs2, group, start, elems, masked, op);
      break;
    default:  postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVredsum_vs(const DecodedInst* di)
{
  execVredop_vs(di, std::plus());
}


template <typename URV>
void
Hart<URV>::execVredand_vs(const DecodedInst* di)
{
  execVredopu_vs(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVredor_vs(const DecodedInst* di)
{
  execVredopu_vs(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVredxor_vs(const DecodedInst* di)
{
  execVredopu_vs(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVredminu_vs(const DecodedInst* di)
{
  execVredopu_vs(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVredmin_vs(const DecodedInst* di)
{
  execVredop_vs(di, MyMin());
}


template <typename URV>
void
Hart<URV>::execVredmaxu_vs(const DecodedInst* di)
{
  execVredopu_vs(di, MyMax());
}


template <typename URV>
void
Hart<URV>::execVredmax_vs(const DecodedInst* di)
{
  execVredop_vs(di, MyMax());
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwredsum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  if (elems == 0)
    return;

  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE2X result = 0;
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result);
  
  ELEM_TYPE e1 = 0;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      vecRegs_.read(vs1, ix, group, e1);
      ELEM_TYPE2X e1dw = e1;
      result += e1dw;
    }

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  ElementWidth dsew{};
  if (not VecRegs::doubleSew(vecRegs_.elemWidth(), dsew))
    assert(0 && "Error: Assertion failed");

  unsigned destElems = vecRegs_.singleMax(dsew);
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
}


template <typename URV>
void
Hart<URV>::execVwredsumu_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  unsigned gx8 = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned elems = vecRegs_.elemCount();
  bool masked = di->isMasked();

  if (not checkWideRedOpVsEmul(di))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwredsum_vs<uint8_t> (vd, vs1, vs2, gx8, start, elems, masked); break;
    case EW::Half:  vwredsum_vs<uint16_t>(vd, vs1, vs2, gx8, start, elems, masked); break;
    case EW::Word:  vwredsum_vs<uint32_t>(vd, vs1, vs2, gx8, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwredsum_vs(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  unsigned gx8 = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned elems = vecRegs_.elemCount();
  bool masked = di->isMasked();

  if (not checkWideRedOpVsEmul(di))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwredsum_vs<int8_t> (vd, vs1, vs2, gx8, start, elems, masked); break;
    case EW::Half:  vwredsum_vs<int16_t>(vd, vs1, vs2, gx8, start, elems, masked); break;
    case EW::Word:  vwredsum_vs<int32_t>(vd, vs1, vs2, gx8, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename OP>
void
Hart<URV>::execVmop_mm(const DecodedInst* di, OP op)
{
  if (not checkSewLmulVstart(di))
    return;

  // Mask operation instructions (vmand.mm, vmor.mm, ...) cannot be masked.
  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.elemCount();
  unsigned vd = di->op0(), vs1 = di->op1(), vs2 = di->op2();

  if (start < elemCount)
    {
      unsigned count = vecRegs_.updateWholeMask() ? bitsPerReg : elemCount;

      for (unsigned ix = start; ix < count; ++ix)
	{
	  // Not masked. Tail elements computed.
	  bool in1 = false, in2 = false;
	  vecRegs_.readMaskRegister(vs1, ix, in1);
	  vecRegs_.readMaskRegister(vs2, ix, in2);
	  bool flag = op(unsigned(in1), unsigned(in2)) & 1;

	  vecRegs_.writeMaskRegister(vd, ix, flag);
	}

      // If not update-whole-mask and mask-agnostic-ones, fill tail with ones.
      if (vecRegs_.isTailAgnosticOnes())
	for (unsigned ix = count; ix < bitsPerReg; ++ix)
	  vecRegs_.writeMaskRegister(vd, ix, true);
    }

  vecRegs_.touchMask(vd);  // In case nothing was written.
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmand_mm(const DecodedInst* di)
{
  execVmop_mm(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVmnand_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitNand());
}


template <typename URV>
void
Hart<URV>::execVmandn_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitAndNot());
}


template <typename URV>
void
Hart<URV>::execVmxor_mm(const DecodedInst* di)
{
  execVmop_mm(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVmor_mm(const DecodedInst* di)
{
  execVmop_mm(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVmnor_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitNor());
}


template <typename URV>
void
Hart<URV>::execVmorn_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitOrNot());
}


template <typename URV>
void
Hart<URV>::execVmxnor_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitXnor());
}


template <typename URV>
void
Hart<URV>::execVcpop_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned rd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();

  uint32_t count = 0;
  for (uint32_t ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.isActive(vs1, ix))
        count++;
    }

  intRegs_.write(rd, count);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfirst_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned rd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();

  SRV first = -1;

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.isActive(vs1, ix))
	{
	  first = ix;
	  break;
	}
    }

  intRegs_.write(rd, first);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsbf_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.updateWholeMask() ? bitsPerReg : vecRegs_.elemCount();

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  if (start < vecRegs_.elemCount())
    {
      bool found = false;  // true if set bit is found in vs1

      for (uint32_t ix = start; ix < elemCount; ++ix)
	{
	  bool flag = false;
	  if (vecRegs_.isMaskDestActive(vd, ix, masked, elemCount, flag))
	    {
	      bool input = false;
	      if (ix < vecRegs_.elemCount())
		vecRegs_.readMaskRegister(vs1, ix, input);
	      found = found or input;
	      flag = not found;
	    }
	  vecRegs_.writeMaskRegister(vd, ix, flag);
	}

      vecRegs_.touchMask(vd);
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsif_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.updateWholeMask() ? bitsPerReg : vecRegs_.elemCount();

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  if (start < vecRegs_.elemCount())
    {
      bool found = false;  // true if set bit is found in vs1

      for (uint32_t ix = start; ix < elemCount; ++ix)
	{
	  bool flag = false;
	  if (vecRegs_.isMaskDestActive(vd, ix, masked, elemCount, flag))
	    {
	      bool input = false;
	      if (ix < vecRegs_.elemCount())
		vecRegs_.readMaskRegister(vs1, ix, input);
	      flag = not found;
	      found = found or input;
	    }
	  vecRegs_.writeMaskRegister(vd, ix, flag);
	}

      vecRegs_.touchMask(vd);
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsof_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.updateWholeMask() ? bitsPerReg : vecRegs_.elemCount();

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  if (start < vecRegs_.elemCount())
    {
      // True if masked-off elements should be set to 1.
      bool ones = vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes();

      bool found = false;  // true if set bit is found in vs1

      for (uint32_t ix = start; ix < elemCount; ++ix)
	{
	  bool flag = false;
	  bool active = vecRegs_.isMaskDestActive(vd, ix, masked, elemCount, flag);

	  bool input = false;
	  if (ix < vecRegs_.elemCount() and active)
	    vecRegs_.readMaskRegister(vs1, ix, input);

	  if (active)
	    vecRegs_.writeMaskRegister(vd, ix, false);
	  else if (ones)
	    vecRegs_.writeMaskRegister(vd, ix, true);

	  if (found or not input)
	    continue;

	  found = true;
	  vecRegs_.writeMaskRegister(vd, ix, true);
	}

      vecRegs_.touchMask(vd);  // In case nothing was written
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execViota_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();
  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  unsigned group = groupx8 <= 8 ? 1 : groupx8/8;

  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0 or
      (vs1 >= vd and vs1 < vd + group))
    {
      postVecFail(di);
      return;
    }

  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (masked and vd == 0)
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, groupx8, {vd}))
    return;

  unsigned sum = 0;

  unsigned destGroup = 8*group;

  elems = vecRegs_.elemMax();
  if (start < vecRegs_.elemCount())
    for (uint32_t ix = start; ix < elems; ++ix)
      {
	bool sourceSet = vecRegs_.isActive(vs1, ix);

	switch (sew)
	  {
	  case ElementWidth::Byte:
	    {
	      int8_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int8_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  case ElementWidth::Half:
	    {
	      int16_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int16_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  case ElementWidth::Word:
	    {
	      int32_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int32_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  case ElementWidth::Word2:
	    {
	      int64_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int64_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  default:
	    postVecFail(di);
	    return;
	  }
	if (sourceSet)
	  sum++;
      }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVid_v(const DecodedInst* di)
{
  // Spec does not mention vstart > 0. Got a clarification saying it
  // is ok not to take an exception in that case.
  uint32_t start = csRegs_.peekVstart();
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  elems = vecRegs_.elemMax();

  if ((masked and vd == 0) or di->op1() != 0)
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {vd}))
    return;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  elems = vecRegs_.elemMax();
  if (start < vecRegs_.elemCount())
    for (uint32_t ix = start; ix < elems; ++ix)
      switch (sew)
	{
	case ElementWidth::Byte:
	  {
	    uint8_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint8_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;
	case ElementWidth::Half:
	  {
	    uint16_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint16_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;
	case ElementWidth::Word:
	  {
	    uint32_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint32_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;
	case ElementWidth::Word2:
	  {
	    uint64_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint64_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;

	default: postVecFail(di); return;
	}

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vslideup(unsigned vd, unsigned vs1, URV amount, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;

  if (start >= vecRegs_.elemCount())
    return;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          if (ix >= amount)
            {
              unsigned from = ix - amount;
              vecRegs_.read(vs1, from, group, e1);
              dest = e1;
            }
	}

      // When the start element is max(start, amount), there is effectively no "body" if
      // amount >= vl. However, we need to account for the tail.
      if (ix >= vecRegs_.elemCount() or
          (ix < vecRegs_.elemCount() and ix >= std::max(URV(start), amount)))
        vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVslideup_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  URV amount = intRegs_.read(rs2);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vslideup<uint8_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half:  vslideup<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word:  vslideup<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2: vslideup<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVslideup_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), imm = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  URV amount = imm;

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vslideup<uint8_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half:  vslideup<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word:  vslideup<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2: vslideup<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVslide1up_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (hasDestSourceOverlap(vd, group, vs1, group))
    {
      postVecFail(di);  // Source/dest vecs cannot overlap
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  if (start < vecRegs_.elemCount())
    {
      URV amount = 1;

      // Sign extend scalar value
      SRV replacement = SRV(intRegs_.read(rs2));

      switch (sew)
	{
	case ElementWidth::Byte:
	  {
	    vslideup<uint8_t>(vd, vs1, amount, group, start, elems, masked);
	    auto dest = int8_t{};
	    if (vecRegs_.isDestActive(vd, 0, group, masked, dest))
	      dest = int8_t(replacement);
	    if (not start)
	      vecRegs_.write(vd, 0, group, dest);
	  }
	  break;

	case ElementWidth::Half:
	  {
	    vslideup<uint16_t>(vd, vs1, amount, group, start, elems, masked);
	    auto dest = int16_t{};
	    if (vecRegs_.isDestActive(vd, 0, group, masked, dest) and not start)
	      dest = int16_t(replacement);
	    if (not start)
	      vecRegs_.write(vd, 0, group, dest);
	  }
	  break;

	case ElementWidth::Word:
	  {
	    vslideup<uint32_t>(vd, vs1, amount, group, start, elems, masked);
	    auto dest = int32_t{};
	    if (vecRegs_.isDestActive(vd, 0, group, masked, dest) and not start)
	      dest = int32_t(replacement);
	    if (not start)
	      vecRegs_.write(vd, 0, group, dest);
	  }
	  break;

	case ElementWidth::Word2:
	  {
	    vslideup<uint64_t>(vd, vs1, amount, group, start, elems, masked);
	    auto dest = int64_t{};
	    if (vecRegs_.isDestActive(vd, 0, group, masked, dest) and not start)
	      dest = int64_t(replacement);
	    if (not start)
	      vecRegs_.write(vd, 0, group, dest);
	  }
	  break;

	default:  postVecFail(di); return;
	}
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vslidedown(unsigned vd, unsigned vs1, URV amount, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;

  if (start >= vecRegs_.elemCount())
    return;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  e1 = 0;
	  if (amount < vecRegs_.bytesInRegisterFile())
	    {
	      URV from = ix + amount;
	      if (vecRegs_.isValidIndex(vs1, from, group, sizeof(e1)))
		vecRegs_.read(vs1, from, group, e1);
	    }
	  dest = e1;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVslidedown_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  URV amount = intRegs_.read(rs2);

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vslidedown<uint8_t> (vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half:   vslidedown<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word:   vslidedown<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2:  vslidedown<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVslidedown_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), imm = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  URV amount = imm;

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vslidedown<uint8_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Half:  vslidedown<uint16_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word:  vslidedown<uint32_t>(vd, vs1, amount, group, start, elems, masked); break;
    case EW::Word2: vslidedown<uint64_t>(vd, vs1, amount, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVslide1down_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  if (start < vecRegs_.elemCount())
    {
      URV amount = 1;

      // Sign extend scalar value
      SRV replacement = SRV(intRegs_.read(rs2));

      unsigned slot = vecRegs_.elemCount() - 1;
      switch (sew)
	{
	case ElementWidth::Byte:
	  vslidedown<uint8_t>(vd, vs1, amount, group, start, elems, masked);
	  if (not masked or vecRegs_.isActive(0, slot))
	    vecRegs_.write(vd, slot, group, int8_t(replacement));
	  break;

	case ElementWidth::Half:
	  vslidedown<uint16_t>(vd, vs1, amount, group, start, elems, masked);
	  if (not masked or vecRegs_.isActive(0, slot))
	    vecRegs_.write(vd, slot, group, int16_t(replacement));
	  break;

	case ElementWidth::Word:
	  vslidedown<uint32_t>(vd, vs1, amount, group, start, elems, masked);
	  if (not masked or vecRegs_.isActive(0, slot))
	    vecRegs_.write(vd, slot, group, int32_t(replacement));
	  break;

	case ElementWidth::Word2:
	  vslidedown<uint64_t>(vd, vs1, amount, group, start, elems, masked);
	  if (not masked or vecRegs_.isActive(0, slot))
	    vecRegs_.write(vd, slot, group, int64_t(replacement));
	  break;

	default:  postVecFail(di); return;
	}
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmul_vv(const DecodedInst* di)
{
  execVop_vv(di, std::multiplies());
}


template <typename URV>
void
Hart<URV>::execVmul_vx(const DecodedInst* di)
{
  execVop_vx(di, std::multiplies());
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulh_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  mulh<ELEM_TYPE>(e1, e2, dest);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmulh_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmulh_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmulh_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmulh_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmulh_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulh_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;
  auto e2 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs2)))); // Sign extend.

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  mulh(e1, e2, dest);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmulh_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmulh_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vmulh_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmulh_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmulh_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmulhu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmulh_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmulh_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmulh_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmulh_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulhu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;
  auto e2 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs2))));

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  mulh(e1, e2, dest);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmulhu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmulhu_vx<uint8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vmulhu_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmulhu_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmulhu_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulhsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  using U_ELEM_TYPE = std::make_unsigned_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0, dest = 0;
  U_ELEM_TYPE e2 = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  mulhsu(e1, e2, dest);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmulhsu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmulhsu_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmulhsu_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmulhsu_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmulhsu_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmulhsu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  using U_ELEM_TYPE = std::make_unsigned_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0, dest = 0;
  auto e2 = U_ELEM_TYPE(int64_t(SRV(intRegs_.read(rs2))));  // Sign extend.

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  mulhsu(e1, e2, dest);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmulhsu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmulhsu_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vmulhsu_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vmulhsu_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vmulhsu_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = (e1 * dest) + e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmadd_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmadd_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadd_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2 = 0, dest = 0;
  auto e1 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs1))));

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(v2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = (e1 * dest) + e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmadd_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // We don't want to check rs1, we pass vs2 instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vs2, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmadd_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmadd_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmadd_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmadd_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = -(e1 * dest) + e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVnmsub_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnmsub_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnmsub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnmsub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsub_vx(unsigned vd, unsigned rs1, unsigned v2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2 = 0, dest = 0;
  auto e1 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs1))));

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(v2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = -(e1 * dest) + e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVnmsub_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs2, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnmsub_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnmsub_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnmsub_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsub_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = (e1 * e2) + dest;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmacc_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmacc_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmacc_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmacc_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmacc_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmacc_vx(unsigned vd, unsigned rs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2 = 0, dest = 0;
  auto e1 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs1))));  // Sign extend

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);

	  dest = (e1 * e2) + dest;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmacc_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check rs1, we pass vs2 instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vs2, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmacc_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vmacc_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vmacc_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vmacc_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = -(e1 * e2) + dest;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVnmsac_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnmsac_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnmsac_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnmsac_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsac_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnmsac_vx(unsigned vd, unsigned rs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2 = 0, dest = 0;
  auto e1 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs1))));  // Sign extend.

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = -(e1 * e2) + dest;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVnmsac_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check rs1, we pass vs2 instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vs2, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vnmsac_vx<int8_t> (vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vnmsac_vx<int16_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vnmsac_vx<int32_t>(vd, rs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vnmsac_vx<int64_t>(vd, rs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE_X2 = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0, e2 = 0;
  ELEM_TYPE_X2 dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = ELEM_TYPE_X2(e1);
	  dest *= e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmulu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmulu_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmulu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmulu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmulu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE_X2 = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_X2 dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = ELEM_TYPE_X2(e1);
	  dest *= e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmulu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmulu_vx<uint8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwmulu_vx<uint16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwmulu_vx<uint32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwmulu_vx<uint64_t>(vd, vs1, int64_t(e2), group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE_X2 = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0, e2 = 0;
  ELEM_TYPE_X2 dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = ELEM_TYPE_X2(e1);
	  dest *= ELEM_TYPE_X2(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmul_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmul_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmul_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmul_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmul_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmul_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE_X2 = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_X2 dest = 0;
  ELEM_TYPE_X2 e2Wide(e2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = ELEM_TYPE_X2(e1);
	  dest *= e2Wide;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmul_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmul_vx<int8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwmul_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwmul_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwmul_vx<int64_t>(vd, vs1, int64_t(e2), group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE_X2 = makeDoubleWide_t<ELEM_TYPE>;
  using ELEM_TYPE_U  = std::make_unsigned_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_U e2u = 0;
  ELEM_TYPE_X2 dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2u);
	  dest = ELEM_TYPE_X2(e1);
	  ELEM_TYPE_X2 tmp2(e2u);
	  dest *= tmp2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmulsu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmulsu_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmulsu_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmulsu_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmulsu_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmulsu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE_X2 = makeDoubleWide_t<ELEM_TYPE>;
  using ELEM_TYPE_U  = std::make_unsigned_t<ELEM_TYPE>;

  ELEM_TYPE e1 = 0;
  ELEM_TYPE_X2 dest = 0;
  auto e2u = ELEM_TYPE_U(e2);
  ELEM_TYPE_X2 e2Wide(e2u);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = ELEM_TYPE_X2(e1);
	  dest *= e2Wide;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmulsu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmulsu_vx<int8_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Half:  vwmulsu_vx<int16_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word:  vwmulsu_vx<int32_t>(vd, vs1, e2, group, start, elems, masked); break;
    case EW::Word2: vwmulsu_vx<int64_t>(vd, vs1, int64_t(e2), group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type
  unsigned wideGroup = group*2;

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, wideGroup, dest);
	  dest += DWT(e1) * DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmaccu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmacc_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmacc_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmacc_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmacc_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccu_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                      unsigned start, unsigned elems, bool masked)
{
  using DWT  = makeDoubleWide_t<ELEM_TYPE>;  // Double wide type
  using SDWT = std::make_signed_t<DWT>;      // Signed double wide type
  unsigned wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT dest = 0;
  SDWT sde1 = SDWT(e1);  // sign extend (per spec)
  DWT de1 = sde1;  // And make unsigned

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, wideGroup, dest);
	  dest += de1 * DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmaccu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e1 = uint64_t(int64_t(SRV(intRegs_.read(rs1))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmaccu_vx<uint8_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmaccu_vx<uint16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmaccu_vx<uint32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccu_vx<uint64_t>(vd, int64_t(e1), vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwmacc_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();

  // Double wide legal.
  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmacc_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmacc_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmacc_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmacc_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmacc_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type
  unsigned wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT dest = 0;
  DWT de1 = DWT(e1);  // sign extend

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, wideGroup, dest);
	  dest += de1 * DWT(e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmacc_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e1 = int64_t(SRV(intRegs_.read(rs1)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmacc_vx<int8_t> (vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmacc_vx<int16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmacc_vx<int32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmacc_vx<int64_t>(vd, int64_t(e1), vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccsu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  using DWT  = makeDoubleWide_t<ELEM_TYPE>;      // Double wide type
  using DWTU = std::make_unsigned_t<DWT>;        // Double wide type unsigned
  using SWTU = std::make_unsigned_t<ELEM_TYPE>;  // Single wide type unsigned

  unsigned wideGroup = group*2;

  ELEM_TYPE e1 = 0, e2 = 0;
  DWT dest = 0, temp = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, wideGroup, dest);
	  mulsu(DWT(e1), DWTU(SWTU(e2)), temp);
	  dest += temp;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmaccsu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmaccsu_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmaccsu_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmaccsu_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccsu_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccsu_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  using DWT  = makeDoubleWide_t<ELEM_TYPE>;      // Double wide type
  using DWTU = std::make_unsigned_t<DWT>;        // Double wide type unsigned
  using SWTU = std::make_unsigned_t<ELEM_TYPE>;  // Single wide type unsigned

  unsigned wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT de1 = DWT(e1);  // Sign extend.
  DWT dest = 0, temp = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, wideGroup, dest);
	  mulsu(de1, DWTU(SWTU(e2)), temp);
	  dest += temp;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmaccsu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  auto e1 = int64_t(SRV(intRegs_.read(rs1)));  // Sign extend.

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmaccsu_vx<int8_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmaccsu_vx<int16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmaccsu_vx<int32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccsu_vx<int64_t>(vd, int64_t(e1), vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vwmaccus_vx(unsigned vd, ELEM_TYPE e1, unsigned vs2, unsigned group,
                       unsigned start, unsigned elems, bool masked)
{
  using DWT  = makeDoubleWide_t<ELEM_TYPE>;      // Double wide type
  using DWTU = std::make_unsigned_t<DWT>;        // Double wide type unsigned
  using SWTU = std::make_unsigned_t<ELEM_TYPE>;  // Single wide type unsigned

  unsigned wideGroup = group*2;

  ELEM_TYPE e2 = 0;
  DWT de1u = DWTU(SWTU(e1));  // Sign extend.
  DWT dest = 0, temp = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), wideGroup);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, wideGroup, dest);
	  mulsu(DWT(e2), de1u, temp);
	  dest += temp;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwmaccus_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  rs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e1 = uint64_t(int64_t(SRV(intRegs_.read(rs1))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwmaccus_vx<int8_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Half:  vwmaccus_vx<int16_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word:  vwmaccus_vx<int32_t>(vd, e1, vs2, group, start, elems, masked); break;
    case EW::Word2: vwmaccus_vx<int64_t>(vd, e1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdivu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = ~ ELEM_TYPE(0); // divide by zero result
	  if (e2 != ELEM_TYPE(0))
	    dest = e1 / e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVdivu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vdivu_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vdivu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vdivu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vdivu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdivu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;
  using SELEM_TYPE = std::make_signed_t<ELEM_TYPE>;

  // Spec (sep 24, 2020) says scalar register value should be sign extended.
  auto e2 = ELEM_TYPE(SELEM_TYPE(SRV(intRegs_.read(rs2))));

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = ~ ELEM_TYPE(0); // divide by zero result
	  if (e2 != ELEM_TYPE(0))
	    dest = e1 / e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVdivu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vdivu_vx<uint8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vdivu_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vdivu_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vdivu_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdiv_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;
  auto minInt = ELEM_TYPE(1) << (elemBits - 1);
  auto negOne = ELEM_TYPE(-1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = negOne; // Divide by zero result
	  if (e2 != ELEM_TYPE(0))
	    {
	      if (e1 == minInt and e2 == negOne)
		dest = e1;
	      else
		dest = e1 / e2;
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVdiv_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vdiv_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vdiv_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vdiv_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vdiv_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vdiv_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE e1 = 0, dest = 0;
  auto e2 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs2))));  // Sign extend.
  auto minInt = ELEM_TYPE(1) << (elemBits - 1);
  auto negOne = ELEM_TYPE(-1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = negOne; // Divide by zero result
	  if (e2 != ELEM_TYPE(0))
	    {
	      if (e1 == minInt and e2 == negOne)
		dest = e1;
	      else
		dest = e1 / e2;
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVdiv_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vdiv_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vdiv_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vdiv_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vdiv_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vremu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1; // divide by zero result
	  if (e2 != ELEM_TYPE(0))
	    dest = e1 % e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVremu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vremu_vv<uint8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vremu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vremu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vremu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vremu_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;

  // Spec (sep 24, 2020) says scalar register value should be sign extended.
  auto e2 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs2))));

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = e1; // divide by zero result
	  if (e2 != ELEM_TYPE(0))
	    dest = e1 % e2;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVremu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vremu_vx<uint8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vremu_vx<uint16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vremu_vx<uint32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vremu_vx<uint64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrem_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  int elemBits = integerWidth<ELEM_TYPE> ();
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;
  auto minInt = ELEM_TYPE(1) << (elemBits - 1);
  auto negOne = ELEM_TYPE(-1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = e1; // Divide by zero remainder
	  if (e2 != ELEM_TYPE(0))
	    {
	      if (e1 == minInt and e2 == negOne)
		dest = 0;
	      else
		dest = e1 % e2;
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVrem_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vrem_vv<int8_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:  vrem_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vrem_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vrem_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vrem_vx(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  unsigned elemBits = integerWidth<ELEM_TYPE> ();

  ELEM_TYPE e1 = 0, dest = 0;
  auto e2 = ELEM_TYPE(int64_t(SRV(intRegs_.read(rs2))));
  auto minInt = ELEM_TYPE(1) << (elemBits - 1);
  auto negOne = ELEM_TYPE(-1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = e1; // Divide by zero remainder
	  if (e2 != ELEM_TYPE(0))
	    {
	      if (e1 == minInt and e2 == negOne)
		dest = 0;
	      else
		dest = e1 % e2;
	    }
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVrem_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vrem_vx<int8_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Half:  vrem_vx<int16_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word:  vrem_vx<int32_t>(vd, vs1, rs2, group, start, elems, masked); break;
    case EW::Word2: vrem_vx<int64_t>(vd, vs1, rs2, group, start, elems, masked); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE, typename FROM_TYPE>
void
Hart<URV>::vsext(unsigned vd, unsigned vs1, unsigned group, unsigned fromGroup,
                 unsigned start, unsigned elems, bool masked)
{
  FROM_TYPE e1 = 0;
  ELEM_TYPE dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, fromGroup, e1);
	  dest = e1;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVsext_vf2(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/2;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not VecRegs::groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 2 and (vs1 % (eg/2)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/2; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 2? eg/2 : 1);  // Track operand group for logging

  using EW = ElementWidth;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: eew = EW::Byte; break;
    case EW::Word: eew = EW::Half; break;
    case EW::Word2: eew = EW::Word; break;
    case EW::Word4: eew = EW::Word2; break;
    case EW::Word8: eew = EW::Word4; break;
    case EW::Word16: eew = EW::Word8; break;
    case EW::Word32: eew = EW::Word16; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemMax(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Half:  vsext<int16_t,int8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word:  vsext<int32_t, int16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vsext<int64_t, int32_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsext_vf4(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/4;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not VecRegs::groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 4 and (vs1 % (eg/4)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/4; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 4? eg/4 : 1);  // Track operand group for logging

  using EW = ElementWidth;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: // Fall-through to invalid case
    case EW::Half: postVecFail(di); return;
    case EW::Word: eew = EW::Byte; break;
    case EW::Word2: eew = EW::Half; break;
    case EW::Word4: eew = EW::Word; break;
    case EW::Word8: eew = EW::Word2; break;
    case EW::Word16: eew = EW::Word4; break;
    case EW::Word32: eew = EW::Word8; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemMax(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Word:  vsext<int32_t, int8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vsext<int64_t, int16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Half:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsext_vf8(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/8;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not VecRegs::groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 8 and (vs1 % (eg/8)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/8; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 8? eg/8 : 1);  // Track operand group for logging

  using EW = ElementWidth;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte:  // Fall-through to invalid case
    case EW::Half:  // Fall-through to invalid case
    case EW::Word: postVecFail(di); return;
    case EW::Word2: eew = EW::Byte; break;
    case EW::Word4: eew = EW::Half; break;
    case EW::Word8: eew = EW::Word; break;
    case EW::Word16: eew = EW::Word2; break;
    case EW::Word32: eew = EW::Word4; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemMax(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Word2: vsext<int64_t, int8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Half:  // Fall-through to invalid case
    case EW::Word:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE, typename FROM_TYPE>
void
Hart<URV>::vzext(unsigned vd, unsigned vs1, unsigned group, unsigned fromGroup,
                 unsigned start, unsigned elems, bool masked)
{
  FROM_TYPE e1 = 0;
  ELEM_TYPE dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, fromGroup, e1);
	  dest = e1;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVzext_vf2(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/2;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not VecRegs::groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 2 and (vs1 % (eg/2)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/2; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 2? eg/2 : 1);  // Track operand group for logging

  using EW = ElementWidth;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: postVecFail(di); return;
    case EW::Half: eew = EW::Byte; break;
    case EW::Word: eew = EW::Half; break;
    case EW::Word2: eew = EW::Word; break;
    case EW::Word4: eew = EW::Word2; break;
    case EW::Word8: eew = EW::Word4; break;
    case EW::Word16: eew = EW::Word8; break;
    case EW::Word32: eew = EW::Word16; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemMax(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Half:  vzext<uint16_t,uint8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word:  vzext<uint32_t, uint16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vzext<uint64_t, uint32_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVzext_vf4(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/4;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not VecRegs::groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 4 and (vs1 % (eg/4)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/4; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 4? eg/4 : 1);  // Track operand group for logging

  using EW = ElementWidth;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: // Fall-through to invalid case
    case EW::Half: postVecFail(di); return;
    case EW::Word: eew = EW::Byte; break;
    case EW::Word2: eew = EW::Half; break;
    case EW::Word4: eew = EW::Word; break;
    case EW::Word8: eew = EW::Word2; break;
    case EW::Word16: eew = EW::Word4; break;
    case EW::Word32: eew = EW::Word8; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemMax(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Word:  vzext<uint32_t, uint8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Word2: vzext<uint64_t, uint16_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Half:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVzext_vf8(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned fromGroup = group/8;
  if (fromGroup == 0)
    {
      postVecFail(di);
      return;
    }
  GroupMultiplier emul = GroupMultiplier::One;
  if (not VecRegs::groupNumberX8ToSymbol(fromGroup, emul))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned eg = group >= 8 ? group / 8 : 1;
  if (vd % eg)
    {
      postVecFail(di);
      return;
    }
  if (eg > 8 and (vs1 % (eg/8)))
    {
      postVecFail(di);
      return;
    }
  unsigned dw = vecRegs_.elemWidthInBits();  // destination width
  unsigned sw = dw/8; // source width.
  if (not checkDestSourceOverlap(vd, dw, group, vs1, sw, fromGroup))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(eg, eg > 8? eg/8 : 1);  // Track operand group for logging

  using EW = ElementWidth;

  EW sew = vecRegs_.elemWidth();
  EW eew = sew;  // Effective elem width of source.
  switch (sew)
    {
    case EW::Byte: // Fall-through to invalid case
    case EW::Half: // Fall-through to invalid case
    case EW::Word: postVecFail(di); return;
    case EW::Word2: eew = EW::Byte; break;
    case EW::Word4: eew = EW::Half; break;
    case EW::Word8: eew = EW::Word; break;
    case EW::Word16: eew = EW::Word2; break;
    case EW::Word32: eew = EW::Word4; break;
    }
  if (not vecRegs_.legalConfig(eew, emul))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned elems = vecRegs_.elemMax(), start = csRegs_.peekVstart();

  switch (sew)
    {
    case EW::Word2: vzext<uint64_t, uint8_t>(vd, vs1, group, fromGroup, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Half:  // Fall-through to invalid case
    case EW::Word:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vadc_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned vcin,
                    unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1 + e2;
	  if (vecRegs_.isActive(vcin, ix))
	    dest += ELEM_TYPE(1);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vadc_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned vcin,
                    unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = e1 + e2;
	  if (vecRegs_.isActive(vcin, ix))
	    dest += ELEM_TYPE(1);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsbc_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned vbin,
                    unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1 - e2;
	  if (vecRegs_.isActive(vbin, ix))
	    dest -= ELEM_TYPE(1);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsbc_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned vbin,
                    unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = e1 - e2;
	  if (vecRegs_.isActive(vbin, ix))
	    dest -= ELEM_TYPE(1);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadc_vvm(unsigned vcout, unsigned vs1, unsigned vs2, bool carry, unsigned vcin,
                     unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool cout = false;
      if (vecRegs_.isMaskDestActive(vcout, ix, false /*masked*/, cout))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1 + e2;
	  bool cin = carry and vecRegs_.isActive(vcin, ix);
	  if (cin)
	    dest += ELEM_TYPE(1);

	  cout = cin? dest <= e1 : dest < e1;
	}
      vecRegs_.writeMaskRegister(vcout, ix, cout);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmadc_vxm(unsigned vcout, unsigned vs1, ELEM_TYPE e2, bool carry, unsigned vcin,
                     unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, dest = 0;

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool cout = false;
      if (vecRegs_.isMaskDestActive(vcout, ix, false /*masked*/, cout))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = e1 + e2;
	  bool cin = carry and vecRegs_.isActive(vcin, ix);
	  if (cin)
	    dest += ELEM_TYPE(1);

	  cout = cin? dest <= e1 : dest < e1;
	}
      vecRegs_.writeMaskRegister(vcout, ix, cout);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmsbc_vvm(unsigned vbout, unsigned vs1, unsigned vs2, bool borrow, unsigned vbin,
                     unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool bout = false;
      if (vecRegs_.isMaskDestActive(vbout, ix, false /*masked*/, bout))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1 - e2;
	  bout = e1 < e2;

	  if (borrow and vecRegs_.isActive(vbin, ix))
	    {
	      dest -= ELEM_TYPE(1);
	      bout = e1 <= e2;
	    }
	}
      vecRegs_.writeMaskRegister(vbout, ix, bout);
    }
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmsbc_vxm(unsigned vbout, unsigned vs1, ELEM_TYPE e2, bool borrow, unsigned vbin,
                     unsigned group, unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, dest = 0;

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool bout = false;
      if (vecRegs_.isMaskDestActive(vbout, ix, false /*masked*/, bout))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = e1 - e2;
	  bout = e1 < e2;

	  if (borrow and vecRegs_.isActive(vbin, ix))
	    {
	      dest -= ELEM_TYPE(1);
	      bout = e1 <= e2;
	    }
	}
      vecRegs_.writeMaskRegister(vbout, ix, bout);
    }
}


template <typename URV>
void
Hart<URV>::execVadc_vvm(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vcin = 0;

  // cannot overlap vcin, unmasked verion reserved
  if (vd == vcin or vs1 == vcin or vs2 == vcin or not masked)
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vadc_vvm<uint8_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    case EW::Half:  vadc_vvm<uint16_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    case EW::Word:  vadc_vvm<uint32_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    case EW::Word2: vadc_vvm<uint64_t>(vd, vs1, vs2, vcin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVadc_vxm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vcin = 0;

  // cannot overlap vcin, unmasked verion reserved
  if (vd == vcin or vs1 == vcin or not masked)
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vadc_vxm<uint8_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Half:  vadc_vxm<uint16_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Word:  vadc_vxm<uint32_t>(vd, vs1, int32_t(e2), vcin, group, start, elems); break;
    case EW::Word2: vadc_vxm<uint64_t>(vd, vs1, int64_t(e2), vcin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVadc_vim(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vcin = 0;

  // cannot overlap vcin, unmasked version reserved
  if (vd == vcin or vs1 == vcin or not masked)
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  SRV e2 = di->op2As<int32_t>();

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vadc_vxm<uint8_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Half:  vadc_vxm<uint16_t>(vd, vs1, e2, vcin, group, start, elems); break;
    case EW::Word:  vadc_vxm<uint32_t>(vd, vs1, int32_t(e2), vcin, group, start, elems); break;
    case EW::Word2: vadc_vxm<uint64_t>(vd, vs1, int64_t(e2), vcin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsbc_vvm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vbin = 0;

  // cannot overlap borrow-in, unmasked version reserved
  if (vd == vbin or vs1 == vbin or vs2 == vbin or not masked)
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vsbc_vvm<uint8_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    case EW::Half:  vsbc_vvm<uint16_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    case EW::Word:  vsbc_vvm<uint32_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    case EW::Word2: vsbc_vvm<uint64_t>(vd, vs1, vs2, vbin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsbc_vxm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vbin = 0;

  // cannot overlap borrow-in, unmasked version reserved
  if (vd == vbin or vs1 == vbin or not masked)
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vsbc_vxm<uint8_t>(vd, vs1, e2, vbin, group, start, elems); break;
    case EW::Half:  vsbc_vxm<uint16_t>(vd, vs1, e2, vbin, group, start, elems); break;
    case EW::Word:  vsbc_vxm<uint32_t>(vd, vs1, int32_t(e2), vbin, group, start, elems); break;
    case EW::Word2: vsbc_vxm<uint64_t>(vd, vs1, int64_t(e2), vbin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmadc_vvm(const DecodedInst* di)
{
  bool carry = di->isMasked();
  unsigned vcout = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vcin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.bitsPerRegister() : vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // cannot overlap vcin if vcin is used
  if (carry and (vs1 == vcin or vs2 == vcin))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecMaskInst(di, vcout, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmadc_vvm<uint8_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    case EW::Half:  vmadc_vvm<uint16_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    case EW::Word:  vmadc_vvm<uint32_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    case EW::Word2: vmadc_vvm<uint64_t>(vcout, vs1, vs2, carry, vcin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmadc_vxm(const DecodedInst* di)
{
  bool carry = di->isMasked();
  unsigned vcout = di->op0(),  vs1 = di->op1(),  vcin = 0;
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.bitsPerRegister() : vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vcout, vs1, group))
    return;

  // cannot overlap vcin if vcin is used
  if (carry and (vs1 == vcin))
    {
      postVecFail(di);
      return;
    }

  SRV e2 = SRV(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmadc_vxm<uint8_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Half:  vmadc_vxm<uint16_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Word:  vmadc_vxm<uint32_t>(vcout, vs1, int32_t(e2), carry, vcin, group, start, elems); break;
    case EW::Word2: vmadc_vxm<uint64_t>(vcout, vs1, int64_t(e2), carry, vcin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmadc_vim(const DecodedInst* di)
{
  bool carry = di->isMasked();
  unsigned vcout = di->op0(),  vs1 = di->op1(),  vcin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.bitsPerRegister() : vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecMaskInst(di, vcout, vs1, group))
    return;

  // cannot overlap vcin if vcin is used
  if (carry and (vs1 == vcin))
    {
      postVecFail(di);
      return;
    }

  SRV e2 = di->op2As<int32_t>();

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmadc_vxm<uint8_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Half:  vmadc_vxm<uint16_t>(vcout, vs1, e2, carry, vcin, group, start, elems); break;
    case EW::Word:  vmadc_vxm<uint32_t>(vcout, vs1, int32_t(e2), carry, vcin, group, start, elems); break;
    case EW::Word2: vmadc_vxm<uint64_t>(vcout, vs1, int64_t(e2), carry, vcin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsbc_vvm(const DecodedInst* di)
{
  bool borrow = di->isMasked();
  unsigned vbout = di->op0(),  vs1 = di->op1(),  vs2 = di->op2(),  vbin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.bitsPerRegister() : vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // cannot overlap vbin if vbin is used
  if (borrow and (vs1 == vbin or vs2 == vbin))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecMaskInst(di, vbout, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmsbc_vvm<uint8_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    case EW::Half:  vmsbc_vvm<uint16_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    case EW::Word:  vmsbc_vvm<uint32_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    case EW::Word2: vmsbc_vvm<uint64_t>(vbout, vs1, vs2, borrow, vbin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsbc_vxm(const DecodedInst* di)
{
  bool borrow = di->isMasked();
  unsigned vbout = di->op0(),  vs1 = di->op1(),  vbin = 0;

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.bitsPerRegister() : vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // cannot overlap vbin if vbin is used
  if (borrow and vs1 == vbin)
    {
      postVecFail(di);
      return;
    }

  if (not checkVecMaskInst(di, vbout, vs1, group))
    return;

  SRV e2 = SRV(intRegs_.read(di->op2()));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmsbc_vxm<uint8_t>(vbout, vs1, e2, borrow, vbin, group, start, elems); break;
    case EW::Half:  vmsbc_vxm<uint16_t>(vbout, vs1, e2, borrow, vbin, group, start, elems); break;
    case EW::Word:  vmsbc_vxm<uint32_t>(vbout, vs1, int32_t(e2), borrow, vbin, group, start, elems); break;
    case EW::Word2: vmsbc_vxm<uint64_t>(vbout, vs1, int64_t(e2), borrow, vbin, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmerge_vvm(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = vecRegs_.isActive(0, ix) ? e2 : e1;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmerge_vvm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();

  // Must be masked, dest must not overlap v0. Source must not overlap v0.
  if (not di->isMasked() or vd == 0 or vs1 == 0 or vs2 == 0)
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmerge_vvm<int8_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Half:  vmerge_vvm<int16_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word:  vmerge_vvm<int32_t>(vd, vs1, vs2, group, start, elems); break;
    case EW::Word2: vmerge_vvm<int64_t>(vd, vs1, vs2, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmerge_vxm(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
		      unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, dest = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = vecRegs_.isActive(0, ix) ? e2 : e1;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmerge_vxm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned start = csRegs_.peekVstart();

  // Must be masked, dest must not overlap v0. Source must not overlap v0.
  if (not di->isMasked() or vd == 0 or vs1 == 0)
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmerge_vxm<int8_t>(vd, vs1, e2, group, start, elems); break;
    case EW::Half:  vmerge_vxm<int16_t>(vd, vs1, e2, group, start, elems); break;
    case EW::Word:  vmerge_vxm<int32_t>(vd, vs1, e2, group, start, elems); break;
    case EW::Word2: vmerge_vxm<int64_t>(vd, vs1, e2, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmerge_vim(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(), vs1 = di->op1(), start = csRegs_.peekVstart();
  auto imm = di->op2As<int32_t>();

  // Must be masked, dest must not overlap v0. Source must not overlap v0.
  if (not di->isMasked() or vd == 0 or vs1 == 0)
    {
      postVecFail(di);
      return;
    }

  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmerge_vxm<int8_t>(vd, vs1, imm, group, start, elems); break;
    case EW::Half:  vmerge_vxm<int16_t>(vd, vs1, imm, group, start, elems); break;
    case EW::Word:  vmerge_vxm<int32_t>(vd, vs1, imm, group, start, elems); break;
    case EW::Word2: vmerge_vxm<int64_t>(vd, vs1, imm, group, start, elems); break;
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmv_x_s(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned rd = di->op0(), vs1 = di->op1(), groupX8 = 8;

  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }
  vecRegs_.setOpEmul(1, eg);  // Track operand group for logging

  ElementWidth sew = vecRegs_.elemWidth();

  switch (sew)
    {
    case ElementWidth::Byte:
      {
        int8_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
      }
      break;

    case ElementWidth::Half:
      {
        int16_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
      }
      break;

    case ElementWidth::Word:
      {
        int32_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
      }
      break;

    case ElementWidth::Word2:
      {
        int64_t val = 0;
        vecRegs_.read(vs1, 0, groupX8, val);
        intRegs_.write(rd, SRV(val));
      }
      break;

    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmv_s_x(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), rs1 = di->op1(), groupX8 = 8, start = csRegs_.peekVstart();
  ElementWidth sew = vecRegs_.elemWidth();
  SRV val = intRegs_.read(rs1);

  bool setTail = vecRegs_.isTailAgnostic() and vecRegs_.isTailAgnosticOnes();
  unsigned tail = vecRegs_.vlmax(sew, GroupMultiplier::One);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      if (start < vecRegs_.elemCount())
	{
	  vecRegs_.write(vd, 0, groupX8, int8_t(val));
	  if (setTail)
	    for (unsigned i = 1; i < tail; ++i)
	      vecRegs_.write(vd, i, groupX8, uint8_t(~0));
	}
      break;
    case EW::Half:
      if (start < vecRegs_.elemCount())
	{
	  vecRegs_.write(vd, 0, groupX8, int16_t(val));
	  if (setTail)
	    for (unsigned i = 1; i < tail; ++i)
	      vecRegs_.write(vd, i, groupX8, uint16_t(~0));
	}
      break;
    case EW::Word:
      if (start < vecRegs_.elemCount())
	{
	  vecRegs_.write(vd, 0, groupX8, int32_t(val));
	  if (setTail)
	    for (unsigned i = 1; i < tail; ++i)
	      vecRegs_.write(vd, i, groupX8, uint32_t(~0));
	}
      break;
    case EW::Word2:
      if (start < vecRegs_.elemCount())
	{
	  vecRegs_.write(vd, 0, groupX8, int64_t(val));
	  if (setTail)
	    for (unsigned i = 1; i < tail; ++i)
	      vecRegs_.write(vd, i, groupX8, uint64_t(~uint64_t(0)));
	}
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmv_v_v(unsigned vd, unsigned vs1, unsigned group,
                   unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = 0, dest = 0;

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, group, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = e1;
	}
      vecRegs_.write(vd, ix, group, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmv_v_v(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1();

  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmv_v_v<int8_t>(vd, vs1, group, start, elems); break;
    case EW::Half:  vmv_v_v<int16_t>(vd, vs1, group, start, elems); break;
    case EW::Word:  vmv_v_v<int32_t>(vd, vs1, group, start, elems); break;
    case EW::Word2: vmv_v_v<int64_t>(vd, vs1, group, start, elems); break;
    default:        postVecFail(di); return;
    }

  vecRegs_.touchReg(vd, group);
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vmv_v_x(unsigned vd, ELEM_TYPE e1, unsigned group,
                   unsigned start, unsigned elems)
{
  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest = 0;
      if (vecRegs_.isDestActive(vd, ix, group, false /*masked*/, dest))
	dest = e1;
      vecRegs_.write(vd, ix, group, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVmv_v_x(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0();
  unsigned rs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd}))
    return;

  SRV e1 = intRegs_.read(rs1);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmv_v_x<int8_t>(vd, e1, group, start, elems); break;
    case EW::Half:  vmv_v_x<int16_t>(vd, e1, group, start, elems); break;
    case EW::Word:  vmv_v_x<int32_t>(vd, e1, group, start, elems); break;
    case EW::Word2: vmv_v_x<int64_t>(vd, e1, group, start, elems); break;
    default:        postVecFail(di); return;
    }

  vecRegs_.touchReg(vd, group);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmv_v_i(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0();
  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd}))
    return;

  auto e1 = di->op1As<int32_t>();

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vmv_v_x<int8_t>(vd, e1, group, start, elems); break;
    case EW::Half:  vmv_v_x<int16_t>(vd, e1, group, start, elems); break;
    case EW::Word:  vmv_v_x<int32_t>(vd, e1, group, start, elems); break;
    case EW::Word2: vmv_v_x<int64_t>(vd, e1, group, start, elems); break;
    default:        postVecFail(di); return;
    }

  vecRegs_.touchReg(vd, group);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::vmvr_v(const DecodedInst* di, unsigned nr)
{
  assert(nr == 1 or nr == 2 or nr == 4 or nr == 8);

  if (not checkSewLmulVstart(di))
    return;

  // Instructions vmv1r.v, vmv2r.v, ... cannot be masked.
  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), vs1 = di->op1();
  if ((vd & (nr-1)) != 0  or (vs1 & (nr-1)) != 0)
    {
      postVecFail(di);   // Vec indices must be multiples of number of register.
      return;
    }

  unsigned bytes = vecRegs_.bytesPerRegister() * nr;

  unsigned start = csRegs_.peekVstart();
  unsigned bytesPerElem = VecRegs::elemWidthInBytes(vecRegs_.elemWidth());
  unsigned elems = bytes / bytesPerElem;

  if (vd != vs1 and start < elems)
    {
      auto dest = vecRegs_.getVecData(vd);
      auto source = vecRegs_.getVecData(vs1);
      bytes -= start*bytesPerElem;

      memcpy(&dest[size_t(start)*bytesPerElem], &source[size_t(start)*bytesPerElem], bytes);
      vecRegs_.setOpEmul(nr, nr);  // Track operand group for logging
    }

  vecRegs_.touchReg(vd, nr*8);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmv1r_v(const DecodedInst* di)
{
  vmvr_v(di, 1);
}


template <typename URV>
void
Hart<URV>::execVmv2r_v(const DecodedInst* di)
{
  vmvr_v(di, 2);
}


template <typename URV>
void
Hart<URV>::execVmv4r_v(const DecodedInst* di)
{
  vmvr_v(di, 4);
}


template <typename URV>
void
Hart<URV>::execVmv8r_v(const DecodedInst* di)
{
  vmvr_v(di, 8);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsaddu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  ELEM_TYPE maxVal = ~ ELEM_TYPE(0);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = e1 + e2;
	  if (dest < e1)
	    {
	      dest = maxVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVsaddu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsaddu_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vsaddu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vsaddu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vsaddu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsaddu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;

  ELEM_TYPE maxVal = ~ ELEM_TYPE(0);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = e1 + e2;
	  if (dest < e1)
	    {
	      dest = maxVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVsaddu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsaddu_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vsaddu_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vsaddu_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vsaddu_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsaddu_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  auto imm = uint64_t(int64_t(di->op2As<int32_t>()));
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsaddu_vx<uint8_t> (vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Half:   vsaddu_vx<uint16_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word:   vsaddu_vx<uint32_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word2:  vsaddu_vx<uint64_t>(vd, vs1, imm,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  static constexpr ELEM_TYPE minVal = std::numeric_limits<ELEM_TYPE>::min();
  static constexpr ELEM_TYPE maxVal = std::numeric_limits<ELEM_TYPE>::max();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1 + e2;
	  bool sameSign = (e1 < 0) == (e2 < 0);
	  if (sameSign and ((e1 < 0) != (dest < 0)))
	    {
	      if (e1 < 0)
		dest = minVal;
	      else
		dest = maxVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVsadd_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsadd_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vsadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vsadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vsadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;

  static constexpr ELEM_TYPE minVal = std::numeric_limits<ELEM_TYPE>::min();
  static constexpr ELEM_TYPE maxVal = std::numeric_limits<ELEM_TYPE>::max();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = e1 + e2;
	  bool sameSign = (e1 < 0) == (e2 < 0);
	  if (sameSign and ((e1 < 0) != (dest < 0)))
	    {
	      if (e1 < 0)
		dest = minVal;
	      else
		dest = maxVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVsadd_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsadd_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vsadd_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vsadd_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vsadd_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVsadd_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(), vs1 = di->op1();
  int64_t imm = di->op2As<int32_t>();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsadd_vx<int8_t> (vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Half:   vsadd_vx<int16_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word:   vsadd_vx<int32_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word2:  vsadd_vx<int64_t>(vd, vs1, imm,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssubu_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  ELEM_TYPE minVal = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1 - e2;
	  if (dest > e1)
	    {
	      dest = minVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
            }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVssubu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssubu_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssubu_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssubu_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssubu_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssubu_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;
  ELEM_TYPE minVal = 0;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = e1 - e2;
	  if (dest > e1)
	    {
	      dest = minVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVssubu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssubu_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vssubu_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vssubu_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vssubu_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0, dest = 0;

  static constexpr ELEM_TYPE minVal = std::numeric_limits<ELEM_TYPE>::min();
  static constexpr ELEM_TYPE maxVal = std::numeric_limits<ELEM_TYPE>::max();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1 - e2;
	  bool sameSign = (e1 < 0) == (e2 >= 0);
	  if (sameSign and ((e1 < 0) != (dest < 0)))
	    {
	      if (e1 < 0)
		dest = minVal;
	      else
		dest = maxVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVssub_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssub_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, dest = 0;

  static constexpr ELEM_TYPE minVal = std::numeric_limits<ELEM_TYPE>::min();
  static constexpr ELEM_TYPE maxVal = std::numeric_limits<ELEM_TYPE>::max();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  dest = e1 - e2;
	  bool sameSign = (e1 < 0) == (e2 >= 0);
	  if (sameSign and ((e1 < 0) != (dest < 0)))
	    {
	      if (e1 < 0)
		dest = minVal;
	      else
		dest = maxVal;
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVssub_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssub_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vssub_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vssub_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vssub_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename T>
static
void
roundoff(VecRoundingMode mode, T& value, unsigned d)
{
  if (d == 0)
    return;

  unsigned bit = 0;

  auto vd = unsigned((value >> d) & 1);
  auto vd_1 = unsigned((value >> (d-1)) & 1);

  switch (mode)
    {
    case VecRoundingMode::NearestUp:
      bit = vd_1;
      break;

    case VecRoundingMode::NearestEven:
      bit = vd_1 & ( ((((T(1) << (d-1)) - 1) & value) != 0)  |  vd );
      break;

    case VecRoundingMode::Down:
      break;

    case VecRoundingMode::Odd:
      bit = (~vd & 1)  & ( (((T(1) << d) - 1) & value) != 0 );
      break;

    default:
      break;
    }


  T extra = bit;
  value = (value >> d) + extra;
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vaadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0;

  using ELEM_TYPE2 = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  ELEM_TYPE2 temp = e1;
	  temp += e2;
	  roundoff(rm, temp, 1);
	  dest = ELEM_TYPE(temp);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVaadd_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vaadd_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vaadd_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vaadd_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vaadd_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVaaddu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vaadd_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vaadd_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vaadd_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vaadd_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vaadd_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0;

  using ELEM_TYPE2 = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  ELEM_TYPE2 temp = e1;
	  temp += e2;
	  roundoff(rm, temp, 1);
	  dest = ELEM_TYPE(temp);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVaadd_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vaadd_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vaadd_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vaadd_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vaadd_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVaaddu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vaadd_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vaadd_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vaadd_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vaadd_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vasub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0;

  using ELEM_TYPE2 = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  ELEM_TYPE2 temp = e1;
	  temp -= e2;
	  roundoff(rm, temp, 1);
	  dest = ELEM_TYPE(temp);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVasub_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vasub_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vasub_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vasub_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vasub_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVasubu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vasub_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vasub_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vasub_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vasub_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vasub_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0;

  using ELEM_TYPE2 = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  ELEM_TYPE2 temp = e1;
	  temp -= e2;
	  roundoff(rm, temp, 1);
	  dest = ELEM_TYPE(temp);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVasub_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vasub_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vasub_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vasub_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vasub_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVasubu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vasub_vx<uint8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vasub_vx<uint16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vasub_vx<uint32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vasub_vx<uint64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0;

  using ELEM_TYPE2 = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  static constexpr ELEM_TYPE minVal = std::numeric_limits<ELEM_TYPE>::min();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  if (e1 == minVal and e2 == minVal)
	    {
	      // Result saturates at max positive value.
	      dest = std::numeric_limits<ELEM_TYPE>::max();
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	  else
	    {
	      ELEM_TYPE2 temp = e1;
	      temp *= e2;
	      roundoff(rm, temp, sizeof(ELEM_TYPE)*8 - 1);
	      dest = ELEM_TYPE(temp);
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVsmul_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsmul_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vsmul_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vsmul_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vsmul_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vsmul_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0;

  using ELEM_TYPE2 = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  static constexpr ELEM_TYPE minVal = std::numeric_limits<ELEM_TYPE>::min();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      bool saturated = false;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  if (e1 == minVal and e2 == minVal)
	    {
	      // Result saturates at max positive value.
	      dest = std::numeric_limits<ELEM_TYPE>::max();
	      csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
              saturated = true;
	    }
	  else
	    {
	      ELEM_TYPE2 temp = e1;
	      temp *= e2;
	      roundoff(rm, temp, sizeof(ELEM_TYPE)*8 - 1);
	      dest = ELEM_TYPE(temp);
	    }
	}
      vecRegs_.vxsat_.push_back(saturated);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVsmul_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vsmul_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vsmul_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vsmul_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vsmul_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssr_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0, e2 = 0;

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE> ();
  unsigned mask = elemBits - 1;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};

      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = e1;
	  unsigned amount = unsigned(e2) & mask;
	  roundoff(rm, dest, amount);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVssrl_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssr_vv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssr_vv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssr_vv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssr_vv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vssr_vx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                   unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = 0;

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE> ();
  unsigned mask = elemBits - 1;
  unsigned amount = unsigned(e2) & mask;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = e1;
	  roundoff(rm, dest, amount);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVssrl_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = uint64_t(int64_t(SRV(intRegs_.read(rs2))));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssr_vx<uint8_t> (vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Half:   vssr_vx<uint16_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word:   vssr_vx<uint32_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<uint64_t>(vd, vs1, e2,           group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssrl_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssr_vx<uint8_t> (vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Half:   vssr_vx<uint16_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word:   vssr_vx<uint32_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<uint64_t>(vd, vs1, imm,           group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssra_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssr_vv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vssr_vv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vssr_vv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vssr_vv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssra_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  // Scalar value is sign extended for XLEN < SEW. Per spec.
  auto e2 = int64_t(SRV(intRegs_.read(rs2)));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssr_vx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vssr_vx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vssr_vx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVssra_vi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vssr_vx<int8_t> (vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Half:   vssr_vx<int16_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word:   vssr_vx<int32_t>(vd, vs1, imm,          group, start, elems, masked); break;
    case EW::Word2:  vssr_vx<int64_t>(vd, vs1, imm,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vnclip_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using U_ELEM_TYPE = std::make_unsigned_t<ELEM_TYPE>;
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE2X e1 = 0;
  ELEM_TYPE e2 = 0;

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned group2x = group*2;
  bool saturated = false; // True if any of the elements saturate.

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};

      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  unsigned amount = unsigned(e2) & mask;
	  roundoff(rm, e1, amount);

	  dest = ELEM_TYPE(e1);
	  if (e1 != ELEM_TYPE2X(dest))
	    {
	      if constexpr (std::is_same<ELEM_TYPE, U_ELEM_TYPE>::value)
		dest = std::numeric_limits<ELEM_TYPE>::max();
	      else
		dest = (e1 < 0) ? std::numeric_limits<ELEM_TYPE>::min() : std::numeric_limits<ELEM_TYPE>::max();
	      saturated = true;
	    }

	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  if (saturated)
    csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
  vecRegs_.vxsat_.push_back(saturated);
}


template <typename URV>
void
Hart<URV>::execVnclipu_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vnclip_wv<uint8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vnclip_wv<uint16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vnclip_wv<uint32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vnclip_wv<uint64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vnclip_wx(unsigned vd, unsigned vs1, ELEM_TYPE e2, unsigned group,
                     unsigned start, unsigned elems, bool masked)
{
  using U_ELEM_TYPE = std::make_unsigned_t<ELEM_TYPE>;
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE2X e1 = 0;

  URV rmVal = peekCsr(CsrNumber::VXRM);
  auto rm = VecRoundingMode(rmVal);

  unsigned elemBits = integerWidth<ELEM_TYPE2X> ();
  unsigned mask = elemBits - 1;
  unsigned amount = unsigned(e2) & mask;
  unsigned group2x = group*2;
  bool saturated = false; // True if any of the elements saturate.

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};

      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);

	  roundoff(rm, e1, amount);

	  dest = ELEM_TYPE(e1);
	  if (e1 != ELEM_TYPE2X(dest))
	    {
	      if constexpr (std::is_same<ELEM_TYPE, U_ELEM_TYPE>::value)
		dest = std::numeric_limits<ELEM_TYPE>::max();
	      else
		dest = (e1 < 0) ? std::numeric_limits<ELEM_TYPE>::min() : std::numeric_limits<ELEM_TYPE>::max();
	      saturated = true;
	    }

	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  if (saturated)
    csRegs_.write(CsrNumber::VXSAT, PrivilegeMode::Machine, 1);
  vecRegs_.vxsat_.push_back(saturated);
}


template <typename URV>
void
Hart<URV>::execVnclipu_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<uint8_t> (vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<uint16_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<uint32_t>(vd, vs1, e2,           group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<uint64_t>(vd, vs1, e2,           group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnclipu_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<uint8_t> (vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<uint16_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<uint32_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<uint64_t>(vd, vs1, imm,           group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnclip_wv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vnclip_wv<int8_t> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Half:   vnclip_wv<int16_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:   vnclip_wv<int32_t>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2:  vnclip_wv<int64_t>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnclip_wx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  SRV e2 = SRV(intRegs_.read(rs2));

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<int8_t> (vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<int16_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<int32_t>(vd, vs1, e2,          group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<int64_t>(vd, vs1, e2,          group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVnclip_wi(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  imm = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:   vnclip_wx<int8_t> (vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Half:   vnclip_wx<int16_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word:   vnclip_wx<int32_t>(vd, vs1, imm,           group, start, elems, masked); break;
    case EW::Word2:  vnclip_wx<int64_t>(vd, vs1, imm,           group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
