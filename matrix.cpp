// Copyright 2026 Tenstorrent Corporation.
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
#include <cassert>
#include <algorithm>
#include <bit>
#include <cfenv>
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "bulk-norm.hpp"


using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execMsettn(const DecodedInst* di)
{
  // Zvt configuration instructions are part of the vector context: Illegal if
  // V is not enabled or if MSTATUS.VS (or VSSTATUS.VS when V=1) is Off.
  if (not isRvzvtbase() or not preVecExec())
    {
      postVecFail(di);
      return;
    }

  if (zvtState_.mtwiden() == 0)
    {
      unsigned rd = di->op0();
      unsigned rs1 = di->op1();
      URV vtypeVal = peekCsr(CsrNumber::VTYPE);
      if (not vsetvl(rd, rs1, vtypeVal))   // Marks VS dirty on success.
        postVecFail(di);
    }
  else
    {
      // Effective number of elements in a vector register
      auto eve = vecRegs_.singleMax(vecRegs_.elemWidth());
      auto ete = zvtState_.effTileDim();
      auto lmulx8 = vecRegs_.groupMultiplierX8();
      auto lmul = lmulx8 < 8 ? 1 : lmulx8 / 8;

      // rd = vl = min(rs1, lmul*eve, ETE)
      auto rd = di->op0(), rs1 = di->op1();
      auto rs1Val = intRegs_.read(rs1);
      auto vl = std::min(rs1Val, std::min(URV(lmul*eve), URV(ete)));

      intRegs_.write(rd, vl);
      pokeCsr(CsrNumber::VL, vl);
      csRegs_.recordWrite (CsrNumber::VL);

      markVsDirty();
    }
}


template <typename URV>
void
Hart<URV>::execMsettm(const DecodedInst* di)
{
  if (not isRvzvtbase()  or  not preVecExec()  or  zvtState_.mtwiden() == 0)
    {
      postVecFail(di);
      return;
    }

  // rd = tm = min(rs1, LMUL*EVE, ETE)

  // Effective number of elements in a vector register
  auto eve = vecRegs_.singleMax(vecRegs_.elemWidth());
  auto ete = zvtState_.effTileDim();
  auto lmulx8 = vecRegs_.groupMultiplierX8();
  auto lmul = lmulx8 < 8 ? 1 : lmulx8 / 8;

  // rd = tm = min(rs1, LMUL*EVE, ETE)
  auto rd = di->op0(), rs1 = di->op1();
  auto rs1Val = intRegs_.read(rs1);
  auto tm = std::min({rs1Val, URV(lmul*eve), URV(ete)});

  intRegs_.write(rd, tm);
  zvtState_.setTm(tm);

  MtypeFields<URV> mfields(peekCsr(CsrNumber::MTYPE));
  mfields.bits_.TM = tm;
  pokeCsr(CsrNumber::MTYPE, mfields.value_);

  csRegs_.recordWrite (CsrNumber::MTYPE);

  // All Zvt instructions (including configuration instructions) are considered to
  // modify vector state. Configuration instructions do not access the tile state, so
  // MSTATUS.MS is not affected.
  markVsDirty();
}


template <typename URV>
void
Hart<URV>::execMsettk(const DecodedInst* di)
{
  // The msettm and msettk instructions raise an illegal-instruction exception when
  // mtwiden=0.
  if (not isRvzvtbase()  or  not preVecExec()  or  zvtState_.mtwiden() == 0)
    {
      postVecFail(di);
      return;
    }

  unsigned kmax = 0;
  if (not zvtState_.kmax(vecRegs_.elemWidthInBits(), kmax))
    {
      illegalInst(di);    // FIX : Is this right? Spec does not say.
      return;
    }

  // rd = tk = min(rs1, KMAX)
  auto rd = di->op0(), rs1 = di->op1();
  auto rs1Val = intRegs_.read(rs1);
  auto tk = std::min(rs1Val, URV(kmax));

  intRegs_.write(rd, tk);

  MtypeFields<URV> mfields(peekCsr(CsrNumber::MTYPE));
  mfields.bits_.TK = tk;
  pokeCsr(CsrNumber::MTYPE, mfields.value_);

  csRegs_.recordWrite (CsrNumber::MTYPE);

  // All Zvt instructions (including configuration instructions) are considered to
  // modify vector state. Configuration instructions do not access the tile state, so
  // MSTATUS.MS is not affected.
  markVsDirty();
}


template <typename URV>
bool
Hart<URV>::msetmtype(URV mtypeVal, URV vtypeVal)
{
  VtypeFields<URV> vtype(vtypeVal);
  MtypeFields<URV> mtype(mtypeVal);

  // if vtype is invalid per vsetvl semantics: vtype.vill = 1. Check the raw value: the
  // VTYPE poke mask would silently drop reserved bits.
  bool vill = not isLegalVtype(vtypeVal);

  // If vtype is already illegal, the matrix fields are irrelevant: everything is zeroed
  // below. Skipping avoids computing with a possibly reserved SEW encoding.
  if (mtype.bits_.MTWIDEN != 0 and not vill)
    {
      unsigned twiden = ZvtState::twidenFromMtwiden(mtype.bits_.MTWIDEN);
      auto sew = ElementWidth(vtype.bits_.SEW);
      unsigned sewBits = VecRegs::elemWidthInBits(sew);
      unsigned tewBits = sewBits * twiden;

      // vtype.vill |= (SEW * TWIDEN > ELEN). Also illegal if SEW/TWIDEN has no KMAX.
      unsigned kmax = 0;
      if (tewBits > vecRegs_.elen() or not ZvtState::kmax(sewBits, twiden, kmax))
        vill = true;
      else
        {
          unsigned ete = zvtState_.ete(tewBits);   // TEW < 64 ? TE : TE/2
          unsigned eve = vecRegs_.singleMax(sew);  // VLEN/SEW
          unsigned ceilEteEve = (ete + eve - 1) / eve;

          // All three terms are powers of 2, so lmul is a power of 2 in [1, 8].
          unsigned lmul = std::min({8/kmax, 8/twiden, ceilEteEve});
          vtype.bits_.LMUL = std::bit_width(lmul) - 1;   // log2(lmul)
          vtype.bits_.VMA = 1;
          vtype.bits_.VTA = 1;

          mtype.bits_.TK = std::min(unsigned(mtype.bits_.TK), kmax);
          mtype.bits_.TM = std::min({unsigned(mtype.bits_.TM), lmul*eve, ete});
        }
    }

  // Like vsetvl: if so configured, trap on an illegal configuration instead of setting
  // vtype.vill. Nothing has been written yet.
  if (vill and vecRegs_.trapVtype_)
    return false;

  if (vill or mtype.bits_.MTWIDEN == 0)
    mtype.value_ = 0;

  if (vill)
    {
      vtype.value_ = 0;
      vtype.bits_.VILL = 1;
    }

  // vl = 0
  pokeCsr(CsrNumber::VL, 0);   // Also updates cached VL in vecRegs_.
  recordCsrWrite(CsrNumber::VL);

  // These pokes also update the cached VTYPE/MTYPE values.
  pokeCsr(CsrNumber::VTYPE, vtype.value_);
  recordCsrWrite(CsrNumber::VTYPE);
  pokeCsr(CsrNumber::MTYPE, mtype.value_);
  recordCsrWrite(CsrNumber::MTYPE);

  markVsDirty();
  // Configuration instructions do not access the tile state: MSTATUS.MS not affected.
  return true;
}


template <typename URV>
void
Hart<URV>::clearMtype()
{
  if (not isRvzvtbase())
    return;

  pokeCsr(CsrNumber::MTYPE, 0);   // Also updates cached MTYPE fields in zvtState_.
  recordCsrWrite(CsrNumber::MTYPE);
}


template <typename URV>
void
Hart<URV>::execMsetmtypei(const DecodedInst* di)
{
  // Zvt configuration instructions are part of the vector context: Illegal if
  // V is not enabled or if MSTATUS.VS (or VSSTATUS.VS when V=1) is Off.
  if (not isRvzvtbase() or not preVecExec())
    {
      postVecFail(di);
      return;
    }

  uint32_t opcode = di->inst();

  MtypeFields<URV> mtype(0);                 // mtype = 0
  mtype.bits_.MTWIDEN = (opcode >> 15) & 3;  // mtype.mtwiden = insn.mtwiden (bits 16:15)

  VtypeFields<URV> vtype(0);                 // vtype = 0
  vtype.bits_.SEW = (opcode >> 23) & 3;      // vtype.vsew = insn.vsew (bits 24:23)
  vtype.bits_.ALTFMT = (opcode >> 28) & 1;   // vtype.altfmt = insn.altfmt (bit 28)

  if (not msetmtype(mtype.value_, vtype.value_))
    postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execMsetmtype(const DecodedInst* di)
{
  // Zvt configuration instructions are part of the vector context: Illegal if
  // V is not enabled or if MSTATUS.VS (or VSSTATUS.VS when V=1) is Off.
  if (not isRvzvtbase() or not preVecExec())
    {
      postVecFail(di);
      return;
    }

  URV rs1Val = intRegs_.read(di->op0());   // mtype = rs1
  URV rs2Val = intRegs_.read(di->op1());   // vtype = rs2

  if (not msetmtype(rs1Val, rs2Val))
    postVecFail(di);
}


template <typename URV>
void
Hart<URV>::setMatStatus(MatStatus value)
{
  if (mstatus_.bits_.MS != unsigned(value))
    {
      mstatus_.bits_.MS = unsigned(value);
      writeMstatus();
    }

  if (virtMode_ and vsstatus_.bits_.MS != unsigned(value))
    {
      vsstatus_.bits_.MS = unsigned(value);
      pokeCsr(CsrNumber::VSSTATUS, vsstatus_.value_);
      recordCsrWrite(CsrNumber::VSSTATUS);
      updateCachedVsstatus();
    }
}


template <typename URV>
bool
Hart<URV>::isLegalTileLdSt(ElementWidth eew)
{
  // Saves vstart for tracing and checks that V is enabled and MSTATUS.VS is not off.
  if (not preVecExec())
    return false;

  if (not isRvzvtbase() or not isMatEnabled())
    return false;

  // If mtwiden=0, these instructions are reserved.
  if (zvtState_.mtwiden() == 0 or zvtState_.te() == 0)
    return false;

  // Supported TEW values are those corresponding to supported SEW settings (8..ELEN).
  if (not vecRegs_.legalConfig(eew, GroupMultiplier::One))
    return false;

  return true;
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::tileLoad(const DecodedInst* di)
{
  constexpr unsigned elemSize = sizeof(ELEM_TYPE);
  constexpr unsigned eewBits = elemSize * 8;

  URV tss = intRegs_.read(di->op0());
  URV base = intRegs_.read(di->op1());

  unsigned tile = 0, pattern = 0, index = 0;
  zvtState_.decodeTss(tss, eewBits, tile, pattern, index);

  // Body elements are those with indices in [vstart, min(vl, ETE) - 1].
  unsigned count = std::min(unsigned(vecRegs_.elemCount()), zvtState_.ete(eewBits));
  unsigned start = csRegs_.peekVstart();

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = true;
  bool tileWritten = false;

  for (unsigned ix = start; ix < count; ++ix)
    {
      uint64_t addr = base + ix * elemSize;

      uint64_t pa1 = addr, pa2 = addr;   // Phys addrs or faulting virtual address.
      uint64_t gpa1 = addr, gpa2 = addr;
      uint64_t faultAddr = addr;

      // Misaligned element addresses are not supported.
      auto cause = ExceptionCause::NONE;
      if ((addr & (elemSize - 1)) != 0)
        {
          cause = ExceptionCause::LOAD_ADDR_MISAL;
          gpa1 = 0;
        }
      else
        {
          cause = determineLoadException(pa1, pa2, gpa1, gpa2, elemSize, false /*hyper*/,
                                         false /*amo*/, ix);
          faultAddr = ldStFaultAddr_;
        }

      if (hasTrig)
        {
          uint64_t pmva = applyPointerMask(addr, isLd);
          if (ldStAddrTriggerHit(pmva, elemSize, timing, isLd))
            {
              if (tileWritten)
                markMsDirty();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              return false;
            }
        }

      if (cause != ExceptionCause::NONE)
        {
          if (tileWritten)
            markMsDirty();
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          initiateLoadException(di, cause, faultAddr, gpa1);
          return false;
        }

      uint64_t data = 0;
      if (not readForLoad<ELEM_TYPE>(di, addr, pa1, pa2, data, ix))
        {
          // readForLoad returns false when a load-data trigger trips on this element.
          assert(breakpOrEnterDebugTripped());
          if (tileWritten)
            markMsDirty();
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          return false;
        }

      ELEM_TYPE elem = data;

      if (hasTrig)
        {
          ldStDataTriggerHit(elem, sizeof(elem), timing, isLd);
          if (breakpOrEnterDebugTripped())
            {
              if (tileWritten)
                markMsDirty();
              markVsDirty();
              csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
              return false;
            }
        }

      unsigned row = 0, col = 0;
      ZvtState::subsetElemRowCol(pattern, index, ix, row, col);
      if (not zvtState_.writeElem<ELEM_TYPE>(tile, row, col, elem))
        assert(0 && "Error: Assertion failed: Tile element out of bounds");
      tileWritten = true;
    }

  if (tileWritten)
    markMsDirty();

  return true;
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::tileStore(const DecodedInst* di)
{
  constexpr unsigned elemSize = sizeof(ELEM_TYPE);
  constexpr unsigned eewBits = elemSize * 8;

  URV tss = intRegs_.read(di->op0());
  URV base = intRegs_.read(di->op1());

  unsigned tile = 0, pattern = 0, index = 0;
  zvtState_.decodeTss(tss, eewBits, tile, pattern, index);

  // Body elements are those with indices in [vstart, min(vl, ETE) - 1].
  unsigned count = std::min(unsigned(vecRegs_.elemCount()), zvtState_.ete(eewBits));
  unsigned start = csRegs_.peekVstart();

  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;

  for (unsigned ix = start; ix < count; ++ix)
    {
      uint64_t addr = base + ix * elemSize;

      unsigned row = 0, col = 0;
      ZvtState::subsetElemRowCol(pattern, index, ix, row, col);
      ELEM_TYPE elem = 0;
      if (not zvtState_.readElem<ELEM_TYPE>(tile, row, col, elem))
        assert(0 && "Error: Assertion failed: Tile element out of bounds");

      uint64_t pa1 = addr, pa2 = addr;   // Phys addrs or faulting virtual address.
      uint64_t gpa1 = addr, gpa2 = addr;
      uint64_t faultAddr = addr;

      // Misaligned element addresses are not supported.
      auto cause = ExceptionCause::NONE;
      if ((addr & (elemSize - 1)) != 0)
        {
          cause = ExceptionCause::STORE_ADDR_MISAL;
          gpa1 = 0;
        }
      else
        {
          cause = determineStoreException(pa1, pa2, gpa1, gpa2, elemSize, false /*hyper*/);
          faultAddr = ldStFaultAddr_;
        }

      if (hasTrig)
        {
          uint64_t pmva = applyPointerMask(addr, isLd);
          ldStAddrTriggerHit(pmva, elemSize, timing, isLd);
          ldStDataTriggerHit(elem, sizeof(elem), timing, isLd);
        }

      if (cause != ExceptionCause::NONE or breakpOrEnterDebugTripped())
        {
          markVsDirty();
          csRegs_.write(CsrNumber::VSTART, PrivilegeMode::Machine, ix);
          if (not breakpOrEnterDebugTripped())
            initiateStoreException(di, cause, faultAddr, gpa1);
          return false;
        }

      if (not writeForStore(addr, pa1, pa2, elem))
        assert(0 && "Error: Assertion failed");
    }

  return true;
}


template <typename URV>
void
Hart<URV>::execVtle8(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Byte))
    {
      postVecFail(di);
      return;
    }

  if (tileLoad<uint8_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
void
Hart<URV>::execVtle16(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Half))
    {
      postVecFail(di);
      return;
    }

  if (tileLoad<uint16_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
void
Hart<URV>::execVtle32(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Word))
    {
      postVecFail(di);
      return;
    }

  if (tileLoad<uint32_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
void
Hart<URV>::execVtle64(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Word2))
    {
      postVecFail(di);
      return;
    }

  if (tileLoad<uint64_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
void
Hart<URV>::execVtse8(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Byte))
    {
      postVecFail(di);
      return;
    }

  if (tileStore<uint8_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
void
Hart<URV>::execVtse16(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Half))
    {
      postVecFail(di);
      return;
    }

  if (tileStore<uint16_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
void
Hart<URV>::execVtse32(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Word))
    {
      postVecFail(di);
      return;
    }

  if (tileStore<uint32_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
void
Hart<URV>::execVtse64(const DecodedInst* di)
{
  if (not isLegalTileLdSt(ElementWidth::Word2))
    {
      postVecFail(di);
      return;
    }

  if (tileStore<uint64_t>(di))
    {
      postVecSuccess(di);
      markVsDirty();  // All Zvt instructions are considered to modify vector state.
    }
}


template <typename URV>
bool
Hart<URV>::checkTileMove(const DecodedInst* di, unsigned vecReg)
{
  // Checks that V is enabled, MSTATUS.VS is not off, vtype.vill is not set, and that
  // vstart is zero if so configured. Takes an illegal instruction trap if not legal.
  if (not checkSewLmulVstart(di))
    return false;

  // MSTATUS.MS must not be off. If mtwiden=0, these instructions are reserved.
  if (not isRvzvtbase() or not isMatEnabled() or zvtState_.mtwiden() == 0 or
      zvtState_.te() == 0 or di->isMasked())
    {
      postVecFail(di);
      return false;
    }

  // Vector register must be a multiple of the register group size.
  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);
  return checkVecOpsVsEmul(di, group, {vecReg});
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vtmv_v_t(unsigned vd, uint64_t tss, unsigned group)
{
  constexpr unsigned tewBits = sizeof(ELEM_TYPE) * 8;   // TEW = SEW

  unsigned tile = 0, pattern = 0, index = 0;
  zvtState_.decodeTss(tss, tewBits, tile, pattern, index);

  // Body elements are those with indices in [vstart, min(vl, ETE) - 1]. Tail elements
  // are those with indices >= min(vl, ETE).
  unsigned count = std::min(unsigned(vecRegs_.elemCount()), zvtState_.ete(tewBits));
  unsigned start = csRegs_.peekVstart();
  if (start >= count)
    return;   // No body elements: Nothing is updated (including tail).

  unsigned vlmax = vecRegs_.elemMax();
  bool tailOnes = vecRegs_.isTailAgnostic() and vecRegs_.isTailAgnosticOnes();

  for (unsigned ix = start; ix < count; ++ix)
    {
      unsigned row = 0, col = 0;
      ZvtState::subsetElemRowCol(pattern, index, ix, row, col);
      ELEM_TYPE elem = 0;
      if (not zvtState_.readElem<ELEM_TYPE>(tile, row, col, elem))
        assert(0 && "Error: Assertion failed: Tile element out of bounds");
      vecRegs_.write(vd, ix, group, elem);
    }

  if (tailOnes)
    for (unsigned ix = count; ix < vlmax; ++ix)
      vecRegs_.write(vd, ix, group, ELEM_TYPE(~ELEM_TYPE(0)));
}


template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vtmv_t_v(unsigned vs2, uint64_t tss, unsigned group)
{
  constexpr unsigned tewBits = sizeof(ELEM_TYPE) * 8;   // TEW = SEW

  unsigned tile = 0, pattern = 0, index = 0;
  zvtState_.decodeTss(tss, tewBits, tile, pattern, index);

  // Body elements are those with indices in [vstart, min(vl, ETE) - 1]. Tail elements
  // of the tile subset are left undisturbed.
  unsigned count = std::min(unsigned(vecRegs_.elemCount()), zvtState_.ete(tewBits));
  unsigned start = csRegs_.peekVstart();

  bool written = false;
  for (unsigned ix = start; ix < count; ++ix)
    {
      ELEM_TYPE elem = 0;
      vecRegs_.read(vs2, ix, group, elem);
      unsigned row = 0, col = 0;
      ZvtState::subsetElemRowCol(pattern, index, ix, row, col);
      if (not zvtState_.writeElem<ELEM_TYPE>(tile, row, col, elem))
        assert(0 && "Error: Assertion failed: Tile element out of bounds");
      written = true;
    }

  return written;
}


template <typename URV>
void
Hart<URV>::execVtmv_v_t(const DecodedInst* di)
{
  unsigned vd = di->op0();
  if (not checkTileMove(di, vd))
    return;

  uint64_t tss = intRegs_.read(di->op1());
  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  using EW = ElementWidth;
  switch (vecRegs_.elemWidth())   // TEW = SEW
    {
    case EW::Byte:  vtmv_v_t<uint8_t> (vd, tss, group); break;
    case EW::Half:  vtmv_v_t<uint16_t>(vd, tss, group); break;
    case EW::Word:  vtmv_v_t<uint32_t>(vd, tss, group); break;
    case EW::Word2: vtmv_v_t<uint64_t>(vd, tss, group); break;
    default:        postVecFail(di); return;
    }

  vecRegs_.touchReg(vd, group);
  postVecSuccess(di);
  markVsDirty();  // All Zvt instructions are considered to modify vector state.
}


template <typename URV>
void
Hart<URV>::execVtmv_t_v(const DecodedInst* di)
{
  unsigned vs2 = di->op1();
  if (not checkTileMove(di, vs2))
    return;

  uint64_t tss = intRegs_.read(di->op0());
  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  bool written = false;
  using EW = ElementWidth;
  switch (vecRegs_.elemWidth())   // TEW = SEW
    {
    case EW::Byte:  written = vtmv_t_v<uint8_t> (vs2, tss, group); break;
    case EW::Half:  written = vtmv_t_v<uint16_t>(vs2, tss, group); break;
    case EW::Word:  written = vtmv_t_v<uint32_t>(vs2, tss, group); break;
    case EW::Word2: written = vtmv_t_v<uint64_t>(vs2, tss, group); break;
    default:        postVecFail(di); return;
    }

  if (written)
    markMsDirty();
  postVecSuccess(di);
  markVsDirty();  // All Zvt instructions are considered to modify vector state.
}

template <typename URV>
template <typename ELEM_TYPE>
bool
Hart<URV>::vtzero(unsigned tile)
{
  constexpr unsigned tewBits = sizeof(ELEM_TYPE) * 8;

  unsigned ete = zvtState_.ete(tewBits);
  unsigned tm = std::min(zvtState_.tm(), ete);
  unsigned tn = std::min(unsigned(vecRegs_.elemCount()), ete);

  // If tm or tn is zero, no state is updated (same as the matrix arithmetic
  // instructions).
  if (tm == 0 or tn == 0)
    return false;

  // Elements outside [0, tm-1] x [0, tn-1] are tail and are handled using a
  // tail-agnostic policy.
  bool tailOnes = vecRegs_.isTailAgnosticOnes();
  unsigned rows = tailOnes ? ete : tm;
  unsigned cols = tailOnes ? ete : tn;

  for (unsigned row = 0; row < rows; ++row)
    for (unsigned col = 0; col < cols; ++col)
      {
        ELEM_TYPE val = (row < tm and col < tn) ? ELEM_TYPE(0) : ELEM_TYPE(~ELEM_TYPE(0));
        if (not zvtState_.writeElem<ELEM_TYPE>(tile, row, col, val))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");
      }

  return true;
}


template <typename URV>
void
Hart<URV>::execVtzero(const DecodedInst* di)
{
  // Checks that V is enabled, MSTATUS.VS is not off, and that vtype.vill is not set.
  if (not preVecExec() or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return;
    }

  // MSTATUS.MS must not be off. Unsupported mtype settings (including mtwiden=0) are
  // reserved. Like the matrix arithmetic instructions, a non-zero vstart is illegal.
  unsigned tewBits = zvtState_.tew();
  if (not isRvzvtbase() or not isMatEnabled() or zvtState_.mtwiden() == 0 or
      zvtState_.te() == 0 or csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  // If the tile number is not valid for the current TEW (tile modulo (16 / NTiles) is
  // non-zero), the instruction is reserved.
  unsigned tile = di->op0();
  unsigned nTiles = ZvtState::tileCount(tewBits);
  if (nTiles == 0 or (tile % (16 / nTiles)) != 0)
    {
      postVecFail(di);
      return;
    }

  bool written = false;
  switch (tewBits)
    {
    case 8:  written = vtzero<uint8_t> (tile); break;
    case 16: written = vtzero<uint16_t>(tile); break;
    case 32: written = vtzero<uint32_t>(tile); break;
    case 64: written = vtzero<uint64_t>(tile); break;
    default: postVecFail(di); return;
    }

  if (written)
    markMsDirty();
  postVecSuccess(di);
  markVsDirty();  // All Zvt instructions are considered to modify vector state.
}

template <typename URV>
void
Hart<URV>::execVtdiscard(const DecodedInst* di)
{
  // Illegal if V is not enabled, if MSTATUS.VS (or VSSTATUS.VS when V=1) is Off, or
  // if vtype.vill is set. Unlike other tile instructions, mtype.mtwiden=0 is legal.
  if (not preVecExec() or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return;
    }

  // Illegal if MSTATUS.MS is Off (or if V=1 and VSSTATUS.MS is Off).
  if (not isRvzvtbase() or not isMatEnabled())
    {
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
  markVsDirty();  // All Zvt instructions are considered to modify vector state.

  // Tile state becomes UNSPECIFIED for ABI purposes but is not actually written.
  setMatStatus(MatStatus::Initial);
}

template <typename URV>
bool
Hart<URV>::checkMatmul(const DecodedInst* di, unsigned sewBits, unsigned twiden)
{
  // V must be enabled, MSTATUS.VS (and VSSTATUS.VS if V=1) must not be Off, and
  // vtype.vill must be clear. An illegal-instruction exception is raised if vstart is
  // non-zero.
  bool ok = preVecExec() and vecRegs_.legalConfig() and csRegs_.peekVstart() == 0;

  // MSTATUS.MS (and VSSTATUS.MS if V=1) must not be Off.
  ok = ok and isRvzvtbase() and isMatEnabled() and zvtState_.te() != 0;

  // Unsupported vtype or mtype settings are reserved.
  ok = ok and vecRegs_.elemWidthInBits() == sewBits;
  ok = ok and ZvtState::twidenFromMtwiden(zvtState_.mtwiden()) == twiden;
  unsigned kmax = 0;
  ok = ok and ZvtState::kmax(sewBits, twiden, kmax);

  // If the tile number is not valid for the TEW (tile modulo (16 / NTiles) is
  // non-zero), the instruction is reserved.
  unsigned tile = di->op0();
  unsigned nTiles = ZvtState::tileCount(sewBits * twiden);
  ok = ok and nTiles != 0 and (tile % (16 / nTiles)) == 0;

  // Vector register specifiers must be divisible by LMUL and, when taken modulo 8,
  // must be less than 8/KMAX.
  if (ok)
    {
      unsigned lmul = std::max(vecRegs_.groupMultiplierX8() / 8, 1u);
      for (unsigned vs : { di->op1(), di->op2() })
        ok = ok and (vs % lmul) == 0 and (vs % 8) < 8 / kmax;
    }

  if (not ok)
    postVecFail(di);
  return ok;
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::tileTail(unsigned tile, unsigned tm, unsigned tn)
{
  // Tile elements outside [0, tm-1] x [0, tn-1] are tail and are handled using a
  // tail-agnostic policy.
  if (not vecRegs_.isTailAgnosticOnes())
    return;

  constexpr unsigned tewBits = sizeof(ELEM_TYPE) * 8;
  unsigned ete = zvtState_.ete(tewBits);
  for (unsigned row = 0; row < ete; ++row)
    for (unsigned col = 0; col < ete; ++col)
      if (row >= tm or col >= tn)
        if (not zvtState_.writeElem<ELEM_TYPE>(tile, row, col, ELEM_TYPE(~ELEM_TYPE(0))))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");
}


template <typename URV>
template <typename A_TYPE, typename B_TYPE>
void
Hart<URV>::vtmmInt(unsigned tile, unsigned vs2, unsigned vs1)
{
  static_assert(sizeof(A_TYPE) == 1 and sizeof(B_TYPE) == 1);
  constexpr unsigned tewBits = 32;
  constexpr unsigned kmax = 4;            // KMAX for SEW=8, TWIDEN=4.
  constexpr unsigned rowStride = 8 / kmax; // Rows are separated by 8/KMAX registers.

  unsigned ete = zvtState_.ete(tewBits);
  unsigned tm = std::min(zvtState_.tm(), ete);
  unsigned tn = std::min(unsigned(vecRegs_.elemCount()), ete);
  unsigned tk = std::min(zvtState_.tk(), kmax);

  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  // C[tm,tn] += A[tk,tm]^T * B[tk,tn]. A row k is in vector register group
  // vs2 + k*rowStride, B row k is in vs1 + k*rowStride. Accumulation is modulo 2^32.
  for (unsigned i = 0; i < tm; ++i)
    for (unsigned j = 0; j < tn; ++j)
      {
        uint32_t acc = 0;
        if (not zvtState_.readElem<uint32_t>(tile, i, j, acc))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");

        for (unsigned k = 0; k < tk; ++k)
          {
            A_TYPE a = 0;
            B_TYPE b = 0;
            vecRegs_.read(vs2 + k*rowStride, i, group, a);
            vecRegs_.read(vs1 + k*rowStride, j, group, b);
            acc += uint32_t(int32_t(a) * int32_t(b));
          }

        if (not zvtState_.writeElem<uint32_t>(tile, i, j, acc))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");
      }

  tileTail<uint32_t>(tile, tm, tn);
}


template <typename URV>
void
Hart<URV>::execVtmmInt(const DecodedInst* di, bool signedA)
{
  // Supported only for SEW=8, TWIDEN=4 (int32 accumulators).
  if (not isRvzvti8i32mm())
    {
      postVecFail(di);
      return;
    }

  if (not checkMatmul(di, 8, 4))
    return;

  unsigned tile = di->op0(), vs2 = di->op1(), vs1 = di->op2();

  // If any of tm, tn, or tk is zero, no computation is performed and no state is
  // updated.
  unsigned tn = std::min(unsigned(vecRegs_.elemCount()), zvtState_.ete(32));
  if (zvtState_.tm() != 0 and zvtState_.tk() != 0 and tn != 0)
    {
      // Type of A (vs2) is given by the opcode, type of B (vs1) by vtype.altfmt.
      bool signedB = vecRegs_.altfmt();
      if (signedA)
        {
          if (signedB) vtmmInt<int8_t, int8_t>(tile, vs2, vs1);
          else         vtmmInt<int8_t, uint8_t>(tile, vs2, vs1);
        }
      else
        {
          if (signedB) vtmmInt<uint8_t, int8_t>(tile, vs2, vs1);
          else         vtmmInt<uint8_t, uint8_t>(tile, vs2, vs1);
        }
      markMsDirty();
    }

  postVecSuccess(di);
  markVsDirty();  // All Zvt instructions are considered to modify vector state.
}


template <typename URV>
void
Hart<URV>::execVtmmu_tvv(const DecodedInst* di)
{
  execVtmmInt(di, false);  // A (vs2) is uint8.
}


template <typename URV>
void
Hart<URV>::execVtmms_tvv(const DecodedInst* di)
{
  execVtmmInt(di, true);   // A (vs2) is int8.
}

namespace WdRiscv
{
  extern
  uint16_t ofp8ToBfloat16(uint8_t x, bool e4m3);
}


/// Clear the inexact and underflow simulator FP flags: Matrix floating point operations
/// only update the invalid and overflow flags.
static void
clearInexactUnderflowFlags()
{
#if SOFT_FLOAT
  softfloat_exceptionFlags &= ~(softfloat_flag_inexact | softfloat_flag_underflow);
#else
  std::feclearexcept(FE_INEXACT | FE_UNDERFLOW);
#endif
}


template <typename URV>
template <typename FT>
void
Hart<URV>::vtfmmBna(unsigned tile, unsigned vs2, unsigned vs1, bool ofp8, bool aE4m3,
                    bool bE4m3)
{
  constexpr unsigned tewBits = 32;               // FP32 accumulators.
  unsigned sewBits = ofp8 ? 8 : 16;
  unsigned kmax = sewBits == 8 ? 4 : 2;
  unsigned rowStride = 8 / kmax;                 // Rows are separated by 8/KMAX registers.

  unsigned ete = zvtState_.ete(tewBits);
  unsigned tm = std::min(zvtState_.tm(), ete);
  unsigned tn = std::min(unsigned(vecRegs_.elemCount()), ete);
  unsigned tk = std::min(zvtState_.tk(), kmax);

  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  // RVBNA uses a static dot product dimension n: we use n = KMAX and substitute zero
  // for the products of rows tk to KMAX-1.
  std::vector<uint16_t> aa(kmax), bb(kmax);

  for (unsigned i = 0; i < tm; ++i)
    for (unsigned j = 0; j < tn; ++j)
      {
        for (unsigned k = 0; k < kmax; ++k)
          {
            uint16_t a = 0, b = 0;
            if (k < tk)
              {
                if (ofp8)
                  {
                    uint8_t a8 = 0, b8 = 0;
                    vecRegs_.read(vs2 + k*rowStride, i, group, a8);
                    vecRegs_.read(vs1 + k*rowStride, j, group, b8);
                    a = ofp8ToBfloat16(a8, aE4m3);
                    b = ofp8ToBfloat16(b8, bE4m3);
                  }
                else
                  {
                    vecRegs_.read(vs2 + k*rowStride, i, group, a);
                    vecRegs_.read(vs1 + k*rowStride, j, group, b);
                  }
              }
            aa.at(k) = a;
            bb.at(k) = b;
          }

        bool inv = false, ovf = false;
        uint32_t udp = bulkNormalizeDotProd<FT, FT, float>(aa, bb, inv, ovf);
        if (inv)
          raiseSimulatorFpFlags(FpFlags::Invalid);
        if (ovf)
          raiseSimulatorFpFlags(FpFlags::Overflow);

        uint32_t acc = 0;
        if (not zvtState_.readElem<uint32_t>(tile, i, j, acc))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");

        // Add to the accumulator in IEEE 754 arithmetic rounding to odd (RVBNA RTO).
        float res = combinePartitionResultsRTO(std::bit_cast<float>(acc),
                                               { std::bit_cast<float>(udp) });

        if (not zvtState_.writeElem<uint32_t>(tile, i, j, std::bit_cast<uint32_t>(res)))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");
      }

  tileTail<uint32_t>(tile, tm, tn);
}


template <typename URV>
template <typename FT>
void
Hart<URV>::vtfmmIeee(unsigned tile, unsigned vs2, unsigned vs1)
{
  using UT = WdRiscv::getSameWidthUintType_t<FT>;
  constexpr unsigned tewBits = sizeof(FT) * 8;   // TWIDEN is 1.
  constexpr unsigned kmax = 1;
  constexpr unsigned rowStride = 8 / kmax;

  unsigned ete = zvtState_.ete(tewBits);
  unsigned tm = std::min(zvtState_.tm(), ete);
  unsigned tn = std::min(unsigned(vecRegs_.elemCount()), ete);
  unsigned tk = std::min(zvtState_.tk(), kmax);

  unsigned group = vecRegs_.groupMultiplierX8();
  group = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  // Products are computed in IEEE 754 arithmetic and rounded to TEW bits using the
  // FRM rounding mode, then added to the accumulator using the same rounding mode.
  for (unsigned i = 0; i < tm; ++i)
    for (unsigned j = 0; j < tn; ++j)
      {
        UT uacc = 0;
        if (not zvtState_.readElem<UT>(tile, i, j, uacc))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");
        FT acc = std::bit_cast<FT>(uacc);

        for (unsigned k = 0; k < tk; ++k)
          {
            FT a = 0, b = 0;
            vecRegs_.read(vs2 + k*rowStride, i, group, a);
            vecRegs_.read(vs1 + k*rowStride, j, group, b);
            acc = doFadd(acc, doFmul(a, b));
          }

        if (not zvtState_.writeElem<UT>(tile, i, j, std::bit_cast<UT>(acc)))
          assert(0 && "Error: Assertion failed: Tile element out of bounds");
      }

  tileTail<UT>(tile, tm, tn);
}


template <typename URV>
void
Hart<URV>::execVtfmm(const DecodedInst* di, bool alt)
{
  unsigned sewBits = vecRegs_.elemWidthInBits();
  unsigned twiden = ZvtState::twidenFromMtwiden(zvtState_.mtwiden());
  bool altfmt = vecRegs_.altfmt();

  // Determine the variant from the opcode (alt), SEW, TWIDEN, and vtype.altfmt.
  // Unsupported settings are reserved.
  enum class Kind { None, Ofp8, Fp16, Bf16, Fp32, Fp64 };
  Kind kind = Kind::None;
  bool extOk = false;

  if (sewBits == 8 and twiden == 4)
    {
      kind = Kind::Ofp8;
      extOk = isRvzvtofp8fmm();
    }
  else if (sewBits == 16 and twiden == 2 and altfmt == alt)
    {
      kind = alt ? Kind::Bf16 : Kind::Fp16;
      extOk = alt ? isRvzvtbf16fmm() : isRvzvtfp16fmm();
    }
  else if (sewBits == 32 and twiden == 1 and not alt and not altfmt)
    {
      kind = Kind::Fp32;
      extOk = isRvzvtfmm();
    }
  else if (sewBits == 64 and twiden == 1 and not alt and not altfmt)
    {
      kind = Kind::Fp64;
      extOk = isRvzvtdmm();
    }

  // F (D for FP64) extension must be enabled and MSTATUS.FS must not be Off.
  bool fpOk = kind == Kind::Fp64 ? isDpLegal() : isFpLegal();

  // For SEW >= 32, the rounding mode in FRM is used and must be valid.
  bool ieee = kind == Kind::Fp32 or kind == Kind::Fp64;
  bool rmOk = not ieee or getFpRoundingMode() < RoundingMode::Invalid1;

  if (kind == Kind::None or not extOk or not fpOk or not rmOk)
    {
      postVecFail(di);
      return;
    }

  if (not checkMatmul(di, sewBits, twiden))
    return;

  unsigned tile = di->op0(), vs2 = di->op1(), vs1 = di->op2();

  // If any of tm, tn, or tk is zero, no computation is performed and no state is
  // updated.
  unsigned tn = std::min(unsigned(vecRegs_.elemCount()), zvtState_.ete(sewBits*twiden));
  if (zvtState_.tm() != 0 and zvtState_.tk() != 0 and tn != 0)
    {
      clearSimulatorFpFlags();
      setSimulatorRoundingMode(getFpRoundingMode());

      // Vector operand formats: vs2 (A) is given by the opcode, vs1 (B) by altfmt.
      switch (kind)
        {
        case Kind::Ofp8: vtfmmBna<BFloat16>(tile, vs2, vs1, true, not alt, not altfmt); break;
        case Kind::Fp16: vtfmmBna<Float16> (tile, vs2, vs1, false, false, false); break;
        case Kind::Bf16: vtfmmBna<BFloat16>(tile, vs2, vs1, false, false, false); break;
        case Kind::Fp32: vtfmmIeee<float>  (tile, vs2, vs1); break;
        case Kind::Fp64: vtfmmIeee<double> (tile, vs2, vs1); break;
        default: assert(0); break;
        }

      // Only the invalid and overflow flags are updated.
      clearInexactUnderflowFlags();
      updateAccruedFpBits();
      markFsDirty();   // Instruction implicitly accesses FCSR.
      markMsDirty();
    }

  postVecSuccess(di);
  markVsDirty();  // All Zvt instructions are considered to modify vector state.
}


template <typename URV>
void
Hart<URV>::execVtfmm_tvv(const DecodedInst* di)
{
  execVtfmm(di, false);
}


template <typename URV>
void
Hart<URV>::execVtfmm_alt_tvv(const DecodedInst* di)
{
  execVtfmm(di, true);
}

template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
