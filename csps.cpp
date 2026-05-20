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

#include <cassert>
#include "DecodedInst.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


// Spec §Smcsps "Instructions":
//   xcspspush:
//     xistatus.psppush = xistatus.sppush             (latch push bit)
//     if (xistatus.sppush == 1) {
//       tmp = sp; sp = xspcs; xspcs = tmp;            (swap sp with xspcs)
//       xistatus.sppush = 0;
//     }
//   xcspspop:
//     xistatus.sppush = xistatus.psppush
//     if (xistatus.psppush == 1) {
//       tmp = xspcs; xspcs = sp; sp = tmp;
//       xistatus.psppush = 0;
//     }
// xistatus.sppush is bit 6, psppush is bit 7.  xspcs is now a full XLEN-wide
// stack-pointer value (no internal PUSH/PPUSH bits; those moved to xistatus
// per spec PR #828).

template <typename URV>
void
Hart<URV>::mcspspush()
{
  URV mis = 0;
  if (not peekCsr(CsrNumber::MISTATUS, mis))
    assert(0);
  bool sppush  = (mis >> 6) & 1;
  bool psppush = sppush;                                // latch
  if (sppush)
    {
      URV sp = intRegs_.read(IntRegNumber::RegSp);
      URV mspcs = 0;
      (void) peekCsr(CsrNumber::MSPCS, mspcs);
      pokeCsr(CsrNumber::MSPCS, sp);
      intRegs_.write(IntRegNumber::RegSp, mspcs);
      sppush = false;
      csRegs_.recordWrite(CsrNumber::MSPCS);
    }
  mis = (mis & ~URV(0xC0)) | (URV(psppush) << 7) | (URV(sppush) << 6);
  pokeCsr(CsrNumber::MISTATUS, mis);
  csRegs_.recordWrite(CsrNumber::MISTATUS);
}


template <typename URV>
void
Hart<URV>::mcspspop()
{
  URV mis = 0;
  if (not peekCsr(CsrNumber::MISTATUS, mis))
    assert(0);
  bool psppush = (mis >> 7) & 1;
  bool sppush  = psppush;                               // restore
  if (psppush)
    {
      URV sp = intRegs_.read(IntRegNumber::RegSp);
      URV mspcs = 0;
      (void) peekCsr(CsrNumber::MSPCS, mspcs);
      pokeCsr(CsrNumber::MSPCS, sp);
      intRegs_.write(IntRegNumber::RegSp, mspcs);
      psppush = false;
      csRegs_.recordWrite(CsrNumber::MSPCS);
    }
  mis = (mis & ~URV(0xC0)) | (URV(psppush) << 7) | (URV(sppush) << 6);
  pokeCsr(CsrNumber::MISTATUS, mis);
  csRegs_.recordWrite(CsrNumber::MISTATUS);
}


template <typename URV>
void
Hart<URV>::execMcspspush(const DecodedInst* di)
{
  if (not isRvsmcsps() or privMode_ < PrivilegeMode::Machine)
    {
      illegalInst(di);
      return;
    }
  mcspspush();
}


template <typename URV>
void
Hart<URV>::execMcspspop(const DecodedInst* di)
{
  if (not isRvsmcsps() or privMode_ < PrivilegeMode::Machine)
    {
      illegalInst(di);
      return;
    }

  mcspspop();
}


template <typename URV>
void
Hart<URV>::scspspush()
{
  URV sis = 0;
  if (not peekCsr(CsrNumber::SISTATUS, sis))
    assert(0);
  bool sppush  = (sis >> 6) & 1;
  bool psppush = sppush;
  if (sppush)
    {
      URV sp = intRegs_.read(IntRegNumber::RegSp);
      URV sspcs = 0;
      (void) peekCsr(CsrNumber::SSPCS, sspcs);
      pokeCsr(CsrNumber::SSPCS, sp);
      intRegs_.write(IntRegNumber::RegSp, sspcs);
      sppush = false;
      csRegs_.recordWrite(CsrNumber::SSPCS);
    }
  sis = (sis & ~URV(0xC0)) | (URV(psppush) << 7) | (URV(sppush) << 6);
  pokeCsr(CsrNumber::SISTATUS, sis);
  csRegs_.recordWrite(CsrNumber::SISTATUS);
}


template <typename URV>
void
Hart<URV>::scspspop()
{
  URV sis = 0;
  if (not peekCsr(CsrNumber::SISTATUS, sis))
    assert(0);
  bool psppush = (sis >> 7) & 1;
  bool sppush  = psppush;
  if (psppush)
    {
      URV sp = intRegs_.read(IntRegNumber::RegSp);
      URV sspcs = 0;
      (void) peekCsr(CsrNumber::SSPCS, sspcs);
      pokeCsr(CsrNumber::SSPCS, sp);
      intRegs_.write(IntRegNumber::RegSp, sspcs);
      psppush = false;
      csRegs_.recordWrite(CsrNumber::SSPCS);
    }
  sis = (sis & ~URV(0xC0)) | (URV(psppush) << 7) | (URV(sppush) << 6);
  pokeCsr(CsrNumber::SISTATUS, sis);
  csRegs_.recordWrite(CsrNumber::SISTATUS);
}


template <typename URV>
void
Hart<URV>::execScspspush(const DecodedInst* di)
{
  if (not isRvs() or not isRvsscsps())
    {
      illegalInst(di);
      return;
    }

  if (privMode_ < PrivilegeMode::Supervisor)
    {
      illegalInst(di);  // Should we check virtMode_ and issue virtualInst.
      return;
    }

  scspspush();
}


template <typename URV>
void
Hart<URV>::execScspspop(const DecodedInst* di)
{
  if (not isRvs() or not isRvsscsps())
    {
      illegalInst(di);
      return;
    }

  if (privMode_ < PrivilegeMode::Supervisor)
    {
      illegalInst(di);  // Should we check virtMode_ and issue virtualInst.
      return;
    }

  scspspop();
}


template <typename URV>
void
Hart<URV>::execMipopret(const DecodedInst* di)
{
  if (not isRvsmip() or not isRvsmcsps() or privMode_ < PrivilegeMode::Machine)
    {
      illegalInst(di);
      return;
    }

  // Tempoarily set MPRV so that we can load from the interrupted context.
  unsigned savedMprv = mstatus_.bits_.MPRV;
  mstatus_.bits_.MPRV = 1;

  mcspspop();

  URV va = intRegs_.read(IntRegNumber::RegSp);
  unsigned regSize = sizeof(URV);
  bool hyper = false;

  ExceptionCause cause = ExceptionCause::NONE;

  uint64_t gpa1 = 0, gpa2 = 0;

  for (unsigned i = IntRegNumber::RegA0; i < IntRegNumber::RegA6; ++i, va += regSize)
    {
      auto regNum = IntRegNumber(i);

      // FIX: Should we evaluate load debug triggers.

      uint64_t pa1 = va, pa2 = va;
      gpa1 = va; gpa2 = va;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, regSize, hyper);
      if (cause != ExceptionCause::NONE)
        break;

      uint64_t value = 0;
      if (not readForLoad<URV>(di, va, pa1, pa2, value))
        assert(0);
      intRegs_.write(regNum, value);
    }

  mstatus_.bits_.MPRV = savedMprv;

  if (cause == ExceptionCause::NONE)
    {
      intRegs_.write(IntRegNumber::RegSp, va);
      execMret(di);
    }
  else
    {
      // We may re-execute mipopret when we return from the trap handler: Re-push
      // so that mcspspop will work.
      mcspspush();
      initiateLoadException(di, cause, va, gpa1);
    }
}


template <typename URV>
void
Hart<URV>::execSipopret(const DecodedInst* di)
{
  if (not isRvs() or not isRvssip() or not isRvsscsps())
    {
      illegalInst(di);
      return;
    }

  if (privMode_ < PrivilegeMode::Supervisor)
    {
      illegalInst(di);  // Should we check virtMode_ and issue virtualInst.
      return;
    }

  // Tempoarily set SUM (supervisor may access user pages) so that we can load from the
  // interrupted context if that context is User.
  unsigned savedSum = virtMem_.getSum();
  virtMem_.setSum(mstatus_.bits_.SUM);

  scspspop();

  URV va = intRegs_.read(IntRegNumber::RegSp);
  unsigned regSize = sizeof(URV);
  bool hyper = false;

  ExceptionCause cause = ExceptionCause::NONE;

  uint64_t gpa1 = 0, gpa2 = 0;

  for (unsigned i = IntRegNumber::RegA0; i < IntRegNumber::RegA6; ++i, va += regSize)
    {
      auto regNum = IntRegNumber(i);

      // FIX: Should we evaluate load debug triggers.

      uint64_t pa1 = va, pa2 = va;
      gpa1 = va; gpa2 = va;
      cause = determineLoadException(pa1, pa2, gpa1, gpa2, regSize, hyper);
      if (cause != ExceptionCause::NONE)
        break;

      uint64_t value = 0;
      if (not readForLoad<URV>(di, va, pa1, pa2, value))
        assert(0);
      intRegs_.write(regNum, value);
    }

  virtMem_.setSum(savedSum);

  if (cause == ExceptionCause::NONE)
    {
      intRegs_.write(IntRegNumber::RegSp, va);
      execSret(di);
    }
  else
    {
      // We may re-execute mipopret when we return from the trap handler: Re-push
      // so that mcspspop will work.
      scspspush();
      initiateLoadException(di, cause, va, gpa1);
    }
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
