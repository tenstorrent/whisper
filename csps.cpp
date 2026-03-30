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


template <typename URV>
void
Hart<URV>::mcspspush()
{
  URV mspVal = 0;
  if (not peekCsr(CsrNumber::MSPCS, mspVal))
    assert(0);
  MspFields<URV> mspFields{mspVal};

  URV spVal = intRegs_.read(IntRegNumber::RegSp);

  mspFields.bits_.PPUSH = mspFields.bits_.PUSH;

  if (mspFields.bits_.PUSH)
    {
      auto tmp = spVal;
      spVal = mspFields.bits_.SP << 4;

      mspFields.bits_.SP = tmp >> 4;
      mspFields.bits_.PUSH = 0;

      if (not pokeCsr(CsrNumber::MSPCS, mspFields.value_))
        assert(0);
      csRegs_.recordWrite(CsrNumber::MSPCS);

      intRegs_.write(IntRegNumber::RegSp, spVal);
    }
}


template <typename URV>
void
Hart<URV>::mcspspop()
{
  URV mspVal = 0;
  if (not peekCsr(CsrNumber::MSPCS, mspVal))
    assert(0);
  MspFields<URV> mspFields{mspVal};

  URV spVal = intRegs_.read(IntRegNumber::RegSp);

  mspFields.bits_.PUSH = mspFields.bits_.PPUSH;

  if (mspFields.bits_.PPUSH)
    {
      auto tmp = mspFields.bits_.SP;
      mspFields.bits_.SP = spVal >> 4;
      mspFields.bits_.PPUSH = 0;
      spVal = tmp << 4;

      if (not pokeCsr(CsrNumber::MSPCS, mspFields.value_))
        assert(0);
      csRegs_.recordWrite(CsrNumber::MSPCS);

      intRegs_.write(IntRegNumber::RegSp, spVal);
    }
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
  URV sspVal = 0;
  if (not peekCsr(CsrNumber::SSPCS, sspVal))
    assert(0);
  MspFields<URV> sspFields{sspVal};   // MSPCS and SSPCS have identical layouts

  URV spVal = intRegs_.read(IntRegNumber::RegSp);

  sspFields.bits_.PPUSH = sspFields.bits_.PUSH;

  if (sspFields.bits_.PUSH)
    {
      auto tmp = spVal;
      spVal = sspFields.bits_.SP << 4;

      sspFields.bits_.SP = tmp >> 4;
      sspFields.bits_.PUSH = 0;

      if (not pokeCsr(CsrNumber::SSPCS, sspFields.value_))
        assert(0);
      csRegs_.recordWrite(CsrNumber::SSPCS);

      intRegs_.write(IntRegNumber::RegSp, spVal);
    }
}


template <typename URV>
void
Hart<URV>::scspspop()
{
  URV sspVal = 0;
  if (not peekCsr(CsrNumber::SSPCS, sspVal))
    assert(0);
  MspFields<URV> sspFields{sspVal};

  URV spVal = intRegs_.read(IntRegNumber::RegSp);

  sspFields.bits_.PUSH = sspFields.bits_.PPUSH;

  if (sspFields.bits_.PPUSH)
    {
      auto tmp = sspFields.bits_.SP;
      sspFields.bits_.SP = spVal >> 4;
      sspFields.bits_.PPUSH = 0;
      spVal = tmp << 4;

      if (not pokeCsr(CsrNumber::SSPCS, sspFields.value_))
        assert(0);
      csRegs_.recordWrite(CsrNumber::SSPCS);

      intRegs_.write(IntRegNumber::RegSp, spVal);
    }
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
