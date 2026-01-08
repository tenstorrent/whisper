// Copyright 2025 Tenstorrent Corporation.
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
#include <cassert>
#include "crypto-util.hpp"
#include "functors.hpp"
#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


template<typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vzip_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned groupx8,
                   unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE dest{};

  unsigned destGroupx8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupx8*2);

  if (start >= vecRegs_.elemCount()*2)
    return;

  for (unsigned ix = start; ix < elems*2; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroupx8, masked, dest))
	{
          if ((ix % 2) == 0)
            vecRegs_.read(vs1, ix/2, groupx8, dest);
          else
            vecRegs_.read(vs2, ix/2, groupx8, dest);
	}
      vecRegs_.write(vd, ix, destGroupx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVzip_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  bool valid = isRvzvzip();
  valid = valid and groupx8 < 64;   // Reserved when LMUL is 8.
  if (not valid)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, groupx8))
    return;

  unsigned destGroupx8 = 2*groupx8;
  unsigned destGroup = destGroupx8 <= 8 ? 1 : destGroupx8 / 8;
  valid = (vd % destGroup) == 0;

  // The destination vector register group may overlap the source vector register group if
  // the overlap is in the highest-numbered part of the destination register group and the
  // source EMUL is at least 1. If the overlap violates these constraints, the instruction
  // encoding is reserved.
  if (destGroup > 1 and valid)
    {
      unsigned srcGroup = groupx8 <= 8 ? 1 : groupx8 / 8;
      bool ok = (vs1 + srcGroup <= vd)  or  (vd + destGroup <= vs1 + 1); // No overlap or overlap at vd + dg -1
      ok = ok and ((vs2 + srcGroup <= vd)  or  (vd + destGroup <= vs2 + 1));
      valid = ok;
    }
  if (not valid)
    {
      postVecFail(di);
      return;
    }

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vzip_vv<int8_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    case EW::Half:
      vzip_vv<int16_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    case EW::Word:
      vzip_vv<int32_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    case EW::Word2:
      vzip_vv<int64_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template<typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vunzip_v(unsigned vd, unsigned vs1, unsigned groupx8, unsigned start,
                    unsigned elems, bool masked, unsigned offset)
{
  ELEM_TYPE dest{};

  unsigned srcGroupx8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupx8*2);
  unsigned destGroupx8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupx8);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroupx8, masked, dest))
	{
          vecRegs_.read(vs1, 2*ix + offset, srcGroupx8, dest);
	}
      vecRegs_.write(vd, ix, srcGroupx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVunzip_v(const DecodedInst* di, unsigned offset)
{
  if (not checkVecIntInst(di))
    return;

  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  bool valid = isRvzvzip();
  valid = valid and groupx8 < 64;   // Reserved when LMUL is 8.
  if (not valid)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  unsigned srcGroupx8 = 2*groupx8;
  unsigned srcGroup = srcGroupx8 <= 8 ? 1 : srcGroupx8 / 8;
  unsigned destGroup = groupx8 <= 8 ? 1 : groupx8 / 8;
  valid = (vs1 % srcGroup) == 0;

  // The destination vector register group may overlap the source vector register group if
  // the overlap is in the lowest-numbered part of the source register group. If the
  // overlap violates these constraints, the instruction encoding is reserved.
  if (srcGroup > 1 and valid)
    {
      bool ok = (vs1 + srcGroup <= vd)  or  (vd + destGroup <= vs1 + 1); // No overlap or overlap at vs1
      valid = ok;
    }
  if (not valid)
    {
      postVecFail(di);
      return;
    }

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vunzip_v<int8_t>(vd, vs1, groupx8, start, elems, masked, offset);
      break;
    case EW::Half:
      vunzip_v<int16_t>(vd, vs1, groupx8, start, elems, masked, offset);
      break;
    case EW::Word:
      vunzip_v<int32_t>(vd, vs1, groupx8, start, elems, masked, offset);
      break;
    case EW::Word2:
      vunzip_v<int64_t>(vd, vs1, groupx8, start, elems, masked, offset);
      break;
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVunzipe_v(const DecodedInst* di)
{
  unsigned offset = 0;
  execVunzip_v(di, offset);
}


template <typename URV>
void
Hart<URV>::execVunzipo_v(const DecodedInst* di)
{
  unsigned offset = 1;
  execVunzip_v(di, offset);
}


template<typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vpaire_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned groupx8,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE dest{};

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, groupx8, masked, dest))
	{
          if ((ix % 2) == 0)
            vecRegs_.read(vs1, ix, groupx8, dest);
          else
            vecRegs_.read(vs2, ix - 1, groupx8, dest);
	}
      vecRegs_.write(vd, ix, groupx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVpaire_vv(const DecodedInst* di)
{
  bool valid = preVecExec() and isRvzvzip();

  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  unsigned group = groupx8 <= 8 ? 1 : groupx8 / 8;

  bool masked = di->isMasked();

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  valid = valid and (vd % group) == 0 and (vs1 % group) == 0 and (vs2 % group) == 0;

  // The destination register cannot overlap the source registers and, if masked, cannot
  // overlap the mask register.
  bool ok = (vs1 + group <= vd) or (vd + group < vs1);  // No dest overlap with vs1
  ok = ok and ((vs2 + group <= vd) or (vd + group < vs2)); // No dest overlap with vs2
  valid = valid and ok;
  if (masked)
    valid = valid and (vs1 > 0) and (vs2 > 0);
  
  if (not valid)
    {
      postVecFail(di);
      return;
    }
}


template<typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vpairo_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned groupx8,
                     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE dest{};

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, groupx8, masked, dest))
	{
          if ((ix % 2) == 0)
            {
              dest = 0;
              if (ix + 1 < vecRegs_.elemMax())
                vecRegs_.read(vs1, ix + 1, groupx8, dest);
            }
          else
            vecRegs_.read(vs2, ix, groupx8, dest);
	}
      vecRegs_.write(vd, ix, groupx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVpairo_vv(const DecodedInst* di)
{
  bool valid = preVecExec() and isRvzvzip();

  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  unsigned group = groupx8 <= 8 ? 1 : groupx8 / 8;

  bool masked = di->isMasked();

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  valid = valid and (vd % group) == 0 and (vs1 % group) == 0 and (vs2 % group) == 0;

  // The destination register cannot overlap the source registers and, if masked, cannot
  // overlap the mask register.
  bool ok = (vs1 + group <= vd) or (vd + group < vs1);  // No dest overlap with vs1
  ok = ok and ((vs2 + group <= vd) or (vd + group < vs2)); // No dest overlap with vs2
  valid = valid and ok;
  if (masked)
    valid = valid and (vs1 > 0) and (vs2 > 0);
  
  if (not valid)
    {
      postVecFail(di);
      return;
    }
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
