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
Hart<URV>::vabs_v(unsigned vd, unsigned vs1, unsigned groupx8,
                  unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, groupx8, masked, dest))
	{
          vecRegs_.read(vs1, ix, groupx8, e1);
          dest = e1 < 0 ? -e1 : e1;   // FIX: Spec does not specify abs(INT_MIN)
	}
      vecRegs_.write(vd, ix, groupx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVabs_v(const DecodedInst* di)
{
  if (not isRvzvabd())
    {
      postVecFail(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, vd, vs1, groupx8))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vabs_v<int8_t>(vd, vs1, groupx8, start, elems, masked);
      break;
    case EW::Half:
      vabs_v<int16_t>(vd, vs1, groupx8, start, elems, masked);
      break;
    case EW::Word:
      vabs_v<int32_t>(vd, vs1, groupx8, start, elems, masked);
      break;
    case EW::Word2:
      vabs_v<int64_t>(vd, vs1, groupx8, start, elems, masked);
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
Hart<URV>::vabd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned groupx8,
                   unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, groupx8, masked, dest))
	{
          vecRegs_.read(vs1, ix, groupx8, e1);
          vecRegs_.read(vs2, ix, groupx8, e2);
          dest = std::max(e1, e2) - std::min(e1, e2);
	}
      vecRegs_.write(vd, ix, groupx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVabd_vv(const DecodedInst* di)
{
  if (not isRvzvabd())
    {
      postVecFail(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), vs2 = di->op2();
  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, groupx8))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vabd_vv<int8_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    case EW::Half:
      vabd_vv<int16_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    case EW::Word:    // SEW boave Half is reserved.
    case EW::Word2:
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVabdu_vv(const DecodedInst* di)
{
  if (not isRvzvabd())
    {
      postVecFail(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(), vs2 = di->op2();
  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();
  
  if (not checkVecOpsVsEmul(di, vd, vs1, vs2, groupx8))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      vabd_vv<uint8_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    case EW::Half:
      vabd_vv<uint16_t>(vd, vs1, vs2, groupx8, start, elems, masked);
      break;
    case EW::Word:    // SEW boave Half is reserved.
    case EW::Word2:
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template<typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vwabda_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned groupx8,
                     unsigned start, unsigned elems, bool masked)
{
  using DWT = makeDoubleWide_t<ELEM_TYPE>; // Double wide type

  ELEM_TYPE e1{}, e2{};
  DWT dest{};

  // We take the max of lmul == 1 to compensate for tail elements.
  unsigned destGroupx8 = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), groupx8*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroupx8, masked, dest))
	{
	  vecRegs_.read(vs1, ix, groupx8, e1);
	  vecRegs_.read(vs2, ix, groupx8, e2);
          dest += std::max(e1, e2) - std::min(e1, e2);
	}
      vecRegs_.write(vd, ix, destGroupx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVwabda_vv(const DecodedInst* di)
{
  if (not isRvzvabd())
    {
      postVecFail(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, groupx8))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, groupx8))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwabda_vv<int8_t>(vd, vs1, vs2, groupx8, start, elems, masked); break;
    case EW::Half:  vwabda_vv<int16_t>(vd, vs1, vs2, groupx8, start, elems, masked); break;
    case EW::Word:  // SEW above Half reserved.
    case EW::Word2:
    default:        postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVwabdau_vv(const DecodedInst* di)
{
  if (not isRvzvabd())
    {
      postVecFail(di);
      return;
    }

  if (not checkVecIntInst(di))
    return;

  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, groupx8))
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned elems = vecRegs_.elemMax(dsew), start = csRegs_.peekVstart();

  if (not checkVecOpsVsEmulW0(di, vd, vs1, vs2, groupx8))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:  vwabda_vv<uint8_t>(vd, vs1, vs2, groupx8, start, elems, masked); break;
    case EW::Half:  vwabda_vv<uint16_t>(vd, vs1, vs2, groupx8, start, elems, masked); break;
    case EW::Word:  // SEW above Half reserved.
    case EW::Word2:
    default:        postVecFail(di); return;
    }

  postVecSuccess(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
