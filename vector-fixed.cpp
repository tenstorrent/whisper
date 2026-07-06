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

	  ELEM_TYPE2 temp = e1; // NOLINT(bugprone-signed-char-misuse)
	  temp += e2; // NOLINT(bugprone-signed-char-misuse)
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

	  ELEM_TYPE2 temp = e1; // NOLINT(bugprone-signed-char-misuse)
	  temp += e2; // NOLINT(bugprone-signed-char-misuse)
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

	  ELEM_TYPE2 temp = e1; // NOLINT(bugprone-signed-char-misuse)
	  temp -= e2; // NOLINT(bugprone-signed-char-misuse)
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

	  ELEM_TYPE2 temp = e1; // NOLINT(bugprone-signed-char-misuse)
	  temp -= e2; // NOLINT(bugprone-signed-char-misuse)
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
	      ELEM_TYPE2 temp = e1; // NOLINT(bugprone-signed-char-misuse)
	      temp *= e2; // NOLINT(bugprone-signed-char-misuse)
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
	      ELEM_TYPE2 temp = e1; // NOLINT(bugprone-signed-char-misuse)
	      temp *= e2; // NOLINT(bugprone-signed-char-misuse)
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
