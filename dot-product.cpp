// Copyright 2024 Tenstorrent Corporation or its affiliates.
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

#include <climits>
#include <cassert>
#include "DecodedInst.hpp"
#include "Hart.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execVqdot_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  auto sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqdotq)  or  sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0, e2 = 0;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = int8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqdot_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  auto sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqdotq)  or  sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  int32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = int8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqdotu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  auto sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqdotq)  or  sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      uint32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  uint32_t e1 = 0, e2 = 0;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      uint32_t b1 = uint8_t(e1 >> i*8);  // Ith byte of e1.
	      uint32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqdotu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  auto sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqdotq)  or  sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  uint32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      uint32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  uint32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      uint32_t b1 = uint8_t(e1 >> i*8);  // Ith byte of e1.
	      uint32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqdotsu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  auto sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqdotq)  or  sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1, vs2}))
    return;

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0;
	  uint32_t e2 = 0;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqdotsu_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  auto sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqdotq)  or  sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  uint32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  int32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = int8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = uint8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqdotus_vx(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqdotq) or sew != ElementWidth::Word)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  int32_t e2 = intRegs_.read(rs);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int32_t dest = 0;
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  uint32_t e1 = 0;
	  vecRegs_.read(vs1, ix, group, e1);

	  for (unsigned i = 0; i < sizeof(e1); ++i)
	    {
	      int32_t b1 = uint8_t(e1 >> i*8);  // Ith byte of e1.
	      int32_t b2 = int8_t(e2 >> i*8);  // Ith byte of e2.
	      dest += b1 * b2;
	    }
	}

      vecRegs_.write(vd, ix, destGroup, dest);
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqldotu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqldot8i) or sew != ElementWidth::Byte)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.
  unsigned elems = vecRegs_.elemMax();

  // Each vector source operand number must be a multiple of the group.
  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  unsigned mask = esg - 1;
  if (((vs1 | vs2) & mask) == 0)
    vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs1/vs2.
  else
    {
      postVecFail(di);
      return;
    }

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  bool op2Signed = vecRegs_.altHalfPrecision();

  int32_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      uint8_t e1 = 0;
      if (vecRegs_.isDestActive(vs1, ix, sgx8, masked, e1))
	{
          if (op2Signed)
            {
              int8_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += e1 * e2;
            }
          else
            {
              uint8_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += e1 * e2;
            }
        }
    }              

  vecRegs_.write(vd, 0, dgx8, dest);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqldots_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqldot8i) or sew != ElementWidth::Byte)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.
  unsigned elems = vecRegs_.elemMax();

  // Each vector source operand number must be a multiple of the group.
  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  unsigned mask = esg - 1;
  if (((vs1 | vs2) & mask) == 0)
    vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs1/vs2.
  else
    {
      postVecFail(di);
      return;
    }

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  bool op2Signed = vecRegs_.altHalfPrecision();

  int32_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int8_t e1 = 0;
      if (vecRegs_.isDestActive(vs1, ix, sgx8, masked, e1))
	{
          if (op2Signed)
            {
              int8_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += e1 * e2;
            }
          else
            {
              uint8_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += e1 * e2;
            }
        }
    }              

  vecRegs_.write(vd, 0, dgx8, dest);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqbdotu_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqbdot8i) or sew != ElementWidth::Byte)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();

  // Instruction asumes an LMUL of 8 for vs1, an an LMUL of 1 for vs2, and an LMUL of
  // ceil(8*EEW/VLEN) for the vd.  EEW is 8.
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned dg = ((8 * 8) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  unsigned elems = vecRegs_.elemMax();

  // Each vector source operand number must be a multiple of the group.
  bool ok = (vs1 & (s1g-1)) == 0 and (vs2 & (s2g-1)) == 0 and (vd & (dg-1)) == 0;
  if (ok)
    vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, esg/esg for vs1/vs2.
  else
    {
      postVecFail(di);
      return;
    }

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  bool op2Signed = vecRegs_.altHalfPrecision();

  for (unsigned n = 0; n < 8; ++n)
    {
      int32_t dest = 0;
      vecRegs_.read(vd, n, dgx8, dest);

      if (not masked or vecRegs_.isActive(0, n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              uint8_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += e1 * std::bit_cast<int8_t>(e2);
              else
                dest += e1 * e2;
          }
        }
      else   // Maksed element
        {
          if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
            dest = 0xffffffff;
        }

      vecRegs_.write(vd, n, dgx8, dest);
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVqbdots_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  ElementWidth sew = vecRegs_.elemWidth();

  if (not extensionIsEnabled(RvExtension::Zvqbdot8i) or sew != ElementWidth::Byte)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();

  // Instruction asumes an LMUL of 8 for vs1, an an LMUL of 1 for vs2, and an LMUL of
  // ceil(8*EEW/VLEN) for the vd.  EEW is 8.
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned dg = ((8 * 8) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  unsigned elems = vecRegs_.elemMax();

  // Each vector source operand number must be a multiple of the group.
  bool ok = (vs1 & (s1g-1)) == 0 and (vs2 & (s2g-1)) == 0 and (vd & (dg-1)) == 0;
  if (ok)
    vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, esg/esg for vs1/vs2.
  else
    {
      postVecFail(di);
      return;
    }

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  bool op2Signed = vecRegs_.altHalfPrecision();

  for (unsigned n = 0; n < 8; ++n)
    {
      int32_t dest = 0;
      vecRegs_.read(vd, n, dgx8, dest);

      if (not masked or vecRegs_.isActive(0, n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              int8_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += e1 * e2;
              else
                dest += e1 * std::bit_cast<uint8_t>(e2);
          }
        }
      else   // Maksed element
        {
          if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
            dest = 0xffffffff;
        }

      vecRegs_.write(vd, n, dgx8, dest);
    }

  postVecSuccess(di);
}

template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
