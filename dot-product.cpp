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
#include <bit>
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

  auto sew = vecRegs_.elemWidth();

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
Hart<URV>::vqwdotau8_vv(const DecodedInst* di, unsigned sgx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

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
}


template <typename URV>
void
Hart<URV>::vqwdotau16_vv(const DecodedInst* di, unsigned sgx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

  int64_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      uint16_t e1 = 0;
      if (vecRegs_.isDestActive(vs1, ix, sgx8, masked, e1))
	{
          if (op2Signed)
            {
              int16_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += int64_t(e1) * int64_t(e2);
            }
          else
            {
              uint16_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += int64_t(e1) * int64_t(e2);
            }
        }
    }              

  vecRegs_.write(vd, 0, dgx8, dest);
}


template <typename URV>
void
Hart<URV>::execVqwdotau_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwdota16i) and sew == Half) );

  unsigned vs1 = di->op1(),  vs2 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs1/vs2.

  // Each vector source operand number must be a multiple of the group.
  unsigned mask = esg - 1;
  ok = ok and ((vs1 | vs2) & mask) == 0;

  // The destination register (EMUL=1) must not overlap either source register
  // group (EMUL=esg); otherwise the instruction encoding is reserved (spec L43-44).
  unsigned vd = di->op0();
  bool srcOverlap = (vd >= vs1 and vd < vs1 + esg) or
                    (vd >= vs2 and vd < vs2 + esg);
  ok = ok and not srcOverlap;

  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  if (sew == Byte)
    vqwdotau8_vv(di, sgx8, dgx8);
  else
    vqwdotau16_vv(di, sgx8, dgx8);

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::vqwdotas8_vv(const DecodedInst* di, unsigned sgx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

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
}


template <typename URV>
void
Hart<URV>::vqwdotas16_vv(const DecodedInst* di, unsigned sgx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

  int64_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      int16_t e1 = 0;
      if (vecRegs_.isDestActive(vs1, ix, sgx8, masked, e1))
	{
          if (op2Signed)
            {
              int16_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += int64_t(e1) * int64_t(e2);
            }
          else
            {
              uint16_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += int64_t(e1) * int64_t(e2);
            }
        }
    }              

  vecRegs_.write(vd, 0, dgx8, dest);
}


template <typename URV>
void
Hart<URV>::execVqwdotas_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwdota16i) and sew == Half) );

  unsigned vs1 = di->op1(),  vs2 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs1/vs2.

  // Each vector source operand number must be a multiple of the group.
  unsigned mask = esg - 1;
  ok = ok and ((vs1 | vs2) & mask) == 0;

  // The destination register (EMUL=1) must not overlap either source register
  // group (EMUL=esg); otherwise the instruction encoding is reserved (spec L43-44).
  unsigned vd = di->op0();
  bool srcOverlap = (vd >= vs1 and vd < vs1 + esg) or
                    (vd >= vs2 and vd < vs2 + esg);
  ok = ok and not srcOverlap;

  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  if (sew == Byte)
    vqwdotas8_vv(di, sgx8, dgx8);
  else
    vqwdotas16_vv(di, sgx8, dgx8);

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::vqwbdotau8_vv(const DecodedInst* di, unsigned s1gx8, unsigned s2gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = vs2 & 0x7;
  vs2 = (vs2 >> 3) << 3;

  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemMax(ElementWidth::Byte);
  bool masked = di->isMasked();

  for (unsigned n = 0; n < 8; ++n)
    {
      int32_t dest = 0;
      vecRegs_.read(vd, ci + n, dgx8, dest);

      if (not masked or vecRegs_.isActive(0, ci + n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              uint8_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs2 + n, k, s1gx8, e1);
                  vecRegs_.read(vs1, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += e1 * std::bit_cast<int8_t>(e2);
              else
                dest += e1 * e2;
          }
        }
      else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
        dest = ~int32_t(0);

      vecRegs_.write(vd, ci + n, dgx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::vqwbdotau16_vv(const DecodedInst* di, unsigned s1gx8, unsigned s2gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = vs2 & 0x7;
  vs2 = (vs2 >> 3) << 3;

  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemMax(ElementWidth::Half);
  bool masked = di->isMasked();

  for (unsigned n = 0; n < 8; ++n)
    {
      int64_t dest = 0;
      vecRegs_.read(vd, ci + n, dgx8, dest);

      if (not masked or vecRegs_.isActive(0, ci + n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              uint16_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs2 + n, k, s1gx8, e1);
                  vecRegs_.read(vs1, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += int64_t(e1 * std::bit_cast<int16_t>(e2));
              else
                dest += int64_t(uint32_t(e1) * uint32_t(e2));  // uint32 avoids signed overflow when both are 65535
          }
        }
      else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
        dest = ~int64_t(0);

      vecRegs_.write(vd, ci + n, dgx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqwbdotau_vv(const DecodedInst* di)
{
  DecodedInst tdi = *di;  // Temp di
  tdi.setOp1((tdi.op1() >> 3) << 3);  // Clear ci bits from vs2.
  if (not checkVecIntInst(&tdi))   // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half and LMUL 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwbdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwbdota16i) and sew == Half) );
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();

  vs2 = (vs2 >> 3) << 3;   // Clear ci bits from vs2.

  // Instruction assumes an LMUL of 8 for vs2, an LMUL of 1 for vs1, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 8 or 16 (byte or half).
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, s1g/s2g for vs2/vs1.

  // Each vector source operand number must be a multiple of its group.
  ok = ok and (vs2 & (s1g-1)) == 0 and (vd & (dg-1)) == 0;
  // vd must not overlap the vs2 group (v[vs2..vs2+s1g-1]) or vs1 (reserved, spec L204-205).
  bool vdOverlapsVs2 = (vd >= vs2 and vd < vs2 + s1g);
  bool vdOverlapsVs1 = (vd == vs1);
  ok = ok and not vdOverlapsVs2 and not vdOverlapsVs1;
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // ci must be in range [0, ciMax): ci >= ciMax is reserved (spec L171-177).
  unsigned ci = di->op1() & 0x7;
  unsigned ciMax = vlen / (8 * 4 * eew);   // EEW_dest = 4*SEW bits
  if (ci >= ciMax)
    {
      postVecFail(di);
      return;
    }

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  if (sew == Byte)
    vqwbdotau8_vv(di, s1gx8, s2gx8, dgx8);
  else
    vqwbdotau16_vv(di, s1gx8, s2gx8, dgx8);

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::vqwbdotas8_vv(const DecodedInst* di, unsigned s1gx8, unsigned s2gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = vs2 & 0x7;
  vs2 = (vs2 >> 3) << 3;

  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemMax(ElementWidth::Byte);
  bool masked = di->isMasked();

  for (unsigned n = 0; n < 8; ++n)
    {
      int32_t dest = 0;
      vecRegs_.read(vd, ci + n, dgx8, dest);

      if (not masked or vecRegs_.isActive(0, ci + n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              int8_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs2 + n, k, s1gx8, e1);
                  vecRegs_.read(vs1, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += e1 * e2;
              else
                dest += e1 * std::bit_cast<uint8_t>(e2);
          }
        }
      else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
        dest = ~int32_t(0);

      vecRegs_.write(vd, ci + n, dgx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::vqwbdotas16_vv(const DecodedInst* di, unsigned s1gx8, unsigned s2gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = vs2 & 0x7;
  vs2 = (vs2 >> 3) << 3;

  bool op2Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemMax(ElementWidth::Half);
  bool masked = di->isMasked();

  for (unsigned n = 0; n < 8; ++n)
    {
      int64_t dest = 0;
      vecRegs_.read(vd, ci + n, dgx8, dest);

      if (not masked or vecRegs_.isActive(0, ci + n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              int16_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs2 + n, k, s1gx8, e1);
                  vecRegs_.read(vs1, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += int64_t(e1 * e2);
              else
                dest += int64_t(e1 * std::bit_cast<uint16_t>(e2));
          }
        }
      else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
        dest = ~int64_t(0);

      vecRegs_.write(vd, ci + n, dgx8, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVqwbdotas_vv(const DecodedInst* di)
{
  DecodedInst tdi = *di;  // Temp di
  tdi.setOp1((tdi.op1() >> 3) << 3);  // Clear ci bits from vs2.
  if (not checkVecIntInst(&tdi))   // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half and LMUL 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwbdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwbdota16i) and sew == Half) );
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();

  vs2 = (vs2 >> 3) << 3;   // Clear ci bits from vs2.

  // Instruction assumes an LMUL of 8 for vs2, an LMUL of 1 for vs1, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 8 or 16 (Byte or Half).
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, s1g/s2g for vs2/vs1.

  // Each vector source operand number must be a multiple of its group.
  ok = ok and (vs2 & (s1g-1)) == 0 and (vd & (dg-1)) == 0;
  // vd must not overlap the vs2 group (v[vs2..vs2+s1g-1]) or vs1 (reserved, spec L204-205).
  bool vdOverlapsVs2 = (vd >= vs2 and vd < vs2 + s1g);
  bool vdOverlapsVs1 = (vd == vs1);
  ok = ok and not vdOverlapsVs2 and not vdOverlapsVs1;
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // ci must be in range [0, ciMax): ci >= ciMax is reserved (spec L171-177).
  unsigned ci = di->op1() & 0x7;
  unsigned ciMax = vlen / (8 * 4 * eew);   // EEW_dest = 4*SEW bits
  if (ci >= ciMax)
    {
      postVecFail(di);
      return;
    }

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  if (sew == Byte)
    vqwbdotas8_vv(di, s1gx8, s2gx8, dgx8);
  else
    vqwbdotas16_vv(di, s1gx8, s2gx8, dgx8);

  postVecSuccess(di);
}


template<typename URV>
float
Hart<URV>::dotProdReduce(float acc, std::vector<float> prods, bool doTree, bool canonNan,
                         unsigned activeCount)
{
  float result = acc;
  if (not doTree)
    {
      for (auto prod : prods)
        result = doFadd(result, prod);
    }
  else
    {
      // Perform group-wise reduction first.
      doVecFpRedSumGroup(prods, ElementWidth::Word, 8 /*groupx8*/);

      // Perform adjacent vec register reduce.
      doVecFpRedSumAdjacent(prods, prods.size(), 2);

      // scalar operand in second-to-last step.
      float e1 = prods.at(0);
      result = doFadd(e1, acc);
      // URV incFlags = activeSimulatorFpFlags();
      // vecRegs_.fpFlags_.push_back(incFlags);
      // vecRegs_.steps_.emplace_back(VSO::ScalarRed, e1, acc, result);

      // remaining operand in last step.
      e1 = prods.at(1);
      acc = result;
      result = doFadd(e1, acc);
      // incFlags = activeSimulatorFpFlags();
      // vecRegs_.fpFlags_.push_back(incFlags);
      // vecRegs_.steps_.emplace_back(VSO::AdjacRed, e1, acc result);
    }

  // Note: NaN canonicalization when there are no active elements
  // is only allowed for vfredusum.vs and NOT for vfredosum.vs,
  // vfredmin.vs, and vfredmax.vs.
  if (activeCount and std::isnan(result) and canonNan)
    result = std::numeric_limits<float>::quiet_NaN();

  return result;
}


template <typename URV>
void
Hart<URV>::execVfbdota_vv(const DecodedInst* di)
{
  DecodedInst tdi = *di;  // Temp di
  tdi.setOp1((tdi.op1() >> 3) << 3);  // Clear least sig 3 bits of op1 (vs2 in spec).
  if (not checkVecIntInst(&tdi))   // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be 32 (word) and LMUL must be 1 (spec ldot-bdot.adoc L328).
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfbdota32f) and isFpLegal() and sew == Word;
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // Instruction assumes an LMUL of 8 for vs1, an LMUL of 1 for vs2, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 32.
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, s1g/s2g for vs1/vs2.

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned ci = (vs1 & 0x7) * 8; // ci field in vs1[2:0], scaled by 8 to element index.
  vs1 = (vs1 >> 3) << 3;         // Strip ci bits to get EMUL=8 group base register.

  // Spec: ci is reserved if ci_field >= VLEN/(8*EEW) (ldot-bdot.adoc L177).
  if ((ci / 8) >= vlen / (8 * eew))
    {
      postVecFail(di);
      return;
    }

  // Spec (ldot-bdot.adoc L204): vd must not overlap the vs2 EMUL=8 group or vs1.
  // In this function vs1(code)=vs2(spec) EMUL=8 group; vs2(code)=vs1(spec) EMUL=1.
  bool vs1Overlap = (vd + dg > vs1) and (vs1 + s1g > vd);
  bool vs2Overlap = (vd + dg > vs2) and (vs2 + s2g > vd);
  if (vs1Overlap or vs2Overlap)
    {
      postVecFail(di);
      return;
    }

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  // Clear any stale softfloat exception flags from prior instructions so that
  // updateAccruedFpBits() only sees flags raised by this instruction.
  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  // The FP32 products are first computed to full precision, setting the invalid operation
  // exception flag as appropriate. The products are then optionally rounded to FP32
  // according to the dynamic rounding mode, setting the inexact, overflow, and underflow
  // exception flags as appropriate. The sum of these products and the accumulator are
  // then computed as though by the vfredusum.vs instruction with SEW=32, including the
  // setting of exception flags.

  unsigned elems = vecRegs_.elemMax(ElementWidth::Word);
  bool masked = di->isMasked();

  bool treeReduce = vecRegs_.fpUnorderedSumTreeRed_.at(__builtin_ctz(sizeof(float)));
  bool canonNan = vecRegs_.fpUnorderedSumCanonical_.at(__builtin_ctz(sizeof(float)));
  bool roundDown = getFpRoundingMode() == RoundingMode::Down;

  std::vector<float> prods(elems);

  for (unsigned n = 0; n < 8; ++n)
    {
      float dest = 0;
      vecRegs_.read(vd, ci + n, dgx8, dest);

      if (not masked or vecRegs_.isActive(0, ci + n))
        {
          // Compute products
          unsigned activeCount = 0;
          for (unsigned k = 0; k < elems; ++k)
            {
              float e1 = 0, e2 = 0;
              prods.at(k) = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
                  prods.at(k) = doFmul(e1, e2);
                  activeCount++;
                }
              else if (treeReduce)
                prods.at(k) = roundDown? float(0) : -float(0);
          }

          dest = dotProdReduce(dest, prods, treeReduce, canonNan, activeCount);
        }
      else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
        dest = ~int32_t(0);

      vecRegs_.write(vd, ci + n, dgx8, dest);
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


// n is the static dimension of the dot product (a power of two)
// In this specification, the number of guard bits, g, and the number of
// overflow bits, o, are defined as:
// g = o = log2(n)
//
// A[i] are IEEE-encoded floating point numbers on (e_l+p_l) bits
// B[i] are IEEE-encoded floating point numbers on (e_r+p_r) bits
// MSB is sign, next e_l (resp. e_r) bits are biased exponent,
// last m_l (resp. m_r) bits are the mantissa.
// exponent bias is lhs_bias (resp. rhs_bias)
// p_l = m_l + 1
// p_r = m_r + 1
// the output is an IEEE-encoded floating-point number on (f+q) bits
// f is the output exponent width and
// q is the size of the output significand (the size of the output mantissa is q-1)

// LT: left input floating point type
// RT: right input floating point type
// OT: output floating point type
//
// The input operands A and B are passed as unsigned integers that contain the
// bits of floating point numbers.
//
// The result is an unsigned integer that contains the bits of a floating point
// number.
//
// Example: to do a dot product of float and float16 generating a float, we would have:
//   LT: uint32_t
//   RT: uint16_t
//   OT: uint32_t
template<typename LT, typename RT, typename OT>
auto
bulkNormalizeDotProd(const std::vector<WdRiscv::getSameWidthUintType_t<LT>>& A,
                     const std::vector<WdRiscv::getSameWidthUintType_t<RT>>& B,
                     bool& invFpFlag, bool& ovFpFlag) -> WdRiscv::getSameWidthUintType_t<OT>
{
  assert(A.size() == B.size());
  assert(sizeof(LT) <= 4);              // Cannot handle double operands
  assert(sizeof(RT) <= 4);              // Cannot handle double operands
  assert(sizeof(OT) >= 2*sizeof(LT));   // Output format at least twice the width as input.
  assert(sizeof(OT) >= 2*sizeof(RT));   // Output format at least twice the width as input.

  invFpFlag = ovFpFlag = false;  // Invalid and overflow FP flags.

  auto n = A.size();
  auto o = uint32_t(std::bit_width(n) - 1);  // Log2(n)
  auto g = o;
  assert((uint32_t(1) << g) == n);   // n must be a lower of 2.

  uint32_t maxExp = 0;

  // Left operand parameters
  uint32_t m_l = std::numeric_limits<LT>::digits - 1;   // Mantissa bit-count
  uint32_t e_l = (sizeof(LT)*8) - m_l - 1;              // Exp bit-count
  uint32_t p_l = m_l + 1;
  uint32_t maskExpLHS = (1 << e_l) - 1;                 // bitmask of exponent
  uint32_t maskMantLHS = (1 << m_l) - 1;                // bitmask for mantissa

  // Right operand parameters
  uint32_t m_r = std::numeric_limits<RT>::digits - 1;   // Mantissa bit-count
  uint32_t e_r = (sizeof(RT)*8) - m_r - 1;              // Exp bit-count
  uint32_t p_r = m_r + 1;
  uint32_t maskExpRHS = (1 << e_r) - 1;                 // Bitmask of exponent
  uint32_t maskMantRHS = (1 << m_r) - 1;                // Bitmask mantissa

  std::vector<uint32_t> prodRefExps(n);                 // Product reference exponents
  std::vector<uint32_t> prodSigns(n);                   // Product signs
  std::vector<uint64_t> prodSigs(n);                    // Product significands

  // Output parameters
  uint32_t q = std::numeric_limits<OT>::digits;         // Significand bit count
  uint32_t f = (sizeof(OT) * 8) - q;                    // Exp bit-count

  // boundary for exponent overflow (output format)
  // this is also the output exponent for infinity and NaN
  uint64_t overflowExp = (1LL << f) - 1;
  uint32_t lhs_bias = (1 << (e_l - 1)) - 1;
  uint32_t rhs_bias = (1 << (e_r - 1)) - 1;
  uint32_t res_bias = (1 << (f - 1)) - 1;
  uint32_t prodOpBias = lhs_bias + rhs_bias;

  // predicate output special cases
  auto nanResult = false;       // expected Not a Number (NaN) result
  auto infiniteResult = false;  // expected infinite result
  auto invalidFlag = false;     // invalid operation flag

  uint64_t infiniteSign = 0;    // sign of infinite result

  // determining maximum reference exponent
  for (size_t i = 0; i < n; ++i)
    {
      // extracting A[i] and B[i]'s encoded exponents
      // (which are also used as reference exponents for product alignment)
      uint32_t A_i_exp = (A.at(i) >> m_l) & maskExpLHS;
      uint32_t B_i_exp = (B.at(i) >> m_r) & maskExpRHS;
      uint32_t A_i_mant = (A.at(i) & maskMantLHS);
      uint32_t B_i_mant = (B.at(i) & maskMantRHS);
      uint32_t A_i_sign = (A.at(i) >> (e_l + m_l)) & 0x1;
      uint32_t B_i_sign = (B.at(i) >> (e_r + m_r)) & 0x1;

      prodSigns.at(i) = A_i_sign ^ B_i_sign;

      bool A_i_isSub = A_i_exp == 0;
      bool B_i_isSub = B_i_exp == 0;
      bool A_i_isZero = (A_i_isSub && A_i_mant == 0);
      bool B_i_isZero = (B_i_isSub && B_i_mant == 0);
      bool prod_isZero = A_i_isZero || B_i_isZero;

      // detecting corner cases
      bool A_i_isInf = (A_i_exp == maskExpLHS) && (A_i_mant == 0);
      bool B_i_isInf = (B_i_exp == maskExpRHS) && (B_i_mant == 0);
      bool A_i_isNaN = (A_i_exp == maskExpLHS) && (A_i_mant != 0);
      bool B_i_isNaN = (B_i_exp == maskExpRHS) && (B_i_mant != 0);
      bool A_i_isSNaN = A_i_isNaN && (A_i_mant & (1 << (m_l - 1))) == 0;
      bool B_i_isSNaN = B_i_isNaN && (B_i_mant & (1 << (m_r - 1))) == 0;

      bool invalidProd = (A_i_isInf && B_i_isZero) || (B_i_isInf && A_i_isZero);
      bool infiniteProdLHS = (A_i_isInf && !B_i_isNaN  && !B_i_isZero);
      bool infiniteProdRHS = (B_i_isInf && !A_i_isNaN  && !A_i_isZero);
      bool infiniteProd = infiniteProdLHS || infiniteProdRHS;
      bool invalidSum = infiniteResult && infiniteProd && (infiniteSign != prodSigns.at(i));

      infiniteResult = infiniteResult || infiniteProd;
      invalidFlag = invalidFlag || invalidProd || invalidSum || A_i_isSNaN || B_i_isSNaN;
      infiniteSign = infiniteProd ? prodSigns.at(i) : infiniteSign;

      nanResult = nanResult || A_i_isNaN || B_i_isNaN || invalidProd || invalidSum;

      uint32_t A_i_sig = ((!A_i_isSub) << (p_l - 1)) | A_i_mant;
      uint32_t B_i_sig = ((!B_i_isSub) << (p_r - 1)) | B_i_mant;

      prodSigs.at(i) =  uint64_t(A_i_sig) * B_i_sig;

      uint32_t A_i_ref_exp = (A_i_isSub ? 1 : A_i_exp);
      uint32_t B_i_ref_exp = (B_i_isSub ? 1 : B_i_exp);

      prodRefExps.at(i) = prod_isZero ? 0 : A_i_ref_exp + B_i_ref_exp;

      maxExp = (prodRefExps.at(i) > maxExp ? prodRefExps.at(i) : maxExp);
    }

  // early exit for special cases
  if (nanResult)
    {
      if (invalidFlag)
        invFpFlag = true; // raise invalid flag
      // canonical quiet NaN
      return (overflowExp << (q - 1)) | (1LL << (q - 2));
    }
  if (infiniteResult)
    return (infiniteSign << (q + f - 1)) | (overflowExp << (q - 1));

  std::vector<uint64_t> alignedProducts(n);
  // aligning products
  auto ep = p_l + p_r;  // Effecttive significand bit-count.
  for (size_t i = 0; i < n; ++i)
    {
      uint32_t alignShift = maxExp - prodRefExps.at(i);

      // aligning i-th product
      uint32_t padRight = q + 1 + g - ep; // Sepc: (p_l + p_r) instead of ep
      // A right shift count >= the operand width (64 for uint64_t) is undefined
      // behavior in C++; on this target it compiles to a hardware shift that
      // masks the count mod 64 instead of yielding zero, leaking a spurious
      // nonzero value into the aligned product for widely-separated exponents
      // (reachable with BF16's wide dynamic range). Spec intends a full-width
      // shift here, which always discards the operand once alignShift >= 64.
      alignedProducts.at(i) = alignShift < 64 ? (prodSigs.at(i) << padRight) >> alignShift : 0;

      // evaluating values of discarded bits
      // a mask is built to extract the discarded bits
      // - mask=0 if alignShift is <= q+1+g-(p_l + p_r)
      // - mask=(1 << (p_l + p_r)) - 1 if alignShift=q+1+g
      uint64_t discardedMask = ((1LL << ep) - 1) >> (q + 1 + g - alignShift); // Sepc: (p_l + p_r) instead of ep
      uint64_t discardedBits = prodSigs.at(i) & discardedMask;
      bool jam = (alignShift >= (q+1+g) ? prodSigs.at(i) : discardedBits) != 0;

      alignedProducts.at(i) |= (jam ? 1 : 0); // rounding to odd aligned product
    }

  // accumulating products
  int64_t accumulator = 0;
  for (size_t i = 0; i < n; ++i)
    {
      auto prod = std::bit_cast<int64_t>(alignedProducts.at(i));
      accumulator += prodSigns.at(i) ? -prod : prod;
    }

  // computing accumulator absolute value and normalizing it
  uint64_t accSign = accumulator < 0;
  uint64_t accAbs = accSign ? -accumulator : accumulator;
    
  // lzc: leading zero count assuming g + q + 1 + o width;
  uint32_t lzc = std::countl_zero(accAbs);
  lzc -= (sizeof(accAbs)*8) - (g + q + 1 + o);

  int32_t resExp = 0;
  if (accumulator != 0)
    resExp = static_cast<int32_t>((maxExp + o + 1u - lzc) - prodOpBias + res_bias);
  uint64_t unroundedSig = (accAbs << lzc) >> (g + o + 1);
  uint64_t rawJamMask = (1LL << (g + o + 1)) - 1;
  uint64_t jamMask = (rawJamMask >> (lzc > (g + o + 1) ? 0 : (g + o + 1 - lzc)));

  bool jamSig = ((accAbs << lzc) & jamMask) != 0;
  uint64_t roundedSig = unroundedSig | (jamSig ? 1 : 0);

  if (accAbs == 0)
    return 0; // a zero result is always +0

  if (resExp >= int32_t(overflowExp))
    {
      ovFpFlag = true; // raise overflow flag
      return (accSign << (q + f - 1)) | overflowExp << (q - 1);
    }

  if (resExp >= 1)
    {
      // normal output
      uint64_t roundedMant = roundedSig & ((1LL << (q - 1)) - 1);
      return (accSign << (q + f - 1)) | (uint64_t(resExp) << (q - 1)) | roundedMant;
    }

  if (resExp < - int32_t(q - 1))
    return (accSign << (q + f - 1)) | (accAbs != 0 ? 1 : 0);

  // denormalization and final round-to-odd
  // (of bits discarded during denormalization)
  uint64_t denormalizedSig = accAbs >> (q - 1 + resExp);
  uint64_t discardedMask = ((1 << (q - 1)) - 1) >> (q - 1 + resExp);
  uint64_t discardedBits = accAbs & discardedMask;
  uint64_t forceLSB =  (discardedBits != 0 ? 1 : 0);
  return (accSign << (q + f - 1)) | denormalizedSig | forceLSB;
}


template <typename URV>
void
Hart<URV>::execVfwdota_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))  // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be half. Altfmt must be 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfwdota16bf) and sew == Half and vecRegs_.altfmt();

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs1/vs2.

  // Each vector source operand number must be a multiple of the group.
  unsigned mask = esg - 1;
  ok = ok and ((vs1 | vs2) & mask) == 0;

  // vd (EMUL=1) must not overlap the vs1/vs2 source groups (spec L43-44).
  ok = ok and not (vd >= vs1 and vd < vs1 + esg);
  ok = ok and not (vd >= vs2 and vd < vs2 + esg);
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  unsigned start = 0;
  if (start >= vecRegs_.elemCount())
    return;

  unsigned elems = vecRegs_.elemMax();
  bool masked = di->isMasked();

  std::vector<uint16_t> aa(elems);
  std::vector<uint16_t> bb(elems);

  // Read only body elements; tail elements stay zero per the bulk-normalization
  // scheme. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  for (unsigned ix = start; ix < vecRegs_.elemCount(); ++ix)
    {
      uint16_t e1 = 0;
      if (vecRegs_.isDestActive(vs1, ix, sgx8, masked, e1))
	{
          uint16_t e2 = 0;
          vecRegs_.read(vs2, ix, sgx8, e2);
          aa.at(ix) = e1;
          bb.at(ix) = e2;
        }
    }              

  bool inv = false, ovf = false;
  uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(aa, bb, inv, ovf);

  if (inv)
    raiseSimulatorFpFlags(FpFlags::Invalid);
  if (ovf)
    raiseSimulatorFpFlags(FpFlags::Overflow);

  auto fdp = std::bit_cast<float>(udp);

  float dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);
  dest = doFadd(dest, fdp);

  vecRegs_.write(vd, 0, dgx8, dest);

  updateAccruedFpBits();
  postVecSuccess(di);
}


namespace WdRiscv
{
  extern
  uint16_t ofp8ToBfloat16(uint8_t x, bool e4m3);
}


template <typename URV>
void
Hart<URV>::execVfqwdota_vv(const DecodedInst* di)
{
  if (not checkVecIntInst(di))  // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be Byte.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfwdota16bf) and sew == Half;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs1/vs2.

  // Each vector source operand number must be a multiple of the group.
  unsigned mask = esg - 1;
  ok = ok and ((vs1 | vs2) & mask) == 0;
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  bool e4m3 = not vecRegs_.altfmt();  // OFP8 e4m3 when true and e5m2 when false.

  unsigned elems = vecRegs_.elemMax();
  bool masked = di->isMasked();

  // Temporary: Promote OFP8 format to BFloat16.
  std::vector<uint16_t> aa(elems);
  std::vector<uint16_t> bb(elems);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      uint8_t e1 = 0;
      if (vecRegs_.isDestActive(vs1, ix, sgx8, masked, e1))
	{
          uint8_t e2 = 0;
          vecRegs_.read(vs2, ix, sgx8, e2);
          aa.at(ix) = ofp8ToBfloat16(e1, e4m3);
          bb.at(ix) = ofp8ToBfloat16(e2, e4m3);
        }
    }              

  bool inv = false, ovf = false;
  uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(aa, bb, inv, ovf);

  if (inv)
    raiseSimulatorFpFlags(FpFlags::Invalid);
  if (ovf)
    raiseSimulatorFpFlags(FpFlags::Overflow);

  auto fdp = std::bit_cast<float>(udp);

  float dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);
  dest = doFadd(dest, fdp);

  vecRegs_.write(vd, 0, dgx8, dest);

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfqwbdota_vv(const DecodedInst* di)
{
  DecodedInst tdi = *di;  // Temp di
  tdi.setOp1((tdi.op1() >> 3) << 3);  // Clear least sig 3 bits of op1 (vs2 in spec).
  if (not checkVecIntInst(&tdi))   // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte and LMUL 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfqwbdota8f) and sew == Byte;
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned ci = (vs1 & 0x7) * 8;  // ci_field in bits[2:0] of rs2; scale to element index.
  vs1 = (vs1 >> 3) << 3;           // Clear ci bits from vs1 (vs2 group base).

  // Instruction assumes an LMUL of 8 for vs1, an LMUL of 1 for vs2, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 8 (byte).
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  // Spec: ci is reserved if ci_field >= VLEN/(8*EEW_dest) where EEW_dest = 4*SEW.
  if ((ci / 8) >= vlen / (32 * eew))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, s1g/s2g for vs1/vs2.

  // Each vector source operand number must be a multiple of the group.
  ok = ok and (vs1 & (s1g-1)) == 0 and (vs2 & (s2g-1)) == 0 and (vd & (dg-1)) == 0;
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // Spec: vd must not overlap the vs1 EMUL=8 group [vs1..vs1+s1g-1] or vs2 register.
  bool vs1Overlap = (vd + dg > vs1) and (vs1 + s1g > vd);
  bool vs2Overlap = (vd + dg > vs2) and (vs2 + s2g > vd);
  if (vs1Overlap or vs2Overlap)
    {
      postVecFail(di);
      return;
    }

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  bool e4m3 = not vecRegs_.altfmt();  // OFP8 e4m3 when true and e5m2 when false.

  unsigned elems = vecRegs_.elemMax(sew);
  bool masked = di->isMasked();

  std::vector<uint16_t> aa(elems);
  std::vector<uint16_t> bb(elems);

  for (unsigned n = 0; n < 8; ++n)
    {
      float dest = 0;
      vecRegs_.read(vd, ci + n, dgx8, dest);

      // Temporary: promote OFP8 format to BFloat16.
      if (not masked or vecRegs_.isActive(0, ci + n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              uint8_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
                }
              aa.at(k) = ofp8ToBfloat16(e1, e4m3);
              bb.at(k) = ofp8ToBfloat16(e2, e4m3);
            }

          bool inv = false, ovf = false;
          uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(aa, bb, inv, ovf);
          if (inv)
            raiseSimulatorFpFlags(FpFlags::Invalid);
          if (ovf)
            raiseSimulatorFpFlags(FpFlags::Overflow);

          auto fdp = std::bit_cast<float>(udp);
          dest = doFadd(dest, fdp);
        }
      else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
        dest = std::bit_cast<float>(~uint32_t(0));

      vecRegs_.write(vd, ci + n, dgx8, dest);
    }


  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfwbdota_vv(const DecodedInst* di)
{
  DecodedInst tdi = *di;
  tdi.setOp1((tdi.op1() >> 3) << 3);  // Clear ci bits from vs2 before mask-register conflict check.
  if (not checkVecIntInst(&tdi))   // Check mask-register conflict and vstart.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be half, LMUL must be 1, and vtype.altfmt must be set.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfwbdota16bf) and sew == Half and vecRegs_.altfmt();
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  unsigned vd  = di->op0();
  unsigned vs2 = di->op1();
  unsigned vs1 = di->op2();

  unsigned ci = (vs2 & 0x7) * 8;  // ci selects which group of 8 vd elements to update
  vs2 = (vs2 >> 3) << 3;          // strip ci bits to get EMUL=8 group base register

  // vs2 spans EMUL=8 (8 registers), vs1 spans EMUL=1, vd spans EMUL=ceil(8*EEW/VLEN).
  unsigned vs2g = 8, vs1g = 1;
  unsigned vs2gx8 = 8*vs2g, vs1gx8 = 8*vs1g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  // Spec: ci is reserved if ci >= VLEN/(8*EEW_dest), where EEW_dest=2*SEW.
  if ((ci / 8) >= vlen / (16 * eew))
    {
      postVecFail(di);
      return;
    }

  vecRegs_.setOpEmul(1, vs2g, vs1g);   // For logging: 1 for vd, vs2g/vs1g for vs2/vs1.

  // Each operand register number must be a multiple of its group size.
  ok = ok and (vs2 & (vs2g-1)) == 0 and (vs1 & (vs1g-1)) == 0 and (vd & (dg-1)) == 0;
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // Spec: vd must not overlap vs2 group [vs2..vs2+vs2g-1] or vs1 register.
  bool vs2Overlap = (vd + dg > vs2) and (vs2 + vs2g > vd);
  bool vs1Overlap = (vd + dg > vs1) and (vs1 + vs1g > vd);
  if (vs2Overlap or vs1Overlap)
    {
      postVecFail(di);
      return;
    }

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned elems = vecRegs_.elemMax(sew);
  bool masked = di->isMasked();

  std::vector<uint16_t> aa(elems);
  std::vector<uint16_t> bb(elems);

  for (unsigned n = 0; n < 8; ++n)
    {
      float dest = 0;
      vecRegs_.read(vd, ci + n, dgx8, dest);

      // Temporary: promote OFP8 format to BFloat16.
      if (not masked or vecRegs_.isActive(0, ci + n))
        {
          for (unsigned k = 0; k < elems; ++k)
            {
              uint16_t e1 = 0, e2 = 0;
              if (k < vecRegs_.elemCount())  // Not a tail elem
                {
                  vecRegs_.read(vs2 + n, k, vs2gx8, e1);
                  vecRegs_.read(vs1,     k, vs1gx8, e2);
                }
              aa.at(k) = e1;
              bb.at(k) = e2;
            }

          bool inv = false, ovf = false;
          uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(aa, bb, inv, ovf);
          if (inv)
            raiseSimulatorFpFlags(FpFlags::Invalid);
          if (ovf)
            raiseSimulatorFpFlags(FpFlags::Overflow);

          auto fdp = std::bit_cast<float>(udp);
          dest = doFadd(dest, fdp);
        }
      else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
        dest = std::bit_cast<float>(~uint32_t(0));

      vecRegs_.write(vd, ci + n, dgx8, dest);
    }


  updateAccruedFpBits();
  postVecSuccess(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
