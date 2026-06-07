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
  bool op2Signed = vecRegs_.altmft();
  unsigned elems = vecRegs_.elemMax();
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
  bool op2Signed = vecRegs_.altmft();
  unsigned elems = vecRegs_.elemMax();
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
              dest += e1 * e2;
            }
          else
            {
              uint16_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += e1 * e2;
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
  if (not ok)
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
  bool op2Signed = vecRegs_.altmft();
  unsigned elems = vecRegs_.elemMax();
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
  bool op2Signed = vecRegs_.altmft();
  unsigned elems = vecRegs_.elemMax();
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
              dest += e1 * e2;
            }
          else
            {
              uint16_t e2 = 0;
              vecRegs_.read(vs2, ix, sgx8, e2);
              dest += e1 * e2;
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
  if (not ok)
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

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned ci = vs1 & 0x7; // Least 3 sig bit of vs1 are ci.
  vs1 = (vs1 >> 3) << 3;   // Clear least sig 3 bits of vs1.

  bool op2Signed = vecRegs_.altmft();
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
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
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

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned ci = vs1 & 0x7; // Least 3 sig bit of vs1 are ci.
  vs1 = (vs1 >> 3) << 3;   // Clear least sig 3 bits of vs1.

  bool op2Signed = vecRegs_.altmft();
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
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += e1 * std::bit_cast<int16_t>(e2);
              else
                dest += e1 * e2;
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
  tdi.setOp1((tdi.op1() >> 3) << 3);  // Clear least sig 3 bits of op1 (vs2 in spec).
  if (not checkVecIntInst(&tdi))   // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half and LMUL 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwbdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwbdota16i) and sew == Half) );
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  vs1 = (vs1 >> 3) << 3;   // Clear least sig 3 bits of vs1.

  // Instruction assumes an LMUL of 8 for vs1, an LMUL of 1 for vs2, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 8/16.
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned dg = ((8 * 8) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, s1g/s2g for vs1/vs2.

  // Each vector source operand number must be a multiple of the group.
  ok = ok and (vs1 & (s1g-1)) == 0 and (vs2 & (s2g-1)) == 0 and (vd & (dg-1)) == 0;
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  if (sew == Byte)
    vqwbdotau8_vv(di, s1gx8, s2gx8, dgx8);
  else
    vqwbdotau8_vv(di, s1gx8, s2gx8, dgx8);

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::vqwbdotas8_vv(const DecodedInst* di, unsigned s1gx8, unsigned s2gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned ci = vs1 & 0x7; // Least 3 sig bit of vs1 are ci.
  vs1 = (vs1 >> 3) << 3;   // Clear least sig 3 bits of vs1.

  bool op2Signed = vecRegs_.altmft();
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
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
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

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned ci = vs1 & 0x7; // Least 3 sig bit of vs1 are ci.
  vs1 = (vs1 >> 3) << 3;   // Clear least sig 3 bits of vs1.

  bool op2Signed = vecRegs_.altmft();
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
                  vecRegs_.read(vs1 + n, k, s1gx8, e1);
                  vecRegs_.read(vs2, k, s2gx8, e2);
                }
              if (op2Signed)
                dest += e1 * e2;
              else
                dest += e1 * std::bit_cast<uint16_t>(e2);
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
  tdi.setOp1((tdi.op1() >> 3) << 3);  // Clear least sig 3 bits of op1 (vs2 in spec).
  if (not checkVecIntInst(&tdi))   // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half and LMUL 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwbdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwbdota16i) and sew == Half) );
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  vs1 = (vs1 >> 3) << 3;   // Clear least sig 3 bits of vs1.

  // Instruction assumes an LMUL of 8 for vs1, an LMUL of 1 for vs2, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 8/16.
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned dg = ((8 * 8) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, s1g/s2g for vs1/vs2.

  // Each vector source operand number must be a multiple of the group.
  ok = ok and (vs1 & (s1g-1)) == 0 and (vs2 & (s2g-1)) == 0 and (vd & (dg-1)) == 0;
  if (not ok)
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

  // SEW must be 32 (word) and LMUL must be 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfbdota32f) and isFpLegal() and sew == Word;
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  // Instruction assumes an LMUL of 8 for vs1, an LMUL of 1 for vs2, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 32.
  unsigned s1g = 8, s2g = 1;
  unsigned s1gx8 = 8*s1g, s2gx8 = 8*s2g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned dg = ((8 * 8) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, s1g, s2g);   // For logging: 1 for vd, s1g/s2g for vs1/vs2.

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned ci = vs1 & 0x7; // Least 3 sig bit of vs1 are ci.
  vs1 = (vs1 >> 3) << 3;   // Clear least sig 3 bits of vs1.

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


// This is adapted from the risc-v spec. Code changed to C++.
// Single letter variable names changed to double letter to make them
// easier to find: n to nn. Variable names like A_i_isInf changed to aIsInf,
//
// nn is the static dimension of the dot product (a power of two).
//
// In this specification, the number of guard bits, gg, and the number of
// overflow bits, oo, are defined as:
// gg = oo = log2(nn)
//
// A[i] and B[i] are IEEE-encoded floating point numbers on (ee+pp) bits
// (MSB is sign, next ee bits are biased exponent, last mm bits are the mantissa)
// exponent bias is prodOpBias
// pp = mm + 1
// the output is an IEEE-encoded floating-point number on (ff+qq) bits
// ff is the output exponent width and
// qq is the size of the output significand (qq - 1 is the size of the output mantissa)
//
// Input type:
//   IFPT is the input floating point type: type of the dot product operands.
//   Operand bits (bit-cast of fp value) are passed as unsigned integers.
//
// Output type:
//   RFPT is the result floating point type.
//   Result bits (bit-cast of fp value) are returned as unsigned integer.
//
// Example: for dot-product of float16_t input resuling in a float output:
//   IFPT is float16_t
//   RFPT is float
//   A and B are of type vector<uint16_t> (each entry contains the bits of a float16_t)
//   Result is of type uint32_t (contains the bits of a float)
//       
template<typename IFPT, typename RFPT>
auto
bulkNormalizeDotProd(const std::vector<WdRiscv::getSameWidthUintType_t<IFPT>>& A,
                     const std::vector<WdRiscv::getSameWidthUintType_t<IFPT>>& B,
                     bool& invFpFlag, bool& ovFpFlag) -> WdRiscv::getSameWidthUintType_t<RFPT>
{
  assert(A.size() == B.size());           // Operand vec sizes must match.
  assert(sizeof(IFPT) <= 4);              // Cannot handle double operands
  assert(sizeof(RFPT) >= 2*sizeof(IFPT)); // Output format at least twice the width as input.

  invFpFlag = ovFpFlag = false;  // Invalid and overflow FP flags.

  uint32_t prodOpBias = std::numeric_limits<IFPT>::max_exponent - 1;
  uint32_t resBias = std::numeric_limits<RFPT>::max_exponent - 1;

  // mm: number of mantissa bits in input FP type.
  uint32_t mm = std::numeric_limits<IFPT>::digits - 1;

  // ee: number of bits in the biased exponent of input FP type.
  uint32_t ee = (sizeof(IFPT) * 8) - mm - 1;

  // qq: output significand size in bits
  uint32_t qq = std::numeric_limits<RFPT>::digits;

  // ff: output exponent size in bits
  uint32_t ff = (sizeof(RFPT) * 8) - qq;

  uint32_t nn = A.size();
  uint32_t oo = std::log2(nn);
  auto gg = oo;
  assert((uint32_t(1) << gg) == nn);

  uint32_t pp = mm + 1;

  uint32_t maxExp = 0;
  uint32_t maskExp = (1 << ee) - 1;      // bitmask for exponent
  uint32_t maskMant = (1 << mm) - 1;     // bitmask for mantissa
  std::vector<uint32_t> prodRefExps(nn); // array of product reference exponents
  std::vector<uint32_t> prodSigns(nn);   // array of product signs
  std::vector<uint64_t> prodSigs(nn);    // array of significand products

  // boundary for exponent overflow (output format)
  // this is also the output exponent for infinity and NaN
  uint64_t overflowExp = (1LL << ff) - 1;

  // predicate output special cases expected Not a Number (NaN) result
  bool nanResult = false;

   // expected infinite result
  bool infiniteResult = false;

  // invalid operation flag
  bool invalidFlag = false;

  // sign of infinite result
  uint64_t infiniteSign = 0;

  // determining maximum reference exponent
  for (uint32_t i = 0; i < nn; ++i)
    {
      // extracting A[i] and B[i]'s encoded exponents
      // (which are also used as reference exponents for product aligment)
      uint32_t aexp = (A.at(i) >> mm) & maskExp;     // Exponent of A[i]
      uint32_t bexp = (B.at(i) >> mm) & maskExp;     // Exponent of B[i]
      uint32_t amant = A.at(i) & maskMant;           // Mantissa of A[i]
      uint32_t bmant = B.at(i) & maskMant;           // Mantissa of B[i]
      uint32_t asign = (A.at(i) >> (ee + mm)) & 0x1; // Sign of A[i]
      uint32_t bsign = (B.at(i) >> (ee + mm)) & 0x1; // Sign of A[i]

      prodSigns.at(i) = asign ^ bsign;

      bool aIsSub =  aexp == 0;                   // A[i] is subnormal
      bool bIsSub =  bexp == 0;                   // B[i] is subnormal
      bool aIsZero = aIsSub and amant == 0;       // A[i] is zero
      bool bIsZero = bIsSub and bmant == 0;       // B[i] is zero
      bool prodIsZero = aIsZero or bIsZero;

      // detecting corner cases
      bool aIsInf = (aexp == maskExp) && (amant == 0);         // A[i] is inf
      bool bIsInf = (bexp == maskExp) && (bmant == 0);         // B[i] is inf
      bool aIsNan = (aexp == maskExp) && (amant != 0);         // A[i] is nan
      bool bIsNan = (bexp == maskExp) && (bmant != 0);         // B[i] is nan
      bool aIsSnan = aIsNan && (amant & (1 << (mm - 1))) == 0; // A[i] is snan
      bool bIsSnan = bIsNan && (bmant & (1 << (mm - 1))) == 0; // B[i] is snan

      bool invalidProd = (aIsInf and bIsZero) or (bIsInf and aIsZero);
      bool infiniteProdLHS = (aIsInf and !bIsNan  and !bIsZero);
      bool infiniteProdRHS = (bIsInf and !aIsNan  and !aIsZero);
      bool infiniteProd = infiniteProdLHS or infiniteProdRHS;
      bool invalidSum = infiniteResult and infiniteProd and (infiniteSign != prodSigns.at(i));

      infiniteResult = infiniteResult or infiniteProd;
      invalidFlag = invalidFlag or invalidProd or invalidSum or aIsSnan or bIsSnan;
      infiniteSign = infiniteProd ? prodSigns[i] : infiniteSign;

      nanResult = nanResult or aIsNan or bIsNan or invalidProd or invalidSum;

      uint32_t aSig = ((!aIsSub) << (pp - 1)) | amant;   // A[i] significand
      uint32_t bSig = ((!bIsSub) << (pp - 1)) | bmant;   // B[i] significand

      prodSigs.at(i) =  uint64_t(aSig) * bSig;

      uint32_t aRefExp = (aIsSub ? 1 : aexp);   // A[i] reference exponent
      uint32_t bRefExp = (bIsSub ? 1 : bexp);   // B[i] reference exponent

      // Sepc seems incorrect:
      //  prodRefExps.at(i) = prodIsZero ? 0 : aRefExp + bRefExp;
      prodRefExps.at(i) = prodIsZero ? 0 : aRefExp + bRefExp - prodOpBias;

      maxExp = (prodRefExps.at(i) > maxExp ? prodRefExps.at(i) : maxExp);
    }

  // early exit for special cases
  if (nanResult)
    {
      if (invalidFlag)
        invFpFlag = true; // raise invalid flag;
      // canonical quiet NaN
      return (overflowExp << (qq - 1)) | (1 << (qq - 2));
    }
  if (infiniteResult)
    return (infiniteSign << (qq + ff - 1)) | (overflowExp << (qq - 1));

  std::vector<uint64_t> alignedProducts(nn);
  // aligning products
  for (uint32_t i = 0; i < nn; ++i)
    {
      uint32_t alignShift = maxExp - prodRefExps.at(i);

      // aligning i-th product
      uint32_t padRight = qq + 1 + gg - (2 * pp);
      alignedProducts.at(i) = (prodSigs.at(i) << padRight) >> alignShift;

      // evaluating values of discarded bits
      // a mask is built to extract the discarded bits
      // - mask=0 if alignShift is <= q+1+g-2*p
      // - mask=(1 << (2*p)) - 1 if alignShift=q+1+g
      // For double, we would need uint128_t.
      uint64_t discardedMask = ((uint64_t(1) << (2*pp)) - 1) >> (qq + 1 + gg - alignShift);
      uint64_t discardedBits = prodSigs[i] & discardedMask;
      bool jam = (alignShift >= (qq+1+gg) ? prodSigs[i] : discardedBits) != 0;

      alignedProducts.at(i) |= (jam ? 1 : 0); // rounding to odd aligned product
    }

  // accumulating products
  int64_t accumulator = 0;
  for (uint32_t i = 0; i < nn; ++i)
    accumulator += prodSigns.at(i) ? -alignedProducts[i] : alignedProducts[i];

  // computing accumulator absolute value and normalizing it
  uint64_t accSign = accumulator < 0;
  uint64_t accAbs = accSign ? -accumulator : accumulator;

  // lzc: leading zero count assuming g + q + 1 + o width;
  uint32_t lzc = std::countl_zero(accAbs);
  lzc -= (sizeof(accAbs)*8) - (gg + qq + 1 + oo);

  int32_t resExp = accumulator == 0 ? 0 : ((maxExp + oo + 1 - lzc) - prodOpBias);
  uint64_t unroundedSig = (accAbs << lzc) >> (gg + oo + 1);
  uint64_t rawJamMask = (uint64_t(1) << (gg + oo + 1)) - 1;
  uint64_t jamMask = rawJamMask >> (lzc > (gg + oo + 1) ? 0 : (gg + oo + 1 - lzc));

  bool jamSig = ((accAbs << lzc) & jamMask) != 0;
  uint64_t roundedSig = unroundedSig | (jamSig ? 1 : 0);

  if (accAbs == 0)
    return 0; // a zero result is always +0

  if (resExp >= int32_t(overflowExp))
    {
      ovFpFlag = true;    // raise overflow flag;
      return (accSign << (qq + ff - 1)) | overflowExp << (qq - 1);
    }

  if (resExp >= 1)
    {
      // normal output
      uint64_t roundedMant = roundedSig & ((uint64_t(1) << (qq - 1)) - 1);
      // Spec seems incorrect:
      //  return (accSign << (qq + ff - 1)) | (uint64_t(resExp) << (qq - 1)) | roundedMant;
      return (accSign << (qq + ff - 1)) | (uint64_t(resExp + resBias) << (qq - 1)) | roundedMant;
    }

  if (resExp < - int32_t(qq - 1))
    return (accSign << (qq + ff - 1)) | (accAbs != 0 ? 1 : 0);

  // denormalization and final round-to-odd
  // (of bits discarded during denormalization)
  uint64_t denormalizedSig = accAbs >> (qq - 1 + resExp);
  uint64_t discardedMask = ((uint64_t(1) << (qq - 1)) - 1) >> (qq - 1 + resExp);
  uint64_t discardedBits = accAbs & discardedMask;
  uint64_t forceLSB = discardedBits != 0 ? 1 : 0;
  return (accSign << (qq + ff - 1)) | denormalizedSig | forceLSB;
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
  bool ok = extensionIsEnabled(Zvfwdota16bf) and sew == Half and vecRegs_.altmft();

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

  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  // Temporary until we implement bulk normalization.

  unsigned elems = vecRegs_.elemMax();
  bool masked = di->isMasked();

  std::vector<uint16_t> aa(elems);
  std::vector<uint16_t> bb(elems);

  for (unsigned ix = start; ix < elems; ++ix)
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
  uint32_t udp = bulkNormalizeDotProd<BFloat16, float>(aa, bb, inv, ovf);

  if (inv)
    raiseSimulatorFpFlags(FpFlags::Invalid);
  if (ovf)
    raiseSimulatorFpFlags(FpFlags::Overflow);

  float fdp = std::bit_cast<float>(udp);

  float dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);
  dest = doFadd(dest, fdp);

  vecRegs_.write(vd, 0, dgx8, dest);

  updateAccruedFpBits();
  postVecSuccess(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
