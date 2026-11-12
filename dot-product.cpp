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
#include "bulk-norm.hpp"

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
  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
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
	  vecRegs_.read(vs2, ix, group, e1);
	  vecRegs_.read(vs1, ix, group, e2);

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
  unsigned vd = di->op0(),  vs2 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs2}))
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
	  vecRegs_.read(vs2, ix, group, e1);

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
  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
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
	  vecRegs_.read(vs2, ix, group, e1);
	  vecRegs_.read(vs1, ix, group, e2);

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
  unsigned vd = di->op0(),  vs2 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs2}))
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
	  vecRegs_.read(vs2, ix, group, e1);

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
  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
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
	  vecRegs_.read(vs2, ix, group, e1);
	  vecRegs_.read(vs1, ix, group, e2);

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
  unsigned vd = di->op0(),  vs2 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs2}))
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
	  vecRegs_.read(vs2, ix, group, e1);

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
  unsigned vd = di->op0(),  vs2 = di->op1(),  rs = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();

  if (not checkVecOpsVsEmul(di, group, {vd, vs2}))
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
	  vecRegs_.read(vs2, ix, group, e1);

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

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  bool vs1Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

  int32_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  // These instructions have a single destination element (EMUL=1); the mask gates the whole
  // dot product, not individual k-indexed source-element pairs, so check it once against
  // v0's bit 0 rather than re-checking (and zeroing) per source index (spec L34, L186-198).
  if (not masked or vecRegs_.isActive(0, 0))
    {
      for (unsigned ix = start; ix < elems; ++ix)
        {
          uint8_t e1 = 0;
          vecRegs_.read(vs2, ix, sgx8, e1);
          if (vs1Signed)
            {
              int8_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
              dest += e1 * e2;
            }
          else
            {
              uint8_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
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

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  bool vs1Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

  int64_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  // These instructions have a single destination element (EMUL=1); the mask gates the whole
  // dot product, not individual k-indexed source-element pairs, so check it once against
  // v0's bit 0 rather than re-checking (and zeroing) per source index (spec L34, L186-198).
  if (not masked or vecRegs_.isActive(0, 0))
    {
      for (unsigned ix = start; ix < elems; ++ix)
        {
          uint16_t e1 = 0;
          vecRegs_.read(vs2, ix, sgx8, e1);
          if (vs1Signed)
            {
              int16_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
              dest += int64_t(e1) * int64_t(e2);
            }
          else
            {
              uint16_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
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
  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

  if (not checkVecIntInst(di))
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwdota16i) and sew == Half) );

  unsigned vs2 = di->op1(),  vs1 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs2/vs1.

  // Each vector source operand number must be a multiple of the group.
  unsigned mask = esg - 1;
  ok = ok and ((vs2 | vs1) & mask) == 0;

  // The destination register (EMUL=1) must not overlap either source register
  // group (EMUL=esg); otherwise the instruction encoding is reserved (spec L43-44).
  unsigned vd = di->op0();
  bool srcOverlap = (vd >= vs2 and vd < vs2 + esg) or
                    (vd >= vs1 and vd < vs1 + esg);
  ok = ok and not srcOverlap;

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

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  bool vs1Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

  int32_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  // These instructions have a single destination element (EMUL=1); the mask gates the whole
  // dot product, not individual k-indexed source-element pairs, so check it once against
  // v0's bit 0 rather than re-checking (and zeroing) per source index (spec L34, L186-198).
  if (not masked or vecRegs_.isActive(0, 0))
    {
      for (unsigned ix = start; ix < elems; ++ix)
        {
          int8_t e1 = 0;
          vecRegs_.read(vs2, ix, sgx8, e1);
          if (vs1Signed)
            {
              int8_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
              dest += e1 * e2;
            }
          else
            {
              uint8_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
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

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  bool vs1Signed = vecRegs_.altfmt();
  unsigned elems = vecRegs_.elemCount();  // body length (vl); tail not summed. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
  bool masked = di->isMasked();

  int64_t dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  // These instructions have a single destination element (EMUL=1); the mask gates the whole
  // dot product, not individual k-indexed source-element pairs, so check it once against
  // v0's bit 0 rather than re-checking (and zeroing) per source index (spec L34, L186-198).
  if (not masked or vecRegs_.isActive(0, 0))
    {
      for (unsigned ix = start; ix < elems; ++ix)
        {
          int16_t e1 = 0;
          vecRegs_.read(vs2, ix, sgx8, e1);
          if (vs1Signed)
            {
              int16_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
              dest += int64_t(e1) * int64_t(e2);
            }
          else
            {
              uint16_t e2 = 0;
              vecRegs_.read(vs1, ix, sgx8, e2);
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
  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

  if (not checkVecIntInst(di))
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte/half.
  auto sew = vecRegs_.elemWidth();
  bool ok = ( (extensionIsEnabled(Zvqwdota8i) and sew == Byte) or
              (extensionIsEnabled(Zvqwdota16i) and sew == Half) );

  unsigned vs2 = di->op1(),  vs1 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs2/vs1.

  // Each vector source operand number must be a multiple of the group.
  unsigned mask = esg - 1;
  ok = ok and ((vs2 | vs1) & mask) == 0;

  // The destination register (EMUL=1) must not overlap either source register
  // group (EMUL=esg); otherwise the instruction encoding is reserved (spec L43-44).
  unsigned vd = di->op0();
  bool srcOverlap = (vd >= vs2 and vd < vs2 + esg) or
                    (vd >= vs1 and vd < vs1 + esg);
  ok = ok and not srcOverlap;

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
Hart<URV>::vqwbdotau8_vv(const DecodedInst* di, unsigned vs2gx8, unsigned vs1gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = (vs2 & 0x7) * 8;  // ci field in vs2[2:0], scaled by 8 to element index.
  vs2 = (vs2 >> 3) << 3;          // Clear ci bits to get EMUL=8 group base register.

  bool vs1Signed = vecRegs_.altfmt();
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
                  vecRegs_.read(vs2 + n, k, vs2gx8, e1);
                  vecRegs_.read(vs1,     k, vs1gx8, e2);
                }
              if (vs1Signed)
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
Hart<URV>::vqwbdotau16_vv(const DecodedInst* di, unsigned vs2gx8, unsigned vs1gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = (vs2 & 0x7) * 8;  // ci field in vs2[2:0], scaled by 8 to element index.
  vs2 = (vs2 >> 3) << 3;          // Clear ci bits to get EMUL=8 group base register.

  bool vs1Signed = vecRegs_.altfmt();
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
                  vecRegs_.read(vs2 + n, k, vs2gx8, e1);
                  vecRegs_.read(vs1,     k, vs1gx8, e2);
                }
              if (vs1Signed)
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

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

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

  vs2 = (vs2 >> 3) << 3;   // Clear ci bits to get EMUL=8 group base register.

  // Instruction assumes an LMUL of 8 for vs2, an LMUL of 1 for vs1, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 8 or 16 (byte or half).
  unsigned vs2g = 8, vs1g = 1;
  unsigned vs2gx8 = 8*vs2g, vs1gx8 = 8*vs1g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, vs2g, vs1g);   // For logging: 1 for vd, vs2g/vs1g for vs2/vs1.

  // Each vector source operand number must be a multiple of its group.
  ok = ok and (vs2 & (vs2g-1)) == 0 and (vs1 & (vs1g-1)) == 0 and (vd & (dg-1)) == 0;

  // Dest register group cannot overlap either source register group (spec L204-205).
  ok = ok and not hasDestSourceOverlap(vd, dgx8, vs2, vs2gx8)
          and not hasDestSourceOverlap(vd, dgx8, vs1, vs1gx8);

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

  if (sew == Byte)
    vqwbdotau8_vv(di, vs2gx8, vs1gx8, dgx8);
  else
    vqwbdotau16_vv(di, vs2gx8, vs1gx8, dgx8);

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::vqwbdotas8_vv(const DecodedInst* di, unsigned vs2gx8, unsigned vs1gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = (vs2 & 0x7) * 8;  // ci field in vs2[2:0], scaled by 8 to element index.
  vs2 = (vs2 >> 3) << 3;          // Clear ci bits to get EMUL=8 group base register.

  bool vs1Signed = vecRegs_.altfmt();
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
                  vecRegs_.read(vs2 + n, k, vs2gx8, e1);
                  vecRegs_.read(vs1,     k, vs1gx8, e2);
                }
              if (vs1Signed)
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
Hart<URV>::vqwbdotas16_vv(const DecodedInst* di, unsigned vs2gx8, unsigned vs1gx8, unsigned dgx8)
{
  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = (vs2 & 0x7) * 8;  // ci field in vs2[2:0], scaled by 8 to element index.
  vs2 = (vs2 >> 3) << 3;          // Clear ci bits to get EMUL=8 group base register.

  bool vs1Signed = vecRegs_.altfmt();
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
                  vecRegs_.read(vs2 + n, k, vs2gx8, e1);
                  vecRegs_.read(vs1,     k, vs1gx8, e2);
                }
              if (vs1Signed)
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

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

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

  vs2 = (vs2 >> 3) << 3;   // Clear ci bits to get EMUL=8 group base register.

  // Instruction assumes an LMUL of 8 for vs2, an LMUL of 1 for vs1, and an LMUL of
  // ceil(8*EEW/VLEN) for vd.  EEW is 8 or 16 (Byte or Half).
  unsigned vs2g = 8, vs1g = 1;
  unsigned vs2gx8 = 8*vs2g, vs1gx8 = 8*vs1g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, vs2g, vs1g);   // For logging: 1 for vd, vs2g/vs1g for vs2/vs1.

  // Each vector source operand number must be a multiple of its group.
  ok = ok and (vs2 & (vs2g-1)) == 0 and (vs1 & (vs1g-1)) == 0 and (vd & (dg-1)) == 0;

  // Dest register group cannot overlap either source register group (spec L204-205).
  ok = ok and not hasDestSourceOverlap(vd, dgx8, vs2, vs2gx8)
          and not hasDestSourceOverlap(vd, dgx8, vs1, vs1gx8);

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

  if (sew == Byte)
    vqwbdotas8_vv(di, vs2gx8, vs1gx8, dgx8);
  else
    vqwbdotas16_vv(di, vs2gx8, vs1gx8, dgx8);

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

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

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

  // vs2 spans EMUL=8 (8 registers), vs1 spans EMUL=1, vd spans EMUL=ceil(8*EEW/VLEN).
  // EEW is 32.
  unsigned vs2g = 8, vs1g = 1;
  unsigned vs2gx8 = 8*vs2g, vs1gx8 = 8*vs1g;
  unsigned vlen = vecRegs_.bitsPerRegister();
  unsigned eew = VecRegs::elemWidthInBits(sew);
  unsigned dg = ((8 * eew) + vlen - 1) / vlen;
  unsigned dgx8 = 8 * dg;  // Destination group times 8.

  vecRegs_.setOpEmul(1, vs2g, vs1g);   // For logging: 1 for vd, vs2g/vs1g for vs2/vs1.

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned ci = (vs2 & 0x7) * 8; // ci field in vs2[2:0], scaled by 8 to element index.
  vs2 = (vs2 >> 3) << 3;         // Strip ci bits to get EMUL=8 group base register.

  // Spec: ci is reserved if ci_field >= VLEN/(8*EEW) (ldot-bdot.adoc L177).
  if ((ci / 8) >= vlen / (8 * eew))
    {
      postVecFail(di);
      return;
    }

  // Spec (ldot-bdot.adoc L204): vd must not overlap the vs2 EMUL=8 group or vs1.
  bool vs2Overlap = (vd + dg > vs2) and (vs2 + vs2g > vd);
  bool vs1Overlap = (vd + dg > vs1) and (vs1 + vs1g > vd);
  if (vs2Overlap or vs1Overlap)
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
                  vecRegs_.read(vs2 + n, k, vs2gx8, e1);
                  vecRegs_.read(vs1, k, vs1gx8, e2);
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




// Combine RVBNA partition results (and the accumulator) using round-to-odd (RTO), per
// doc/zvdota.adoc's DPBulkNorm section ("Sum the resulting floating-point numbers and the
// accumulator input ... rounding to odd", L495-501) and RVBNARTO (L789-841). Uses
// sequential (not tree) reduction, matching vfredusum.vs's own base definition.
template <typename URV>
float
Hart<URV>::combinePartitionResultsRTO(float acc, const std::vector<float>& results)
{
#if SOFT_FLOAT
  int origRounding = softfloat_roundingMode;
  softfloat_roundingMode = softfloat_round_odd;
  // TBD FIX: what if not using SOFT_FLOAT
#endif

  float result = acc;
  for (float r : results)
    {
      bool overflowBefore = (activeSimulatorFpFlags() & unsigned(FpFlags::Overflow)) != 0;
      result = doFadd(result, r);
      bool overflowAfter = (activeSimulatorFpFlags() & unsigned(FpFlags::Overflow)) != 0;

      // RVBNA diverges from generic round-to-odd: an overflowing sum rounds to signed
      // infinity rather than round-to-odd's usual largest-finite-magnitude result (spec
      // L522-524, L831-840). Guard on a newly-set flag so a pre-existing overflow (from
      // an earlier, already-infinite partition result) isn't reapplied to a later,
      // unrelated, still-finite sum.
      if (overflowAfter and not overflowBefore and std::isfinite(result))
        result = std::copysign(std::numeric_limits<float>::infinity(), result);
    }

  // RVBNA (spec L814) permits only invalid-operation and overflow to be raised by this
  // instruction family. Round-to-odd only changes how an unrepresentable sum is rounded,
  // not whether IEEE-754 inexact/underflow get recorded, so clear those two here before
  // they leak into the architectural fflags CSR via the caller's updateAccruedFpBits().
#if SOFT_FLOAT
  softfloat_exceptionFlags &= ~(softfloat_flag_inexact | softfloat_flag_underflow);
#else
  std::feclearexcept(FE_INEXACT | FE_UNDERFLOW);
#endif

#if SOFT_FLOAT
  softfloat_roundingMode = origRounding;
#endif

  // RVBNA: a zero result is always positive, regardless of intermediate signs (spec
  // L796-797, L808-810).
  if (result == 0.0f)
    result = 0.0f;

  return result;
}


template <typename URV>
void
Hart<URV>::execVfwdota_vv(const DecodedInst* di)
{
  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

  if (not checkVecIntInst(di))  // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be half. Altfmt must be 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfwdota16bf) and sew == Half and vecRegs_.altfmt();

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs2/vs1.

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

  unsigned start = 0;
  if (start >= vecRegs_.elemCount())
    return;

  // Clear any stale softfloat exception flags from prior instructions so that
  // updateAccruedFpBits() only sees flags raised by this instruction.
  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  unsigned elems = vecRegs_.elemMax();
  bool masked = di->isMasked();

  // RVBNA partitioning (see VecRegs::dotProdPartitionSize()); elems % partSize != 0 is a
  // reserved encoding. Checked before the mask gate below since it depends only on vtype,
  // never on any data read.
  unsigned partSize = vecRegs_.dotProdPartitionSize(sew, elems);
  if (partSize == 0 or elems % partSize != 0)
    {
      postVecFail(di);
      return;
    }
  unsigned numParts = elems / partSize;
  auto scheme = vecRegs_.dotProdPartitionScheme(sew);

  float dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  // These instructions have a single destination element (EMUL=1); the mask gates the whole
  // dot product, not individual k-indexed source-element pairs, so check it once against
  // v0's bit 0 rather than re-checking (and zeroing) per source index (spec L34, L186-198).
  if (not masked or vecRegs_.isActive(0, 0))
    {
      std::vector<uint16_t> aa(elems);
      std::vector<uint16_t> bb(elems);

      // Read only body elements; tail elements stay zero per the bulk-normalization
      // scheme. elemMax (VLMAX) over-reads a fractional-LMUL source group -> invalid index.
      for (unsigned ix = start; ix < vecRegs_.elemCount(); ++ix)
        {
          uint16_t e1 = 0, e2 = 0;
          vecRegs_.read(vs2, ix, sgx8, e1);
          vecRegs_.read(vs1, ix, sgx8, e2);
          aa.at(ix) = e1;
          bb.at(ix) = e2;
        }

      std::vector<float> partResults(numParts);
      bool inv = false, ovf = false;
      for (unsigned p = 0; p < numParts; ++p)
        {
          std::vector<uint16_t> pa(partSize), pb(partSize);
          for (unsigned i = 0; i < partSize; ++i)
            {
              unsigned member = dotProdPartitionMember(scheme, p, i, partSize);
              pa.at(i) = aa.at(member);
              pb.at(i) = bb.at(member);
            }

          bool partInv = false, partOvf = false;
          uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(pa, pb, partInv, partOvf);
          partResults.at(p) = std::bit_cast<float>(udp);
          inv = inv or partInv;
          ovf = ovf or partOvf;
        }

      if (inv)
        raiseSimulatorFpFlags(FpFlags::Invalid);
      if (ovf)
        raiseSimulatorFpFlags(FpFlags::Overflow);

      dest = combinePartitionResultsRTO(dest, partResults);
    }
  else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
    dest = std::bit_cast<float>(~uint32_t(0));

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
  // The Zvdota family reserves executing with a nonzero vstart (spec L40-41).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

  if (not checkVecIntInst(di))  // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be Byte.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfqwdota8f) and sew == Byte;

  unsigned vd = di->op0(),  vs2 = di->op1(),  vs1 = di->op2();
  unsigned sgx8 = vecRegs_.groupMultiplierX8();  // Source group times 8
  unsigned dgx8 = 8;  // Destination group times 8.

  unsigned esg = sgx8 < 8 ? 1 : sgx8/8;  // Effective source group
  vecRegs_.setOpEmul(1, esg, esg);   // For logging: 1 for vd, esg/esg for vs2/vs1.

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

  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  // Clear any stale softfloat exception flags from prior instructions so that
  // updateAccruedFpBits() only sees flags raised by this instruction.
  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  // vs1's format follows vtype.altfmt (spec L134-135); vs2's format is fixed
  // by the mnemonic/funct6 -- E4M3 for vfqwdota.vv, E5M2 for vfqwdota.alt.vv
  // (spec L131-133) -- independent of altfmt.
  bool vs1E4m3 = not vecRegs_.altfmt();
  bool vs2E4m3 = ((di->inst() >> 26) & 1) == 0;

  // Bulk normalization substitutes inactive/tail source elements with zero
  // and proceeds as though vl=VLMAX (spec L483-484); elemMax() instead
  // extends past VLMAX to cover a full physical register for fractional
  // LMUL, which is a destination-tail concept and does not apply to these
  // EMUL=LMUL source reads -- iterating that far throws for LMUL<1 groups
  // since vs1/vs2's valid physical range is bounded by VLMAX, not a full
  // register.
  unsigned elems = vecRegs_.vlmax();
  bool masked = di->isMasked();

  // RVBNA partitioning (see VecRegs::dotProdPartitionSize()); elems % partSize != 0 is a
  // reserved encoding. Checked before the mask gate below since it depends only on vtype,
  // never on any data read.
  unsigned partSize = vecRegs_.dotProdPartitionSize(sew, elems);
  if (partSize == 0 or elems % partSize != 0)
    {
      postVecFail(di);
      return;
    }
  unsigned numParts = elems / partSize;
  auto scheme = vecRegs_.dotProdPartitionScheme(sew);

  float dest = 0;
  vecRegs_.read(vd, 0, dgx8, dest);

  // These instructions have a single destination element (EMUL=1); the mask gates the whole
  // dot product, not individual k-indexed source-element pairs, so check it once against
  // v0's bit 0 rather than re-checking (and zeroing) per source index (spec L34, L186-198).
  if (not masked or vecRegs_.isActive(0, 0))
    {
      // Temporary: Promote OFP8 format to BFloat16.
      std::vector<uint16_t> aa(elems);
      std::vector<uint16_t> bb(elems);

      for (unsigned ix = start; ix < elems; ++ix)
        {
          uint8_t e1 = 0, e2 = 0;
          if (ix < vecRegs_.elemCount())  // Not a tail elem
            {
              vecRegs_.read(vs1, ix, sgx8, e1);
              vecRegs_.read(vs2, ix, sgx8, e2);
            }
          aa.at(ix) = ofp8ToBfloat16(e1, vs1E4m3);
          bb.at(ix) = ofp8ToBfloat16(e2, vs2E4m3);
        }

      std::vector<float> partResults(numParts);
      bool inv = false, ovf = false;
      for (unsigned p = 0; p < numParts; ++p)
        {
          std::vector<uint16_t> pa(partSize), pb(partSize);
          for (unsigned i = 0; i < partSize; ++i)
            {
              unsigned member = dotProdPartitionMember(scheme, p, i, partSize);
              pa.at(i) = aa.at(member);
              pb.at(i) = bb.at(member);
            }

          bool partInv = false, partOvf = false;
          uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(pa, pb, partInv, partOvf);
          partResults.at(p) = std::bit_cast<float>(udp);
          inv = inv or partInv;
          ovf = ovf or partOvf;
        }

      if (inv)
        raiseSimulatorFpFlags(FpFlags::Invalid);
      if (ovf)
        raiseSimulatorFpFlags(FpFlags::Overflow);

      dest = combinePartitionResultsRTO(dest, partResults);
    }
  else if (vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes())
    dest = std::bit_cast<float>(~uint32_t(0));

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

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

  if (not checkVecIntInst(&tdi))   // Check dest/mask and source/mask overlap, vstart > 0.
    return;

  using enum RvExtension;
  using enum ElementWidth;

  // SEW must be byte and LMUL 1.
  auto sew = vecRegs_.elemWidth();
  bool ok = extensionIsEnabled(Zvfqwbdota8f) and sew == Byte;
  ok = ok and vecRegs_.groupMultiplierX8() == 8;  // LMUL must be 1.

  unsigned vd = di->op0();
  unsigned vs2 = di->op1();
  unsigned vs1 = di->op2();

  unsigned ci = (vs2 & 0x7) * 8;  // ci_field in bits[2:0] of vs2; scale to element index.
  vs2 = (vs2 >> 3) << 3;           // Clear ci bits from vs2 (EMUL=8 group base).

  // vs2 spans EMUL=8 (8 registers), vs1 spans EMUL=1, vd spans EMUL=ceil(8*EEW/VLEN).
  // EEW is 8 (byte).
  unsigned vs2g = 8, vs1g = 1;
  unsigned vs2gx8 = 8*vs2g, vs1gx8 = 8*vs1g;
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

  vecRegs_.setOpEmul(1, vs2g, vs1g);   // For logging: 1 for vd, vs2g/vs1g for vs2/vs1.

  // Each vector source operand number must be a multiple of the group.
  ok = ok and (vs2 & (vs2g-1)) == 0 and (vs1 & (vs1g-1)) == 0 and (vd & (dg-1)) == 0;
  if (not ok)
    {
      postVecFail(di);
      return;
    }

  // Spec: vd must not overlap the vs2 EMUL=8 group [vs2..vs2+vs2g-1] or vs1 register.
  bool vs2Overlap = (vd + dg > vs2) and (vs2 + vs2g > vd);
  bool vs1Overlap = (vd + dg > vs1) and (vs1 + vs1g > vd);
  if (vs2Overlap or vs1Overlap)
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  // Clear any stale softfloat exception flags from prior instructions so that
  // updateAccruedFpBits() only sees flags raised by this instruction.
  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  // vs1's format follows vtype.altfmt (spec L298-299); vs2's format is fixed
  // by the mnemonic/funct6 -- E4M3 for vfqwbdota.vv, E5M2 for vfqwbdota.alt.vv
  // (spec L295-297) -- independent of altfmt.
  bool vs1E4m3 = not vecRegs_.altfmt();
  bool vs2E4m3 = ((di->inst() >> 26) & 1) == 0;

  unsigned elems = vecRegs_.elemMax(sew);
  bool masked = di->isMasked();

  // RVBNA partitioning (see VecRegs::dotProdPartitionSize()); elems % partSize != 0 is a
  // reserved encoding. Not to be confused with the per-lane loop variable n (in [0,8)) below.
  unsigned partSize = vecRegs_.dotProdPartitionSize(sew, elems);
  if (partSize == 0 or elems % partSize != 0)
    {
      postVecFail(di);
      return;
    }
  unsigned numParts = elems / partSize;
  auto scheme = vecRegs_.dotProdPartitionScheme(sew);

  std::vector<uint16_t> aa(elems);
  std::vector<uint16_t> bb(elems);
  std::vector<uint16_t> pa(partSize);
  std::vector<uint16_t> pb(partSize);
  std::vector<float> partResults(numParts);

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
                  vecRegs_.read(vs2 + n, k, vs2gx8, e1);
                  vecRegs_.read(vs1, k, vs1gx8, e2);
                }
              aa.at(k) = ofp8ToBfloat16(e1, vs2E4m3);
              bb.at(k) = ofp8ToBfloat16(e2, vs1E4m3);
            }

          bool inv = false, ovf = false;
          for (unsigned p = 0; p < numParts; ++p)
            {
              for (unsigned i = 0; i < partSize; ++i)
                {
                  unsigned member = dotProdPartitionMember(scheme, p, i, partSize);
                  pa.at(i) = aa.at(member);
                  pb.at(i) = bb.at(member);
                }

              bool partInv = false, partOvf = false;
              uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(pa, pb, partInv, partOvf);
              partResults.at(p) = std::bit_cast<float>(udp);
              inv = inv or partInv;
              ovf = ovf or partOvf;
            }

          if (inv)
            raiseSimulatorFpFlags(FpFlags::Invalid);
          if (ovf)
            raiseSimulatorFpFlags(FpFlags::Overflow);

          dest = combinePartitionResultsRTO(dest, partResults);
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

  // The Zvbdota family reserves executing with a nonzero vstart (spec L207-208).
  if (csRegs_.peekVstart() != 0)
    {
      postVecFail(di);   // Illegal-instruction exceptions leave vstart unchanged.
      return;
    }

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

  unsigned start = csRegs_.peekVstart();
  if (start >= vecRegs_.elemCount())
    return;

  // Clear any stale softfloat exception flags from prior instructions so that
  // updateAccruedFpBits() only sees flags raised by this instruction.
  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  unsigned elems = vecRegs_.elemMax(sew);
  bool masked = di->isMasked();

  // RVBNA partitioning (see VecRegs::dotProdPartitionSize()); elems % partSize != 0 is a
  // reserved encoding. Not to be confused with the per-lane loop variable n (in [0,8)) below.
  unsigned partSize = vecRegs_.dotProdPartitionSize(sew, elems);
  if (partSize == 0 or elems % partSize != 0)
    {
      postVecFail(di);
      return;
    }
  unsigned numParts = elems / partSize;
  auto scheme = vecRegs_.dotProdPartitionScheme(sew);

  std::vector<uint16_t> aa(elems);
  std::vector<uint16_t> bb(elems);
  std::vector<uint16_t> pa(partSize);
  std::vector<uint16_t> pb(partSize);
  std::vector<float> partResults(numParts);

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
          for (unsigned p = 0; p < numParts; ++p)
            {
              for (unsigned i = 0; i < partSize; ++i)
                {
                  unsigned member = dotProdPartitionMember(scheme, p, i, partSize);
                  pa.at(i) = aa.at(member);
                  pb.at(i) = bb.at(member);
                }

              bool partInv = false, partOvf = false;
              uint32_t udp = bulkNormalizeDotProd<BFloat16, BFloat16, float>(pa, pb, partInv, partOvf);
              partResults.at(p) = std::bit_cast<float>(udp);
              inv = inv or partInv;
              ovf = ovf or partOvf;
            }

          if (inv)
            raiseSimulatorFpFlags(FpFlags::Invalid);
          if (ovf)
            raiseSimulatorFpFlags(FpFlags::Overflow);

          dest = combinePartitionResultsRTO(dest, partResults);
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
