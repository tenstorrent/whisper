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
template <typename OP>
void
Hart<URV>::execVmop_mm(const DecodedInst* di, OP op)
{
  if (not checkSewLmulVstart(di))
    return;

  // Mask operation instructions (vmand.mm, vmor.mm, ...) cannot be masked.
  if (di->isMasked())
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.elemCount();
  unsigned vd = di->op0(), vs1 = di->op1(), vs2 = di->op2();

  if (start < elemCount)
    {
      unsigned count = vecRegs_.updateWholeMask() ? bitsPerReg : elemCount;

      for (unsigned ix = start; ix < count; ++ix)
	{
	  // Not masked. Tail elements computed.
	  bool in1 = false, in2 = false;
	  vecRegs_.readMaskRegister(vs1, ix, in1);
	  vecRegs_.readMaskRegister(vs2, ix, in2);
	  bool flag = op(unsigned(in1), unsigned(in2)) & 1;

	  vecRegs_.writeMaskRegister(vd, ix, flag);
	}

      // If not update-whole-mask and mask-agnostic-ones, fill tail with ones.
      if (vecRegs_.isTailAgnosticOnes())
	for (unsigned ix = count; ix < bitsPerReg; ++ix)
	  vecRegs_.writeMaskRegister(vd, ix, true);
    }

  vecRegs_.touchMask(vd);  // In case nothing was written.
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmand_mm(const DecodedInst* di)
{
  execVmop_mm(di, std::bit_and());
}


template <typename URV>
void
Hart<URV>::execVmnand_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitNand());
}


template <typename URV>
void
Hart<URV>::execVmandn_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitAndNot());
}


template <typename URV>
void
Hart<URV>::execVmxor_mm(const DecodedInst* di)
{
  execVmop_mm(di, std::bit_xor());
}


template <typename URV>
void
Hart<URV>::execVmor_mm(const DecodedInst* di)
{
  execVmop_mm(di, std::bit_or());
}


template <typename URV>
void
Hart<URV>::execVmnor_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitNor());
}


template <typename URV>
void
Hart<URV>::execVmorn_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitOrNot());
}


template <typename URV>
void
Hart<URV>::execVmxnor_mm(const DecodedInst* di)
{
  execVmop_mm(di, MyBitXnor());
}


template <typename URV>
void
Hart<URV>::execVcpop_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned rd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();

  uint32_t count = 0;
  for (uint32_t ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.isActive(vs1, ix))
        count++;
    }

  intRegs_.write(rd, count);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfirst_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned rd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();

  SRV first = -1;

  for (uint32_t ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
	continue;

      if (vecRegs_.isActive(vs1, ix))
	{
	  first = ix;
	  break;
	}
    }

  intRegs_.write(rd, first);
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsbf_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.updateWholeMask() ? bitsPerReg : vecRegs_.elemCount();

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  if (start < vecRegs_.elemCount())
    {
      bool found = false;  // true if set bit is found in vs1

      for (uint32_t ix = start; ix < elemCount; ++ix)
	{
	  bool flag = false;
	  if (vecRegs_.isMaskDestActive(vd, ix, masked, elemCount, flag))
	    {
	      bool input = false;
	      if (ix < vecRegs_.elemCount())
		vecRegs_.readMaskRegister(vs1, ix, input);
	      found = found or input;
	      flag = not found;
	    }
	  vecRegs_.writeMaskRegister(vd, ix, flag);
	}

      vecRegs_.touchMask(vd);
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsif_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.updateWholeMask() ? bitsPerReg : vecRegs_.elemCount();

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  if (start < vecRegs_.elemCount())
    {
      bool found = false;  // true if set bit is found in vs1

      for (uint32_t ix = start; ix < elemCount; ++ix)
	{
	  bool flag = false;
	  if (vecRegs_.isMaskDestActive(vd, ix, masked, elemCount, flag))
	    {
	      bool input = false;
	      if (ix < vecRegs_.elemCount())
		vecRegs_.readMaskRegister(vs1, ix, input);
	      flag = not found;
	      found = found or input;
	    }
	  vecRegs_.writeMaskRegister(vd, ix, flag);
	}

      vecRegs_.touchMask(vd);
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVmsof_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0)
    {
      postVecFail(di);
      return;
    }

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned bitsPerReg = vecRegs_.bitsPerRegister();
  unsigned elemCount = vecRegs_.updateWholeMask() ? bitsPerReg : vecRegs_.elemCount();

  if (vd == vs1 or (masked and vd == 0))
    {
      postVecFail(di);
      return;
    }

  if (start < vecRegs_.elemCount())
    {
      // True if masked-off elements should be set to 1.
      bool ones = vecRegs_.isMaskAgnostic() and vecRegs_.isMaskAgnosticOnes();

      bool found = false;  // true if set bit is found in vs1

      for (uint32_t ix = start; ix < elemCount; ++ix)
	{
	  bool flag = false;
	  bool active = vecRegs_.isMaskDestActive(vd, ix, masked, elemCount, flag);

	  bool input = false;
	  if (ix < vecRegs_.elemCount() and active)
	    vecRegs_.readMaskRegister(vs1, ix, input);

	  if (active)
	    vecRegs_.writeMaskRegister(vd, ix, false);
	  else if (ones)
	    vecRegs_.writeMaskRegister(vd, ix, true);

	  if (found or not input)
	    continue;

	  found = true;
	  vecRegs_.writeMaskRegister(vd, ix, true);
	}

      vecRegs_.touchMask(vd);  // In case nothing was written
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execViota_m(const DecodedInst* di)
{
  uint32_t start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1(),  elems = vecRegs_.elemCount();
  unsigned groupx8 = vecRegs_.groupMultiplierX8();
  unsigned group = groupx8 <= 8 ? 1 : groupx8/8;

  if (not preVecExec() or not vecRegs_.legalConfig() or start > 0 or
      (vs1 >= vd and vs1 < vd + group))
    {
      postVecFail(di);
      return;
    }

  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (masked and vd == 0)
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, groupx8, {vd}))
    return;

  unsigned sum = 0;

  unsigned destGroup = 8*group;

  elems = vecRegs_.elemMax();
  if (start < vecRegs_.elemCount())
    for (uint32_t ix = start; ix < elems; ++ix)
      {
	bool sourceSet = vecRegs_.isActive(vs1, ix);

	switch (sew)
	  {
	  case ElementWidth::Byte:
	    {
	      int8_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int8_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  case ElementWidth::Half:
	    {
	      int16_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int16_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  case ElementWidth::Word:
	    {
	      int32_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int32_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  case ElementWidth::Word2:
	    {
	      int64_t dest{};
	      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
		dest = int64_t(sum);
	      else
		sourceSet = false;
	      vecRegs_.write(vd, ix, destGroup, dest);
	    }
	    break;
	  default:
	    postVecFail(di);
	    return;
	  }
	if (sourceSet)
	  sum++;
      }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVid_v(const DecodedInst* di)
{
  // Spec does not mention vstart > 0. Got a clarification saying it
  // is ok not to take an exception in that case.
  uint32_t start = csRegs_.peekVstart();
  if (not checkSewLmulVstart(di))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  elems = vecRegs_.elemMax();

  if ((masked and vd == 0) or di->op1() != 0)
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {vd}))
    return;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  elems = vecRegs_.elemMax();
  if (start < vecRegs_.elemCount())
    for (uint32_t ix = start; ix < elems; ++ix)
      switch (sew)
	{
	case ElementWidth::Byte:
	  {
	    uint8_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint8_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;
	case ElementWidth::Half:
	  {
	    uint16_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint16_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;
	case ElementWidth::Word:
	  {
	    uint32_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint32_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;
	case ElementWidth::Word2:
	  {
	    uint64_t dest{};
	    if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	      dest = uint64_t(ix);
	    vecRegs_.write(vd, ix, destGroup, dest);
	  }
	  break;

	default: postVecFail(di); return;
	}

  postVecSuccess(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
