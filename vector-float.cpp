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
#include "float-util.hpp"


using namespace WdRiscv;


namespace WdRiscv
{
  template <> struct makeDoubleWide<Float16>    { using type = float; };
  template <> struct makeDoubleWide<BFloat16>   { using type = float; };
  template <> struct makeDoubleWide<float>      { using type = double; };
}


// Floating-point widening operations may raise floating-point exceptions.
// In the case of a scalar operand, the widening operation should not
// raise any exceptions when all vector elements are masked off.  As a
// result, this class exists to perform the widening instruction just
// prior to first actual use (i.e. within the vector instruction's
// operation) and cache the result so that successive uses don't have to
// re-perform the widening.
template <typename ELEM_TYPE>
class WidenedFpScalar
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  public:
    constexpr WidenedFpScalar(ELEM_TYPE value): v_{value}, vDw_{fpConvertTo<ELEM_TYPE2X, true>(value)} {};

    constexpr operator ELEM_TYPE2X()
    {
      if (not vDw_.has_value())
        vDw_ = fpConvertTo<ELEM_TYPE2X, true>(v_);
      return *vDw_;
    }

  private:
    ELEM_TYPE v_;
    std::optional<ELEM_TYPE2X> vDw_;
};


static
float minfp(float a, float b)
{
  return std::fminf(a, b);
}


static
double minfp(double a, double b)
{
  return std::fmin(a, b);
}


static
Float16 minfp(Float16 a, Float16 b)
{
  return std::fmin(a, b);
}


static
BFloat16 minfp(BFloat16 a, BFloat16 b)
{
  return std::fmin(a, b);
}


template <typename FT>
static FT
doFmin(FT f1, FT f2)
{
  FT res{};

  bool isNan1 = std::isnan(f1), isNan2 = std::isnan(f2);
  if (isNan1 and isNan2)
    res = std::numeric_limits<FT>::quiet_NaN();
  else if (isNan1)
    res = f2;
  else if (isNan2)
    res = f1;
  else
    res = minfp(f1, f2);  // std::fminf or std::fmin

  if (isSnan(f1) or isSnan(f2))
    raiseSimulatorFpFlags(FpFlags::Invalid);
  else if (std::signbit(f1) != std::signbit(f2) and f1 == f2)
    res = std::copysign(res, -FT{});  // Make sure min(-0, +0) is -0.

  return res;
}


static
float maxfp(float a, float b)
{
  return std::fmaxf(a, b);
}


static
double maxfp(double a, double b)
{
  return std::fmax(a, b);
}


static
Float16 maxfp(Float16 a, Float16 b)
{
  return std::fmax(a, b);
}


static
BFloat16 maxfp(BFloat16 a, BFloat16 b)
{
  return std::fmax(a, b);
}


template <typename FT>
static FT
doFmax(FT f1, FT f2)
{
  FT res{};

  bool isNan1 = std::isnan(f1), isNan2 = std::isnan(f2);
  if (isNan1 and isNan2)
    res = std::numeric_limits<FT>::quiet_NaN();
  else if (isNan1)
    res = f2;
  else if (isNan2)
    res = f1;
  else
    res = maxfp(f1, f2);  // std::fmaxf or std::fmax

  if (isSnan(f1) or isSnan(f2))
    raiseSimulatorFpFlags(FpFlags::Invalid);
  else if (std::signbit(f1) != std::signbit(f2) and f1 == f2)
    res = std::copysign(res, FT{});  // Make sure max(-0, +0) is +0.

  return res;
}


static constexpr std::array<uint32_t, 128> frsqrt7Table = {
  52,  51,  50,  48,  47,  46,  44,  43,  42,  41,  40,  39,  38,  36,  35,  34,
  33,  32,  31,  30,  30,  29,  28,  27,  26,  25,  24,  23,  23,  22,  21,  20,
  19,  19,  18,  17,  16,  16,  15,  14,  14,  13,  12,  12,  11,  10,  10,  9,
  9,   8,   7,   7,   6,   6,   5,   4,   4,   3,   3,   2,   2,   1,   1,   0,
  127, 125, 123, 121, 119, 118, 116, 114, 113, 111, 109, 108, 106, 105, 103, 102,
  100, 99,  97,  96,  95,  93,  92,  91,  90,  88,  87,  86,  85,  84,  83,  82,
  80,  79,  78,  77,  76,  75,  74,  73,  72,  71,  70,  70,  69,  68,  67,  66,
  65,  64,  63,  63,  62,  61,  60,  59,  59,  58,  57,  56,  56,  55,  54,  53
};


// Approximate 1 / sqrt(val)
template <typename T>
static T
doFrsqrt7(T val, bool& divByZero, bool& invalid)
{
  static constexpr int bias            = std::numeric_limits<T>::max_exponent - 1;
  static constexpr int bitsOfPrecision = std::numeric_limits<T>::digits - 1;

  using UT = getSameWidthUintType_t<T>;

  divByZero = false;
  invalid = false;

  bool signBit = std::signbit(val);
  if (val == T{})
    {
      val = std::numeric_limits<T>::infinity();
      if (signBit)
	val = -val;
      divByZero = true;
    }
  else if (std::isinf(val) and not signBit)
    {
      val = T{};
    }
  else if (std::isnan(val))
    {
      if (isSnan(val))
	invalid = true;
      val = std::numeric_limits<T>::quiet_NaN();
    }
  else if (signBit)
    {
      val = std::numeric_limits<T>::quiet_NaN();
      invalid = true;
    }
  else
    {
      int inExp  = 0;
      T   inFrac = std::frexp(val, &inExp);
      inExp += bias - 1;
      auto u       = std::bit_cast<UT>(inFrac);
      auto sigMs6  = (u >> (bitsOfPrecision - 6)) & 0x3f;  // Most sig 6 bits of significand
      UT outExp    = (3 * bias - 1 - inExp) / 2;
      auto index   = (UT(inExp & 1) << 6) | sigMs6;
      UT outSigMs7 = frsqrt7Table.at(index);
      u            = (outSigMs7 << (bitsOfPrecision - 7)) | (outExp << bitsOfPrecision);
      val          = std::bit_cast<T>(u);
    }

  return val;
}


static constexpr std::array<uint32_t, 128> frec7Table = {
  127, 125, 123, 121, 119, 117, 116, 114, 112, 110, 109, 107, 105, 104, 102, 100, 
  99,  97,  96,  94,  93,  91,  90,  88,  87,  85,  84,  83,  81,  80,  79,  77,  
  76,  75,  74,  72,  71,  70,  69,  68,  66,  65,  64,  63,  62,  61,  60,  59,  
  58,  57,  56,  55,  54,  53,  52,  51,  50,  49,  48,  47,  46,  45,  44,  43,  
  42,  41,  40,  40,  39,  38,  37,  36,  35,  35,  34,  33,  32,  31,  31,  30,  
  29,  28,  28,  27,  26,  25,  25,  24,  23,  23,  22,  21,  21,  20,  19,  19,  
  18,  17,  17,  16,  15,  15,  14,  14,  13,  12,  12,  11,  11,  10,  9,   9,   
  8,   8,   7,   7,   6,   5,   5,   4,   4,   3,   3,   2,   2,   1,   1,   0
};

// Approximate 1 / x
template <typename T>
static T
doFrec7(T val, RoundingMode mode, FpFlags& flags)
{
  static constexpr int bias            = std::numeric_limits<T>::max_exponent - 1;
  static constexpr int bitsOfPrecision = std::numeric_limits<T>::digits - 1;

  using UT = getSameWidthUintType_t<T>;

  flags = FpFlags::None;
  bool signBit = std::signbit(val);

  if (val == T{})
    {
      val = std::numeric_limits<T>::infinity();
      if (signBit)
	val = -val;
      flags = FpFlags(unsigned(FpFlags::DivByZero) | unsigned(flags));
    }
  else if (std::isinf(val))
    {
      val = signBit? -T{} : T{};
    }
  else if (std::isnan(val))
    {
      if (isSnan(val))
	flags = FpFlags(unsigned(FpFlags::Invalid) | unsigned(flags));
      val = std::numeric_limits<T>::quiet_NaN();
    }
  else
    {
      int inExp  = 0;
      T   inFrac = std::frexp(val, &inExp);
      inExp += bias - 1;

      if (inExp < -1 or inExp > 2*bias)
	{
	  auto upDown = signBit? RoundingMode::Up : RoundingMode::Down;
	  if (mode == upDown or mode == RoundingMode::Zero)
	    {
	      val = std::numeric_limits<T>::max();
	      if (signBit)
		val = -val;
	      flags = FpFlags(unsigned(FpFlags::Inexact) | unsigned(FpFlags::Overflow) |
			      unsigned(flags));
	    }
	  else
	    {
	      val = std::numeric_limits<T>::infinity();
	      if (signBit)
		val = -val;
	      flags = FpFlags(unsigned(FpFlags::Inexact) | unsigned(FpFlags::Overflow) |
			      unsigned(flags));
	    }
	}
      else
        {
          auto u         = std::bit_cast<UT>(inFrac);
          int  sigMs7    = (u >> (bitsOfPrecision - 7)) & 0x7f;  // Most sig 7 bits of significand
          int  outExp    = ((2*bias) - 1 - inExp);
          auto outSigMs7 = UT(frec7Table.at(sigMs7)) << (bitsOfPrecision - 7);

          if (outExp < 1)
            {
              outSigMs7 = ((UT(1) << bitsOfPrecision) | outSigMs7) >> (1 - outExp);
              outExp    = 0;
            }

          u   = outSigMs7                       |
                (UT(outExp) << bitsOfPrecision) |
                (UT(signBit) << (std::numeric_limits<UT>::digits - 1));
          val = std::bit_cast<T>(u);
        }
    }

  return val;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecFpMaskInst(const DecodedInst* di, unsigned dest,
				   unsigned src, unsigned groupX8)
{
  if (not checkVecMaskInst(di, dest, src, groupX8))
    return false;

  bool ok = false;
  if (checkRoundingModeCommon(di))
    {
      using EW = ElementWidth;
      EW sew = vecRegs_.elemWidth();
      switch (sew)
        {
        case EW::Half:  ok = isHalfFpLegal(); break;
        case EW::Word:  ok = isFpLegal();     break;
        case EW::Word2: ok = isDpLegal();     break;
        default:        ok = false;           break;
        }
    }

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
inline
bool
Hart<URV>::checkVecFpMaskInst(const DecodedInst* di, unsigned dest,
				   unsigned src1, unsigned src2, unsigned groupX8)
{
  if (not checkVecMaskInst(di, dest, src1, src2, groupX8))
    return false;

  bool ok = false;
  if (checkRoundingModeCommon(di))
    {
      using EW = ElementWidth;
      EW sew = vecRegs_.elemWidth();
      switch (sew)
        {
        case EW::Half:  ok = isHalfFpLegal(); break;
        case EW::Word:  ok = isFpLegal();     break;
        case EW::Word2: ok = isDpLegal();     break;
        default:        ok = false;           break;
        }
    }

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
bool
Hart<URV>::checkFpSewLmulVstart(const DecodedInst* di, bool wide,
				 bool (Hart::*fp16LegalFn)() const)
{
  // Vector extension must be enabled, MSTATUS.VS must not be off, sew/lmul must
  // be legal, vtype.vill must not be set.
  if (not preVecExec() or not vecRegs_.legalConfig())
    {
      postVecFail(di);
      return false;
    }

  // Trap on use of non-zero vstart for arithmetic vector ops.
  URV vstart = csRegs_.peekVstart();
  if (trapNonZeroVstart_ and vstart > 0)
    {
      postVecFail(di);
      return false;
    }

  ElementWidth sew = vecRegs_.elemWidth();
  bool ok = false;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      switch (di->instId())
        {
        case InstId::vfwcvtbf16_f_f_v:
        case InstId::vfncvtbf16_f_f_w:
        case InstId::vfncvtbf16_sat_f_f_w:
          ok = this->isRvzvfofp8min();
          break;
        default:
          ok = false;
          break;
        }
      break;
    case EW::Half:
      if (vecRegs_.altfmt())
        switch (di->instId())
          {
          case InstId::vfdiv_vv:
          case InstId::vfdiv_vf:
          case InstId::vfsqrt_v:
          case InstId::vfredusum_vs:
          case InstId::vfredosum_vs:
          case InstId::vfredmin_vs:
          case InstId::vfredmax_vs:
          case InstId::vfwcvt_xu_f_v:
          case InstId::vfwcvt_x_f_v:
          case InstId::vfwcvt_rtz_xu_f_v:
          case InstId::vfwcvt_rtz_x_f_v:
          case InstId::vfwcvt_f_xu_v:
          case InstId::vfwcvt_f_x_v:
            ok = false;
            break;
          case InstId::vfwmaccbf16_vv:
          case InstId::vfwmaccbf16_vf:
            ok = isRvzvfbfa() and isZvfbfwmaLegal();
            break;
          case InstId::vfwcvtbf16_f_f_v:
          case InstId::vfncvtbf16_f_f_w:
            ok = isRvzvfbfa() and isZvfbfminLegal();
            break;
          default:
            ok = isRvzvfbfa();
            break;
          }
      else
        ok = (this->*fp16LegalFn)();
      break;
    case EW::Word:   ok = isFpLegal();            break;
    case EW::Word2:  ok = isDpLegal();            break;
    default:         ok = false;                  break;
    }

  if (ok and wide)
    {
      switch (sew)
	{
	case EW::Byte:
	case EW::Half:   ok = isFpLegal(); break;
	case EW::Word:   ok = isDpLegal();  break;
	default:         ok = false;        break;
	}
    }

  ok = ok and checkRoundingModeCommon(di);

  // Clear soft-float library or x86 exception flags
  clearSimulatorFpFlags();

  // Set soft-float library or x86 rounding mode
  setSimulatorRoundingMode(getFpRoundingMode());

  if (not ok)
    postVecFail(di);

  return ok;
}


template <typename URV>
bool
Hart<URV>::checkVecFpInst(const DecodedInst* di, bool wide,
                          bool (Hart::*fp16LegalFn)() const)
{
  if (not checkVecIntInst(di))
    return false;

  return checkFpSewLmulVstart(di, wide, fp16LegalFn);
}


template <typename URV>
void
Hart<URV>::execVfmv_f_s(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked() or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  unsigned rd = di->op0(), vs1 = di->op1(), groupX8 = 8;
  unsigned eg = groupX8 >= 8 ? groupX8 / 8 : 1;
  vecRegs_.setOpEmul(1, eg);  // Track operand group for logging

  ElementWidth sew = vecRegs_.elemWidth();

  bool ok = false;

  switch (sew)
    {
    case ElementWidth::Half:
      if (isHalfFpLegal())
        {
          Float16 val{};
          vecRegs_.read(vs1, 0, groupX8, val);
          fpRegs_.writeHalf(rd, val);
          markFsDirty();
          ok = true;
        }
      break;

    case ElementWidth::Word:
      if (isFpLegal())
        {
          float val{};
          vecRegs_.read(vs1, 0, groupX8, val);
          fpRegs_.writeSingle(rd, val);
          markFsDirty();
          ok = true;
        }
      break;

    case ElementWidth::Word2:
      if (isDpLegal())
        {
          double val{};
          vecRegs_.read(vs1, 0, groupX8, val);
          fpRegs_.writeDouble(rd, val);
          markFsDirty();
          ok = true;
        }
      break;

    case ElementWidth::Byte:
    default:
      break;
    }

  if (ok)
    postVecSuccess(di);
  else
    postVecFail(di);
}


template <typename URV>
void
Hart<URV>::execVfmv_s_f(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked() or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  unsigned vd = di->op0(), rs1 = di->op1(), groupX8 = 8, start = csRegs_.peekVstart();
  ElementWidth sew = vecRegs_.elemWidth();

  bool setTail = vecRegs_.isTailAgnostic() and vecRegs_.isTailAgnosticOnes();
  unsigned tail = vecRegs_.vlmax(sew, GroupMultiplier::One);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (not isHalfFpLegal())
        {
          postVecFail(di);
          return;
        }
      if (start < vecRegs_.elemCount())
	{
          uint16_t u16 = 0;
          if (vecRegs_.altfmt())
            u16 = std::bit_cast<uint16_t>(fpRegs_.readBFloat16(rs1));
          else
            u16 = std::bit_cast<uint16_t>(fpRegs_.readHalf(rs1));
          vecRegs_.write(vd, 0, groupX8, u16);

	  if (setTail)
	    for (unsigned i = 1; i < tail; ++i)
	      vecRegs_.write(vd, i, groupX8, uint16_t(~0));
	}
      break;
    case EW::Word:
      if (not isFpLegal())
        {
          postVecFail(di);
          return;
        }
      if (start < vecRegs_.elemCount())
	{
	  float val = fpRegs_.readSingle(rs1);
	  vecRegs_.write(vd, 0, groupX8, val);
	  if (setTail)
	    for (unsigned i = 1; i < tail; ++i)
	      vecRegs_.write(vd, i, groupX8, uint32_t(~0));
	}
      break;
    case EW::Word2:
      if (not isDpLegal())
        {
          postVecFail(di);
          return;
        }
      if (start < vecRegs_.elemCount())
	{
	  double val = fpRegs_.readDouble(rs1);
	  vecRegs_.write(vd, 0, groupX8, val);
	  if (setTail)
	    for (unsigned i = 1; i < tail; ++i)
	      vecRegs_.write(vd, i, groupX8, uint64_t(~uint64_t(0)));
	}
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfop_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		   unsigned start, unsigned elems, bool masked,
		   std::function<ELEM_TYPE(ELEM_TYPE, ELEM_TYPE)> fop)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = fop(e1, e2);
          URV incFlags = activeSimulatorFpFlags(); // Incremental RISCV FP flags generated by current instruction.
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfop_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked, doFadd<BFloat16>);
      else
        vfop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFadd<Float16>);
      break;
    case EW::Word:
      vfop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFadd<float>);
      break;
    case EW::Word2:
      vfop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFadd<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfadd_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFadd(e1, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfadd_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfadd_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfadd_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfadd_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfop_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked, doFsub<BFloat16>);
      else
        vfop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFsub<Float16>);
      break;
    case EW::Word:
      vfop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFsub<float>);
      break;
    case EW::Word2:
      vfop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFsub<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsub_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          ELEM_TYPE negE2 = doNegate(e2);

	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFadd(e1, negE2);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfsub_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfsub_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfsub_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfsub_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrsub_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
          e1 = doNegate(e1);
	  dest = doFadd(e2, e1);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfrsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfrsub_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfrsub_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfrsub_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfrsub_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFadd(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true /*widen*/))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwadd_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwadd_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwadd_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    case EW::Word2:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  ELEM_TYPE2X e1dw{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must widen here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          WidenedFpScalar e2dw{e2};
	  vecRegs_.read(vs1, ix, group, e1);
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  dest = doFadd<ELEM_TYPE2X>(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwadd_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfwadd_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwadd_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          e2dw = doNegate(e2dw);
	  dest = doFadd(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwsub_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwsub_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwsub_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    case EW::Word2:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  ELEM_TYPE2X e1dw{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE2 = doNegate(e2);
          WidenedFpScalar negE2dw{negE2};

	  vecRegs_.read(vs1, ix, group, e1);
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  dest = doFadd<ELEM_TYPE2X>(e1dw, negE2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwsub_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfwsub_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwsub_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1dw);
	  vecRegs_.read(vs2, ix, group, e2);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFadd(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwadd_wv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}, {vs2, false}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwadd_wv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwadd_wv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwadd_wv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    case EW::Word2:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwadd_wf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  ELEM_TYPE2X e1dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must widen here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          WidenedFpScalar e2dw{e2};

	  vecRegs_.read(vs1, ix, group2x, e1dw);
	  dest = doFadd<ELEM_TYPE2X>(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwadd_wf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwadd_wf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfwadd_wf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwadd_wf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_wv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE  e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1dw);
	  vecRegs_.read(vs2, ix, group, e2);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          e2dw = doNegate(e2dw);
	  dest = doFadd(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwsub_wv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}, {vs2, false}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwsub_wv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwsub_wv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwsub_wv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwsub_wf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  ELEM_TYPE2X e1dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE2 = doNegate(e2);
          WidenedFpScalar negE2dw{negE2};

	  vecRegs_.read(vs1, ix, group2x, e1dw);
	  dest = doFadd<ELEM_TYPE2X>(e1dw, negE2dw);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwsub_wf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, true}}))
    return;

  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax(dsew);

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwsub_wf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfwsub_wf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwsub_wf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfmul_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfop_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked, doFmul<BFloat16>);
      else
        vfop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFmul<Float16>);
      break;
    case EW::Word:
      vfop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFmul<float>);
      break;
    case EW::Word2:
      vfop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFmul<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmul_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFmul(e1, e2);
          URV incFlags = activeSimulatorFpFlags(); 
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfmul_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmul_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfmul_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmul_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmul_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfdiv_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfop_vv<Float16>(vd, vs1, vs2, group, start, elems, masked, doFdiv<Float16>);
      break;
    case EW::Word:
      vfop_vv<float>  (vd, vs1, vs2, group, start, elems, masked, doFdiv<float>);
      break;
    case EW::Word2:
      vfop_vv<double> (vd, vs1, vs2, group, start, elems, masked, doFdiv<double>);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfdiv_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFdiv(e1, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfdiv_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfdiv_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfdiv_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfdiv_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrdiv_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFdiv(e2, e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfrdiv_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfrdiv_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfrdiv_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfrdiv_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmul_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = doFmul(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwmul_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  // Double wide legal. Destination register multiple of emul.
  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwmul_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwmul_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwmul_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmul_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);
  ELEM_TYPE2X e1dw{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must widen here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          WidenedFpScalar e2dw{e2};

	  vecRegs_.read(vs1, ix, group, e1);
	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  dest = doFmul<ELEM_TYPE2X>(e1dw, e2dw);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfwmul_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  // Double wide legal. Destination register multiple of emul.
  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwmul_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfwmul_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwmul_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = fusedMultiplyAdd(e1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmadd_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfmadd_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmadd_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmadd_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmadd_vf(unsigned vd, unsigned f1, unsigned vf2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vf2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = fusedMultiplyAdd(e1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check f1, we pass vd instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vd, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmadd_vf<BFloat16>(vd, f1, vs2, group, start, elems, masked);
      else
        vfmadd_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmadd_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmadd_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmadd_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          e1 = doNegate(e1);
          e2 = doNegate(e2);
	  dest = fusedMultiplyAdd(e1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmadd_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmadd_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfnmadd_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmadd_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmadd_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmadd_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE1 = doNegate(e1);

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          e2 = doNegate(e2);
	  dest = fusedMultiplyAdd(negE1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmadd_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vd, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmadd_vf<BFloat16>(vd, f1, vs2, group, start, elems, masked);
      else
        vfnmadd_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmadd_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmadd_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          e2 = doNegate(e2);
	  dest = fusedMultiplyAdd(e1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmsub_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfmsub_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmsub_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmsub_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsub_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          e2 = doNegate(e2);
	  dest = fusedMultiplyAdd(e1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check f1, we pass vd instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vd, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmsub_vf<BFloat16>(vd, f1, vs2, group, start, elems, masked);
      else
        vfmsub_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmsub_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmsub_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsub_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          e1 = doNegate(e1);
	  dest = fusedMultiplyAdd(e1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmsub_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmsub_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfnmsub_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmsub_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmsub_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsub_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE1 = doNegate(e1);

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = fusedMultiplyAdd(negE1, dest, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmsub_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check f1, we pass vd instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vd, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmsub_vf<BFloat16>(vd, f1, vs2, group, start, elems, masked);
      else
        vfnmsub_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmsub_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmsub_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = fusedMultiplyAdd(e1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmacc_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmacc_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmacc_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = fusedMultiplyAdd(e1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check f1, we pass vd instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vd, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmacc_vf<BFloat16>(vd, f1, vs2, group, start, elems, masked);
      else
        vfmacc_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;

    case EW::Word:
      vfmacc_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;

    case EW::Word2:
      vfmacc_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;

    default: postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          e1 = doNegate(e1);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd(e1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmacc_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfnmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmacc_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmacc_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE1 = doNegate(e1);

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd(negE1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  v2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check f1, we pass vd instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vd, v2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmacc_vf<BFloat16>(vd, f1, v2, group, start, elems, masked);
      else
        vfnmacc_vf<Float16>(vd, f1, v2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmacc_vf<float>  (vd, f1, v2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmacc_vf<double> (vd, f1, v2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd(e1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmsac_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmsac_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmsac_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd(e1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check f1, we pass vd instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vd, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmsac_vf<BFloat16>(vd, f1, vs2, group, start, elems, masked);
      else
        vfmsac_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmsac_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmsac_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE(), dest = ELEM_TYPE();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
          e1 = doNegate(e1);
	  dest = fusedMultiplyAdd(e1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmsac_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfnmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmsac_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfnmsac_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{}, dest{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE1 = doNegate(e1);

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group, dest);
	  dest = fusedMultiplyAdd(negE1, e2, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfnmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  f1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  // Don't want to check f1, we pass vd instead. Should cause no harm.
  if (not checkVecOpsVsEmul(di, group, {vd, vd, vs2}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfnmsac_vf<BFloat16>(vd, f1, vs2, group, start, elems, masked);
      else
        vfnmsac_vf<Float16>(vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfnmsac_vf<float>  (vd, f1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfnmsac_vf<double> (vd, f1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);

	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = fusedMultiplyAdd(e1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwmacc_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmacc_vf(unsigned vd, unsigned f1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e2{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(f1);

  ELEM_TYPE2X e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must widen here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          WidenedFpScalar e1dw{e1};

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);

	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = fusedMultiplyAdd<ELEM_TYPE2X>(e1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwmacc_vf<BFloat16>(vd, fs1, vs2, group, start, elems, masked);
      else
        vfwmacc_vf<Float16>(vd, fs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwmacc_vf<float>  (vd, fs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmacc_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);

	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          e1dw = doNegate(e1dw);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd(e1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwnmacc_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwnmacc_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwnmacc_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwnmacc_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmacc_vf(unsigned vd, unsigned fs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e2{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(fs1);

  ELEM_TYPE2X e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE1 = doNegate(e1);
          WidenedFpScalar e1dw{negE1};

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd<ELEM_TYPE2X>(e1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwnmacc_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwnmacc_vf<BFloat16>(vd, fs1, vs2, group, start, elems, masked);
      else
        vfwnmacc_vf<Float16>(vd, fs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwnmacc_vf<float>  (vd, fs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);

	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd(e1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwmsac_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}

template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwmsac_vf(unsigned vd, unsigned fs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e2{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(fs1);

  ELEM_TYPE2X e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must widen here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          WidenedFpScalar e1dw{e1};

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          dest = doNegate(dest);
	  dest = fusedMultiplyAdd<ELEM_TYPE2X>(e1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwmsac_vf<BFloat16>(vd, fs1, vs2, group, start, elems, masked);
      else
        vfwmsac_vf<Float16>(vd, fs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwmsac_vf<float>  (vd, fs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmsac_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();
  ELEM_TYPE2X e1dw{}, e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);

	  e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
          e1dw = doNegate(e1dw);
	  dest = fusedMultiplyAdd(e1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwnmsac_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwnmsac_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfwnmsac_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwnmsac_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfwnmsac_vf(unsigned vd, unsigned fs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>; // Double wide

  ELEM_TYPE e2{};
  auto e1 = fpRegs_.read<ELEM_TYPE>(fs1);

  ELEM_TYPE2X e2dw{}, dest{};

  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group2x);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
          // We must negate here and not outside the loop; otherwise, we may set INVALID
          // fflag when it should not be set.
          auto negE1 = doNegate(e1);
          WidenedFpScalar negE1dw{negE1};

	  vecRegs_.read(vs2, ix, group, e2);
	  vecRegs_.read(vd, ix, group2x, dest);
	  e2dw = fpConvertTo<ELEM_TYPE2X, true>(e2);
	  dest = fusedMultiplyAdd<ELEM_TYPE2X>(negE1dw, e2dw, dest);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwnmsac_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwnmsac_vf<BFloat16>(vd, fs1, vs2, group, start, elems, masked);
      else
        vfwnmsac_vf<Float16>(vd, fs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfwnmsac_vf<float>  (vd, fs1, vs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsqrt_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFsqrt(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfsqrt_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfsqrt_v<Float16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vfsqrt_v<float>  (vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vfsqrt_v<double> (vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmerge(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		   unsigned start, unsigned elems)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = vecRegs_.isActive(0, ix) ? e2 : e1;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfmerge_vfm(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();

  // Must be masked, dest must not overlap v0. Source must not overlap v0.
  if (not di->isMasked() or vd == 0 or vs1 == 0 or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      if (vecRegs_.altfmt())
        vfmerge<BFloat16>(vd, vs1, rs2, group, start, elems);
      else
        vfmerge<Float16>(vd, vs1, rs2, group, start, elems);
      break;

    case EW::Word:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfmerge<float>  (vd, vs1, rs2, group, start, elems);
      break;

    case EW::Word2:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfmerge<double> (vd, vs1, rs2, group, start, elems);
      break;

    default: postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmv_v_f(unsigned vd, unsigned rs1, unsigned group,
		    unsigned start, unsigned elems)
{
  ELEM_TYPE e1 = fpRegs_.read<ELEM_TYPE>(rs1), dest;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, false /*masked*/, dest))
	dest = e1;
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfmv_v_f(const DecodedInst* di)
{
  if (not checkSewLmulVstart(di))
    return;

  if (di->isMasked() or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  rs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      if (vecRegs_.altfmt())
        vfmv_v_f<BFloat16>(vd, rs1, group, start, elems);
      else
        vfmv_v_f<Float16>(vd, rs1, group, start, elems);
      break;

    case EW::Word:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfmv_v_f<float>  (vd, rs1, group, start, elems);
      break;

    case EW::Word2:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfmv_v_f<double> (vd, rs1, group, start, elems);
      break;

    default: postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfeq_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;
      if (vecRegs_.isMaskDestActive(vd, ix,  masked, flag))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 == e2;
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfeq_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfeq_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vmfeq_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfeq_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfeq_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfeq_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  flag = false;
	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 == e2;
	}

      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfeq_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfeq_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vmfeq_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfeq_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfeq_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfne_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = true;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 != e2;
	}

      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfne_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfne_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vmfne_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfne_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfne_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfne_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = true;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = true;
	  vecRegs_.read(vs1, ix, group, e1);
	  if (std::isnan(e1) or std::isnan(e2))
	    {
	      if (isSnan(e1) or isSnan(e2))
		orFcsrFlags(FpFlags::Invalid);
	    }
	  else
	    flag = e1 != e2;
	}

      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfne_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfne_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vmfne_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfne_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfne_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmflt_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = false;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 < e2;
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmflt_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmflt_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vmflt_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmflt_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmflt_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmflt_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = false;
	  vecRegs_.read(vs1, ix, group, e1);
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 < e2;
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }
  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmflt_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmflt_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vmflt_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmflt_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmflt_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfle_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1 = ELEM_TYPE(), e2 = ELEM_TYPE();

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = false;
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 <= e2;
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfle_vv(const DecodedInst* di)
{
  unsigned group = vecRegs_.groupMultiplierX8();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();

  if (not checkVecFpMaskInst(di, vd, vs1, vs2, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfle_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vmfle_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfle_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfle_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfle_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = false;
	  vecRegs_.read(vs1, ix, group, e1);
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 <= e2;
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfle_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfle_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vmfle_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfle_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfle_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfgt_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = false;
	  vecRegs_.read(vs1, ix, group, e1);
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 > e2;
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfgt_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfgt_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vmfgt_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfgt_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfgt_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vmfge_vf(unsigned vd, unsigned vs1, unsigned rs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(rs2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      bool flag = false;

      if (vecRegs_.isMaskDestActive(vd, ix, masked, flag))
	{
	  flag = false;
	  vecRegs_.read(vs1, ix, group, e1);
	  if (std::isnan(e1) or std::isnan(e2))
	    orFcsrFlags(FpFlags::Invalid);
	  else
	    flag = e1 >= e2;
	}
      vecRegs_.writeMaskRegister(vd, ix, flag);
    }

  vecRegs_.finishMaskDest(vd, elems);
}


template <typename URV>
void
Hart<URV>::execVmfge_vf(const DecodedInst* di)
{
  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.updateWholeMask()? vecRegs_.elemMax() : vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecFpMaskInst(di, vd, vs1, group))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vmfge_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vmfge_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vmfge_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vmfge_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfclass_v(unsigned vd, unsigned vs1, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      using INT_TYPE = getSameWidthIntType_t<ELEM_TYPE>;
      INT_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpClassifyRiscv(e1);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfclass_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfclass_v<BFloat16>(vd, vs1, group, start, elems, masked);
      else
        vfclass_v<Float16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vfclass_v<float>  (vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vfclass_v<double> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_xu_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      using UINT_TYPE = getSameWidthUintType_t<ELEM_TYPE>;
      UINT_TYPE dest{};

      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<UINT_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfcvt_xu_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:   vfcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_xu_f_v<double> (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:   // Fall-through to invalid case
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_x_f_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      using INT_TYPE = getSameWidthIntType_t<ELEM_TYPE>;
      INT_TYPE dest{};

      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<INT_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfcvt_x_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:   vfcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_x_f_v<double> (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:   // Fall-through to invalid case
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfcvt_rtz_xu_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:   vfcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_xu_f_v<double> (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:   // Fall-through to invalid case
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfcvt_rtz_x_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:   vfcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_x_f_v<double> (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:   // Fall-through to invalid case
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_f_xu_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  using UINT_TYPE = getSameWidthUintType_t<ELEM_TYPE>;

  UINT_TYPE e1{0};
  ELEM_TYPE dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<ELEM_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfcvt_f_xu_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:   vfcvt_f_xu_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_f_xu_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_f_xu_v<double> (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:   // Fall-through to invalid case
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfcvt_f_x_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  using INT_TYPE = getSameWidthIntType_t<ELEM_TYPE>;

  INT_TYPE e1{0};
  ELEM_TYPE dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<ELEM_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfcvt_f_x_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:   vfcvt_f_x_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:   vfcvt_f_x_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Word2:  vfcvt_f_x_v<double> (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:   // Fall-through to invalid case
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_xu_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      using UINT_TYPE   = getSameWidthUintType_t<ELEM_TYPE>;
      using UINT_TYPE2X = makeDoubleWide_t<UINT_TYPE>;
      UINT_TYPE2X dest{};

      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<UINT_TYPE2X>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwcvt_xu_f_v(const DecodedInst* di)
{
  // Float to double-wide integer.
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:  vfwcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfwcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_x_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      using INT_TYPE   = getSameWidthIntType_t<ELEM_TYPE>;
      using INT_TYPE2X = makeDoubleWide_t<INT_TYPE>;
      INT_TYPE2X dest{};

      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<INT_TYPE2X>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwcvt_x_f_v(const DecodedInst* di)
{
  // Float to double-wide integer
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:  vfwcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfwcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfwcvt_rtz_xu_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:  vfwcvt_xu_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfwcvt_xu_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfwcvt_rtz_x_f_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:  vfwcvt_x_f_v<Float16>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfwcvt_x_f_v<float>  (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_f_xu_v(unsigned vd, unsigned vs1, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;
  using FP_TYPE2X   = getSameWidthFloatType_t<ELEM_TYPE2X>;

  if constexpr (std::is_same_v<ELEM_TYPE, uint8_t> or std::is_same_v<ELEM_TYPE, int8_t>)
    if (vecRegs_.altfmt())
      {
        ELEM_TYPE e1{};
        BFloat16 dest{};

        unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

        if (start >= vecRegs_.elemCount())
          return;

        for (unsigned ix = start; ix < elems; ++ix)
          {
            if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
              {
                vecRegs_.read(vs1, ix, group, e1);
                float fp32 = fpConvertTo<float>(e1);
                dest = fpConvertTo<BFloat16, true>(fp32);
                URV incFlags = activeSimulatorFpFlags();
                vecRegs_.fpFlags_.push_back(incFlags);
              }
            else
              vecRegs_.fpFlags_.push_back(0);
            vecRegs_.write(vd, ix, destGroup, dest);
          }

        updateAccruedFpBits();
        return;
      }

  ELEM_TYPE e1{};
  FP_TYPE2X dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<FP_TYPE2X>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwcvt_f_xu_v(const DecodedInst* di)
{
  // Unsigned to double-wide fp.
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group) or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      vfwcvt_f_xu_v<uint8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfwcvt_f_xu_v<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfwcvt_f_xu_v<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_f_x_v(unsigned vd, unsigned vs1, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;
  using FP_TYPE2X   = getSameWidthFloatType_t<ELEM_TYPE2X>;

  ELEM_TYPE e1{};
  FP_TYPE2X dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<FP_TYPE2X>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwcvt_f_x_v(const DecodedInst* di)
{
  // signed to double-wide fp.
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group) or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      vfwcvt_f_x_v<int8_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfwcvt_f_x_v<int16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfwcvt_f_x_v<int32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwcvt_f_f_v(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE e1{};
  ELEM_TYPE2X dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = fpConvertTo<ELEM_TYPE2X, false>(e1);
	  if (isSnan(dest))
	    {
	      dest = std::numeric_limits<ELEM_TYPE2X>::quiet_NaN();
	      raiseSimulatorFpFlags(FpFlags::Invalid);
	    }
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwcvt_f_f_v(const DecodedInst* di)
{
  // Float to double-wide float.
  if (not checkVecFpInst(di, true, &Hart::isZvfhminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfwcvt_f_f_v<BFloat16>(vd, vs1, group, start, elems, masked);
      else
        vfwcvt_f_f_v<Float16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vfwcvt_f_f_v<float>  (vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:  // Fall-through
    case EW::Byte:   // Fall-through
    default:       postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_xu_f_w(unsigned vd, unsigned vs1, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X  = makeDoubleWide_t<ELEM_TYPE>;
  using FLOAT_TYPE2X = getSameWidthFloatType_t<ELEM_TYPE2X>;

  if constexpr (std::is_same_v<ELEM_TYPE, uint8_t> or std::is_same_v<ELEM_TYPE, int8_t>)
    if (vecRegs_.altfmt())
      {
        BFloat16 e1{};
        unsigned group2x = group*2;
        unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

        if (start >= vecRegs_.elemCount())
          return;

        for (unsigned ix = start; ix < elems; ++ix)
          {
            ELEM_TYPE dest{};
            if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
              {
                vecRegs_.read(vs1, ix, group2x, e1);
                float fp32 = fpConvertTo<float, false>(e1);
                dest = fpConvertTo<ELEM_TYPE>(fp32);
                URV incFlags = activeSimulatorFpFlags();
                vecRegs_.fpFlags_.push_back(incFlags);
              }
            else
              vecRegs_.fpFlags_.push_back(0);
            vecRegs_.write(vd, ix, destGroup, dest);
          }

        updateAccruedFpBits();
        return;
      }

  FLOAT_TYPE2X e1{};
  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  dest = fpConvertTo<ELEM_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfncvt_xu_f_w(const DecodedInst* di)
{
  // Double-wide float to unsigned
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group) or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:       postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_x_f_w(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X  = makeDoubleWide_t<ELEM_TYPE>;
  using FLOAT_TYPE2X = getSameWidthFloatType_t<ELEM_TYPE2X>;

  FLOAT_TYPE2X e1{};
  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      ELEM_TYPE dest{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  dest = fpConvertTo<ELEM_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfncvt_x_f_w(const DecodedInst* di)
{
  // Double-wide float to int.
  if (not checkVecIntInst(di))
    return;

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(getFpRoundingMode());

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, group) or not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkVecOpsVsEmul(di, group, {{vd, false}, {vs1, true}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfncvt_rtz_xu_f_w(const DecodedInst* di)
{
  // Double-wide float to unsigned
  if (not checkVecIntInst(di))
    return;

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
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
    case EW::Byte:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_xu_f_w<uint32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfncvt_rtz_x_f_w(const DecodedInst* di)
{
  // double-wide float to int
  if (not checkVecIntInst(di))
    return;

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  clearSimulatorFpFlags();
  setSimulatorRoundingMode(RoundingMode::Zero);

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
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
    case EW::Byte:
      if (not isHalfFpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int8_t> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Half:
      if (not isFpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int16_t>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      if (not isDpLegal()) { postVecFail(di); return; }
      vfncvt_x_f_w<int32_t>(vd, vs1, group, start, elems, masked);
      break;
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_f_xu_w(unsigned vd, unsigned vs1, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  using FLOAT_TYPE  = getSameWidthFloatType_t<ELEM_TYPE>;
  using UINT_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  UINT_TYPE2X e1{0};
  FLOAT_TYPE dest{};
  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  dest = fpConvertTo<FLOAT_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfncvt_f_xu_w(const DecodedInst* di)
{
  // Double-wide unsigned to float
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
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
    case EW::Half:  vfncvt_f_xu_w<uint16_t>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfncvt_f_xu_w<uint32_t>(vd, vs1, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_f_x_w(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  using FLOAT_TYPE = getSameWidthFloatType_t<ELEM_TYPE>;
  using INT_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  INT_TYPE2X e1{0};
  FLOAT_TYPE dest{};
  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  dest = fpConvertTo<FLOAT_TYPE>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfncvt_f_x_w(const DecodedInst* di)
{
  // Double-wide int to float
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
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
    case EW::Half:  vfncvt_f_x_w<int16_t>(vd, vs1, group, start, elems, masked); break;
    case EW::Word:  vfncvt_f_x_w<int32_t> (vd, vs1, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:        postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfncvt_f_f_w(unsigned vd, unsigned vs1, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE2X e1{};
  ELEM_TYPE dest{};
  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  dest = fpConvertTo<ELEM_TYPE, false>(e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}



// Convert finite non-zero float32 bits to OFP8 E4M3 (layout: 1-4-3). Uses the host
// rounding mode (set by setSimulatorRoundingMode / checkVecFpInst) via fenv.
static uint8_t
floatToOfp8E4m3(uint32_t ui32, RoundingMode rm, bool& inf)
{
  constexpr unsigned NUM_SIGNIFICAND_BITS = 4;
  constexpr unsigned EXP_MASK           = 15;
  constexpr unsigned MAX_EXPONENT       = 8;
  constexpr unsigned FLOAT_THIS_EXP_DIFF =
    unsigned(std::numeric_limits<float>::max_exponent) - MAX_EXPONENT;
  constexpr unsigned FLOAT_THIS_SIG_DIFF =
    unsigned(std::numeric_limits<float>::digits) - NUM_SIGNIFICAND_BITS;

  inf = false;    // FIX : set inf if nan produced because inf cannot be represented.

  bool     sign = ui32 >> 31;
  int      exp  = static_cast<int>((ui32 >> 23) & 0xFF);
  uint32_t sig  = ui32 & 0x007FFFFF;

  assert(exp != 0xFF);

  if (exp == 0 and sig == 0)
    return static_cast<uint8_t>(sign) << 7;

  sig |= static_cast<uint32_t>(exp > 0 or FLOAT_THIS_EXP_DIFF != 0)
         << (std::numeric_limits<float>::digits - 1);
  exp -= static_cast<int>(FLOAT_THIS_EXP_DIFF + 1);

  using RM = RoundingMode;

  auto roundNearest = false;
  uint32_t roundIncrement = 0;

  if (rm == RM::Down or rm == RM::Up or rm == RM::Zero)
    {
      if ((rm != RM::Zero) and (sign == (rm == RM::Down)))
        roundIncrement = (1U << FLOAT_THIS_SIG_DIFF) - 1;
      roundNearest = false;
    }
  else
    {
      roundIncrement = (1U << (FLOAT_THIS_SIG_DIFF - 1));
      roundNearest = (rm == RM::NearestEven);
    }
  uint32_t roundBits = sig & ((1U << FLOAT_THIS_SIG_DIFF) - 1);

  if (exp < 0)
    {
      bool isTiny =
        (exp < -1)
        or (sig + roundIncrement < (1U << (std::numeric_limits<float>::digits + 1)));
      if (sig & (1U << (std::numeric_limits<float>::digits - 1)))
        sig = (exp >= -31) ? (sig >> -exp) | ((sig << (exp & 31)) != 0) : (sig != 0);
      exp = 0;
      roundBits = sig & ((1U << FLOAT_THIS_SIG_DIFF) - 1);
      if (isTiny and roundBits)
        raiseSimulatorFpFlags(FpFlags::Underflow);
    }
  else if (exp > static_cast<int>(EXP_MASK - 2)
           or (exp == static_cast<int>(EXP_MASK - 2)
               and (sig + roundIncrement
                    >= (1U << std::numeric_limits<float>::digits))))
    {
      raiseSimulatorFpFlags(FpFlags::Overflow); raiseSimulatorFpFlags(FpFlags::Inexact);
      unsigned maxSig = (1U << (NUM_SIGNIFICAND_BITS - 1)) - 1;
      return static_cast<uint8_t>((sign << 7) | ((EXP_MASK - 1U) << 3) | maxSig);
    }

  sig = (sig + roundIncrement) >> FLOAT_THIS_SIG_DIFF;
  if (roundBits)
    raiseSimulatorFpFlags(FpFlags::Inexact);
  sig &= ~static_cast<uint32_t>(not(roundBits ^ (1U << (FLOAT_THIS_SIG_DIFF - 1)))
                                and roundNearest);
  if (not sig)
    exp = 0;

  if (exp > static_cast<int>(EXP_MASK - 1))
    {
      raiseSimulatorFpFlags(FpFlags::Overflow); raiseSimulatorFpFlags(FpFlags::Inexact);
      unsigned maxSig = (1U << (NUM_SIGNIFICAND_BITS - 1)) - 1;
      return static_cast<uint8_t>((sign << 7) | ((EXP_MASK - 1U) << 3) | maxSig);
    }

  return static_cast<uint8_t>((static_cast<uint8_t>(sign) << 7)
                              | ((static_cast<uint8_t>(exp) << 3)
                                 + static_cast<uint8_t>(sig)));
}


// Convert finite non-zero float32 bits to OFP8 E5M2 (layout: 1-5-2).
static uint8_t
floatToOfp8E5m2(uint32_t ui32, RoundingMode rm)
{
  constexpr unsigned NUM_SIGNIFICAND_BITS = 3;
  constexpr unsigned EXP_MASK             = 31;
  constexpr unsigned MAX_EXPONENT         = 16;
  constexpr unsigned FLOAT_THIS_EXP_DIFF =
    unsigned(std::numeric_limits<float>::max_exponent) - MAX_EXPONENT;
  constexpr unsigned FLOAT_THIS_SIG_DIFF =
    unsigned(std::numeric_limits<float>::digits) - NUM_SIGNIFICAND_BITS;

  bool     sign = ui32 >> 31;
  int      exp  = static_cast<int>((ui32 >> 23) & 0xFF);
  uint32_t sig  = ui32 & 0x007FFFFF;

  assert(exp != 0xFF);

  if (exp == 0 and sig == 0)
    return static_cast<uint8_t>(sign) << 7;

  sig |= static_cast<uint32_t>(exp > 0 or FLOAT_THIS_EXP_DIFF != 0)
         << (std::numeric_limits<float>::digits - 1);
  exp -= static_cast<int>(FLOAT_THIS_EXP_DIFF + 1);

  using RM = RoundingMode;

  auto roundNearest = false;
  uint32_t roundIncrement = 0;

  if (rm == RM::Down or rm == RM::Up or rm == RM::Zero)
    {
      if ((rm != RM::Zero) and (sign == (rm == RM::Down)))
        roundIncrement = (1U << FLOAT_THIS_SIG_DIFF) - 1;
      roundNearest = false;
    }
  else
    {
      roundIncrement = (1U << (FLOAT_THIS_SIG_DIFF - 1));
      roundNearest = (rm == RM::NearestEven);
    }
  uint32_t roundBits = sig & ((1U << FLOAT_THIS_SIG_DIFF) - 1);

  if (exp < 0)
    {
      bool isTiny =
        (exp < -1)
        or (sig + roundIncrement < (1U << (std::numeric_limits<float>::digits + 1)));
      if (sig & (1U << (std::numeric_limits<float>::digits - 1)))
        sig = (exp >= -31) ? (sig >> -exp) | ((sig << (exp & 31)) != 0) : (sig != 0);
      exp = 0;
      roundBits = sig & ((1U << FLOAT_THIS_SIG_DIFF) - 1);
      if (isTiny and roundBits)
        raiseSimulatorFpFlags(FpFlags::Underflow);
    }
  else if (exp > static_cast<int>(EXP_MASK - 2)
           or (exp == static_cast<int>(EXP_MASK - 2)
               and (sig + roundIncrement
                    >= (1U << std::numeric_limits<float>::digits))))
    {
      raiseSimulatorFpFlags(FpFlags::Overflow); raiseSimulatorFpFlags(FpFlags::Inexact);
      return static_cast<uint8_t>((sign << 7) | (EXP_MASK << 2));
    }

  sig = (sig + roundIncrement) >> FLOAT_THIS_SIG_DIFF;
  if (roundBits)
    raiseSimulatorFpFlags(FpFlags::Inexact);
  sig &= ~static_cast<uint32_t>(not(roundBits ^ (1U << (FLOAT_THIS_SIG_DIFF - 1)))
                                and roundNearest);
  if (not sig)
    exp = 0;

  if (exp > static_cast<int>(EXP_MASK - 1))
    {
      raiseSimulatorFpFlags(FpFlags::Overflow); raiseSimulatorFpFlags(FpFlags::Inexact);
      return static_cast<uint8_t>((sign << 7) | (EXP_MASK << 2));
    }

  return static_cast<uint8_t>((static_cast<uint8_t>(sign) << 7)
                              | ((static_cast<uint8_t>(exp) << 2)
                                 + static_cast<uint8_t>(sig)));
}


// Given the bits of a bfloat16_t, convert it to ofp8 returing the bits of the result.
static uint8_t
bfloat16ToOfp8(uint16_t x, bool e4m3, RoundingMode rm, bool saturate)
{
  auto bf16 = std::bit_cast<BFloat16>(x);

  constexpr uint8_t nan8 = 0x7f;  // Canonical NAN for e4m3 and e5m2.

  if (std::isnan(bf16))
    {
      if (isSnan(bf16))
        raiseSimulatorFpFlags(FpFlags::Invalid);
      return nan8;
    }

  unsigned neg = std::signbit(bf16);

  if (e4m3)
    {
      if (std::isinf(bf16))
        {
          if (saturate)
            return uint8_t(neg << 7) | 0b1111'110;
          return nan8;
        }
    }
  else if (std::isinf(bf16))
    {
      if (saturate)
        return uint8_t(neg << 7) | 0b11110'11;
      return uint8_t(neg << 7) | 0b11111'00;
    }

  const auto val = static_cast<float>(bf16);
  if (val == 0.f)
    return static_cast<uint8_t>(neg << 7);

  const auto ui32 = std::bit_cast<uint32_t>(val);

  if (e4m3)
    {
      bool inf = false;
      auto res = floatToOfp8E4m3(ui32, rm, inf);
      if (saturate and inf)
        res = uint8_t(neg << 7) | uint8_t(0b1110'000);
      return res;
    }

  auto res = floatToOfp8E5m2(ui32, rm);
  if (saturate)
    {
      if (res == 0b0'11111'00)      // +inf
        res = 0b0'11110'11;         // Max val
      else if (res == 0b1'11110'00) // -inf
        res = 0b1'11110'11;         // Neg max val
    }
  return res;
}


template <typename URV>
void
Hart<URV>::vfncvtBfloat16ToOfp8(unsigned vd, unsigned vs1, unsigned group,
                                unsigned start, unsigned elems, bool masked,
                                bool e4m3, bool saturate)
{
  if (start >= vecRegs_.elemCount())
    return;

  uint16_t e1{};
  uint8_t dest{};
  unsigned group2x = group*2;
  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  auto rm = getFpRoundingMode();

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group2x, e1);
	  dest = bfloat16ToOfp8(e1, e4m3, rm, saturate);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfncvt_f_f_w(const DecodedInst* di)
{
  // Double-wide float to float.
  if (not checkVecFpInst(di, true, &Hart::isZvfhminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfncvt_f_f_w<BFloat16>(vd, vs1, group, start, elems, masked);
      else
        vfncvt_f_f_w<Float16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vfncvt_f_f_w<float>  (vd, vs1, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    case EW::Word2:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfncvt_rod_f_f_w(const DecodedInst* di)
{
  // Double-wide float to float.

  if (not checkVecFpInst(di))
    return;

#if SOFT_FLOAT
  softfloat_roundingMode = softfloat_round_odd;
  // TBD FIX: what if not using SOFT_FLOAT
#endif

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfncvt_f_f_w<BFloat16>(vd, vs1, group, start, elems, masked);
      else
        vfncvt_f_f_w<Float16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vfncvt_f_f_w<float>  (vd, vs1, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::doVecFpRedSumGroup(std::vector<ELEM_TYPE>& elems, ElementWidth eew, unsigned groupX8)
{
  // No reduction needed, LMUL <= 1.
  if (groupX8 <= 8)
    return;

  const unsigned group = groupX8 >> 3;
  const unsigned numGroupRed = group >> 1;
  const unsigned elemsPerVec = vecRegs_.singleMax(eew);

  using VSO = VecRegs::Step::Operation;

  for (unsigned gn = 0; gn < numGroupRed; gn++)
    {
      for (unsigned ix = 0; ix < elemsPerVec; ix++)
        {
          unsigned elemIx = gn*elemsPerVec + ix;
          unsigned oelemIx = (gn + numGroupRed)*elemsPerVec + ix;

          ELEM_TYPE e1 = elems.at(elemIx);
          ELEM_TYPE e2 = elems.at(oelemIx);

          ELEM_TYPE result = doFadd(e1, e2);
          elems.at(elemIx) = result;
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
          vecRegs_.steps_.emplace_back(VSO::CrossRegRed, e1, e2, result);
        }
    }

  return doVecFpRedSumGroup(elems, eew, numGroupRed * 8);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::doVecFpRedSumAdjacent(std::vector<ELEM_TYPE>& elems, unsigned numElems, unsigned numResult)
{
  if (numElems <= numResult)
    return;

  using VSO = VecRegs::Step::Operation;

  for (unsigned ix = 0; ix < numElems; ix+=2)
    {
      ELEM_TYPE e1 = elems.at(ix);
      ELEM_TYPE e2 = elems.at(ix + 1);

      ELEM_TYPE result = doFadd(e1, e2);
      elems.at(ix >> 1) = result;
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.steps_.emplace_back(VSO::AdjacRed, e1, e2, result);
    }

  return doVecFpRedSumAdjacent(elems, numElems >> 1, numResult);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::doVecFpRedSumStride(std::vector<ELEM_TYPE>& elems, unsigned numElems, unsigned numResult)
{
  if (numElems <= numResult)
    return;

  using VSO = VecRegs::Step::Operation;

  unsigned resIx = 0;
  unsigned stride = 3;
  for (unsigned ix = 0; ix < numElems; ix+=stride, ++resIx)
    {
      ELEM_TYPE e1 = elems.at(ix);
      ELEM_TYPE e2 = elems.at(ix + 2);

      ELEM_TYPE result = doFadd(e1, e2);
      elems.at(resIx) = result;
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.steps_.emplace_back(VSO::StrideRed, e1, e2, result);
      stride ^= 2;
    }

  return doVecFpRedSumStride(elems, numElems >> 1, numResult);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredusum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2);

  ELEM_TYPE e1{}, result{e2};

  bool anyActive = false;

  if (not vecRegs_.fpUnorderedSumTreeRed_.at(__builtin_ctz(sizeof(ELEM_TYPE))))
    {
      for (unsigned ix = start; ix < elems; ++ix)
        {
          if (masked and not vecRegs_.isActive(0, ix))
            {
              vecRegs_.fpFlags_.push_back(0);
              continue;
            }

          anyActive = true;
          vecRegs_.read(vs1, ix, group, e1);
          result = doFadd(result, e1);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
        }
    }
  else
    {
      std::vector<ELEM_TYPE> tree(vecRegs_.elemMax());
      bool roundDown = getFpRoundingMode() == RoundingMode::Down;

      // Replace inactive elements with additive identity.
      for (unsigned ix = start; ix < vecRegs_.elemMax(); ix++)
        if ((ix >= elems) or
            (masked and not vecRegs_.isActive(0, ix)))
          tree.at(ix) = roundDown? ELEM_TYPE{0} : -ELEM_TYPE{0};
        else
          {
            vecRegs_.read(vs1, ix, group, e1);
            tree.at(ix) = e1;
            anyActive = true;
          }

      // Perform group-wise reduction first.
      if (group)
        doVecFpRedSumGroup(tree, vecRegs_.elemWidth(), group);

      // Perform adjacent vec register reduce.
      doVecFpRedSumAdjacent(tree, vecRegs_.singleMax(vecRegs_.elemWidth()), 2);

      using VSO = VecRegs::Step::Operation;

      // scalar operand in second-to-last step.
      e1 = tree.at(0);
      result = doFadd(e1, e2);
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.steps_.emplace_back(VSO::ScalarRed, e1, e2, result);

      // remaining operand in last step.
      e1 = tree.at(1);
      e2 = result;
      result = doFadd(e1, e2);
      incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.steps_.emplace_back(VSO::AdjacRed, e1, e2, result);
    }

  // Note: NaN canonicalization when there are no active elements
  // is only allowed for vfredusum.vs and NOT for vfredosum.vs,
  // vfredmin.vs, and vfredmax.vs.
  if (not anyActive and std::isnan(result) and
      vecRegs_.fpUnorderedSumCanonical_.at(__builtin_ctz(sizeof(ELEM_TYPE))))
    result = std::numeric_limits<decltype(result)>::quiet_NaN();

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  unsigned destElems = vecRegs_.singleMax(vecRegs_.elemWidth());
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
}


template <typename URV>
void
Hart<URV>::execVfredusum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkRedOpVsEmul(di))
    return;

  if (elems == 0)
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfredusum_vs<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfredusum_vs<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfredusum_vs<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredosum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2);
  
  ELEM_TYPE e1{}, result{e2};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
        {
          vecRegs_.fpFlags_.push_back(0);
          continue;
        }

      vecRegs_.read(vs1, ix, group, e1);
      result = doFadd(result, e1);
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
    }

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  unsigned destElems = vecRegs_.singleMax(vecRegs_.elemWidth());
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
}


template <typename URV>
void
Hart<URV>::execVfredosum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkRedOpVsEmul(di))
    return;

  if (elems == 0)
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfredosum_vs<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfredosum_vs<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfredosum_vs<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredmin_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2);

  ELEM_TYPE e1{}, result{e2};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
        {
          vecRegs_.fpFlags_.push_back(0);
          continue;
        }

      vecRegs_.read(vs1, ix, group, e1);
      result = doFmin(result, e1);
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
    }

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  unsigned destElems = vecRegs_.singleMax(vecRegs_.elemWidth());
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfredmin_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkRedOpVsEmul(di))
    return;

  if (elems == 0)
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfredmin_vs<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfredmin_vs<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfredmin_vs<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfredmax_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		       unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e2{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, e2);

  ELEM_TYPE e1{}, result{e2};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
        {
          vecRegs_.fpFlags_.push_back(0);
          continue;
        }

      vecRegs_.read(vs1, ix, group, e1);
      result = doFmax(result, e1);
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
    }

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  unsigned destElems = vecRegs_.singleMax(vecRegs_.elemWidth());
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfredmax_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di))
    return;

  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemCount();
  ElementWidth sew = vecRegs_.elemWidth();
  bool masked = di->isMasked();

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkRedOpVsEmul(di))
    return;

  if (elems == 0)
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfredmax_vs<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfredmax_vs<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfredmax_vs<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    default:
      postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwredusum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
			unsigned start, unsigned elems, bool masked)
{
  if (elems == 0)
    return;

  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE2X result{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result);

  ElementWidth dsew = vecRegs_.elemWidth();
  if (not VecRegs::doubleSew(vecRegs_.elemWidth(), dsew))
    assert(0 && "Error: Assertion failed");

  ELEM_TYPE e1{};

  bool anyActive = false;

  if (not vecRegs_.fpUnorderedSumTreeRed_.at(__builtin_ctz(sizeof(ELEM_TYPE2X))))
    {
      for (unsigned ix = start; ix < elems; ++ix)
        {
          if (masked and not vecRegs_.isActive(0, ix))
            {
              vecRegs_.fpFlags_.push_back(0);
              continue;
            }

          anyActive = true;
          vecRegs_.read(vs1, ix, group, e1);
          ELEM_TYPE2X e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
          result           = doFadd(result, e1dw);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
        }
    }
  else
    {
      std::vector<ELEM_TYPE2X> tree(vecRegs_.elemMax());
      bool roundDown = getFpRoundingMode() == RoundingMode::Down;

      // Replace inactive elements with additive identity.
      for (unsigned ix = start; ix < vecRegs_.elemMax(); ix++)
        {
          if ((ix >= elems) or
              (masked and not vecRegs_.isActive(0, ix)))
            tree.at(ix) = roundDown? ELEM_TYPE2X{0} : -ELEM_TYPE2X{0};
          else
            {
              vecRegs_.read(vs1, ix, group, e1);
              ELEM_TYPE2X e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
              tree.at(ix) = e1dw;
              anyActive = true;
            }
        }

      // Perform reduction first for double-wide on register group.
      if constexpr (not std::is_same_v<ELEM_TYPE, Float16>)
        doVecFpRedSumAdjacent(tree, vecRegs_.elemMax(), vecRegs_.elemMax() / 2);
      else
        doVecFpRedSumStride(tree, vecRegs_.elemMax(), vecRegs_.elemMax() / 2);

      // Perform group-wise reduction.
      if (group > 8)
        doVecFpRedSumGroup(tree, dsew, group);

      // Perform adjacent vec register elements reduce.
      doVecFpRedSumAdjacent(tree, vecRegs_.singleMax(dsew), 2);

      using VSO = VecRegs::Step::Operation;

      // scalar operand in second-to-last step.
      ELEM_TYPE2X e1dw = tree.at(0);
      ELEM_TYPE2X e2dw = result;
      result = doFadd(e1dw, e2dw);
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.steps_.emplace_back(VSO::ScalarRed, e1dw, e2dw, result);

      // remaining operand in last step.
      e1dw = tree.at(1);
      e2dw = result;
      result = doFadd(e1dw, e2dw);
      incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.steps_.emplace_back(VSO::AdjacRed, e1dw, e2dw, result);
    }

  // Note: NaN canonicalization when there are no active elements
  // is only allowed for vfwredusum.vs and NOT for vfwredosum.vs.
  if (not anyActive and std::isnan(result) and
      vecRegs_.fpUnorderedSumCanonical_.at(__builtin_ctz(sizeof(ELEM_TYPE2X))))
    result = std::numeric_limits<decltype(result)>::quiet_NaN();

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  unsigned destElems = vecRegs_.singleMax(dsew);
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
}


template <typename URV>
void
Hart<URV>::execVfwredusum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di, true))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  unsigned gx8 = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned elems = vecRegs_.elemCount();
  bool masked = di->isMasked();

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkWideRedOpVsEmul(di))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfwredusum_vs<Float16>(vd, vs1, vs2, gx8, start, elems, masked);
      break;
    case EW::Word:
      vfwredusum_vs<float>  (vd, vs1, vs2, gx8, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template<typename ELEM_TYPE>
void
Hart<URV>::vfwredosum_vs(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
			 unsigned start, unsigned elems, bool masked)
{
  if (elems == 0)
    return;

  using ELEM_TYPE2X = makeDoubleWide_t<ELEM_TYPE>;

  ELEM_TYPE2X result{};
  unsigned scalarElemIx = 0, scalarElemGroupX8 = 8;

  vecRegs_.read(vs2, scalarElemIx, scalarElemGroupX8, result);
  
  ELEM_TYPE e1{};

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (masked and not vecRegs_.isActive(0, ix))
        {
          vecRegs_.fpFlags_.push_back(0);
          continue;
        }

      vecRegs_.read(vs1, ix, group, e1);

      ELEM_TYPE2X e1dw = fpConvertTo<ELEM_TYPE2X, true>(e1);
      result = doFadd(result, e1dw);
      URV incFlags = activeSimulatorFpFlags();
      vecRegs_.fpFlags_.push_back(incFlags);
    }

  vecRegs_.write(vd, scalarElemIx, scalarElemGroupX8, result);
  ElementWidth dsew{};
  if (not VecRegs::doubleSew(vecRegs_.elemWidth(), dsew))
    assert(0 && "Error: Assertion failed");

  unsigned destElems = vecRegs_.singleMax(dsew);
  for (unsigned ix = 1; ix < destElems; ++ix)
    if (vecRegs_.tailAgn_ and vecRegs_.tailAgnOnes_)
      {
        setAllBits(result);
        vecRegs_.write(vd, ix, scalarElemGroupX8, result);
      }
}


template <typename URV>
void
Hart<URV>::execVfwredosum_vs(const DecodedInst* di)
{
  if (not checkFpSewLmulVstart(di, true))
    return;

  ElementWidth sew = vecRegs_.elemWidth();
  unsigned gx8 = vecRegs_.groupMultiplierX8();
  unsigned start = csRegs_.peekVstart();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();

  unsigned elems = vecRegs_.elemCount();
  bool masked = di->isMasked();

  if (not checkRoundingModeCommon(di))
    {
      postVecFail(di);
      return;
    }

  if (not checkWideRedOpVsEmul(di))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        { postVecFail(di); return; }
      vfwredosum_vs<Float16>(vd, vs1, vs2, gx8, start, elems, masked);
      break;
    case EW::Word:
      vfwredosum_vs<float>  (vd, vs1, vs2, gx8, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:
      postVecFail(di); return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrsqrt7_v(unsigned vd, unsigned vs1, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};

  bool inv = false, dbz = false;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  bool edbz = false, einv = false;  // Element divide-by-zero and invalid
	  dest = doFrsqrt7(e1, edbz, einv);
	  dbz = dbz or edbz;
	  inv = inv or einv;
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  clearSimulatorFpFlags();

  if (inv) raiseSimulatorFpFlags(FpFlags::Invalid);
  if (dbz) raiseSimulatorFpFlags(FpFlags::DivByZero);

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfrsqrt7_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfrsqrt7_v<BFloat16>(vd, vs1, group, start, elems, masked);
      else
        vfrsqrt7_v<Float16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vfrsqrt7_v<float>  (vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vfrsqrt7_v<double> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfrec7_v(unsigned vd, unsigned vs1, unsigned group,
		     unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};

  FpFlags flags = FpFlags::None;
  auto mode = getFpRoundingMode();

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  FpFlags elemFlags = FpFlags::None;
	  dest = doFrec7(e1, mode, elemFlags);
	  flags = FpFlags(unsigned(flags) | unsigned(elemFlags));
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  orFcsrFlags(flags);
}


template <typename URV>
void
Hart<URV>::execVfrec7_v(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfrec7_v<BFloat16>(vd, vs1, group, start, elems, masked);
      else
        vfrec7_v<Float16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:
      vfrec7_v<float>  (vd, vs1, group, start, elems, masked);
      break;
    case EW::Word2:
      vfrec7_v<double> (vd, vs1, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmin_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  dest = doFmin(e1, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmin_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmin_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfmin_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmin_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmin_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:   // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmin_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFmin(e1, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmin_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmin_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfmin_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmin_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmin_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall through.
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmax_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = doFmax(e1, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmax_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmax_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked);
      else
        vfmax_vv<Float16>(vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmax_vv<float>  (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmax_vv<double> (vd, vs1, vs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfmax_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = doFmax(e1, e2);
          URV incFlags = activeSimulatorFpFlags();
          vecRegs_.fpFlags_.push_back(incFlags);
	}
      else
        vecRegs_.fpFlags_.push_back(0);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfmax_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfmax_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfmax_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfmax_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfmax_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:  // Fall through
    default:
      postVecFail(di);
      return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnj_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
                    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  dest = std::copysign(e1, e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfsgnj_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:  vfsgnj_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnj_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnj_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnj_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		    unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = std::copysign(e1, e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfsgnj_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfsgnj_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfsgnj_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfsgnj_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfsgnj_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjn_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);
	  e2 = -e2;
	  dest = std::copysign(e1, e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfsgnjn_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:  vfsgnjn_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnjn_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnjn_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjn_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  e2 = -e2;

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = std::copysign(e1, e2);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfsgnjn_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfsgnjn_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfsgnjn_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfsgnjn_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfsgnjn_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjx_vv(unsigned vd, unsigned vs1, unsigned vs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, e2{}, dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  vecRegs_.read(vs2, ix, group, e2);

	  int s1 = (std::signbit(e1) == 0) ? 0 : 1;
	  int s2 = (std::signbit(e2) == 0) ? 0 : 1;
	  int sign = s1 ^ s2;
	  ELEM_TYPE x{};
	  if (sign)
	    x = -x;
	  dest = std::copysign(e1, x);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfsgnjx_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
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
    case EW::Half:  vfsgnjx_vv<Float16>(vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word:  vfsgnjx_vv<float>  (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Word2: vfsgnjx_vv<double> (vd, vs1, vs2, group, start, elems, masked); break;
    case EW::Byte:  // Fall-through to invalid case
    default:        postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
template <typename ELEM_TYPE>
void
Hart<URV>::vfsgnjx_vf(unsigned vd, unsigned vs1, unsigned fs2, unsigned group,
		      unsigned start, unsigned elems, bool masked)
{
  ELEM_TYPE e1{}, dest{};
  auto e2 = fpRegs_.read<ELEM_TYPE>(fs2);

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);

	  int s1 = (std::signbit(e1) == 0) ? 0 : 1;
	  int s2 = (std::signbit(e2) == 0) ? 0 : 1;
	  int sign = s1 ^ s2;
	  ELEM_TYPE x{};
	  if (sign)
	    x = -x;
	  dest = std::copysign(e1, x);
	}
      vecRegs_.write(vd, ix, destGroup, dest);
    }
}


template <typename URV>
void
Hart<URV>::execVfsgnjx_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  rs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not checkVecOpsVsEmul(di, group, {vd, vs1}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:
      if (vecRegs_.altfmt())
        vfsgnjx_vf<BFloat16>(vd, vs1, rs2, group, start, elems, masked);
      else
        vfsgnjx_vf<Float16>(vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word:
      vfsgnjx_vf<float>  (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Word2:
      vfsgnjx_vf<double> (vd, vs1, rs2, group, start, elems, masked);
      break;
    case EW::Byte:
    default:
      postVecFail(di); return;
    }

  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfncvtbf16_f_f_w(const DecodedInst* di)
{
  // Double-wide float to float.
  if (not checkVecFpInst(di, true, &Hart::isZvfbfminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
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
    case EW::Byte:
      {
        if (not isRvzvfofp8min())
          {
            postVecFail(di);
            return;
          }
        bool alt = (peekCsr(CsrNumber::VTYPE) >> 8) & 1;
        bool e4m3 = not alt;
        vfncvtBfloat16ToOfp8(vd, vs1, group, start, elems, masked, e4m3, false);
      }
      break;

    case EW::Half:
      vfncvt_f_f_w<BFloat16>(vd, vs1, group, start, elems, masked);
      break;

    case EW::Word:
    case EW::Word2:
    default:
      postVecFail(di);
      return;
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


namespace WdRiscv
{
  uint16_t ofp8ToBfloat16(uint8_t x, bool e4m3)
  {
    unsigned bias16 = 127;             // Expoenet bias in bfloat16_t.
    unsigned bias8 = e4m3 ? 7 : 15;    // Exponent bias in opf8 e4m3/e5m2
    unsigned biasDiff = bias16 - bias8;

    unsigned mantBits16 = 7;
    unsigned mantBits8 = e4m3 ? 3 : 2;
    unsigned mantBitsDiff = mantBits16 - mantBits8;  // Difference in mantissa width,

    uint32_t maxExp = e4m3 ? 0xf : 0x1f;

    uint32_t sign = x >> 7;
    uint32_t exp = e4m3 ? ((x >> 3) & 0xf) : ((x >> 2) & 0x1f);
    uint32_t mant = e4m3 ? (x & 7) : (x & 3);

    if (not e4m3 and exp == maxExp and mant == 0)  // Infinity
      {
        auto bf16 = std::numeric_limits<BFloat16>::infinity();
        if (sign)
          bf16 = -bf16;
        return std::bit_cast<uint16_t>(bf16);
      }

    bool nan = exp == maxExp and ((e4m3 and mant == 7) or (not e4m3 and mant != 0));
    if (nan)
      {
        auto bf16 = std::numeric_limits<BFloat16>::quiet_NaN();
        return std::bit_cast<uint16_t>(bf16);
      }

    if (exp > 0)   // Normalized
      return (sign << 15) | ((exp + biasDiff) << 7) | (mant << mantBitsDiff);

    if (mant == 0) // Zero
      return sign << 15;

    // Subnormal.
    exp = biasDiff + 1;

    if (e4m3)
      {
        if (mant >= 4)
          {
            mant &= ~4;
            mant <<= 5;
            exp -= 1;
          }
        else if (mant >= 2)
          {
            mant &= ~2;
            mant <<= 6;
            exp -= 2;
          }
        else
          {
            mant = 0;
            exp -= 3;
          }
      }
    else
      {
        if (mant >= 2)
          {
            mant &= ~2;
            mant <<= 6;
            exp -= 1;
          }
        else
          {
            mant = 0;
            exp -= 2;
          }
      }
    return (sign << 15) | (exp << 7) | mant;
  }
}


template <typename URV>
void
Hart<URV>::vfncvtOfp8ToBfloat16(unsigned vd, unsigned vs1, unsigned group, unsigned start,
                                unsigned elems, bool masked, bool e4m3)
{
  uint8_t e1{};
  uint16_t dest{};

  unsigned destGroup = std::max(VecRegs::groupMultiplierX8(GroupMultiplier::One), group*2);

  if (start >= vecRegs_.elemCount())
    return;

  for (unsigned ix = start; ix < elems; ++ix)
    {
      URV incFlags{};
      if (vecRegs_.isDestActive(vd, ix, destGroup, masked, dest))
	{
	  vecRegs_.read(vs1, ix, group, e1);
	  dest = ofp8ToBfloat16(e1, e4m3);
	}
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.write(vd, ix, destGroup, dest);
    }

  updateAccruedFpBits();
}


template <typename URV>
void
Hart<URV>::execVfwcvtbf16_f_f_v(const DecodedInst* di)
{
  // Float to double-wide float.
  if (not checkVecFpInst(di, true, &Hart::isZvfbfminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}}))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Byte:
      {
        if (not isRvzvfofp8min())
          {
            postVecFail(di);
            return;
          }
        bool alt = (peekCsr(CsrNumber::VTYPE) >> 8) & 1;
        bool e4m3 = not alt;
        vfncvtOfp8ToBfloat16(vd, vs1, group, start, elems, masked, e4m3);
      }
      break;
    case EW::Half:
      vfwcvt_f_f_v<BFloat16>(vd, vs1, group, start, elems, masked);
      break;
    case EW::Word:  // Fall-through to invalid case
    case EW::Word2: // Fall-through to invalid case
    default:
      postVecFail(di);
      return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfwmaccbf16_vv(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true, &Hart::isZvfbfwmaLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs1, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half:   vfwmacc_vv<BFloat16>(vd, vs1, vs2, group, start, elems, masked); break;
    default:         postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfwmaccbf16_vf(const DecodedInst* di)
{
  if (not checkVecFpInst(di, true, &Hart::isZvfbfwmaLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  fs1 = di->op1(),  vs2 = di->op2();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  ElementWidth dsew{}, sew = vecRegs_.elemWidth();

  if (not vecRegs_.isDoubleWideLegal(sew, dsew, group))
    {
      postVecFail(di);
      return;
    }

  unsigned elems = vecRegs_.elemMax(dsew);

  bool vdSrc = true;  // Vd is also a source operand.
  // Check interface assumes all vector operands. We pass vs2 for rs1 -- no harm.
  if (not checkVecOpsVsEmul(di, group, {{vd, true}, {vs2, false}, {vs2, false}}, vdSrc))
    return;

  using EW = ElementWidth;
  switch (sew)
    {
    case EW::Half: vfwmacc_vf<BFloat16>(vd, fs1, vs2, group, start, elems, masked); break;
    default:       postVecFail(di); return;
    }
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfncvtbf16_sat_f_f_w(const DecodedInst* di)
{
  // Bfloat16 to OFP8
  if (not checkVecFpInst(di, true, &Hart::isZvfbfminLegal))
    return;

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned group = vecRegs_.groupMultiplierX8(),  start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  if (not isRvzvfofp8min() or not vecRegs_.isDoubleWideLegal(sew, group) or sew != ElementWidth::Byte)
    {
      postVecFail(di);
      return;
    }

  bool alt = (peekCsr(CsrNumber::VTYPE) >> 8) & 1;
  bool e4m3 = not alt;
  vfncvtBfloat16ToOfp8(vd, vs1, group, start, elems, masked, e4m3, true);

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::vfncvt_f_f_q(const DecodedInst* di, bool saturate)
{
  // fp32 to ofp8

  bool masked = di->isMasked();
  unsigned vd = di->op0(),  vs1 = di->op1();
  unsigned dgx8 = vecRegs_.groupMultiplierX8();  // Destination group times 8
  unsigned start = csRegs_.peekVstart();
  unsigned elems = vecRegs_.elemMax();
  ElementWidth sew = vecRegs_.elemWidth();

  unsigned dg = dgx8 >= 8 ? dgx8 / 8 : 1;  // Destination group.

  // Source group: EMUL = 4*LMUL
  unsigned sgx8 = 4 * dgx8;   // Source group times 8
  unsigned sg = sgx8 >= 8 ? sgx8 / 8 : 1;  // Source group.

  bool valid = ( isRvzvfofp8min() and isFpLegal() and checkRoundingModeCommon(di) and
                 sgx8 >= 8 and sg <= 8 and sew == ElementWidth::Byte and
                 (vs1 % sg) == 0 and (vd % dg) == 0 );

  unsigned desElemWid = 8, srcElemWid = 32;  // Elem width in bits.

  valid = valid and checkDestSourceOverlap(vd, desElemWid, dgx8, vs1, srcElemWid, sgx8);
  if (not valid)
    {
      postVecFail(di);
      return;
    }

  bool alt = (peekCsr(CsrNumber::VTYPE) >> 8) & 1;
  bool e4m3 = not alt;

  auto rm = getFpRoundingMode();

  if (start >= vecRegs_.elemCount())
    {
      postVecSuccess(di);
      return;
    }

  dgx8 = std::max(dgx8, 8u);
  sgx8 = std::max(sgx8, 8u);

  for (unsigned ix = start; ix < elems; ++ix)
    {
      URV incFlags{};
      uint8_t dest{};
      if (vecRegs_.isDestActive(vd, ix, dgx8, masked, dest))
        {
          uint32_t e1{};
          vecRegs_.read(vs1, ix, sgx8, e1);
          bool inputNan = ((e1 & 0x7f80'0000) == 0x7f80'0000) and (e1 & 0x007f'ffff);
          bool inputInf = ((e1 & 0x7fff'ffff) == 0x7f80'0000);
          if (inputNan)
            {
              dest = 0x7f;
              if ((e1 & 0x0040'0000) == 0)
                raiseSimulatorFpFlags(FpFlags::Invalid);
            }
          else if (e4m3 and inputInf)
            {
              unsigned neg = (e1 >> 31);
              dest = saturate ? uint8_t(neg << 7) | uint8_t(0b1111'110) : uint8_t(0x7f);
            }
          else if ((not e4m3) and inputInf)
            {
              unsigned neg = (e1 >> 31);
              dest = uint8_t(neg << 7) | uint8_t(0b11111'00);
              if (saturate)
                dest = uint8_t(neg << 7) | uint8_t(0b11110'11);
            }
          if (e4m3)
            {
              if (not inputNan and not inputInf)
                {
                  unsigned neg = (e1 >> 31);
                  bool inf = false;
                  dest = floatToOfp8E4m3(e1, rm, inf);
                  if (saturate and inf)
                    dest = uint8_t(neg << 7) | uint8_t(0b1111'110);
                }
            }
          else
            {
              if (not inputNan and not inputInf)
                {
                  dest = floatToOfp8E5m2(e1, rm);
                  if (saturate)
                    {
                      if (dest == 0b0'11111'00)      // +inf
                        dest = 0b0'11110'11;         // Max val
                      else if (dest == 0b1'11110'00) // -inf
                        dest = 0b1'11110'11;         // Neg max val
                    }
                }
            }
        }
      vecRegs_.fpFlags_.push_back(incFlags);
      vecRegs_.write(vd, ix, dgx8, dest);
    }

  updateAccruedFpBits();
  postVecSuccess(di);
}


template <typename URV>
void
Hart<URV>::execVfncvt_f_f_q(const DecodedInst* di)
{
  vfncvt_f_f_q(di, false /*saturate*/);
}


template <typename URV>
void
Hart<URV>::execVfncvt_sat_f_f_q(const DecodedInst* di)
{
  vfncvt_f_f_q(di, true /*saturate*/);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
