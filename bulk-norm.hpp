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

#pragma once

// RISC-V Bulk Normalization Algorithm (RVBNA) used by the floating point dot product
// (Zvfdota) and matrix multiply (Zvt) instructions.

#include <bit>
#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>
#include "float-util.hpp"


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
inline auto
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
  uint32_t p_l = m_l + 1;                               // Significand bit-count
  uint32_t maskExpLHS = (1 << e_l) - 1;                 // bitmask of exponent
  uint32_t maskMantLHS = (1 << m_l) - 1;                // bitmask for mantissa

  // Right operand parameters
  uint32_t m_r = std::numeric_limits<RT>::digits - 1;   // Mantissa bit-count
  uint32_t e_r = (sizeof(RT)*8) - m_r - 1;              // Exp bit-count
  uint32_t p_r = m_r + 1;                               // Significand bit-count
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

      prodSigs.at(i) =  uint64_t(A_i_sig) * uint64_t(B_i_sig);

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
  uint32_t lzc = std::countl_zero(accAbs);  // Leading zero count with width of 64
  lzc -= (sizeof(accAbs)*8) - (g + q + 1 + o); // Required lzc

  int32_t resExp = 0;
  if (accumulator != 0)
    resExp = static_cast<int32_t>((maxExp + o + 1u - lzc) - prodOpBias + res_bias);
  uint64_t unroundedSig = (accAbs << lzc) >> (g + o + 1);
  uint64_t rawJamMask = (1LL << (g + o + 1)) - 1;
  uint64_t jamMaskShift = (lzc > (g + o + 1)) ? (g + o + 1) : lzc;
  uint64_t jamMask = rawJamMask >> jamMaskShift;

  bool jamSig = (accAbs & jamMask) != 0;
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
  int64_t denomShift = -resExp;
  uint64_t denormalizedSig = (accAbs << lzc) >> (g + o + 1 + 1 + denomShift);
  uint64_t discardedMask = ((1LL << (g + o + 1 + 1 + denomShift)) - 1);
  uint64_t discardedBits = (accAbs << lzc) & discardedMask;
  uint64_t forceLSB =  (discardedBits != 0 ? 1 : 0);
  return (accSign << (q + f - 1)) | denormalizedSig | forceLSB;
}
