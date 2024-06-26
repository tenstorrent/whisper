#pragma once

#include <bit>
extern "C" {
#include <softfloat.h>
}
#include "float16-compat.hpp"

namespace WdRiscv
{

  /// Convert softfloat float16_t type to Float16.
  constexpr Float16 softToNative(float16_t f16)
  {
    return std::bit_cast<Float16>(f16.v);
  }

  /// Convert softfloat bfloat16_t type to BFloat16.
  constexpr BFloat16 softToNative(bfloat16_t f16)
  {
    return std::bit_cast<BFloat16>(f16.v);
  }


  /// Convert softfloat float32_t type to float.
  constexpr float softToNative(float32_t f32)
  {
    return std::bit_cast<float>(f32.v);
  }


  /// Convert softfloat float64_t to double.
  constexpr double softToNative(float64_t f64)
  {
    return std::bit_cast<double>(f64.v);
  }


  /// Convert Float16 to a softfloat float16_t
  constexpr float16_t
  nativeToSoft(Float16 x)
  {
    return float16_t{std::bit_cast<uint16_t>(x)};
  }


  /// Convert BFloat16 to a softfloat bfloat16_t
  constexpr bfloat16_t
  nativeToSoft(BFloat16 x)
  {
    return bfloat16_t{std::bit_cast<uint16_t>(x)};
  }


  /// Convert a native float to a softfloat float32_t
  constexpr float32_t
  nativeToSoft(float x)
  {
    return float32_t{std::bit_cast<uint32_t>(x)};
  }


  /// Convert a native double to a softfloat float64_t
  constexpr float64_t
  nativeToSoft(double x)
  {
    return float64_t{std::bit_cast<uint64_t>(x)};
  }


  /// Perform a floating point add using the softfloat library.
  inline float
  softAdd(float a, float b)
  {
    float res = softToNative(f32_add(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point add using the softfloat library.
  inline double
  softAdd(double a, double b)
  {
    double res = softToNative(f64_add(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point add using the softfloat library.
  inline Float16
  softAdd(Float16 a, Float16 b)
  {
    Float16 res = softToNative(f16_add(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point subtract using the softfloat library.
  inline float
  softSub(float a, float b)
  {
    float res = softToNative(f32_sub(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point subtract using the softfloat library.
  inline double
  softSub(double a, double b)
  {
    double res = softToNative(f64_sub(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point subtract using the softfloat library.
  inline Float16
  softSub(Float16 a, Float16 b)
  {
    Float16 res = softToNative(f16_sub(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point mul using the softfloat library.
  inline float
  softMul(float a, float b)
  {
    float res = softToNative(f32_mul(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point mul using the softfloat library.
  inline double
  softMul(double a, double b)
  {
    double res = softToNative(f64_mul(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point mul using the softfloat library.
  inline Float16
  softMul(Float16 a, Float16 b)
  {
    Float16 res = softToNative(f16_mul(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point divide using the softfloat library.
  inline float
  softDiv(float a, float b)
  {
    float res = softToNative(f32_div(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point divide using the softfloat library.
  inline double
  softDiv(double a, double b)
  {
    double res = softToNative(f64_div(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a floating point divide using the softfloat library.
  inline Float16
  softDiv(Float16 a, Float16 b)
  {
    Float16 res = softToNative(f16_div(nativeToSoft(a), nativeToSoft(b)));
    return res;
  }


  /// Perform a fused multiply-add using the softfloat library.
  inline float
  softFma(float a, float b, float c)
  {
    float32_t tmp = f32_mulAdd(nativeToSoft(a), nativeToSoft(b), nativeToSoft(c));
    float res = softToNative(tmp);
    return res;
  }


  /// Perform a fused multiply-add using the softfloat library.
  inline double
  softFma(double a, double b, double c)
  {
    float64_t tmp = f64_mulAdd(nativeToSoft(a), nativeToSoft(b), nativeToSoft(c));
    double res = softToNative(tmp);
    return res;
  }


  /// Perform a fused multiply-add using the softfloat library.
  inline Float16
  softFma(Float16 a, Float16 b, Float16 c)
  {
    float16_t tmp = f16_mulAdd(nativeToSoft(a), nativeToSoft(b), nativeToSoft(c));
    Float16 res = softToNative(tmp);
    return res;
  }


  /// Perform a floating point sqrt using the softfloat library.
  inline float
  softSqrt(float a)
  {
    float res = softToNative(f32_sqrt(nativeToSoft(a)));
    return res;
  }


  /// Perform a floating point sqrt using the softfloat library.
  inline double
  softSqrt(double a)
  {
    double res = softToNative(f64_sqrt(nativeToSoft(a)));
    return res;
  }


  /// Perform a floating point sqrt using the softfloat library.
  inline Float16
  softSqrt(Float16 a)
  {
    Float16 res = softToNative(f16_sqrt(nativeToSoft(a)));
    return res;
  }


  /// Perform a floating point sqrt using the softfloat library.
  inline float
  softRound(float a, bool exact)
  {
    float res = softToNative(f32_roundToInt(nativeToSoft(a), softfloat_roundingMode, exact));
    return res;
  }


  /// Perform a floating point sqrt using the softfloat library.
  inline double
  softRound(double a, bool exact)
  {
    double res = softToNative(f64_roundToInt(nativeToSoft(a), softfloat_roundingMode, exact));
    return res;
  }


  /// Perform a floating point sqrt using the softfloat library.
  inline Float16
  softRound(Float16 a, bool exact)
  {
    Float16 res = softToNative(f16_roundToInt(nativeToSoft(a), softfloat_roundingMode, exact));
    return res;
  }

}
