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

#pragma once


/// Provide fixed size integer types wider than 64-bit.
/// The reason we do not use boost is that we want 2's complement representation
/// of signed integers.

#include <algorithm>
#include <array>
#include <bit>
#include <type_traits>
#include <concepts>
#include <cstdint>


namespace WdRiscv
{

  /// Dummy class to use as a wide integer concept.
  class WideIntBase
  {
  };

  /// Class for unsigned integers of width 128/256/512/1024 bits. Half
  /// is the half-wide unsigned type. Quarter is the quarter-wide
  /// unsigned type. For example, for Uint256, Half would be Uint128,
  /// and quarter uint64_t. The quarter type is used in the
  /// implementation of multiplication.
  template <typename Half, typename Quarter>
  class UwideInt;

  /// Class signed integers of width 128/256/512/1024 bits. Half is
  /// the half-wide signed type. Quarter is the quarter-wide unsigned
  /// type. For example, for Int256, Half would be Int128, and quarter
  /// int64_t.
  template <typename Half, typename Quarter>
  class WideInt;

#ifdef __SIZEOF_INT128__

  __extension__ using Int128  = __int128;
  __extension__ using Uint128 = unsigned __int128;

#else

  using Uint128  = UwideInt<uint64_t,  uint32_t>;
  using Int128  = WideInt<int64_t,  int32_t>;

#endif

  using Uint256  = UwideInt<Uint128,   uint64_t>;
  using Uint512  = UwideInt<Uint256,   Uint128>;
  using Uint1024 = UwideInt<Uint512,   Uint256>;
  using Uint2048 = UwideInt<Uint1024,  Uint512>;

  using Int256  = WideInt<Int128,   int64_t>;
  using Int512  = WideInt<Int256,   Int128>;
  using Int1024 = WideInt<Int512,   Int256>;
  using Int2048 = WideInt<Int1024,  Int512>;
}

namespace WdRiscv
{
  /// Like std::make_unsigned but extended to the wide integer
  /// types. Specializing the std traits for user types is undefined
  /// behavior (and rejected by newer libc++).
  template <typename T> struct MakeUnsigned { using type = std::make_unsigned_t<T>; };

  template <> struct MakeUnsigned<Int128>   { using type = Uint128; };
  template <> struct MakeUnsigned<Int256>   { using type = Uint256; };
  template <> struct MakeUnsigned<Int512>   { using type = Uint512; };
  template <> struct MakeUnsigned<Int1024>  { using type = Uint1024; };
  template <> struct MakeUnsigned<Int2048>  { using type = Uint2048; };
  template <> struct MakeUnsigned<Uint128>  { using type = Uint128; };
  template <> struct MakeUnsigned<Uint256>  { using type = Uint256; };
  template <> struct MakeUnsigned<Uint512>  { using type = Uint512; };
  template <> struct MakeUnsigned<Uint1024> { using type = Uint1024; };
  template <> struct MakeUnsigned<Uint2048> { using type = Uint2048; };

  /// Like std::make_signed but extended to the wide integer types.
  template <typename T> struct MakeSigned { using type = std::make_signed_t<T>; };

  template <> struct MakeSigned<Uint128>  { using type = Int128; };
  template <> struct MakeSigned<Uint256>  { using type = Int256; };
  template <> struct MakeSigned<Uint512>  { using type = Int512; };
  template <> struct MakeSigned<Uint1024> { using type = Int1024; };
  template <> struct MakeSigned<Uint2048> { using type = Int2048; };
  template <> struct MakeSigned<Int128>   { using type = Int128; };
  template <> struct MakeSigned<Int256>   { using type = Int256; };
  template <> struct MakeSigned<Int512>   { using type = Int512; };
  template <> struct MakeSigned<Int1024>  { using type = Int1024; };
  template <> struct MakeSigned<Int2048>  { using type = Int2048; };

  template <typename T> using MakeUnsignedT = typename MakeUnsigned<T>::type;
  template <typename T> using MakeSignedT   = typename MakeSigned<T>::type;
}


namespace WdRiscv
{
  /// Wide (wider than 64-bits) unsigned integer template.
  template <typename Half_, typename Quarter_>
  class UwideInt : public WideIntBase
  {
  public:

    using Half          = Half_;
    using Quarter       = Quarter_;
    using Self          = UwideInt<Half, Quarter>;
    using HalfSigned    = MakeSignedT<Half>;
    using QuarterSigned = MakeSignedT<Quarter>;
    using Signed        = WideInt<HalfSigned, QuarterSigned>;

    static constexpr int width()     { return 8*sizeof(Self); }
    static constexpr int halfWidth() { return 8*sizeof(Half); }

    /// Default constructor.
    constexpr UwideInt() = default;

    /// Copy constructor.
    constexpr UwideInt(const Self&) = default;

    /// Construct from a similar-width signed int: Copy bits.
    constexpr UwideInt(const Signed x)
      : bits_(static_cast<Half>(x.high()), static_cast<Half>(x.low()))
    { }

    /// Construct from a half-size unsigned type.
    constexpr UwideInt(Half x)
      : bits_(0, x)
    { }

    /// Construct from an unsigned integral type.
    template <std::unsigned_integral UINT>
    constexpr UwideInt(UINT x)
      : bits_(0, x)
    { }

    /// Construct from an signed integral type.
    template <std::signed_integral INT>
    constexpr UwideInt(INT x)
      : bits_(0, x)
    {
      if (x < 0)
	high_() = ~high_();
    }

    /// Construct from a pair of half-width unsigned ints.
    constexpr UwideInt(Half high, Half low)
      : bits_(high, low)
    { }

    /// Assignment operator.
    constexpr Self& operator = (const Self&) = default;

    /// Return least sig half.
    constexpr Half low() const
    { return bits_.halfs_.low_; }

    /// Return most sig half.
    constexpr Half high() const
    { return bits_.halfs_.high_; }

    /// Convert to a built-in integral type.
    template <std::integral INT>
    constexpr explicit operator INT() const
    { return static_cast<INT>(low_()); }

    /// Convert to boolean: Zero is false, non-zero is true.
    constexpr explicit operator bool() const
    { return high_() != 0 or low_() != 0; }

    /// Plus-equal operator.
    constexpr Self& operator += (const Self& x)
    {
      Half prevLow = low_();
      low_() += x.low_();
      high_() += x.high_();
      if (low_() < prevLow)
        high_()++;
      return *this;
    }

    /// Minus-equal operator.
    constexpr Self& operator -= (const Self& x)
    {
      Half prevLow = low_();
      low_() -= x.low_();
      high_() -= x.high_();
      if (low_() > prevLow)
        high_()--;
      return *this;
    }

    /// Bitwise or-equal operator.
    constexpr Self& operator |= (const Self& x)
    {
      low_() |= x.low_();
      high_() |= x.high_();
      return *this;
    }

    /// Bitwise and-equal operator.
    constexpr Self& operator &= (const Self& x)
    {
      low_() &= x.low_();
      high_() &= x.high_();
      return *this;
    }

    /// Bitwise xor-equal operator.
    constexpr Self& operator ^= (const Self& x)
    {
      low_() ^= x.low_();
      high_() ^= x.high_();
      return *this;
    }

    /// Pre-increment operator.
    constexpr Self& operator ++ ()
    { *this += 1; return *this; }

    /// Post-increment operator.
    constexpr Self operator ++ (int)
    { Self temp = *this; temp += 1u; return temp; }

    /// Pre-decrement operator.
    constexpr Self& operator -- ()
    { *this -= 1; return *this; }

    /// Post-decrement operator.
    constexpr Self operator -- (int)
    { Self temp = *this; temp -= 1u; return temp; }

    /// Bitwise complement operator.
    constexpr Self operator ~ () const
    { Self temp( ~high_(), ~low_()); return temp; }

    /// Multiply-equal operator.
    constexpr Self& operator *= (const Self& x)
    {
      constexpr unsigned qw = width() / 4;
      Quarter a0{bits_.quarters_.q0_}, a1 = bits_.quarters_.q1_;
      Quarter a2 = bits_.quarters_.q2_, a3 = bits_.quarters_.q3_;
      Quarter b0 = x.bits_.quarters_.q0_, b1 = x.bits_.quarters_.q1_;
      Quarter b2 = x.bits_.quarters_.q2_, b3 = x.bits_.quarters_.q3_;

      *this = 0;
      Self shifted = 0;

      Half prod = a0; prod *= b0; *this += prod;
      prod = a0; prod *= b1; shifted = prod; shifted <<= qw; *this += shifted;
      prod = a0; prod *= b2; shifted = prod; shifted <<= 2*qw; *this += shifted;
      prod = a0; prod *= b3; shifted = prod; shifted <<= 3*qw; *this += shifted;

      prod = a1; prod *= b0; shifted = prod; shifted <<= qw; *this += shifted;
      prod = a1; prod *= b1; shifted = prod; shifted <<= 2*qw; *this += shifted;
      prod = a1; prod *= b2; shifted = prod; shifted <<= 3*qw; *this += shifted;
      prod = a1; prod *= b3; shifted = prod; shifted <<= 4*qw; *this += shifted;

      prod = a2; prod *= b0; shifted = prod; shifted <<= 2*qw; *this += shifted;
      prod = a2; prod *= b1; shifted = prod; shifted <<= 3*qw; *this += shifted;
      prod = a2; prod *= b2; shifted = prod; shifted <<= 4*qw; *this += shifted;
      prod = a2; prod *= b3; shifted = prod; shifted <<= 5*qw; *this += shifted;

      prod = a3; prod *= b0; shifted = prod; shifted <<= 3*qw; *this += shifted;
      prod = a3; prod *= b1; shifted = prod; shifted <<= 4*qw; *this += shifted;
      prod = a3; prod *= b2; shifted = prod; shifted <<= 5*qw; *this += shifted;
      prod = a3; prod *= b3; shifted = prod; shifted <<= 6*qw; *this += shifted;

      return *this;
    }

    /// Divide-equal operator.
    constexpr Self& operator /= (const Self& x)
    {
      constexpr unsigned n = width();
      Self rem(0), result(0);

      Self y = *this;  // Dividend

      for (unsigned i = 0; i < n; ++i)
	{
	  uint8_t yMsb = y.bits_.bytes_.at(sizeof(y) - 1) >> 7; // Most sig bit of dividend
	  rem <<= 1;
	  result <<= 1;
	  y <<= 1;
          rem.bits_.bytes_.at(0) |= yMsb;  // Update least sig byte of remainder.
	  if (x <= rem)
	    {
	      result.bits_.bytes_.at(0) |= 1;  // Update least sig byte of result.
	      rem -= x;
	    }
	}

      *this = result;
      return *this;
    }

    /// Remainder-equal operator.
    constexpr Self& operator %= (const Self& x)
    {
      constexpr unsigned n = width();
      Self rem(0), result(0);

      Self y = *this;  // Dividend

      for (unsigned i = 0; i < n; ++i)
	{
	  uint8_t yMsb = y.bits_.bytes_.at(sizeof(y)-1) >> 7; // Most sig bit of dividend
	  rem <<= 1;
	  result <<= 1;
	  y <<= 1;
	  rem.bits_.bytes_.at(0) |= yMsb;   // Update least sig byte of rem
	  if (x <= rem)
	    {
	      result.bits_.bytes_.at(0) |= 1;  // Update least sig byte of result.
	      rem -= x;
	    }
	}

      *this = rem;
      return *this;
    }

    /// Right-shift operator.
    constexpr Self& operator >>= (int n)
    {
      if (n == 0)
	return *this;

      constexpr int halfw = halfWidth();

      if (n >= width())
	low_() = high_() = Half(0);
      else if (n >= halfw)
	{
	  low_() = high_();
	  high_() = Half(0);
	  low_() >>= (n - halfw);
	}
      else
	{
	  Half temp = high_() << (halfw - n);
	  high_() >>= n;
	  low_() >>= n;
	  low_() |= temp;
	}
      return *this;
    }

    /// Left-shift operator.
    constexpr Self& operator <<= (int n)
    {
      if (n == 0)
	return *this;

      constexpr int halfw = halfWidth();

      if (n >= width())
	low_() = high_() = Half(0);
      else if (n >= halfw)
	{
	  high_() = low_();
	  low_() = Half(0);
	  high_() <<= (n - halfw);
	}
      else
	{
	  Half temp = low_() >> (halfw - n);
	  high_() <<= n;
	  low_() <<= n;
	  high_() |= temp;
	}
      return *this;
    }

    /// Equal operator.
    constexpr bool operator == (const Self& x) const
    { return high_() == x.high_() and low_() == x.low_(); }

    /// Not-equal operator.
    constexpr bool operator != (const Self& x) const
    { return not (*this == x); }

    /// Less-than operator.
    constexpr bool operator < (const Self& x) const
    { return high_() < x.high_() or (high_() == x.high_() and low_() < x.low_()); }

    /// Greater-than operator.
    constexpr bool operator > (const Self& x) const
    { return high_() > x.high_() or (high_() == x.high_() and low_() > x.low_()); }

    /// Less-than-or-equal operator.
    constexpr bool operator <= (const Self& x) const
    { return high_() < x.high_() or (high_() == x.high_() and low_() <= x.low_()); }

    /// Greater-than-or-equal operator.
    constexpr bool operator >= (const Self& x) const
    { return high_() > x.high_() or (high_() == x.high_() and low_() >= x.low_()); }

  protected:

    /// Reference to most sig half.
    constexpr Half& low_()
    { return bits_.halfs_.low_; }

    /// Reference to least sig half.
    constexpr Half& high_()
    { return bits_.halfs_.high_; }

    /// Reference to most sig half.
    constexpr const Half& low_() const
    { return bits_.halfs_.low_; }

    /// Reference to least sig half.
    constexpr const Half& high_() const
    { return bits_.halfs_.high_; }

    /// Wide integer represented as a pair of half-width integers.
    struct Halfs
    {
      constexpr Halfs(Half high = 0, Half low = 0)
	: low_(low), high_(high)
      { }

      Half low_ = 0;
      Half high_ = 0;
    };

    /// Wide integer represented as four quarter-width integers. This
    /// is useful for implementing multiplication.
    struct Quarters
    {
      Quarter q0_;
      Quarter q1_;
      Quarter q2_;
      Quarter q3_;
    };

    /// Wide integer as a union of two half-width, or four quarter-width values, or array
    /// of uint8_t.  For example, Uint128 is a pair of uint64_t or a quad of uint32_t
    /// values.
    union Bits
    {
      constexpr Bits(Half high = 0, Half low = 0)
	: halfs_(high, low)
      { }
      Halfs halfs_;
      Quarters quarters_;
      std::array<uint8_t, sizeof(Halfs)> bytes_;
    };

    Bits bits_;
  };


  /// Wide (wider than 64-bits) unsigned integer template.
  template <typename Half_, typename Quarter_>
  class WideInt : public WideIntBase
  {
  public:

    using Half            = Half_;
    using Quarter         = Quarter_;
    using Self            = WideInt<Half, Quarter>;
    using HalfUnsigned    = MakeUnsignedT<Half>;
    using QuarterUnsigned = MakeUnsignedT<Quarter>;
    using Unsigned        = UwideInt<HalfUnsigned, QuarterUnsigned>;

    static constexpr int width()     { return 8*sizeof(Self); }
    static constexpr int halfWidth() { return 8*sizeof(Half); }

    /// Default constructor.
    constexpr WideInt() = default;

    /// Copy constructor.
    constexpr WideInt(const Self&) = default;

    /// Construct from an unsigned same-width unsigned int: Copy bits.
    constexpr WideInt(const Unsigned& x)
      : low_(static_cast<Half>(x.low())), high_(static_cast<Half>(x.high()))
    { }

    /// Construct from a half-size signed type.
    constexpr WideInt(Half x)
      : low_(x), high_(x < 0? ~Half(0) : 0)
    { }

    /// Construct from a unsgined half-size unsigned type.
    constexpr WideInt(HalfUnsigned x)
      : low_(x), high_(0)
    { }

    /// Construct from an unsigned integral type.
    template <std::unsigned_integral INT>
    constexpr WideInt(INT x)
      : low_(x), high_(0)
    { }

    /// Construct from an signed integral type.
    template <std::signed_integral INT>
    constexpr WideInt(INT x)
      : low_(x), high_(0)
    {
      if (x < 0)
	high_ = ~high_;
    }

    /// Construct from a pair of half-width signed ints.
    constexpr WideInt(Half high, Half low)
      : low_(low), high_(high)
    { }

    /// Assignment operator.
    constexpr Self& operator = (const Self&) = default;

    /// Return least sig half.
    constexpr Half low() const
    { return low_; }

    /// Return most sig half.
    constexpr Half high() const
    { return high_; }

    /// Convert to a built-in integral type.
    template <std::integral INT>
    constexpr explicit operator INT() const
    { return static_cast<INT>(low_); }

    /// Convert to boolean: Zero is false, non-zero is true.
    constexpr explicit operator bool() const
    { return high_() != 0 and low_() != 0; }

    /// Plus-equal operator.
    constexpr Self& operator += (const Self& x)
    {
      auto prevLow = static_cast<HalfUnsigned>(low_);
      low_ += x.low_;
      high_ += x.high_;
      if (HalfUnsigned(low_) < prevLow)
        high_++;
      return *this;
    }

    /// Minus-equal operator.
    constexpr Self& operator -= (const Self& x)
    {
      auto prevLow = static_cast<HalfUnsigned>(low_);
      low_ -= x.low_;
      high_ -= x.high_;
      if (HalfUnsigned(low_) > prevLow)
        high_--;
      return *this;
    }

    /// Bitwise or-equal operator.
    constexpr Self& operator |= (const Self& x)
    {
      low_ |= x.low_;
      high_ |= x.high_;
      return *this;
    }

    /// Bitwise and-equal operator.
    constexpr Self& operator &= (const Self& x)
    {
      low_ &= x.low_;
      high_ &= x.high_;
      return *this;
    }

    /// Bitwise xor-equal operator.
    constexpr Self& operator ^= (const Self& x)
    {
      low_ ^= x.low_;
      high_ ^= x.high_;
      return *this;
    }

    /// Pre-increment operator.
    constexpr Self& operator ++ ()
    { *this += 1; return *this; }

    /// Post-increment operator.
    constexpr Self operator ++ (int)
    { Self temp = *this; temp += 1; return temp; }

    /// Pre-decrement operator.
    constexpr Self& operator -- ()
    { *this -= 1; return *this; }

    /// Post-decrement operator.
    constexpr Self operator -- (int)
    { Self temp = *this; temp -= 1; return temp; }

    /// Bitwise complement operator.
    constexpr Self operator ~ () const
    { Self temp( ~high_, ~low_); return temp; }

    /// Multiply-equal operator.
    constexpr Self& operator *= (const Self& xx)
    {
      if (*this >= Self(0) and xx >= Self(0))
	{
	  Unsigned a = *this, b = xx;
	  a *= b;
	  *this = a;
	  return *this;
	}

      Self minInt(1);
      minInt <<= width() - 1;

      if (*this == minInt or xx == minInt)
	{
	  *this = Self(0);
	  return *this;
	}

      Self b = xx;
      bool neg = false;

      if (*this < Self(0))
	{
	  neg = true;
	  *this = - *this;
	}
      if (xx < Self(0))
	{
	  neg = ! neg;
	  b = -xx;
	}

      Self a = *this;
      a *= b;
      if (neg)
	a = -a;

      *this = a;

      return *this;
    }

    /// Divide-equal operator.
    constexpr Self& operator /= (const Self& xx)
    {
      if (*this == xx)
	{
	  *this = 1;
	  return *this;
	}

      Self minInt(1);
      minInt <<= width() - 1;

      if (xx == minInt)
	{
	  *this = 0;
	  return *this;
	}

      Unsigned bb = xx;
      bool neg = false;
      if (*this < Self(0))
	{
	  *this = - *this;
	  neg = true;
	}
      if (xx < Self(0))
	{
	  bb = -xx;
	  neg = ! neg;
	}

      Unsigned aa = *this;
      aa /= bb;
      *this = aa;
      if (neg)
	*this = - *this;

      return *this;
    }

    /// Remainder-equal operator.
    constexpr Self& operator %= (const Self& xx)
    {
      if (*this == xx)
	{
	  *this = 0;
	  return *this;
	}

      Self minInt(1);
      minInt <<= width() - 1;

      if (xx == minInt)
	return *this;

      Unsigned bb = xx;

      bool neg = false;
      if (*this < 0)
	{
	  *this = - *this;
	  neg = true;
	}
      if (xx < 0)
	{
	  bb = -xx;
	  neg = ! neg;
	}

      Unsigned aa = *this;
      aa %= bb;
      *this = aa;
      if (neg)
	*this = - *this;

      return *this;
    }

    /// Right-shift operator.
    constexpr Self& operator >>= (int n)
    {
      if (n == 0)
	return *this;

      bool neg = high_ < Half(0);

      constexpr int halfw = halfWidth();

      if (n >= width())
	{
	  if (neg)
	    low_ = high_ = ~Half(0);
	  else
	    low_ = high_ = Half(0);
	}
      else if (n >= halfw)
	{
	  low_ = high_;
	  if (neg)
	    low_ |= Half(1) << int(sizeof(low_)*8 - 1);
	  high_ = neg? ~Half(0) : Half(0);
	  low_ >>= (n - halfw);
	}
      else
	{
	  HalfUnsigned temp = high_ << (halfw - n);
	  high_ >>= n;
	  low_ = static_cast<Half>(HalfUnsigned(low_) >> n);
	  low_ |= static_cast<Half>(temp);
	}

      return *this;
    }

    /// Left-shift operator.
    constexpr Self& operator <<= (int n)
    {
      if (n == 0)
	return *this;

      constexpr int halfw = halfWidth();

      if (n >= width())
	low_ = high_ = Half(0);
      else if (n >= halfw)
	{
	  high_ = low_;
	  low_ = Half(0);
	  high_ <<= (n - halfw);
	}
      else
	{
	  HalfUnsigned temp = low_ >> (halfw - n);
	  high_ <<= n;
	  low_ <<= n;
	  high_ |= static_cast<Half>(temp);
	}
      return *this;
    }

    /// Equal operator.
    constexpr bool operator == (const Self& x) const
    { return high_ == x.high_ and low_ == x.low_; }

    /// Not-equal operator.
    constexpr bool operator != (const Self& x) const
    { return not (*this == x); }

    /// Less-than operator.
    constexpr bool operator < (const Self& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) < HalfUnsigned(x.low_)));
    }

    /// Greater-than operator.
    constexpr bool operator > (const Self& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) > HalfUnsigned(x.low_)));
    }

    /// Less-than-or-equal operator.
    constexpr bool operator <= (const Self& x) const
    {
      return (high_ < x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) <= HalfUnsigned(x.low_)));
    }

    /// Greater-than-or-equal operator.
    constexpr bool operator >= (const Self& x) const
    {
      return (high_ > x.high_ or
              (high_ == x.high_ and HalfUnsigned(low_) >= HalfUnsigned(x.low_)));
    }

  protected:

    /// Wide integer as a pair of two half-width values.  For example,
    /// Int128 is a pair of int64_t values.
    Half low_ = 0;
    Half high_ = 0;
  };


  /// Plus operator: Return sum of wide-int and a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator + (Left a, Right b)
  { a += b; return a; }

  /// Plus operator: Return sum of a regular-int and a wide-int
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator + (Left a, Right b)
  { b += a; return b; }

  /// Minus operator: Return difference of wide-int and a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator - (Left a, Right b)
  { a -= b; return a; }

  /// Minus operator: Return difference of a regular-int and a wide-int
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator - (Left a, Right b)
  { b -= a; return -b; }

  /// Multiply operator: Return product of wide-int and a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator * (Left a, Right b)
  { a *= b; return a; }

  /// Multiply operator: Return product of a regular-int and a wide-int
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator * (Left a, Right b)
  { b *= a; return b; }

  /// Divide operator: Return quotient of wide-int and a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator / (Left a, Right b)
  { a /= b; return a; }

  /// Divide operator: Return quotient of a regular-int and a wide-int
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator / (Left a, Right b)
  { Right c = a; c /= b; return c; }

  /// Remainder operator: Return remainder of wide-int and a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator % (Left a, Right b)
  { a %= b; return a; }

  /// Remainder operator: Return remainde of a regular-int and a wide-int
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator % (Left a, Right b)
  { Right c = a; c %= b; return c; }

  /// Unary minus operator: Return negative of a wide-int.
  template <typename Wide>
  requires std::derived_from<Wide, WideIntBase>
  constexpr Wide operator - (Wide a)
  { Wide c = Wide{0UL}; c -= a; return c; }

  /// Unary plus operator.
  template <typename Wide>
  requires std::derived_from<Wide, WideIntBase>
  constexpr Wide operator + (Wide a)
  { return a; }

  /// Right shift operator: Return a wide-int shifted right by a regular-int.
  template <typename Left>
  requires std::derived_from<Left, WideIntBase>
  constexpr Left operator >> (Left x, int n)
  { x >>= n; return x; }

  /// Left shift operator: Return a wide-int shifted left by a regular-int.
  template <typename Left>
  requires std::derived_from<Left, WideIntBase>
  constexpr Left operator << (Left x, int n)
  { x <<= n; return x; }

  /// Bitwise-or operator: Return a wide-int ored with a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator | (Left a, Right b)
  { a |= b; return a; }

  /// Bitwise-or operator: Return or of a regular-int with a wide-int.
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator | (Left a, Right b)
  { b |= a; return b; }

  /// Bitwise-and operator: Return a wide-int anded with a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator & (Left a, Right b)
  { a &= b; return a; }

  /// Bitwise-and operator: Return and of a regular-int with a wide-int.
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator & (Left a, Right b)
  { b &= a; return b; }

  /// Bitwise-xor operator: Return a wide-int xored with a wide-int/regular-int.
  template <typename Left, typename Right>
  requires std::derived_from<Left, WideIntBase> &&
    (std::is_integral_v<Right> || std::is_same_v<Left, Right>)
  constexpr Left operator ^ (Left a, Right b)
  { a ^= b; return a; }

  /// Bitwise-xor operator: Return xor of a regular-int with a wide-int.
  template <typename Left, typename Right>
  requires std::is_integral_v<Left> && std::derived_from<Right, WideIntBase>
  constexpr Right operator ^ (Left a, Right b)
  { b ^= a; return b; }

  /// Return the width in bits of the given integer type T. This is
  /// usually 8*sizeof(T).
  template <typename T>
  constexpr unsigned
  integerWidth()
  {
    if constexpr (std::is_same_v<Int128, T>)   return 128;
    if constexpr (std::is_same_v<Int256, T>)   return 256;
    if constexpr (std::is_same_v<Int512, T>)   return 512;
    if constexpr (std::is_same_v<Int1024, T>)  return 1024;
    if constexpr (std::is_same_v<Uint128, T>)  return 128;
    if constexpr (std::is_same_v<Uint256, T>)  return 256;
    if constexpr (std::is_same_v<Uint512, T>)  return 512;
    if constexpr (std::is_same_v<Uint1024, T>) return 1024;

    return 8*sizeof(T);
  }


  /// Return the integral type that is twice as wide as the given
  /// type. For example:
  ///    makeDoubleWide<uint16_t>::type
  /// yields the type
  ///    uint32_t.
  template <typename T>
  struct makeDoubleWide;

  template <> struct makeDoubleWide<uint8_t>    { using type = uint16_t; };
  template <> struct makeDoubleWide<uint16_t>   { using type = uint32_t; };
  template <> struct makeDoubleWide<uint32_t>   { using type = uint64_t; };
  template <> struct makeDoubleWide<uint64_t>   { using type = Uint128; };
  template <> struct makeDoubleWide<Uint128>    { using type = Uint256; };
  template <> struct makeDoubleWide<Uint256>    { using type = Uint512; };
  template <> struct makeDoubleWide<Uint512>    { using type = Uint1024; };

  template <> struct makeDoubleWide<int8_t>     { using type = int16_t; };
  template <> struct makeDoubleWide<int16_t>    { using type = int32_t; };
  template <> struct makeDoubleWide<int32_t>    { using type = int64_t; };
  template <> struct makeDoubleWide<int64_t>    { using type = Int128; };
  template <> struct makeDoubleWide<Int128>     { using type = Int256; };
  template <> struct makeDoubleWide<Int256>     { using type = Int512; };
  template <> struct makeDoubleWide<Int512>     { using type = Int1024; };

  template <typename T>
  concept hasDoubleWide = requires { typename makeDoubleWide<T>::type; };

  template <typename T>
  using makeDoubleWide_t = typename makeDoubleWide<T>::type;


  /// Return the integral type that is half as wide as the given
  /// type. For example:
  ///    makeDoubleWide<uint32_t>::type
  /// yields the type
  ///    uint16_t.
  template <typename>
  struct makeHalfWide;

  template <> struct makeHalfWide<uint16_t>   { using type = uint8_t; };
  template <> struct makeHalfWide<uint32_t>   { using type = uint16_t; };
  template <> struct makeHalfWide<uint64_t>   { using type = uint32_t; };
  template <> struct makeHalfWide<Uint128>    { using type = uint64_t; };
  template <> struct makeHalfWide<int16_t>    { using type = int8_t; };
  template <> struct makeHalfWide<int32_t>    { using type = int16_t; };
  template <> struct makeHalfWide<int64_t>    { using type = int32_t; };
  template <> struct makeHalfWide<Int128>     { using type = int64_t; };

  template <std::derived_from<WideIntBase> T>
  struct makeHalfWide<T>
  { using type = typename T::Half; };

  template <typename T>
  concept hasHalfWide = requires { typename makeHalfWide<T>::type; };


  /// Converts the given value into an array containing it two halves
  /// in order of significance (i.e. LS first, MS last)
  template <hasHalfWide Whole>
  constexpr auto toHalves(const Whole& w)
    -> std::array<typename makeHalfWide<Whole>::type, 2>
  {
    using RetType = decltype(toHalves(w));

    if constexpr (std::is_base_of_v<WideIntBase, Whole>)
     return std::array{ w.low(), w.high() };

    RetType result = std::bit_cast<RetType>(w);
    if constexpr (std::endian::native == std::endian::big)
      std::reverse(result.begin(), result.end());

    return result;
  }


  /// Converts the given value into an array containing it four quarters
  /// in order of significance (i.e. LS first, MS last)
  template <typename Whole>
  requires hasHalfWide<Whole> and hasHalfWide<typename makeHalfWide<Whole>::type>
  constexpr auto toQuarters(const Whole& w)
    -> std::array<typename makeHalfWide<typename makeHalfWide<Whole>::type>::type, 4>
  {
    using RetType = decltype(toQuarters(w));

    if constexpr (std::is_base_of_v<WideIntBase, Whole>)
      {
        auto&& [h0, h1] = toHalves(w);
        auto&& [q0, q1] = toHalves(h0);
        auto&& [q2, q3] = toHalves(h1);
        return std::array{q0, q1, q2, q3};
      }

    RetType result = std::bit_cast<RetType>(w);
    if constexpr (std::endian::native == std::endian::big)
      std::reverse(result.begin(), result.end());

    return result;
  }


  /// Converts the given two half values into a single value.  Halves are provided
  /// in order of significance (i.e. LS first, MS last)
  template <hasDoubleWide Half>
  constexpr auto fromHalves(Half h0, Half h1)
    -> makeDoubleWide_t<Half>
  {
    using RetType = decltype(fromHalves(h0, h1));

    if constexpr (std::is_base_of_v<WideIntBase, RetType>)
      return RetType{ h1, h0 }; // Upper half is provided first

    auto halves = std::array{ h0, h1 };
    if constexpr (std::endian::native == std::endian::big)
      std::reverse(halves.begin(), halves.end());

    return std::bit_cast<RetType>(halves);
  }


  /// Converts the given four quarter values into a single value.  Quarters are provided
  /// in order of significance (i.e. LS first, MS last)
  template <typename Quarter>
  requires hasDoubleWide<Quarter> and hasDoubleWide<makeDoubleWide_t<Quarter>>
  constexpr auto fromQuarters(Quarter q0, Quarter q1, Quarter q2, Quarter q3)
    -> makeDoubleWide_t<makeDoubleWide_t<Quarter>>
  {
    using RetType = decltype(fromQuarters(q0, q1, q2, q3));

    if constexpr (std::is_base_of_v<WideIntBase, RetType>)
      {
        // Upper half is provided first
        auto h0 = fromHalves(q0, q1);
        auto h1 = fromHalves(q2, q3);
        return fromHalves(h0, h1);
      }

    auto quarters = std::array{ q0, q1, q2, q3 };
    if constexpr (std::endian::native == std::endian::big)
      std::reverse(quarters.begin(), quarters.end());

    return std::bit_cast<RetType>(quarters);
  }

}

