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

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <vector>
#include <type_traits>
#include <unordered_map>
#include <string>
#include <cassert>

namespace WdRiscv
{

    /// Symbolic names of the integer registers.
    enum FpRegNumber
      {
	RegF0   = 0,
	RegF1   = 1,
	RegF2   = 2,
	RegF3   = 3,
	RegF4   = 4,
	RegF5   = 5,
	RegF6   = 6,
	RegF7   = 7,
	RegF8   = 8,
	RegF9   = 9,
	RegF10  = 10,
	RegF11  = 11,
	RegF12  = 12,
	RegF13  = 13,
	RegF14  = 14,
	RegF15  = 15,
	RegF16  = 16,
	RegF17  = 17,
	RegF18  = 18,
	RegF19  = 19,
	RegF20  = 20,
	RegF21  = 21,
	RegF22  = 22,
	RegF23  = 23,
	RegF24  = 24,
	RegF25  = 25,
	RegF26  = 26,
	RegF27  = 27,
	RegF28  = 28,
	RegF29  = 29,
	RegF30  = 30,
	RegF31  = 31,
	RegFt0  = RegF0,
	RegFt1  = RegF1,
	RegFt2  = RegF2,
	RegFt3  = RegF3,
	RegFt4  = RegF4,
	RegFt5  = RegF5,
	RegFt6  = RegF6,
	RegFt7  = RegF7,
	RegFs0  = RegF8,
	RegFs1  = RegF9,
	RegFa0  = RegF10,
	RegFa1  = RegF11,
	RegFa2  = RegF12,
	RegFa3  = RegF13,
	RegFa4  = RegF14,
	RegFa5  = RegF15,
	RegFa6  = RegF16,
	RegFa7  = RegF17,
	RegFs2  = RegF18,
	RegFs3  = RegF19,
	RegFs4  = RegF20,
	RegFs5  = RegF21,
	RegFs6  = RegF22,
	RegFs7  = RegF23,
	RegFs8  = RegF24,
	RegFs9  = RegF25,
	RegFs10 = RegF26,
	RegFs11 = RegF27,
	RegFt8  = RegF28,
	RegFt9  = RegF29,
	RegFt10 = RegF30,
	RegFt11 = RegF31
      };


  /// RISCV floating point rounding modes.
  enum class RoundingMode : uint32_t
    {
      NearestEven,     // Round to nearest, ties to even
      Zero,            // Round towards zero.
      Down,            // Round down (towards negative infinity)
      Up,              // Round up (towards positive infinity)
      NearestMax,      // Round to nearest, ties to max magnitude
      Invalid1,
      Invalid2,
      Dynamic,
      FcsrMask = 0xe0, // Mask of mode-bits in FCSR.
      FcsrShift = 5    // Index of least-significant mode bit in FCSR.
    };


  /// RISCV floating point exception flags.
  enum class FpFlags : uint32_t
    {
      None = 0,
      Inexact = 1,
      Underflow = 2,
      Overflow = 4,
      DivByZero = 8,
      Invalid = 16,
      FcsrMask = 0x1f   // Mask of flag-bits in the FCSR.
    };


  /// RISCV values used to synthesize the results of the classify
  /// instructions (e.g. flcass.s).
  enum class FpClassifyMasks : uint32_t
    {
     NegInfinity  = 1,       // bit 0
     NegNormal    = 1 << 1,  // bit 1
     NegSubnormal = 1 << 2,  // bit 2
     NegZero      = 1 << 3,  // bit 3
     PosZero      = 1 << 4,  // bit 4
     PosSubnormal = 1 << 5,  // bit 5
     PosNormal    = 1 << 6,  // bit 6
     PosInfinity  = 1 << 7,  // bit 7
     SignalingNan = 1 << 8,  // bit 8
     QuietNan     = 1 << 9   // bit 9
    };

  /// Values of FS field in mstatus.
  enum class FpFs : uint32_t
    {
     Off = 0,
     Initial = 1,
     Clean = 2,
     Dirty = 3
    };

  template <typename URV>
  class Hart;


  /// Unsigned-float union: reinterpret bits as uint32_t or float
  union Uint32FloatUnion
  {
    Uint32FloatUnion(uint32_t u) : u(u)
    { }

    Uint32FloatUnion(float f) : f(f)
    { }

    uint32_t u = 0;
    float f;
  };


  class Float16
  {
  public:

    /// Construct a Float16 from a 16-bit integer by reinterpreting
    /// the bits as a float (no conversion from integer to float).
    explicit Float16(uint16_t x = 0)
      : i16(x)
    { }

    /// Construct a Float16 from a float by dropping the leat
    /// significat 16 bits of the float.
    explicit Float16(float x)
    {
      Uint32FloatUnion uf{x};
      i16 = uf.u >> 16;
    }

    /// Return true if this Float16 is equal to the given Float16
    /// according to the floating point rules (-0 is equal to +0).
    bool operator==(const Float16& x) const
    { return this->toFloat() == x.toFloat(); }

    /// Return the bits of the Float16 as uint16_t (no conversion from
    /// float to itneger).
    uint16_t bits() const
    { return i16; }

    /// Convert this Float16 to a float by shifting left by 16.
    float toFloat() const
    {
      uint32_t word = uint32_t(i16) << 16;
      Uint32FloatUnion uf{word};
      return uf.f;
    }

    /// Return the sign bit of this Float16 in the least significant
    /// bit of the result.
    unsigned signBit() const
    { return i16 >> 15; }

    /// Return true if this number is subnormal.
    bool isSubnormal() const
    {
      // Exponent bits (bits 7 to 14) must be zero
      return (i16 & 0x7f80) == 0;
    }

    /// Return copy of this Float16 with cleared  mantissa (bits 0 to 6).
    Float16 clearMantissa() const
    { Float16 x{*this}; x.i16 &= 0xff80; return x; }

    /// Return the negative of this Float16.
    Float16 negate() const
    { Float16 x{*this}; x.i16 ^= 0x8000; return x; }

    /// Return true if this number encodes not-a-number.
    bool isNan() const
    { return std::isnan(this->toFloat()); }

    /// Return true if this number encodes a signaling not-a-number.
    bool isSnan() const
    { return ((i16 >> 6) & 1) == 0; } // Upper bit of significand (bit 6) is zero.

    /// Return a Float16 with magnitude of x and sign of y.
    static Float16 copySign(Float16 x, Float16 y)
    { x.i16 = (x.i16 << 1) >> 1;  x.i16 |= (y.i16 >> 15 << 15); return x; }

    /// Return the quiet NAN Float16 number.
    static Float16 quietNan()
    { return Float16{std::numeric_limits<float>::quiet_NaN()}; }

  private:

    uint16_t i16 = 0;
  } __attribute__((packed));


  /// Model a RISCV floating point register file. We use double precision
  /// representation for each register and nan-boxing for single precision
  /// and float16 values.
  class FpRegs
  {
  public:

    friend class Hart<uint32_t>;
    friend class Hart<uint64_t>;

    /// Constructor: Define a register file with the given number of
    /// registers. All registers initialized to zero.
    FpRegs(unsigned registerCount);

    /// Destructor.
    ~FpRegs()
    { regs_.clear(); }
    
    /// Return value of ith register.
    double readDouble(unsigned i) const
    { assert(flen_ >= 64); return regs_[i]; }

    /// Return the bit pattern of the ith register as an unsigned
    /// integer. If the register contains a nan-boxed value, return
    /// that value without the box.
    uint64_t readBitsUnboxed(unsigned i) const
    {
      FpUnion u{regs_.at(i)};
      if (hasHalf_ and u.isBoxedHalf())
	return (u.i64 << 48) >> 48;

      if (hasSingle_ and u.isBoxedSingle())
	return (u.i64 << 32) >> 32;

      return u.i64;
    }

    /// Return true if given bit pattern represents a nan-boxed
    /// single precision value.
    bool isBoxedSingle(uint64_t value) const
    {
      FpUnion u{value};
      return u.isBoxedSingle();
    }

    /// Return true if given bite pattern represents a nan-boxed
    /// half precision value.
    bool isBoxedHalf(uint64_t value) const
    {
      FpUnion u{value};
      return u.isBoxedHalf();
    }

    /// Return the bit pattern of the ith register as an unsigned
    /// integer. If the register contains a nan-boxed value, do not
    /// unbox it (return the 64-bit NaN).
    uint64_t readBitsRaw(unsigned i) const
    {
      FpUnion u {regs_.at(i)};
      return u.i64 & mask_;
    }

    /// Set FP register i to the given value.
    void pokeBits(unsigned i, uint64_t val)
    {
      FpUnion fpu(val);
      regs_.at(i) = fpu.dp;
    }

    /// Set value of ith register to the given value.
    void writeDouble(unsigned i, double value)
    {
      assert(flen_ >= 64);
      originalValue_ = regs_.at(i);
      regs_.at(i) = value;
      lastWrittenReg_ = i;
    }

    /// Read a single precision floating point number from the ith
    /// register.  If the register width is greater than 32 bits, this
    /// will recover the least significant 32 bits (it assumes that
    /// the number in the register is NAN-boxed). If the register
    /// width is 32-bit, this will simply recover the number in it.
    float readSingle(unsigned i) const;

    /// Write a single precision number into the ith register. NAN-box
    /// the number if the register is 64-bit wide.
    void writeSingle(unsigned i, float x);

    Float16 readHalf(unsigned i) const;
    void writeHalf(unsigned i, Float16 x);

    /// Return the count of registers in this register file.
    size_t size() const
    { return regs_.size(); }

    /// Set ix to the number of the register corresponding to the
    /// given name returning true on success and false if no such
    /// register.  For example, if name is "f2" then ix will be set to
    /// 2. If name is "fa0" then ix will be set to 10.
    bool findReg(const std::string& name, unsigned& ix) const;

    /// Return the name of the given register.
    std::string regName(unsigned i, bool abiNames = false) const
    {
      if (abiNames)
	{
	  if (i < numberToAbiName_.size())
	    return numberToAbiName_[i];
	  return std::string("f?");
	}
      if (i < numberToName_.size())
	return numberToName_[i];
      return std::string("f?");
    }

  protected:

    void reset(bool hasHalf, bool hasSingle, bool hasDouble);

    /// Clear the number denoting the last written register.
    void clearLastWrittenReg()
    { lastWrittenReg_ = -1; lastFpFlags_ = 0; }

    /// Return the number of the last written register or -1 if no register has
    /// been written since the last clearLastWrittenReg.
    int getLastWrittenReg() const
    { return lastWrittenReg_; }

    /// Set regIx and regValue to the index and previous value (before
    /// write) of the last written register returning true on success
    /// and false if no integer was written by the last executed
    /// instruction (in which case regIx and regVal are left
    /// unmodified).
    bool getLastWrittenReg(unsigned& regIx, uint64_t& regValue) const
    {
      if (lastWrittenReg_ < 0) return false;
      regIx = lastWrittenReg_;

      // Copy bits of last written value inot regValue
      union
      {
        double d;
        uint64_t u;
      } tmp;
      tmp.d = originalValue_;
      regValue = tmp.u;

      return true;
    }

    /// Return the incremental floating point flag values resulting from
    /// the execution of the last instruction. Return 0 if last instructions
    /// is not an FP instruction or if it does not set any of the FP flags.
    unsigned getLastFpFlags() const
    { return lastFpFlags_; }

    /// Set the incremental FP flags produced by the last executed FP
    /// instruction.
    void setLastFpFlags(unsigned flags)
    { lastFpFlags_ = flags; }

    bool setFlen(unsigned length)
    {
      if (length != 32 and length != 64)
        return false;
      flen_ = length;
      mask_ = ~uint64_t(0) >> (64 - length);
      return true;
    }

  private:

    // Union of double and single precision numbers used for NAN boxing.
    union FpUnion
    {
      FpUnion(double x)   : dp(x)  { }
      FpUnion(uint64_t x) : i64(x) { }
      FpUnion(float x)    : sp(x)  { i64 |= ~uint64_t(0) << 32; }
      FpUnion(Float16 x)  : hp(x)  { i64 |= ~uint64_t(0) << 16; }

      /// Return true if bit pattern corresponds to a nan-boxed single
      /// precision float.
      bool isBoxedSingle() const
      { return (i64 >> 32) == ~uint32_t(0); }

      /// Return true if bit pattern corresponds to a nan-boxed half
      /// precision (16-bit) float.
      bool isBoxedHalf() const
      { return (i64 >> 16) == (~uint64_t(0) >> 16); }

      float    sp;
      Float16  hp;
      double   dp;
      uint64_t i64;
    };
	
  private:

    std::vector<double> regs_;
    bool hasHalf_ = false;         // True if half (16-bit) precision enabled.
    bool hasSingle_ = false;       // True if F extension enabled.
    bool hasDouble_ = false;       // True if D extension enabled.
    int lastWrittenReg_ = -1;      // Register accessed in most recent write.
    unsigned lastFpFlags_ = 0;
    double originalValue_ = 0;     // Original value of last written reg.
    unsigned flen_ = 64;           // Floating point register width.
    uint64_t mask_ = ~uint64_t(0);
    std::unordered_map<std::string, FpRegNumber> nameToNumber_;
    std::vector<std::string> numberToAbiName_;
    std::vector<std::string> numberToName_;
  };


  inline
  float
  FpRegs::readSingle(unsigned i) const
  {
    assert(flen_ >= 32);

    FpUnion u{regs_.at(i)};
    if (flen_ == 32 or u.isBoxedSingle())
      return u.sp;

    // Not properly boxed single, replace with NaN.
    return std::numeric_limits<float>::quiet_NaN();
  }


  inline
  void
  FpRegs::writeSingle(unsigned i, float x)
  {
    assert(flen_ >= 32);
    originalValue_ = regs_.at(i);

    FpUnion u{x};
    regs_.at(i) = u.dp;
    lastWrittenReg_ = i;
  }


  inline
  Float16
  FpRegs::readHalf(unsigned i) const
  {
    assert(flen_ >= 16);

    FpUnion u{regs_.at(i)};
    if (flen_ == 16 or u.isBoxedHalf())
      return u.hp;

    return Float16::quietNan();
  }


  inline
  void
  FpRegs::writeHalf(unsigned i, Float16 x)
  {
    assert(flen_ >= 16);

    FpUnion u{x};
    writeDouble(i, u.dp);
  }

}
