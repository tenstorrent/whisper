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

#include <cstdint>
#include <cstring>
#include <vector>
#include "VecRegs.hpp"

namespace WdRiscv
{

  using MatStatus = FpStatus;  // Values for the MSTATUS.MS field.

  /// Model the processor state required by the Zvt extension. This includes the tile
  /// state and the values defined byte the MTYPE CSR.
  class ZvtState
  {
  public:

    ZvtState();

    ~ZvtState();

    /// Configure the state. The generic tile is a TExTE matrix with an element size of 32
    /// bit. The tile state (all tiles) occupies 16*TE*TE bytes. TE must be a power of 2
    /// and at least 4.
    void config(unsigned te)
    {
      te_ = te;
      data_.assign(size_t(16) * te * te, 0);
    }

    /// Return the effective tile dimension corresponding to the given tile element width
    /// (in bits).
    unsigned ete(unsigned tewBits) const
    { return tewBits < 64 ? te_ : te_ / 2; }

    /// Return the number of tiles corresponding to the given tile element width (in
    /// bits).
    static unsigned tileCount(unsigned tewBits)
    {
      switch (tewBits)
        {
        case 8:  return 16;
        case 16: return 8;
        case 32: return 4;
        case 64: return 8;
        default: return 0;
        }
    }

    /// Decode the given tile subset specifier (TSS) for the given tile element width
    /// (in bits) into a tile number, a pattern (0=row, 1=col), and an index (row or col
    /// index). Reserved values are handled the way the spec expects most
    /// implementations to: pattern is taken mod 2 and index is taken mod ETE.
    void decodeTss(uint64_t tss, unsigned tewBits, unsigned& tile, unsigned& pattern,
                   unsigned& index) const
    {
      tile = (tss >> 27) & 0xf;
      unsigned count = tileCount(tewBits);
      if (count)
        tile &= ~((16 / count) - 1);  // Ignore log2(16/NTiles) least sig bits.
      pattern = ((tss >> 24) & 7) % 2;
      index = tss & 0xffffff;
      unsigned e = ete(tewBits);
      if (e)
        index %= e;
    }

    /// Return the byte offset in the tile state of the element at the given row and
    /// column of the given tile for the given tile element width (in bits). This is the
    /// "punning" algorithm of the spec.
    uint64_t elemOffset(unsigned tile, unsigned row, unsigned col, unsigned tewBits) const
    {
      const uint64_t te = te_;
      uint64_t ptile = 0, minor = 0, major = 0;

      switch (tewBits)
        {
        case 8:
          ptile = tile;
          minor = (row % 4) * 4 + (col % 4);
          major = (row / 4) * (te / 4) + (col / 4);
          break;

        case 16:
          ptile = tile + ((row & 2) >> 1);
          minor = (row % 2) * 4 + (col % 2) * 2 + ((col / 2) % 2) * 8;
          major = (row / 4) * (te / 4) + (col / 4);
          break;

        case 32:
          ptile = tile + 2 * (row / (te / 2)) + ((col & 2) >> 1);
          minor = (row % 2) * 8 + (col % 2) * 4;
          major = ((row / 2) % (te / 4)) * (te / 4) + (col / 4);
          break;

        case 64:
          ptile = tile + (row / (te / 4));
          minor = (col % 2) * 8;
          major = (row % (te / 4)) * (te / 4) + (col / 2);
          break;

        default:
          return 0;
        }

      return ptile * te * te + major * 16 + minor;
    }

    /// Return the row and column of the ith element of the tile subset defined by the
    /// given pattern (0=row, 1=col) and index.
    static void subsetElemRowCol(unsigned pattern, unsigned index, unsigned ix,
                                 unsigned& row, unsigned& col)
    {
      if (pattern == 0)
        { row = index; col = ix; }
      else
        { row = ix; col = index; }
    }

    /// Read the element at the given row/col of the given tile. Element type determines
    /// the tile element width.  Return true on success and false if out of bounds.
    template <typename T>
    bool readElem(unsigned tile, unsigned row, unsigned col, T& value) const
    {
      uint64_t offset = elemOffset(tile, row, col, sizeof(T)*8);
      if (offset + sizeof(T) > data_.size())
        return false;
      std::memcpy(&value, data_.data() + offset, sizeof(T));
      return true;
    }

    /// Write the element at the given row/col of the given tile. Element type determines
    /// the tile element width.  Return true on success and false if out of bounds.
    template <typename T>
    bool writeElem(unsigned tile, unsigned row, unsigned col, T value)
    {
      uint64_t offset = elemOffset(tile, row, col, sizeof(T)*8);
      if (offset + sizeof(T) > data_.size())
        return false;
      std::memcpy(data_.data() + offset, &value, sizeof(T));
      return true;
    }

    /// Return the tile state as a linearized buffer of 16*TE*TE bytes.
    const std::vector<uint8_t>& data() const
    { return data_; }

    /// Return the tile dimension TE.
    unsigned te() const
    { return te_; }
    
    /// Return the effective tile dimension corresponding to the current TEW.
    unsigned effTileDim() const
    { return ete(tew()); }

    /// Return the current tile element width in bits: SEW*TWIDEN. Return 0 if the
    /// matrix unit is not configured (MTYPE.MTWIDEN is 0). This is derived from the
    /// cached SEW and MTWIDEN so it is never stale regardless of the order in which
    /// VTYPE and MTYPE are updated.
    unsigned tew() const
    { return sewBits_ * twidenFromMtwiden(mtwiden_); }

    /// Return the current tile element width in bits.
    unsigned tileElemBits() const
    { return tew(); }

    /// Set the tile m dimension.
    void setTm(unsigned tm)
    { tm_ = tm; }

    /// Return the tile m dimension.
    unsigned tm() const
    { return tm_; }

    /// Set the tile k dimension.
    void setTk(unsigned tk)
    { tk_ = tk; }

    /// Return the tile k dumension.
    unsigned tk() const
    { return tk_; }

    /// Set the cached SEW (in bits) value. This must be updated whenever VTYPE changes.
    void setSewBits(unsigned sewBits)
    { sewBits_ = sewBits; }

    /// Return the cached SEW (in bits) value.
    unsigned sewBits() const
    { return sewBits_; }

    /// Set the cached MTYPE.MTWIDEN value.
    void setMtwiden(unsigned val)
    { mtwiden_ = val; }

    /// Return the cached MTYPE.MTWIDEN value.
    unsigned mtwiden() const
    { return mtwiden_; }

    /// Return the TWIDEN value corresponding to the given MTYPE.MTWIDEN field value:
    /// 0 (not configured), 1, 2, or 4.
    static unsigned twidenFromMtwiden(unsigned mtwiden)
    { return mtwiden == 0 ? 0 : 1u << (mtwiden - 1); }

    /// Set kmx to the KMAX value corresponding to the given SEW (in bits) and TWIDEN
    /// (1, 2, or 4; not the encoded MTWIDEN field) returning true if the combination is
    /// supported. Return false otherwise leaving kmx unmodified. TEW is SEW*TWIDEN.
    static bool kmax(unsigned sewBits, unsigned twiden, unsigned& kmx)
    {
      switch (sewBits * twiden)   // TEW
        {
        case 8:
          if (sewBits == 8  and twiden == 1) { kmx = 4; return true; }
          return false;

        case 16:
          if (sewBits == 8  and twiden == 2) { kmx = 4; return true; }
          if (sewBits == 16 and twiden == 1) { kmx = 2; return true; }
          return false;

        case 32:
          if (sewBits == 8  and twiden == 4) { kmx = 4; return true; }
          if (sewBits == 16 and twiden == 2) { kmx = 2; return true; }
          if (sewBits == 32 and twiden == 1) { kmx = 1; return true; }
          return false;

        case 64:
          if (sewBits == 16 and twiden == 4) { kmx = 2; return true; }
          if (sewBits == 32 and twiden == 2) { kmx = 1; return true; }
          if (sewBits == 64 and twiden == 1) { kmx = 1; return true; }
          return false;

        default:
          return false;
        }
    }

    /// Set kmx to the KMAX value corresponding to the given SEW (in bits) and the
    /// current (cached) MTYPE.MTWIDEN. Return false if the combination is not supported.
    bool kmax(unsigned sewBits, unsigned& kmx) const
    { return kmax(sewBits, twidenFromMtwiden(mtwiden_), kmx); }

  protected:

    /// Return the offset in the data_ array to the element defined by the given tile
    /// index, row, and col and by the given element width. This is the function
    /// "punning" in the spec.
    unsigned elemOffset(unsigned te, unsigned tile, unsigned row, unsigned col,
                        unsigned tew) const;

  private:

    unsigned mtwiden_ = 0;  // Cached value of the MTYPE.MTWIDEN.

    unsigned sewBits_ = 8;  // Cached VTYPE.SEW in bits (TEW = SEW*TWIDEN).
    unsigned te_ = 0;       // Tile dimension.
    unsigned tm_ = 0;
    unsigned tk_ = 0;

    std::vector<uint8_t> data_;  // Tile state: 16*TE*TE bytes.
  };
}
