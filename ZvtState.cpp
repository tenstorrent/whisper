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

#include <iostream>
#include <cmath>
#include <cstring>
#include "ZvtState.hpp"


using namespace WdRiscv;


ZvtState::ZvtState()
{
}


ZvtState::~ZvtState()
{
}


unsigned
ZvtState::elemOffset(unsigned te, unsigned tile, unsigned row, unsigned col,
                     unsigned tew) const
{
  unsigned ptile = 0, minorOffset = 0, majorOffset = 0;

  switch (tew)
    {
    case 8: {
      ptile = minorOffset;
      tile = (row % 4) * 4 + (col % 4);
      majorOffset = (row / 4) * (te / 4) + (col / 4);
      break;
    }
    case 16: {
      ptile = tile + ((row & 2) >> 1);
      minorOffset = (row % 2) * 4 + (col % 2) * 2 + ((col / 2) % 2) * 8;
      majorOffset = (row / 4) * (te / 4) + (col / 4);
      break;
    }
    case 32: {
      ptile = tile + 2 * (row / (te / 2)) + ((col & 2) >> 1);
      minorOffset = (row % 2) * 8 + (col % 2) * 4;
      majorOffset = ((row / 2) % (te / 4)) * (te / 4) + (col / 4);
      break;
    }
    case 64: {
      ptile = tile + (row / (te / 4));
      minorOffset = (col % 2) * 8;
      majorOffset = (row % (te / 4)) * (te / 4) + (col / 2);
      break;
    }
    default:
      assert(0);
      return 0;
    }

  unsigned offset = (ptile * te * te) + (majorOffset * 16) + minorOffset;

  return offset;
}
