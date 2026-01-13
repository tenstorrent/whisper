#pragma once

namespace WdRiscv
{
  /// Address translation modes.
  enum class SvMode : unsigned { Bare = 0, Sv32 = 1, Sv39 = 8, Sv48 = 9, Sv57 = 10,
                                 Sv64 = 11, Limit_ = 12 };
}
