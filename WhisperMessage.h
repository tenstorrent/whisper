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

#include <array>
#include <span>
#include <cstdint>


enum WhisperMessageType { Peek, Poke, Step, Until, Change, ChangeCount,
                          Quit, Invalid, Reset, Exception, EnterDebug,
                          ExitDebug, LoadFinished, CancelDiv, CancelLr,
                          DumpMemory, McmRead, McmInsert, McmWrite,
			  PageTableWalk, Translate, CheckInterrupt,
                          SeiPin };

// Be careful changing this: test-bench file (defines.svh) needs to be
// updated.
enum WhisperExceptionType { InstAccessFault, DataAccessFault,
                            ImpreciseStoreFault, ImpreciseLoadFault,
                            PreciseStoreFault, PreciseLoadFault, 
                            NonMaskableInterrupt };

/// Resource identifiers for peek special.
enum WhisperSpecialResource { PrivMode, PrevPrivMode, FpFlags, Trap, DeferredInterrupts };


/// Structure used to communicate with the whisper program using
/// sockets.  When a ChangeCount message is returned by whisper (as a
/// reply to a Step or a ChangeCount request), the address is set to
/// the program-counter of the last executed instruction, the resource
/// is set to the opcode of that instruction and the value is set to
/// the number of change records generated by that instruction.
struct WhisperMessage
{
  WhisperMessage(uint32_t hart = 0, WhisperMessageType type = Invalid,
		 uint32_t resource = 0, uint64_t address = 0,
		 uint64_t value = 0, uint32_t size = 0, uint64_t instrTag = 0,
		 uint64_t time = 0)
    : hart(hart), type(type), resource(resource), size(size),
      instrTag(instrTag), time(time), address(address), value(value)
  {
    buffer.fill(0);
    tag.fill(0);
  }

  /// Unpack socket message into the returned WhisperMessage object.
  static WhisperMessage deserializeFrom(std::span<char> buffer);

  /// Serialize the current WhisperMessage into the given buffer.
  /// Return the number of bytes written into buffer.
  size_t serializeTo(std::span<char> buffer) const;

  uint32_t hart;
  uint32_t type;
  uint32_t resource;
  uint32_t size;
  uint32_t flags = 0;
  uint64_t instrTag; // Instruction tag.
  uint64_t time;     // Time stamp.
  uint64_t address;
  uint64_t value;
  std::array<char, 128> buffer;
  std::array<char, 20>  tag;
};
