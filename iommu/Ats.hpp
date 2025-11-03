// Copyright 2024 Tenstorrent Corporation.
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

namespace TT_IOMMU
{
  // Forward declarations
  struct AtsInvalCommand;
  struct AtsPrgrCommand;
  struct IodirCommand;
  struct IofenceCCommand;
  struct IotinvalCommand;

  // ATS command functions as defined in section 4.1.4
  enum class AtsFunc : uint32_t
  {
    INVAL = 0,  // Send ATS "Invalidation Request" messages
    PRGR  = 1   // Send ATS "Page Request Group Response" messages
    // 2-7 reserved for future standard use
  };

  // IODIR command functions
  enum class IodirFunc : uint32_t
  {
    INVAL_DDT = 0,
    INVAL_PDT = 1,
  };

  // IOFENCE command functions
  enum class IofenceFunc : uint32_t
  {
    C = 0  // Command queue fence
    // 1-15 reserved for future standard use
  };

  // IOTINVAL command functions for page table cache invalidation
  enum class IotinvalFunc : uint32_t
  {
    VMA = 0,   // Invalidate first-stage page table cache entries (IOTINVAL.VMA)
    GVMA = 1   // Invalidate second-stage page table cache entries (IOTINVAL.GVMA)
    // 2-15 reserved for future standard use
  };

  // Command opcodes
  enum class CommandOpcode : uint32_t
  {
    // 0 reserved
    IOTINVAL = 1, // IOMMU Translation Table Cache invalidation commands
    IOFENCE = 2,  // IOMMU fence commands
    IODIR = 3,    // IOMMU directory cache invalidation commands
    ATS = 4       // IOMMU PCIe ATS commands
  };

  // For backward compatibility
  using AtsOpcode = CommandOpcode;

  /// Ats command as 2 double words.
  struct AtsCommandData
  {
    uint64_t dw0 = 0;
    uint64_t dw1 = 0;
  };

  // ATS.INVAL command structure
  struct AtsInvalCommand
  {
    AtsOpcode     opcode    : 7{AtsOpcode::ATS};  // Command opcode (bits 0-6)
    AtsFunc       func3     : 3{AtsFunc::INVAL};  // Function (bits 7-9)
    uint64_t      reserved0 : 2{0};  // Reserved bits (bits 10-11)
    uint64_t      PID       : 20 = 0UL; // PASID (bits 12-31)
    uint64_t      PV        : 1 = 0UL;  // PASID Valid (bit 32)
    uint64_t      DSV       : 1 = 0UL;  // Destination Segment Valid (bit 33)
    uint64_t      reserved1 : 6{0};  // Reserved bits (bits 34-39)
    uint64_t      RID       : 16 = 0UL; // PCIe Routing ID (bits 40-55)
    uint64_t      DSEG      : 8 = 0UL;  // Destination Segment Number (bits 56-63)
    uint64_t      G         : 1 = 0UL;
    uint64_t      zero      : 10{0};
    uint64_t      s         : 1 = 0UL;
    uint64_t      address   : 52 = 0UL;

    // Constructor to initialize opcode and func3
    AtsInvalCommand() = default;
  };

  // ATS.PRGR command structure
  struct AtsPrgrCommand
  {
    AtsOpcode     opcode    : 7{AtsOpcode::ATS};  // Command opcode (bits 0-6)
    AtsFunc       func3     : 3{AtsFunc::PRGR};  // Function (bits 7-9)
    uint64_t      reserved0 : 2{0};  // Reserved bits (bits 10-11)
    uint64_t      PID       : 20 = 0UL; // PASID (bits 12-31)
    uint64_t      PV        : 1 = 0UL;  // PASID Valid (bit 32)
    uint64_t      DSV       : 1 = 0UL;  // Destination Segment Valid (bit 33)
    uint64_t      reserved1 : 6{0};  // Reserved bits (bits 34-39)
    uint64_t      RID       : 16 = 0UL; // PCIe Routing ID (bits 40-55)
    uint64_t      DSEG      : 8 = 0UL;  // Destination Segment Number (bits 56-63)
    uint64_t      zero0         : 32{0};
    uint64_t      prgi          : 9 = 0UL;
    uint64_t      zero1         : 3{0};
    uint64_t      responsecode  : 4 = 0UL;
    uint64_t      destId        : 16{0};

    // Constructor to initialize opcode and func3
    AtsPrgrCommand() = default;
  };

  // IODIR command structure
  struct IodirCommand
  {
    CommandOpcode   opcode      : 7  = CommandOpcode::IODIR;
    IodirFunc       func3       : 3  = IodirFunc::INVAL_DDT;
    uint64_t        reserved0   : 2  = 0;  //
    uint64_t        PID         : 20 = 0;  //
    uint64_t        reserved1   : 1  = 0;  //
    uint64_t        DV          : 1  = 0;  //
    uint64_t        reserved2   : 6  = 0;  //
    uint64_t        DID         : 24 = 0;  //

    IodirCommand() = default;
  };

  // IOFENCE.C command structure based on specification
  struct IofenceCCommand
  {
    CommandOpcode   opcode      : 7{CommandOpcode::IOFENCE};  // Command opcode (bits 0-6) = IOFENCE (0x2)
    IofenceFunc     func3       : 3{IofenceFunc::C};  // Function (bits 7-9) = C (0x0)
    uint64_t        AV          : 1{0};  // Address Valid (bit 10)
    uint64_t        WSI         : 1{0};  // Wire-Signaled-Interrupt (bit 11)
    uint64_t        PR          : 1{0};  // Previous Reads (bit 12)
    uint64_t        PW          : 1{0};  // Previous Writes (bit 13)
    uint64_t        reserved0   : 18{0}; // Reserved bits (bits 14-31)
    uint64_t        DATA        : 32{0}; // Data to write (bits 32-63)
    uint64_t        ADDR        : 62{0}; // Address[63:2] for 4-byte aligned writes (bits 64-125)
    uint64_t        reserved1   : 2 = 0UL;

    // Constructor to initialize opcode and func3
    IofenceCCommand() = default;
  };

  // IOTINVAL command structure for page table cache invalidation (both VMA and GVMA)
  struct IotinvalCommand
  {
    CommandOpcode   opcode      : 7{CommandOpcode::IOTINVAL};  // Command opcode (bits 0-6) = IOTINVAL (0x1)
    IotinvalFunc    func3       : 3{IotinvalFunc::VMA};  // Function (bits 7-9) = VMA (0x0) or GVMA (0x1)
    uint64_t        AV          : 1{0};  // Address Valid (bit 10)
    uint64_t        reserved0   : 1{0};  // Reserved bit (bit 11)
    uint64_t        PSCID       : 20{0}; // Process Soft-Context ID (bits 12-31)
    uint64_t        PSCV        : 1{0};  // PSCID Valid (bit 32) - must be 0 for GVMA
    uint64_t        GV          : 1{0};  // GSCID Valid (bit 33)
    uint64_t        reserved1   : 10{0}; // Reserved bits (bits 34-43)
    uint64_t        GSCID       : 16{0}; // Guest Soft-Context ID (bits 44-59)
    uint64_t        reserved2   : 14{0}; // Reserved bits (bits 60-73)
    uint64_t        ADDR        : 52{0}; // Address[63:12] for page-aligned addresses (bits 74-125)
    uint64_t        reserved3   : 2{0};  // Reserved bits (bits 126-127)

    // Constructor for VMA command
    IotinvalCommand(IotinvalFunc func = IotinvalFunc::VMA) : func3(func)
    {
      if (func == IotinvalFunc::GVMA) {
        PSCV = 0;  // Must be 0 for GVMA per specification
      }
    }
  };

  // Union to reinterpret 2 double words as different commands.
  union Command
  {
    /// Construct from an AtsInvalCommand
    Command(AtsInvalCommand inval)
      : inval(inval)
    {}

    /// Construct from an AtsPrgrCommand
    Command(AtsPrgrCommand prgr)
      : prgr(prgr)
    {}

    /// Construct from an IodirCommand
    Command(IodirCommand iodir)
      : iodir(iodir)
    {}

    /// Construct from an IofenceCCommand
    Command(IofenceCCommand iofence)
      : iofence(iofence)
    {}

    /// Construct from an IotinvalCommand
    Command(IotinvalCommand iotinval)
      : iotinval(iotinval)
    {}

    /// Construct from an AtsCommandData.
    Command(AtsCommandData data)
      : data(data)
    {}

    /// Construct from 2 double words.
    Command(uint64_t dw0, uint64_t dw1)
      : data{dw0, dw1}
    {}

    /// True if the opcode corresponds to ATS.
    bool isAts() const
    { return (CommandOpcode)inval.opcode == CommandOpcode::ATS; }

    /// True if the opcode corresponds to IODIR.
    bool isIodir() const
    { return (CommandOpcode)iodir.opcode == CommandOpcode::IODIR; }

    /// True if the opcode corresponds to IOFENCE.
    bool isIofence() const
    { return (CommandOpcode)iofence.opcode == CommandOpcode::IOFENCE; }

    /// True if the opcode corresponds to IOTINVAL.
    bool isIotinval() const
    { return (CommandOpcode)iotinval.opcode == CommandOpcode::IOTINVAL; }

    /// True if ATS invalidate command.
    bool isInval() const
    { return isAts() and inval.func3 == AtsFunc::INVAL; }

    /// True if ATS page request group command.
    bool isPrgr() const
    { return isAts() and prgr.func3 == AtsFunc::PRGR; }

    /// True if IODIR.INVAL_DDT
    bool isIodirInvalDdt() const
    { return isIodir() and iodir.func3 == IodirFunc::INVAL_DDT; }

    /// True if IODIR.INVAL_PDT
    bool isIodirInvalPdt() const
    { return isIodir() and iodir.func3 == IodirFunc::INVAL_PDT; }

    /// True if IOFENCE.C command.
    bool isIofenceC() const
    { return isIofence() and iofence.func3 == IofenceFunc::C; }

    /// True if IOTINVAL.VMA command.
    bool isIotinvalVma() const
    { return isIotinval() and iotinval.func3 == IotinvalFunc::VMA; }

    /// True if IOTINVAL.GVMA command.
    bool isIotinvalGvma() const
    { return isIotinval() and iotinval.func3 == IotinvalFunc::GVMA; }

    /// Return the first double word of this command.
    uint64_t dw0() const
    { return data.dw0; }

    /// Return the second double word of this command.
    uint64_t dw1() const
    { return data.dw1; }

    AtsInvalCommand   inval;
    AtsPrgrCommand    prgr;
    IodirCommand      iodir;
    IofenceCCommand   iofence;
    IotinvalCommand   iotinval;
    AtsCommandData    data;
  };

  // For backward compatibility
  using AtsCommand = Command;

} // namespace TT_IOMMU


