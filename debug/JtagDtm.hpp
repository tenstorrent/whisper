// Copyright 2026 David_Harris@hmc.edu.
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


namespace WdRiscv
{

  /// Target of a Debug Module Interface (DMI): the debug module. The JTAG
  /// DTM forwards DMI requests to an object of this type.
  class DmiTarget
  {
  public:

    virtual ~DmiTarget() = default;

    /// Return the value of the DM register at the given DMI address.
    virtual uint32_t dmiRead(unsigned addr) = 0;

    /// Write the given value to the DM register at the given DMI address.
    virtual void dmiWrite(unsigned addr, uint32_t value) = 0;

    /// Called once for every TCK cycle the TAP spends in Run-Test/Idle. Busy
    /// delays of the DM (abstract commands, system bus) count these cycles.
    virtual void runTestIdle() = 0;
  };


  /// Configuration of the JTAG debug transport module.
  struct JtagDtmConfig
  {
    uint32_t idcode = 0xdeadbeef;  // Value captured by the IDCODE instruction.
    unsigned abits = 7;            // Width of the DMI address (dtmcs.abits).
    unsigned idle = 0;             // Run-Test/Idle hint reported in dtmcs.idle.
    unsigned dmiBusyCycles = 0;    // Run-Test/Idle cycles a DMI access takes.
  };


  /// JTAG Debug Transport Module (Debug Spec 1.0, chapter 6): a JTAG TAP
  /// controller with a 5-bit instruction register and the IDCODE, DTMCS, DMI
  /// and BYPASS data registers. Unimplemented instructions select BYPASS.
  class JtagDtm
  {
  public:

    JtagDtm(DmiTarget& target, const JtagDtmConfig& config);

    /// Drive the TCK, TMS and TDI pins. State changes and register actions
    /// take place on the rising edge of TCK.
    void setPins(bool tck, bool tms, bool tdi);

    /// Return the TDO pin.
    bool tdo() const
    { return tdo_; }

    /// Asynchronous TAP reset (TRST): go to Test-Logic-Reset and reset the DTM.
    void reset();

  private:

    /// TAP controller states (IEEE 1149.1).
    enum class State : uint8_t
      {
        TestLogicReset, RunTestIdle,
        SelectDr, CaptureDr, ShiftDr, Exit1Dr, PauseDr, Exit2Dr, UpdateDr,
        SelectIr, CaptureIr, ShiftIr, Exit1Ir, PauseIr, Exit2Ir, UpdateIr
      };

    /// Return the state that follows the current one for the given TMS.
    State nextState(bool tms) const;

    /// Load the data register selected by the instruction register.
    void captureDr();

    /// Act on the shifted data register.
    void updateDr();

    /// Reset the DMI state: clear the sticky error and any access in progress.
    void resetDmi();

    DmiTarget& target_;
    JtagDtmConfig config_;

    State state_ = State::TestLogicReset;
    bool tck_ = false;
    bool tdo_ = false;

    uint32_t ir_ = 0;          // Instruction register.
    uint32_t irShift_ = 0;     // Instruction shift register.
    uint64_t dr_ = 0;          // Data shift register.
    unsigned drLength_ = 1;    // Length of the selected data register.

    unsigned dmiStat_ = 0;     // Sticky DMI status: 0 ok, 2 failed, 3 busy.
    uint64_t dmiResult_ = 0;   // Address, data and op of the last DMI access.
    unsigned busyLeft_ = 0;    // Run-Test/Idle cycles until the DMI access completes.
  };

}
