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

#include "JtagDtm.hpp"


using namespace WdRiscv;


namespace
{
  // JTAG instructions (Debug Spec 6.1.2). Every other value selects BYPASS.
  constexpr uint32_t irIdcode = 0x01;
  constexpr uint32_t irDtmcs  = 0x10;
  constexpr uint32_t irDmi    = 0x11;
  constexpr unsigned irLength = 5;

  // Value loaded into the instruction shift register in Capture-IR (IEEE 1149.1
  // requires the 2 least significant bits to be 01).
  constexpr uint32_t irCaptureValue = 0x01;

  // dtmcs fields.
  constexpr uint32_t dtmcsVersion      = 1;   // Debug Spec 0.13 and 1.0.
  constexpr uint32_t dtmcsDmireset     = uint32_t(1) << 16;
  constexpr uint32_t dtmcsDmihardreset = uint32_t(1) << 17;

  // DMI op field: requests (Update-DR) and status (Capture-DR).
  constexpr unsigned dmiOpRead = 1;
  constexpr unsigned dmiOpWrite = 2;
  constexpr unsigned dmiStatusBusy = 3;
}


JtagDtm::JtagDtm(DmiTarget& target, const JtagDtmConfig& config)
  : target_(target), config_(config)
{
  reset();
}


void
JtagDtm::reset()
{
  state_ = State::TestLogicReset;
  ir_ = irIdcode;
  tdo_ = false;
  resetDmi();
}


void
JtagDtm::resetDmi()
{
  dmiStat_ = 0;
  dmiResult_ = 0;
  busyLeft_ = 0;
}


JtagDtm::State
JtagDtm::nextState(bool tms) const
{
  using S = State;
  switch (state_)
    {
    case S::TestLogicReset: return tms? S::TestLogicReset : S::RunTestIdle;
    case S::RunTestIdle:    return tms? S::SelectDr : S::RunTestIdle;
    case S::SelectDr:       return tms? S::SelectIr : S::CaptureDr;
    case S::CaptureDr:
    case S::ShiftDr:        return tms? S::Exit1Dr : S::ShiftDr;
    case S::Exit1Dr:        return tms? S::UpdateDr : S::PauseDr;
    case S::PauseDr:        return tms? S::Exit2Dr : S::PauseDr;
    case S::Exit2Dr:        return tms? S::UpdateDr : S::ShiftDr;
    case S::UpdateDr:       return tms? S::SelectDr : S::RunTestIdle;
    case S::SelectIr:       return tms? S::TestLogicReset : S::CaptureIr;
    case S::CaptureIr:
    case S::ShiftIr:        return tms? S::Exit1Ir : S::ShiftIr;
    case S::Exit1Ir:        return tms? S::UpdateIr : S::PauseIr;
    case S::PauseIr:        return tms? S::Exit2Ir : S::PauseIr;
    case S::Exit2Ir:        return tms? S::UpdateIr : S::ShiftIr;
    case S::UpdateIr:       return tms? S::SelectDr : S::RunTestIdle;
    }
  return S::TestLogicReset;
}


void
JtagDtm::setPins(bool tck, bool tms, bool tdi)
{
  bool rising = tck and not tck_;
  bool falling = not tck and tck_;
  tck_ = tck;

  if (falling)
    {
      // TDO changes on the falling edge: it shows the bit about to be shifted out.
      if (state_ == State::ShiftDr)
        tdo_ = (dr_ & 1) != 0;
      else if (state_ == State::ShiftIr)
        tdo_ = (irShift_ & 1) != 0;
      else
        tdo_ = false;
      return;
    }

  if (not rising)
    return;

  // Rising edge: act on the current state, then move to the next one.
  switch (state_)
    {
    case State::TestLogicReset:
      ir_ = irIdcode;
      break;

    case State::RunTestIdle:
      if (busyLeft_ > 0)
        busyLeft_--;
      target_.runTestIdle();
      break;

    case State::CaptureDr:
      captureDr();
      break;

    case State::ShiftDr:
      dr_ >>= 1;
      if (tdi)
        dr_ |= uint64_t(1) << (drLength_ - 1);
      break;

    case State::UpdateDr:
      updateDr();
      break;

    case State::CaptureIr:
      irShift_ = irCaptureValue;
      break;

    case State::ShiftIr:
      irShift_ >>= 1;
      if (tdi)
        irShift_ |= uint32_t(1) << (irLength - 1);
      break;

    case State::UpdateIr:
      ir_ = irShift_ & ((uint32_t(1) << irLength) - 1);
      break;

    default:
      break;
    }

  state_ = nextState(tms);
  if (state_ == State::TestLogicReset)
    ir_ = irIdcode;
}


void
JtagDtm::captureDr()
{
  switch (ir_)
    {
    case irIdcode:
      dr_ = config_.idcode;
      drLength_ = 32;
      break;

    case irDtmcs:
      dr_ = (dtmcsVersion | (config_.abits << 4) | (dmiStat_ << 10) |
             ((config_.idle & 7) << 12));
      drLength_ = 32;
      break;

    case irDmi:
      // A scan that starts while an access is in progress reports busy, which
      // stays set until the debugger writes dtmcs.dmireset.
      if (busyLeft_ > 0 and dmiStat_ == 0)
        dmiStat_ = dmiStatusBusy;
      dr_ = (dmiResult_ & ~uint64_t(3)) | dmiStat_;
      drLength_ = config_.abits + 34;
      break;

    default:
      // BYPASS, selected also by unimplemented instructions, captures 0.
      dr_ = 0;
      drLength_ = 1;
      break;
    }
}


void
JtagDtm::updateDr()
{
  if (ir_ == irDtmcs)
    {
      if (dr_ & dtmcsDmihardreset)
        resetDmi();
      if (dr_ & dtmcsDmireset)
        dmiStat_ = 0;
      return;
    }

  if (ir_ != irDmi)
    return;

  // An access is not started while the sticky status reports an error or busy.
  if (dmiStat_ != 0)
    return;

  if (busyLeft_ > 0)
    {
      dmiStat_ = dmiStatusBusy;
      return;
    }

  auto op = unsigned(dr_ & 3);
  auto data = uint32_t(dr_ >> 2);
  uint64_t addrMask = (uint64_t(1) << config_.abits) - 1;
  auto addr = unsigned((dr_ >> 34) & addrMask);

  if (op == dmiOpRead)
    {
      uint32_t value = target_.dmiRead(addr);
      dmiResult_ = (uint64_t(addr) << 34) | (uint64_t(value) << 2);
    }
  else if (op == dmiOpWrite)
    {
      target_.dmiWrite(addr, data);
      dmiResult_ = (uint64_t(addr) << 34) | (uint64_t(data) << 2);
    }
  else
    return;  // Nop (and the reserved op) send nothing over the DMI.

  busyLeft_ = config_.dmiBusyCycles;
}
