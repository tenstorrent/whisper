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

#include <array>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <vector>
#include "debug/JtagDtm.hpp"
#include "debug/RemoteBitbang.hpp"
#include "trapEnums.hpp"
#include "util.hpp"


namespace WdRiscv
{

  template <typename URV>
  class Hart;

  template <typename URV>
  class System;

  class Memory;
  class IoDevice;


  /// Configuration of the debug module: one field per key of the "debug_module"
  /// block of the JSON configuration file (see debug/README.md).
  struct DebugModuleConfig
  {
    /// Behavior of abstractcs.relaxedpriv.
    enum class RelaxedPriv : uint8_t { Fixed, Writable };

    bool enable = true;
    unsigned slice = 1000;            // Instructions a hart runs between DM services.

    uint64_t base = 0x400;            // Program buffer and data registers in memory.

    unsigned datacount = 2;
    unsigned progbufsize = 2;
    bool impebreak = true;
    bool abstractauto = true;

    // Access Register command.
    bool arGpr = true;
    bool arFpr = true;
    bool arCsr = true;
    unsigned arSizes = 0b1100;        // Bit n set: aarsize n supported (2: 32-bit, 3: 64-bit).
    bool arPostincrement = false;
    bool arPostexec = true;

    bool quickAccess = false;

    // Access Memory command.
    bool amEnable = true;
    bool amVirtual = true;
    unsigned amSizes = 0b1111;        // Bit n set: aamsize n supported.
    bool amPostincrement = true;

    unsigned abstractBusyCycles = 0;  // Run-Test/Idle cycles an abstract command stays busy.

    // System bus access.
    bool sbaEnable = false;
    unsigned sbaAsize = 64;
    unsigned sbaSizes = 0b1111;       // Bit n set: sbaccess n supported.
    unsigned sbaBusyCycles = 20;      // Run-Test/Idle cycles a system bus access takes.

    bool hasel = true;
    unsigned haltgroups = 1;          // Highest halt group number (0: no halt groups).
    unsigned resumegroups = 0;        // Highest resume group number (0: no resume groups).
    unsigned externalTriggers = 0;    // Number of DM external triggers.
    bool authentication = false;
    bool resethaltreq = true;
    bool hartreset = true;
    RelaxedPriv relaxedpriv = RelaxedPriv::Fixed;
    bool stickyunavail = false;
    std::optional<uint64_t> confstrptr;
    uint32_t nextdm = 0;

    /// Spike's debugger-test features: the custom DMI register 0x1f controls hart
    /// availability, and odd abstract register numbers from 0xc000 are custom
    /// registers whose value tracks the register number.
    bool spikeTestHooks = false;

    JtagDtmConfig dtm;
  };


  /// RISC-V Debug Module (Debug Spec 1.0) reached through a JTAG DTM over a
  /// remote_bitbang socket. A halted hart executes only what the debugger asks
  /// for: GPRs and FPRs are accessed directly, a CSR or memory access executes one
  /// instruction, and the program buffer runs in debug mode, from a window of
  /// memory at the configured base address (layout in debug/README.md).
  template <typename URV>
  class DebugModule : public DmiTarget
  {
  public:

    using HartPtr = std::shared_ptr<Hart<URV>>;

    DebugModule(System<URV>& system, const DebugModuleConfig& config);

    ~DebugModule() override;

    DebugModule(const DebugModule&) = delete;
    DebugModule& operator=(const DebugModule&) = delete;

    /// Check the memory window, point the debug-mode park loop and trap address
    /// of every hart at it, map the data registers, and open the remote bitbang
    /// socket on the given port (0: chosen by the OS). Return true on success.
    bool open(unsigned port);

    /// Run the harts in slices, servicing the debugger between slices, until
    /// every hart (or, if waitAll is false, any hart) finishes. Return false if
    /// a hart finished with a failing condition.
    bool run(std::vector<util::file::SharedFile>& traceFiles, bool waitAll);

    /// Total size in bytes of the memory window.
    static constexpr uint64_t windowSize = 0x80;

    // DmiTarget interface.
    uint32_t dmiRead(unsigned addr) override;
    void dmiWrite(unsigned addr, uint32_t value) override;
    void runTestIdle() override;

    /// Read size bytes of the data registers at the given offset in the data area.
    uint64_t readData(uint64_t offset, unsigned size) const;

    /// Write size bytes of the data registers at the given offset in the data area.
    void writeData(uint64_t offset, uint64_t value, unsigned size);

  private:

    /// Values of abstractcs.cmderr.
    enum class CmdErr : uint8_t
      { None = 0, Busy = 1, NotSup = 2, Exception = 3, HaltResume = 4, Bus = 5, Other = 7 };

    /// Debug state of one hart, kept by the DM.
    struct HartState
    {
      bool haltReq = false;
      bool resumeAck = false;
      bool haveReset = true;      // Harts come out of reset at power on.
      bool resetHaltReq = false;
      bool inHartReset = false;   // Held in reset by dmcontrol.hartreset.
      bool unavailSticky = false;
      bool finished = false;      // Program finished (or stopped): never runs again.
      bool wasHalted = false;
      bool customAvailable = true;
      std::array<unsigned, 2> group = { 0, 0 };  // Halt group, resume group.
    };

    unsigned hartCount() const
    { return static_cast<unsigned>(harts_.size()); }

    Hart<URV>& hart(unsigned ix) const
    { return *harts_.at(ix); }

    /// True if the hart is held in reset by ndmreset or hartreset.
    bool inReset(unsigned ix) const
    { return ndmreset_ or states_.at(ix).inHartReset; }

    bool available(unsigned ix) const;
    bool halted(unsigned ix) const;
    bool running(unsigned ix) const;

    /// Indices of the currently selected (existing) harts.
    std::vector<unsigned> selectedHarts() const;

    /// True if hartsel names an existing hart.
    bool hartselExists() const
    { return hartsel_ < hartCount(); }

    /// Return to the reset state (dmactive cleared).
    void resetDm();

    void writeDmcontrol(uint32_t value);
    void writeDmcs2(uint32_t value);
    void writeSbcs(uint32_t value);
    uint32_t readDmstatus();
    uint32_t readHartinfo() const;
    uint32_t readHaltsum(unsigned level) const;
    uint32_t readDmcs2() const;
    uint32_t readSbcs() const;

    /// Read or write data/progbuf register i through the DMI, with the busy and
    /// autoexec rules.
    uint32_t accessDataReg(unsigned i, bool isProgbuf, bool write, uint32_t value);

    void haltHart(unsigned ix, DebugModeCause cause);
    void resumeHart(unsigned ix);
    void assertReset(unsigned ix);
    void releaseReset(unsigned ix);

    /// Propagate halts through halt groups and remember which harts are halted.
    void noteHalts();

    /// Mark a hart unavailable for the sticky unavail status.
    void noteUnavailable(unsigned ix);

    /// Replace the authentication challenge by a new one.
    void newChallenge();

    /// Execute the command register, or set cmderr.
    void executeCommand();
    CmdErr accessRegister(uint32_t cmd, bool& runProgbuf);
    CmdErr transferRegister(unsigned regno, unsigned bits, bool write);
    CmdErr accessCsr(unsigned csr, unsigned bits, bool write);
    CmdErr accessMemory(uint32_t cmd);
    CmdErr quickAccess();

    /// Execute one instruction on the given halted hart at the abstract command
    /// slot of the window. Return false if it took an exception.
    bool execAbstractInst(unsigned ix, uint32_t inst);

    /// Start executing the program buffer on the given hart.
    void startProgbuf(unsigned ix, bool quick);

    /// Execute at most maxSteps program buffer instructions; finish the command
    /// when the program buffer ends. Return true if it ended.
    bool continueProgbuf(uint64_t maxSteps);

    /// End the command in progress with the given error.
    void finishCommand(CmdErr err);

    /// Stop any program buffer execution (DM or hart reset).
    void abortProgbuf();

    void setCmdErr(CmdErr err)
    { if (cmderr_ == CmdErr::None) cmderr_ = err; }

    /// Argument i of the data registers with the given width in bits.
    uint64_t readArg(unsigned i, unsigned bits) const;
    void writeArg(unsigned i, unsigned bits, uint64_t value);

    /// Write a program buffer word to the DM and to memory.
    void writeProgbuf(unsigned i, uint32_t value);

    // System bus access.
    bool sbPresent(unsigned addr) const;
    void sbStart(bool write);
    void sbAccess();
    uint64_t sbAddress() const;

    std::vector<HartPtr> harts_;
    Memory& memory_;
    DebugModuleConfig config_;
    JtagDtm dtm_;
    RemoteBitbang rbb_;
    std::shared_ptr<IoDevice> dataDev_;

    uint64_t progbufAddr_ = 0;
    uint64_t parkAddr_ = 0;
    uint64_t exceptionAddr_ = 0;
    uint64_t slotAddr_ = 0;
    uint64_t dataAddr_ = 0;

    std::vector<HartState> states_;
    std::vector<FILE*> traceFiles_;
    unsigned hartselLen_ = 0;
    unsigned hawindowselLen_ = 0;

    // DM state reset by dmactive.
    bool dmactive_ = false;
    bool ndmreset_ = false;
    bool hartresetBit_ = false;
    bool hasel_ = false;
    unsigned hartsel_ = 0;
    std::array<uint32_t, 12> data_ = {};
    std::array<uint32_t, 16> progbuf_ = {};
    uint32_t command_ = 0;
    CmdErr cmderr_ = CmdErr::None;
    bool busy_ = false;
    unsigned busyCycles_ = 0;     // Run-Test/Idle cycles left before busy clears.
    bool progbufRunning_ = false;
    bool quickAccessRun_ = false;
    unsigned progbufHart_ = 0;
    uint32_t autoexecData_ = 0;
    uint32_t autoexecProgbuf_ = 0;
    bool relaxedpriv_ = false;
    unsigned hawindowsel_ = 0;
    std::vector<bool> hartArray_;
    bool authenticated_ = true;
    uint32_t challenge_ = 0x2545f491;
    bool grouptype_ = false;
    bool hgselect_ = false;
    unsigned dmexttrigger_ = 0;
    std::vector<std::array<unsigned, 2>> extTriggerGroups_;
    int32_t customBase_ = 0;      // Spike test hook custom registers.

    // System bus state.
    bool sbReadOnAddr_ = false;
    unsigned sbAccess_ = 2;
    bool sbAutoIncrement_ = false;
    bool sbReadOnData_ = false;
    unsigned sbError_ = 0;
    bool sbBusyError_ = false;
    std::array<uint32_t, 4> sbAddress_ = {};
    std::array<uint32_t, 4> sbData_ = {};
    unsigned sbBusyLeft_ = 0;
    bool sbPendingWrite_ = false;
  };

}
