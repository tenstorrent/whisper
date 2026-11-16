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

#include <iostream>
#include <sys/time.h>
#include "debug/DebugModule.hpp"
#include "Hart.hpp"
#include "IoDevice.hpp"
#include "System.hpp"


using namespace WdRiscv;


namespace
{
  // DMI register addresses (Debug Spec 1.0, section 3.14).
  enum DmReg : uint8_t
    {
      Data0 = 0x04, Data11 = 0x0f, Dmcontrol = 0x10, Dmstatus = 0x11, Hartinfo = 0x12,
      Haltsum1 = 0x13, Hawindowsel = 0x14, Hawindow = 0x15, Abstractcs = 0x16,
      Command = 0x17, Abstractauto = 0x18, Confstrptr0 = 0x19, Confstrptr3 = 0x1c,
      Nextdm = 0x1d, Custom = 0x1f, Progbuf0 = 0x20, Progbuf15 = 0x2f, Authdata = 0x30,
      Dmcs2 = 0x32, Haltsum2 = 0x34, Haltsum3 = 0x35, Sbaddress3 = 0x37, Sbcs = 0x38,
      Sbaddress0 = 0x39, Sbaddress1 = 0x3a, Sbaddress2 = 0x3b, Sbdata0 = 0x3c,
      Sbdata1 = 0x3d, Sbdata2 = 0x3e, Sbdata3 = 0x3f, Haltsum0 = 0x40
    };

  // Offsets in the memory window (see debug/README.md).
  constexpr uint64_t parkOffset = 0x44;
  constexpr uint64_t exceptionOffset = 0x48;
  constexpr uint64_t slotOffset = 0x4c;
  constexpr uint64_t dataOffset = 0x50;
  constexpr uint64_t dataBytes = 48;

  // Program buffer instructions executed per call when a program buffer runs
  // long; the DM services the socket between calls.
  constexpr uint64_t progbufBudget = 100000;

  // Instruction encodings.
  constexpr uint32_t ebreakInst = 0x00100073;
  constexpr unsigned regS0 = 8;
  constexpr unsigned regS1 = 9;

  constexpr uint32_t
  iType(uint32_t imm, unsigned rs1, unsigned funct3, unsigned rd, unsigned opcode)
  { return (imm << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode; }

  /// csrrs rd, csr, x0
  constexpr uint32_t
  csrRead(unsigned csr, unsigned rd)
  { return iType(csr, 0, 2, rd, 0x73); }

  /// csrrw x0, csr, rs1
  constexpr uint32_t
  csrWrite(unsigned csr, unsigned rs1)
  { return iType(csr, rs1, 1, 0, 0x73); }

  /// Load (lbu, lhu, lwu or ld; lw on RV32) of size 1 << sizeLog2 bytes: rd <- [rs1].
  constexpr uint32_t
  load(unsigned sizeLog2, unsigned rs1, unsigned rd, bool rv64)
  {
    unsigned funct3 = 4 + sizeLog2;  // Unsigned loads.
    if (sizeLog2 == 3 or (sizeLog2 == 2 and not rv64))
      funct3 = sizeLog2;             // ld, and lw on RV32.
    return iType(0, rs1, funct3, rd, 0x03);
  }

  /// Store of size 1 << sizeLog2 bytes: [rs1] <- rs2.
  constexpr uint32_t
  store(unsigned sizeLog2, unsigned rs1, unsigned rs2)
  { return (rs2 << 20) | (rs1 << 15) | (sizeLog2 << 12) | 0x23; }

  uint64_t
  lowBits(uint64_t value, unsigned bits)
  { return bits >= 64 ? value : value & ((uint64_t(1) << bits) - 1); }

  /// Number of bits needed to hold values 0 to n-1.
  unsigned
  indexBits(uint64_t n)
  {
    unsigned bits = 0;
    while (n > (uint64_t(1) << bits))
      bits++;
    return bits;
  }
}


namespace WdRiscv
{

  /// The data registers of a debug module mapped in memory (hartinfo.dataaddr),
  /// so that program buffer loads and stores reach them.
  template <typename URV>
  class DebugDataDevice : public IoDevice
  {
  public:

    DebugDataDevice(DebugModule<URV>& dm, uint64_t addr, uint64_t size)
      : IoDevice("debug_module", addr, size), dm_(dm)
    { }

    uint32_t read(uint64_t addr) override
    { return static_cast<uint32_t>(dm_.readData(addr - address(), 4)); }

    void write(uint64_t addr, uint32_t value) override
    { dm_.writeData(addr - address(), value, 4); }

    uint64_t readSized(uint64_t addr, unsigned size) override
    { return dm_.readData(addr - address(), size); }

    void writeSized(uint64_t addr, uint64_t value, unsigned size) override
    { dm_.writeData(addr - address(), value, size); }

    void enable() override
    { }

    void disable() override
    { }

    bool saveSnapshot(const std::string&) const override
    { return true; }

    bool loadSnapshot(const std::string&) override
    { return true; }

  private:

    DebugModule<URV>& dm_;
  };

}


template <typename URV>
DebugModule<URV>::DebugModule(System<URV>& system, const DebugModuleConfig& config)
  : memory_(*system.memory()), config_(config),
    dtm_(*this, config.dtm), rbb_(dtm_)
{
  for (unsigned i = 0; i < system.hartCount(); ++i)
    harts_.push_back(system.ithHart(i));

  states_.resize(hartCount());
  hartArray_.resize(hartCount());
  traceFiles_.resize(hartCount());
  extTriggerGroups_.resize(config_.externalTriggers);

  hartselLen_ = indexBits(hartCount());
  hawindowselLen_ = indexBits((hartCount() + 31) / 32);

  progbufAddr_ = config_.base;
  parkAddr_ = config_.base + parkOffset;
  exceptionAddr_ = config_.base + exceptionOffset;
  slotAddr_ = config_.base + slotOffset;
  dataAddr_ = config_.base + dataOffset;

  resetDm();
}


template <typename URV>
DebugModule<URV>::~DebugModule() = default;


template <typename URV>
bool
DebugModule<URV>::open(unsigned port)
{
  // The window must be ordinary memory that the harts can execute, read and write.
  if (config_.base + windowSize > memory_.size() or config_.base + windowSize < config_.base)
    {
      std::cerr << "Error: debug_module.base 0x" << std::hex << config_.base
                << " -- window 0x" << config_.base << " to 0x" << config_.base + windowSize - 1
                << " is outside memory (size 0x" << memory_.size() << ")" << std::dec << '\n';
      return false;
    }

  for (uint64_t addr = config_.base; addr < config_.base + windowSize; addr += 4)
    {
      Pma pma = hart(0).getPma(addr);
      if (not pma.isRead() or not pma.isWrite() or not pma.isExec())
        {
          std::cerr << "Error: debug_module.base 0x" << std::hex << config_.base
                    << " -- address 0x" << addr << " in the debug module window is not"
                    << " readable, writable and executable: add a memmap.pma region"
                    << " with read, write and exec for 0x" << config_.base << " to 0x"
                    << config_.base + windowSize - 1 << std::dec << '\n';
          return false;
        }
    }

  for (auto& hartPtr : harts_)
    {
      if (hartPtr->hasDebugParkLoop())
        std::cerr << "Warning: debug_module overrides debug_park_loop and debug_trap_address\n";
      hartPtr->setDebugParkLoop(URV(parkAddr_));
      hartPtr->setDebugTrapAddress(URV(exceptionAddr_));
      hartPtr->setExternalDebugModule(true);

      URV dcsr = 0;
      if (not hartPtr->peekCsr(CsrNumber::DCSR, dcsr))
        {
          std::cerr << "Error: debug_module requires the dcsr CSR (debug mode)\n";
          return false;
        }
    }

  // Program buffer (zero), implicit ebreak, park and exception words.
  for (unsigned i = 0; i < 16; ++i)
    writeProgbuf(i, 0);
  if (config_.impebreak)
    memory_.poke(progbufAddr_ + (uint64_t(4) * config_.progbufsize), ebreakInst);
  memory_.poke(parkAddr_, ebreakInst);
  memory_.poke(exceptionAddr_, ebreakInst);
  memory_.poke(slotAddr_, ebreakInst);

  dataDev_ = std::make_shared<DebugDataDevice<URV>>(*this, dataAddr_, dataBytes);
  memory_.registerIoDevice(dataDev_);

  return rbb_.open(port);
}


template <typename URV>
uint64_t
DebugModule<URV>::readData(uint64_t offset, unsigned size) const
{
  uint64_t value = 0;
  for (unsigned i = 0; i < size; ++i)
    {
      uint64_t byteIx = offset + i;
      if (byteIx >= 4*data_.size())
        break;
      uint64_t byte = (data_.at(byteIx / 4) >> (8 * (byteIx % 4))) & 0xff;
      value |= byte << (8*i);
    }
  return value;
}


template <typename URV>
void
DebugModule<URV>::writeData(uint64_t offset, uint64_t value, unsigned size)
{
  for (unsigned i = 0; i < size; ++i)
    {
      uint64_t byteIx = offset + i;
      if (byteIx >= 4*data_.size())
        break;
      uint32_t& word = data_.at(byteIx / 4);
      unsigned shift = 8 * (byteIx % 4);
      word = (word & ~(uint32_t(0xff) << shift)) | (uint32_t((value >> (8*i)) & 0xff) << shift);
    }
}


template <typename URV>
void
DebugModule<URV>::writeProgbuf(unsigned i, uint32_t value)
{
  progbuf_.at(i) = value;
  if (i < config_.progbufsize)
    {
      uint64_t addr = progbufAddr_ + (uint64_t(4) * i);
      memory_.poke(addr, value);
      for (auto& hartPtr : harts_)
        hartPtr->invalidateDecodeCacheRange(addr, 4);
    }
}


template <typename URV>
bool
DebugModule<URV>::available(unsigned ix) const
{
  if (ix >= hartCount())
    return false;
  const auto& st = states_.at(ix);
  return not st.finished and not inReset(ix) and st.customAvailable;
}


template <typename URV>
bool
DebugModule<URV>::halted(unsigned ix) const
{
  return available(ix) and hart(ix).inDebugMode();
}


template <typename URV>
bool
DebugModule<URV>::running(unsigned ix) const
{
  return available(ix) and not hart(ix).inDebugMode();
}


template <typename URV>
std::vector<unsigned>
DebugModule<URV>::selectedHarts() const
{
  std::vector<unsigned> sel;
  for (unsigned i = 0; i < hartCount(); ++i)
    if (i == hartsel_ or (hasel_ and hartArray_.at(i)))
      sel.push_back(i);
  return sel;
}


template <typename URV>
void
DebugModule<URV>::resetDm()
{
  abortProgbuf();

  // Release the harts held in reset by the DM.
  bool heldByNdmreset = ndmreset_;
  ndmreset_ = false;
  for (unsigned i = 0; i < hartCount(); ++i)
    {
      auto& st = states_.at(i);
      bool held = heldByNdmreset or st.inHartReset;
      st.inHartReset = false;
      st.haltReq = false;
      st.resetHaltReq = false;
      st.resumeAck = false;
      st.unavailSticky = false;
      st.group = { 0, 0 };
      if (held)
        releaseReset(i);
    }

  dmactive_ = false;
  hartresetBit_ = false;
  hasel_ = false;
  hartsel_ = 0;
  data_.fill(0);
  for (unsigned i = 0; i < progbuf_.size(); ++i)
    writeProgbuf(i, 0);
  command_ = 0;
  cmderr_ = CmdErr::None;
  busy_ = false;
  busyCycles_ = 0;
  autoexecData_ = 0;
  autoexecProgbuf_ = 0;
  relaxedpriv_ = false;
  hawindowsel_ = 0;
  hartArray_.assign(hartArray_.size(), false);

  // Authentication: a new challenge for each activation.
  authenticated_ = not config_.authentication;
  newChallenge();

  grouptype_ = false;
  hgselect_ = false;
  dmexttrigger_ = 0;
  for (auto& groups : extTriggerGroups_)
    groups = { 0, 0 };

  sbReadOnAddr_ = false;
  sbAccess_ = 2;
  sbAutoIncrement_ = false;
  sbReadOnData_ = false;
  sbError_ = 0;
  sbBusyError_ = false;
  sbAddress_.fill(0);
  sbData_.fill(0);
  sbBusyLeft_ = 0;
  sbPendingWrite_ = false;
}


template <typename URV>
void
DebugModule<URV>::noteUnavailable(unsigned ix)
{
  states_.at(ix).unavailSticky = true;
}


template <typename URV>
void
DebugModule<URV>::newChallenge()
{
  // xorshift32.
  challenge_ ^= challenge_ << 13;
  challenge_ ^= challenge_ >> 17;
  challenge_ ^= challenge_ << 5;
}


template <typename URV>
void
DebugModule<URV>::haltHart(unsigned ix, DebugModeCause cause)
{
  auto& h = hart(ix);
  if (h.inDebugMode())
    return;
  h.enterDebugMode_(cause, h.peekPc());
}


template <typename URV>
void
DebugModule<URV>::resumeHart(unsigned ix)
{
  auto& h = hart(ix);
  auto& st = states_.at(ix);
  if (not h.inDebugMode())
    return;

  h.exitDebugMode();
  st.resumeAck = true;

  // With dcsr.step the hart executes one instruction and halts again. Run it
  // now so that the fast execution loops need no single-step check.
  if (h.hasDcsrStep())
    {
      h.singleStep(traceFiles_.at(ix));
      h.evaluateDebugStep();
      if (h.hasTargetProgramFinished())
        {
          st.finished = true;
          noteUnavailable(ix);
        }
    }
}


template <typename URV>
void
DebugModule<URV>::assertReset(unsigned ix)
{
  // Called before the hart is marked as held: reset only on the first hold.
  if (inReset(ix))
    return;
  if (progbufRunning_ and progbufHart_ == ix)
    abortProgbuf();
  hart(ix).reset();
  noteUnavailable(ix);
}


template <typename URV>
void
DebugModule<URV>::releaseReset(unsigned ix)
{
  if (inReset(ix))
    return;   // Still held by the other reset.

  auto& st = states_.at(ix);
  st.haveReset = true;
  if (st.finished or not st.customAvailable)
    return;

  if (config_.resethaltreq and st.resetHaltReq)
    haltHart(ix, DebugModeCause::RESETHALTREQ);
  else if (st.haltReq)
    haltHart(ix, DebugModeCause::HALTREQ);
}


template <typename URV>
void
DebugModule<URV>::noteHalts()
{
  if (config_.haltgroups)
    for (unsigned i = 0; i < hartCount(); ++i)
      {
        unsigned group = states_.at(i).group.at(0);
        if (group == 0 or not halted(i) or states_.at(i).wasHalted)
          continue;
        for (unsigned j = 0; j < hartCount(); ++j)
          if (j != i and states_.at(j).group.at(0) == group and running(j))
            haltHart(j, DebugModeCause::GROUP);
      }

  for (unsigned i = 0; i < hartCount(); ++i)
    states_.at(i).wasHalted = halted(i);
}


template <typename URV>
uint64_t
DebugModule<URV>::readArg(unsigned i, unsigned bits) const
{
  unsigned words = bits <= 32 ? 1 : bits / 32;
  unsigned ix = i * words;
  uint64_t value = data_.at(ix);
  if (words > 1)
    value |= uint64_t(data_.at(ix + 1)) << 32;
  return lowBits(value, bits);
}


template <typename URV>
void
DebugModule<URV>::writeArg(unsigned i, unsigned bits, uint64_t value)
{
  unsigned words = bits <= 32 ? 1 : bits / 32;
  unsigned ix = i * words;
  data_.at(ix) = uint32_t(value);
  if (words > 1)
    data_.at(ix + 1) = uint32_t(value >> 32);
}


template <typename URV>
uint32_t
DebugModule<URV>::readDmstatus()
{
  uint32_t value = 3;  // version: Debug Spec 1.0
  if (not authenticated_)
    return value;  // authenticated = 0, authbusy = 0

  auto sel = selectedHarts();
  bool anyNonexistent = not hartselExists();
  bool allNonexistent = sel.empty();

  bool anyHalted = false, allHalted = not sel.empty();
  bool anyRunning = false, allRunning = not sel.empty();
  bool anyUnavail = false, allUnavail = not sel.empty();
  bool anyResumeAck = false, allResumeAck = not sel.empty();
  bool anyHaveReset = false, allHaveReset = not sel.empty();

  for (unsigned i : sel)
    {
      const auto& st = states_.at(i);
      bool isHalted = halted(i), isRunning = running(i);
      bool unavail = not available(i) or (config_.stickyunavail and st.unavailSticky);
      anyHalted = anyHalted or isHalted;
      allHalted = allHalted and isHalted;
      anyRunning = anyRunning or isRunning;
      allRunning = allRunning and isRunning;
      anyUnavail = anyUnavail or unavail;
      allUnavail = allUnavail and unavail;
      anyResumeAck = anyResumeAck or st.resumeAck;
      allResumeAck = allResumeAck and st.resumeAck;
      anyHaveReset = anyHaveReset or st.haveReset;
      allHaveReset = allHaveReset and st.haveReset;
    }

  bool impebreak = config_.impebreak and config_.progbufsize > 0;

  value |= uint32_t(ndmreset_) << 24;
  value |= uint32_t(config_.stickyunavail) << 23;
  value |= uint32_t(impebreak) << 22;
  value |= uint32_t(allHaveReset) << 19;
  value |= uint32_t(anyHaveReset) << 18;
  value |= uint32_t(allResumeAck) << 17;
  value |= uint32_t(anyResumeAck) << 16;
  value |= uint32_t(allNonexistent) << 15;
  value |= uint32_t(anyNonexistent) << 14;
  value |= uint32_t(allUnavail) << 13;
  value |= uint32_t(anyUnavail) << 12;
  value |= uint32_t(allRunning) << 11;
  value |= uint32_t(anyRunning) << 10;
  value |= uint32_t(allHalted) << 9;
  value |= uint32_t(anyHalted) << 8;
  value |= uint32_t(authenticated_) << 7;
  value |= uint32_t(config_.resethaltreq) << 5;
  value |= uint32_t(config_.confstrptr.has_value()) << 4;
  return value;
}


template <typename URV>
uint32_t
DebugModule<URV>::readHartinfo() const
{
  if (not hartselExists())
    return 0;

  auto& h = hart(hartsel_);
  URV val = 0;
  unsigned nscratch = 0;
  if (h.peekCsr(CsrNumber::DSCRATCH0, val))
    nscratch++;
  if (h.peekCsr(CsrNumber::DSCRATCH1, val))
    nscratch++;

  uint32_t value = nscratch << 20;

  // The data registers are reachable with x0-relative loads and stores when their
  // address fits in a sign-extended 12-bit offset.
  URV addr = URV(dataAddr_);
  bool reachable = addr <= 0x7ff or addr >= URV(-0x800);
  if (reachable)
    value |= (uint32_t(1) << 16) | (config_.datacount << 12) | uint32_t(addr & 0xfff);
  return value;
}


template <typename URV>
uint32_t
DebugModule<URV>::readHaltsum(unsigned level) const
{
  // Each bit of haltsum<level> covers 32^level harts.
  uint64_t groupSize = uint64_t(1) << (5 * level);
  uint64_t base = hartsel_ & ~((32*groupSize) - 1);
  uint32_t value = 0;
  for (unsigned i = 0; i < hartCount(); ++i)
    if (i >= base and i < base + 32*groupSize and halted(i))
      value |= uint32_t(1) << ((i - base) / groupSize);
  return value;
}


template <typename URV>
uint32_t
DebugModule<URV>::readDmcs2() const
{
  unsigned group = 0;
  if (hgselect_)
    {
      if (dmexttrigger_ < extTriggerGroups_.size())
        group = extTriggerGroups_.at(dmexttrigger_).at(grouptype_);
    }
  else if (hartselExists())
    group = states_.at(hartsel_).group.at(grouptype_);

  return ((uint32_t(grouptype_) << 11) | (dmexttrigger_ << 7) | (group << 2) |
          uint32_t(hgselect_));
}


template <typename URV>
void
DebugModule<URV>::writeDmcs2(uint32_t value)
{
  grouptype_ = config_.resumegroups > 0 and ((value >> 11) & 1);
  hgselect_ = config_.externalTriggers > 0 and (value & 1);
  unsigned trig = (value >> 7) & 0xf;
  dmexttrigger_ = trig < config_.externalTriggers ? trig : 0;

  if (((value >> 1) & 1) == 0)
    return;  // hgwrite clear.

  unsigned group = (value >> 2) & 0x1f;
  unsigned maxGroup = grouptype_ ? config_.resumegroups : config_.haltgroups;
  if (group > maxGroup)
    return;  // Unsupported group: no change.

  if (hgselect_)
    extTriggerGroups_.at(dmexttrigger_).at(grouptype_) = group;
  else
    for (unsigned i : selectedHarts())
      states_.at(i).group.at(grouptype_) = group;
}


template <typename URV>
uint32_t
DebugModule<URV>::accessDataReg(unsigned i, bool isProgbuf, bool write, uint32_t value)
{
  uint32_t result = isProgbuf ? progbuf_.at(i) : data_.at(i);

  if (busy_)
    {
      setCmdErr(CmdErr::Busy);
      return result;
    }

  if (write)
    {
      if (isProgbuf)
        writeProgbuf(i, value);
      else
        data_.at(i) = value;
    }

  uint32_t autoexec = isProgbuf ? autoexecProgbuf_ : autoexecData_;
  if (i < 32 and ((autoexec >> i) & 1))
    executeCommand();
  return result;
}


template <typename URV>
uint32_t
DebugModule<URV>::dmiRead(unsigned addr)
{
  if (not dmactive_)
    return 0;

  if (not authenticated_)
    {
      // Until the debugger authenticates, only these registers can be read.
      if (addr == Dmstatus)
        return readDmstatus();
      if (addr == Dmcontrol)
        return 1;
      if (addr == Authdata)
        return challenge_;
      return 0;
    }

  if (addr >= Data0 and addr < Data0 + config_.datacount)
    return accessDataReg(addr - Data0, false, false, 0);

  if (addr >= Progbuf0 and addr < Progbuf0 + config_.progbufsize)
    return accessDataReg(addr - Progbuf0, true, false, 0);

  if (not sbPresent(addr))
    return 0;

  switch (addr)
    {
    case Dmcontrol:
      return ((uint32_t(hartresetBit_) << 29) | (uint32_t(hasel_) << 26) |
              ((hartsel_ & 0x3ff) << 16) | (((hartsel_ >> 10) & 0x3ff) << 6) |
              (uint32_t(ndmreset_) << 1) | uint32_t(dmactive_));

    case Dmstatus:
      return readDmstatus();

    case Hartinfo:
      return readHartinfo();

    case Haltsum0:
      return readHaltsum(0);

    case Haltsum1:
      return readHaltsum(1);

    case Haltsum2:
      return readHaltsum(2);

    case Haltsum3:
      return readHaltsum(3);

    case Hawindowsel:
      return config_.hasel ? hawindowsel_ : 0;

    case Hawindow:
      {
        if (not config_.hasel)
          return 0;
        uint32_t value = 0;
        for (unsigned i = 0; i < 32; ++i)
          {
            uint64_t ix = (uint64_t(hawindowsel_) * 32) + i;
            if (ix < hartCount() and hartArray_.at(ix))
              value |= uint32_t(1) << i;
          }
        return value;
      }

    case Abstractcs:
      return ((config_.progbufsize << 24) | (uint32_t(busy_) << 12) |
              (uint32_t(relaxedpriv_) << 11) | (unsigned(cmderr_) << 8) |
              config_.datacount);

    case Abstractauto:
      return (autoexecProgbuf_ << 16) | autoexecData_;

    case Confstrptr0: case Confstrptr0 + 1: case Confstrptr0 + 2: case Confstrptr3:
      {
        if (not config_.confstrptr)
          return 0;
        unsigned shift = 32 * (addr - Confstrptr0);
        return shift < 64 ? uint32_t(*config_.confstrptr >> shift) : 0;
      }

    case Nextdm:
      return config_.nextdm;

    case Custom:
      {
        if (not config_.spikeTestHooks)
          return 0;
        uint32_t value = 0;
        for (unsigned i = 0; i < hartCount() and i < 32; ++i)
          if (states_.at(i).customAvailable)
            value |= uint32_t(1) << i;
        return value;
      }

    case Authdata:
      return config_.authentication ? challenge_ : 0;

    case Dmcs2:
      return readDmcs2();

    case Sbcs:
      return readSbcs();

    case Sbaddress0: case Sbaddress1: case Sbaddress2:
      return sbAddress_.at(addr - Sbaddress0);

    case Sbaddress3:
      return sbAddress_.at(3);

    case Sbdata0:
      {
        uint32_t value = sbData_.at(0);
        if (sbBusyLeft_ > 0)
          sbBusyError_ = true;
        else if (sbReadOnData_ and sbError_ == 0 and not sbBusyError_)
          sbStart(false);
        return value;
      }

    case Sbdata1: case Sbdata2: case Sbdata3:
      if (sbBusyLeft_ > 0)
        sbBusyError_ = true;
      return sbData_.at(addr - Sbdata0);

    default:
      return 0;  // Unimplemented registers read 0.
    }
}


template <typename URV>
void
DebugModule<URV>::dmiWrite(unsigned addr, uint32_t value)
{
  if (addr == Dmcontrol)
    {
      writeDmcontrol(value);
      return;
    }

  // Only dmcontrol can be written while the DM is inactive, and only dmcontrol
  // and authdata until the debugger authenticates.
  if (not dmactive_)
    return;
  if (not authenticated_ and addr != Authdata)
    return;

  if (addr >= Data0 and addr < Data0 + config_.datacount)
    {
      accessDataReg(addr - Data0, false, true, value);
      return;
    }

  if (addr >= Progbuf0 and addr < Progbuf0 + config_.progbufsize)
    {
      accessDataReg(addr - Progbuf0, true, true, value);
      return;
    }

  if (not sbPresent(addr))
    return;

  switch (addr)
    {
    case Hawindowsel:
      if (config_.hasel)
        hawindowsel_ = value & ((uint32_t(1) << hawindowselLen_) - 1);
      return;

    case Hawindow:
      if (config_.hasel)
        for (unsigned i = 0; i < 32; ++i)
          {
            uint64_t ix = (uint64_t(hawindowsel_) * 32) + i;
            if (ix < hartCount())
              hartArray_.at(ix) = (value >> i) & 1;
          }
      return;

    case Abstractcs:
      if (busy_)
        {
          setCmdErr(CmdErr::Busy);
          return;
        }
      cmderr_ = CmdErr(unsigned(cmderr_) & ~((value >> 8) & 7));
      if (config_.relaxedpriv == DebugModuleConfig::RelaxedPriv::Writable)
        relaxedpriv_ = (value >> 11) & 1;
      return;

    case Command:
      if (busy_)
        {
          setCmdErr(CmdErr::Busy);
          return;
        }
      command_ = value;
      executeCommand();
      return;

    case Abstractauto:
      if (not config_.abstractauto)
        return;
      if (busy_)
        {
          setCmdErr(CmdErr::Busy);
          return;
        }
      // Bits of data and progbuf registers that do not exist are 0.
      autoexecData_ = value & ((uint32_t(1) << config_.datacount) - 1);
      autoexecProgbuf_ = (value >> 16) & ((uint32_t(1) << config_.progbufsize) - 1);
      return;

    case Custom:
      if (config_.spikeTestHooks)
        for (unsigned i = 0; i < hartCount() and i < 32; ++i)
          {
            bool avail = (value >> i) & 1;
            if (not avail)
              noteUnavailable(i);
            states_.at(i).customAvailable = avail;
          }
      return;

    case Authdata:
      if (config_.authentication)
        {
          // Authentication scheme: write the challenge read from authdata plus 1.
          authenticated_ = value == challenge_ + 1;
          if (not authenticated_)
            newChallenge();
        }
      return;

    case Dmcs2:
      writeDmcs2(value);
      noteHalts();
      return;

    case Sbcs:
      writeSbcs(value);
      return;

    case Sbaddress0: case Sbaddress1: case Sbaddress2: case Sbaddress3:
      if (sbBusyLeft_ > 0)
        {
          sbBusyError_ = true;
          return;
        }
      sbAddress_.at(addr == Sbaddress3 ? 3 : addr - Sbaddress0) = value;
      if (addr == Sbaddress0 and sbReadOnAddr_ and sbError_ == 0 and not sbBusyError_)
        sbStart(false);
      return;

    case Sbdata0: case Sbdata1: case Sbdata2: case Sbdata3:
      if (sbBusyLeft_ > 0)
        {
          sbBusyError_ = true;
          return;
        }
      sbData_.at(addr - Sbdata0) = value;
      if (addr == Sbdata0 and sbError_ == 0 and not sbBusyError_)
        sbStart(true);
      return;

    default:
      return;  // Writes to read-only and unimplemented registers have no effect.
    }
}


template <typename URV>
void
DebugModule<URV>::writeDmcontrol(uint32_t value)
{
  bool active = value & 1;
  if (not active)
    {
      resetDm();
      return;
    }

  if (not dmactive_)
    {
      resetDm();
      dmactive_ = true;
    }

  if (not authenticated_)
    return;

  bool haltreq = (value >> 31) & 1;
  bool resumereq = (value >> 30) & 1;
  bool hartreset = (value >> 29) & 1;
  bool ackhavereset = (value >> 28) & 1;
  bool ackunavail = (value >> 27) & 1;
  bool setresethaltreq = (value >> 3) & 1;
  bool clrresethaltreq = (value >> 2) & 1;
  bool ndmreset = (value >> 1) & 1;

  // The selection, and the halt/resume/reset acknowledgement bits, are ignored
  // while an abstract command is executing.
  if (not busy_)
    {
      hasel_ = config_.hasel and ((value >> 26) & 1);
      unsigned sel = (((value >> 6) & 0x3ff) << 10) | ((value >> 16) & 0x3ff);
      hartsel_ = sel & ((uint32_t(1) << hartselLen_) - 1);
    }

  auto sel = selectedHarts();

  // ndmreset: reset every hart, and hold them in reset while the bit is 1.
  if (ndmreset and not ndmreset_)
    {
      for (unsigned i = 0; i < hartCount(); ++i)
        assertReset(i);
      ndmreset_ = true;
    }
  else if (not ndmreset and ndmreset_)
    {
      ndmreset_ = false;
      for (unsigned i = 0; i < hartCount(); ++i)
        releaseReset(i);
    }

  // hartreset: reset the selected harts, and hold them while the bit is 1.
  if (config_.hartreset)
    {
      hartresetBit_ = hartreset;
      for (unsigned i : sel)
        {
          auto& st = states_.at(i);
          if (hartreset and not st.inHartReset)
            {
              assertReset(i);
              st.inHartReset = true;
            }
          else if (not hartreset and st.inHartReset)
            {
              st.inHartReset = false;
              releaseReset(i);
            }
        }
    }

  if (not busy_)
    {
      for (unsigned i : sel)
        {
          auto& st = states_.at(i);
          if (ackhavereset)
            st.haveReset = false;
          if (config_.resethaltreq)
            {
              if (clrresethaltreq)
                st.resetHaltReq = false;
              else if (setresethaltreq)
                st.resetHaltReq = true;
            }
          st.haltReq = haltreq;
          if (haltreq and running(i))
            haltHart(i, DebugModeCause::HALTREQ);
        }

      // resumereq is ignored when haltreq is set, and for harts that are not halted.
      if (resumereq and not haltreq)
        {
          std::vector<bool> resume(hartCount());
          for (unsigned i : sel)
            if (halted(i))
              {
                resume.at(i) = true;
                // Resume groups: the other halted harts of the group resume too.
                unsigned group = states_.at(i).group.at(1);
                if (group != 0)
                  for (unsigned j = 0; j < hartCount(); ++j)
                    if (states_.at(j).group.at(1) == group and halted(j))
                      resume.at(j) = true;
              }
          for (unsigned i = 0; i < hartCount(); ++i)
            if (resume.at(i))
              resumeHart(i);
        }
    }

  for (unsigned i : sel)
    if (ackunavail and available(i))
      states_.at(i).unavailSticky = false;

  noteHalts();
}


template <typename URV>
void
DebugModule<URV>::runTestIdle()
{
  if (busy_ and not progbufRunning_)
    {
      if (busyCycles_ > 0)
        busyCycles_--;
      if (busyCycles_ == 0)
        busy_ = false;
    }

  if (sbBusyLeft_ > 0)
    {
      --sbBusyLeft_;
      if (sbBusyLeft_ == 0)
        sbAccess();
    }
}


template <typename URV>
void
DebugModule<URV>::executeCommand()
{
  if (cmderr_ != CmdErr::None)
    return;  // No command starts until cmderr is cleared.

  if (busy_)
    {
      setCmdErr(CmdErr::Busy);
      return;
    }

  busy_ = true;
  busyCycles_ = config_.abstractBusyCycles;

  if (not available(hartsel_))
    {
      finishCommand(CmdErr::HaltResume);
      return;
    }

  unsigned cmdtype = command_ >> 24;
  CmdErr err = CmdErr::NotSup;

  if (cmdtype == 0)
    {
      bool runProgbuf = false;
      err = accessRegister(command_, runProgbuf);
      if (err == CmdErr::None and runProgbuf)
        {
          startProgbuf(hartsel_, false);
          return;
        }
    }
  else if (cmdtype == 1)
    {
      err = quickAccess();
      if (err == CmdErr::None)
        return;  // The program buffer ends the command.
    }
  else if (cmdtype == 2)
    err = accessMemory(command_);

  finishCommand(err);
}


template <typename URV>
void
DebugModule<URV>::finishCommand(CmdErr err)
{
  if (err != CmdErr::None)
    cmderr_ = err;
  progbufRunning_ = false;
  quickAccessRun_ = false;
  if (busyCycles_ == 0)
    busy_ = false;
}


template <typename URV>
typename DebugModule<URV>::CmdErr
DebugModule<URV>::accessRegister(uint32_t cmd, bool& runProgbuf)
{
  if ((cmd >> 23) & 1)
    return CmdErr::NotSup;  // Bit 23 must be 0.

  unsigned aarsize = (cmd >> 20) & 7;
  bool postincrement = (cmd >> 19) & 1;
  bool postexec = (cmd >> 18) & 1;
  bool transfer = (cmd >> 17) & 1;
  bool write = (cmd >> 16) & 1;
  unsigned regno = cmd & 0xffff;

  if (not halted(hartsel_))
    return CmdErr::HaltResume;

  if (postincrement and not config_.arPostincrement)
    return CmdErr::NotSup;

  if (postexec and (not config_.arPostexec or config_.progbufsize == 0))
    return CmdErr::NotSup;

  if (transfer)
    {
      // aarsize matters only with a transfer.
      unsigned bits = 8u << aarsize;
      if (((config_.arSizes >> aarsize) & 1) == 0 or bits/32 > config_.datacount)
        return CmdErr::NotSup;

      CmdErr err = transferRegister(regno, bits, write);
      if (err != CmdErr::None)
        return err;

      if (postincrement)
        command_ = (command_ & ~uint32_t(0xffff)) | ((regno + 1) & 0xffff);
    }

  runProgbuf = postexec;
  return CmdErr::None;
}


template <typename URV>
typename DebugModule<URV>::CmdErr
DebugModule<URV>::transferRegister(unsigned regno, unsigned bits, bool write)
{
  auto& h = hart(hartsel_);
  constexpr unsigned xlen = 8 * sizeof(URV);

  if (regno < 0x1000)
    {
      if (not config_.arCsr)
        return CmdErr::NotSup;
      return accessCsr(regno, bits, write);
    }

  if (regno < 0x1020)
    {
      if (not config_.arGpr)
        return CmdErr::NotSup;
      if (bits > xlen)
        return CmdErr::NotSup;
      unsigned reg = regno - 0x1000;
      if (reg >= h.intRegCount())
        return CmdErr::Exception;  // Register does not exist (RV32E).
      if (write)
        {
          uint64_t value = readArg(0, bits);
          if (bits == 32)
            value = uint64_t(int64_t(int32_t(value)));  // As lw does.
          if (reg != 0)
            h.pokeIntReg(reg, URV(value));
        }
      else
        writeArg(0, bits, lowBits(h.peekIntReg(reg), bits));
      return CmdErr::None;
    }

  if (regno < 0x1040)
    {
      if (not config_.arFpr)
        return CmdErr::NotSup;
      if (not h.isRvf() and not h.isRvd())
        return CmdErr::Exception;  // No FPRs.
      unsigned flen = h.isRvd() ? 64 : 32;
      if (bits > flen)
        return CmdErr::NotSup;
      unsigned reg = regno - 0x1020;
      if (write)
        {
          uint64_t value = readArg(0, bits);
          if (bits < flen)
            value |= ~uint64_t(0) << bits;  // NaN-box, as flw does.
          h.pokeFpReg(reg, value);
        }
      else
        {
          uint64_t value = 0;
          h.peekFpReg(reg, value);
          writeArg(0, bits, lowBits(value, bits));
        }
      return CmdErr::None;
    }

  // Spike's debugger-test custom registers: writing V to custom register N makes
  // reads of register M return V - N + M.
  if (config_.spikeTestHooks and regno >= 0xc000 and (regno & 1))
    {
      auto number = int32_t(regno - 0xc000);
      if (write)
        customBase_ = int32_t(data_.at(0)) - number;
      else
        {
          data_.at(0) = uint32_t(customBase_ + number);
          if (config_.datacount > 1)
            data_.at(1) = 0;
        }
      return CmdErr::None;
    }

  return CmdErr::NotSup;
}


template <typename URV>
typename DebugModule<URV>::CmdErr
DebugModule<URV>::accessCsr(unsigned csr, unsigned bits, bool write)
{
  constexpr unsigned xlen = 8 * sizeof(URV);
  if (bits > xlen)
    return CmdErr::NotSup;

  auto& h = hart(hartsel_);

  // As Spike does, make fflags, frm and fcsr accessible when mstatus.FS is off,
  // restoring mstatus afterwards.
  URV mstatus = 0;
  bool restoreMstatus = false;
  if (csr >= 1 and csr <= 3 and (h.isRvf() or h.isRvd()) and
      h.peekCsr(CsrNumber::MSTATUS, mstatus) and ((mstatus >> 13) & 3) == 0)
    {
      h.pokeCsr(CsrNumber::MSTATUS, mstatus | (URV(3) << 13));
      restoreMstatus = true;
    }

  // The hart executes the CSR instruction, so that the access has the effects of
  // an M-mode access (WARL legalization, side effects, and an exception if the CSR
  // does not exist).
  URV s0 = h.peekIntReg(regS0);
  bool ok = false;
  if (write)
    {
      uint64_t value = readArg(0, bits);
      if (bits == 32)
        value = uint64_t(int64_t(int32_t(value)));
      h.pokeIntReg(regS0, URV(value));
      ok = execAbstractInst(hartsel_, csrWrite(csr, regS0));
    }
  else
    {
      ok = execAbstractInst(hartsel_, csrRead(csr, regS0));
      if (ok)
        writeArg(0, bits, lowBits(h.peekIntReg(regS0), bits));
    }
  h.pokeIntReg(regS0, s0);

  if (restoreMstatus)
    h.pokeCsr(CsrNumber::MSTATUS, mstatus);

  return ok ? CmdErr::None : CmdErr::Exception;
}


template <typename URV>
typename DebugModule<URV>::CmdErr
DebugModule<URV>::accessMemory(uint32_t cmd)
{
  constexpr unsigned xlen = 8 * sizeof(URV);

  if (not config_.amEnable)
    return CmdErr::NotSup;

  // Bits 18:17 and 13:0 must be 0.
  if (cmd & ((uint32_t(3) << 17) | 0x3fff))
    return CmdErr::NotSup;

  bool isVirtual = (cmd >> 23) & 1;
  unsigned aamsize = (cmd >> 20) & 7;
  bool postincrement = (cmd >> 19) & 1;
  bool write = (cmd >> 16) & 1;
  unsigned bits = 8u << aamsize;

  if ((isVirtual and not config_.amVirtual) or (postincrement and not config_.amPostincrement))
    return CmdErr::NotSup;
  if (((config_.amSizes >> aamsize) & 1) == 0 or bits > xlen)
    return CmdErr::NotSup;
  if (2 * xlen / 32 > config_.datacount)
    return CmdErr::NotSup;  // arg0 and arg1 are DXLEN wide.

  if (not halted(hartsel_))
    return CmdErr::HaltResume;

  auto& h = hart(hartsel_);
  uint64_t addr = readArg(1, xlen);
  uint64_t pa = addr;

  if (isVirtual)
    {
      // Translate as M-mode with MPRV set: privilege MPP (and MPV).
      URV mstatus = h.peekCsr(CsrNumber::MSTATUS);
      auto mpp = PrivilegeMode((mstatus >> 11) & 3);
      bool mpv = false;
      if (h.isRvh())
        {
          if constexpr (sizeof(URV) == 8)
            mpv = (mstatus >> 39) & 1;
          else
            {
              URV mstatush = 0;
              if (h.peekCsr(CsrNumber::MSTATUSH, mstatush))
                mpv = (mstatush >> 7) & 1;
            }
        }
      if (mpp != PrivilegeMode::Machine)
        {
          auto cause = h.transAddrNoUpdate(addr, mpp, mpv, not write, write, false, pa);
          if (cause != ExceptionCause::NONE)
            return CmdErr::Exception;
        }
    }

  // The hart performs the access as an M-mode load or store with MPRV clear, so
  // that it sees the same devices and faults as M-mode code.
  URV s0 = h.peekIntReg(regS0);
  URV s1 = h.peekIntReg(regS1);
  URV mstatus = h.peekCsr(CsrNumber::MSTATUS);
  constexpr URV mprv = URV(1) << 17;
  if (mstatus & mprv)
    h.pokeCsr(CsrNumber::MSTATUS, mstatus & ~mprv);

  h.pokeIntReg(regS0, URV(pa));
  bool ok = false;
  if (write)
    {
      h.pokeIntReg(regS1, URV(readArg(0, xlen)));
      ok = execAbstractInst(hartsel_, store(aamsize, regS0, regS1));
    }
  else
    {
      ok = execAbstractInst(hartsel_, load(aamsize, regS0, regS1, xlen == 64));
      if (ok)
        writeArg(0, xlen, lowBits(h.peekIntReg(regS1), bits));
    }

  h.pokeIntReg(regS0, s0);
  h.pokeIntReg(regS1, s1);
  if (mstatus & mprv)
    h.pokeCsr(CsrNumber::MSTATUS, mstatus);

  if (not ok)
    return CmdErr::Exception;

  if (write)
    for (auto& hartPtr : harts_)
      hartPtr->invalidateDecodeCacheRange(pa, bits / 8);

  if (postincrement)
    writeArg(1, xlen, lowBits(addr + (bits / 8), xlen));
  return CmdErr::None;
}


template <typename URV>
typename DebugModule<URV>::CmdErr
DebugModule<URV>::quickAccess()
{
  if (not config_.quickAccess or config_.progbufsize == 0 or (command_ & 0xffffff))
    return CmdErr::NotSup;

  if (not running(hartsel_))
    return CmdErr::HaltResume;

  haltHart(hartsel_, DebugModeCause::HALTREQ);
  startProgbuf(hartsel_, true);
  return CmdErr::None;
}


template <typename URV>
bool
DebugModule<URV>::execAbstractInst(unsigned ix, uint32_t inst)
{
  auto& h = hart(ix);
  memory_.poke(slotAddr_, inst);
  h.invalidateDecodeCacheRange(slotAddr_, 4);

  URV savedPc = h.peekPc();
  h.pokePc(URV(slotAddr_));
  h.singleStep(nullptr);  // Not traced: this is the DM's own work.
  bool ok = h.peekPc() == URV(slotAddr_ + 4);
  h.pokePc(savedPc);
  return ok;
}


template <typename URV>
void
DebugModule<URV>::startProgbuf(unsigned ix, bool quick)
{
  hart(ix).pokePc(URV(progbufAddr_));
  progbufRunning_ = true;
  quickAccessRun_ = quick;
  progbufHart_ = ix;
  continueProgbuf(progbufBudget);
}


template <typename URV>
bool
DebugModule<URV>::continueProgbuf(uint64_t maxSteps)
{
  if (not progbufRunning_)
    return true;

  unsigned ix = progbufHart_;
  auto& h = hart(ix);
  bool done = false, ok = false;

  for (uint64_t n = 0; n < maxSteps and not done; ++n)
    {
      h.singleStep(traceFiles_.at(ix));
      URV pc = h.peekPc();
      if (h.hasTargetProgramFinished())
        {
          states_.at(ix).finished = true;
          noteUnavailable(ix);
          done = true;
        }
      else if (h.inDebugMode() and pc == URV(parkAddr_))
        done = ok = true;  // ebreak (implicit or explicit).
      else if (not h.inDebugMode() or pc == URV(exceptionAddr_))
        done = true;  // An exception, or dret left debug mode (unspecified by the spec).
    }

  if (not done)
    return false;

  if (h.inDebugMode())
    h.pokePc(URV(parkAddr_));

  bool quick = quickAccessRun_;
  finishCommand(ok ? CmdErr::None : CmdErr::Exception);

  // Quick access resumes the hart unless the program buffer took an exception.
  if (quick and ok and h.inDebugMode())
    h.exitDebugMode();

  noteHalts();
  return true;
}


template <typename URV>
void
DebugModule<URV>::abortProgbuf()
{
  if (not progbufRunning_)
    return;
  auto& h = hart(progbufHart_);
  if (h.inDebugMode())
    h.pokePc(URV(parkAddr_));
  progbufRunning_ = false;
  quickAccessRun_ = false;
  busy_ = false;
  busyCycles_ = 0;
}


template <typename URV>
bool
DebugModule<URV>::sbPresent(unsigned addr) const
{
  bool any = config_.sbaEnable and config_.sbaSizes != 0;
  switch (addr)
    {
    case Sbcs:       return config_.sbaEnable;
    case Sbaddress0: return config_.sbaEnable and config_.sbaAsize > 0;
    case Sbaddress1: return config_.sbaEnable and config_.sbaAsize > 32;
    case Sbaddress2: return config_.sbaEnable and config_.sbaAsize > 64;
    case Sbaddress3: return config_.sbaEnable and config_.sbaAsize > 96;
    case Sbdata0:    return any;
    case Sbdata1:    return any and (config_.sbaSizes & 0b11000);
    case Sbdata2:
    case Sbdata3:    return any and (config_.sbaSizes & 0b10000);
    default:         return true;
    }
}


template <typename URV>
uint32_t
DebugModule<URV>::readSbcs() const
{
  return ((uint32_t(1) << 29) | (uint32_t(sbBusyError_) << 22) |
          (uint32_t(sbBusyLeft_ > 0) << 21) | (uint32_t(sbReadOnAddr_) << 20) |
          (sbAccess_ << 17) | (uint32_t(sbAutoIncrement_) << 16) |
          (uint32_t(sbReadOnData_) << 15) | (sbError_ << 12) |
          ((config_.sbaAsize & 0x7f) << 5) | (config_.sbaSizes & 0x1f));
}


template <typename URV>
void
DebugModule<URV>::writeSbcs(uint32_t value)
{
  if ((value >> 22) & 1)
    sbBusyError_ = false;
  sbError_ &= ~((value >> 12) & 7);
  sbReadOnAddr_ = (value >> 20) & 1;
  sbAccess_ = (value >> 17) & 7;
  sbAutoIncrement_ = (value >> 16) & 1;
  sbReadOnData_ = (value >> 15) & 1;
}


template <typename URV>
uint64_t
DebugModule<URV>::sbAddress() const
{
  uint64_t addr = sbAddress_.at(0) | (uint64_t(sbAddress_.at(1)) << 32);
  return lowBits(addr, config_.sbaAsize);
}


template <typename URV>
void
DebugModule<URV>::sbStart(bool write)
{
  if (((config_.sbaSizes >> sbAccess_) & 1) == 0)
    {
      sbError_ = 4;  // Unsupported size.
      return;
    }

  if (sbAddress() & ((uint64_t(1) << sbAccess_) - 1))
    {
      sbError_ = 3;  // Misaligned.
      return;
    }

  sbPendingWrite_ = write;
  if (config_.sbaBusyCycles == 0)
    sbAccess();
  else
    sbBusyLeft_ = config_.sbaBusyCycles;
}


template <typename URV>
void
DebugModule<URV>::sbAccess()
{
  uint64_t addr = sbAddress();
  unsigned bytes = 1u << sbAccess_;
  uint64_t value = sbData_.at(0) | (uint64_t(sbData_.at(1)) << 32);
  bool ok = false;

  if (sbPendingWrite_)
    {
      switch (bytes)
        {
        case 1: ok = memory_.write(addr, uint8_t(value)); break;
        case 2: ok = memory_.write(addr, uint16_t(value)); break;
        case 4: ok = memory_.write(addr, uint32_t(value)); break;
        default: ok = memory_.write(addr, value); break;
        }
      if (ok)
        for (auto& hartPtr : harts_)
          hartPtr->invalidateDecodeCacheRange(addr, bytes);
    }
  else
    {
      uint8_t v8 = 0;
      uint16_t v16 = 0;
      uint32_t v32 = 0;
      switch (bytes)
        {
        case 1: ok = memory_.read(addr, v8); value = v8; break;
        case 2: ok = memory_.read(addr, v16); value = v16; break;
        case 4: ok = memory_.read(addr, v32); value = v32; break;
        default: ok = memory_.read(addr, value); break;
        }
      if (ok)
        {
          sbData_.at(0) = uint32_t(value);
          sbData_.at(1) = uint32_t(value >> 32);
        }
    }

  if (not ok)
    {
      sbError_ = 2;  // Bad address.
      return;
    }

  if (sbAutoIncrement_)
    {
      uint64_t next = lowBits(addr + bytes, config_.sbaAsize);
      sbAddress_.at(0) = uint32_t(next);
      sbAddress_.at(1) = uint32_t(next >> 32);
    }
}


template <typename URV>
bool
DebugModule<URV>::run(std::vector<util::file::SharedFile>& traceFiles, bool waitAll)
{
  auto stopHandler = Hart<URV>::installStopHandler();

  for (unsigned i = 0; i < hartCount(); ++i)
    traceFiles_.at(i) = i < traceFiles.size() ? traceFiles.at(i).get() : nullptr;

  std::vector<uint64_t> instCount0(hartCount()), retCount0(hartCount());
  for (unsigned i = 0; i < hartCount(); ++i)
    {
      instCount0.at(i) = hart(i).getInstructionCount();
      retCount0.at(i) = hart(i).getRetiredInstructionCount();
    }

  struct timeval t0{};
  gettimeofday(&t0, nullptr);

  bool result = true;
  noteHalts();

  while (not Hart<URV>::userStopRequested())
    {
      if (rbb_.hasInput())
        rbb_.service();

      if (progbufRunning_)
        continueProgbuf(progbufBudget);

      bool ran = false;
      for (unsigned i = 0; i < hartCount(); ++i)
        {
          if (not running(i))
            continue;

          bool stop = false;
          try
            {
              result = hart(i).runSlice(config_.slice, stop, traceFiles_.at(i)) and result;
            }
          catch (const CoreException&)
            {
              stop = true;  // Snapshots are not supported with the debug module.
            }
          ran = true;
          if (stop)
            {
              states_.at(i).finished = true;
              noteUnavailable(i);
            }
        }

      if (ran)
        noteHalts();

      unsigned finished = 0;
      for (const auto& st : states_)
        finished += st.finished;
      if (finished == hartCount() or (finished > 0 and not waitAll))
        break;

      // Nothing to run: wait for the debugger.
      if (not ran and not progbufRunning_ and not rbb_.hasInput())
        rbb_.wait(100);
    }

  struct timeval t1{};
  gettimeofday(&t1, nullptr);
  double elapsed = (double(t1.tv_sec - t0.tv_sec) + (double(t1.tv_usec - t0.tv_usec)*1e-6));

  for (unsigned i = 0; i < hartCount(); ++i)
    hart(i).reportInstsPerSec(hart(i).getInstructionCount() - instCount0.at(i),
                              hart(i).getRetiredInstructionCount() - retCount0.at(i),
                              elapsed, Hart<URV>::userStopRequested());

  return result;
}


template class WdRiscv::DebugModule<uint32_t>;
template class WdRiscv::DebugModule<uint64_t>;
