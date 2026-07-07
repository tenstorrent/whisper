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

#include <iomanip>
#include <iostream>
#include <sstream>
#include <climits>
#include <map>
#include <mutex>
#include <array>
#include <atomic>
#include "atomic_ref_fallback.hpp"
#include <numeric>
#include <cstring>
#include <ctime>
#include <bit>
#include <poll.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>

#include <cassert>
#include <csignal>

#include <cinttypes>
#include <sys/socket.h>
#include <netinet/in.h>

#include <thread>
#include <chrono>

#include <boost/algorithm/string.hpp>

#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "Mcm.hpp"
#include "PerfApi.hpp"
#include "wideint.hpp"

#if defined(__cpp_lib_atomic_ref)
using std::atomic_ref;
#else
using compat::atomic_ref;
#endif

#ifndef SO_REUSEPORT
#define SO_REUSEPORT SO_REUSEADDR
#endif


using namespace WdRiscv;


template <typename TYPE>
static
bool
parseNumber(std::string_view numberStr, TYPE& number)
{
  bool good = not numberStr.empty();

  if (good)
    {
      char* end = nullptr;
      if constexpr (sizeof(TYPE) == 4)
        number = strtoul(numberStr.data(), &end, 0);
      else if constexpr (sizeof(TYPE) == 8)
        number = strtoull(numberStr.data(), &end, 0);
      else
	{
	  std::cerr << "Error: parseNumber: Only 32/64-bit RISCV harts supported\n";
	  return false;
	}
      if (end and *end)
	good = false;  // Part of the string are non parseable.
    }
  return good;
}


template <typename URV>
Hart<URV>::Hart(unsigned hartIx, URV hartId, unsigned numHarts, Memory& memory,
                MmRegs& mmr, Syscall<URV>& syscall, uint64_t& time)
  : hartIx_(hartIx), numHarts_(numHarts), memory_(memory),
    pmaMgr_(memory.size()),
    intRegs_(32),
    csRegs_(pmpMgr_, pmaMgr_),
    fpRegs_(32),
    syscall_(syscall),
    time_(time),
    decodeCacheSize_(128*1024),
    decodeCacheMask_(decodeCacheSize_ - 1),
    virtMem_(hartIx, memory.pageSize(), 2048)
{
  pmaMgr_.attachMmr(&mmr);

  setupVirtMemCallbacks();

  // Enable default extensions
  for (RvExtension ext : { RvExtension::C, RvExtension::M })
    enableExtension(ext, true);

  decodeCache_.resize(decodeCacheSize_);

  interruptStat_.resize(size_t(InterruptCause::MAX_CAUSE) + 1);
  exceptionStat_.resize(size_t(ExceptionCause::MAX_CAUSE) + 1);

  // Tie frequently updated CSR to variables held in the hart so that their values can be
  // obtained directly by the hart and without having to use the read/write/peek/poke
  // interfaces. This is done for speed.
  tieCsrs();

  // Configure MHARTID CSR.
  bool implemented = true, shared = false;
  URV mask = 0, pokeMask = 0;

  csRegs_.configCsr(CsrNumber::MHARTID, implemented, hartId, mask, pokeMask, shared);

  // Give disassembler a way to get abi-names of CSRs.
  auto callback = [this](unsigned ix) {
    auto csr = this->findCsr(CsrNumber(ix));
    return csr? csr->getName() : std::string_view{};
  };
  disas_.setCsrNameCallback(callback);

  using IC = InterruptCause;

  // Define the default machine interrupts in high to low priority. VS interrupts
  // VSTIP/VSEIP/VSSIP are always delegated to supervisor privilege (section 19.4.2 of
  // privileged spec).
  mInterrupts_ = { IC::M_EXTERNAL, IC::M_SOFTWARE, IC::M_TIMER,
                   IC::S_EXTERNAL, IC::S_SOFTWARE, IC::S_TIMER,
                   IC::G_EXTERNAL, IC::LCOF };

  // Define the default supervisor (S/HS) interrupts in high to low priority.
  sInterrupts_ = { IC::M_EXTERNAL, IC::M_SOFTWARE, IC::M_TIMER,
                   IC::S_EXTERNAL, IC::S_SOFTWARE, IC::S_TIMER,
                   IC::G_EXTERNAL, IC::VS_EXTERNAL, IC::VS_SOFTWARE,
                   IC::VS_TIMER, IC::LCOF };

  // Define the virtual supervisor (VS) interrupts in high to low priority.
  vsInterrupts_ = { IC::VS_EXTERNAL, IC::VS_SOFTWARE, IC::VS_TIMER, IC::LCOF };

  // Define possible NMIs.
  nmInterrupts_ = { 0xf0001000, 0xf0000001, 0xf0000000, 3, 2, 1, 0 };
}


template <typename URV>
Hart<URV>::~Hart()
{
  if (branchBuffer_.max_size() and not branchTraceFile_.empty())
    saveBranchTrace(branchTraceFile_);
  if (traceCacheOn_)
    saveCacheTrace(cacheTraceFile_);
}


template <typename URV>
void Hart<URV>::filterMachineInterrupts(bool verbose) {
  // Get the poke masks for the MIP and MIE CSRs.
  const Csr<URV>* mipCsr = csRegs_.findCsr(CsrNumber::MIP);
  const Csr<URV>* mieCsr = csRegs_.findCsr(CsrNumber::MIE);

  URV maskMIP = mipCsr->getPokeMask();
  URV maskMIE = mieCsr->getPokeMask();

  // Combine the masks (only bits allowed in both are effective).
  URV combinedMask = maskMIP & maskMIE;

  // For each bit allowed by the hardware, warn if the user did not provide it.
  if (verbose) {
    // Build a set of the interrupt causes provided by the user.
    std::unordered_set<unsigned> userCauses;
    for (const auto &ic : mInterrupts_)
      userCauses.insert(static_cast<unsigned>(ic));

    for (unsigned bitPos = 0; bitPos < sizeof(URV) * 8; ++bitPos) {
      if (combinedMask & (URV(1) << bitPos)) {
        if (userCauses.find(bitPos) == userCauses.end()) {
          std::cerr << "Warning: Interrupt cause " << bitPos
                    << " is allowed by hardware mask but not provided in configuration.\n";
        }
      }
    }
  }

  // Remove any interrupt cause for which the corresponding bit in the combined mask is 0.
  mInterrupts_.erase(
    std::remove_if(
      mInterrupts_.begin(), mInterrupts_.end(),
      [combinedMask](InterruptCause ic) {
        auto bitPos = static_cast<unsigned>(ic);
        return ((combinedMask & (URV(1) << bitPos)) == 0);
      }
    ),
    mInterrupts_.end()
  );
}

template <typename URV>
void Hart<URV>::filterSupervisorInterrupts(bool verbose) {
  // Get the poke masks for SIP and SIE.
  const Csr<URV>* sipCsr = csRegs_.findCsr(CsrNumber::SIP);
  const Csr<URV>* sieCsr = csRegs_.findCsr(CsrNumber::SIE);

  URV maskSIP = sipCsr->getPokeMask();
  URV maskSIE = sieCsr->getPokeMask();

  // Combined mask: only bits allowed by both.
  URV combinedMask = maskSIP & maskSIE;

  // Always allow S_EXTERNAL regardless of the mask.
  const auto s_external = static_cast<unsigned>(InterruptCause::S_EXTERNAL);
  combinedMask |= (URV(1) << s_external);

  // Warn if a bit is allowed by hardware but not configured.
  if (verbose) {
    std::unordered_set<unsigned> userCauses;
    for (const auto &ic : sInterrupts_)
      userCauses.insert(static_cast<unsigned>(ic));
    for (unsigned bitPos = 0; bitPos < sizeof(URV) * 8; ++bitPos) {
      if (combinedMask & (URV(1) << bitPos)) {
        if (userCauses.find(bitPos) == userCauses.end())
          std::cerr << "Error: Supervisor interrupt cause " << bitPos
                    << " allowed by hardware but missing in configuration.\n";
      }
    }
  }

  // Remove any supervisor interrupt cause whose bit is 0 in the mask.
  sInterrupts_.erase(
    std::remove_if(
      sInterrupts_.begin(), sInterrupts_.end(),
      [combinedMask](InterruptCause ic) {
        auto bitPos = static_cast<unsigned>(ic);
        return ((combinedMask & (URV(1) << bitPos)) == 0);
      }
    ),
    sInterrupts_.end()
  );
}


template <typename URV>
void
Hart<URV>::tieCsrs()
{
  // Tie the retired instruction and cycle counter CSRs to variables held in the hart.
  if constexpr (sizeof(URV) == 4)
    {
      virtMem_.setSupportedModes({VirtMem::Mode::Bare, VirtMem::Mode::Sv32});

      auto split = util::view_arith_as_arr_of<URV>(minstret_);
      csRegs_.findCsr(CsrNumber::MINSTRET)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::INSTRET)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::MINSTRETH)->tie(&split[1]);
      csRegs_.findCsr(CsrNumber::INSTRETH)->tie(&split[1]);

      split = util::view_arith_as_arr_of<URV>(cycleCount_);
      csRegs_.findCsr(CsrNumber::MCYCLE)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::CYCLE)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::MCYCLEH)->tie(&split[1]);
      csRegs_.findCsr(CsrNumber::CYCLEH)->tie(&split[1]);

      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      split = util::view_arith_as_arr_of<URV>(time_);
      csRegs_.findCsr(CsrNumber::TIME)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::TIMEH)->tie(&split[1]);

      split = util::view_arith_as_arr_of<URV>(stimecmp_);
      csRegs_.findCsr(CsrNumber::STIMECMP)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::STIMECMPH)->tie(&split[1]);

      split = util::view_arith_as_arr_of<URV>(vstimecmp_);
      csRegs_.findCsr(CsrNumber::VSTIMECMP)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::VSTIMECMPH)->tie(&split[1]);

      split = util::view_arith_as_arr_of<URV>(htimedelta_);
      csRegs_.findCsr(CsrNumber::HTIMEDELTA)->tie(&split[0]);
      csRegs_.findCsr(CsrNumber::HTIMEDELTAH)->tie(&split[1]);
    }
  else
    {
      virtMem_.setSupportedModes({VirtMem::Mode::Bare, VirtMem::Mode::Sv39,
	  VirtMem::Mode::Sv48, VirtMem::Mode::Sv57 });

      csRegs_.findCsr(CsrNumber::MINSTRET)->tie(&minstret_);
      csRegs_.findCsr(CsrNumber::MCYCLE)->tie(&cycleCount_);

      // INSTRET and CYCLE are read-only shadows of MINSTRET and MCYCLE.
      csRegs_.findCsr(CsrNumber::INSTRET)->tie(&minstret_);
      csRegs_.findCsr(CsrNumber::CYCLE)->tie(&cycleCount_);

      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      csRegs_.findCsr(CsrNumber::TIME)->tie(&time_);

      csRegs_.findCsr(CsrNumber::STIMECMP)->tie(&stimecmp_);
      csRegs_.findCsr(CsrNumber::VSTIMECMP)->tie(&vstimecmp_);
      csRegs_.findCsr(CsrNumber::HTIMEDELTA)->tie(&htimedelta_);
    }

  // Tie the FCSR register to variable held in the hart.
  csRegs_.findCsr(CsrNumber::FCSR)->tie(&fcsrValue_);

  // Tie the SSP register to variable held in the hart.
  csRegs_.findCsr(CsrNumber::SSP)->tie(&ssp_);
}


template <typename URV>
void
Hart<URV>::setupVirtMemCallbacks()
{

  virtMem_.setMemReadCallback([this](uint64_t addr, bool bigEndian, unsigned size, uint64_t& data) -> bool {
    if (steeEnabled_)
      {
        if (!stee_.isValidAddress(addr))
          return false;
        addr = stee_.clearSecureBits(addr);
      }

    auto pma = pmaMgr_.accessPma(addr);
    if (not pma.isRead())
      return false;

    // Proceed with normal memory read based on size.
    bool result = false;
    if (size == 4)
      {
        uint32_t data32 = 0;
        if (mcm_ and dataCache_)
          result = peekMemory(addr, data32, false);
        if (not result)
          result = memory_.read(addr, data32);
        if (bigEndian)
          data32 = util::byteswap(data32);
        if (result)
          data = data32;
      }
    else if (size == 8)
      {
        uint64_t data64 = 0;
        if (mcm_ and dataCache_)
          result = peekMemory(addr, data64, false);
        if (not result)
          result = memory_.read(addr, data64);
        if (bigEndian)
          data64 = util::byteswap(data64);
        if (result)
          data = data64;
      }
    return result;
  });

  virtMem_.setMemWriteCallback([this](uint64_t addr, bool bigEndian, unsigned size, uint64_t data) -> bool {
    if (steeEnabled_)
      {
        if (!stee_.isValidAddress(addr))
          return false;
        addr = stee_.clearSecureBits(addr);
      }

    if (not pmaMgr_.accessPma(addr).isRsrv())
      return false;

    if (size == 4)
      {
        auto value = static_cast<uint32_t>(data);
        if (bigEndian)
          value = util::byteswap(value);

        if (mcm_ and dataCache_)
          {
            bool ok = true;
            for (unsigned i = 0; i < 4 and ok; ++i)
              {
                auto byte = uint8_t(value >> uint8_t(8*i));
                if (pokeMcmCache<McmMem::Data>(addr + i, byte))
                  continue;
                ok = memory_.write(addr + i, byte);
              }
            return ok;
          }
        return memory_.write(addr, value);
      }
    if (size == 8)
      {
        auto value = data;
        if (bigEndian)
          value = util::byteswap(value);

        if (mcm_ and dataCache_)
          {
            bool ok = true;
            for (unsigned i = 0; i < 8 and ok; ++i)
              {
                auto byte = uint8_t(value >> uint8_t(8*i));
                if (pokeMcmCache<McmMem::Data>(addr + i, byte))
                  continue;
                ok = memory_.write(addr + i, byte);
              }
            return ok;
          }
        return memory_.write(addr, value);
      }
    return false;
  });

  virtMem_.setIsReadableCallback([this](uint64_t addr) -> bool {
    // Access control (PMP + STEE) for a page-table-walk PTE read. The PMA
    // read-permission check is intentionally omitted here: every PTE read calls
    // this immediately before memRead (Memory::read) on the same address, and
    // memRead re-checks PMA readability (returning false -> same access fault).
    // Dropping that redundant accessPma is the bulk of this callback's cost; the
    // cheap PMP/STEE checks stay so access control is enforced at the check site
    // (and stays symmetric with isWritable).
    if (pmpMgr_.isEnabled())
      {
        auto pmp = pmpMgr_.accessPmp(PrivilegeMode::Supervisor, addr);
        if (not pmp.isRead())
          return false;
      }
    if (steeEnabled_ and not stee_.isValidAddress(addr))
      return false;
    return true;
  });

  virtMem_.setIsWritableCallback([this](uint64_t addr) -> bool {
    if (pmpMgr_.isEnabled())
      {
        auto pmp = pmpMgr_.accessPmp(PrivilegeMode::Supervisor, addr);
        if (not pmp.isWrite())
          return false;
      }
    if (steeEnabled_)
      {
        if (!stee_.isValidAddress(addr))
          return false;
        addr = stee_.clearSecureBits(addr);
      }

    auto pma = pmaMgr_.accessPma(addr);
    pma = overridePmaWithPbmt(pma, virtMem_.lastPbmt());

    // To write PTE after update of A/D bits we require PMA with write and atomicity
    // attributes.
    bool ok = pma.isWrite() and (pma.isAmo() or pma.isRsrv());

    // if (mcm_ and dataCache_)
    // return dataCache_->isLineResident(addr);

    return ok;
  });

  // Keep the page-walk PTE cache coherent: any committed RAM write invalidates a
  // cached copy of that location (so a write to a page-table entry is observed).
  memory_.setWriteObserver(
    [](void* ctx, uint64_t addr, unsigned size) {
      static_cast<VirtMem*>(ctx)->invalidatePteCache(addr, size);
    },
    &virtMem_);

  // Enable the PTE cache where it can stay coherent (single hart); see
  // updatePteCacheActive. Both store-commit paths (Memory::write and Memory::poke)
  // fire the invalidation observer, so MCM/perfApi no longer need to disable it.
  updatePteCacheActive();
}


template <typename URV>
void
Hart<URV>::getImplementedCsrs(std::vector<CsrNumber>& vec) const
{
  vec.clear();

  for (unsigned i = 0; i <= unsigned(CsrNumber::MAX_CSR_); ++i)
    {
      auto csrn = CsrNumber(i);
      if (csRegs_.isImplemented(csrn))
	vec.push_back(csrn);
    }
}


template <typename URV>
unsigned
Hart<URV>::countImplementedPmpRegisters() const
{
  using std::cerr;

  unsigned count = 0;

  auto num = unsigned(CsrNumber::PMPADDR0);
  for (unsigned ix = 0; ix < 64; ++ix, ++num)
    if (csRegs_.isImplemented(CsrNumber(num)))
      count++;

  if (count and count != 16 and count != 64 and hartIx_ == 0)
    cerr << "Warning: Some but not all PMPADDR CSRs are implemented\n";

  unsigned cfgCount = 0;
  if (mxlen_ == 32)
    {
      num = unsigned(CsrNumber::PMPCFG0);
      for (unsigned ix = 0; ix < 16; ++ix, ++num)
        if (csRegs_.isImplemented(CsrNumber(num)))
          cfgCount++;
      if (count and cfgCount != 4 and cfgCount != 16 and hartIx_ == 0)
        cerr << "Warning: Physical memory protection enabled but only "
	     << cfgCount << "/16" << " PMPCFG CSRs implemented\n";
    }
  else
    {
      num = unsigned(CsrNumber::PMPCFG0);
      for (unsigned ix = 0; ix < 16; ++ix, ++num)
        if (csRegs_.isImplemented(CsrNumber(num)))
          {
            if ((ix & 1) == 1)
              cerr << "Error: Odd numbered PMPCFG" << ix << " CSR should not be implemented.\n";
            cfgCount++;
          }

      // Count should be 0, 16, or 14. cfgCount should be count/8.
      if (cfgCount != count / 8)
        {
          cerr << "Error: The number of implemented PMPADDR CSRs is " << count
               << ", but the number of implemented PMPCFG CSRs is " << cfgCount
               << " (should be " << count << "/8 = " << (count/8) << ")\n";
        }
    }

  return count;
}


template <typename URV>
void
Hart<URV>::processExtensions(bool verbose)
{
  URV value = 0;
  if (not peekCsr(CsrNumber::MISA, value))
    std::cerr << "Error: CSR MISA is not defined\n";

  bool flag = value & (URV(1) << ('s' - 'a'));  // Supervisor-mode option.
  flag = flag and isa_.isEnabled(RvExtension::S);
  enableSupervisorMode(flag);

  flag = value & (URV(1) << ('u' - 'a'));  // User-mode option.
  flag = flag and isa_.isEnabled(RvExtension::U);
  enableUserMode(flag);

  flag = value & (URV(1) << ('h' - 'a'));  // Hypervisor.
  flag = flag and isa_.isEnabled(RvExtension::H);
  enableHypervisorMode(flag);

  flag = (value & 1) and isa_.isEnabled(RvExtension::A);   // Atomic
  enableExtension(RvExtension::A, flag);

  flag = (value & 2) and isa_.isEnabled(RvExtension::B);   // Bit-manip
  enableExtension(RvExtension::B, flag);

  flag = (value & (URV(1) << ('c' - 'a')));  // Compress option.
  flag = flag and (isa_.isEnabled(RvExtension::C) or isa_.isEnabled(RvExtension::Zca));
  enableRvc(flag);

  flag = value & (URV(1) << ('f' - 'a'));  // Single precision FP
  flag = flag and isa_.isEnabled(RvExtension::F);
  enableRvf(flag);

  // D requires F and is enabled only if F is enabled.
  flag = value & (URV(1) << ('d' - 'a'));  // Double precision FP
  flag = flag and isa_.isEnabled(RvExtension::D);
  if (flag and not extensionIsEnabled(RvExtension::F))
    {
      flag = false;
      if (verbose and hartIx_ == 0)
	std::cerr << "Warning: Bit 3 (d) is set in the MISA register but f "
		  << "extension (bit 5) is not enabled -- ignored\n";
    }
  enableRvd(flag);

  flag = value & (URV(1) << ('e' - 'a'));
  flag = flag and isa_.isEnabled(RvExtension::E);
  if (flag)
    intRegs_.regs_.resize(16);
  enableExtension(RvExtension::E, flag);

  flag = value & (URV(1) << ('i' - 'a'));
  if (not flag and not extensionIsEnabled(RvExtension::E) and verbose and hartIx_ == 0)
    std::cerr << "Warning: Bit 8 (i extension) is cleared in the MISA register "
	      << " but extension is mandatory -- assuming bit 8 set\n";

  flag = value & (URV(1) << ('m' - 'a'));
  flag = flag and isa_.isEnabled(RvExtension::M);
  enableExtension(RvExtension::M, flag);

  flag = value & (URV(1) << ('v' - 'a'));  // User-mode option.
  if (flag and not (extensionIsEnabled(RvExtension::F) and extensionIsEnabled(RvExtension::D)))
    {
      flag = false;
      if (verbose and hartIx_ == 0)
	std::cerr << "Warning: Bit 21 (v) is set in the MISA register but the d/f "
		  << "extensions are not enabled -- ignored\n";
    }
  flag = flag and isa_.isEnabled(RvExtension::V);
  enableVectorExtension(flag);

  if (verbose and hartIx_ == 0)
    for (auto ec : { 'j', 'k', 'l', 'n', 'o', 'p',
		     'q', 'r', 't', 'w', 'x', 'y', 'z' } )
      {
	unsigned bit = ec - 'a';
	if (value & (URV(1) << bit))
	  std::cerr << "Warninig: Bit " << bit << " (" << ec << ") set in the MISA "
		    << "register but extension is not supported "
		    << "-- ignored\n";
      }

  enableExtension(RvExtension::Zba,      isa_.isEnabled(RvExtension::Zba));
  enableExtension(RvExtension::Zbb,      isa_.isEnabled(RvExtension::Zbb));
  enableExtension(RvExtension::Zbc,      isa_.isEnabled(RvExtension::Zbc));
  enableExtension(RvExtension::Zbs,      isa_.isEnabled(RvExtension::Zbs));
  enableExtension(RvExtension::Zfbfmin,  isa_.isEnabled(RvExtension::Zfbfmin));
  enableExtension(RvExtension::Zfh,      isa_.isEnabled(RvExtension::Zfh));
  enableExtension(RvExtension::Zfhmin,   isa_.isEnabled(RvExtension::Zfhmin));
  enableExtension(RvExtension::Zknd,     isa_.isEnabled(RvExtension::Zknd));
  enableExtension(RvExtension::Zkne,     isa_.isEnabled(RvExtension::Zkne));
  enableExtension(RvExtension::Zknh,     isa_.isEnabled(RvExtension::Zknh));
  enableExtension(RvExtension::Zbkb,     isa_.isEnabled(RvExtension::Zbkb));
  enableExtension(RvExtension::Zbkc,     isa_.isEnabled(RvExtension::Zbkc));
  enableExtension(RvExtension::Zbkx,     isa_.isEnabled(RvExtension::Zbkx));
  enableExtension(RvExtension::Zksed,    isa_.isEnabled(RvExtension::Zksed));
  enableExtension(RvExtension::Zksh,     isa_.isEnabled(RvExtension::Zksh));
  enableExtension(RvExtension::Zicbom,   isa_.isEnabled(RvExtension::Zicbom));
  enableExtension(RvExtension::Zicboz,   isa_.isEnabled(RvExtension::Zicboz));
  enableExtension(RvExtension::Zicbop,   isa_.isEnabled(RvExtension::Zicbop));
  enableExtension(RvExtension::Zawrs,    isa_.isEnabled(RvExtension::Zawrs));
  enableExtension(RvExtension::Zmmul,    isa_.isEnabled(RvExtension::Zmmul));
  enableExtension(RvExtension::Zvbb,     isa_.isEnabled(RvExtension::Zvbb));
  enableExtension(RvExtension::Zvbc,     isa_.isEnabled(RvExtension::Zvbc));
  enableExtension(RvExtension::Zvfbfmin, isa_.isEnabled(RvExtension::Zvfbfmin));
  enableExtension(RvExtension::Zvfbfwma, isa_.isEnabled(RvExtension::Zvfbfwma));
  enableExtension(RvExtension::Zvqdotq,  isa_.isEnabled(RvExtension::Zvqdotq));
  enableExtension(RvExtension::Zvfh,     isa_.isEnabled(RvExtension::Zvfh));
  enableExtension(RvExtension::Zvfhmin,  isa_.isEnabled(RvExtension::Zvfhmin));
  enableExtension(RvExtension::Zvkg,     isa_.isEnabled(RvExtension::Zvkg));
  enableExtension(RvExtension::Zvkned,   isa_.isEnabled(RvExtension::Zvkned));
  enableExtension(RvExtension::Zvknha,   isa_.isEnabled(RvExtension::Zvknha));
  enableExtension(RvExtension::Zvknhb,   isa_.isEnabled(RvExtension::Zvknhb));
  enableExtension(RvExtension::Zvksed,   isa_.isEnabled(RvExtension::Zvksed));
  enableExtension(RvExtension::Zvksh,    isa_.isEnabled(RvExtension::Zvksh));
  enableExtension(RvExtension::Zvkb,     isa_.isEnabled(RvExtension::Zvkb));
  enableExtension(RvExtension::Zvzip,    isa_.isEnabled(RvExtension::Zvzip));
  enableExtension(RvExtension::Zvabd,    isa_.isEnabled(RvExtension::Zvabd));
  enableExtension(RvExtension::Zicond,   isa_.isEnabled(RvExtension::Zicond));
  enableExtension(RvExtension::Zca,      isa_.isEnabled(RvExtension::Zca));
  enableExtension(RvExtension::Zcb,      isa_.isEnabled(RvExtension::Zcb));
  enableExtension(RvExtension::Zfa,      isa_.isEnabled(RvExtension::Zfa));
  enableExtension(RvExtension::Zacas,    isa_.isEnabled(RvExtension::Zacas));
  enableExtension(RvExtension::Zimop,    isa_.isEnabled(RvExtension::Zimop));
  enableExtension(RvExtension::Zcmop,    isa_.isEnabled(RvExtension::Zcmop));
  enableExtension(RvExtension::Smaia,    isa_.isEnabled(RvExtension::Smaia));
  enableExtension(RvExtension::Ssaia,    isa_.isEnabled(RvExtension::Ssaia));
  enableExtension(RvExtension::Zicsr,    true /*isa_.isEnabled(RvExtension::Zicsr)*/); // Default true until we fix riscof
  enableExtension(RvExtension::Zifencei, true /*isa_.isEnabled(RvExtension::Zifencei)*/); // Default true until RTL catches up
  enableExtension(RvExtension::Zaamo,    isa_.isEnabled(RvExtension::Zaamo));
  enableExtension(RvExtension::Zalrsc,   isa_.isEnabled(RvExtension::Zalrsc));
  enableExtension(RvExtension::Zabha,    isa_.isEnabled(RvExtension::Zabha));
  enableExtension(RvExtension::Zalasr,   isa_.isEnabled(RvExtension::Zalasr));
  enableExtension(RvExtension::Zilsd,    isa_.isEnabled(RvExtension::Zilsd));
  enableExtension(RvExtension::Zclsd,    isa_.isEnabled(RvExtension::Zclsd));
  enableExtension(RvExtension::Zvfbfa,   isa_.isEnabled(RvExtension::Zvfbfa));
  enableExtension(RvExtension::Zvfofp8min, isa_.isEnabled(RvExtension::Zvfofp8min));
  enableExtension(RvExtension::Smcsps,   isa_.isEnabled(RvExtension::Smcsps));
  enableExtension(RvExtension::Sscsps,   isa_.isEnabled(RvExtension::Sscsps));
  enableExtension(RvExtension::Smijt,    isa_.isEnabled(RvExtension::Smijt));
  enableExtension(RvExtension::Ssijt,    isa_.isEnabled(RvExtension::Ssijt));
  enableExtension(RvExtension::Smnip,    isa_.isEnabled(RvExtension::Smnip));
  enableExtension(RvExtension::Ssnip,    isa_.isEnabled(RvExtension::Ssnip));
  enableExtension(RvExtension::Smidctrl, isa_.isEnabled(RvExtension::Smidctrl));
  enableExtension(RvExtension::Ssidctrl, isa_.isEnabled(RvExtension::Ssidctrl));
  enableExtension(RvExtension::Zvqwdota8i, isa_.isEnabled(RvExtension::Zvqwdota8i));
  enableExtension(RvExtension::Zvqwdota16i, isa_.isEnabled(RvExtension::Zvqwdota16i));
  enableExtension(RvExtension::Zvqwbdota8i, isa_.isEnabled(RvExtension::Zvqwbdota8i));
  enableExtension(RvExtension::Zvqwbdota16i, isa_.isEnabled(RvExtension::Zvqwbdota16i));
  enableExtension(RvExtension::Zvfbdota32f, isa_.isEnabled(RvExtension::Zvfbdota32f));
  enableExtension(RvExtension::Zvfwdota16bf, isa_.isEnabled(RvExtension::Zvfwdota16bf));
  enableExtension(RvExtension::Zvfqwdota8f, isa_.isEnabled(RvExtension::Zvfqwdota8f));
  enableExtension(RvExtension::Zvfqwbdota8f, isa_.isEnabled(RvExtension::Zvfqwbdota8f));
  enableExtension(RvExtension::Zvfwbdota16bf, isa_.isEnabled(RvExtension::Zvfwbdota16bf));

  // Smeihv (external interrupt HW vectoring, mode=10).
  enableExtension(RvExtension::Smeihv,   isa_.isEnabled(RvExtension::Smeihv));
  // Sseihv requires Smeihv.
  flag = isa_.isEnabled(RvExtension::Smeihv) and isa_.isEnabled(RvExtension::Sseihv);
  enableExtension(RvExtension::Sseihv,   flag);

  // Smehv requires Smijt.
  flag = isa_.isEnabled(RvExtension::Smijt) and isa_.isEnabled(RvExtension::Smehv);
  enableExtension(RvExtension::Smehv,    flag);

  // Ssehv requires Ssijt.
  flag = isa_.isEnabled(RvExtension::Ssijt) and isa_.isEnabled(RvExtension::Ssehv);
  enableExtension(RvExtension::Ssehv,    flag);

  if (isa_.isEnabled(RvExtension::Sstc))
    enableRvsstc(true);
  if (isa_.isEnabled(RvExtension::Svinval))
    enableSvinval(true);
  if (isa_.isEnabled(RvExtension::Svnapot))
    enableTranslationNapot(true);
  if (isa_.isEnabled(RvExtension::Svpbmt))
    enableTranslationPbmt(true);
  if (isa_.isEnabled(RvExtension::Svadu))
    enableTranslationAdu(true);
  if (isa_.isEnabled(RvExtension::Smrnmi))
    enableSmrnmi(true);
  if (isa_.isEnabled(RvExtension::Smdbltrp))
    enableSmdbltrp(true);
  if (isa_.isEnabled(RvExtension::Ssdbltrp))
    enableSsdbltrp(true);
  if (isa_.isEnabled(RvExtension::Smip))
    enableSmip(true);
  if (isa_.isEnabled(RvExtension::Ssip))
    enableSsip(true);
  if (isa_.isEnabled(RvExtension::Zicntr))
    enableZicntr(true);
  if (isa_.isEnabled(RvExtension::Zihpm))
    enableZihpm(true);
  if (isa_.isEnabled(RvExtension::Sscofpmf))
    enableSscofpmf(true);
  if (isa_.isEnabled(RvExtension::Zkr))
    enableZkr(true);
  if (isa_.isEnabled(RvExtension::Smepmp))
    enableSmepmp(true);
  if (isa_.isEnabled(RvExtension::Smstateen))
    enableSmstateen(true);
  if (isa_.isEnabled(RvExtension::Ssqosid))
    enableSsqosid(true);
  if (isa_.isEnabled(RvExtension::Sdtrig))
    enableSdtrig(true);
  if (isa_.isEnabled(RvExtension::Smcntrpmf))
    enableSmcntrpmf(true);
  if (isa_.isEnabled(RvExtension::Smcsrind))
    enableSmcsrind(true);
  if (isa_.isEnabled(RvExtension::Sscsrind))
    enableSscsrind(true);
  if (isa_.isEnabled(RvExtension::Smcdeleg))
    {
      if (isa_.isEnabled(RvExtension::Sscsrind))
        enableSmcdeleg(true);
      else
        std::cerr << "Warning: Extension Smcdeleg requires Sscsrind which is not enabled\n";
    }

  if (isa_.isEnabled(RvExtension::Zvknha) and isa_.isEnabled(RvExtension::Zvknhb))
    {
      std::cerr << "Info: Both Zvknha/b enabled.";
      if (rv64_)
        {
          std::cerr << "Info:  Using Zvknhb.\n";
          enableExtension(RvExtension::Zvknha, false);
        }
      else
        {
          std::cerr << "Info:  Using Zvknha.\n";
          enableExtension(RvExtension::Zvknhb, false);
        }
    }

  enableSmmpm(isa_.isEnabled(RvExtension::Smmpm));
  enableSsnpm(isa_.isEnabled(RvExtension::Ssnpm));
  enableSmnpm(isa_.isEnabled(RvExtension::Smnpm));
  enableAiaExtension(isa_.isEnabled(RvExtension::Smaia));
  enableZicfilp(isa_.isEnabled(RvExtension::Zicfilp));
  enableZicfiss(isa_.isEnabled(RvExtension::Zicfiss));
  enableZibi(isa_.isEnabled(RvExtension::Zibi));

  bool zca = isRvc() or isa_.isEnabled(RvExtension::Zca);  // C implies Zca
  enableExtension(RvExtension::Zca, zca);

  if (isa_.isEnabled(RvExtension::Zcd) and not zca)
    std::cerr << "Warning: Zcd extension enabled but pre-requisite Zca extension is not\n";

  bool zcd = isRvc() and isRvd();   // C+D implise Zcd
  zcd = zcd or (zca and isa_.isEnabled(RvExtension::Zcd));  // Zcd explcitly enabled.
  enableExtension(RvExtension::Zcd, zcd);

  if (isRv64())
    {
      if (isa_.isEnabled(RvExtension::Zcf))
        std::cerr << "Warning: Zcf extension enabled in Rv64\n";
    }
  else
    {
      if (isa_.isEnabled(RvExtension::Zcf) and not zca)
        std::cerr << "Warning: Zcf extension enabled but pre-requisite Zca extension is not\n";

      bool zcf = isRvc() and isRvf();   // C+F implise Zcf
      zcf = zcf or (zca and isa_.isEnabled(RvExtension::Zcf));  // Zcf explcitly enabled.
      enableExtension(RvExtension::Zcf, zcf);
    }

  if (not rv64_ and extensionIsEnabled(RvExtension::Zclsd)
      and extensionIsEnabled(RvExtension::Zcf))
    {
      if (hartIx_ == 0)
        std::cerr << "Warning: Zclsd and Zcf are incompatible -- keeping Zcf and disabling Zclsd\n";
      enableExtension(RvExtension::Zclsd, false);
    }
  enableSmcsps(isa_.isEnabled(RvExtension::Smcsps));
  enableSscsps(isa_.isEnabled(RvExtension::Sscsps));
  enableSmnip(isa_.isEnabled(RvExtension::Smnip));
  enableSsnip(isa_.isEnabled(RvExtension::Ssnip));
  enableSmijt(isa_.isEnabled(RvExtension::Smijt));
  enableSsijt(isa_.isEnabled(RvExtension::Ssijt));
  enableSmeihv(isa_.isEnabled(RvExtension::Smeihv));
  enableSseihv(isa_.isEnabled(RvExtension::Sseihv));

  stimecmpActive_ = csRegs_.menvcfgStce();
  vstimecmpActive_ = csRegs_.henvcfgStce();
}


template <typename URV>
void
Hart<URV>::updateMemoryProtection()
{
  pmpMgr_.reset();
  virtMem_.flushPteCache();  // PMP regions changed -> cached PTE access results stale.

  const unsigned count = 64;
  unsigned impCount = 0;  // Count of implemented PMP registers

  for (unsigned ix = 0; ix < count; ++ix)
    {
      uint64_t low = 0, high = 0;

      Pmp pmp;
      if (unpackMemoryProtection(ix, pmp, low, high))
        {
          impCount++;
          if (pmp.type() != Pmp::Type::Off)
            pmpMgr_.defineRegion(low, high, pmp, ix);
        }
    }

#ifndef FAST_SLOPPY
  pmpEnabled_ = impCount > 0;
#endif

  pmpMgr_.enable(pmpEnabled_);
}


template <typename URV>
bool
Hart<URV>::unpackMemoryProtection(unsigned entryIx, Pmp& pmp,
                                  uint64_t& low, uint64_t& high) const
{
  low = high = 0;

  if (entryIx >= 64)
    return false;

  auto csrn = CsrNumber(unsigned(CsrNumber::PMPADDR0) + entryIx);

  URV pmpVal = 0;
  if (not peekCsr(csrn, pmpVal))
    return false;  // PMPADDRn not implemented.

  URV lowerVal = 0;   // Value of preceding PMPADDR CSR if any.
  if (entryIx > 0)
    {
      auto lowerCsrn = CsrNumber(unsigned(csrn) - 1);
      if (not peekCsr(lowerCsrn, lowerVal))
        return false;  // Should not happen
    }

  unsigned config = csRegs_.getPmpConfigByteFromPmpAddr(csrn);

  return pmpMgr_.unpackMemoryProtection(config, pmpVal, lowerVal, not rv64_,
                                        pmp, low, high);
}


template <typename URV>
void
Hart<URV>::updateAddressTranslation()
{
  URV value = 0;
  if (peekCsr(CsrNumber::SATP, value))
    {
      SatpFields<URV> satp(value);
      if constexpr (sizeof(URV) != 4)
	if ((satp.bits_.MODE >= 1 and satp.bits_.MODE <= 7) or satp.bits_.MODE >= 12)
	  satp.bits_.MODE = 0;

      if (virtMode_)
        virtMem_.configStage1(VirtMem::Mode(satp.bits_.MODE), satp.bits_.ASID, satp.bits_.PPN,
                              vsstatus_.bits_.SUM);
      else
        virtMem_.configTranslation(VirtMem::Mode(satp.bits_.MODE), satp.bits_.ASID, satp.bits_.PPN);
    }

  if (peekCsr(CsrNumber::VSATP, value))
    {
      SatpFields<URV> satp(value);
      if constexpr (sizeof(URV) != 4)
	if ((satp.bits_.MODE >= 1 and satp.bits_.MODE <= 7) or satp.bits_.MODE >= 12)
	  satp.bits_.MODE = 0;

      virtMem_.configStage1(VirtMem::Mode(satp.bits_.MODE), satp.bits_.ASID, satp.bits_.PPN,
                            vsstatus_.bits_.SUM);
    }

  if (peekCsr(CsrNumber::HGATP, value))
    {
      HgatpFields<URV> hgatp(value);
      virtMem_.configStage2(VirtMem::Mode(hgatp.bits_.MODE), hgatp.bits_.VMID, hgatp.bits_.PPN);
    }
}


template <typename URV>
void
Hart<URV>::reset(bool resetMemoryMappedRegs)
{
  privMode_ = PrivilegeMode::Machine;
  virtMode_ = false;

  intRegs_.reset();
  csRegs_.reset();
  vecRegs_.reset();

  // Suppress resetting memory mapped register on initial resets sent
  // by the test bench. Otherwise, initial resets obliterate memory
  // mapped register data loaded from the ELF/HEX file.
  if (resetMemoryMappedRegs)
    memory_.resetMemoryMappedRegisters();
  cancelLr(CancelLrCause::RESET); // Clear LR reservation (if any).

  clearPendingNmi();

  setPc(resetPc_);
  currPc_ = pc_;
  bbPc_ = pc_;

  // Enable extensions if corresponding bits are set in the MISA CSR.
  processExtensions();

  csRegs_.reset();
  effectiveMie_ = csRegs_.effectiveMie();
  effectiveSie_ = csRegs_.effectiveSie();
  effectiveVsie_ = csRegs_.effectiveVsie();

  updateCachedHvictl();

  perfControl_ = ~uint32_t(0);
  URV value = 0;
  if (peekCsr(CsrNumber::MCOUNTINHIBIT, value))
    perfControl_ = ~value;

  prevPerfControl_ = perfControl_;

  debugMode_ = false;
  updateCachedTriggerState();

  dcsrStepIe_ = false;
  dcsrStep_ = false;

  if (peekCsr(CsrNumber::DCSR, value))
    {
      DcsrFields<URV> dcsr(value);
      dcsrStep_ = dcsr.bits_.STEP;
      dcsrStepIe_ = dcsr.bits_.STEPIE;
    }

  resetVector();
  resetFloat();

  // Refresh the mstatus_ cache from the CSR before modifying individual bits below.
  // writeMstatus() reads from mstatus_, so the cache must be current to avoid
  // overwriting bits that csRegs_.reset() just restored.
  updateCachedMstatus();

  if (isRvsmdbltrp())
    {
      mstatus_.bits_.MDT = 1;   // MSTATUS.MDT set to 1 on reset.
      writeMstatus();
    }

  // Update cached values of MSTATUS.
  updateCachedMstatus();
  if (isRvh())
    updateCachedHstatus();

  // Update cached shadow stack control flags from envcfg CSRs
  updateShadowStackEnable();

  updateAddressTranslation();

  updateMemoryProtection();
  countImplementedPmpRegisters();

  csRegs_.updateCounterPrivilege();

  alarmLimit_ = alarmInterval_? alarmInterval_ + time_ : ~uint64_t(0);
  consecIllCount_ = 0;

  // Trigger software interrupt in hart 0 on reset.
  if (aclintSiOnReset_ and hartIx_ == 0)
    pokeMemory(aclintSwStart_, uint32_t(1), true);

  clearTraceData();

  decoder_.enableRv64(isRv64());
  decoder_.enableRvzclsd(isRvzclsd());
  disas_.enableRv64(isRv64());

  // Reflect initial state of menvcfg CSR on pbmt and sstc.
  updateTranslationPbmt();
  updateTranslationAdu();
  updateTranslationPmm();
  csRegs_.updateSstc();

  // If any PMACFG CSR is defined, change the default PMA to no access.
  bool hasPmacfg = false;
  using CN = CsrNumber;
  for (auto ix = unsigned(CN::PMACFG0); ix <= unsigned(CN::PMACFG15); ++ix)
    {
      if (csRegs_.getImplementedCsr(CN(ix)))
        {
          hasPmacfg = true;
          URV val = csRegs_.peek(CN(ix));
          processPmacfgChange(CN(ix), val);
        }
    }

  if (hasPmacfg)
    {
      pmaMgr_.clearDefaultPma();  // No access.
      pmaMgr_.enableInDefaultPma(Pma::Attrib::MisalAccFault); // Access fault on misal.

      // Make sure all 64 PMA configs have associated regions.
      if (pmaMgr_.regionCount() < 64)
        pmaMgr_.defineRegion(64, 0, 0, Pma{});
    }

  // Update IID priority for benefit of *topi registers.
  csRegs_.updateIidPrio(mInterrupts_, sInterrupts_, vsInterrupts_);

  // Apply privilege mode filtering on MCYCLE and MINSTRET (Smcntrpmf extension).
  applySpmcntrpmf();
}


template <typename URV>
void
Hart<URV>::resetVector()
{
  if (isRvv())
    {
      bool configured = vecRegs_.registerCount() > 0;
      if (not configured) {
        constexpr uint32_t bytesPerReg = std::is_same<URV, uint32_t>::value ? 32 : 64;
        constexpr uint32_t maxBytesPerElem = std::is_same<URV, uint32_t>::value ? 4 : 8;
        vecRegs_.config(
            bytesPerReg,
            1 /*minBytesPerElem*/,
            maxBytesPerElem,
            nullptr /*minSewPerLmul*/,
            nullptr /*maxSewPerLmul*/
        );
      }
      unsigned bytesPerReg = vecRegs_.bytesPerRegister();
      csRegs_.configCsr(CsrNumber::VLENB, true, bytesPerReg, 0, 0, false /*shared*/);
      auto vstartBits = uint32_t(std::bit_width(bytesPerReg*8) - 1);  // Log2(bytesPerReg*8)
      URV vstartMask = (URV(1) << vstartBits) - 1;
      auto csr = csRegs_.findCsr(CsrNumber::VSTART);
      if (not csr or csr->getWriteMask() != vstartMask)
	{
	  if (hartIx_ == 0 and configured)
	    std::cerr << "Warning: Write mask of CSR VSTART changed to 0x" << std::hex
		      << vstartMask << " to be compatible with VLEN=" << std::dec
		      << (bytesPerReg*8) << '\n';
	  csRegs_.configCsr(CsrNumber::VSTART, true, 0, vstartMask, vstartMask, false);
	}
    }

  // Make cached vector engine parameters match reset value of the VTYPE CSR.
  URV value = 0;
  if (peekCsr(CsrNumber::VTYPE, value))
    {
      VtypeFields<URV> vtype(value);
      bool vill = vtype.bits_.VILL;
      bool ma = vtype.bits_.VMA;
      bool ta = vtype.bits_.VTA;
      auto gm = GroupMultiplier(vtype.bits_.LMUL);
      auto ew = ElementWidth(vtype.bits_.SEW);
      vecRegs_.updateConfig(ew, gm, ma, ta, vill);
      vecRegs_.setAltfmt(vtype.bits_.ALTFMT);

      disas_.setVecSew(ew);
      disas_.setVecAltfmt(vtype.bits_.ALTFMT);
    }

  // Update cached VL
  if (peekCsr(CsrNumber::VL, value))
    vecRegs_.elemCount(value);

  // Set VS to initial in MSTATUS if linux/newlib emulation. This
  // allows linux/newlib program to run without startup code.
  if (isRvv() and (newlib_ or linux_))
    {
      URV val = csRegs_.peekMstatus();
      MstatusFields<URV> fields(val);
      fields.bits_.VS = unsigned(VecStatus::Initial);
      csRegs_.write(CsrNumber::MSTATUS, PrivilegeMode::Machine, fields.value_);
    }
}


namespace WdRiscv
{

  template <>
  void
  Hart<uint32_t>::updateCachedMstatus()
  {
    uint32_t csrVal = csRegs_.peekMstatus();
    mstatus_.value_.low_ = csrVal;

    csrVal = peekCsr(CsrNumber::MSTATUSH);
    mstatus_.value_.high_ = csrVal;

    virtMem_.setExecReadable(mstatus_.bits_.MXR);
    virtMem_.setStage1ExecReadable(mstatus_.bits_.MXR);
    virtMem_.setSum(mstatus_.bits_.SUM);
    if (virtMode_)
      updateCachedVsstatus();

    pmaskManager_.setExecReadable(mstatus_.bits_.MXR);
    pmaskManager_.setStage1ExecReadable(mstatus_.bits_.MXR);

    updateBigEndian();
  }


  template <>
  void
  Hart<uint64_t>::updateCachedMstatus()
  {
    uint64_t csrVal = csRegs_.peekMstatus();
    mstatus_.value_ = csrVal;

    virtMem_.setExecReadable(mstatus_.bits_.MXR);
    virtMem_.setStage1ExecReadable(mstatus_.bits_.MXR);
    virtMem_.setSum(mstatus_.bits_.SUM);

    if (virtMode_)
      updateCachedVsstatus();

    pmaskManager_.setExecReadable(mstatus_.bits_.MXR);
    pmaskManager_.setStage1ExecReadable(mstatus_.bits_.MXR);

    updateBigEndian();
  }


  template <>
  void
  Hart<uint32_t>::writeMstatus()
  {
    csRegs_.write(CsrNumber::MSTATUS, PrivilegeMode::Machine, mstatus_.value_.low_);
    csRegs_.write(CsrNumber::MSTATUSH, PrivilegeMode::Machine, mstatus_.value_.high_);
    updateCachedMstatus();
  }


  template <>
  void
  Hart<uint64_t>::writeMstatus()
  {
    csRegs_.write(CsrNumber::MSTATUS, PrivilegeMode::Machine, mstatus_.value_);
    updateCachedMstatus();
  }

}


template <typename URV>
void
Hart<URV>::updateCachedVsstatus()
{
  vsstatus_.value_ = peekCsr(CsrNumber::VSSTATUS);

  virtMem_.setStage1ExecReadable(vsstatus_.bits_.MXR);
  virtMem_.setVsSum(vsstatus_.bits_.SUM);

  pmaskManager_.setStage1ExecReadable(vsstatus_.bits_.MXR);

  updateBigEndian();
}


template <typename URV>
void
Hart<URV>::updateCachedHstatus()
{
  hstatus_.value_ = peekCsr(CsrNumber::HSTATUS);
  updateBigEndian();
}


template <typename URV>
void
Hart<URV>::updateBigEndian()
{
  PrivilegeMode pm = privMode_;
  bool virt = virtMode_;
  if (mstatusMprv() and not nmieOverridesMprv())
    {
      pm = mstatusMpp();
      virt = mstatus_.bits_.MPV;
    }

  if (pm == PrivilegeMode::Machine)
    bigEnd_ = mstatus_.bits_.MBE;
  else if (pm == PrivilegeMode::Supervisor)
    bigEnd_ = virt? hstatus_.bits_.VSBE : mstatus_.bits_.SBE;
  else if (pm == PrivilegeMode::User)
    bigEnd_ = virt? vsstatus_.bits_.UBE : mstatus_.bits_.UBE;

  if (pm != PrivilegeMode::Machine)
    {
      bool tbe = virt? hstatus_.bits_.VSBE : mstatus_.bits_.SBE; // translatiom big end
      virtMem_.setBigEndianStage1(tbe);
      virtMem_.setBigEndianStage2(tbe);
    }
}


template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t pa, uint8_t& val, bool usePma, bool skipData) const
{
  if (mcm_ and dataCache_ and not skipData)
    return peekMcmCache<McmMem::Data>(pa, val);

  auto pma = pmaMgr_.getPma(pa);
  if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(pa))
    return pmaMgr_.readRegister(pa, val);

  if (usePma and not pma.isRead() and not pma.isExec())
    return false;

  return memory_.peek(pa, val);
}


template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t pa, uint16_t& val, bool usePma, bool skipData) const
{
  if (mcm_ and dataCache_ and not skipData)
    return peekMcmCache<McmMem::Data>(pa, val);

  auto pma = pmaMgr_.getPma(pa);
  if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(pa))
    return pmaMgr_.readRegister(pa, val);

  if (usePma and not pma.isRead() and not pma.isExec())
    return false;

  return memory_.peek(pa, val);
}


template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t pa, uint32_t& val, bool usePma, bool skipData) const
{
  if (mcm_ and dataCache_ and not skipData)
    return peekMcmCache<McmMem::Data>(pa, val);

  auto pma = pmaMgr_.getPma(pa);
  if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(pa))
    return pmaMgr_.readRegister(pa, val);

  if (usePma and not pma.isRead() and not pma.isExec())
    return false;

  return memory_.peek(pa, val);
}


template <typename URV>
bool
Hart<URV>::peekMemory(uint64_t pa, uint64_t& val, bool usePma, bool skipData) const
{
  if (mcm_ and dataCache_ and not skipData)
    return peekMcmCache<McmMem::Data>(pa, val);

  auto pma = pmaMgr_.getPma(pa);
  if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(pa))
    {
      if (pmaMgr_.readRegister(pa, val))
        return true;
 
      uint32_t high = 0, low = 0;
      if (pmaMgr_.readRegister(pa, low) and pmaMgr_.readRegister(pa + 4, high))
        {
          val = (uint64_t(high) << 32) | low;
          return true;
        }

      return false;
    }

  if (usePma and not pma.isRead() and not pma.isExec())
    return false;

  return memory_.peek(pa, val);
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint8_t val, bool usePma, bool skipFetch, bool skipData, bool skipMem)
{
  std::unique_lock lock(memory_.amoMutex_);

  memory_.invalidateAllHartsLr(addr, sizeof(val));
  invalidateDecodeCache(addr, sizeof(val));

  if (mcm_ and not skipFetch and fetchCache_)
    pokeMcmCache<McmMem::Fetch>(addr, val);

  bool ok = false;
  if (mcm_ and not skipData and dataCache_)
    ok = pokeMcmCache<McmMem::Data>(addr, val);

  if (not skipMem and not ok)
    {
      auto pma = pmaMgr_.getPma(addr);
      if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(addr))
        return pmaMgr_.writeRegister(addr, val);

      if (usePma and not pma.isWrite())
        ok = false;
      else
        ok = memory_.poke(addr, val);
    }

  return ok;
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint16_t val, bool usePma, bool skipFetch,
                      bool skipData, bool skipMem)
{
  std::unique_lock lock(memory_.amoMutex_);

  memory_.invalidateAllHartsLr(addr, sizeof(val));
  invalidateDecodeCache(addr, sizeof(val));

  if (isPciAddr(addr))
    {
      pci_->access<uint16_t>(addr, val, true);
      return true;
    }

  if (mcm_ and not skipFetch and fetchCache_)
    {
      pokeMcmCache<McmMem::Fetch>(addr, uint8_t(val));
      pokeMcmCache<McmMem::Fetch>(addr + 1, uint8_t(val >> 8));
    }

  std::array<bool, sizeof(val)> b{false};
  if (mcm_ and not skipData and dataCache_)
    {
      for (unsigned i = 0; i < sizeof(val); ++i)
        b.at(i) = pokeMcmCache<McmMem::Data>(addr + i, uint8_t(val >> (i*8)));
    }

  bool ok = std::reduce(b.begin(), b.end(), true, std::logical_and<>());
  if (not skipMem and not ok)
    {
      auto pma = pmaMgr_.getPma(addr);
      if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(addr))
        return pmaMgr_.writeRegister(addr, val);

      if (usePma)
        {
          if (not pma.isRead() and not pma.isExec())
            return false;
          if (addr & (sizeof(val) - 1))  // If misaligned
            {
              auto pma2 = pmaMgr_.getPma(addr + sizeof(val) - 1);
              if (not pma2.isRead() and not pma2.isExec())
                return false;
            }
        }

      if (skipData)
        ok = memory_.poke(addr, val);
      else
        {
          for (unsigned i = 0; i < sizeof(val); ++i)
            if (not b.at(i))
              b.at(i) = memory_.poke(addr + i, uint8_t(val >> (i*8)));
          ok = std::reduce(b.begin(), b.end(), true, std::logical_and<>());
        }
    }

  return ok;
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint32_t val, bool usePma, bool skipFetch,
                      bool skipData, bool skipMem)
{
  // We allow poke to bypass masking for memory mapped registers
  // otherwise, there is no way for external driver to clear bits that
  // are read-only to this hart.

  std::unique_lock lock(memory_.amoMutex_);

  memory_.invalidateAllHartsLr(addr, sizeof(val));
  invalidateDecodeCache(addr, sizeof(val));

  if (isDeviceAddr(addr))
    {
      deviceWrite(addr, val);
      return true;
    }

  if (mcm_ and not skipFetch and fetchCache_)
    for (unsigned i = 0; i < sizeof(val); ++i)
      pokeMcmCache<McmMem::Fetch>(addr + i, uint8_t(val >> (i*8)));

  std::array<bool, sizeof(val)> b{false};
  if (mcm_ and not skipData and dataCache_)
    {
      for (unsigned i = 0; i < sizeof(val); ++i)
        b.at(i) = pokeMcmCache<McmMem::Data>(addr + i, uint8_t(val >> (i*8)));
    }

  bool ok = std::reduce(b.begin(), b.end(), true, std::logical_and<>());
  if (not skipMem and not ok)
    {
      auto pma = pmaMgr_.getPma(addr);
      if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(addr))
        return pmaMgr_.writeRegister(addr, val);

      if (usePma)
        {
          if (not pma.isRead() and not pma.isExec())
            return false;
          if (addr & (sizeof(val) - 1))  // If misaligned
            {
              auto pma2 = pmaMgr_.getPma(addr + sizeof(val) - 1);
              if (not pma2.isRead() and not pma2.isExec())
                return false;
            }
        }

      if (skipData)
        ok = memory_.poke(addr, val);
      else
        {
          for (unsigned i = 0; i < sizeof(val); ++i)
            if (not b.at(i))
              b.at(i) = memory_.poke(addr + i, uint8_t(val >> (i*8)));
          ok = std::reduce(b.begin(), b.end(), true, std::logical_and<>());
        }
    }

  return ok;
}


template <typename URV>
bool
Hart<URV>::pokeMemory(uint64_t addr, uint64_t val, bool usePma, bool skipFetch,
                      bool skipData, bool skipMem)
{
  std::unique_lock lock(memory_.amoMutex_);

  memory_.invalidateAllHartsLr(addr, sizeof(val));
  invalidateDecodeCache(addr, sizeof(val));

  if (isDeviceAddr(addr))
    {
      deviceWrite(addr, val);
      return true;
    }

  if (mcm_ and not skipFetch and fetchCache_)
    for (unsigned i = 0; i < sizeof(val); ++i)
      pokeMcmCache<McmMem::Fetch>(addr + i, uint8_t(val >> (i*8)));

  std::array<bool, sizeof(val)> b{false};
  if (mcm_ and not skipData and dataCache_)
    {
      for (unsigned i = 0; i < sizeof(val); ++i)
        b.at(i) = pokeMcmCache<McmMem::Data>(addr + i, uint8_t(val >> (i*8)));
    }

  bool ok = std::reduce(b.begin(), b.end(), true, std::logical_and<>());
  if (not skipMem and not ok)
    {
      auto pma = pmaMgr_.getPma(addr);
      if (pma.hasMemMappedReg() and pmaMgr_.isMemMappedReg(addr))
        return pmaMgr_.writeRegister(addr, val);

      if (usePma)
        {
          if (not pma.isRead() and not pma.isExec())
            return false;
          if (addr & (sizeof(val) - 1))  // If misaligned
            {
              auto pma2 = pmaMgr_.getPma(addr + sizeof(val) - 1);
              if (not pma2.isRead() and not pma2.isExec())
                return false;
            }
        }

      if (skipData)
        ok = memory_.poke(addr, val);
      else
        {
          for (unsigned i = 0; i < sizeof(val); ++i)
            if (not b.at(i))
              b.at(i) = memory_.poke(addr + i, uint8_t(val >> (i*8)));
          ok = std::reduce(b.begin(), b.end(), true, std::logical_and<>());
        }
    }

  return ok;
}


template <typename URV>
void
Hart<URV>::setPendingNmi(URV cause)
{
  pendingNmis_.insert(cause);
  nmiPending_ = true;

  // Set DCSR.NMI.
  URV val = 0;  // DCSR value
  if (peekCsr(CsrNumber::DCSR, val))
    {
      DcsrFields<URV> dcsr(val);
      dcsr.bits_.NMIP = 1;
      pokeCsr(CsrNumber::DCSR, dcsr.value_);
      recordCsrWrite(CsrNumber::DCSR);
    }
}


template <typename URV>
void
Hart<URV>::clearPendingNmi()
{
  pendingNmis_.clear();
  nmiPending_ = false;

  // Clear DCSR.NMI.
  URV val = 0;  // DCSR value
  if (peekCsr(CsrNumber::DCSR, val))
    {
      DcsrFields<URV> dcsr(val);
      dcsr.bits_.NMIP = 0;
      pokeCsr(CsrNumber::DCSR, dcsr.value_);
      recordCsrWrite(CsrNumber::DCSR);
    }
}


template <typename URV>
void
Hart<URV>::clearPendingNmi(URV cause)
{
  pendingNmis_.erase(cause);
  nmiPending_ = not pendingNmis_.empty();

  if (not nmiPending_)
    {
      // Clear DCSR.NMI.
      URV val = 0;  // DCSR value
      if (peekCsr(CsrNumber::DCSR, val))
        {
          DcsrFields<URV> dcsr(val);
          dcsr.bits_.NMIP = 0;
          pokeCsr(CsrNumber::DCSR, dcsr.value_);
          recordCsrWrite(CsrNumber::DCSR);
        }
    }
}


template <typename URV>
void
Hart<URV>::setToHostAddress(uint64_t address)
{
  toHost_ = URV(address);
  toHostValid_ = true;
}


template <typename URV>
void
Hart<URV>::clearToHostAddress()
{
  toHost_ = 0;
  toHostValid_ = false;
}


template <typename URV>
inline
void
Hart<URV>::execBeq(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 != v2)
    return;

  URV nextPc = currPc_ + di->op2As<SRV>();
  if (not isRvzca() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
inline
void
Hart<URV>::execBne(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 == v2)
    return;

  URV nextPc = currPc_ + di->op2As<SRV>();
  if (not isRvzca() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
void
Hart<URV>::execBeqi(const DecodedInst* di)
{
  if (not isRvzibi())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op0());
  SRV cimm = di->op1As<SRV>();
  if (v1 != URV(cimm))
    return;

  URV nextPc = currPc_ + di->op2As<SRV>();
  if (not isRvzca() and (nextPc & 3))
    {
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
void
Hart<URV>::execBnei(const DecodedInst* di)
{
  if (not isRvzibi())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op0());
  SRV cimm = di->op1As<SRV>();
  if (v1 == URV(cimm))
    return;

  URV nextPc = currPc_ + di->op2As<SRV>();
  if (not isRvzca() and (nextPc & 3))
    {
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
inline
void
Hart<URV>::execAddi(const DecodedInst* di)
{
  SRV imm = di->op2As<SRV>();
  URV op1Val = intRegs_.read(di->op1());
  SRV v = SRV(op1Val) + imm;
  intRegs_.write(di->op0(), v);

  if (hintOps_ and di->op0() == 0)
    {
      switch (di->op1())
        {
          case 31: throw CoreException(CoreException::Snapshot, "Taking snapshot from HINT.");
          case 30: throw CoreException(CoreException::Stop, "Stopping run from HINT.");
          case 29: throw CoreException(CoreException::SnapshotAndStop, "Taking snapshot and stopping run from HINT.");
          case 26: std::cerr << "Info: Executed instructions: " << execCount_ << "\n"; break;
          case 25: setPendingNmi(URV(v)); break;
          case 24: clearPendingNmi(); break;
          case 23: defineNmiPc(URV(v)); break;
#if ACLIC_HINTS
          case 22: aclic_->setSourceState(intRegs_.read(di->op1()), bool(imm)); break;
#endif
          case 21: pokeCsr(CsrNumber::MIP, op1Val); break;  // Post/clear interrupts specified by value of 1st source op
          default: break;
        }

      if (hasRoiRange_)
        {
          if (di->op1() == 12)
            traceOn_ = false;
          if (di->op1() == 11)
            traceOn_ = true;
        }
    }
}


template <typename URV>
inline
void
Hart<URV>::execAdd(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) + intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
inline
void
Hart<URV>::execAndi(const DecodedInst* di)
{
  SRV imm = di->op2As<SRV>();
  URV v = intRegs_.read(di->op1()) & imm;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::reportInstructionFrequency(FILE* file) const
{
  std::vector<size_t> indices;
  instProfs_.sort(indices);

  for (auto profIx : indices)
    {
      const InstProfile* profPtr = instProfs_.ithEntry(profIx);
      if (not profPtr or not profPtr->freq_)
	continue;

      const InstProfile& prof = *profPtr;
      const InstEntry& entry = decoder_.getInstructionEntry(prof.id_);

      std::string instr;
      // Don't collect non-vector repeats
      if (entry.isVector())
        instr = util::join(".", entry.name(), VecRegs::to_string(prof.elemWidth_));
      else if (prof.elemWidth_ == ElementWidth::Byte)
        instr = entry.name();
      else
        continue;

      fprintf(file, "%s %jd\n", instr.c_str(), prof.freq_);

#if 0
      if (prof.user_)
        fprintf(file, "  +user %" PRIuMAX "\n", prof.user_);
      if (prof.supervisor_)
        fprintf(file, "  +supervisor %" PRIuMAX "\n", prof.supervisor_);
      if (prof.machine_)
        fprintf(file, "  +machine %" PRIuMAX "\n", prof.machine_);
#endif
    }
}


template <typename URV>
void
Hart<URV>::reportTrapStat(FILE* file) const
{
  fprintf(file, "\n");
  fprintf(file, "Interrupts (incuding NMI): %" PRIu64 "\n", interruptCount_);
  for (unsigned i = 0; i < interruptStat_.size(); ++i)
    {
      auto cause = InterruptCause(i);
      uint64_t count = interruptStat_.at(i);
      if (not count)
        continue;
      switch(cause)
        {
        case InterruptCause::S_SOFTWARE:
          fprintf(file, "  + S_SOFTWARE  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::VS_SOFTWARE:
          fprintf(file, "  + VS_SOFTWARE : %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_SOFTWARE:
          fprintf(file, "  + M_SOFTWARE  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::S_TIMER:
          fprintf(file, "  + S_TIMER     : %" PRIu64 "\n", count);
          break;
        case InterruptCause::VS_TIMER:
          fprintf(file, "  + VS_TIMER    : %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_TIMER:
          fprintf(file, "  + M_TIMER     : %" PRIu64 "\n", count);
          break;
        case InterruptCause::S_EXTERNAL:
          fprintf(file, "  + S_EXTERNAL  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::VS_EXTERNAL:
          fprintf(file, "  + VS_EXTERNAL : %" PRIu64 "\n", count);
          break;
        case InterruptCause::M_EXTERNAL:
          fprintf(file, "  + M_EXTERNAL  : %" PRIu64 "\n", count);
          break;
        case InterruptCause::G_EXTERNAL:
          fprintf(file, "  + G_EXTERNAL  : %" PRIu64 "\n", count);
          break;
        default:
          fprintf(file, "  + INTR-NO-%d  : %" PRIu64 "\n", unsigned(cause), count);
        }
    }

  fprintf(file, "\n");
  fprintf(file, "Non maskable interrupts: %" PRIu64 "\n", nmiCount_);

  fprintf(file, "\n");
  fprintf(file, "Exceptions: %" PRIu64 "\n", exceptionCount_);
  for (unsigned i = 0; i < exceptionStat_.size(); ++i)
    {
      auto cause = ExceptionCause(i);
      uint64_t count = exceptionStat_.at(i);
      if (not count)
        continue;

      switch(cause)
        {
        case ExceptionCause::INST_ADDR_MISAL :
          fprintf(file, "  + INST_ADDR_MISAL : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::INST_ACC_FAULT  :
          fprintf(file, "  + INST_ACC_FAULT  : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::ILLEGAL_INST    :
          fprintf(file, "  + ILLEGAL_INST    : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::BREAKP          :
          fprintf(file, "  + BREAKP          : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::LOAD_ADDR_MISAL :
          fprintf(file, "  + LOAD_ADDR_MISAL : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::LOAD_ACC_FAULT  :
          fprintf(file, "  + LOAD_ACC_FAULT  : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::STORE_ADDR_MISAL:
          fprintf(file, "  + STORE_ADDR_MISAL: %" PRIu64 "\n", count);
          break;
        case ExceptionCause::STORE_ACC_FAULT :
          fprintf(file, "  + STORE_ACC_FAULT : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::U_ENV_CALL      :
          fprintf(file, "  + U_ENV_CALL      : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::S_ENV_CALL      :
          fprintf(file, "  + S_ENV_CALL      : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::M_ENV_CALL      :
          fprintf(file, "  + M_ENV_CALL      : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::INST_PAGE_FAULT :
          fprintf(file, "  + INST_PAGE_FAULT : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::LOAD_PAGE_FAULT :
          fprintf(file, "  + LOAD_PAGE_FAULT : %" PRIu64 "\n", count);
          break;
        case ExceptionCause::STORE_PAGE_FAULT:
          fprintf(file, "  + STORE_PAGE_FAULT: %" PRIu64 "\n", count);
          break;
        case ExceptionCause::NONE            :
          fprintf(file, "  + NONE            : %" PRIu64 "\n", count);
          break;
        default:
          fprintf(file, "  + ????            : %" PRIu64 "\n", count);
          break;
        }
    }
}


template <typename URV>
void
Hart<URV>::reportLrScStat(FILE* file) const
{
  fprintf(file, "Load-reserve dispatched: %jd\n", uintmax_t(lrCount_));
  fprintf(file, "Load-reserve successful: %jd\n", uintmax_t(lrSuccess_));
  fprintf(file, "Store-conditional dispatched: %jd\n", uintmax_t(scCount_));
  fprintf(file, "Store-conditional successful: %jd\n", uintmax_t(scSuccess_));
}


template <typename URV>
void
Hart<URV>::initiateLoadException(const DecodedInst* di, ExceptionCause cause, URV addr1, URV addr2)
{
  initiateException(cause, currPc_, addr1, addr2, di);
}


template <typename URV>
void
Hart<URV>::initiateStoreException(const DecodedInst* di, ExceptionCause cause, URV addr1, URV addr2)
{
  initiateException(cause, currPc_, addr1, addr2, di);
}


template <typename URV>
ExceptionCause
Hart<URV>::determineLoadException(uint64_t& addr1, uint64_t& addr2, uint64_t& gaddr1,
                                  uint64_t& gaddr2, unsigned ldSize, bool hyper,
                                  bool amo, unsigned elemIx)
{
  // NOLINTNEXTLINE(modernize-use-auto)
  uint64_t va1 = URV(addr1); // Virtual address. Truncate to 32-bits in 32-bit mode.
  uint64_t va2 = va1;        // Used if crossing page boundary
  ldStFaultAddr_ = va1;
  addr1 = gaddr1 = va1;
  addr2 = gaddr2 = va2;  // Phys addr of 2nd page when crossing page boundary.

  // Misaligned load from io section triggers an exception.
  uint64_t alignMask = ldSize - 1;
  bool misal = addr1 & alignMask;
  misalignedLdSt_ = misal;

  using EC = ExceptionCause;
  using PM = PrivilegeMode;

  auto [pm, virt] = effLdStMode(hyper);

  ldStFaultAddr_ = addr1 = gaddr1 = va1 = applyPointerMask(va1, true, hyper);
  addr2 = gaddr2 = va2 = va1;

  // If misaligned exception has priority take exception.
  if (misal)
    {
      if (misalHasPriority_ and not misalDataOk_)
	return ExceptionCause::LOAD_ADDR_MISAL;
      va2 = (va1 + ldSize - 1) & ~alignMask;
    }

  setMemProtAccIsFetch(false);
  steeInsec1_ = false;
  steeInsec2_ = false;

  auto checkPa = [this, pm, ldSize, misal, amo] (uint64_t va, uint64_t& pa, Pma& pma, bool lower) -> EC {
    ldStFaultAddr_ = va;

    if (pmpEnabled_)
      {
        auto pmp = pmpMgr_.accessPmp(pm, pa);
        if (not pmp.isRead()  or  (virtMem_.isExecForRead() and not pmp.isExec()))
          return EC::LOAD_ACC_FAULT;
      }

    if (steeEnabled_)
      {
        if (not stee_.isValidAddress(pa))
          return EC::LOAD_ACC_FAULT;
        bool& steeInsec = lower? steeInsec1_ : steeInsec2_;
        steeInsec = stee_.isInsecureAccess(pa);
        if (steeTrapRead_ and steeInsec)
          return EC::LOAD_ACC_FAULT;
        pa = stee_.clearSecureBits(pa);
      }

    pma = accessPma(pa);
    pma = overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt());
    if (not pma.isRead() or (virtMem_.isExecForRead() and not pma.isExec()))
      return EC::LOAD_ACC_FAULT;

    auto size = ldSize;

    if (misal)
      {
        bool ok = pma.isMisalignedOk();
        if (amo)
          {
            ok = false;
            if (auto mag = pma.misalAtomicGranule(); mag)
              {
                auto mask = ~uint64_t(mag - 1);
                ok = (pa & mask) == ((pa + ldSize - 1) & mask);
              }
          }
        if (not ok)
          return pma.misalOnMisal()? EC::LOAD_ADDR_MISAL : EC::LOAD_ACC_FAULT;

        // If checking the lower part of a misal address, do not cross alignment boundary.
        if (lower)
          size = ((pa + (size - 1)) & ~uint64_t(size - 1)) - pa;
      }

    // In case memory size is less that what the PMA/PMP declares as accessible.
    if (pa > memory_.size() - size)
      return EC::LOAD_ACC_FAULT;

    return EC::NONE;
  };

  bool translate = isRvs() and pm != PM::Machine;
  if (translate)
    {
      if (auto cause = virtMem_.translateForLoad(va1, pm, virt, gaddr1, addr1);
          cause != EC::NONE)
        {
          ldStFaultAddr_ = addr1;
          return cause;
        }
    }

  gaddr2 = gaddr1;
  addr2 = addr1;
  uint64_t pa1 = addr1;  // We do this because checkPa modifies addr1 (clears STEE bits).

  ldStPma1_ = ldStPma2_ = Pma{};

  if (not misal)
    {
      if (auto cause = checkPa(va1, addr1, ldStPma1_, true); cause != EC::NONE)
        return cause;
      addr2 = addr1;  // checkPa may clear STEE bits of addr1
    }
  else
    {
      if (inSeqnMisaligned_)
        if (auto cause = checkPa(va1, addr1, ldStPma1_, true); cause != EC::NONE)
          return cause;

      bool cross = virtMem_.pageNumber(va1) != virtMem_.pageNumber(va2);
      addr2 = (pa1 + (ldSize - 1)) & ~alignMask;

      if (cross and translate)
        {
          auto cause = virtMem_.translateForLoad(va2, pm, virt, gaddr2, addr2);
          if (cause != EC::NONE)
            {
              ldStFaultAddr_ = addr2;
              gaddr1 = gaddr2;  // We report faulting GPA in gaddr.
              return cause;
            }
        }

      if (inSeqnMisaligned_)
        if (auto cause = checkPa(va2, addr2, ldStPma2_, false); cause != EC::NONE)
          return cause;

      if (not inSeqnMisaligned_)
        {
          if (auto cause = checkPa(va1, addr1, ldStPma1_, true); cause != EC::NONE)
            return cause;
          if (auto cause = checkPa(va2, addr2, ldStPma2_, false); cause != EC::NONE)
            return cause;
        }

      if (not cross)
        addr2 = addr1; // addr2 is different from addr1 only if we cross page boundary
    }

  if (injectException_ != EC::NONE and injectExceptionIsLd_ and elemIx == injectExceptionElemIx_)
    {
      ldStFaultAddr_ = va1;
      // Adjust ldStFaultAddr for line crossers if fault is on 2nd line.
      if (injectAddr_ != 0 and cacheLineNum(va1) != cacheLineNum(injectAddr_))
        {
          if (misal)
            ldStFaultAddr_ = va2;
          else
            std::cerr << "Error: hart-id= " << hartId() << " tag=" << execCount_
                      << " injected exception pa does not match instruction data pa.\n";
        }
      return injectException_;
    }

  return EC::NONE;
}


template <typename URV>
template <typename LOAD_TYPE>
inline
bool
Hart<URV>::fastLoad(const DecodedInst* di, uint64_t addr, uint64_t& value)
{
  // Unsigned version of LOAD_TYPE
  using ULT = std::make_unsigned_t<LOAD_TYPE>;

  ULT uval = 0;
  if (memory_.read(addr, uval))
    {
      if constexpr (std::is_same<ULT, LOAD_TYPE>::value)
        value = uval;
      else
        value = SRV(LOAD_TYPE(uval)); // Sign extend.

      if (dataLineTrace_)
	memory_.traceDataLine(addr, addr);

      return true;  // Success.
    }

  initiateLoadException(di, ExceptionCause::LOAD_ACC_FAULT, addr);
  return false;
}


/// Dump initial state of a memory line to the given file.
template <typename URV>
void
Hart<URV>::dumpInitState(const char* tag, uint64_t vaddr, uint64_t paddr)
{
  bool isFetch = (*tag == 'f'); // If tag is "fetch"

  auto& lineSet = isFetch? initInstrLines_ : initDataLines_;

  uint64_t pline = memory_.getLineNumber(paddr);
  if (lineSet.find(pline) != lineSet.end())
    return;  // Already dumped

  lineSet.insert(pline);

  uint64_t vline = memory_.getLineNumber(vaddr);
  unsigned lineSize = memory_.lineSize();
  fprintf(initStateFile_.get(), "%s,%0jx,%0jx,", tag, uintmax_t(vline*lineSize), uintmax_t(pline*lineSize));

  uint64_t byteAddr = pline * lineSize + lineSize - 1;
  for (unsigned i = 0; i < lineSize; ++i, --byteAddr)
    {
      uint8_t byte = 0;
      memory_.peek(byteAddr, byte);
      virtMem_.getPrevByte(byteAddr, byte); // Get PTE value before PTE update.
      fprintf(initStateFile_.get(), "%02x", unsigned(byte));
    }

  bool cacheable = pmaMgr_.getPma(paddr).isCacheable();
  fprintf(initStateFile_.get(), ",%d", cacheable);
  fprintf(initStateFile_.get(), "\n");
}


#include <termios.h>
#undef VSTART

static bool
hasPendingInput(int fd)
{
  static bool firstTime = true;

  if (firstTime)
    {
      firstTime = false;
      if (isatty(fd))
	{
	  struct termios term{};
	  tcgetattr(fd, &term);
	  cfmakeraw(&term);
	  term.c_lflag &= ~ECHO;
	  tcsetattr(fd, 0, &term);
	}
    }

  struct pollfd inPollfd{};
  inPollfd.fd = fd;
  inPollfd.events = POLLIN;
  int code = poll(&inPollfd, 1, 0);
  return code == 1 and (inPollfd.revents & POLLIN) != 0;
}


static int
readCharNonBlocking(int fd)
{
  if (not hasPendingInput(fd))
    return 0;

  char c = 0;
  auto code = ::read(fd, &c, sizeof(c));
  if (code == 1)
    {
      if (isatty(fd))
	{
	  static char prev = 0;

	  // Force a stop if control-a x is seen.
	  if (prev == 1 and c == 'x')
	    throw CoreException(CoreException::Stop, "Keyboard stop", 3);
	  prev = c;
	}

      return c;
    }

  if (code == 0)
    return 0;

  if (code == -1)
    std::cerr << "Error: readCharNonBlocking: unexpected fail on read\n";

  return -1;
}


template <typename URV>
bool
Hart<URV>::getOooLoadValue(uint64_t va, uint64_t pa1, uint64_t pa2, unsigned size,
			   bool isVec, uint64_t& value, unsigned elemIx, unsigned field)
{
  if (not ooo_)
    return false;
  if (mcm_)
    return mcm_->getCurrentLoadValue(*this, execCount_, va, pa1, pa2, size, isVec,
				     value, elemIx, field);
  if (perfApi_)
    return perfApi_->getLoadData(hartIx_, execCount_, va, pa1, pa2, size, value,
                                 elemIx, field);
  assert(0 && "Error: Assertion failed");
  return false;
}


template <typename URV>
template <typename LOAD_TYPE>
bool
Hart<URV>::load(const DecodedInst* di, uint64_t virtAddr, uint64_t& data)
{
  hyperLs_ = di->isHypervisor();

  ldStAddr_ = virtAddr;   // For reporting ld/st addr in trace-mode.
  ldStFaultAddr_ = applyPointerMask(virtAddr, true /*isLoad*/, hyperLs_);
  ldStPhysAddr1_ = ldStPhysAddr2_ = virtAddr;
  ldStSize_ = sizeof(LOAD_TYPE);

#if FAST_SLOPPY
  return fastLoad<LOAD_TYPE>(di, virtAddr, data);
#else

  if (hasActiveTrigger())
    {
      ldStAddrTriggerHit(ldStFaultAddr_, ldStSize_, TriggerTiming::Before, true /*isLoad*/);
      if (breakpOrEnterDebugTripped())
        return false;
    }

  uint64_t addr1 = virtAddr;
  uint64_t addr2 = addr1;
  uint64_t gaddr1 = virtAddr;
  uint64_t gaddr2 = virtAddr;

  auto cause = determineLoadException(addr1, addr2, gaddr1, gaddr2, ldStSize_, hyperLs_);
  if (cause != ExceptionCause::NONE)
    {
      initiateLoadException(di, cause, ldStFaultAddr_, gaddr1);
      return false;
    }
  ldStPhysAddr1_ = addr1;
  ldStPhysAddr2_ = addr2;

  return readForLoad<LOAD_TYPE>(di, virtAddr, addr1, addr2, data);
#endif
}


template <typename URV>
void
Hart<URV>::deviceRead(uint64_t pa, unsigned size, uint64_t& val)
{
  val = 0;
  if (isAclintAddr(pa))
    {
      processClintRead(pa, size, val);
      return;
    }

  if (isImsicAddr(pa))
    {
      if (imsicRead_)
        imsicRead_(pa, size, val);
      return;
    }

  if (isPciAddr(pa))
    {
      switch (size)
	{
	case 1:
	  {
	    uint8_t pciVal = 0;
            pci_->access<uint8_t>(pa, pciVal, false);
	    val = pciVal;
	  }
	  break;

	case 2:
	  {
	    uint16_t pciVal = 0;
            pci_->access<uint16_t>(pa, pciVal, false);
	    val = pciVal;
	  }
	  break;

	case 4:
	  {
	    uint32_t pciVal = 0;
            pci_->access<uint32_t>(pa, pciVal, false);
	    val = pciVal;
	  }
	  break;

	case 8:
	  {
	    uint64_t pciVal = 0;
            pci_->access<uint64_t>(pa, pciVal, false);
	    val = pciVal;
	  }
	  break;

	default:
	  assert(0 && "Error: Assertion failed");
	}
      return;
    }

  if (isAplicAddr(pa))
    {
      uint32_t val32 = 0;
      if (not aplic_->read(pa, size, val32))
        {
          std::cerr << "Warning: unsupported APLIC read: address = 0x" <<
            std::hex << pa << std::dec << ", size = " << size << " bytes\n";
        }
      val = val32;
      return;
    }

  if (isIommuAddr(pa))
    {
      uint64_t val64 = 0;
      iommu_->read(pa, size, val64);
      val = val64;
      return;
    }

  assert(0 && "Error: Assertion failed");  // No device contains given address.
}


template <typename URV>
template<typename STORE_TYPE>
void
Hart<URV>::deviceWrite(uint64_t pa, STORE_TYPE storeVal)
{
  if (isAclintAddr(pa))
    {
      URV val = storeVal;
      processClintWrite(pa, sizeof(storeVal), val);
      processTimerInterrupt();  // In case TIMER or TIMECMP was written in ACLINT.
      storeVal = val;
      memWrite(pa, pa, storeVal);
      return;
    }

  if (isImsicAddr(pa))
    {
      imsicWrite_(pa, sizeof(storeVal), storeVal);
      return;
    }

  if (isPciAddr(pa))
    {
      pci_->access<STORE_TYPE>(pa, storeVal, true);
      return;
    }

  if (isAplicAddr(pa))
    {
      uint32_t val32 = storeVal;
      if (not aplic_->write(pa, sizeof(storeVal), val32))
        {
          std::cerr << "Warning: unsupported APLIC write: address = 0x" <<
            std::hex << pa << std::dec << ", size = " << sizeof(storeVal) <<
            " bytes, data = 0x" << std::hex << uint64_t(storeVal) << std::dec << "\n";
        }
      return;
    }

  if (isIommuAddr(pa))
    {
      iommu_->write(pa, sizeof(storeVal), storeVal);
      return;
    }

  assert(0 && "Error: Assertion failed");
}


template <typename URV>
template <typename LOAD_TYPE>
bool
Hart<URV>::readForLoad([[maybe_unused]] const DecodedInst* di, uint64_t virtAddr,
		       [[maybe_unused]] uint64_t addr1, [[maybe_unused]] uint64_t addr2,
		       uint64_t& data, unsigned elemIx, unsigned field)
{
#if FAST_SLOPPY
  return fastLoad<LOAD_TYPE>(di, virtAddr, data);
#else

  // Loading from console-io does a standard input read.
  if (conIoValid_ and addr1 == conIo_ and enableConIn_ and not breakpOrEnterDebugTripped())
    {
      data = readCharNonBlocking(syscall_.effectiveFd(STDIN_FILENO));
      return true;
    }

  // Unsigned version of LOAD_TYPE
  using ULT = std::make_unsigned_t<LOAD_TYPE>;

  ULT uval = 0;   // Unsigned loaded value

  bool hasOooVal = false;
  if (ooo_)
    {
      uint64_t val = 0;
      hasOooVal = getOooLoadValue(virtAddr, addr1, addr2, sizeof(LOAD_TYPE), di->isVector(),
				  val, elemIx, field);
      if (hasOooVal)
	uval = val;
    }

  if (not hasOooVal)
    {
      if (toHostValid_ and addr1 == toHost_)
	{
	  data = 0;
	  return true;
	}
      if (isDeviceAddr(addr1))
	{
	  uint64_t dv = 0;
	  deviceRead(addr1, sizeof(ULT), dv);
	  uval = dv;
	}
      else
	memRead(addr1, addr2, uval);
    }

  data = uval;
  if (not std::is_same<ULT, LOAD_TYPE>::value)
    data = int64_t(LOAD_TYPE(uval)); // Loading signed: Sign extend.

  if (initStateFile_)
    {
      dumpInitState("load", virtAddr, addr1);
      if (addr1 != addr2 or memory_.getLineNumber(addr1) != memory_.getLineNumber(addr1 + ldStSize_))
	dumpInitState("load", virtAddr + ldStSize_, addr2 + ldStSize_);
    }

  if (dataLineTrace_)
    memory_.traceDataLine(virtAddr, addr1);

  if (traceCacheOn_)
    traceCache(virtAddr, addr1, addr2, true, false, false, false, false);

  // Check for load-data-trigger.
  if (hasActiveTrigger())
    {
      TriggerTiming timing = TriggerTiming::Before;
      bool isLoad = true;
      ldStDataTriggerHit(uval, timing, isLoad);
      if (breakpOrEnterDebugTripped())
        return false;
    }

  return true;  // Success.
#endif
}


template <typename URV>
inline
void
Hart<URV>::execLw(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<int32_t>(di, virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
inline
void
Hart<URV>::execLh(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<int16_t>(di, virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
template <typename STORE_TYPE>
inline
bool
Hart<URV>::fastStore(const DecodedInst* di, uint64_t addr, STORE_TYPE storeVal)
{
  if (memory_.write(addr, storeVal))
    {
      ldStWrite_ = true;

      if (toHostValid_ and addr == toHost_ and storeVal != 0)
	{
	  throw CoreException(CoreException::Stop, "write to to-host", storeVal);
	}

      if (dataLineTrace_)
	memory_.traceDataLine(addr, addr, true /*write*/);

      ldStWrite_ = true;
      ldStData_ = storeVal;
      return true;
    }

  initiateStoreException(di, ExceptionCause::STORE_ACC_FAULT, addr);
  return false;
}


template <typename URV>
template <typename STORE_TYPE>
void
Hart<URV>::handleStoreToHost(URV physAddr, STORE_TYPE storeVal)
{
  // We assume that the HTIF device is little endian.
  ldStWrite_ = true;
  ldStData_ = storeVal;
  memory_.write(physAddr, storeVal);

  uint64_t val = storeVal;
  uint64_t data = (val << 16) >> 16;
  unsigned cmd = (val >> 48) & 0xff;
  unsigned dev = (val >> 56) & 0xff;
  if (dev == 1)
    {
      if (cmd == 1)
	{
	  char c = std::bit_cast<char>(uint8_t(data));
	  if (c)
	    {
	      if (::write(syscall_.effectiveFd(STDOUT_FILENO), &c, 1) != 1)
		std::cerr << "Error: Hart::handleStoreToHost: write failed\n";
	    }
	}
      else if (cmd == 0 and fromHostValid_)
	{
	  int ch = readCharNonBlocking(syscall_.effectiveFd(STDIN_FILENO));
	  if (ch > 0)
	    memory_.poke(fromHost_, ((val >> 48) << 48) | uint64_t(ch));
	  else
	    ++pendingHtifGetc_;
	}
    }
  else if (dev == 0 and cmd == 0 and (storeVal & 1))
    throw CoreException(CoreException::Stop, "write to to-host", val);
}


template <typename URV>
template <typename STORE_TYPE>
inline
bool
Hart<URV>::store(const DecodedInst* di, URV virtAddr, STORE_TYPE storeVal,
                 [[maybe_unused]] bool amoLock)
{
  hyperLs_ = di->isHypervisor();
  ldStAddr_ = virtAddr;   // For reporting ld/st addr in trace-mode.
  ldStFaultAddr_ = applyPointerMask(virtAddr, false /*isLoad*/, hyperLs_);
  ldStPhysAddr1_ = ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = sizeof(STORE_TYPE);

#if FAST_SLOPPY
  return fastStore(di, virtAddr, storeVal);
#else

  // With a single hart there is no other accessor, so the amoMutex_ is pure overhead.
  auto lock = (amoLock and numHarts_ > 1)? std::unique_lock(memory_.amoMutex_) :
                         std::unique_lock<std::shared_mutex>();

  // ld/st-address or instruction-address triggers have priority over
  // ld/st access or misaligned exceptions.
  bool hasTrig = hasActiveTrigger();
  TriggerTiming timing = TriggerTiming::Before;
  bool isLd = false;  // Not a load.
  if (hasTrig)
    {
      ldStAddrTriggerHit(ldStFaultAddr_, ldStSize_, timing, isLd);
      ldStDataTriggerHit(storeVal, timing, isLd);
      if (breakpOrEnterDebugTripped())
        return false;
    }

  // Determine if a store exception is possible. Determine sore exception will do address
  // translation and change pa1/pa2 to physical addresses. Ga1/ga2 are the guest addresses
  // for 2-stage address translation.
  uint64_t pa1 = virtAddr, pa2 = virtAddr;
  uint64_t ga1 = virtAddr, ga2 = virtAddr;
  ExceptionCause cause = determineStoreException(pa1, pa2, ga1, ga2, ldStSize_, hyperLs_, di->isAmo());
  ldStPhysAddr1_ = pa1;
  ldStPhysAddr2_ = pa2;

  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(di, cause, ldStFaultAddr_, ga1);
      return false;
    }

  return writeForStore<STORE_TYPE>(virtAddr, pa1, pa2, storeVal);
#endif
}


template <typename URV>
template <typename STORE_TYPE>
bool
Hart<URV>::writeForStore(uint64_t virtAddr, uint64_t pa1, uint64_t pa2, STORE_TYPE storeVal)
{
  // If addr is special location, then write to console.
  if (conIoValid_ and pa1 == conIo_)
    {
      if (consoleOut_)
	{
	  fputc(storeVal, consoleOut_.get());
	  if (storeVal == '\n')
	    fflush(consoleOut_.get());
	}
      return true;
    }

  if (initStateFile_)
    {
      dumpInitState("store", virtAddr, pa1);
      if (pa1 != pa2 or memory_.getLineNumber(pa1) != memory_.getLineNumber(pa1 + ldStSize_))
	dumpInitState("store", virtAddr + ldStSize_, pa2 + ldStSize_);
    }

  ldStWrite_ = true;
  ldStData_ = storeVal;

  invalidateDecodeCache(pa1, ldStSize_); // this could be smaller
  if (pa1 != pa2)
    invalidateDecodeCache(pa2, ldStSize_);

  // If we write to special location, end the simulation.
  if (isToHostAddr(pa1) and mcm_)
    {
      handleStoreToHost(pa1, storeVal);
      return true;
    }

  if (dataLineTrace_)
    memory_.traceDataLine(virtAddr, pa1, true /*write*/);

  if (ooo_)
    {
      if (perfApi_)
	perfApi_->setStoreData(hartIx_, execCount_, pa1, pa2, sizeof(storeVal), storeVal);
      return true;  // Memory updated & lr-canceled when merge buffer is written.
    }

  // If we write to special location, end the simulation.
  if (isToHostAddr(pa1))
    {
      handleStoreToHost(pa1, storeVal);
      return true;
    }

  if (isDeviceAddr(pa1))
    {
      assert(pa1 == pa2);
      deviceWrite(pa1, storeVal);
      return true;
    }

  memory_.invalidateOtherHartLr(hartIx_, pa1, ldStSize_);
  if (pa2 != pa1)
    memory_.invalidateOtherHartLr(hartIx_, pa2, ldStSize_);

  memWrite(pa1, pa2, storeVal);

  STORE_TYPE temp = 0;
  memPeek(pa1, pa2, temp);
  ldStData_ = temp;

  if (traceCacheOn_)
    traceCache(virtAddr, pa1, pa2, false, true, false, false, false);

  return true;
}


template <typename URV>
void
Hart<URV>::processClintRead(uint64_t addr, unsigned size, uint64_t& val)
{
  val = 0;

  if (size != 4 and size != 8)
    return;    // Size must be 4 or 8.

  if ((addr & 3) != 0)
    return;    // Address must be word aligned.

  if (addr >= aclintMtimeStart_ and addr < aclintMtimeEnd_)
    {
      uint64_t tt = getTime();

      if (size == 4)
        {
          if ((addr & 7) == 0)       // Addr is double word aligned, matches time register.
            val = (tt << 32) >> 32;  // Clear top 32 bits.
          else                       // Addr is word aligned, matches top word of time register.
            val = tt >> 32;          // Keep top 32 btis.
        }
      else if (size == 8 and (addr & 7) == 0)   // Exact match of time register address.
        val = tt;
      return;  // Timer.
    }

  if (addr >= aclintSwStart_ and addr < aclintSwEnd_)
    {
      if (size == 4)
        {
          uint32_t u32 = 0;
          peekMemory(addr, u32, true /*usePma*/);
          val = u32;
        }
      return;
    }

  if (addr >= aclintMtimeCmpStart_ and addr < aclintMtimeCmpEnd_)
    {
      if (size == 4)
        {
          uint32_t u32 = 0;
          peekMemory(addr, u32, true /*usePma*/);
          val = u32;
        }
      else if (size == 8 and (addr & 7) == 0)
        peekMemory(addr, val, true /*usePma*/);
    }
}


template <typename URV>
void
Hart<URV>::processClintWrite(uint64_t addr, unsigned stSize, URV& storeVal)
{
  // We assume that the CLINT device is little endian.
  if (addr >= aclintSwStart_ and addr < aclintSwEnd_)
    {
      unsigned hartIx = (addr - aclintSwStart_) / 4;
      auto hart = indexToHart_(hartIx);
      if (hart and stSize == 4 and (addr & 3) == 0)
	{
	  storeVal = storeVal & 1;  // Only bit zero is implemented.
	  if (aclintDeliverInterrupts_)
	    hart->setSwInterrupt((1 << 1) | storeVal);
	  return;
	}
    }
  else if (addr >= aclintMtimeStart_ and addr < aclintMtimeEnd_)
    {
      if (stSize == 4)
      {
        uint64_t orig = 0, desired = 0;
        do {
          orig = atomic_ref(time_).load(std::memory_order_relaxed);

          if ((addr & 7) == 0)  // low 32
            desired = (orig & 0xFFFFFFFF00000000ULL) | (uint32_t)storeVal;
          else if ((addr & 3) == 0)  // high 32
            desired = (orig & 0x00000000FFFFFFFFULL) | ((uint64_t)storeVal << 32);
          else
            return; // Misaligned 4-byte access
        } while (!atomic_ref(time_).compare_exchange_weak(orig, desired, std::memory_order_relaxed));
      }
      else if (stSize == 8)
      {
        if ((addr & 7) == 0)  // aligned 64-bit write
          atomic_ref(time_).store(storeVal, std::memory_order_relaxed);
        else
          return; // Misaligned 8-byte write
      }
    }
  else if (addr >= aclintMtimeCmpStart_ and addr < aclintMtimeCmpEnd_)
    {
      // don't expect software to modify clint alarm from two different harts
      unsigned hartIx = (addr - aclintMtimeCmpStart_) / 8;
      auto hart = indexToHart_(hartIx);
      if (hart)
	hart->markTimerStale();  // mtimecmp store changes the timer threshold.
      if (hart and (stSize == 4 or stSize == 8))
	{
	  if (stSize == 4 and aclintDeliverInterrupts_)
	    {
	      if ((addr & 7) == 0)  // Multiple of 8
		{
		  hart->aclintAlarm_ = (hart->aclintAlarm_ >> 32) << 32;  // Clear low 32
		  hart->aclintAlarm_ |= uint32_t(storeVal);  // Update low 32.
		}
	      else if ((addr & 3) == 0)  // Multiple of 4
		{
		  hart->aclintAlarm_ = (hart->aclintAlarm_ << 32) >> 32;  // Clear high 32
		  hart->aclintAlarm_ |= (uint64_t(storeVal) << 32);  // Update high 32.
		}
	    }
	  else if (stSize == 8)
	    {
              // Whisper fake timer is based on instruction count which appears to be much
              // faster than what Linux typically expects. We adjust the time compare (we
              // add 10000 by default) so that Linux does not see too many timer
              // interrupts.
	      if ((addr & 7) == 0 and aclintDeliverInterrupts_)
                {
                  if (storeVal + aclintAdjustTimeCmp_ < storeVal)
                    hart->aclintAlarm_ = storeVal;  // No adjustment if large value would overflow.
                  else
                    hart->aclintAlarm_ = storeVal + aclintAdjustTimeCmp_;
                }

	      // An htif_getc may be pending, send char back to target.
	      auto inFd = syscall_.effectiveFd(STDIN_FILENO);
	      if (pendingHtifGetc_ and hasPendingInput(inFd))
		{
		  uint64_t v = 0;
		  peekMemory(fromHost_, v, true);
		  if (v == 0)
		    {
		      int c = readCharNonBlocking(inFd);
		      if (c > 0)
			{
			  memory_.poke(fromHost_, (uint64_t(1) << 56) | (char) c);
			  --pendingHtifGetc_;
			}
		    }
		}
	    }
	  return;
	}
    }

  // Address did not match any hart entry in clint.
  storeVal = 0;
}


template <typename URV>
unsigned
Hart<URV>::vecLdStElemSize(const DecodedInst& di) const
{
  assert(di.isVectorLoad() or di.isVectorStore());

  if (di.isVectorLoadIndexed() or di.isVectorStoreIndexed())
    return vecRegs_.elemWidthInBytes();

  return di.vecLoadOrStoreElemSize();
}


template <typename URV>
unsigned
Hart<URV>::vecLdStIndexElemSize(const DecodedInst& di) const
{
  assert(di.isVectorLoadIndexed() or di.isVectorStoreIndexed());
  return di.vecLoadOrStoreElemSize();
}


template <typename URV>
inline
void
Hart<URV>::execSw(const DecodedInst* di)
{
  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  auto value = uint32_t(intRegs_.read(di->op0()));

  store<uint32_t>(di, addr, value);
}


template <typename URV>
bool
Hart<URV>::readInst(uint64_t va, uint64_t& pa, uint32_t& inst)
{
  inst = 0;
  pa = va;
  bool translate = isRvs() and privMode_ != PrivilegeMode::Machine;

  if (translate)
    if (virtMem_.transAddrNoUpdate(va, privMode_, virtMode_, false, false, true, pa) != ExceptionCause::NONE)
      return false;

  if (not pmaMgr_.accessPma(pa).isExec())
    return false;

  uint16_t low = 0;  // Low 2 bytes of instruction.
  if (not memory_.readInst(pa, low))
    return false;

  inst = low;
  if ((inst & 0x3) != 3)
    return true;  // Compressed instruction.

  uint16_t high = 0;
  uint64_t va2 = va + 2, pa2 = pa + 2;
  if (translate and memory_.getPageIx(va) != memory_.getPageIx(va2))
    if (virtMem_.transAddrNoUpdate(va2, privMode_, virtMode_, false, false, true, pa2) != ExceptionCause::NONE)
      {
	inst = 0;
	return false;
      }

  if (not pmaMgr_.accessPma(pa2).isExec())
    return false;

  if (memory_.readInst(pa2, high))
    {
      inst |= (uint32_t(high) << 16);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::readInst(uint64_t va, uint32_t& inst)
{
  uint64_t pa = 0;
  return readInst(va, pa, inst);
}


template <typename URV>
inline
ExceptionCause
Hart<URV>::fetchInstNoTrap(uint64_t& va, uint64_t& pa, [[maybe_unused]] uint64_t& pa2,
			   uint64_t& gpa, uint32_t& inst)
{
#if FAST_SLOPPY

  assert((va & 1) == 0);
  gpa = pa = pa2 = va;
  if (not memory_.readInst(pa, inst))
    return ExceptionCause::INST_ACC_FAULT;
  return ExceptionCause::NONE;

#else

  uint64_t steePhysAddr = 0;
  pa = pa2 = steePhysAddr = va;
  gpa = 0;
  if (isRvs() and privMode_ != PrivilegeMode::Machine)
    {
      gpa = va;
      auto cause = virtMem_.translateForFetch(va, privMode_, virtMode_, gpa, pa);
      if (cause != ExceptionCause::NONE)
	return cause;
    }

  if (va & 1)
    return ExceptionCause::INST_ADDR_MISAL;

  if (pmpEnabled_)
    {
      auto pmp = pmpMgr_.accessPmp(privMode_, pa);
      if (not pmp.isExec())
	return ExceptionCause::INST_ACC_FAULT;
    }

  if (steeEnabled_)
    {
      if (not stee_.isValidAddress(pa))
        return ExceptionCause::INST_ACC_FAULT;
      if (stee_.isInsecureAccess(pa))
        {
          if (steeTrapRead_)
            return ExceptionCause::INST_ACC_FAULT;
          inst = 0;   // Secure device returns zero on insecure fetch.
          return ExceptionCause::NONE;
        }
      pa = stee_.clearSecureBits(pa);
    }

  if (not pmaMgr_.accessPma(pa).isExec())
    return ExceptionCause::INST_ACC_FAULT;

  bool wordAligned = (pa & 3) == 0;
  bool umfc = mcm_ and fetchCache_;   // Use MCM fetch cache.

  if (wordAligned)
    {
      if (not memory_.readInst(pa, inst))   // Read opcode.
	return ExceptionCause::INST_ACC_FAULT;

      // Override with MCM fetch cache. Complain if missing leaving opcode unomdified.
      // If line is io/nc, we cache it anyway counting on the test-bench to evict it.
      if (umfc and not readInstFromFetchCache(pa, inst))
        mcm_->reportMissingFetch(*this, execCount_, pa);

      if (initStateFile_)
	dumpInitState("fetch", va, pa);
      if (traceCacheOn_)
        traceCache(va, pa, pa, false, false, true, false, false);

      if (isCompressedInst(inst))
	inst = (inst << 16) >> 16;
      return ExceptionCause::NONE;
    }

  uint16_t half = 0;
  if (not memory_.readInst(pa, half))
    return ExceptionCause::INST_ACC_FAULT;

  if (umfc and not readInstFromFetchCache(pa, half))
    mcm_->reportMissingFetch(*this, execCount_, pa);

  if (initStateFile_)
    dumpInitState("fetch", va, pa);
  inst = half;
  if (isCompressedInst(inst))
    {
      if (traceCacheOn_)
        traceCache(va, pa, pa, false, false, true, false, false);
      return ExceptionCause::NONE;
    }

  // If we cross page boundary, translate address of other page.
  pa2 = steePhysAddr + 2;
  gpa = pa2;
  auto pi = memory_.getPageIx(pa);
  auto pi2 = memory_.getPageIx(pa2);
  if (pi != pi2 and isRvs() and privMode_ != PrivilegeMode::Machine)
    {
      auto cause = virtMem_.translateForFetch(va + 2, privMode_, virtMode_, gpa, pa2);
      if (cause != ExceptionCause::NONE)
        {
          va += 2;  // To report faulting portion of fetch.
          return cause;
        }
    }

  if (pmpEnabled_ and not pmpMgr_.accessPmp(privMode_, pa2).isExec())
    {
      va += 2; // To report faulting portion of fetch.
      return ExceptionCause::INST_ACC_FAULT;
    }
  if (steeEnabled_)
    {
      if (not stee_.isValidAddress(pa2))
        return ExceptionCause::INST_ACC_FAULT;
      bool insecure = stee_.isInsecureAccess(pa2);
      pa2 = stee_.clearSecureBits(pa2);
      if (insecure)
        {
          if (steeTrapRead_)
            {
              va += 2; // To report faulting portion of fetch.
              return ExceptionCause::INST_ACC_FAULT;
            }
          return ExceptionCause::NONE;   // Upper half of inst is zero.
        }
    }

  if (not pmaMgr_.accessPma(pa2).isExec())
    {
      va += 2;  // To report faulting portion of fetch.
      return ExceptionCause::INST_ACC_FAULT;
    }

  uint16_t upperHalf = 0;
  if (not memory_.readInst(pa2, upperHalf))
    {
      va += 2;  // To report faulting portion of fetch.
      return ExceptionCause::INST_ACC_FAULT;
    }

  if (umfc and not readInstFromFetchCache(pa2, upperHalf))
    mcm_->reportMissingFetch(*this, execCount_, pa2);

  if (initStateFile_)
    dumpInitState("fetch", va, pa2);

  if (traceCacheOn_)
    traceCache(va, pa, pa2, false, false, true, false, false);

  inst = inst | (uint32_t(upperHalf) << 16);
  return ExceptionCause::NONE;

#endif
}


template <typename URV>
inline
bool
Hart<URV>::fetchInst(URV virtAddr, uint64_t& physAddr, uint32_t& inst)
{
  uint64_t gPhysAddr = 0, physAddr2 = 0, va = virtAddr;

  // If a trap occurs on a page crossing fetch, va is updated with the
  // portion of the access that caused the trap.
  auto cause = fetchInstNoTrap(va, physAddr, physAddr2, gPhysAddr, inst);
  if (cause != ExceptionCause::NONE)
    {
      if (not breakpOrEnterDebugTripped())
        initiateException(cause, virtAddr, va, gPhysAddr);
      return false;
    }
  return true;
}


template <typename URV>
bool
Hart<URV>::fetchInstPostTrigger(URV virtAddr, uint64_t& physAddr,
				uint32_t& inst, FILE* traceFile)
{
  if (fetchInst(virtAddr, physAddr, inst))
    return true;

  // Fetch failed: take pending trigger-exception or instruction trigger.
  // If fetch fails, it is not possible to have another etrigger fire.
  URV info = virtAddr;
  takeTriggerAction(traceFile, virtAddr, info, execCount_, nullptr /*di*/);
  return false;
}


template <typename URV>
void
Hart<URV>::illegalInst(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  uint32_t inst = di->inst();
  if (isCompressedInst(inst))
    inst = inst & 0xffff;

  URV info = clearMtvalOnIllInst_ ? 0 : inst;
  initiateException(ExceptionCause::ILLEGAL_INST, currPc_, info);
}


template <typename URV>
void
Hart<URV>::virtualInst(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  uint32_t inst = di->inst();
  if (isCompressedInst(inst))
    inst = inst & 0xffff;

  initiateException(ExceptionCause::VIRT_INST, currPc_, inst);
}


template <typename URV>
void
Hart<URV>::unimplemented(const DecodedInst* di)
{
  illegalInst(di);
}


// Start an asynchronous exception.
template <typename URV>
void
Hart<URV>::initiateInterrupt(InterruptCause cause, PrivilegeMode nextMode,
                              bool nextVirt, URV pc, bool hvi)
{
  hasInterrupt_ = true;
  interruptCount_++;

  bool interrupt = true;
  URV info = 0;  // This goes into mtval.

  // Remap the cause to non-VS cause (e.g. VSTIME becomes STIME) if the interrupt was
  // delegated as opposed to injected by hvictl.
  using IC = InterruptCause;
  URV causeNum = URV(cause);
  if (nextVirt and (cause == IC::VS_EXTERNAL or cause == IC::VS_TIMER or
                    cause == IC::VS_SOFTWARE) and not hvi)
      causeNum--;

  initiateTrap(nullptr, interrupt, causeNum, nextMode, nextVirt, pc, info);

  if (not enableCounters_ or not hasActivePerfCounter())
    return;

  PerfRegs& pregs = csRegs_.mPerfRegs_;
  pregs.updateCounters(EventNumber::Interrupt, prevPerfControl_, lastPriv_, lastVirt_);

  if (cause == IC::M_EXTERNAL)
    pregs.updateCounters(EventNumber::ExternalInterrupt, prevPerfControl_,
                         lastPriv_, lastVirt_);
  else if (cause == IC::M_TIMER)
    pregs.updateCounters(EventNumber::TimerInterrupt, prevPerfControl_,
                         lastPriv_, lastVirt_);
}


// Start a synchronous exception.
template <typename URV>
void
Hart<URV>::initiateException(ExceptionCause cause, URV pc, URV info, URV info2, const DecodedInst* di)
{
  // Check if stuck because of lack of exception handler. Disable if
  // you do want the stuck behavior.
#if 1
  if (di == nullptr or di->instId() == InstId::illegal)
    {
      if (execCount_ == execCountLastIll_ + 1)
	consecIllCount_++;
      else
	consecIllCount_ = 0;

      if (consecIllCount_ > 16)  // FIX: Make a parameter
	throw CoreException(CoreException::Stop, "16 consecutive illegal instructions", 3);

      execCountLastIll_ = execCount_;
    }
#endif

  exceptionCount_++;
  hasException_ = true;

  // In debug mode no exception is taken. If we get an ebreak exception and debug park
  // loop is defined, we jump to it. If we get a non-ebreak exception and debug trap entry
  // point is defined, we jump to it.
  if (debugMode_)
    {
      if (cause == ExceptionCause::BREAKP)
	{
	  if (debugParkLoop_ != ~URV(0))
	    {
	      inDebugParkLoop_ = true;
	      setPc(debugParkLoop_);
	    }
	}
      else if (debugTrapAddr_ != ~URV(0))
	setPc(debugTrapAddr_);
      return;
    }

  bool interrupt = false;
  exceptionCount_++;
  hasException_ = true;

  // By default, exceptions are taken in machine mode.
  using PM = PrivilegeMode;
  PM nextMode = PM::Machine;
  bool nextVirt = false;

  // But they can be delegated to supervisor.
  if (isRvs() and privMode_ != PM::Machine)
    {
      URV delegVal = peekCsr(CsrNumber::MEDELEG);
      if (delegVal & (URV(1) << URV(cause)))
        {
          nextMode = PM::Supervisor;

          // In hypervisor, traps can be further delegated to virtual supervisor (VS)
          // except for guest page faults
          if (isRvh() and virtMode_)
            {
              delegVal = peekCsr(CsrNumber::HEDELEG);
              if (delegVal & (URV(1) << URV(cause)))
                nextVirt = true;
            }
        }
    }

  initiateTrap(di, interrupt, URV(cause), nextMode, nextVirt, pc, info, info2);

  PerfRegs& pregs = csRegs_.mPerfRegs_;
  if (enableCounters_ and hasActivePerfCounter())
    pregs.updateCounters(EventNumber::Exception, prevPerfControl_, lastPriv_, lastVirt_);
}


/// Return true if given trap number would result in a guest virtual
/// address being written to mtval/stval if a trap was taken from
/// VS/VU to M/HS.
bool
isGvaTrap(bool virtMode, unsigned causeCode)
{
  using EC = ExceptionCause;

  // These may be generated by hypervisor ld/store instructions (e.g. hlv.w).
  EC cause = EC{causeCode};
  switch (cause)
    {
    case EC::INST_GUEST_PAGE_FAULT:
    case EC::LOAD_GUEST_PAGE_FAULT:
    case EC::STORE_GUEST_PAGE_FAULT:
      return true;

    default:
      break;
    }

  if (not virtMode)
    return false;

  switch (cause)
    {
    case EC::BREAKP:
    case EC::INST_ADDR_MISAL:
    case EC::INST_ACC_FAULT:
    case EC::LOAD_ADDR_MISAL:
    case EC::LOAD_ACC_FAULT:
    case EC::STORE_ADDR_MISAL:
    case EC::STORE_ACC_FAULT:
    case EC::INST_PAGE_FAULT:
    case EC::LOAD_PAGE_FAULT:
    case EC::STORE_PAGE_FAULT:
      return true;

    default:
      return false;
    }

  return false;
}


/// Return true if given trap number corresponds to a guest page fault.
bool
isGpaTrap(unsigned causeCode)
{
  using EC = ExceptionCause;

  EC cause = EC{causeCode};
  switch (cause)
    {
    case EC::INST_GUEST_PAGE_FAULT:
    case EC::LOAD_GUEST_PAGE_FAULT:
    case EC::STORE_GUEST_PAGE_FAULT:
      return true;
    default:
      return false;
    }
  return false;
}

template <typename URV>
uint32_t
Hart<URV>::createTrapInst(const DecodedInst* di, bool interrupt, unsigned causeCode,
                          URV info, URV info2) const
{
  using EC = ExceptionCause;

  if (interrupt)
    return 0;

  EC cause = EC{causeCode};
  switch (cause)
    {
    case EC::INST_ADDR_MISAL:
    case EC::INST_ACC_FAULT:
    case EC::ILLEGAL_INST:
    case EC::BREAKP:
    case EC::U_ENV_CALL:
    case EC::S_ENV_CALL:
    case EC::VS_ENV_CALL:
    case EC::M_ENV_CALL:
    case EC::INST_PAGE_FAULT:
    case EC::VIRT_INST:
      return 0;
    default:
      break;
    }

  // Implicit accesses for VS-stage address translation generate a pseudocode.
  if (isGpaTrap(causeCode))
    {
      bool s1ImplicitWrite = false;
      // FIXME: info2 should be checked non-zero first
      if (virtMem_.s1ImplAccTrap(s1ImplicitWrite) and info2)
        {
          /// From Table 8.12 of privileged spec.
          if constexpr (sizeof(URV) == 4)
            return 0x2000 | (uint32_t(s1ImplicitWrite) << 5);
          else
            return 0x3000 | (uint32_t(s1ImplicitWrite) << 5);
        }
    }

  if (not di)
    return 0;

  // Spec does not specify how vector ld/st should be handled.
  if (di->isVector())
    return 0;

  // Spec does not specify how shadow stack instructions should be handled.
  if (di->isSspush() or di->isCsspush() or di->isSspopchk() or di->isCsspopchk() or
      di->instId() == InstId::ssamoswap_w or di->instId() == InstId::ssamoswap_d)
    return 0;

  if (clearTinstOnCboInval_ and di->instId() == InstId::cbo_inval)
    return 0;

  if (clearTinstOnCboFlush_ and di->instId() == InstId::cbo_flush)
    return 0;

  // Otherwise we write a transformed instruction.
  uint32_t uncompressed = 0;
  if (not di->isCompressed())
    uncompressed = di->inst();
  else
    {
      uncompressed = decoder_.expandCompressedInst(di->inst() & 0xffff);
      uncompressed &= ~uint32_t(2); // Clear bit 1 to indicate expanded compressed instruction
    }

  // Clear relevant fields.
  if (di->isLoad() and not di->isHypervisor() and not di->isLr())
    uncompressed &= 0x000fffff;
  else if (di->isStore() and not di->isHypervisor() and not di->isSc())
    uncompressed &= 0x01fff07f;
  else if (di->isCmo())
    uncompressed &= 0xfffff07f;
  else if (di->isAtomic() or di->isHypervisor())
    uncompressed &= 0xfff07fff;
  else
    assert(false);

  // Set address offset field for misaligned exceptions. For a page crossing access the
  // max offset would be 7 (load double-word).
  uncompressed &= ~(uint32_t(0x1f) << 15);
  URV base = applyPointerMask(ldStAddr_, di->isLoad(), hyperLs_);
  URV offset = info - base;
  if (offset > 7)
    {
      std::cerr << "Error: Hart::createTrapInst: Larger than 7 offset: " << offset << '\n';
      offset = offset & 0x1f;
    }
  uncompressed |= (offset) << 15;
  return uncompressed;
}


template <typename URV>
void
Hart<URV>::initiateTrap(const DecodedInst* di, bool interrupt,
                        URV cause,
                        PrivilegeMode nextMode, bool nextVirt,
                        URV pcToSave, URV info, URV info2)
{
  if (cancelLrOnTrap_)
    cancelLr(CancelLrCause::TRAP);

  using PM = PrivilegeMode;
  PM origMode = privMode_;

  if (isRvsmdbltrp())
    {
      // Section 3.1.6.2 of priv spec: Double Trap Control in mstatus Register
      bool nmie = MnstatusFields{csRegs_.peekMnstatus()}.bits_.NMIE;
      bool unexpTrap = ( (nextMode == PM::Machine and mstatus_.bits_.MDT) or
                         (origMode == PM::Machine and isRvsmrnmi() and not nmie) );
      if (unexpTrap)
        {
          if (isRvsmrnmi() and nmie)
            {
              initiateNmi(cause, pcToSave, /*isDoubleTrap=*/true);
              return;
            }
          throw CoreException(CoreException::Stop,
                              "Core entered critical-error state due to an unexpected trap", 3);
        }

      if (nextMode == PM::Machine)
        mstatus_.bits_.MDT = 1;
    }

  // Ssdbltrp: if trapping to Supervisor mode while SDT is already set, escalate
  // to M-mode as a double-trap exception (supervisor.adoc §sstatus_sdt_trap).
  // Must be done before writing S-mode EPC/CAUSE/TVAL registers.
  if (isRvssdbltrp() and nextMode == PM::Supervisor and not nextVirt and mstatus_.bits_.SDT)
    {
      URV origCause = cause;

      // Build M-mode trap state inline (same pattern as aclicSaveContext Ssdbltrp path).
      mstatus_.bits_.MPP  = unsigned(origMode);
      mstatus_.bits_.MPIE = mstatus_.bits_.MIE;
      mstatus_.bits_.MIE  = 0;
      if (isRvsmdbltrp())
        mstatus_.bits_.MDT = 1;
      writeMstatus();

      if (not csRegs_.write(CsrNumber::MEPC, PM::Machine, pcToSave & ~(URV(1))))
        assert(0 and "Failed to write MEPC in Ssdbltrp double-trap escalation");

      using EC2 = ExceptionCause;
      if (not csRegs_.write(CsrNumber::MCAUSE, PM::Machine, URV(EC2::DOUBLE_TRAP)))
        assert(0 and "Failed to write MCAUSE in Ssdbltrp double-trap escalation");

      if (not csRegs_.write(CsrNumber::MTVAL, PM::Machine, 0))
        assert(0 and "Failed to write MTVAL in Ssdbltrp double-trap escalation");

      pokeCsr(CsrNumber::MTVAL2, origCause);  // original cause → mtval2

      privMode_ = PM::Machine;
      URV tvec2 = 0;
      if (not csRegs_.read(CsrNumber::MTVEC, PM::Machine, tvec2))
        assert(0 and "Failed to read MTVEC in Ssdbltrp double-trap escalation");
      setPc((tvec2 >> 2) << 2);
      return;
    }

  bool origVirtMode = virtMode_;
  bool gvaVirtMode = effectiveVirtualMode();

  uint32_t tinst = isRvh()? createTrapInst(di, interrupt, cause, info, info2) : 0;

  // Traps are taken in machine mode.
  setPrivilegeMode(nextMode);
  virtMode_ = nextVirt;

  csRegs_.setVirtualMode(virtMode_);

  // Enable/disable virtual mode for CSR read/writes
  if (virtMode_ != origVirtMode)
    setVirtualMode(virtMode_);

  CsrNumber epcNum = CsrNumber::MEPC;
  CsrNumber causeNum = CsrNumber::MCAUSE;
  CsrNumber tvalNum = CsrNumber::MTVAL;
  CsrNumber tvecNum = CsrNumber::MTVEC;

  if (nextMode == PM::Supervisor)
    {
      epcNum = CsrNumber::SEPC;
      causeNum = CsrNumber::SCAUSE;
      tvalNum = CsrNumber::STVAL;
      tvecNum = CsrNumber::STVEC;
    }

  // Save address of instruction that caused the exception or address
  // of interrupted instruction.
  if (not csRegs_.write(epcNum, privMode_, pcToSave & ~(URV(1))))
    assert(0 and "Failed to write EPC register");

  // Save the exception cause.
  URV causeRegVal = cause;
  if (interrupt)
    causeRegVal |= URV(1) << (mxlen_ - 1);
  if (not csRegs_.write(causeNum, privMode_, causeRegVal))
    assert(0 and "Failed to write CAUSE register");
  trapCause_ = causeRegVal;

  if (clearMtvalOnEgs_ and egsConstraint_)
    info = 0;

  // Clear mtval on interrupts. Save synchronous exception info.
  if (not csRegs_.write(tvalNum, privMode_, info))
    assert(0 and "Failed to write TVAL register");

  URV tval2 = 0;  // New values of MTVAL2/HTVAL CSR.
  if (isGpaTrap(cause))
    tval2 = info2 >> 2;

  using EC = ExceptionCause;
  injectException_ = EC::NONE;

  bool gva = isRvh() and not interrupt and (hyperLs_ or isGvaTrap(gvaVirtMode, cause));
  if (origVirtMode  and  cause == unsigned(EC::HARDWARE_ERROR)  and not  interrupt)
    gva = true;
  else if (lastEbreak_)
    {
      if (clearMtvalOnEbreak_)
        gva = false;
    }
  else if ((cause == unsigned(EC::BREAKP)) and icountTrig_) // icount trigger
    gva = false;

  // Update status register saving xIE in xPIE and previous privilege
  // mode in xPP by getting current value of xstatus, updating
  // its fields and putting it back.
  if (nextMode == PM::Machine)
    {
      mstatus_.bits_.MPP = unsigned(origMode);
      mstatus_.bits_.MPIE = mstatus_.bits_.MIE;
      mstatus_.bits_.MIE = 0;
      mstatus_.bits_.GVA = gva;
      mstatus_.bits_.MPV = origVirtMode;
      if (isRvZicfilp())
        mstatus_.bits_.MPELP = elp_;
      writeMstatus();

      // Smnip: on any trap to M-mode, save current mithreshold into mistatus.pithreshprio.
      // On interrupt traps, additionally update mithreshold to the IPRIO of the taken
      // interrupt. Synchronous exceptions save pithreshprio but do not modify mithreshold.
      // Spec (aclic.adoc §Smnip): "When a trap is taken into level x, the current value of
      // xithreshold.iprio is written to xistatus.pithreshprio. Additionally, if the trap
      // was taken on an interrupt, xithreshold.iprio is set to xtopsi.IPRIO."
      // pithreshprio occupies mistatus[16:8] (9 bits).
      if (extensionIsEnabled(RvExtension::Smnip) and aclic_ and aclic_->isMnipEnabled())
        {
          URV curMisVal = 0, curThresh = 0;
          [[maybe_unused]] bool ok = false;
          ok = csRegs_.peek(CsrNumber::MISTATUS, curMisVal);
          assert(ok);
          ok = csRegs_.peek(CsrNumber::MITHRESHOLD, curThresh);
          assert(ok);
          // Clear current pithreshprio[16:8] and write the saved 9-bit threshold.
          curMisVal = (curMisVal & ~(URV(0x1FF) << 8))
                    | ((curThresh & URV(0x1FF)) << 8);
          csRegs_.poke(CsrNumber::MISTATUS, curMisVal);
          if (interrupt)
            {
              unsigned iprio = 0;
              unsigned srcId = aclic_->topInterrupt(true, &iprio);
              uint16_t newThresh = srcId ? static_cast<uint16_t>(iprio) : uint16_t(0);
              aclic_->setMithreshold(newThresh);
              csRegs_.poke(CsrNumber::MITHRESHOLD, URV(aclic_->getMithreshold()));
            }
        }

      if (isRvh() and not csRegs_.write(CsrNumber::MTVAL2, privMode_, tval2))
        assert(0 and "Failed to write MTVAL2 register");
      if (isRvh() and not csRegs_.write(CsrNumber::MTINST, PM::Machine, tinst))
	assert(0 and "Failed to write MTINST register");
      if (sdtrigOn_)
	csRegs_.saveTcontrolMte();
    }
  else if (nextMode == PM::Supervisor)
    {
      // Trap taken into S/HS or, if V is 1, into VS-mode.
      MstatusFields<URV> msf(csRegs_.peekSstatus(virtMode_));
      msf.bits_.SPP = unsigned(origMode);
      msf.bits_.SPIE = msf.bits_.SIE;
      msf.bits_.SIE = 0;
      if (isRvZicfilp())
        msf.bits_.SPELP = elp_;
      if (not csRegs_.write(CsrNumber::SSTATUS, privMode_, msf.value_))
	assert(0 and "Failed to write SSTATUS register");

      // Ssnip: on any trap to S-mode, save current sithreshold into sistatus.pithreshprio.
      // On interrupt traps, additionally update sithreshold to the IPRIO of the taken
      // interrupt. Same semantics as Smnip but for supervisor level.
      // pithreshprio occupies sistatus[16:8] (9 bits).
      if (extensionIsEnabled(RvExtension::Ssnip) and aclic_ and aclic_->isSnipEnabled() and not virtMode_)
        {
          URV curSisVal = 0, curThresh = 0;
          [[maybe_unused]] bool ok = false;
          ok = csRegs_.peek(CsrNumber::SISTATUS, curSisVal);
          assert(ok);
          ok = csRegs_.peek(CsrNumber::SITHRESHOLD, curThresh);
          assert(ok);
          curSisVal = (curSisVal & ~(URV(0x1FF) << 8))
                    | ((curThresh & URV(0x1FF)) << 8);
          csRegs_.poke(CsrNumber::SISTATUS, curSisVal);
          if (interrupt)
            {
              unsigned iprio = 0;
              unsigned srcId = aclic_->topInterrupt(false, &iprio);
              uint16_t newThresh = srcId ? static_cast<uint16_t>(iprio) : uint16_t(0);
              aclic_->setSithreshold(newThresh);
              csRegs_.poke(CsrNumber::SITHRESHOLD, URV(aclic_->getSithreshold()));
            }
        }

      if (not virtMode_)
	{
	  // Trap taken into HS privilege.
	  hstatus_.bits_.SPV = origVirtMode;  // Save virt mode.
	  if (origVirtMode)
	    {
	      assert(origMode == PM::User or origMode == PM::Supervisor);
	      hstatus_.bits_.SPVP = unsigned(origMode);
	    }
	  hstatus_.bits_.GVA = gva;
	}
      updateCachedSstatus();

      // Ssdbltrp: set mstatus.SDT=1 on trap entry to Supervisor mode.
      if (isRvssdbltrp() and not virtMode_)
        {
          mstatus_.bits_.SDT = 1;
          writeMstatus();
        }

      if (isRvh())
	{
	  if (not csRegs_.write(CsrNumber::HSTATUS, PM::Machine, hstatus_.value_))
	    assert(0 and "Failed to write HSTATUS register");

	  if (not virtMode_) 	  // Update HTVAL/HTINST if trapping to HS mode.
	    {
	      if (not csRegs_.write(CsrNumber::HTVAL, privMode_, tval2))
		assert(0 and "Failed to write HTVAL register");
	      if (not csRegs_.write(CsrNumber::HTINST, privMode_, tinst))
		assert(0 and "Failed to write HTINST register");
	    }
	}
    }

  // Set program counter to trap handler address.
  URV tvec = 0;
  if (not csRegs_.read(tvecNum, privMode_, tvec))
    assert(0 and "Failed to read TVEC register");

  URV base = (tvec >> 2) << 2;  // Clear least sig 2 bits.
  auto tvecMode = TrapVectorMode(tvec & 0x3);

  auto nextPc = base;
  if (interrupt and tvecMode == TrapVectorMode::Vectored)
    nextPc = base + 4*cause;

  // Smeihv/Sseihv (xtvec.mode=10): hardware vectoring of major + external
  // interrupts.  PC = OBASE + 4*SIID, where SIID is the minor IID (positive)
  // for external interrupts or -major_iid (signed) for major interrupts.
  // Synchronous exceptions in this mode go to OBASE (offset 0).
  if (interrupt and tvecMode == TrapVectorMode::HwVectored)
    {
      bool isSuper = (nextMode == PM::Supervisor);
      bool hvOn = isSuper ? isRvSseihv() : isRvSmeihv();
      if (hvOn)
        {
          using IC = InterruptCause;
          auto ic = IC(cause);
          bool externalCause = (ic == IC::M_EXTERNAL or ic == IC::S_EXTERNAL or
                                ic == IC::VS_EXTERNAL or ic == IC::G_EXTERNAL);
          using SRV = typename std::make_signed_t<URV>;
          SRV siid = 0;
          if (externalCause and aclic_)
            {
              unsigned id = aclic_->topInterrupt(not isSuper, nullptr, /*ignoreThreshold=*/true);
              siid = SRV(id);
            }
          else
            siid = -SRV(cause);  // major interrupt: SIID = -cause
          nextPc = base + URV(siid * SRV(4));
        }
    }

  // Reset ELP.
  if (isRvZicfilp())
    setElp(false);

  // ACLIC support: Update trap handler PC if table vectored mode is on.
  if (tvecMode == TrapVectorMode::TableVectored and
      not getTableVectoredTrapPc(base, interrupt, cause, origMode, nextMode, nextVirt,
                                 pcToSave, nextPc))
    return;  // Double trap while fetching trap handler PC.

  // ACLIC support: Partially save context (regs a0 to 15) on stack.
  if (interrupt and not aclicSaveContext(origMode, nextMode, pcToSave))
    return;  // Double trap while saving context.

  // If exception happened while in an NMI handler, we go to the NMI exception
  // handler address.
  if (not interrupt and extensionIsEnabled(RvExtension::Smrnmi) and
      MnstatusFields{csRegs_.peekMnstatus()}.bits_.NMIE == 0 and
      origMode == PM::Machine)
    nextPc = indexedNmi_ ? nmiExceptionPc_ + 4*cause : nmiExceptionPc_;;

  setPc(nextPc);

  if (instFreq_)
    accumulateTrapStats(false /*isNmi*/);

  if (branchBuffer_.max_size() and not branchTraceFile_.empty())
    traceBranch(nullptr);

  if (hasActiveTrigger())
    {
      if (interrupt)
	{
	  if (csRegs_.intTriggerHit(cause, privMode_, virtMode_, isBreakpInterruptEnabled()))
            initiateException(ExceptionCause::BREAKP, pc_, 0, 0, di);
	}
      else if (cause != URV(ExceptionCause::BREAKP))
	{
	  if (csRegs_.expTriggerHit(cause, privMode_, virtMode_, isBreakpInterruptEnabled()))
            initiateException(ExceptionCause::BREAKP, pc_, 0, 0, di);
	}
    }
}


template <typename URV>
bool
Hart<URV>::getTableVectoredTrapPc(URV base, bool interrupt, URV cause,
                                  PrivilegeMode origMode, PrivilegeMode nextMode,
                                  bool nextVirt, URV origPc, URV& nextPc)
{
  using CN = CsrNumber;
  using EC = ExceptionCause;
  using PM = PrivilegeMode;

  bool isSuper = (nextMode == PM::Supervisor);

  if (not interrupt)     // If exception
    {
      // Smehv / Ssehv (spec §Smehv): when xtvec.mode=11 AND xijt.EHV[3:2] != 0
      // AND Smehv/Ssehv is enabled, synchronous exceptions vector to
      //   PC = xtvec[XLEN-1:2]<<2 + 4*exccode
      // Otherwise the exception trap PC is OBASE (caller already set nextPc).
      bool ehvExtOn = isSuper ? isRvSsehv() : isRvSmehv();
      if (ehvExtOn)
        {
          URV xijt = isSuper ? peekCsr(CN::SIJT) : peekCsr(CN::MIJT);
          if (((xijt >> 2) & 0x3) != 0)
            nextPc = base + 4*cause;
        }
      return true;
    }

  bool ijtOn = isSuper ? isRvSsijt() : isRvSmijt();
  if (not ijtOn)
    return true;

  // Smijt / Ssijt jump-table vectoring (xtvec.mode=11), spec §Smijt:
  //   JTBASE   = xijt[XLEN-1:6] << 6          (64-byte aligned base)
  //   SHAMT    = xijt[1:0]                    (additional left-shift on offset)
  //   vtoffset = SIID (signed, two's complement)
  //   entry    = M[JTBASE + (vtoffset << (SHAMT + 2))]   XLEN-bit read
  //   VEN      = entry & 1                    (vectoring-enable bit)
  //   if VEN: PC = entry & JTMASK             (JTMASK = ~1 if IALIGN=16 else ~3)
  //   else:   PC = xtvec[XLEN-1:2]<<2         (fall back to common dispatcher)
  // Faults during the table read abort the interrupt trap and take a precise
  // interrupt fault trap (synchronous exception) — see §Interrupt Fault Handling.
  URV xijt   = isSuper ? peekCsr(CN::SIJT) : peekCsr(CN::MIJT);
  URV jtbase = (xijt >> 6) << 6;
  auto shamt = unsigned(xijt & 0x3);

  // Compute signed vtoffset.  External interrupts use the actual ACLIC source id
  // (xtopei.IID); major interrupts use the negated major id.
  using IC = InterruptCause;
  auto ic = IC(cause);
  bool external = (ic == IC::M_EXTERNAL or ic == IC::S_EXTERNAL or
                   ic == IC::VS_EXTERNAL or ic == IC::G_EXTERNAL);
  using SRV = typename std::make_signed_t<URV>;
  SRV vtoffset = 0;
  unsigned extSrcId = 0;
  if (external and aclic_)
    {
      extSrcId = aclic_->topInterrupt(not isSuper, nullptr, /*ignoreThreshold=*/true);
      vtoffset = SRV(extSrcId);
    }
  else
    vtoffset = -SRV(cause);

  URV regSize = isRv64() ? 8 : 4;
  // Sign-extended shift of vtoffset by (shamt + 2).
  URV byteOffset = URV(SRV(vtoffset) << SRV(shamt + 2));
  URV vaddr = jtbase + byteOffset;

  // Per spec, the table access is a *read* at handler-mode privilege (load
  // permissions; PMP/ePMP).  We re-use translateForFetch + the PMP exec check
  // as a conservative stand-in (refinement: use a load-class translate).
  uint64_t paddr = 0, gpa = 0;
  auto ffc = virtMem_.translateForFetch(vaddr, nextMode, nextVirt, gpa, paddr);

  auto readBytes = [this] (PM pm, uint64_t pa, uint32_t& word) -> EC {
    if (pmpEnabled_)
      {
        auto pmp = pmpMgr_.accessPmp(pm, pa);
        if (not pmp.isExec())
          return EC::INST_ACC_FAULT;
      }
    if (steeEnabled_)
      {
        if (not stee_.isValidAddress(pa))
          return EC::INST_ACC_FAULT;
        if (stee_.isInsecureAccess(pa))
          {
            if (steeTrapRead_)
              return EC::INST_ACC_FAULT;
            word = 0;
            return EC::NONE;
          }
        pa = stee_.clearSecureBits(pa);
      }
    return memory_.readInst(pa, word) ? EC::NONE : EC::INST_ACC_FAULT;
  };

  if (ffc == EC::NONE)
    {
      assert((paddr & 3) == 0);
      uint32_t low = 0;
      ffc = readBytes(nextMode, paddr, low);
      URV entry = low;
      if (isRv64() and ffc == EC::NONE)
        {
          uint32_t high = 0;
          ffc = readBytes(nextMode, paddr+4, high);
          entry = entry | (uint64_t(high) << 32);
        }

      if (ffc == EC::NONE)
        {
          bool ven = (entry & URV(1)) != 0;
          if (ven)
            {
              URV jtmask = isRvc() ? ~URV(1) : ~URV(3);
              nextPc = entry & jtmask;
              // Spec: "clears the corresponding interrupt-pending bit if possible".
              if (aclic_ and external)
                aclic_->tryClearPending(not isSuper, extSrcId);
            }
          // VEN=0: nextPc stays at OBASE (caller already set nextPc = base).
          return true;
        }
    }

  (void) regSize;

  if (isRvsmdbltrp())
    {
      // Section 3.1.6.2 of priv spec: Double Trap Control in mstatus Register
      bool nmie = MnstatusFields{csRegs_.peekMnstatus()}.bits_.NMIE;
      bool unexpTrap = ( (nextMode == PM::Machine and mstatus_.bits_.MDT) or
                         (origMode == PM::Machine and isRvsmrnmi() and not nmie) );
      if (unexpTrap)
        {
          if (isRvsmrnmi() and nmie)
            {
              initiateNmi(URV(ffc), origPc, /*isDoubleTrap=*/true);
              return false;
            }
          throw CoreException(CoreException::Stop,
                              "Core entered critical-error state due to an unexpected trap", 3);
        }

      if (nextMode == PM::Machine)
        mstatus_.bits_.MDT = 1;
    }

  // Without Smdbltrp there is no unexpected-trap machinery; take the fault
  // as a normal M-mode synchronous exception.  This is typically
  // unrecoverable (the handler will encounter the same fault again), but
  // the spec does not require any special handling in this case.
  initiateTrap(nullptr, /*interrupt=*/false, URV(ffc),
               PrivilegeMode::Machine, false /*virt*/, origPc, vaddr, 0);
  return false;
}


template <typename URV>
bool
Hart<URV>::aclicSaveContext(PrivilegeMode origMode, PrivilegeMode nextMode, URV origPc)
{
  using PM = PrivilegeMode;

  // mipu/sipu: Smip/Ssip extension present AND the mipu/sipu bit set in
  // miconfig/siconfig (spec §Smip "Controlling the extension behavior").
  // Software can clear miconfig.{mipu,sipu} to suppress the automatic context
  // push/pop without disabling the extension.
  bool mipu = isRvsmip() and aclic_ and aclic_->isMipuEnabled();
  bool sipu = isRvssip() and aclic_ and aclic_->isSipuEnabled();

  if (privMode_ == PM::Machine and not mipu)
    return true;

  if (privMode_ == PM::Supervisor and not sipu)
    return true;

  // Tempoarily set MPRV so that we can load from the interrupted context.
  unsigned savedMprv = mstatus_.bits_.MPRV;
  mstatus_.bits_.MPRV = 1;

  // Temporarily set SUM
  bool savedSum = virtMem_.getSum();
  virtMem_.setSum(true);

  URV va = intRegs_.read(IntRegNumber::RegSp);
  unsigned regSize = sizeof(URV);
  bool hyper = false;

  ExceptionCause cause = ExceptionCause::NONE;

  uint64_t gpa1 = 0, gpa2 = 0;

  // Push regs in reverse order (A5 to A0 with A0 on top of the stack) so that restore is
  // in order (A0 to A5).
  for (unsigned i = IntRegNumber::RegA5; i >= IntRegNumber::RegA0; --i)
    {
      va -= regSize;

      auto regNum = IntRegNumber(i);

      // FIX: Should we evaluate store debug triggers.

      uint64_t pa1 = va, pa2 = va;
      gpa1 = va; gpa2 = va;
      cause = determineStoreException(pa1, pa2, gpa1, gpa2, regSize, hyper);
      if (cause != ExceptionCause::NONE)
        break;

      URV value = intRegs_.read(regNum);
      if (not  writeForStore<URV>(va, pa1, pa2, value))
        assert(0);
    }

  mstatus_.bits_.MPRV = savedMprv;
  virtMem_.setSum(savedSum);

  if (cause == ExceptionCause::NONE)
    {
      intRegs_.write(IntRegNumber::RegSp, va);
      if (privMode_ == PM::Machine)
        mcspspush();
      else
        scspspush();
      return true;
    }

  // Ssdbltrp: S-mode double trap. Context-save fault while SDT=1 (set when
  // the interrupt was taken to S-mode) escalates to M-mode as DOUBLE_TRAP.
  if (isRvssdbltrp() and nextMode == PM::Supervisor and mstatus_.bits_.SDT)
    {
      // Escalate to M-mode: set M-mode trap state directly (like initiateNmi).
      mstatus_.bits_.MPP  = unsigned(privMode_);  // save current S-mode
      mstatus_.bits_.MPIE = mstatus_.bits_.MIE;
      mstatus_.bits_.MIE  = 0;
      if (isRvsmdbltrp())
        mstatus_.bits_.MDT = 1;  // entering M-mode, set MDT
      writeMstatus();

      // mepc = origPc (same value already in sepc)
      if (not csRegs_.write(CsrNumber::MEPC, PM::Machine, origPc))
        assert(0 and "Failed to write MEPC in Ssdbltrp escalation");

      // mcause = DOUBLE_TRAP (16)
      using EC = ExceptionCause;
      if (not csRegs_.write(CsrNumber::MCAUSE, PM::Machine, URV(EC::DOUBLE_TRAP)))
        assert(0 and "Failed to write MCAUSE in Ssdbltrp escalation");

      // mtval2 = original cause (e.g., STORE_PAGE_FAULT = 15).
      // MTVAL2 is enabled by enableSsdbltrp; if H extension is also present,
      // it was already enabled via enableHypervisorMode. Either way it works.
      pokeCsr(CsrNumber::MTVAL2, URV(cause));  // best-effort; not fatal if absent

      // mtval = 0 (no specific address for double-trap at context save)
      if (not csRegs_.write(CsrNumber::MTVAL, PM::Machine, 0))
        assert(0 and "Failed to write MTVAL in Ssdbltrp escalation");

      // switch to M-mode and redirect to mtvec
      setPrivilegeMode(PM::Machine);
      URV tvec = 0;
      if (not csRegs_.read(CsrNumber::MTVEC, privMode_, tvec))
        assert(0 and "Failed to read MTVEC in Ssdbltrp escalation");
      setPc((tvec >> 2) << 2);  // direct mode (exception, not vectored)
      return false;
    }

  if (isRvsmdbltrp())
    {
      // Section 3.1.6.2 of priv spec: Double Trap Control in mstatus Register
      bool nmie = MnstatusFields{csRegs_.peekMnstatus()}.bits_.NMIE;
      bool unexpTrap = ( (nextMode == PM::Machine and mstatus_.bits_.MDT) or
                         (origMode == PM::Machine and isRvsmrnmi() and not nmie) );
      if (unexpTrap)
        {
          if (isRvsmrnmi() and nmie)
            {
              initiateNmi(URV(cause), origPc, /*isDoubleTrap=*/true);
              return false;
            }
          throw CoreException(CoreException::Stop,
                              "Core entered critical-error state due to an unexpected trap", 3);
        }

      if (nextMode == PM::Machine)
        mstatus_.bits_.MDT = 1;
    }

  // Without Smdbltrp there is no unexpected-trap machinery; take the fault
  // as a normal M-mode synchronous exception.  This is typically
  // unrecoverable (the handler will encounter the same fault again), but
  // the spec does not require any special handling in this case.
  initiateTrap(nullptr, /*interrupt=*/false, URV(cause),
               PrivilegeMode::Machine, false /*virt*/, origPc,
               ldStFaultAddr_, 0);
  return false;
}


template <typename URV>
bool
Hart<URV>::initiateNmi(URV cause, URV pcToSave, bool isDoubleTrap)
{
  URV nextPc = indexedNmi_ ? nmiPc_ + 4*cause : nmiPc_;

  if (extensionIsEnabled(RvExtension::Smrnmi))
    {
      MnstatusFields mnf{csRegs_.peekMnstatus()};
      if (mnf.bits_.NMIE == 0)
	return false;  // mnstatus.nmie is off

      hasInterrupt_ = true;
      interruptCount_++;

      mnf.bits_.NMIE = 0;  // Clear mnstatus.mnie

      mnf.bits_.MNPP = unsigned(privMode_);  // Save privilege mode
      setPrivilegeMode(PrivilegeMode::Machine);

      mnf.bits_.MNPV = virtMode_;  // Save virtual mode
      setVirtualMode(false);  // Clear virtual mode

      if (isRvZicfilp())
        {
          mnf.bits_.MNPELP = elp_;
          setElp(false);
        }

      if (not csRegs_.write(CsrNumber::MNEPC, privMode_, pcToSave))
        assert(0 and "Failed to write MNEPC register");
      // Spec (rnmi.html line 735): For an exception-triggered double trap (Smdbltrp),
      // bit MXLEN-1 of mncause must be 0. For a real hardware NMI, it is 1.
      if (not isDoubleTrap)
        cause |= URV(1) << (sizeof(URV)*8 - 1);
      if (not csRegs_.write(CsrNumber::MNCAUSE, privMode_, cause))
        assert(0 and "Failed to write MNCAUSE register");

      // Update mnstatus, need to poke it to clear NMIE
      if (not pokeCsr(CsrNumber::MNSTATUS, mnf.value_))
        assert(0 and "Failed to write MNSTATUS register");
      recordCsrWrite(CsrNumber::MNSTATUS);

      // Set the pc to the nmi handler.
      setPc(nextPc);
    }
  else
    undelegatedInterrupt(cause, pcToSave, nextPc);

  nmiCount_++;
  if (instFreq_)
    accumulateTrapStats(true);

  if (hasActiveTrigger())
    {
      bool isNmi = true;
      if (csRegs_.intTriggerHit(cause, privMode_, virtMode_, isBreakpInterruptEnabled(), isNmi))
        initiateException(ExceptionCause::BREAKP, pc_, 0, 0);

#if 0
      // This should be an option. Do we evaluate instruction address triggers on NMI?
      // Do we evaluate them on interrupts?
      if (hasActiveTrigger())
        {
          instAddrTriggerHit(pcToSave, 4 /*size*/, TriggerTiming::Before))
        }
#endif
    }

  return true;
}


template <typename URV>
void
Hart<URV>::undelegatedInterrupt(URV cause, URV pcToSave, URV nextPc)
{
  hasInterrupt_ = true;
  interruptCount_++;

  if (cancelLrOnTrap_)
    cancelLr(CancelLrCause::TRAP);  // Clear LR reservation (if any).

  PrivilegeMode origMode = privMode_;

  // NMI is taken in machine mode.
  setPrivilegeMode(PrivilegeMode::Machine);
  setVirtualMode(false);

  // Save address of instruction that caused the exception or address
  // of interrupted instruction.
  pcToSave = (pcToSave >> 1) << 1; // Clear least sig bit.
  if (not csRegs_.write(CsrNumber::MEPC, privMode_, pcToSave))
    assert(0 and "Failed to write EPC register");

  // Save the exception cause.
  if (not csRegs_.write(CsrNumber::MCAUSE, privMode_, cause))
    assert(0 and "Failed to write CAUSE register");

  // Clear mtval
  if (not csRegs_.write(CsrNumber::MTVAL, privMode_, 0))
    assert(0 and "Failed to write MTVAL register");

  // Update status register saving MIE in MPIE and previous privilege
  // mode in MPP by getting current value of mstatus, updating
  // its fields and putting it back.
  mstatus_.bits_.MPP = unsigned(origMode);
  mstatus_.bits_.MPIE = mstatus_.bits_.MIE;
  mstatus_.bits_.MIE = 0;
  writeMstatus();

  setPc(nextPc);
}


template <typename URV>
bool
Hart<URV>::peekIntReg(unsigned ix, URV& val) const
{
  if (ix < intRegs_.size())
    {
      val = intRegs_.read(ix);
      return true;
    }
  return false;
}


template <typename URV>
URV
Hart<URV>::peekIntReg(unsigned ix) const
{
  assert(ix < intRegs_.size());
  return intRegs_.read(ix);
}


template <typename URV>
bool
Hart<URV>::peekIntReg(unsigned ix, URV& val, std::string_view& name) const
{
  if (ix < intRegs_.size())
    {
      val = intRegs_.read(ix);
      name = intRegName(ix);
      return true;
    }
  return false;
}


template <typename URV>
bool
Hart<URV>::peekFpReg(unsigned ix, uint64_t& val) const
{
  if (not isRvf() and not isRvd())
    return false;

  if (ix < fpRegs_.size())
    {
      val = fpRegs_.readBitsRaw(ix);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::peekUnboxedFpReg(unsigned ix, uint64_t& val) const
{
  if (not isRvf() and not isRvd())
    return false;

  if (ix < fpRegs_.size())
    {
      val = fpRegs_.readBitsUnboxed(ix);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeFpReg(unsigned ix, uint64_t val)
{
  if (not isRvf() and not isRvd())
    return false;

  if (ix < fpRegs_.size())
    {
      fpRegs_.pokeBits(ix, val);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::pokeIntReg(unsigned ix, URV val)
{
  if (ix < intRegs_.size())
    {
      intRegs_.poke(ix, val);
      return true;
    }
  return false;
}


template <typename URV>
URV
Hart<URV>::peekCsr(CsrNumber csrn, bool quiet) const
{
  URV value = 0;

  if (not peekCsr(csrn, value))
    if (not quiet)
      std::cerr << "Warning: Invalid CSR number in peekCsr: 0x" << std::hex
		<<  unsigned(csrn) << std::dec << '\n';
  return value;
}


template <typename URV>
bool
Hart<URV>::peekCsr(CsrNumber csrn, URV& val, URV& reset, URV& writeMask,
		   URV& pokeMask, URV& readMask, bool virtMode) const
{
  const Csr<URV>* csr = csRegs_.getImplementedCsr(csrn);
  if (not csr)
    return false;

  if (not peekCsr(csrn, val, virtMode))
    return false;

  reset = csr->getResetValue();
  writeMask = csr->getWriteMask();
  pokeMask = csr->getPokeMask();
  readMask = csr->getReadMask();
  return true;
}


template <typename URV>
bool
Hart<URV>::peekCsr(CsrNumber csrn, URV& val, std::string_view& name) const
{
  const Csr<URV>* csr = csRegs_.getImplementedCsr(csrn);
  if (not csr)
    return false;

  if (not peekCsr(csrn, val))
    return false;

  name = csr->getName();
  return true;
}


template <typename URV>
bool
Hart<URV>::peekCsr(CsrNumber csrn, std::string_view field, URV& val) const
{
  const Csr<URV>* csr = csRegs_.getImplementedCsr(csrn);
  if (not csr)
    return false;

  return csr->field(field, val);
}


template <typename URV>
bool
Hart<URV>::processPmacfgChange(CsrNumber csr, URV newVal)
{
  using CN = CsrNumber;

  virtMem_.flushPteCache();  // PMA regions change -> cached PTE access results stale.

  auto maskCsr = CN{};

  auto ix = unsigned(csr);
  if (ix >= unsigned(CN::PMACFG0) and ix <= unsigned(CN::PMACFG15))
    {
      ix -= unsigned(CN::PMACFG0);
      maskCsr = csRegs_.advance(CN::PMAMASK0, ix);
    }
  else if (csr == CN::MIREG)
    {
      if (not CsRegs<URV>::isPmaSelect(peekCsr(CN::MISELECT), ix))
        return false;
      maskCsr = CN::MIREG2;
    }
  else
    return false;

  auto maskPtr = this->findCsr(maskCsr);
  bool hasMask = maskPtr and maskPtr->isImplemented();

  // We want the value actually written in the PMACFG CSR.
  if (not peekCsr(csr, newVal))
    return false;

  uint64_t low = 0, high = 0, mask = 0;
  Pma pma;

  if (pmaMgr_.unpackPmacfg(newVal, low, high, mask, pma))
    {
      if (not definePmaRegion(ix, low, high, pma))
	return false;

      // Mark region as having memory mapped registers if it overlapps such registers.
      pmaMgr_.updateMemMappedAttrib(ix);
      pmaMgr_.setAddressMask(ix, mask);

      if (hasMask and pmaMgr_.isLegalPmacfg(newVal))
        {
          // When PMACFG is written corresponding PMAMASK.MASK is set to all zeros which
          // translates to all ones in PmaManager.
          uint64_t maskVal = ((uint64_t(1) << 40) - 1) << 12; // Bits 52:12 all ones.
          URV prev = 0;
          if (not peekCsr(maskCsr, prev))
            assert(0);
          uint64_t value = prev & ~maskVal;
          if (not csRegs_.poke(maskCsr, value))
            assert(0);
          if (prev != value)
            csRegs_.recordWrite(maskCsr);
        }

      return true;
    }

  invalidatePmaEntry(ix);
  return true;
}


template <typename URV>
bool
Hart<URV>::processPmamaskChange(CsrNumber csr)
{
  using CN = CsrNumber;

  virtMem_.flushPteCache();  // PMA mask change -> cached PTE access results stale.

  auto cfgCsr = CN{};

  auto ix = unsigned(csr);
  if (ix >= unsigned(CN::PMAMASK0) and ix <= unsigned(CN::PMAMASK15))
    {
      ix -= unsigned(CN::PMAMASK0);
      cfgCsr = csRegs_.advance(CN::PMACFG0, ix);
    }
  else if (csr == CN::MIREG2)
    {
      if (not CsRegs<URV>::isPmaSelect(peekCsr(CN::MISELECT), ix))
        return false;
      cfgCsr = CN::MIREG;
    }
  else
    return false;

  auto maskPtr = this->findCsr(csr);
  bool hasMask = maskPtr and maskPtr->isImplemented();

  if (not hasMask)
    return true;

  URV val = 0;
  if (not peekCsr(csr, val))
    return false;

  auto mask = ~val;   // Bit interpretation is reversed in PmaManager.
  mask = (mask >> 12) << 12;  // Clear least sig 12 bits.
  mask = (mask << 12) >> 12;  // Cleat most sig 12 bits.

  URV cfgVal = 0;
  if (not peekCsr(cfgCsr, cfgVal))
    return false;

  uint64_t low = 0, high = 0, cfgMask = 0;
  Pma pma;

  if (pmaMgr_.unpackPmacfg(cfgVal, low, high, cfgMask, pma))
    mask &= cfgMask;

  pmaMgr_.setAddressMask(ix, mask);
  return true;
}


template <typename URV>
void
Hart<URV>::postCsrUpdate(CsrNumber csr, URV val, URV lastVal)
{
  using CN = CsrNumber;

  // A CSR write may change a timer input; invalidate to keep the timer fast-path correct.
  markTimerStale();

  // This makes sure that counters stop counting after corresponding
  // event reg is written.
  if (enableCounters_)
    if ((csr >= CN::MHPMEVENT3 and csr <= CN::MHPMEVENT31) or
        (csr >= CN::MHPMEVENT3H and csr <= CN::MHPMEVENT31H))
      {
	csRegs_.applyPerfEventAssign();
	return;
      }

  if (csr == CN::DCSR)
    {
      DcsrFields<URV> dcsr(val);
      dcsrStep_ = dcsr.bits_.STEP;
      dcsrStepIe_ = dcsr.bits_.STEPIE;
      return;
    }

  if (csr >= CN::PMPCFG0 and csr <= CN::PMPCFG15)
    {
      updateMemoryProtection();
      return;
    }

  if (csr >= CN::PMPADDR0 and csr <= CN::PMPADDR63)
    {
      unsigned config = csRegs_.getPmpConfigByteFromPmpAddr(csr);
      auto type = Pmp::Type((config >> 3) & 3);
      if (type != Pmp::Type::Off)
        updateMemoryProtection();
      return;
    }

  if ( (csr >= CN::PMACFG0 and csr <= CN::PMACFG15) or
       (csr == CN::MIREG and CsRegs<URV>::isPmaSelect(peekCsr(CN::MISELECT))) )
    {
      if (not processPmacfgChange(csr, val))
	assert(0 && "Error: Assertion failed");
      return;
    }

  if ( (csr >= CN::PMAMASK0 and csr <= CN::PMAMASK15) or
       (csr == CN::MIREG2 and CsRegs<URV>::isPmaSelect(peekCsr(CN::MISELECT))) )
    {
      if (not processPmamaskChange(csr))
        assert(0 && "Error: Assertion failed");
      return;
    }

  if (steeEnabled_ and csr == CN::C_MATP)
    {
      unsigned world = val & 1;
      stee_.setSecureWorld(world);
      virtMem_.setWorldId(world);
      return;
    }

  if (csr == CN::SATP or csr == CN::VSATP or csr == CN::HGATP)
    updateAddressTranslation();
  else if (csr == CN::FCSR or csr == CN::FRM or csr == CN::FFLAGS)
    markFsDirty(); // Update FS field of MSTATUS if FCSR is written

  // Update cached value of VTYPE
  if (csr == CN::VTYPE)
    {
      if (URV newVal = 0; peekCsr(csr, newVal))
        {
          VtypeFields<URV> vtype(newVal);
          bool vill = vtype.bits_.VILL;
          bool ma = vtype.bits_.VMA;
          bool ta = vtype.bits_.VTA;
          auto gm = GroupMultiplier(vtype.bits_.LMUL);
          auto ew = ElementWidth(vtype.bits_.SEW);
          vecRegs_.updateConfig(ew, gm, ma, ta, vill);
          vecRegs_.setAltfmt(vtype.bits_.ALTFMT);

          disas_.setVecSew(ew);
          disas_.setVecAltfmt(vtype.bits_.ALTFMT);
        }
    }
  else if (csr == CN::VL)
    vecRegs_.elemCount(val);

  if (csr == CN::VSTART or csr == CN::VXSAT or csr == CN::VXRM or
      csr == CN::VCSR or csr == CN::VL or csr == CN::VTYPE or
      csr == CN::VLENB)
    markVsDirty();

  if (csr == CN::MISA and lastVal != val)
    processExtensions(false);
  else if (csr == CN::MENVCFG or csr == CN::SENVCFG or csr == CN::HENVCFG or
           csr == CN::MENVCFGH or csr == CN::HENVCFGH)
    {
      updateTranslationPbmt();
      updateTranslationAdu();
      updateTranslationPmm();
      updateLandingPadEnable();
      if (shadowStackOn_)
        updateShadowStackEnable();
      csRegs_.updateSstc();
      csRegs_.updateSsp();
      stimecmpActive_ = csRegs_.menvcfgStce();
      vstimecmpActive_ = csRegs_.henvcfgStce();
    }
  else if (csr == CN::MSECCFG)
    {
      updateTranslationPmm();
      updateLandingPadEnable();

      auto msf = MseccfgFields<URV>(peekCsr(csr));
      pmpMgr_.setMmLockdown(msf.bits_.MML);
      pmpMgr_.setMmWhitelist(msf.bits_.MMWP);
      pmpMgr_.setRuleLockBypass(msf.bits_.RLB);
    }

  if (csr == CN::STIMECMP)
    {
      // An htif_getc may be pending, send char back to target.
      auto inFd = syscall_.effectiveFd(STDIN_FILENO);
      if (pendingHtifGetc_ and hasPendingInput(inFd))
        {
          uint64_t v = 0;
          peekMemory(fromHost_, v, true);
          if (v == 0)
            {
              int c = readCharNonBlocking(inFd);
              if (c > 0)
		{
		  memory_.poke(fromHost_, (uint64_t(1) << 56) | (char) c);
		  --pendingHtifGetc_;
		}
            }
        }
    }

  // Update cached values of M/S/VS/H STATUS.
  if (csr == CN::SSTATUS)
    {
      updateCachedSstatus();
      // Ssdbltrp: sstatus write path — SDT=1 forces SIE=0. Must be checked here
      // because updateCachedSstatus() already synced mstatus_ from the backing store,
      // so the peekMstatus() != mstatus_.value() branch below will be false.
      if (isRvssdbltrp() and mstatus_.bits_.SDT and mstatus_.bits_.SIE)
        {
          mstatus_.bits_.SIE = 0;
          writeMstatus();
        }
    }
  else if (csr == CN::VSSTATUS)
    updateCachedVsstatus();

  if (csRegs_.peekMstatus() != mstatus_.value())
    {
      updateCachedMstatus();
      if (isRvsmdbltrp() and mstatus_.bits_.MDT)
        {
          mstatus_.bits_.MIE = 0;  // When MDT is set to 1, MIE is cleared.
          writeMstatus();
        }
      // Ssdbltrp: mstatus write path — SDT=1 forces SIE=0
      // (supervisor.adoc §sstatus_sdt: "When sstatus.SDT is set to 1 by an
      //  explicit CSR write, SIE is cleared to 0.")
      if (isRvssdbltrp() and mstatus_.bits_.SDT and mstatus_.bits_.SIE)
        {
          mstatus_.bits_.SIE = 0;
          writeMstatus();
        }
      csRegs_.recordWrite(CN::MSTATUS);
    }
  else if (isRvh() and csRegs_.peekHstatus() != hstatus_.value())
    {
      updateCachedHstatus();
      if (csRegs_.peekHstatus() != hstatus_.value())
	csRegs_.recordWrite(CN::HSTATUS);
    }

  if (csr == CN::HVICTL)
    updateCachedHvictl();
  else if (csr == CN::MVIEN or csr == CN::MIDELEG)
    csRegs_.updateHidelegMasks();

  // FIXME: support mtimecmp
  if (csr == CN::TIME or csr == CN::STIMECMP or csr == CN::VSTIMECMP or
      csr == CN::HTIMEDELTA or csr == CN::MENVCFG or
      csr == CN::HENVCFG)  // MENVCFG/HENVCFG may enable/disbale STIMECMP/VSTIMECMP.
    processTimerInterrupt();

  if (imsic_)
    {
      if      (csr == CN::MTOPEI)   imsic_->checkMInterrupt();
      else if (csr == CN::STOPEI)   imsic_->checkSInterrupt();
      else if (csr == CN::VSTOPEI)  imsic_->checkGInterrupt(hstatus_.bits_.VGEIN);
    }

  effectiveMie_ = csRegs_.effectiveMie();
  effectiveSie_ = csRegs_.effectiveSie();
  effectiveVsie_ = csRegs_.effectiveVsie();

  updateCachedTriggerState();  // In case trigger control CSR written.
}


template <typename URV>
bool
Hart<URV>::pokeCsr(CsrNumber csr, URV val, bool virtMode)
{
  using CN = CsrNumber;
  if (csr == CN::VSTART or csr == CN::VXSAT or csr == CN::VXRM or
      csr == CN::VCSR or csr == CN::VL or csr == CN::VTYPE or
      csr == CN::VLENB)
    {
      // At start of test, the test-bench pokes VSTART even though MSTATUS.VS set to
      // off. We ignore the poke and return true (otherwise test-bench fails). We ignore
      // the poke because it would otherwise change MSTATUS.VS to dirty enabling vector
      // extension. The test-bench should not expect VSTART poke to succeed when
      // MSTATUS.VS is off.
      if (not isVecEnabled())
        return true;
    }

  URV lastVal = 0;

  // Some/all bits of some CSRs are read only to CSR instructions but
  // are modifiable. Use the poke method (instead of write) to make
  // sure modifiable value are changed.
  if (not csRegs_.peek(csr, lastVal, virtMode) or not csRegs_.poke(csr, val, virtMode))
    return false;

  postCsrUpdate(csr, val, lastVal);

  return true;
}


template <typename URV>
bool
Hart<URV>::peekVecReg(unsigned ix, std::vector<uint8_t>& value) const
{
  if (not isRvv())
    return false;

  if (ix >= vecRegs_.size())
    return false;

  auto data = vecRegs_.getVecData(ix);
  unsigned byteCount = vecRegs_.bytesPerRegister();
  value.resize(byteCount);

  for (unsigned i = 0; i < byteCount; ++i)
    value.at(i) = data[byteCount - 1 - i];

  return true;
}


template <typename URV>
bool
Hart<URV>::pokeVecReg(unsigned ix, const std::vector<uint8_t>& val)
{
  if (not isRvv() or ix >= vecRegs_.size() or val.empty())
    return false;

  auto regData = vecRegs_.getVecData(ix);
  if (regData.empty())
    return false;

  // Bytes in val are in reverse order (most signficant first).
  std::vector<uint8_t> data = val;
  std::reverse(data.begin(), data.end());

  uint32_t count = vecRegs_.bytesPerRegister();
  for (uint32_t i = 0; i < count; ++i)
    {
      uint8_t byte = i < data.size() ? data.at(i) : 0;
      regData[i] = byte;
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::peekVecRegLsb(unsigned ix, std::vector<uint8_t>& value) const
{
  if (not isRvv())
    return false;

  if (ix >= vecRegs_.size())
    return false;

  auto data = vecRegs_.getVecData(ix);
  unsigned byteCount = vecRegs_.bytesPerRegister();
  value.resize(byteCount);

  for (unsigned i = 0; i < byteCount; ++i)
    value.at(i) = data[i];

  return true;
}


template <typename URV>
bool
Hart<URV>::pokeVecRegLsb(unsigned ix, const std::vector<uint8_t>& val)
{
  if (not isRvv() or ix >= vecRegs_.size() or val.empty())
    return false;

  auto regData = vecRegs_.getVecData(ix);
  if (regData.empty())
    return false;

  uint32_t count = vecRegs_.bytesPerRegister();
  for (uint32_t i = 0; i < count; ++i)
    {
      uint8_t byte = i < val.size() ? val.at(i) : 0;
      regData[i] = byte;
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::pokeVecRegLsb(unsigned ix, const std::span<const uint8_t>& val)
{
  if (not isRvv() or ix > vecRegs_.size() or val.empty())
    return false;

  auto regData = vecRegs_.getVecData(ix);
  if (regData.empty())
    return false;

  uint32_t count = vecRegs_.bytesPerRegister();
  for (uint32_t i = 0; i < count; ++i)
    {
      uint8_t byte = i < val.size() ? val[i] : 0;
      regData[i] = byte;
    }

  return true;
}


template <typename URV>
URV
Hart<URV>::peekPc() const
{
  return pc_;
}


template <typename URV>
void
Hart<URV>::pokePc(URV address)
{
  setPc(address);
  bbPc_ = pc_;
}


template <typename URV>
bool
Hart<URV>::findIntReg(std::string_view name, unsigned& num) const
{
  if (intRegs_.findReg(name, num))
    return true;

  unsigned n = 0;
  if (parseNumber<unsigned>(name, n) and n < intRegs_.size())
    {
      num = n;
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::findFpReg(std::string_view name, unsigned& num) const
{
  if (not isRvf())
    return false;   // Floating point extension not enabled.

  if (fpRegs_.findReg(name, num))
    return true;

  if (name.empty())
    return false;

  if (name.at(0) == 'f')
    {
      std::string_view numStr = name.substr(1);
      unsigned n = 0;
      if (parseNumber<unsigned>(numStr, num) and n < fpRegCount())
	return true;
    }

  unsigned n = 0;
  if (parseNumber<unsigned>(name, n) and n < fpRegCount())
    {
      num = n;
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::findVecReg(std::string_view name, unsigned& num) const
{
  if (not isRvv())
    return false;

  return VecRegs::findReg(name, num);
}


template <typename URV>
Csr<URV>*
Hart<URV>::findCsr(std::string_view name)
{
  Csr<URV>* csr = csRegs_.findCsr(name);

  if (not csr)
    {
      unsigned n = 0;
      if (parseNumber<unsigned>(name, n))
	csr = csRegs_.findCsr(CsrNumber(n));
    }

  return csr;
}


template <typename URV>
bool
Hart<URV>::configCsr(std::string_view name, bool implemented, URV resetValue,
                     URV mask, URV pokeMask, bool shared)
{
  return csRegs_.configCsr(name, implemented, resetValue, mask, pokeMask, shared);
}


template <typename URV>
bool
Hart<URV>::configCsrByUser(std::string_view name, bool implemented, URV resetValue,
			   URV mask, URV pokeMask, bool shared, bool isDebug, bool isHExt)
{
  return csRegs_.configCsrByUser(name, implemented, resetValue, mask, pokeMask, shared,
                                 isDebug, isHExt);
}


template <typename URV>
bool
Hart<URV>::defineCsr(std::string name, CsrNumber num, bool implemented, URV resetVal,
		     URV mask, URV pokeMask)
{
  bool mandatory = false, quiet = true;
  auto c = csRegs_.defineCsr(std::move(name), num, mandatory, implemented, resetVal,
			     mask, pokeMask, quiet);
  return c != nullptr;
}


template <typename URV>
bool
Hart<URV>::configIsa(std::string_view isa, bool updateMisa)
{
  if (not isa_.configIsa(isa))
    return false;

  if (updateMisa)
    {
      Csr<URV>* csr = this->findCsr("misa");
      if (not csr)
	return false;

      URV misaReset = csr->getResetValue();
      if (isa_.isEnabled(RvExtension::A))
	misaReset |= URV(1);
      if (isa_.isEnabled(RvExtension::B))
	misaReset |= URV(2);
      if (isa_.isEnabled(RvExtension::C))
	misaReset |= URV(4);
      if (isa_.isEnabled(RvExtension::D))
	misaReset |= URV(8);
      if (isa_.isEnabled(RvExtension::F))
	misaReset |= URV(32);
      if (isa_.isEnabled(RvExtension::M))
	misaReset |= URV(0x1000);
      if (isa_.isEnabled(RvExtension::V))
	misaReset |= URV(0x200000);

      URV mask = 0, pokeMask = 0;
      bool implemented = true, shared = true;

      if (not this->configCsr("misa", implemented, misaReset, mask, pokeMask, shared))
	return false;
    }

  // Make VTYPE.ALTFMT writable if extension zvfbfa, zvfofp8min, zvfwbdota16bf, or zvfqwbdota8f.
  if (isa_.isEnabled(RvExtension::Zvfbfa) or isa_.isEnabled(RvExtension::Zvfofp8min)
      or isa_.isEnabled(RvExtension::Zvfwbdota16bf) or isa_.isEnabled(RvExtension::Zvfqwbdota8f))
    {
      auto csr = csRegs_.findCsr(CsrNumber::VTYPE);
      URV pm = csr->getPokeMask();
      VtypeFields<URV> fields(pm);
      fields.bits_.ALTFMT = 1;
      csr->setPokeMask(fields.value_);
      csr->setWriteMask(fields.value_);
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::configMachineModePerfCounters(unsigned numCounters, bool cof)
{
  return csRegs_.configMachineModePerfCounters(numCounters, cof);
}


template <typename URV>
bool
Hart<URV>::configUserModePerfCounters(unsigned numCounters)
{
  return csRegs_.configUserModePerfCounters(numCounters);
}


template <typename URV>
bool
Hart<URV>::configMemoryProtectionGrain(uint64_t size)
{
  bool ok = true;

  if (size < 4)
    {
      if (hartIx_ == 0)
	std::cerr << "Error: Memory protection grain size (" << size << ") is "
		  << "smaller than 4. Using 4.\n";
      size = 4;
      ok = false;
    }

  auto log2Size = uint64_t(std::bit_width(size) - 1);
  uint64_t powerOf2 = uint64_t(1) << log2Size;
  if (size != powerOf2)
    {
      if (hartIx_ == 0)
	std::cerr << "Error: Memory protection grain size (0x" << std::hex
		  << size << ") is not a power of 2. Using: 0x"
		  << powerOf2 << std::dec << '\n';
      size = powerOf2;
      ok = false;
    }

  uint64_t limit = sizeof(URV)*8 + 3;
  if constexpr (sizeof(URV) == 4)
    {
      if (log2Size > limit)  // This can only happen in RV32.
        {
          if (hartIx_ == 0)
            std::cerr << "Error: Memory protection grain size (0x" << std::hex
                      << size << ") is larger than 2 to the power "
                      << std::dec << limit << ". "
                      << "Using 2 to the power " << limit << ".\n";
          size = uint64_t(1) << limit;
          powerOf2 = size;
          log2Size = limit;
          ok = false;
        }
    }

  unsigned pmpG = log2Size - 2;
  pmpMgr_.setPmpG(pmpG);

  return ok;
}


template <typename URV>
bool
Hart<URV>::configGuestInterruptCount(unsigned n)
{
  csRegs_.setGuestInterruptCount(n);
  return true;
}


template <typename URV>
void
Hart<URV>::recordDivInst(unsigned rd, URV value)
{
  hasLastDiv_ = true;
  priorDivRdVal_ = value;

  lastDivRd_ = rd;
}


template <typename URV>
bool
Hart<URV>::redirectOutputDescriptor(int fd, const std::string& path)
{
  return syscall_.redirectOutputDescriptor(fd, path);
}


template <typename URV>
bool
Hart<URV>::redirectInputDescriptor(int fd, const std::string& path)
{
  return syscall_.redirectInputDescriptor(fd, path);
}


template <typename URV>
bool
Hart<URV>::cancelLastDiv()
{
  if (not hasLastDiv_)
    return false;

  hasLastDiv_ = false;
  return pokeIntReg(lastDivRd_, priorDivRdVal_);
}



template <typename URV>
void
Hart<URV>::undoForTrigger()
{
  uint64_t value = 0;
  int regIx = intRegs_.getLastWrittenReg(value);
  if (regIx >= 0)
    {
      pokeIntReg(regIx, value);
      intRegs_.clearLastWrittenReg();
    }

  regIx = fpRegs_.getLastWrittenReg(value);
  if (regIx >= 0)
    {
      pokeFpReg(regIx, value);
      fpRegs_.clearLastWrittenReg();
    }

  setPc(currPc_);
}


/// Return true if given hart is in debug mode and the stop count bit of
/// the DSCR register is set.
template <typename URV>
bool
isDebugModeStopCount(const Hart<URV>& hart)
{
  if (not hart.inDebugMode())
    return false;

  URV dcsrVal = 0;
  if (not hart.peekCsr(CsrNumber::DCSR, dcsrVal))
    return false;

  DcsrFields<URV> dcsr(dcsrVal);

  return bool(dcsr.bits_.STOPCOUNT);
}


template <typename URV>
void
Hart<URV>::updatePerformanceCounters(const DecodedInst& di)
{
  InstId id = di.instId();

  if (isDebugModeStopCount(*this))
    return;

  if (hasInterrupt_)
    return;

  if (not hasActivePerfCounter())
    return;

  PerfRegs& pregs = csRegs_.mPerfRegs_;

  using EN = EventNumber;

  // We do not update the performance counters if an instruction causes an exception
  // unless it is an ebreak or an ecall.
  if (hasException_)
    {
      if (id == InstId::ebreak or id == InstId::c_ebreak or id == InstId::ecall)
	{
	  pregs.updateCounters(EN::InstCommited, prevPerfControl_, lastPriv_, lastVirt_);
	  if (id == InstId::ebreak or id == InstId::c_ebreak)
	    pregs.updateCounters(EN::Ebreak, prevPerfControl_, lastPriv_, lastVirt_);
	  else if (id == InstId::ecall)
	    pregs.updateCounters(EN::Ecall, prevPerfControl_, lastPriv_, lastVirt_);
	}
      return;
    }

  pregs.updateCounters(EN::InstCommited, prevPerfControl_, lastPriv_, lastVirt_);
  pregs.updateCounters(EN::CpuCycles, prevPerfControl_, lastPriv_, lastVirt_);

  if (isCompressedInst(di.inst()))
    pregs.updateCounters(EN::Inst16Commited, prevPerfControl_, lastPriv_, lastVirt_);
  else
    pregs.updateCounters(EN::Inst32Commited, prevPerfControl_, lastPriv_, lastVirt_);

  switch (di.extension())
    {
    case RvExtension::I:
      if (id == InstId::fence)
	pregs.updateCounters(EN::Fence, prevPerfControl_, lastPriv_, lastVirt_);
      else if (id == InstId::fence_i)
	pregs.updateCounters(EN::Fencei, prevPerfControl_, lastPriv_, lastVirt_);
      else if (id == InstId::mret)
	pregs.updateCounters(EN::Mret, prevPerfControl_, lastPriv_, lastVirt_);
      else if (di.isBranch())
	{
	  pregs.updateCounters(EN::Branch, prevPerfControl_, lastPriv_, lastVirt_);
	  if (lastBranchTaken_)
	    pregs.updateCounters(EN::BranchTaken, prevPerfControl_, lastPriv_, lastVirt_);
          if (di.isConditionalBranch())
            pregs.updateCounters(EN::CondBranch, prevPerfControl_, lastPriv_, lastVirt_);
          else
            {
              if (di.isCall())
                pregs.updateCounters(EN::Call, prevPerfControl_, lastPriv_, lastVirt_);
              if (di.isBranchToRegister())
                {
                  pregs.updateCounters(EN::IndirectBranch, prevPerfControl_, lastPriv_, lastVirt_);
                  if (di.isReturn())
                    pregs.updateCounters(EN::Return, prevPerfControl_, lastPriv_, lastVirt_);
                }
              else
                pregs.updateCounters(EN::DirectBranch, prevPerfControl_, lastPriv_, lastVirt_);
            }
	}
      else if (id != InstId::illegal)
	pregs.updateCounters(EN::Alu, prevPerfControl_, lastPriv_, lastVirt_);
      break;

    case RvExtension::Zmmul:
    case RvExtension::M:
      if (di.isMultiply())
	pregs.updateCounters(EN::Mult, prevPerfControl_, lastPriv_, lastVirt_);
      else
	pregs.updateCounters(EN::Div, prevPerfControl_, lastPriv_, lastVirt_);
      pregs.updateCounters(EN::MultDiv, prevPerfControl_, lastPriv_, lastVirt_);
      break;

    case RvExtension::A:
      if (id == InstId::lr_w or id == InstId::lr_d)
	pregs.updateCounters(EN::Lr, prevPerfControl_, lastPriv_, lastVirt_);
      else if (id == InstId::sc_w or id == InstId::sc_d)
        {
          pregs.updateCounters(EN::Sc, prevPerfControl_, lastPriv_, lastVirt_);
          EN en = scPassed_ ? EN::ScPass : EN::ScFail;
          pregs.updateCounters(en, prevPerfControl_, lastPriv_, lastVirt_);
        }
      else
	pregs.updateCounters(EN::Atomic, prevPerfControl_, lastPriv_, lastVirt_);
      break;

    case RvExtension::F:
      if (fpLdStCountAsFp_ or not (di.isLoad() or di.isStore()))
        {
          pregs.updateCounters(EN::Fp, prevPerfControl_, lastPriv_, lastVirt_);
          pregs.updateCounters(EN::FpSingle, prevPerfControl_, lastPriv_, lastVirt_);
        }
      break;

    case RvExtension::D:
      if (fpLdStCountAsFp_ or not (di.isLoad() or di.isStore()))
        {
          pregs.updateCounters(EN::Fp, prevPerfControl_, lastPriv_, lastVirt_);
          pregs.updateCounters(EN::FpDouble, prevPerfControl_, lastPriv_, lastVirt_);
        }
      break;

    case RvExtension::Zfh:
      if (fpLdStCountAsFp_ or not (di.isLoad() or di.isStore()))
        {
          pregs.updateCounters(EN::Fp, prevPerfControl_, lastPriv_, lastVirt_);
          pregs.updateCounters(EN::FpHalf, prevPerfControl_, lastPriv_, lastVirt_);
        }
      break;

    case RvExtension::V:
      {
        bool ld = di.isVectorLoad(), st = di.isVectorStore();
        if (ld)
          pregs.updateCounters(EN::VectorLoad, prevPerfControl_, lastPriv_, lastVirt_);
        if (st)
          pregs.updateCounters(EN::VectorStore, prevPerfControl_, lastPriv_, lastVirt_);
        bool ldSt = ld or st;
        if (vecLdStCountAsVec_ or not ldSt)
        pregs.updateCounters(EN::Vector, prevPerfControl_, lastPriv_, lastVirt_);
      }
      break;

    case RvExtension::Zba:
    case RvExtension::Zbb:
    case RvExtension::Zbc:
    case RvExtension::Zbs:
      pregs.updateCounters(EN::Bitmanip, prevPerfControl_, lastPriv_, lastVirt_);
      break;

    case RvExtension::Zicsr:
      if ((id == InstId::csrrw or id == InstId::csrrwi))
	{
	  auto evNum = di.op0() == 0 ? EN::CsrWrite : EN::CsrReadWrite;
	  pregs.updateCounters(evNum, prevPerfControl_, lastPriv_, lastVirt_);
	}
      else
	{
	  auto evNum = di.op1() == 0 ? EN::CsrRead : EN::CsrReadWrite;
	  pregs.updateCounters(evNum, prevPerfControl_, lastPriv_, lastVirt_);
	}
      pregs.updateCounters(EN::Csr, prevPerfControl_, lastPriv_, lastVirt_);
      break;

    default:
      break;
    }

  // Some insts (e.g. flw) can be both load/store and FP
  if (di.isPerfLoad())
    {
      pregs.updateCounters(EN::Load, prevPerfControl_, lastPriv_, lastVirt_);
      if (misalignedLdSt_)
	pregs.updateCounters(EN::MisalignLoad, prevPerfControl_, lastPriv_, lastVirt_);
    }
  else if (di.isPerfStore())
    {
      pregs.updateCounters(EN::Store, prevPerfControl_, lastPriv_, lastVirt_);
      if (misalignedLdSt_)
	pregs.updateCounters(EN::MisalignStore, prevPerfControl_, lastPriv_, lastVirt_);
    }
}


template <typename URV>
void
Hart<URV>::updatePerformanceCountersForCsr(const DecodedInst& di)
{
  if (not enableCounters_ or not hasActivePerfCounter())
    return;

  if (di.isCsr())
    updatePerformanceCounters(di);
}


template <typename URV>
void
Hart<URV>::accumulateInstructionStats(const DecodedInst& di)
{
  const InstEntry& info = *(di.instEntry());

  if (enableCounters_ and hasActivePerfCounter())
    {
      // For CSR instruction we need to let the counters count before
      // letting CSR instruction write. Consequently we update the counters
      // from within the code executing the CSR instruction.
      if (not info.isCsr())
        updatePerformanceCounters(di);
    }

  prevPerfControl_ = perfControl_;

  // We do not update the instruction stats if an instruction causes
  // an exception unless it is an ebreak or an ecall.
  InstId id = info.instId();
  if (hasException_ and id != InstId::ecall and id != InstId::ebreak and
      id != InstId::c_ebreak)
    return;

  misalignedLdSt_ = false;

  if (not instFreq_)
    return;

  InstProfile* profPtr = nullptr;
  if (info.isVector())
    profPtr = instProfs_.find(id, vecRegs_.elemWidth());
  else
    profPtr = instProfs_.find(id);

  if (not profPtr)
    return;

  InstProfile& prof = *profPtr;

  prof.freq_++;
  if (lastPriv_ == PrivilegeMode::User)
    prof.user_++;
  else if (lastPriv_ == PrivilegeMode::Supervisor)
    prof.supervisor_++;
  else if (lastPriv_ == PrivilegeMode::Machine)
    prof.machine_++;
}


template <typename URV>
void
Hart<URV>::accumulateTrapStats(bool isNmi)
{
  URV causeVal = peekCsr(CsrNumber::MCAUSE);

  // If most sig bit of mcause is 1, we have an interrupt.
  bool isInterrupt = causeVal >> (sizeof(causeVal)*8 - 1);

  if (isNmi)
    ;
  else if (isInterrupt)
    {
      causeVal = (causeVal << 1) >> 1;  // Clear most sig bit.
      if (causeVal < interruptStat_.size())
        interruptStat_.at(causeVal)++;
    }
  else
    {
      if (causeVal < exceptionStat_.size())
	exceptionStat_.at(causeVal)++;
    }
}


template <typename URV>
inline
void
Hart<URV>::clearTraceData()
{
  // Always needed: the page-table-walk record is consumed by translation every
  // instruction, and these two flags feed branch tracing.
  virtMem_.clearPageTableWalk();
  lastBranchTaken_ = false;
  misalignedLdSt_ = false;

#if 0
  // The remaining clears reset per-instruction last-written register/CSR/vector/memory
  // trace state. It is only read back by a consumer: instruction logging, an armed debug
  // trigger, instruction-frequency stats, perfApi, or MCM. With none active, skip it.
  if (not (traceFileActive_ or activeTrig_ or instFreq_ or perfApi_ or mcm_))
    return;
#endif

  intRegs_.clearLastWrittenReg();
  fpRegs_.clearLastWrittenReg();
  csRegs_.clearLastWrittenRegs();
  vecRegs_.clearTraceData();
  pmpMgr_.clearPmpTrace();
  pmaMgr_.clearPmaTrace();
  if (imsic_)
    imsic_->clearTrace();
}


template <typename URV>
inline
void
Hart<URV>::setTargetProgramBreak(URV addr)
{
  uint64_t progBreak = addr;

  uint64_t pageAddr = memory_.getPageStartAddr(addr);
  if (pageAddr != addr)
    progBreak = pageAddr + memory_.pageSize();

  syscall_.setTargetProgramBreak(progBreak);
}


template <typename URV>
inline
bool
pokeString(Hart<URV>& hart, uint64_t addr, std::string_view str)
{
  for (uint8_t c : str)
    if (not hart.pokeMemory(addr++, c, true))
      return false;
  return hart.pokeMemory(addr, uint8_t(0), true);   // null byte at end
}


template <typename URV>
inline
bool
Hart<URV>::setTargetProgramArgs(const std::vector<std::string>& args,
                                const std::vector<std::string>& envVars)
{
  URV sp = peekIntReg(RegSp);

  // Make sp 16-byte aligned.
  if ((sp & 0xf) != 0)
    sp -= (sp & 0xf);

  // Push the arguments on the stack recording their addresses.
  std::vector<URV> argvAddrs;  // Address of the argv strings.
  for (const auto& arg : args)
    {
      sp -= arg.size() + 1;  // Make room for arg and null char.
      argvAddrs.push_back(sp);
      if (not pokeString(*this, sp, arg))
	return false;
    }
  argvAddrs.push_back(0);  // Null pointer at end of argv.

  // Setup default envp on the stack (LANG is needed for clang compiled code).
  static constexpr auto envs = std::to_array<std::string_view>({ "LANG=C", "LC_ALL=C" });
  std::vector<URV> envpAddrs;  // Addresses of the envp strings.
  for (const auto& env : envs)
    {
      sp -= env.size() + 1;  // Make room for env entry and null char.
      envpAddrs.push_back(sp);
      if (not pokeString(*this, sp, env))
	return false;
    }
  // Setup user envp on the stack.
  for (const auto& env : envVars)
    {
      sp -= env.size() + 1;  // Make room for env entry and null char.
      envpAddrs.push_back(sp);
      if (not pokeString(*this, sp, env))
	return false;
    }
  envpAddrs.push_back(0);  // Null pointer at end of envp.

  // Push on stack null for aux vector.
  sp -= sizeof(URV);
  if (not pokeMemory(sp, URV(0), true))
    return false;

  // Push argv/envp entries on the stack.
  sp -= URV(envpAddrs.size() + argvAddrs.size() + 1) * sizeof(URV); // Make room for envp, argv, & argc

  if ((sp & 0xf) != 0)
    sp -= (sp & 0xf);  // Make sp 16-byte aligned.

  size_t ix = 1;  // Index 0 is for argc

  // Push argv entries on the stack.
  for (auto addr : argvAddrs)
    {
      if (bigEnd_)
	addr = util::byteswap(addr);
      if (not pokeMemory(sp + ix++*sizeof(URV), addr, true))
	return false;
    }

  // Set environ for newlib. This is superfluous for Linux.
  URV ea = sp + ix*sizeof(URV);  // Address of envp array
  ElfSymbol sym;
  if (memory_.findElfSymbol("environ", sym))
    {
      if (bigEnd_)
	ea = util::byteswap(ea);
      pokeMemory(URV(sym.addr_), ea, true);
    }

  // Push envp entries on the stack.
  for (auto addr : envpAddrs)
    {
      if (bigEnd_)
	addr = util::byteswap(addr);
      if (not pokeMemory(sp + ix++*sizeof(URV), addr, true))
	return false;
    }

  // Put argc on the stack.
  URV argc = args.size();
  if (bigEnd_)
    argc = util::byteswap(argc);
  if (not pokeMemory(sp, argc, true))
    return false;

  if (not pokeIntReg(RegSp, sp))
    return false;

  return true;
}


template <typename URV>
int
Hart<URV>::lastVecReg(const DecodedInst& di, unsigned& group) const
{
  unsigned groupX8 = 8;
  int vecReg = vecRegs_.getLastWrittenReg(groupX8);
  if (vecReg < 0)
    {
      group = 0;
      return vecReg;
    }

  // We want to report all the registers in the group.
  group  = (groupX8 >= 8) ? groupX8/8 : 1;
  vecReg = static_cast<int>(di.op0());  // Make sure we have 1st reg in group.
  unsigned fc = di.vecFieldCount();
  if (fc > 0)
    group = group * fc;  // Scale by field count

  return vecReg;
}


template <typename URV>
void
Hart<URV>::lastCsr(std::vector<CsrNumber>& csrs,
		   std::vector<unsigned>& triggers) const
{
  csRegs_.getLastWrittenRegs(csrs, triggers);
}


template <typename URV>
void
handleExceptionForGdb(WdRiscv::Hart<URV>& hart, int fd);


// Return true if debug mode is entered and false otherwise.
template <typename URV>
bool
Hart<URV>::takeTriggerAction(FILE* traceFile, URV pc, URV info,
			     uint64_t instrTag, const DecodedInst* di)
{
  // Check triggers configuration to determine action: take breakpoint
  // exception or enter debugger (or nothing).

  bool enteredDebug = false;

  if (csRegs_.hasEnterDebugModeTripped())
    {
      enterDebugMode_(DebugModeCause::TRIGGER, pc);
      enteredDebug = true;
    }
  else if (csRegs_.hasBreakpTripped())
    {
      initiateException(ExceptionCause::BREAKP, pc, info);
      if (dcsrStep_)
	{
	  enterDebugMode_(DebugModeCause::TRIGGER, pc_);
	  enteredDebug = true;
	}
    }

  if (traceFile)
    {
      std::string instStr;
      if (di)
        printDecodedInstTrace(*di, instrTag, instStr, traceFile);
      else
        {
          uint32_t inst = 0;
          readInst(currPc_, inst);
          printInstTrace(inst, instrTag, instStr, traceFile);
        }
    }

  return enteredDebug;
}


template <typename URV>
bool
Hart<URV>::getLastVecLdStRegsUsed(const DecodedInst& di, unsigned opIx,
                                  unsigned& regBase, unsigned& regCount) const
{
  unsigned elemSize = 0, elemCount = 0;
  if (not vecRegs_.vecLdStElemsUsed(elemSize, elemCount))
    return false;

  if (not elemCount)
    return false;

  if (di.ithOperandType(opIx) != OperandType::VecReg)
    return false;

  unsigned fieldCount = di.vecFieldCount();

  // Index register use EEW encoded in the instruction.
  bool isIndexed = di.isVectorLoadIndexed() or di.isVectorStoreIndexed();
  if (isIndexed and opIx == 2)  // Operand is index register.
    {
      unsigned width = (di.inst() >> 12) & 7;
      switch (width)
      {
        case 0: elemSize = 1; break;
        case 5: elemSize = 2; break;
        case 6: elemSize = 4; break;
        case 7: elemSize = 8; break;
        default: assert(false);
      }
    }

  unsigned elemsPerVec = vecRegSize() / elemSize;

  // Trim by vstart.
  unsigned start = csRegs_.peekVstart();
  regBase = di.ithOperand(opIx) + (start/elemsPerVec);

  // Trim by vl.
  assert(di.ithOperandType(opIx) == OperandType::VecReg);

  unsigned group = vecOpEmul(opIx);

  if (fieldCount and opIx == 2 and isIndexed)  // Operand 2 in vector index register.
    elemCount /= fieldCount;  // Adjust index vector element count.

  regCount = group;
  if (elemCount < elemsPerVec*group)
    regCount = (elemCount + elemsPerVec - 1) / elemsPerVec;

  return true;
}


//NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
// True if keyboard interrupt (user hit control-c) pending.
static std::atomic<bool> userStop = false;

// Negation of the preceding variable. Exists for speed (obsessive
// compulsive engineering).
static std::atomic<bool> noUserStop = true;
//NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

void
forceUserStop(int)
{
  userStop = true;
  noUserStop = false;
}


static void
clearUserStop()
{
  userStop = false;
  noUserStop = true;
}


/// Install a signal handler for SIGINT (keyboard) interrupts on
/// construction. Restore to previous handlers on destruction. This
/// allows us to catch a control-c typed by the user in the middle of
/// a long-run and return to the top level interactive command
/// processor.
class SignalHandlers
{
public:

  SignalHandlers()
  {
    clearUserStop();

    struct sigaction newKbdAction{};
    memset(&newKbdAction, 0, sizeof(newKbdAction));
    newKbdAction.sa_handler = forceUserStop;
    sigaction(SIGINT, &newKbdAction, &prevKbdAction_);
  }

  ~SignalHandlers()
  {
    sigaction(SIGINT, &prevKbdAction_, nullptr);
  }

private:

  struct sigaction prevKbdAction_{};
};


template <typename URV>
inline
bool
Hart<URV>::fetchInstWithTrigger(URV addr, uint64_t& physAddr, uint32_t& inst, FILE* file)
{
  // Process pre-execute address trigger.
  bool hasTrig = hasActiveInstTrigger();
  if (hasTrig)
    {
      instAddrTriggerHit(addr, 4 /*size*/, TriggerTiming::Before);
      if (breakpOrEnterDebugTripped())
        {
          if (mcycleEnabled())
            ++cycleCount_;
          takeTriggerAction(file, addr, addr /*info*/, execCount_, nullptr /*di*/);
          return false;  // Next instruction in trap handler.
        }
    }

  setMemProtAccIsFetch(true);

  // Fetch instruction.
  bool fetch = fetchInst(addr, physAddr, inst);
  if (not fetch or
      (injectException_ != ExceptionCause::NONE and not injectExceptionIsLd_))
    {
      if (mcycleEnabled())
	++cycleCount_;

      // Fetch was successful, but injected exception.
      if (fetch)
        {
          uint64_t tval = pc_;  // For MTVAL/STVAL
          // Adjust MTVAL/STVAL for line crossers if fault is on 2nd line.
          if (injectAddr_ != 0 and cacheLineNum(pc_) != cacheLineNum(injectAddr_))
            tval = cacheLineAlign(tval) + cacheLineSize();
          initiateException(injectException_, pc_, tval);
        }

      std::string instStr;
      printInstTrace(inst, execCount_, instStr, file);
      return false;  // Next instruction in trap handler.
    }

  // Process pre-execute opcode trigger.
  if (hasTrig)
    {
      instOpcodeTriggerHit(inst, TriggerTiming::Before);
      if (breakpOrEnterDebugTripped())
        {
          if (mcycleEnabled())
            ++cycleCount_;
          takeTriggerAction(file, addr, addr /*info*/, execCount_, nullptr /*di*/);
          return false;  // Next instruction in trap handler.
        }
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::untilAddress(uint64_t address, FILE* traceFile)
{
  traceFileActive_ = (traceFile != nullptr);  // gates the per-instruction trace reset

  std::string instStr;
  instStr.reserve(128);

  const uint64_t instLim = instCountLim_;
  const uint64_t retInstLim = retCountLim_;

  bool statsEnabled = instFreq_ or enableCounters_;
  bool roiActive = hasRoiTraceEnabled();
  bool traceBranchOn = branchBuffer_.max_size() and not branchTraceFile_.empty();

  // Check for gdb break every 1000000 instructions.
  unsigned gdbCount = 0, gdbLimit = 1000000;

  if (enableGdb_)
    handleExceptionForGdb(*this, gdbInputFd_);

  uint64_t& effectiveInstCounter = hasRoiTraceEnabled()? traceCount_ : execCount_;

  while (pc_ != address and effectiveInstCounter < instLim and
           retireCount_ < retInstLim)
    {
      if (userStop)
        break;

      resetExecInfo(); clearTraceData();

      bool traceWasOn = traceOn_;
      if (enableGdb_ and ++gdbCount >= gdbLimit)
        {
          gdbCount = 0;
          if (hasPendingInput(gdbInputFd_))
            {
              handleExceptionForGdb(*this, gdbInputFd_);
              continue;
            }
        }

      if (preInst_)
        {
          bool halt = false, reset = false;
          while (true)
            {
              preInst_(*this, halt, reset);
              if (reset)
                {
                  this->reset();
                  return true;
                }
              if (not halt)
                break;
            }
        }

      try
	{
          // We want amo instructions to print in the same order as executed.
	  // This avoid interleaving of amo execution and tracing.
	  static std::mutex execMutex;
	  auto lock = (ownTrace_ or !traceFile)? std::unique_lock<std::mutex>() : std::unique_lock<std::mutex>(execMutex);

	    tickTime();

          uint32_t inst = 0;
	  currPc_ = pc_;

	  ++execCount_;
	  if (mcycleEnabled())
	    ++cycleCount_;

          if (hasActiveTrigger() and icountTriggerFired() and breakpOrEnterDebugTripped())
            {
              icountTrig_ = true;
              if (takeTriggerAction(traceFile, currPc_, 0, execCount_, nullptr /*di*/))
                {
                  evaluateDebugStep();
                  icountTrig_ = false;
                  return true;
                }
              icountTrig_ = false;
              continue;
            }

          if (processExternalInterrupt(traceFile, instStr))
            {
              if (sdtrigOn_)
                {
                  if (hasActiveTrigger())
                    evaluateIcountTrigger();
                  evaluateDebugStep();
                }
              continue;  // Next instruction in trap handler.
            }

	  uint64_t physPc = 0;
          if (not fetchInstWithTrigger(pc_, physPc, inst, traceFile))
            {
              if (sdtrigOn_)
                {
                  if (hasActiveTrigger())
                    evaluateIcountTrigger();
                  evaluateDebugStep();
                }
              continue;  // Next instruction in trap handler.
            }

	  // Decode unless match in decode cache.
	  uint32_t ix = (physPc >> 1) & decodeCacheMask_;
	  DecodedInst* di = &decodeCache_[ix];
	  if (not di->isValid() or di->physAddress() != physPc or di->inst() != inst)
	    decode(pc_, physPc, inst, *di);
          di->resetAddr(pc_);

          // Increment pc and execute instruction
	  pc_ += di->instSize();
	  execute(di);

          if (hasActiveTrigger())
            evaluateIcountTrigger();

	  bool doStats = statsEnabled and (not roiActive or traceOn_);

	  if (hasException_)
	    {
              if (doStats)
                accumulateInstructionStats(*di);
	      printDecodedInstTrace(*di, execCount_, instStr, traceFile);
              evaluateDebugStep();
	      continue;
	    }

	  if (initStateFile_)
	    {
	      for (const auto& walk : virtMem_.getFetchWalks())
		for (auto addr : walk.pteAddrs())
                  dumpInitState("ipt", addr, addr);
	      for (const auto& walk : virtMem_.getDataWalks())
		for (auto addr : walk.pteAddrs())
                  dumpInitState("dpt", addr, addr);
	    }

	  if (breakpOrEnterDebugTripped())
	    {
	      URV tval = ldStFaultAddr_;
	      if (takeTriggerAction(traceFile, currPc_, tval, execCount_, di))
                {
                  evaluateDebugStep();
                  return true;
                }
	      continue;
	    }

          if (minstretEnabled())
            ++minstret_;

          // Unlike minstret, this is not inhibited.
          ++retireCount_;

	  if (bbFile_)
	    {
	      countBasicBlocks(bbPrevIsBranch_, physPc);
	      bbPrevIsBranch_ = di->isBranch();
	    }

	  if (instrLineTrace_)
	    memory_.traceInstructionLine(currPc_, physPc);

	  if (doStats)
	    accumulateInstructionStats(*di);

	  if (traceOn_) // and lastPriv_ == PrivilegeMode::User)
	    {
	      traceCount_++;
	      printDecodedInstTrace(*di, execCount_, instStr, traceFile);
              if (not traceWasOn)
                throw CoreException(CoreException::RoiEntry, "Taking snapshot on ROI entry.");
	    }

          if (sdtrigOn_)
            evaluateDebugStep();

          prevPerfControl_ = perfControl_;

	  if (traceBranchOn and (di->isBranch() or di->isXRet()))
	    traceBranch(di);
	}
      catch (const CoreException& ce)
	{
	  bool success = logStop(ce, execCount_, traceFile);
          if (ce.type() == CoreException::Snapshot or
              ce.type() == CoreException::RoiEntry or
              ce.type() == CoreException::SnapshotAndStop)
            throw;
          return success;
	}
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::runUntilAddress(uint64_t address, FILE* traceFile)
{
  struct timeval t0{};
  gettimeofday(&t0, nullptr);

  const uint64_t instLim = instCountLim_;
  const uint64_t retInstLim = retCountLim_;
  const uint64_t counter0 = execCount_;
  const uint64_t counter1 = retireCount_;

  // Setup signal handlers. Restore on destruction.
  SignalHandlers handlers;

  bool success = untilAddress(address, traceFile);

  if (execCount_ >= instLim or retireCount_ >= retInstLim)
    {
      std::cerr << "Info: Stopped -- Reached instruction limit hart=" << hartIx_ << "\n";
      if (failOnInstLimit_)
        success = false;
    }
  else if (pc_ == address)
    std::cerr << "Info: Stopped -- Reached end address hart=" << hartIx_ << "\n";

  // Simulator stats.
  struct timeval t1{};
  gettimeofday(&t1, nullptr);
  double elapsed = (double(t1.tv_sec - t0.tv_sec) +
		    double(t1.tv_usec - t0.tv_usec)*1e-6);

  uint64_t numInsts = execCount_ - counter0;
  uint64_t numRetInsts = retireCount_ - counter1;

  reportInstsPerSec(numInsts, numRetInsts, elapsed, userStop);
  return success;
}


template <typename URV>
bool
Hart<URV>::runSteps(uint64_t steps, bool& stop, FILE* traceFile)
{
  // Setup signal handlers. Restore on destruction.
  SignalHandlers handlers;

  const uint64_t instLim = instCountLim_;
  const uint64_t retInstLim = retCountLim_;
  URV stopAddr = stopAddrValid_? stopAddr_ : ~URV(0); // ~URV(0): No-stop PC.
  stop = false;

  for (unsigned i = 0; i < steps; i++)
    {
      if (execCount_ >= instLim or retireCount_ >= retInstLim)
        {
          stop = true;
          std::cerr << "Info: Stopped -- Reached instruction limit\n";
          return not failOnInstLimit_;
        }
      if (pc_ == stopAddr)
        {
          stop = true;
          std::cerr << "Info: Stopped -- Reached end address\n";
          return true;
        }

      singleStep(traceFile);

      if (hasTargetProgramFinished())
        {
          stop = true;
          return stepResult_;
        }
    }
  return true;
}


template <typename URV>
bool
Hart<URV>::simpleRun()
{
  // For speed: do not record/clear CSR changes.
  csRegs_.enableRecordWrite(false);
  pmpMgr_.enableTrace(false);
  virtMem_.enableTrace(false);

  bool success = true;

  try
    {
      while (true)
        {
          bool hasLim = (instCountLim_ < ~uint64_t(0)) or bbFile_ or instrLineTrace_;
          hasLim = hasLim or isRvs() or isRvu() or isRvv() or hasAclint() or imsic_ or aplic_;
          hasLim = hasLim or traceCacheOn_;
          hasLim = hasLim or canReceiveInterrupts() or hintOps_;

          if (hasLim)
            simpleRunWithLimit();
          else
            simpleRunNoLimit();

          if (userStop)
            {
              std::cerr << "Info: Stopped -- interrupted\n";
              break;
            }

          if (hasLim)
            {
              std::cerr << "Info: Stopped -- Reached instruction limit\n";
              if (failOnInstLimit_)
                success = false;
            }
          break;
        }
    }
  catch (const CoreException& ce)
    {
      success = logStop(ce, 0, nullptr);
      if (ce.type() == CoreException::Snapshot or
          ce.type() == CoreException::RoiEntry or
          ce.type() == CoreException::SnapshotAndStop)
        throw;
    }

  csRegs_.enableRecordWrite(true);
  pmpMgr_.enableTrace(true);
  virtMem_.enableTrace(true);

  return success;
}


template <typename URV>
void
Hart<URV>::dumpBasicBlocks()
{
  if (bbFile_)
    {
      bool first = true;
      for (const auto& kv : basicBlocks_)
        {
          const BbStat& stat = kv.second;
          if (stat.count_)
            {
              if (first)
                {
                  fprintf(bbFile_.get(), "T");
                  first = false;
                }
              fprintf(bbFile_.get(), ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 ":%" PRIu64 " ", kv.first,
		      stat.count_, stat.access_, stat.hit_);
            }
        }
      if (not first)
        fprintf(bbFile_.get(), "\n");
    }
  bbInsts_ = 0;

  // Clear basic block stats.
  for (auto& kv : basicBlocks_)
    {
      auto& stat = kv.second;
      stat.count_ = 0;
      stat.access_ = 0;
      stat.hit_ = 0;
    }
}


template <typename URV>
void
Hart<URV>::countBasicBlocks(bool isBranch, uint64_t physPc)
{
  if (not traceOn_)
    return;

  if (bbInsts_ >= bbLimit_)
    dumpBasicBlocks();

  bbInsts_++;

  if (isBranch)
    {
      auto& blockStat = basicBlocks_[physPc];
      blockStat.count_++;
      bbPc_ = physPc;
    }
  else
    {
      auto iter = basicBlocks_.find(physPc);
      if (iter != basicBlocks_.end())
	{
	  iter->second.count_++;
	  bbPc_ = physPc;
	}
      else
	basicBlocks_[bbPc_].count_++;
    }
}


template <typename URV>
bool
Hart<URV>::simpleRunWithLimit()
{
  std::string instStr;

  bool traceBranchOn = branchBuffer_.max_size() and not branchTraceFile_.empty();

  uint64_t& effectiveInstCounter = hasRoiTraceEnabled()? traceCount_ : execCount_;

  const uint64_t instLim = instCountLim_;
  const uint64_t retInstLim = retCountLim_;

  while (noUserStop and effectiveInstCounter < instLim and retireCount_ < retInstLim)
    {
      tickTime();

      resetExecInfo();

      bool traceWasOn = traceOn_;

      currPc_ = pc_;
      ++execCount_;

      if (mcycleEnabled())
	++cycleCount_;

      if ((effectiveMie_ or
          (privMode_ != PrivilegeMode::Machine and effectiveSie_) or
          (virtMode_ and (effectiveVsie_ or hasHvi())))
            and processExternalInterrupt(nullptr, instStr))
        continue;  // Next instruction in trap handler.

      // Fetch/decode unless match in decode cache.
      uint32_t inst = 0;
      uint64_t physPc = 0;
      if (not fetchInst(pc_, physPc, inst))
        continue;
      uint32_t ix = (physPc >> 1) & decodeCacheMask_;
      DecodedInst* di = &decodeCache_[ix];
      if (not di->isValid() or di->physAddress() != physPc or di->inst() != inst)
	decode(pc_, physPc, inst, *di);
      di->resetAddr(pc_);

      pc_ += di->instSize();
      execute(di);

      if (not hasException_)
        {
          if (minstretEnabled())
            ++minstret_;
          ++retireCount_;
        }

      if (instrLineTrace_)
	memory_.traceInstructionLine(currPc_, physPc);

      if (bbFile_)
	{
	  countBasicBlocks(bbPrevIsBranch_, physPc);
	  bbPrevIsBranch_ = di->isBranch();
	}

      if (traceBranchOn and (di->isBranch() or di->isXRet()))
	traceBranch(di);

      if (traceOn_) // and lastPriv_ == PrivilegeMode::User)
        {
          traceCount_++;
          if (not traceWasOn)
            throw CoreException(CoreException::RoiEntry, "Taking snapshot on ROI entry.");
        }
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::simpleRunNoLimit()
{
  while (noUserStop)
    {
      tickTime();

      currPc_ = pc_;
      ++execCount_;

      // Fetch/decode unless match in decode cache.
      uint32_t ix = (pc_ >> 1) & decodeCacheMask_;
      DecodedInst* di = &decodeCache_[ix];
      if (not di->isValid() or di->address() != pc_)
        {
          uint32_t inst = 0;
	  uint64_t physPc = 0;
          if (not fetchInst(pc_, physPc, inst))
            continue;
          decode(pc_, physPc, inst, *di);
        }

      pc_ += di->instSize();
      execute(di);

      ++retireCount_;
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::saveBranchTrace(const std::string& path, bool compress)
{
  util::file::SharedFile file;
  auto zpath = path + ".zst";

  if (compress)
    {
      std::string cmd = "zstd -15 -q -f -o ";
      cmd += zpath;
      file = util::file::make_shared_file(popen(cmd.c_str(), "w"),
                                          util::file::FileCloseF::PCLOSE);
      if (not file)
        std::cerr << "Warning: Failed to run zstd for branch trace " << zpath << "\n";
    }

  if (not file)
    {
      file = util::file::make_shared_file(fopen(path.c_str(), "w"));
      if (not file)
        {
          std::cerr << "Error: Failed to open branch-trace output file '" << path << "' for writing\n";
          return false;
        }
    }

  for (auto iter = branchBuffer_.begin(); iter != branchBuffer_.end(); ++iter)
    {
      auto& rec = *iter;
      if (rec.type_ != 0)
	fprintf(file.get(), "%c 0x%jx 0x%jx %d\n", rec.type_, uintmax_t(rec.pc_),
		uintmax_t(rec.nextPc_), rec.size_);
    }
  return true;
}


template <typename URV>
bool
Hart<URV>::loadBranchTrace(const std::string& path, bool compress)
{
  if (not branchBuffer_.max_size())
    return true;

  util::file::SharedFile file;

  if (compress)
    {
      auto zpath = path;
      if (not zpath.ends_with(".zst"))
        zpath = path + ".zst";

      namespace FS = std::filesystem;
      if (FS::exists(FS::path{zpath}))
        {
          std::string cmd = "zstd -q -dc " + zpath;
          file = util::file::make_shared_file(popen(cmd.c_str(), "r"),
                                              util::file::FileCloseF::PCLOSE);
          if (not file)
            std::cerr << "Warning: Failed to open compressed branch trace file " << zpath << " for input.\n";
        }
    }

  if (not file)
    {
      auto filePath = path;
      if (filePath.ends_with(".zst"))
        filePath = filePath.substr(0, filePath.size() - 4);

      file = util::file::make_shared_file(fopen(filePath.c_str(), "r"),
                                          util::file::FileCloseF::FCLOSE);
      if (not file)
        {
          std::cerr << "Error: Failed to open branch trace file " << filePath << " for input.\n";
          return false;
        }
    }

  branchBuffer_.clear();
  char* buf = nullptr;
  size_t bufSize = 0;
  while (getline(&buf, &bufSize, file.get()) != -1)
    {
      std::string line(buf);
      if (not line.empty() and line.back() == '\n')
        line.pop_back();

      std::vector<std::string> tokens;
      boost::split(tokens, line, boost::is_any_of("\t "), boost::token_compress_on);

      if (tokens.size() != 4)
        {
          std::cerr << "Error: Failed to load branch record from line.\n";
          return false;
        }

      char type = tokens.at(0).at(0);
      uint64_t pc = strtoull(tokens.at(1).c_str(), nullptr, 0);
      uint64_t nextPc = strtoull(tokens.at(2).c_str(), nullptr, 0);
      uint8_t size = strtoull(tokens.at(3).c_str(), nullptr, 0);

      branchBuffer_.push_back(BranchRecord(type, pc, nextPc, size));
    }

  if (branchBuffer_.empty())
    std::cerr << "Warning: No branch records loaded from " << path << "\n";

  return true;
}


template <typename URV>
void
Hart<URV>::traceBranch(const DecodedInst* di)
{
  bool hasTrap = hasInterrupt_ or hasException_;
  if (hasTrap)
    {
      char type = 'x';
      if (branchBuffer_.max_size())
        branchBuffer_.push_back(BranchRecord(type, currPc_, pc_, 0));
      return;
    }

  assert(di != nullptr);

  char type = lastBranchTaken_ ? 't' : 'n';  // For conditional branch.
  if (not di->isConditionalBranch())
    {
      bool indirect = di->isBranchToRegister();
      if (di->op0() == 1 or di->op0() == 5)
	type = indirect ? 'k' : 'c';  // call
      else if (di->operandCount() >= 2 and (di->op1() == 1 or di->op1() == 5))
	type = 'r';  // return
      else
	type = indirect? 'i' : 'j';    // indirect-jump or jump.
    }

  if (di->isXRet())
    type = 'e';

  if (branchBuffer_.max_size())
    branchBuffer_.push_back(BranchRecord(type, currPc_, pc_, di->instSize()));
}


template <typename URV>
bool
Hart<URV>::saveCacheTrace(const std::string &path, bool compress) {
  
  util::file::SharedFile file;
  const std::string outPath = path + ".zst";

  if (compress) {
    std::string cmd = "zstd -15 -q -f -o ";
    cmd += outPath;
    file = util::file::make_shared_file(popen(cmd.c_str(), "w"),
                                        util::file::FileCloseF::PCLOSE);
    if (not file) {
      std::cerr << "Warning: Failed to run zstd for cache trace " << outPath << "\n";
    }
  }
  
  if (not file){
    file = util::file::make_shared_file(fopen(path.c_str(), "w"),
                                          util::file::FileCloseF::FCLOSE);

    if (not file) {
      std::cerr << "Error: Failed to open cache-trace output file " << path
                << " for writing\n";
      return false;
    }
  }

  for (auto iter = cacheBuffer_.begin(); iter != cacheBuffer_.end(); ++iter) {
    auto &rec = *iter;
    if (rec.type_ != 0)
      fprintf(file.get(), "%c 0x%jx 0x%jx 0x%jx\n", rec.type_,
              uintmax_t(rec.vlineNum_), uintmax_t(rec.plineNum_),
              uintmax_t(rec.count_));
  }

  file.reset();
  return true;
}


template <typename URV>
bool
Hart<URV>::loadCacheTrace(const std::string& path, bool compress)
{
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
  if (not cacheBuffer_.max_size())
    return true;

  util::file::SharedFile file;
  std::string filePath = path;

  if (compress)
    {
      if (not filePath.ends_with(".zst"))
        filePath = filePath + ".zst";

      namespace FS = std::filesystem;
      if (FS::exists(FS::path{filePath}))
        {
          std::string cmd = "zstd -q -dc " + filePath;
          file = util::file::make_shared_file(popen(cmd.c_str(), "r"),
                                              util::file::FileCloseF::PCLOSE);
          if (not file)
            std::cerr << "Warning: Failed to open compressed cache trace file " << filePath << " for input.\n";
        }
    } 
  
  if (not file)
    {
      if (filePath.ends_with(".zst"))
        filePath = filePath.substr(0, filePath.size() - 4);

      file = util::file::make_shared_file(fopen(filePath.c_str(), "r"),
                                          util::file::FileCloseF::FCLOSE);
      if (not file)
        {
          std::cerr << "Error: Failed to open cache trace file " << filePath << " for input.\n";
          return false;
        }
    }
  
  cacheBuffer_.clear();
  char* buf = nullptr;
  std::unique_ptr<char, decltype(std::free)*> autoFree(buf, std::free);
  size_t bufSize = 0;
  while (getline(&buf, &bufSize, file.get()) != -1)
    {
      std::string line(buf);
      if (not line.empty() and line.back() == '\n')
        line.pop_back();

      std::vector<std::string> tokens;
      // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
      boost::split(tokens, line, boost::is_any_of("\t "), boost::token_compress_on);

      if (tokens.size() != 4)
        {
          std::cerr << "Error: Failed to load cache record from line.\n";
          return false;
        }

      char type = tokens.at(0).at(0);
      uint64_t va = strtoull(tokens.at(1).c_str(), nullptr, 0);
      uint64_t pa = strtoull(tokens.at(2).c_str(), nullptr, 0);
      uint64_t count = strtoull(tokens.at(3).c_str(), nullptr, 0);

      cacheBuffer_.push_back(CacheRecord(type, va, pa, count));
    }

  if (cacheBuffer_.empty())
    std::cerr << "Warning: No cache records loaded from " << filePath << "\n";

  file.reset();
  return true;
}


template <typename URV>
void
Hart<URV>::traceCache(uint64_t va, uint64_t pa1, uint64_t pa2, bool r, bool w, bool x, bool fencei, bool inval)
{
  assert(unsigned(r) + unsigned(w) + unsigned(x) + unsigned(fencei) + unsigned(inval) == 1);

  char type = r? 'r' : w? 'w' : x? 'x' : fencei? 'e' : 'v';

  uint64_t lineNum1 = cacheLineNum(pa1);
  uint64_t lineNum2 = cacheLineNum(pa2);

  // We only want fence.i and cbo.inval to show up once.
  bool line1Cache = true;
  bool line2Cache = false;
  if (r or w or x)
    {
      // We only include cacheable lines.
      Pma pma = pmaMgr_.getPma(pa1);
      pma = overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt());
      line1Cache = pma.isCacheable();

      pma = pmaMgr_.getPma(pa2);
      pma = overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt());
      line2Cache = pma.isCacheable() and (lineNum1 != lineNum2);
    }

  CacheRecord* last = x? lastCacheFetch_ : lastCacheData_;
  if (fencei or inval)
    last = nullptr;

  if (last)
    {
      if ((line1Cache and lineNum1 == last->plineNum_) or
          (line2Cache and lineNum2 == last->plineNum_))
        {
          last->vlineNum_ = cacheLineNum(va);
          last->count_ = execCount_;
          // change r to w.
          if (w)
            last->type_ = 'w';
        }
    }

  bool updateLast = false;
  if ((not last or lineNum1 != last->plineNum_) and line1Cache)
    {
      cacheBuffer_.push_back(CacheRecord(type, cacheLineNum(va), lineNum1, execCount_));
      updateLast = true;
    }

  if ((not last or lineNum2 != last->plineNum_) and line2Cache)
    {
      cacheBuffer_.push_back(CacheRecord(type, cacheLineNum(va), lineNum2, execCount_));
      updateLast = true;
    }

  if (updateLast and (r or w or x))
    {
      if (r or w)
        lastCacheData_ = &cacheBuffer_.back();
      if (x)
        lastCacheFetch_ = &cacheBuffer_.back();
      return;
    }

  if (inval)
    {
      assert(lineNum1 == lineNum2);
      if (lineNum1 == lastCacheData_->plineNum_)
        lastCacheData_ = nullptr;
      return;
    }

  if (fencei)
    {
      lastCacheFetch_ = nullptr;
      return;
    }
}


template <typename URV>
bool
Hart<URV>::openTcpForGdb()
{
  struct sockaddr_in address{};
  socklen_t addrlen = sizeof(address);

  memset(&address, 0, addrlen);

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons( gdbTcpPort_ );

  int gdbFd = socket(AF_INET, SOCK_STREAM, 0);
  if (gdbFd < 0)
    {
      std::cerr << "Error: Failed to create gdb socket at port " << gdbTcpPort_ << '\n';
      return false;
    }

#ifndef __APPLE__
  int opt = 1;
  if (setsockopt(gdbFd, SOL_SOCKET,
		 SO_REUSEADDR | SO_REUSEPORT, &opt,
		 sizeof(opt)) != 0)
    {
      std::cerr << "Error: Failed to set socket option for gdb socket\n";
      return false;
    }
#endif

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  if (bind(gdbFd, reinterpret_cast<sockaddr*>(&address), addrlen) < 0)
    {
      std::cerr << "Error: Failed to bind gdb socket\n";
      return false;
    }

  if (listen(gdbFd, 3) < 0)
    {
      std::cerr << "Error: Failed to listen to gdb socket\n";
      return false;
    }

  signal(SIGPIPE, SIG_IGN);  // Don't die when writing to a disconnected client.

  gdbServerFd_ = gdbFd;  // Keep server socket open so we can re-accept later.

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
  gdbInputFd_ = accept(gdbFd, (sockaddr*) &address, &addrlen);
  if (gdbInputFd_ < 0)
    {
      std::cerr << "Error: Failed to accept from gdb socket\n";
      return false;
    }

  return true;
}


template <typename URV>
int
Hart<URV>::acceptGdbConnection()
{
  if (gdbServerFd_ < 0)
    return -1;

  if (gdbInputFd_ >= 0)
    {
      close(gdbInputFd_);
      gdbInputFd_ = -1;
    }

  struct sockaddr_in address{};
  socklen_t addrlen = sizeof(address);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
  gdbInputFd_ = accept(gdbServerFd_, (sockaddr*) &address, &addrlen);
  if (gdbInputFd_ < 0)
    {
      std::cerr << "Error: Failed to accept new gdb connection\n";
      return -1;
    }
  return gdbInputFd_;
}


//NOLINTNEXTLINE(bugprone-reserved-identifier, cppcoreguidelines-avoid-non-const-global-variables)
extern void (*__tracerExtension)(void*);


/// Run indefinitely.  If the tohost address is defined, then run till
/// a write is attempted to that address.
template <typename URV>
bool
Hart<URV>::run(FILE* file)
{
  if (gdbTcpPort_ >= 0)
    openTcpForGdb();
  else if (enableGdb_)
    gdbInputFd_ = STDIN_FILENO;

  // To run fast, this method does not do much besides
  // straight-forward execution. If any option is turned on, we switch
  // to runUntilAddress which supports all features.
  URV stopAddr = stopAddrValid_? stopAddr_ : ~URV(0); // ~URV(0): No-stop PC.
  bool complex = (stopAddrValid_ or instFreq_ or sdtrigOn_ or enableGdb_
                  or enableCounters_ or alarmInterval_ or file
		  or __tracerExtension or initStateFile_);
  if (complex)
    return runUntilAddress(stopAddr, file);

  const uint64_t counter0 = execCount_;
  const uint64_t counter1 = retireCount_;

  struct timeval t0{};
  gettimeofday(&t0, nullptr);

  // Setup signal handlers. Restore on destruction.
  SignalHandlers handlers;

  bool success = simpleRun();

  // Simulator stats.
  struct timeval t1{};
  gettimeofday(&t1, nullptr);
  double elapsed = (double(t1.tv_sec - t0.tv_sec) +
		    double(t1.tv_usec - t0.tv_usec)*1e-6);

  uint64_t numInsts = execCount_ - counter0;
  uint64_t numRetInsts = retireCount_ - counter1;

  reportInstsPerSec(numInsts, numRetInsts, elapsed, userStop);
  return success;
}


template <typename URV>
void
Hart<URV>::setMcm(std::shared_ptr<Mcm<URV>> mcm, std::shared_ptr<TT_CACHE::Cache> fetchCache,
                  std::shared_ptr<TT_CACHE::Cache> dataCache)
{
  mcm_ = std::move(mcm);
  ooo_ = mcm_ != nullptr or perfApi_ != nullptr;
  fetchCache_ = std::move(fetchCache);
  dataCache_ = std::move(dataCache);
}


template <typename URV>
void
Hart<URV>::setPerfApi(std::shared_ptr<TT_PERF::PerfApi<URV>> perfApi)
{
  perfApi_ = std::move(perfApi);
  ooo_ = mcm_ != nullptr or perfApi_ != nullptr;
}


template <typename URV>
bool
Hart<URV>::isInterruptPossible(URV mip, URV sip, [[maybe_unused]] URV vsip,
                               InterruptCause& cause, PrivilegeMode& nextMode,
                               bool& nextVirt, bool& hvi) const
{
  if (debugMode_)
    return false;

  // If in a non-maskable interrupt handler, then all interrupts disabled.
  if (extensionIsEnabled(RvExtension::Smrnmi) and
      (MnstatusFields{csRegs_.peekMnstatus()}.bits_.NMIE) == 0)
    return false;

  using PM = PrivilegeMode;

  nextVirt = false;         // Next virtual mode if interrupt is possible.
  nextMode = PM::Machine;   // Next privilege mode if interrupt is possible.

  // Non-delegated interrupts are destined for machine mode.
  URV mdest = mip & effectiveMie_;  // Interrupts destined for machine mode.
  if ((mstatus_.bits_.MIE or privMode_ != PM::Machine) and mdest != 0)
    {
      // Check for interrupts destined for machine-mode (not-delegated).
      for (InterruptCause ic : mInterrupts_)
        {
          URV mask = URV(1) << unsigned(ic);
          if ((mdest & mask) != 0)
            {
              cause = ic;
              return true;
            }
        }
    }
  if (privMode_ == PM::Machine)
    return false;   // Interrupts destined for lower privileges are disabled.

  nextMode = PM::Supervisor;

  // Delegated but non-h-delegated interrupts are destined for supervisor mode (S/HS).
  URV sdest = sip & effectiveSie_;
  if ((mstatus_.bits_.SIE or virtMode_ or privMode_ == PM::User) and sdest != 0)
    {
      for (InterruptCause ic : sInterrupts_)
        {
          URV mask = URV(1) << unsigned(ic);
          if ((sdest & mask) != 0)
            {
              cause = ic;
              return true;
            }
        }
    }

  // We now check for interrupts destined for VS mode. These are disabled if running in
  // M/HS/U modes. If mode is M/HS then VS-destined are disabled because of VS is a lower
  // privilege. If mode is U then VS-destined are explicilty disabled (see section 9.1 of
  // privileged spec).
  if (not virtMode_)
    return false;

  // Check for interrupts destined to VS privilege. Possible if pending (mip), enabled
  // (mie), delegated, and h-delegated.
  bool vsEnabled = vsstatus_.bits_.SIE  or  (virtMode_ and privMode_ == PM::User);
  if (not vsEnabled)
    return false;

  nextVirt = true;

  auto hvictl = csRegs_.getImplementedCsr(CsrNumber::HVICTL);
  if (not isRvaia() or not hvictl)
    {
      URV vsdest = vsip & effectiveVsie_;
      if (vsdest)
        {
          // Only VS interrupts can be delegated in HIDELEG.
          for (InterruptCause ic : vsInterrupts_)
            {
              URV mask = URV(1) << unsigned(ic);
              if ((vsdest & mask) != 0)
                {
                  cause = ic;
                  return true;
                }
            }
        }
    }
  else
    {
      URV vstopi;
      if (csRegs_.readTopi(CsrNumber::VSTOPI, vstopi, false, hvi))
        {
          if (vstopi)
            {
              unsigned iid = vstopi >> 16;  // Interrupt id.
              if (deferredInterrupts_ & (URV(1) << (iid+1))) // Deferred interrupts are on MIP where VS interrupts are r-shifted by 1.
                return false;
              cause = static_cast<InterruptCause>(iid);
              return true;
            }
        }
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::isInterruptPossible(InterruptCause& cause, PrivilegeMode& nextMode, bool& nextVirt, bool& hvi) const
{
  // MIP read value is ored with supervisor external interrupt pin and
  // mvip if mvien is not set.
  URV mip = csRegs_.effectiveMip();

  // SIP read value will alias mvip if not delegated and mvien is set.
  URV sip = csRegs_.effectiveSip();

  // VSIP read value may alias hvip (for bits 13-63). These bits don't alias
  // HIP/HIE and are delgated through hvien.
  URV vsip = csRegs_.effectiveVsip();

  mip &= ~deferredInterrupts_;  // Inhibited by test-bench.
  sip &= ~deferredInterrupts_;
  vsip &= ~(deferredInterrupts_ >> 1);

  if (not (mip & effectiveMie_) and
      not (sip & effectiveSie_) and
      not (vsip & effectiveVsie_) and
      not hasHvi())
    return false;

  return isInterruptPossible(mip, sip, vsip, cause, nextMode, nextVirt, hvi);
}


template <typename URV>
bool
Hart<URV>::processNmi(FILE* traceFile, std::string& instStr)
{
  if (not nmiPending_)
    return false;

  if (pendingNmis_.empty())
    return false;  // Should not happen.

  for (auto nmi : nmInterrupts_)   // Potential NMIs in high to low priority order
    {
      auto iter = pendingNmis_.find(nmi);
      if (iter == pendingNmis_.end())
        continue;  // NMI is not pending.

      if (isDeferredNmi(nmi))
        continue;

      if (initiateNmi(URV(nmi), pc_))
        {
          uint32_t inst = 0; // Load interrupted inst.
          readInst(currPc_, inst);
          printInstTrace(inst, execCount_, instStr, traceFile);
          if (mcycleEnabled())
            ++cycleCount_;
          return true;
        }
      break;  // NMI could not be delivered (NMIs not enabled).
    }

  // Process pending NMIs not in the priority list.
  for (auto nmi : pendingNmis_)
    {
      if (isDeferredNmi(nmi))
        continue;
      if (initiateNmi(URV(nmi), pc_))
        {
          uint32_t inst = 0; // Load interrupted inst.
          readInst(currPc_, inst);
          printInstTrace(inst, execCount_, instStr, traceFile);
          if (mcycleEnabled())
            ++cycleCount_;
          return true;
        }
      return false;  // NMI could not be delivered (NMIs not enabled).
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::processExternalInterrupt(FILE* traceFile, std::string& instStr)
{
  // If mip poked exernally we avoid over-writing it for 1 instruction.
  if (not mipPoked_)
    processTimerInterrupt();
  mipPoked_ = false;

  if (inDebugParkLoop_)
    return false;

  if (dcsrStep_ and not dcsrStepIe_)
    return false;

  if (nmiPending_ and processNmi(traceFile, instStr))
    return true;  // NMI was delivered.

  // If interrupts enabled and one is pending, take it.
  InterruptCause cause{};
  auto nextMode = PrivilegeMode::Machine;
  bool nextVirt = false;
  bool hvi = false;
  if (isInterruptPossible(cause, nextMode, nextVirt, hvi))
    {
      // Attach changes to interrupted instruction.
      uint32_t inst = 0; // Load interrupted inst.
      uint64_t pc = pc_, physPc = 0;
      readInst(pc, physPc, inst);
      if (inst)
	{
#if 0
	  // Enable when RTL is ready.
	  DecodedInst di;
	  decode(pc, physPc, inst, di);
	  if (di.instId() == InstId::wfi)
	    pc = pc + 4;
#endif
	}
      initiateInterrupt(cause, nextMode, nextVirt, pc, hvi);
      printInstTrace(inst, execCount_, instStr, traceFile);
      if (mcycleEnabled())
	++cycleCount_;
      return true;
    }
  return false;
}


template <typename URV>
void
Hart<URV>::processTimerInterrupt()
{
  using IC = InterruptCause;

  // Fast path: before the next threshold and with no timer input changed, no MIP timer
  // bit can flip, so the recompute below is a no-op; skip it. The VS-timer is excluded
  // from the deadline (wraparound safety), so disable the fast path when it is active.
  if (not timerStateStale_ and not vstimecmpActive_ and time_ < nextTimerDeadline_)
    return;

  URV mipVal = csRegs_.overrideWithMvip(csRegs_.peekMip());
  URV prev = mipVal;

  if (mtipEnabled_)
    {
      if (hasAclint() and aclintDeliverInterrupts_)
        {
          // Deliver/clear machine timer interrupt from clint.
          if (time_ >= aclintAlarm_)
            mipVal = mipVal | (URV(1) << URV(IC::M_TIMER));
          else
            mipVal = mipVal & ~(URV(1) << URV(IC::M_TIMER));
        }
      else
        {
          // Deliver/clear machine timer interrupt from periodic alarm.
          bool hasAlarm = alarmLimit_ != ~uint64_t(0);
          if (hasAlarm)
            {
              if (time_ >= alarmLimit_)
                {
                  alarmLimit_ += alarmInterval_;
                  mipVal = mipVal | (URV(1) << URV(IC::M_TIMER));
                }
              else
                mipVal = mipVal & ~(URV(1) << URV(IC::M_TIMER));
            }
        }
    }

  if (swInterrupt_.bits_.alarm_ and aclintDeliverInterrupts_)
    {
      if (swInterrupt_.bits_.flag_)
        {
          // Only deliver when 1 is written. Deliver and clear to
          // follow doorbell model.
          mipVal = mipVal | (URV(1) << URV(InterruptCause::M_SOFTWARE));
          setSwInterrupt(0);
        }
      else
        mipVal = mipVal & ~(URV(1) << URV(InterruptCause::M_SOFTWARE));
    }

  // Deliver/clear supervisor timer from stimecmp CSR.
  if (stimecmpActive_)
    {
      if (time_ >= stimecmp_)
        mipVal = mipVal | (URV(1) << URV(IC::S_TIMER));
      else
        mipVal = mipVal & ~(URV(1) << URV(IC::S_TIMER));
    }

  // Deliver/clear virtual supervisor timer from vstimecmp CSR.
  URV vstipMask = URV(1) << URV(IC::VS_TIMER);
  if (vstimecmpActive_)
    {
      if ((time_ + htimedelta_) >= vstimecmp_)
        mipVal = mipVal | vstipMask;
      else
        {
          // Bits HIP.VSTIP (alias of MIP.VSTIP) is the logical-OR
          // of HVIP.VSTIP and the timer interrupt signal
          // resulting from vstimecmp. Section 9.2.3 of priv sepc.
          mipVal = (mipVal & ~vstipMask) | (csRegs_.peekHvip() & vstipMask);
        }
    }
  else
    mipVal = (mipVal & ~vstipMask) | (csRegs_.peekHvip() & vstipMask);

  if (mipVal != prev)
    csRegs_.poke(CsrNumber::MIP, mipVal);

  // HIP.VSTIP aliases MIP.VSTIP
  auto hip = csRegs_.getImplementedCsr(CsrNumber::HIP);
  if (hip)
    {
      auto hipVal = hip->read();
      if ((mipVal & vstipMask) != (hipVal & vstipMask))
        hip->poke((hip->read() & ~vstipMask) | (mipVal & vstipMask));
    }

  // Recompute the next time_ at which a timer bit can flip off->on; only future
  // thresholds bound the skip window. VS-timer is omitted (its fast path is disabled).
  uint64_t deadline = ~uint64_t(0);
  if (mtipEnabled_)
    {
      if (hasAclint() and aclintDeliverInterrupts_)
        { if (aclintAlarm_ > time_) deadline = std::min(deadline, aclintAlarm_); }
      else if (alarmLimit_ != ~uint64_t(0) and alarmLimit_ > time_)
        deadline = std::min(deadline, alarmLimit_);
    }
  if (stimecmpActive_ and stimecmp_ > time_)
    deadline = std::min(deadline, stimecmp_);
  nextTimerDeadline_ = deadline;
  timerStateStale_ = false;
}


template <typename URV>
void
Hart<URV>::invalidateDecodeCache(URV addr, unsigned storeSize)
{
  // Consider putting this in a callback associated with memory
  // write/poke. This way it can be applied only to pages marked
  // execute.

  // We want to check the location before the address just in case it
  // contains a 4-byte instruction that overlaps what was written.
  storeSize += 3;
  addr -= 3;

  for (unsigned i = 0; i < storeSize; i += 2)
    {
      URV instAddr = (addr + i) >> 1;
      uint32_t cacheIx = instAddr & decodeCacheMask_;
      auto& entry = decodeCache_[cacheIx];
      if ((entry.address() >> 1) == instAddr)
	entry.invalidate();
    }
}


template <typename URV>
void
Hart<URV>::invalidateDecodeCache()
{
  for (auto& entry : decodeCache_)
    entry.invalidate();
}


template <typename URV>
void
Hart<URV>::singleStep(FILE* traceFile)
{
  DecodedInst di;
  singleStep(di, traceFile);
}


template <typename URV>
void
Hart<URV>::singleStep(DecodedInst& di, FILE* traceFile)
{
  traceFileActive_ = (traceFile != nullptr);  // gates the per-instruction trace reset

  std::string instStr;

  // Single step is mostly used for follow-me mode where we want to
  // know the changes after the execution of each instruction.
  bool statsEnabled = instFreq_ or enableCounters_;
  bool roiActive = hasRoiTraceEnabled();

  try
    {
      tickTime();

      uint32_t inst = 0;
      currPc_ = pc_;

      resetExecInfo(); clearTraceData();

      ++execCount_;
      if (mcycleEnabled())
	++cycleCount_;

      if (hasActiveTrigger() and icountTriggerFired() and breakpOrEnterDebugTripped())
        {
          icountTrig_ = true;
          takeTriggerAction(traceFile, currPc_, 0, execCount_, nullptr /*di*/);
          evaluateDebugStep();
          injectException_ = ExceptionCause::NONE;
          icountTrig_ = false;
          return;
        }

      if (processExternalInterrupt(traceFile, instStr))
        {
          if (sdtrigOn_)
            {
              if (hasActiveTrigger())
                evaluateIcountTrigger();
              evaluateDebugStep();
            }
          injectException_ = ExceptionCause::NONE;
          return;  // Next instruction in interrupt handler.
        }

      uint64_t physPc = 0;
      if (not fetchInstWithTrigger(pc_, physPc, inst, traceFile))
        {
          if (sdtrigOn_)
            {
              if (hasActiveTrigger())
                evaluateIcountTrigger();
              evaluateDebugStep();
            }
          injectException_ = ExceptionCause::NONE;
          return;
        }

      decode(pc_, physPc, inst, di);

      auto debugStopCount = isDebugModeStopCount(*this);

      // Increment pc and execute instruction
      pc_ += di.instSize();
      execute(&di);
      injectException_ = ExceptionCause::NONE;

      if (hasActiveTrigger())
        evaluateIcountTrigger();

      bool doStats = statsEnabled and (not roiActive or traceOn_);

      if (lastInstructionTrapped())
	{
	  if (doStats)
	    accumulateInstructionStats(di);
	  printDecodedInstTrace(di, execCount_, instStr, traceFile);
          evaluateDebugStep();
	  return;
	}

      if (breakpOrEnterDebugTripped())
	{
          URV tval = ldStFaultAddr_;
	  takeTriggerAction(traceFile, currPc_, tval, execCount_, &di);
          evaluateDebugStep();
	  return;
	}

      if (minstretEnabled() and not ebreakInstDebug_ and not debugStopCount)
        ++minstret_;

      if (doStats)
	accumulateInstructionStats(di);
      printInstTrace(inst, execCount_, instStr, traceFile);

      if (sdtrigOn_)
        evaluateDebugStep();

      prevPerfControl_ = perfControl_;
    }
  catch (const CoreException& ce)
    {
      evaluateDebugStep();

      stepResult_ = logStop(ce, execCount_, traceFile);
      if (ce.type() == CoreException::Snapshot or
          ce.type() == CoreException::RoiEntry or
          ce.type() == CoreException::SnapshotAndStop)
        throw;
    }
}


template <typename URV>
inline
void
Hart<URV>::execLui(const DecodedInst* di)
{
  intRegs_.write(di->op0(), SRV(int32_t(di->op1())));
}


// NOLINTBEGIN(readability-function-size)
template <typename URV>
void
Hart<URV>::execute(const DecodedInst* di)
{
  const InstEntry* entry = di->instEntry();
  hyperLs_ = false;

  if (isRvZicfilp() and elp_)
    {
      execLpad(di);
      return;
    }

  switch (entry->instId())
    {
    case InstId::illegal:
      illegalInst(di);
      return;

    case InstId::lui:
      execLui(di);
      return;

    case InstId::auipc:
      execAuipc(di);
      return;

    case InstId::jal:
      execJal(di);
      return;

    case InstId::jalr:
      execJalr(di);
      return;

    case InstId::beq:
      execBeq(di);
      return;

    case InstId::bne:
      execBne(di);
      return;

    case InstId::beqi:
      execBeqi(di);
      return;

    case InstId::bnei:
      execBnei(di);
      return;

    case InstId::blt:
      execBlt(di);
      return;

    case InstId::bge:
      execBge(di);
      return;

    case InstId::bltu:
      execBltu(di);
      return;

    case InstId::bgeu:
      execBgeu(di);
      return;

    case InstId::lb:
      execLb(di);
      return;

    case InstId::lb_aq:
      execLb_aq(di);
      return;

    case InstId::lh_aq:
      execLh_aq(di);
      return;

    case InstId::lw_aq:
      execLw_aq(di);
      return;

    case InstId::ld_aq:
      execLd_aq(di);
      return;

    case InstId::sb_rl:
      execSb_rl(di);
      return;

    case InstId::sh_rl:
      execSh_rl(di);
      return;

    case InstId::sw_rl:
      execSw_rl(di);
      return;

    case InstId::sd_rl:
      execSd_rl(di);
      return;

    case InstId::lh:
      execLh(di);
      return;

    case InstId::lw:
      execLw(di);
      return;

    case InstId::lbu:
      execLbu(di);
      return;

    case InstId::lhu:
      execLhu(di);
      return;

    case InstId::sb:
      execSb(di);
      return;

    case InstId::sh:
      execSh(di);
      return;

    case InstId::sw:
      execSw(di);
      return;

    case InstId::addi:
      execAddi(di);
      return;

    case InstId::slti:
      execSlti(di);
      return;

    case InstId::sltiu:
      execSltiu(di);
      return;

    case InstId::xori:
      execXori(di);
      return;

    case InstId::ori:
      execOri(di);
      return;

    case InstId::andi:
      execAndi(di);
      return;

    case InstId::slli:
      execSlli(di);
      return;

    case InstId::srli:
      execSrli(di);
      return;

    case InstId::srai:
      execSrai(di);
      return;

    case InstId::add:
      execAdd(di);
      return;

    case InstId::sub:
      execSub(di);
      return;

    case InstId::sll:
      execSll(di);
      return;

    case InstId::slt:
      execSlt(di);
      return;

    case InstId::sltu:
      execSltu(di);
      return;

    case InstId::xor_:
      execXor(di);
      return;

    case InstId::srl:
      execSrl(di);
      return;

    case InstId::sra:
      execSra(di);
      return;

    case InstId::or_:
      execOr(di);
      return;

    case InstId::and_:
      execAnd(di);
      return;

    case InstId::fence:
    case InstId::pause:
      execFence(di);
      return;

    case InstId::fence_tso:
      execFence_tso(di);
      return;

    case InstId::fence_i:
      execFencei(di);
      return;

    case InstId::ecall:
      execEcall(di);
      return;

    case InstId::ebreak:
      execEbreak(di);
      return;

    case InstId::csrrw:
      execCsrrw(di);
      return;

    case InstId::csrrs:
      execCsrrs(di);
      return;

    case InstId::csrrc:
      execCsrrc(di);
      return;

    case InstId::csrrwi:
      execCsrrwi(di);
      return;

    case InstId::csrrsi:
      execCsrrsi(di);
      return;

    case InstId::csrrci:
      execCsrrci(di);
      return;

    case InstId::lwu:
      execLwu(di);
      return;

    case InstId::ld:
      execLd(di);
      return;

    case InstId::sd:
      execSd(di);
      return;

    case InstId::addiw:
      execAddiw(di);
      return;

    case InstId::slliw:
      execSlliw(di);
      return;

    case InstId::srliw:
      execSrliw(di);
      return;

    case InstId::sraiw:
      execSraiw(di);
      return;

    case InstId::addw:
      execAddw(di);
      return;

    case InstId::subw:
      execSubw(di);
      return;

    case InstId::sllw:
      execSllw(di);
      return;

    case InstId::srlw:
      execSrlw(di);
      return;

    case InstId::sraw:
      execSraw(di);
      return;

    case InstId::mul:
      execMul(di);
      return;

    case InstId::mulh:
      execMulh(di);
      return;

    case InstId::mulhsu:
      execMulhsu(di);
      return;

    case InstId::mulhu:
      execMulhu(di);
      return;

    case InstId::div:
      execDiv(di);
      return;

    case InstId::divu:
      execDivu(di);
      return;

    case InstId::rem:
      execRem(di);
      return;

    case InstId::remu:
      execRemu(di);
      return;

    case InstId::mulw:
      execMulw(di);
      return;

    case InstId::divw:
      execDivw(di);
      return;

    case InstId::divuw:
      execDivuw(di);
      return;

    case InstId::remw:
      execRemw(di);
      return;

    case InstId::remuw:
      execRemuw(di);
      return;

    case InstId::lr_w:
      execLr_w(di);
      return;

    case InstId::sc_w:
      execSc_w(di);
      return;

    case InstId::amoswap_w:
      execAmoswap_w(di);
      return;

    case InstId::amoadd_w:
      execAmoadd_w(di);
      return;

    case InstId::amoxor_w:
      execAmoxor_w(di);
      return;

    case InstId::amoand_w:
      execAmoand_w(di);
      return;

    case InstId::amoor_w:
      execAmoor_w(di);
      return;

    case InstId::amomin_w:
      execAmomin_w(di);
      return;

    case InstId::amomax_w:
      execAmomax_w(di);
      return;

    case InstId::amominu_w:
      execAmominu_w(di);
      return;

    case InstId::amomaxu_w:
      execAmomaxu_w(di);
      return;

    case InstId::amoswap_b:
      execAmoswap_b(di);
      return;
    case InstId::amoadd_b:
      execAmoadd_b(di);
      return;
    case InstId::amoxor_b:
      execAmoxor_b(di);
      return;
    case InstId::amoand_b:
      execAmoand_b(di);
      return;
    case InstId::amoor_b:
      execAmoor_b(di);
      return;
    case InstId::amomin_b:
      execAmomin_b(di);
      return;
    case InstId::amomax_b:
      execAmomax_b(di);
      return;
    case InstId::amominu_b:
      execAmominu_b(di);
      return;
    case InstId::amomaxu_b:
      execAmomaxu_b(di);
      return;
    case InstId::amoswap_h:
      execAmoswap_h(di);
      return;
    case InstId::amoadd_h:
      execAmoadd_h(di);
      return;
    case InstId::amoxor_h:
      execAmoxor_h(di);
      return;
    case InstId::amoand_h:
      execAmoand_h(di);
      return;
    case InstId::amoor_h:
      execAmoor_h(di);
      return;
    case InstId::amomin_h:
      execAmomin_h(di);
      return;
    case InstId::amomax_h:
      execAmomax_h(di);
      return;
    case InstId::amominu_h:
      execAmominu_h(di);
      return;
    case InstId::amomaxu_h:
      execAmomaxu_h(di);
      return;
    case InstId::amocas_b:
      execAmocas_b(di);
      return;
    case InstId::amocas_h:
      execAmocas_h(di);
      return;

    case InstId::lr_d:
      execLr_d(di);
      return;

    case InstId::sc_d:
      execSc_d(di);
      return;

    case InstId::amoswap_d:
      execAmoswap_d(di);
      return;

    case InstId::amoadd_d:
      execAmoadd_d(di);
      return;

    case InstId::amoxor_d:
      execAmoxor_d(di);
      return;

    case InstId::amoand_d:
      execAmoand_d(di);
      return;

    case InstId::amoor_d:
      execAmoor_d(di);
      return;

    case InstId::amomin_d:
      execAmomin_d(di);
      return;

    case InstId::amomax_d:
      execAmomax_d(di);
      return;

    case InstId::amominu_d:
      execAmominu_d(di);
      return;

    case InstId::amomaxu_d:
      execAmomaxu_d(di);
      return;

    case InstId::flw:
      execFlw(di);
      return;

    case InstId::fsw:
      execFsw(di);
      return;

    case InstId::fmadd_s:
      execFmadd_s(di);
      return;

    case InstId::fmsub_s:
      execFmsub_s(di);
      return;

    case InstId::fnmsub_s:
      execFnmsub_s(di);
      return;

    case InstId::fnmadd_s:
      execFnmadd_s(di);
      return;

    case InstId::fadd_s:
      execFadd_s(di);
      return;

    case InstId::fsub_s:
      execFsub_s(di);
      return;

    case InstId::fmul_s:
      execFmul_s(di);
      return;

    case InstId::fdiv_s:
      execFdiv_s(di);
      return;

    case InstId::fsqrt_s:
      execFsqrt_s(di);
      return;

    case InstId::fsgnj_s:
      execFsgnj_s(di);
      return;

    case InstId::fsgnjn_s:
      execFsgnjn_s(di);
      return;

    case InstId::fsgnjx_s:
      execFsgnjx_s(di);
      return;

    case InstId::fmin_s:
      execFmin_s(di);
      return;

    case InstId::fmax_s:
      execFmax_s(di);
      return;

    case InstId::fcvt_w_s:
      execFcvt_w_s(di);
      return;

    case InstId::fcvt_wu_s:
      execFcvt_wu_s(di);
      return;

    case InstId::fmv_x_w:
      execFmv_x_w(di);
      return;

    case InstId::feq_s:
      execFeq_s(di);
      return;

    case InstId::flt_s:
      execFlt_s(di);
      return;

    case InstId::fle_s:
      execFle_s(di);
      return;

    case InstId::fclass_s:
      execFclass_s(di);
      return;

    case InstId::fcvt_s_w:
      execFcvt_s_w(di);
      return;

    case InstId::fcvt_s_wu:
      execFcvt_s_wu(di);
      return;

    case InstId::fmv_w_x:
      execFmv_w_x(di);
      return;

    case InstId::fcvt_l_s:
      execFcvt_l_s(di);
      return;

    case InstId::fcvt_lu_s:
      execFcvt_lu_s(di);
      return;

    case InstId::fcvt_s_l:
      execFcvt_s_l(di);
      return;

    case InstId::fcvt_s_lu:
      execFcvt_s_lu(di);
      return;

    case InstId::fld:
      execFld(di);
      return;

    case InstId::fsd:
      execFsd(di);
      return;

    case InstId::fmadd_d:
      execFmadd_d(di);
      return;

    case InstId::fmsub_d:
      execFmsub_d(di);
      return;

    case InstId::fnmsub_d:
      execFnmsub_d(di);
      return;

    case InstId::fnmadd_d:
      execFnmadd_d(di);
      return;

    case InstId::fadd_d:
      execFadd_d(di);
      return;

    case InstId::fsub_d:
      execFsub_d(di);
      return;

    case InstId::fmul_d:
      execFmul_d(di);
      return;

    case InstId::fdiv_d:
      execFdiv_d(di);
      return;

    case InstId::fsqrt_d:
      execFsqrt_d(di);
      return;

    case InstId::fsgnj_d:
      execFsgnj_d(di);
      return;

    case InstId::fsgnjn_d:
      execFsgnjn_d(di);
      return;

    case InstId::fsgnjx_d:
      execFsgnjx_d(di);
      return;

    case InstId::fmin_d:
      execFmin_d(di);
      return;

    case InstId::fmax_d:
      execFmax_d(di);
      return;

    case InstId::fcvt_s_d:
      execFcvt_s_d(di);
      return;

    case InstId::fcvt_d_s:
      execFcvt_d_s(di);
      return;

    case InstId::feq_d:
      execFeq_d(di);
      return;

    case InstId::flt_d:
      execFlt_d(di);
      return;

    case InstId::fle_d:
      execFle_d(di);
      return;

    case InstId::fclass_d:
      execFclass_d(di);
      return;

    case InstId::fcvt_w_d:
      execFcvt_w_d(di);
      return;

    case InstId::fcvt_wu_d:
      execFcvt_wu_d(di);
      return;

    case InstId::fcvt_d_w:
      execFcvt_d_w(di);
      return;

    case InstId::fcvt_d_wu:
      execFcvt_d_wu(di);
      return;

    case InstId::fcvt_l_d:
      execFcvt_l_d(di);
      return;

    case InstId::fcvt_lu_d:
      execFcvt_lu_d(di);
      return;

    case InstId::fmv_x_d:
      execFmv_x_d(di);
      return;

    case InstId::fcvt_d_l:
      execFcvt_d_l(di);
      return;

    case InstId::fcvt_d_lu:
      execFcvt_d_lu(di);
      return;

    case InstId::fmv_d_x:
      execFmv_d_x(di);
      return;

    case InstId::flh:
      execFlh(di);
      return;

    case InstId::fsh:
      execFsh(di);
      return;

    case InstId::fmadd_h:
      execFmadd_h(di);
      return;

    case InstId::fmsub_h:
      execFmsub_h(di);
      return;

    case InstId::fnmsub_h:
      execFnmsub_h(di);
      return;

    case InstId::fnmadd_h:
      execFnmadd_h(di);
      return;

    case InstId::fadd_h:
      execFadd_h(di);
      return;

    case InstId::fsub_h:
      execFsub_h(di);
      return;

    case InstId::fmul_h:
      execFmul_h(di);
      return;

    case InstId::fdiv_h:
      execFdiv_h(di);
      return;

    case InstId::fsqrt_h:
      execFsqrt_h(di);
      return;

    case InstId::fsgnj_h:
      execFsgnj_h(di);
      return;

    case InstId::fsgnjn_h:
      execFsgnjn_h(di);
      return;

    case InstId::fsgnjx_h:
      execFsgnjx_h(di);
      return;

    case InstId::fmin_h:
      execFmin_h(di);
      return;

    case InstId::fmax_h:
      execFmax_h(di);
      return;

    case InstId::fcvt_s_h:
      execFcvt_s_h(di);
      return;

    case InstId::fcvt_d_h:
      execFcvt_d_h(di);
      return;

    case InstId::fcvt_h_s:
      execFcvt_h_s(di);
      return;

    case InstId::fcvt_h_d:
      execFcvt_h_d(di);
      return;

    case InstId::fcvt_w_h:
      execFcvt_w_h(di);
      return;

    case InstId::fcvt_wu_h:
      execFcvt_wu_h(di);
      return;

    case InstId::fmv_x_h:
      execFmv_x_h(di);
      return;

    case InstId::feq_h:
      execFeq_h(di);
      return;

    case InstId::flt_h:
      execFlt_h(di);
      return;

    case InstId::fle_h:
      execFle_h(di);
      return;

    case InstId::fclass_h:
      execFclass_h(di);
      return;

    case InstId::fcvt_h_w:
      execFcvt_h_w(di);
      return;

    case InstId::fcvt_h_wu:
      execFcvt_h_wu(di);
      return;

    case InstId::fmv_h_x:
      execFmv_h_x(di);
      return;

    case InstId::fcvt_l_h:
      execFcvt_l_h(di);
      return;

    case InstId::fcvt_lu_h:
      execFcvt_lu_h(di);
      return;

    case InstId::fcvt_h_l:
      execFcvt_h_l(di);
      return;

    case InstId::fcvt_h_lu:
      execFcvt_h_lu(di);
      return;

    case InstId::fcvt_bf16_s:
      execFcvt_bf16_s(di);
      return;

    case InstId::fcvt_s_bf16:
      execFcvt_s_bf16(di);
      return;

    case InstId::mret:
      execMret(di);
      return;

    case InstId::sret:
      execSret(di);
      return;

    case InstId::mnret:
      execMnret(di);
      return;

    case InstId::wfi:
      execWfi(di);
      return;

    case InstId::dret:
      execDret(di);
      return;

    case InstId::sfence_vma:
      execSfence_vma(di);
      return;

    case InstId::c_addi4spn:
      if (isRvzca()) execAddi(di); else illegalInst(di);
      return;

    case InstId::c_fld:
      if (isRvzca() and isRvzcd()) execFld(di); else illegalInst(di);
      return;

    case InstId::c_lq:
      if (isRvzca()) execLq(di); else illegalInst(di);
      return;

    case InstId::c_lw:
      if (isRvzca()) execLw(di); else illegalInst(di);
      return;

    case InstId::c_flw:
      if (isRvzca() and isRvzcf()) execFlw(di);  else illegalInst(di);
      return;

    case InstId::c_ld:
      if (not isRvzca() or (not isRv64() and not isRvzclsd()))
        illegalInst(di);  // Must have Zca, and in Rv32 must have Zclsd.
      else
        execLd(di);
      return;

    case InstId::c_fsd:
      if (isRvzca() and isRvzcd()) execFsd(di); else illegalInst(di);
      return;

    case InstId::c_sq:
      if (isRvzca()) execSq(di); else illegalInst(di);
      return;

    case InstId::c_sw:
      if (isRvzca()) execSw(di); else illegalInst(di);
      return;

    case InstId::c_fsw:
      if (isRvzca() and isRvzcf()) execFsw(di);  else illegalInst(di);
      return;

    case InstId::c_sd:
      if (not isRvzca() or (not isRv64() and not isRvzclsd()))
        illegalInst(di);  // Must have Zca, and in Rv32 must have Zclsd.
      else
        execSd(di);
      return;

    case InstId::c_addi:
      if (isRvzca()) execAddi(di); else illegalInst(di);
      return;

    case InstId::c_jal:
      if (isRvzca()) execJal(di); else illegalInst(di);
      return;

    case InstId::c_li:
    case InstId::c_addi16sp:
      if (isRvzca()) execAddi(di); else illegalInst(di);
      return;

    case InstId::c_lui:
      if (isRvzca()) execLui(di); else illegalInst(di);
      return;

    case InstId::c_srli:
      if (isRvzca()) execSrli(di); else illegalInst(di);
      return;

    case InstId::c_srli64:
      illegalInst(di);  // Only valid in rv128 which is not supported.
      return;

    case InstId::c_srai:
      if (isRvzca()) execSrai(di); else illegalInst(di);
      return;

    case InstId::c_srai64:
      illegalInst(di);  // Only valid in rv128 which is not supported.
      return;

    case InstId::c_andi:
      if (isRvzca()) execAndi(di); else illegalInst(di);
      return;

    case InstId::c_sub:
      if (isRvzca()) execSub(di); else illegalInst(di);
      return;

    case InstId::c_xor:
      if (isRvzca()) execXor(di); else illegalInst(di);
      return;

    case InstId::c_or:
      if (isRvzca()) execOr(di); else illegalInst(di);
      return;

    case InstId::c_and:
      if (isRvzca()) execAnd(di); else illegalInst(di);
      return;

    case InstId::c_subw:
      if (isRvzca()) execSubw(di); else illegalInst(di);
      return;

    case InstId::c_addw:
      if (isRvzca()) execAddw(di); else illegalInst(di);
      return;

    case InstId::c_j:
      if (isRvzca()) execJal(di); else illegalInst(di);
      return;

    case InstId::c_beqz:
      if (isRvzca()) execBeq(di); else illegalInst(di);
      return;

    case InstId::c_bnez:
      if (isRvzca()) execBne(di); else illegalInst(di);
      return;

    case InstId::c_slli:
    case InstId::c_slli64:
      if (isRvzca()) execSlli(di); else illegalInst(di);
      return;

    case InstId::c_fldsp:
      if (isRvzca() and isRvzcd()) execFld(di); else illegalInst(di);
      return;

    case InstId::c_lwsp:
      if (isRvzca()) execLw(di); else illegalInst(di);
      return;

    case InstId::c_flwsp:
      if (isRvzca() and isRvzcf()) execFlw(di); else illegalInst(di);
      return;

    case InstId::c_ldsp:
      if (not isRvzca() or (not isRv64() and not isRvzclsd()))
        illegalInst(di);  // Must have Zca, and in Rv32 must have Zclsd.
      else
        execLd(di);
      return;

    case InstId::c_jr:
      if (isRvzca()) execJalr(di); else illegalInst(di);
      return;

    case InstId::c_mv:
      if (isRvzca()) execAdd(di); else illegalInst(di);
      return;

    case InstId::c_ebreak:
      if (isRvzca()) execEbreak(di); else illegalInst(di);
      return;

    case InstId::c_jalr:
      if (isRvzca()) execJalr(di); else illegalInst(di);
      return;

    case InstId::c_add:
      if (isRvzca()) execAdd(di); else illegalInst(di);
      return;

    case InstId::c_fsdsp:
      if (isRvzca() and isRvzcd()) execFsd(di); else illegalInst(di);
      return;

    case InstId::c_swsp:
      if (isRvzca()) execSw(di); else illegalInst(di);
      return;

    case InstId::c_fswsp:
      if (isRvzca() and isRvzcf()) execFsw(di); else illegalInst(di);
      return;

    case InstId::c_addiw:
      if (isRvzca()) execAddiw(di); else illegalInst(di);
      return;

    case InstId::c_sdsp:
      if (not isRvzca() or (not isRv64() and not isRvzclsd()))
        illegalInst(di);  // Must have Zca, and in Rv32 must have Zclsd.
      else
        execSd(di);
      return;

    case InstId::clz:
      execClz(di);
      return;

    case InstId::ctz:
      execCtz(di);
      return;

    case InstId::cpop:
      execCpop(di);
      return;

    case InstId::clzw:
      execClzw(di);
      return;

    case InstId::ctzw:
      execCtzw(di);
      return;

    case InstId::cpopw:
      execCpopw(di);
      return;

    case InstId::min:
      execMin(di);
      return;

    case InstId::max:
      execMax(di);
      return;

    case InstId::minu:
      execMinu(di);
      return;

    case InstId::maxu:
      execMaxu(di);
      return;

    case InstId::sext_b:
      execSext_b(di);
      return;

    case InstId::sext_h:
      execSext_h(di);
      return;

    case InstId::andn:
      execAndn(di);
      return;

    case InstId::orc_b:
      execOrc_b(di);
      return;

    case InstId::orn:
      execOrn(di);
      return;

    case InstId::xnor:
      execXnor(di);
      return;

    case InstId::rol:
      execRol(di);
      return;

    case InstId::ror:
      execRor(di);
      return;

    case InstId::rori:
      execRori(di);
      return;

    case InstId::rolw:
      execRolw(di);
      return;

    case InstId::rorw:
      execRorw(di);
      return;

    case InstId::roriw:
      execRoriw(di);
      return;

    case InstId::pack:
      execPack(di);
      return;

    case InstId::packh:
      execPackh(di);
      return;

    case InstId::packw:
      execPackw(di);
      return;

    case InstId::brev8:
      execBrev8(di);
      return;

    case InstId::rev8_32:
      execRev8_32(di);
      return;

    case InstId::rev8_64:
      execRev8_64(di);
      return;

    case InstId::zip:
      execZip(di);
      return;

    case InstId::unzip:
      execUnzip(di);
      return;

    case InstId::xperm_n:
      execXperm_n(di);
      return;

    case InstId::xperm_b:
      execXperm_b(di);
      return;

    case InstId::bset:
      execBset(di);
      return;

    case InstId::bclr:
      execBclr(di);
      return;

    case InstId::binv:
      execBinv(di);
      return;

    case InstId::bext:
      execBext(di);
      return;

    case InstId::bseti:
      execBseti(di);
      return;

    case InstId::bclri:
      execBclri(di);
      return;

    case InstId::binvi:
      execBinvi(di);
      return;

    case InstId::bexti:
      execBexti(di);
      return;

    case InstId::clmul:
      execClmul(di);
      return;

    case InstId::clmulh:
      execClmulh(di);
      return;

    case InstId::clmulr:
      execClmulr(di);
      return;

    case InstId::sh1add:
      execSh1add(di);
      return;

    case InstId::sh2add:
      execSh2add(di);
      return;

    case InstId::sh3add:
      execSh3add(di);
      return;

    case InstId::sh1add_uw:
      execSh1add_uw(di);
      return;

    case InstId::sh2add_uw:
      execSh2add_uw(di);
      return;

    case InstId::sh3add_uw:
      execSh3add_uw(di);
      return;

    case InstId::add_uw:
      execAdd_uw(di);
      return;

    case InstId::slli_uw:
      execSlli_uw(di);
      return;

    case InstId::vsetvli:
      execVsetvli(di);
      return;

    case InstId::vsetivli:
      execVsetivli(di);
      return;

    case InstId::vsetvl:
      execVsetvl(di);
      return;

    case InstId::vadd_vv:
      execVadd_vv(di);
      return;

    case InstId::vadd_vx:
      execVadd_vx(di);
      return;

    case InstId::vadd_vi:
      execVadd_vi(di);
      return;

    case InstId::vsub_vv:
      execVsub_vv(di);
      return;

    case InstId::vsub_vx:
      execVsub_vx(di);
      return;

    case InstId::vrsub_vx:
      execVrsub_vx(di);
      return;

    case InstId::vrsub_vi:
      execVrsub_vi(di);
      return;

    case InstId::vwaddu_vv:
      execVwaddu_vv(di);
      return;

    case InstId::vwaddu_vx:
      execVwaddu_vx(di);
      return;

    case InstId::vwsubu_vv:
      execVwsubu_vv(di);
      return;

    case InstId::vwsubu_vx:
      execVwsubu_vx(di);
      return;

    case InstId::vwadd_vv:
      execVwadd_vv(di);
      return;

    case InstId::vwadd_vx:
      execVwadd_vx(di);
      return;

    case InstId::vwsub_vv:
      execVwsub_vv(di);
      return;

    case InstId::vwsub_vx:
      execVwsub_vx(di);
      return;

    case InstId::vwaddu_wv:
      execVwaddu_wv(di);
      return;

    case InstId::vwaddu_wx:
      execVwaddu_wx(di);
      return;

    case InstId::vwsubu_wv:
      execVwsubu_wv(di);
      return;

    case InstId::vwsubu_wx:
      execVwsubu_wx(di);
      return;

    case InstId::vwadd_wv:
      execVwadd_wv(di);
      return;

    case InstId::vwadd_wx:
      execVwadd_wx(di);
      return;

    case InstId::vwsub_wv:
      execVwsub_wv(di);
      return;

    case InstId::vwsub_wx:
      execVwsub_wx(di);
      return;

    case InstId::vmseq_vv:
      execVmseq_vv(di);
      return;

    case InstId::vmseq_vx:
      execVmseq_vx(di);
      return;

    case InstId::vmseq_vi:
      execVmseq_vi(di);
      return;

    case InstId::vmsne_vv:
      execVmsne_vv(di);
      return;

    case InstId::vmsne_vx:
      execVmsne_vx(di);
      return;

    case InstId::vmsne_vi:
      execVmsne_vi(di);
      return;

    case InstId::vmsltu_vv:
      execVmsltu_vv(di);
      return;

    case InstId::vmsltu_vx:
      execVmsltu_vx(di);
      return;

    case InstId::vmslt_vv:
      execVmslt_vv(di);
      return;

    case InstId::vmslt_vx:
      execVmslt_vx(di);
      return;

    case InstId::vmsleu_vv:
      execVmsleu_vv(di);
      return;

    case InstId::vmsleu_vx:
      execVmsleu_vx(di);
      return;

    case InstId::vmsleu_vi:
      execVmsleu_vi(di);
      return;

    case InstId::vmsle_vv:
      execVmsle_vv(di);
      return;

    case InstId::vmsle_vx:
      execVmsle_vx(di);
      return;

    case InstId::vmsle_vi:
      execVmsle_vi(di);
      return;

    case InstId::vmsgtu_vx:
      execVmsgtu_vx(di);
      return;

    case InstId::vmsgtu_vi:
      execVmsgtu_vi(di);
      return;

    case InstId::vmsgt_vx:
      execVmsgt_vx(di);
      return;

    case InstId::vmsgt_vi:
      execVmsgt_vi(di);
      return;

    case InstId::vminu_vv:
      execVminu_vv(di);
      return;

    case InstId::vminu_vx:
      execVminu_vx(di);
      return;

    case InstId::vmin_vv:
      execVmin_vv(di);
      return;

    case InstId::vmin_vx:
      execVmin_vx(di);
      return;

    case InstId::vmaxu_vv:
      execVmaxu_vv(di);
      return;

    case InstId::vmaxu_vx:
      execVmaxu_vx(di);
      return;

    case InstId::vmax_vv:
      execVmax_vv(di);
      return;

    case InstId::vmax_vx:
      execVmax_vx(di);
      return;

    case InstId::vand_vv:
      execVand_vv(di);
      return;

    case InstId::vand_vx:
      execVand_vx(di);
      return;

    case InstId::vand_vi:
      execVand_vi(di);
      return;

    case InstId::vor_vv:
      execVor_vv(di);
      return;

    case InstId::vor_vx:
      execVor_vx(di);
      return;

    case InstId::vor_vi:
      execVor_vi(di);
      return;

    case InstId::vxor_vv:
      execVxor_vv(di);
      return;

    case InstId::vxor_vx:
      execVxor_vx(di);
      return;

    case InstId::vxor_vi:
      execVxor_vi(di);
      return;

    case InstId::vsll_vv:
      execVsll_vv(di);
      return;

    case InstId::vsll_vx:
      execVsll_vx(di);
      return;

    case InstId::vsll_vi:
      execVsll_vi(di);
      return;

    case InstId::vsrl_vv:
      execVsrl_vv(di);
      return;

    case InstId::vsrl_vx:
      execVsrl_vx(di);
      return;

    case InstId::vsrl_vi:
      execVsrl_vi(di);
      return;

    case InstId::vsra_vv:
      execVsra_vv(di);
      return;

    case InstId::vsra_vx:
      execVsra_vx(di);
      return;

    case InstId::vsra_vi:
      execVsra_vi(di);
      return;

    case InstId::vnsrl_wv:
      execVnsrl_wv(di);
      return;

    case InstId::vnsrl_wx:
      execVnsrl_wx(di);
      return;

    case InstId::vnsrl_wi:
      execVnsrl_wi(di);
      return;

    case InstId::vnsra_wv:
      execVnsra_wv(di);
      return;

    case InstId::vnsra_wx:
      execVnsra_wx(di);
      return;

    case InstId::vnsra_wi:
      execVnsra_wi(di);
      return;

    case InstId::vrgather_vv:
      execVrgather_vv(di);
      return;

    case InstId::vrgather_vx:
      execVrgather_vx(di);
      return;

    case InstId::vrgather_vi:
      execVrgather_vi(di);
      return;

    case InstId::vrgatherei16_vv:
      execVrgatherei16_vv(di);
      return;

    case InstId::vcompress_vm:
      execVcompress_vm(di);
      return;

    case InstId::vredsum_vs:
      execVredsum_vs(di);
      return;

    case InstId::vredand_vs:
      execVredand_vs(di);
      return;

    case InstId::vredor_vs:
      execVredor_vs(di);
      return;

    case InstId::vredxor_vs:
      execVredxor_vs(di);
      return;

    case InstId::vredminu_vs:
      execVredminu_vs(di);
      return;

    case InstId::vredmin_vs:
      execVredmin_vs(di);
      return;

    case InstId::vredmaxu_vs:
      execVredmaxu_vs(di);
      return;

    case InstId::vredmax_vs:
      execVredmax_vs(di);
      return;

    case InstId::vwredsumu_vs:
      execVwredsumu_vs(di);
      return;

    case InstId::vwredsum_vs:
      execVwredsum_vs(di);
      return;

    case InstId::vmand_mm:
      execVmand_mm(di);
      return;

    case InstId::vmnand_mm:
      execVmnand_mm(di);
      return;

    case InstId::vmandn_mm:
      execVmandn_mm(di);
      return;

    case InstId::vmxor_mm:
      execVmxor_mm(di);
      return;

    case InstId::vmor_mm:
      execVmor_mm(di);
      return;

    case InstId::vmnor_mm:
      execVmnor_mm(di);
      return;

    case InstId::vmorn_mm:
      execVmorn_mm(di);
      return;

    case InstId::vmxnor_mm:
      execVmxnor_mm(di);
      return;

    case InstId::vcpop_m:
      execVcpop_m(di);
      return;

    case InstId::vfirst_m:
      execVfirst_m(di);
      return;

    case InstId::vmsbf_m:
      execVmsbf_m(di);
      return;

    case InstId::vmsif_m:
      execVmsif_m(di);
      return;

    case InstId::vmsof_m:
      execVmsof_m(di);
      return;

    case InstId::viota_m:
      execViota_m(di);
      return;

    case InstId::vid_v:
      execVid_v(di);
      return;

    case InstId::vslideup_vx:
      execVslideup_vx(di);
      return;

    case InstId::vslideup_vi:
      execVslideup_vi(di);
      return;

    case InstId::vslide1up_vx:
      execVslide1up_vx(di);
      return;

    case InstId::vslidedown_vx:
      execVslidedown_vx(di);
      return;

    case InstId::vslidedown_vi:
      execVslidedown_vi(di);
      return;

    case InstId::vslide1down_vx:
      execVslide1down_vx(di);
      return;

    case InstId::vfslide1up_vf:
      execVfslide1up_vf(di);
      return;

    case InstId::vfslide1down_vf:
      execVfslide1down_vf(di);
      return;

    case InstId::vmul_vv:
      execVmul_vv(di);
      return;

    case InstId::vmul_vx:
      execVmul_vx(di);
      return;

    case InstId::vmulh_vv:
      execVmulh_vv(di);
      return;

    case InstId::vmulh_vx:
      execVmulh_vx(di);
      return;

    case InstId::vmulhu_vv:
      execVmulhu_vv(di);
      return;

    case InstId::vmulhu_vx:
      execVmulhu_vx(di);
      return;

    case InstId::vmulhsu_vv:
      execVmulhsu_vv(di);
      return;

    case InstId::vmulhsu_vx:
      execVmulhsu_vx(di);
      return;

    case InstId::vmadd_vv:
      execVmadd_vv(di);
      return;

    case InstId::vmadd_vx:
      execVmadd_vx(di);
      return;

    case InstId::vnmsub_vv:
      execVnmsub_vv(di);
      return;

    case InstId::vnmsub_vx:
      execVnmsub_vx(di);
      return;

    case InstId::vmacc_vv:
      execVmacc_vv(di);
      return;

    case InstId::vmacc_vx:
      execVmacc_vx(di);
      return;

    case InstId::vnmsac_vv:
      execVnmsac_vv(di);
      return;

    case InstId::vnmsac_vx:
      execVnmsac_vx(di);
      return;

    case InstId::vwmulu_vv:
      execVwmulu_vv(di);
      return;

    case InstId::vwmulu_vx:
      execVwmulu_vx(di);
      return;

    case InstId::vwmul_vv:
      execVwmul_vv(di);
      return;

    case InstId::vwmul_vx:
      execVwmul_vx(di);
      return;

    case InstId::vwmulsu_vv:
      execVwmulsu_vv(di);
      return;

    case InstId::vwmulsu_vx:
      execVwmulsu_vx(di);
      return;

    case InstId::vwmaccu_vv:
      execVwmaccu_vv(di);
      return;

    case InstId::vwmaccu_vx:
      execVwmaccu_vx(di);
      return;

    case InstId::vwmacc_vv:
      execVwmacc_vv(di);
      return;

    case InstId::vwmacc_vx:
      execVwmacc_vx(di);
      return;

    case InstId::vwmaccsu_vv:
      execVwmaccsu_vv(di);
      return;

    case InstId::vwmaccsu_vx:
      execVwmaccsu_vx(di);
      return;

    case InstId::vwmaccus_vx:
      execVwmaccus_vx(di);
      return;

    case InstId::vdivu_vv:
      execVdivu_vv(di);
      return;

    case InstId::vdivu_vx:
      execVdivu_vx(di);
      return;

    case InstId::vdiv_vv:
      execVdiv_vv(di);
      return;

    case InstId::vdiv_vx:
      execVdiv_vx(di);
      return;

    case InstId::vremu_vv:
      execVremu_vv(di);
      return;

    case InstId::vremu_vx:
      execVremu_vx(di);
      return;

    case InstId::vrem_vv:
      execVrem_vv(di);
      return;

    case InstId::vrem_vx:
      execVrem_vx(di);
      return;

    case InstId::vsext_vf2:
      execVsext_vf2(di);
      return;

    case InstId::vsext_vf4:
      execVsext_vf4(di);
      return;

    case InstId::vsext_vf8:
      execVsext_vf8(di);
      return;

    case InstId::vzext_vf2:
      execVzext_vf2(di);
      return;

    case InstId::vzext_vf4:
      execVzext_vf4(di);
      return;

    case InstId::vzext_vf8:
      execVzext_vf8(di);
      return;

    case InstId::vadc_vvm:
      execVadc_vvm(di);
      return;

    case InstId::vadc_vxm:
      execVadc_vxm(di);
      return;

    case InstId::vadc_vim:
      execVadc_vim(di);
      return;

    case InstId::vsbc_vvm:
      execVsbc_vvm(di);
      return;

    case InstId::vsbc_vxm:
      execVsbc_vxm(di);
      return;

    case InstId::vmadc_vvm:
      execVmadc_vvm(di);
      return;

    case InstId::vmadc_vxm:
      execVmadc_vxm(di);
      return;

    case InstId::vmadc_vim:
      execVmadc_vim(di);
      return;

    case InstId::vmsbc_vvm:
      execVmsbc_vvm(di);
      return;

    case InstId::vmsbc_vxm:
      execVmsbc_vxm(di);
      return;

    case InstId::vmerge_vvm:
      execVmerge_vvm(di);
      return;

    case InstId::vmerge_vxm:
      execVmerge_vxm(di);
      return;

    case InstId::vmerge_vim:
      execVmerge_vim(di);
      return;

    case InstId::vmv_x_s:
      execVmv_x_s(di);
      return;

    case InstId::vmv_s_x:
      execVmv_s_x(di);
      return;

    case InstId::vfmv_f_s:
      execVfmv_f_s(di);
      return;

    case InstId::vfmv_s_f:
      execVfmv_s_f(di);
      return;

    case InstId::vmv_v_v:
      execVmv_v_v(di);
      return;

    case InstId::vmv_v_x:
      execVmv_v_x(di);
      return;

    case InstId::vmv_v_i:
      execVmv_v_i(di);
      return;

    case InstId::vmv1r_v:
      execVmv1r_v(di);
      return;

    case InstId::vmv2r_v:
      execVmv2r_v(di);
      return;

    case InstId::vmv4r_v:
      execVmv4r_v(di);
      return;

    case InstId::vmv8r_v:
      execVmv8r_v(di);
      return;

    case InstId::vsaddu_vv:
      execVsaddu_vv(di);
      return;

    case InstId::vsaddu_vx:
      execVsaddu_vx(di);
      return;

    case InstId::vsaddu_vi:
      execVsaddu_vi(di);
      return;

    case InstId::vsadd_vv:
      execVsadd_vv(di);
      return;

    case InstId::vsadd_vx:
      execVsadd_vx(di);
      return;

    case InstId::vsadd_vi:
      execVsadd_vi(di);
      return;

    case InstId::vssubu_vv:
      execVssubu_vv(di);
      return;

    case InstId::vssubu_vx:
      execVssubu_vx(di);
      return;

    case InstId::vssub_vv:
      execVssub_vv(di);
      return;

    case InstId::vssub_vx:
      execVssub_vx(di);
      return;

    case InstId::vaaddu_vv:
      execVaaddu_vv(di);
      return;

    case InstId::vaaddu_vx:
      execVaaddu_vx(di);
      return;

    case InstId::vaadd_vv:
      execVaadd_vv(di);
      return;

    case InstId::vaadd_vx:
      execVaadd_vx(di);
      return;

    case InstId::vasubu_vv:
      execVasubu_vv(di);
      return;

    case InstId::vasubu_vx:
      execVasubu_vx(di);
      return;

    case InstId::vasub_vv:
      execVasub_vv(di);
      return;

    case InstId::vasub_vx:
      execVasub_vx(di);
      return;

    case InstId::vsmul_vv:
      execVsmul_vv(di);
      return;

    case InstId::vsmul_vx:
      execVsmul_vx(di);
      return;

    case InstId::vssrl_vv:
      execVssrl_vv(di);
      return;

    case InstId::vssrl_vx:
      execVssrl_vx(di);
      return;

    case InstId::vssrl_vi:
      execVssrl_vi(di);
      return;

    case InstId::vssra_vv:
      execVssra_vv(di);
      return;

    case InstId::vssra_vx:
      execVssra_vx(di);
      return;

    case InstId::vssra_vi:
      execVssra_vi(di);
      return;

    case InstId::vnclipu_wv:
      execVnclipu_wv(di);
      return;

    case InstId::vnclipu_wx:
      execVnclipu_wx(di);
      return;

    case InstId::vnclipu_wi:
      execVnclipu_wi(di);
      return;

    case InstId::vnclip_wv:
      execVnclip_wv(di);
      return;

    case InstId::vnclip_wx:
      execVnclip_wx(di);
      return;

    case InstId::vnclip_wi:
      execVnclip_wi(di);
      return;

    case InstId::vle8_v:
      execVle8_v(di);
      return;

    case InstId::vle16_v:
      execVle16_v(di);
      return;

    case InstId::vle32_v:
      execVle32_v(di);
      return;

    case InstId::vle64_v:
      execVle64_v(di);
      return;

    case InstId::vle128_v:
      execVle128_v(di);
      return;

    case InstId::vle256_v:
      execVle256_v(di);
      return;

    case InstId::vle512_v:
      execVle512_v(di);
      return;

    case InstId::vle1024_v:
      execVle1024_v(di);
      return;

    case InstId::vse8_v:
      execVse8_v(di);
      return;

    case InstId::vse16_v:
      execVse16_v(di);
      return;

    case InstId::vse32_v:
      execVse32_v(di);
      return;

    case InstId::vse64_v:
      execVse64_v(di);
      return;

    case InstId::vse128_v:
      execVse128_v(di);
      return;

    case InstId::vse256_v:
      execVse256_v(di);
      return;

    case InstId::vse512_v:
      execVse512_v(di);
      return;

    case InstId::vse1024_v:
      execVse1024_v(di);
      return;

    case InstId::vlm_v:
      execVlm_v(di);
      return;

    case InstId::vsm_v:
      execVsm_v(di);
      return;

    case InstId::vlre8_v:
      execVlre8_v(di);
      return;

    case InstId::vlre16_v:
      execVlre16_v(di);
      return;

    case InstId::vlre32_v:
      execVlre32_v(di);
      return;

    case InstId::vlre64_v:
      execVlre64_v(di);
      return;

    case InstId::vlre128_v:
      execVlre128_v(di);
      return;

    case InstId::vlre256_v:
      execVlre256_v(di);
      return;

    case InstId::vlre512_v:
      execVlre512_v(di);
      return;

    case InstId::vlre1024_v:
      execVlre1024_v(di);
      return;

    case InstId::vs1r_v:
      execVs1r_v(di);
      return;

    case InstId::vs2r_v:
      execVs2r_v(di);
      return;

    case InstId::vs4r_v:
      execVs4r_v(di);
      return;

    case InstId::vs8r_v:
      execVs8r_v(di);
      return;

    case InstId::vle8ff_v:
      execVle8ff_v(di);
      return;

    case InstId::vle16ff_v:
      execVle16ff_v(di);
      return;

    case InstId::vle32ff_v:
      execVle32ff_v(di);
      return;

    case InstId::vle64ff_v:
      execVle64ff_v(di);
      return;

    case InstId::vle128ff_v:
      execVle128ff_v(di);
      return;

    case InstId::vle256ff_v:
      execVle256ff_v(di);
      return;

    case InstId::vle512ff_v:
      execVle512ff_v(di);
      return;

    case InstId::vle1024ff_v:
      execVle1024ff_v(di);
      return;

    case InstId::vlse8_v:
      execVlse8_v(di);
      return;

    case InstId::vlse16_v:
      execVlse16_v(di);
      return;

    case InstId::vlse32_v:
      execVlse32_v(di);
      return;

    case InstId::vlse64_v:
      execVlse64_v(di);
      return;

    case InstId::vlse128_v:
      execVlse128_v(di);
      return;

    case InstId::vlse256_v:
      execVlse256_v(di);
      return;

    case InstId::vlse512_v:
      execVlse512_v(di);
      return;

    case InstId::vlse1024_v:
      execVlse1024_v(di);
      return;

    case InstId::vsse8_v:
      execVsse8_v(di);
      return;

    case InstId::vsse16_v:
      execVsse16_v(di);
      return;

    case InstId::vsse32_v:
      execVsse32_v(di);
      return;

    case InstId::vsse64_v:
      execVsse64_v(di);
      return;

    case InstId::vsse128_v:
      execVsse128_v(di);
      return;

    case InstId::vsse256_v:
      execVsse256_v(di);
      return;

    case InstId::vsse512_v:
      execVsse512_v(di);
      return;

    case InstId::vsse1024_v:
      execVsse1024_v(di);
      return;

    case InstId::vloxei8_v:
      execVloxei8_v(di);
      return;

    case InstId::vloxei16_v:
      execVloxei16_v(di);
      return;

    case InstId::vloxei32_v:
      execVloxei32_v(di);
      return;

    case InstId::vloxei64_v:
      execVloxei64_v(di);
      return;

    case InstId::vloxei128_v:
      execVloxei128_v(di);
      return;

    case InstId::vloxei256_v:
      execVloxei256_v(di);
      return;

    case InstId::vloxei512_v:
      execVloxei512_v(di);
      return;

    case InstId::vloxei1024_v:
      execVloxei1024_v(di);
      return;

    case InstId::vluxei8_v:
      execVluxei8_v(di);
      return;

    case InstId::vluxei16_v:
      execVluxei16_v(di);
      return;

    case InstId::vluxei32_v:
      execVluxei32_v(di);
      return;

    case InstId::vluxei64_v:
      execVluxei64_v(di);
      return;

    case InstId::vluxei128_v:
      execVluxei128_v(di);
      return;

    case InstId::vluxei256_v:
      execVluxei256_v(di);
      return;

    case InstId::vluxei512_v:
      execVluxei512_v(di);
      return;

    case InstId::vluxei1024_v:
      execVluxei1024_v(di);
      return;

    case InstId::vsoxei8_v:
      execVsoxei8_v(di);
      return;

    case InstId::vsoxei16_v:
      execVsoxei16_v(di);
      return;

    case InstId::vsoxei32_v:
      execVsoxei32_v(di);
      return;

    case InstId::vsoxei64_v:
      execVsoxei64_v(di);
      return;

    case InstId::vsoxei128_v:
      execVsoxei128_v(di);
      return;

    case InstId::vsoxei256_v:
      execVsoxei256_v(di);
      return;

    case InstId::vsoxei512_v:
      execVsoxei512_v(di);
      return;

    case InstId::vsoxei1024_v:
      execVsoxei1024_v(di);
      return;

    case InstId::vsuxei8_v:
      execVsuxei8_v(di);
      return;

    case InstId::vsuxei16_v:
      execVsuxei16_v(di);
      return;

    case InstId::vsuxei32_v:
      execVsuxei32_v(di);
      return;

    case InstId::vsuxei64_v:
      execVsuxei64_v(di);
      return;

    case InstId::vsuxei128_v:
      execVsuxei128_v(di);
      return;

    case InstId::vsuxei256_v:
      execVsuxei256_v(di);
      return;

    case InstId::vsuxei512_v:
      execVsuxei512_v(di);
      return;

    case InstId::vsuxei1024_v:
      execVsuxei1024_v(di);
      return;

    case InstId::vlsege8_v:
      execVlsege8_v(di);
      return;

    case InstId::vlsege16_v:
      execVlsege16_v(di);
      return;

    case InstId::vlsege32_v:
      execVlsege32_v(di);
      return;

    case InstId::vlsege64_v:
      execVlsege64_v(di);
      return;

    case InstId::vlsege128_v:
      execVlsege128_v(di);
      return;

    case InstId::vlsege256_v:
      execVlsege256_v(di);
      return;

    case InstId::vlsege512_v:
      execVlsege512_v(di);
      return;

    case InstId::vlsege1024_v:
      execVlsege1024_v(di);
      return;

    case InstId::vssege8_v:
      execVssege8_v(di);
      return;

    case InstId::vssege16_v:
      execVssege16_v(di);
      return;

    case InstId::vssege32_v:
      execVssege32_v(di);
      return;

    case InstId::vssege64_v:
      execVssege64_v(di);
      return;

    case InstId::vssege128_v:
      execVssege128_v(di);
      return;

    case InstId::vssege256_v:
      execVssege256_v(di);
      return;

    case InstId::vssege512_v:
      execVssege512_v(di);
      return;

    case InstId::vssege1024_v:
      execVssege1024_v(di);
      return;

    case InstId::vlssege8_v:
      execVlssege8_v(di);
      return;

    case InstId::vlssege16_v:
      execVlssege16_v(di);
      return;

    case InstId::vlssege32_v:
      execVlssege32_v(di);
      return;

    case InstId::vlssege64_v:
      execVlssege64_v(di);
      return;

    case InstId::vlssege128_v:
      execVlssege128_v(di);
      return;

    case InstId::vlssege256_v:
      execVlssege256_v(di);
      return;

    case InstId::vlssege512_v:
      execVlssege512_v(di);
      return;

    case InstId::vlssege1024_v:
      execVlssege1024_v(di);
      return;

    case InstId::vsssege8_v:
      execVsssege8_v(di);
      return;

    case InstId::vsssege16_v:
      execVsssege16_v(di);
      return;

    case InstId::vsssege32_v:
      execVsssege32_v(di);
      return;

    case InstId::vsssege64_v:
      execVsssege64_v(di);
      return;

    case InstId::vsssege128_v:
      execVsssege128_v(di);
      return;

    case InstId::vsssege256_v:
      execVsssege256_v(di);
      return;

    case InstId::vsssege512_v:
      execVsssege512_v(di);
      return;

    case InstId::vsssege1024_v:
      execVsssege1024_v(di);
      return;

    case InstId::vluxsegei8_v:
      execVluxsegei8_v(di);
      return;

    case InstId::vluxsegei16_v:
      execVluxsegei16_v(di);
      return;

    case InstId::vluxsegei32_v:
      execVluxsegei32_v(di);
      return;

    case InstId::vluxsegei64_v:
      execVluxsegei64_v(di);
      return;

    case InstId::vluxsegei128_v:
      execVluxsegei128_v(di);
      return;

    case InstId::vluxsegei256_v:
      execVluxsegei256_v(di);
      return;

    case InstId::vluxsegei512_v:
      execVluxsegei512_v(di);
      return;

    case InstId::vluxsegei1024_v:
      execVluxsegei1024_v(di);
      return;

    case InstId::vsuxsegei8_v:
      execVsuxsegei8_v(di);
      return;

    case InstId::vsuxsegei16_v:
      execVsuxsegei16_v(di);
      return;

    case InstId::vsuxsegei32_v:
      execVsuxsegei32_v(di);
      return;

    case InstId::vsuxsegei64_v:
      execVsuxsegei64_v(di);
      return;

    case InstId::vsuxsegei128_v:
      execVsuxsegei128_v(di);
      return;

    case InstId::vsuxsegei256_v:
      execVsuxsegei256_v(di);
      return;

    case InstId::vsuxsegei512_v:
      execVsuxsegei512_v(di);
      return;

    case InstId::vsuxsegei1024_v:
      execVsuxsegei1024_v(di);
      return;

    case InstId::vloxsegei8_v:
      execVloxsegei8_v(di);
      return;

    case InstId::vloxsegei16_v:
      execVloxsegei16_v(di);
      return;

    case InstId::vloxsegei32_v:
      execVloxsegei32_v(di);
      return;

    case InstId::vloxsegei64_v:
      execVloxsegei64_v(di);
      return;

    case InstId::vloxsegei128_v:
      execVloxsegei128_v(di);
      return;

    case InstId::vloxsegei256_v:
      execVloxsegei256_v(di);
      return;

    case InstId::vloxsegei512_v:
      execVloxsegei512_v(di);
      return;

    case InstId::vloxsegei1024_v:
      execVloxsegei1024_v(di);
      return;

    case InstId::vsoxsegei8_v:
      execVsoxsegei8_v(di);
      return;

    case InstId::vsoxsegei16_v:
      execVsoxsegei16_v(di);
      return;

    case InstId::vsoxsegei32_v:
      execVsoxsegei32_v(di);
      return;

    case InstId::vsoxsegei64_v:
      execVsoxsegei64_v(di);
      return;

    case InstId::vsoxsegei128_v:
      execVsoxsegei128_v(di);
      return;

    case InstId::vsoxsegei256_v:
      execVsoxsegei256_v(di);
      return;

    case InstId::vsoxsegei512_v:
      execVsoxsegei512_v(di);
      return;

    case InstId::vsoxsegei1024_v:
      execVsoxsegei1024_v(di);
      return;

    case InstId::vlsege8ff_v:
      execVlsege8ff_v(di);
      return;

    case InstId::vlsege16ff_v:
      execVlsege16ff_v(di);
      return;

    case InstId::vlsege32ff_v:
      execVlsege32ff_v(di);
      return;

    case InstId::vlsege64ff_v:
      execVlsege64ff_v(di);
      return;

    case InstId::vlsege128ff_v:
      execVlsege128ff_v(di);
      return;

    case InstId::vlsege256ff_v:
      execVlsege256ff_v(di);
      return;

    case InstId::vlsege512ff_v:
      execVlsege512ff_v(di);
      return;

    case InstId::vlsege1024ff_v:
      execVlsege1024ff_v(di);
      return;

    case InstId::vfadd_vv:
      execVfadd_vv(di);
      return;

    case InstId::vfadd_vf:
      execVfadd_vf(di);
      return;

    case InstId::vfsub_vv:
      execVfsub_vv(di);
      return;

    case InstId::vfsub_vf:
      execVfsub_vf(di);
      return;

    case InstId::vfrsub_vf:
      execVfrsub_vf(di);
      return;

    case InstId::vfwadd_vv:
      execVfwadd_vv(di);
      return;

    case InstId::vfwadd_vf:
      execVfwadd_vf(di);
      return;

    case InstId::vfwsub_vv:
      execVfwsub_vv(di);
      return;

    case InstId::vfwsub_vf:
      execVfwsub_vf(di);
      return;

    case InstId::vfwadd_wv:
      execVfwadd_wv(di);
      return;

    case InstId::vfwadd_wf:
      execVfwadd_wf(di);
      return;

    case InstId::vfwsub_wv:
      execVfwsub_wv(di);
      return;

    case InstId::vfwsub_wf:
      execVfwsub_wf(di);
      return;

    case InstId::vfmul_vv:
      execVfmul_vv(di);
      return;

    case InstId::vfmul_vf:
      execVfmul_vf(di);
      return;

    case InstId::vfdiv_vv:
      execVfdiv_vv(di);
      return;

    case InstId::vfdiv_vf:
      execVfdiv_vf(di);
      return;

    case InstId::vfrdiv_vf:
      execVfrdiv_vf(di);
      return;

    case InstId::vfwmul_vv:
      execVfwmul_vv(di);
      return;

    case InstId::vfwmul_vf:
      execVfwmul_vf(di);
      return;

    case InstId::vfmadd_vv:
      execVfmadd_vv(di);
      return;

    case InstId::vfmadd_vf:
      execVfmadd_vf(di);
      return;

    case InstId::vfnmadd_vv:
      execVfnmadd_vv(di);
      return;

    case InstId::vfnmadd_vf:
      execVfnmadd_vf(di);
      return;

    case InstId::vfmsub_vv:
      execVfmsub_vv(di);
      return;

    case InstId::vfmsub_vf:
      execVfmsub_vf(di);
      return;

    case InstId::vfnmsub_vv:
      execVfnmsub_vv(di);
      return;

    case InstId::vfnmsub_vf:
      execVfnmsub_vf(di);
      return;

    case InstId::vfmacc_vv:
      execVfmacc_vv(di);
      return;

    case InstId::vfmacc_vf:
      execVfmacc_vf(di);
      return;

    case InstId::vfnmacc_vv:
      execVfnmacc_vv(di);
      return;

    case InstId::vfnmacc_vf:
      execVfnmacc_vf(di);
      return;

    case InstId::vfmsac_vv:
      execVfmsac_vv(di);
      return;

    case InstId::vfmsac_vf:
      execVfmsac_vf(di);
      return;

    case InstId::vfnmsac_vv:
      execVfnmsac_vv(di);
      return;

    case InstId::vfnmsac_vf:
      execVfnmsac_vf(di);
      return;

    case InstId::vfwmacc_vv:
      execVfwmacc_vv(di);
      return;

    case InstId::vfwmacc_vf:
      execVfwmacc_vf(di);
      return;

    case InstId::vfwnmacc_vv:
      execVfwnmacc_vv(di);
      return;

    case InstId::vfwnmacc_vf:
      execVfwnmacc_vf(di);
      return;

    case InstId::vfwmsac_vv:
      execVfwmsac_vv(di);
      return;

    case InstId::vfwmsac_vf:
      execVfwmsac_vf(di);
      return;

    case InstId::vfwnmsac_vv:
      execVfwnmsac_vv(di);
      return;

    case InstId::vfwnmsac_vf:
      execVfwnmsac_vf(di);
      return;

    case InstId::vfsqrt_v:
      execVfsqrt_v(di);
      return;

    case InstId::vfmerge_vfm:
      execVfmerge_vfm(di);
      return;

    case InstId::vfmv_v_f:
      execVfmv_v_f(di);
      return;

    case InstId::vmfeq_vv:
      execVmfeq_vv(di);
      return;

    case InstId::vmfeq_vf:
      execVmfeq_vf(di);
      return;

    case InstId::vmfne_vv:
      execVmfne_vv(di);
      return;

    case InstId::vmfne_vf:
      execVmfne_vf(di);
      return;

    case InstId::vmflt_vv:
      execVmflt_vv(di);
      return;

    case InstId::vmflt_vf:
      execVmflt_vf(di);
      return;

    case InstId::vmfle_vv:
      execVmfle_vv(di);
      return;

    case InstId::vmfle_vf:
      execVmfle_vf(di);
      return;

    case InstId::vmfgt_vf:
      execVmfgt_vf(di);
      return;

    case InstId::vmfge_vf:
      execVmfge_vf(di);
      return;

    case InstId::vfclass_v:
      execVfclass_v(di);
      return;

    case InstId::vfcvt_xu_f_v:
      execVfcvt_xu_f_v(di);
      return;

    case InstId::vfcvt_x_f_v:
      execVfcvt_x_f_v(di);
      return;

    case InstId::vfcvt_rtz_xu_f_v:
      execVfcvt_rtz_xu_f_v(di);
      return;

    case InstId::vfcvt_rtz_x_f_v:
      execVfcvt_rtz_x_f_v(di);
      return;

    case InstId::vfcvt_f_xu_v:
      execVfcvt_f_xu_v(di);
      return;

    case InstId::vfcvt_f_x_v:
      execVfcvt_f_x_v(di);
      return;

    case InstId::vfwcvt_xu_f_v:
      execVfwcvt_xu_f_v(di);
      return;

    case InstId::vfwcvt_x_f_v:
      execVfwcvt_x_f_v(di);
      return;

    case InstId::vfwcvt_rtz_xu_f_v:
      execVfwcvt_rtz_xu_f_v(di);
      return;

    case InstId::vfwcvt_rtz_x_f_v:
      execVfwcvt_rtz_x_f_v(di);
      return;

    case InstId::vfwcvt_f_xu_v:
      execVfwcvt_f_xu_v(di);
      return;

    case InstId::vfwcvt_f_x_v:
      execVfwcvt_f_x_v(di);
      return;

    case InstId::vfwcvt_f_f_v:
      execVfwcvt_f_f_v(di);
      return;

    case InstId::vfncvt_xu_f_w:
      execVfncvt_xu_f_w(di);
      return;

    case InstId::vfncvt_x_f_w:
      execVfncvt_x_f_w(di);
      return;

    case InstId::vfncvt_rtz_xu_f_w:
      execVfncvt_rtz_xu_f_w(di);
      return;

    case InstId::vfncvt_rtz_x_f_w:
      execVfncvt_rtz_x_f_w(di);
      return;

    case InstId::vfncvt_f_xu_w:
      execVfncvt_f_xu_w(di);
      return;

    case InstId::vfncvt_f_x_w:
      execVfncvt_f_x_w(di);
      return;

    case InstId::vfncvt_f_f_w:
      execVfncvt_f_f_w(di);
      return;

    case InstId::vfncvt_rod_f_f_w:
      execVfncvt_rod_f_f_w(di);
      return;

    case InstId::vfredusum_vs:
      execVfredusum_vs(di);
      return;

    case InstId::vfredosum_vs:
      execVfredosum_vs(di);
      return;

    case InstId::vfredmin_vs:
      execVfredmin_vs(di);
      return;

    case InstId::vfredmax_vs:
      execVfredmax_vs(di);
      return;

    case InstId::vfwredusum_vs:
      execVfwredusum_vs(di);
      return;

    case InstId::vfwredosum_vs:
      execVfwredosum_vs(di);
      return;

    case InstId::vfrsqrt7_v:
      execVfrsqrt7_v(di);
      return;

    case InstId::vfrec7_v:
      execVfrec7_v(di);
      return;

    case InstId::vfmin_vv:
      execVfmin_vv(di);
      return;

    case InstId::vfmin_vf:
      execVfmin_vf(di);
      return;

    case InstId::vfmax_vv:
      execVfmax_vv(di);
      return;

    case InstId::vfmax_vf:
      execVfmax_vf(di);
      return;

    case InstId::vfsgnj_vv:
      execVfsgnj_vv(di);
      return;

    case InstId::vfsgnj_vf:
      execVfsgnj_vf(di);
      return;

    case InstId::vfsgnjn_vv:
      execVfsgnjn_vv(di);
      return;

    case InstId::vfsgnjn_vf:
      execVfsgnjn_vf(di);
      return;

    case InstId::vfsgnjx_vv:
      execVfsgnjx_vv(di);
      return;

    case InstId::vfsgnjx_vf:
      execVfsgnjx_vf(di);
      return;

    case InstId::vandn_vv:
      execVandn_vv(di);
      return;

    case InstId::vandn_vx:
      execVandn_vx(di);
      return;

    case InstId::vbrev_v:
      execVbrev_v(di);
      return;

    case InstId::vbrev8_v:
      execVbrev8_v(di);
      return;

    case InstId::vrev8_v:
      execVrev8_v(di);
      return;

    case InstId::vclz_v:
      execVclz_v(di);
      return;

    case InstId::vctz_v:
      execVctz_v(di);
      return;

    case InstId::vcpop_v:
      execVcpop_v(di);
      return;

    case InstId::vrol_vv:
      execVrol_vv(di);
      return;

    case InstId::vrol_vx:
      execVrol_vx(di);
      return;

    case InstId::vror_vv:
      execVror_vv(di);
      return;

    case InstId::vror_vx:
      execVror_vx(di);
      return;

    case InstId::vror_vi:
      execVror_vi(di);
      return;

    case InstId::vwsll_vv:
      execVwsll_vv(di);
      return;

    case InstId::vwsll_vx:
      execVwsll_vx(di);
      return;

    case InstId::vwsll_vi:
      execVwsll_vi(di);
      return;

    case InstId::vfncvtbf16_f_f_w:
      execVfncvtbf16_f_f_w(di);
      return;

    case InstId::vfwcvtbf16_f_f_v:
      execVfwcvtbf16_f_f_v(di);
      return;

    case InstId::vfwmaccbf16_vv:
      execVfwmaccbf16_vv(di);
      return;

    case InstId::vfwmaccbf16_vf:
      execVfwmaccbf16_vf(di);
      return;

    case InstId::vfncvtbf16_sat_f_f_w:
      execVfncvtbf16_sat_f_f_w(di);
      return;

    case InstId::vfncvt_f_f_q:
      execVfncvt_f_f_q(di);
      return;

    case InstId::vfncvt_sat_f_f_q:
      execVfncvt_sat_f_f_q(di);
      return;

    case InstId::vclmul_vv:
      execVclmul_vv(di);
      return;

    case InstId::vclmul_vx:
      execVclmul_vx(di);
      return;

    case InstId::vclmulh_vv:
      execVclmulh_vv(di);
      return;

    case InstId::vclmulh_vx:
      execVclmulh_vx(di);
      return;

    case InstId::vghsh_vv:
      execVghsh_vv(di);
      return;

    case InstId::vgmul_vv:
      execVgmul_vv(di);
      return;

    case InstId::vaesdf_vv:
      execVaesdf_vv(di);
      return;

    case InstId::vaesdf_vs:
      execVaesdf_vs(di);
      return;

    case InstId::vaesef_vv:
      execVaesef_vv(di);
      return;

    case InstId::vaesef_vs:
      execVaesef_vs(di);
      return;

    case InstId::vaesem_vv:
      execVaesem_vv(di);
      return;

    case InstId::vaesem_vs:
      execVaesem_vs(di);
      return;

    case InstId::vaesdm_vv:
      execVaesdm_vv(di);
      return;

    case InstId::vaesdm_vs:
      execVaesdm_vs(di);
      return;

    case InstId::vaeskf1_vi:
      execVaeskf1_vi(di);
      return;

    case InstId::vaeskf2_vi:
      execVaeskf2_vi(di);
      return;

    case InstId::vaesz_vs:
      execVaesz_vs(di);
      return;

    case InstId::vsha2ms_vv:
      execVsha2ms_vv(di);
      return;

    case InstId::vsha2ch_vv:
      execVsha2ch_vv(di);
      return;

    case InstId::vsha2cl_vv:
      execVsha2cl_vv(di);
      return;

    case InstId::vsm4k_vi:
      execVsm4k_vi(di);
      return;

    case InstId::vsm4r_vv:
      execVsm4r_vv(di);
      return;

    case InstId::vsm4r_vs:
      execVsm4r_vs(di);
      return;

    case InstId::vsm3me_vv:
      execVsm3me_vv(di);
      return;

    case InstId::vsm3c_vi:
      execVsm3c_vi(di);
      return;

    case InstId::aes32dsi:
      execAes32dsi(di);
      return;

    case InstId::aes32dsmi:
      execAes32dsmi(di);
      return;

    case InstId::aes32esi:
      execAes32esi(di);
      return;

    case InstId::aes32esmi:
      execAes32esmi(di);
      return;

    case InstId::aes64ds:
      execAes64ds(di);
      return;

    case InstId::aes64dsm:
      execAes64dsm(di);
      return;

    case InstId::aes64es:
      execAes64es(di);
      return;

    case InstId::aes64esm:
      execAes64esm(di);
      return;

    case InstId::aes64im:
      execAes64im(di);
      return;

    case InstId::aes64ks1i:
      execAes64ks1i(di);
      return;

    case InstId::aes64ks2:
      execAes64ks2(di);
      return;

    case InstId::sha256sig0:
      execSha256sig0(di);
      return;

    case InstId::sha256sig1:
      execSha256sig1(di);
      return;

    case InstId::sha256sum0:
      execSha256sum0(di);
      return;

    case InstId::sha256sum1:
      execSha256sum1(di);
      return;

    case InstId::sha512sig0h:
      execSha512sig0h(di);
      return;

    case InstId::sha512sig0l:
      execSha512sig0l(di);
      return;

    case InstId::sha512sig1h:
      execSha512sig1h(di);
      return;

    case InstId::sha512sig1l:
      execSha512sig1l(di);
      return;

    case InstId::sha512sum0r:
      execSha512sum0r(di);
      return;

    case InstId::sha512sum1r:
      execSha512sum1r(di);
      return;

    case InstId::sha512sig0:
      execSha512sig0(di);
      return;

    case InstId::sha512sig1:
      execSha512sig1(di);
      return;

    case InstId::sha512sum0:
      execSha512sum0(di);
      return;

    case InstId::sha512sum1:
      execSha512sum1(di);
      return;

    case InstId::sm3p0:
      execSm3p0(di);
      return;

    case InstId::sm3p1:
      execSm3p1(di);
      return;

    case InstId::sm4ed:
      execSm4ed(di);
      return;

    case InstId::sm4ks:
      execSm4ks(di);
      return;

    case InstId::vqdot_vv:
      execVqdot_vv(di);
      return;

    case InstId::vqdot_vx:
      execVqdot_vx(di);
      return;

    case InstId::vqdotu_vv:
      execVqdotu_vv(di);
      return;

    case InstId::vqdotu_vx:
      execVqdotu_vx(di);
      return;

    case InstId::vqdotsu_vv:
      execVqdotsu_vv(di);
      return;

    case InstId::vqdotsu_vx:
      execVqdotsu_vx(di);
      return;

    case InstId::vqdotus_vx:
      execVqdotus_vx(di);
      return;

    case InstId::vzip_vv:
      execVzip_vv(di);
      return;

    case InstId::vunzipe_v:
      execVunzipe_v(di);
      return;

    case InstId::vunzipo_v:
      execVunzipo_v(di);
      return;

    case InstId::vpaire_vv:
      execVpaire_vv(di);
      return;

    case InstId::vpairo_vv:
      execVpairo_vv(di);
      return;

    case InstId::vabs_v:
      execVabs_v(di);
      return;

    case InstId::vabd_vv:
      execVabd_vv(di);
      return;

    case InstId::vabdu_vv:
      execVabdu_vv(di);
      return;

    case InstId::vwabda_vv:
      execVwabda_vv(di);
      return;

    case InstId::vwabdau_vv:
      execVwabdau_vv(di);
      return;

    case InstId::sinval_vma:
      execSinval_vma(di);
      return;

    case InstId::sfence_w_inval:
      execSfence_w_inval(di);
      return;

    case InstId::sfence_inval_ir:
      execSfence_inval_ir(di);
      return;

    case InstId::cbo_clean:
      execCbo_clean(di);
      return;

    case InstId::cbo_flush:
      execCbo_flush(di);
      return;

    case InstId::cbo_inval:
      execCbo_inval(di);
      return;

    case InstId::cbo_zero:
      execCbo_zero(di);
      return;

    case InstId::prefetch_i:
      execPrefetch_i(di);
      return;

    case InstId::prefetch_r:
      execPrefetch_r(di);
      return;

    case InstId::prefetch_w:
      execPrefetch_w(di);
      return;

    case InstId::wrs_nto:
      execWrs_nto(di);
      return;

    case InstId::wrs_sto:
      execWrs_sto(di);
      return;

    case InstId::hfence_vvma:
      execHfence_vvma(di);
      return;

    case InstId::hfence_gvma:
      execHfence_gvma(di);
      return;

    case InstId::hlv_b:
      execHlv_b(di);
      return;

    case InstId::hlv_bu:
      execHlv_bu(di);
      return;

    case InstId::hlv_h:
      execHlv_h(di);
      return;

    case InstId::hlv_hu:
      execHlv_hu(di);
      return;

    case InstId::hlv_w:
      execHlv_w(di);
      return;

    case InstId::hlvx_hu:
      execHlvx_hu(di);
      return;

    case InstId::hlvx_wu:
      execHlvx_wu(di);
      return;

    case InstId::hsv_b:
      execHsv_b(di);
      return;

    case InstId::hsv_h:
      execHsv_h(di);
      return;

    case InstId::hsv_w:
      execHsv_w(di);
      return;

    case InstId::hlv_wu:
      execHlv_wu(di);
      return;

    case InstId::hlv_d:
      execHlv_d(di);
      return;

    case InstId::hsv_d:
      execHsv_d(di);
      return;

    case InstId::hinval_vvma:
      execHinval_vvma(di);
      return;

    case InstId::hinval_gvma:
      execHinval_gvma(di);
      return;

    case InstId::czero_eqz:
      execCzero_eqz(di);
      return;

    case InstId::czero_nez:
      execCzero_nez(di);
      return;

    case InstId::c_lbu:
      if (isRvzca() and isRvzcb()) execLbu(di); else illegalInst(di);
      return;

    case InstId::c_lhu:
      if (isRvzca() and isRvzcb()) execLhu(di); else illegalInst(di);
      return;

    case InstId::c_lh:
      if (isRvzca() and isRvzcb()) execLh(di); else illegalInst(di);
      return;

    case InstId::c_sb:
      if (isRvzca() and isRvzcb()) execSb(di); else illegalInst(di);
      return;

    case InstId::c_sh:
      if (isRvzca() and isRvzcb()) execSh(di); else illegalInst(di);
      return;

    case InstId::c_zext_b:
      if (isRvzca() and isRvzcb()) execAndi(di); else illegalInst(di);
      return;

    case InstId::c_sext_b:
      if (isRvzca() and isRvzcb()) execSext_b(di); else illegalInst(di);
      return;

    case InstId::c_zext_h:
      if (isRvzca() and isRvzcb()) execC_zext_h(di); else illegalInst(di);
      return;

    case InstId::c_sext_h:
      if (isRvzca() and isRvzcb()) execSext_h(di); else illegalInst(di);
      return;

    case InstId::c_zext_w:
      if (isRvzca() and isRvzcb()) execAdd_uw(di); else illegalInst(di);
      return;

    case InstId::c_not:
      if (isRvzca() and isRvzcb()) execXori(di);  else illegalInst(di);
      return;

    case InstId::c_mul:
      if (isRvzca() and isRvzcb()) execMul(di); else illegalInst(di);
      return;

    // Zfa
    case InstId::fcvtmod_w_d:
      execFcvtmod_w_d(di);
      return;

    case InstId::fli_h:
      execFli_h(di);
      return;

    case InstId::fli_s:
      execFli_s(di);
      return;

    case InstId::fli_d:
      execFli_d(di);
      return;

    case InstId::fleq_h:
      execFleq_h(di);
      return;

    case InstId::fleq_s:
      execFleq_s(di);
      return;

    case InstId::fleq_d:
      execFleq_d(di);
      return;

    case InstId::fltq_h:
      execFltq_h(di);
      return;

    case InstId::fltq_s:
      execFltq_s(di);
      return;

    case InstId::fltq_d:
      execFltq_d(di);
      return;

    case InstId::fmaxm_h:
      execFmaxm_h(di);
      return;

    case InstId::fmaxm_s:
      execFmaxm_s(di);
      return;

    case InstId::fmaxm_d:
      execFmaxm_d(di);
      return;

    case InstId::fminm_h:
      execFminm_h(di);
      return;

    case InstId::fminm_s:
      execFminm_s(di);
      return;

    case InstId::fminm_d:
      execFminm_d(di);
      return;

    case InstId::fmvh_x_d:
      execFmvh_x_d(di);
      return;

    case InstId::fmvp_d_x:
      execFmvp_d_x(di);
      return;

    case InstId::fround_h:
      execFround_h(di);
      return;

    case InstId::fround_s:
      execFround_s(di);
      return;

    case InstId::fround_d:
      execFround_d(di);
      return;

    case InstId::froundnx_h:
      execFroundnx_h(di);
      return;

    case InstId::froundnx_s:
      execFroundnx_s(di);
      return;

    case InstId::froundnx_d:
      execFroundnx_d(di);
      return;

    case InstId::amocas_w:
      execAmocas_w(di);
      return;

    case InstId::amocas_d:
      execAmocas_d(di);
      return;

    case InstId::amocas_q:
      execAmocas_q(di);
      return;

    case InstId::mop_r:
      execMop_r(di);
      return;

    case InstId::mop_rr:
      execMop_rr(di);
      return;

    case InstId::c_mop:
      execCmop(di);
      return;

    case InstId::ssamoswap_w:
      execSsamoswap_w(di);
      return;

    case InstId::ssamoswap_d:
      execSsamoswap_d(di);
      return;

    case InstId::mcspspush:
      execMcspspush(di);
      return;

    case InstId::mcspspop:
      execMcspspop(di);
      return;

    case InstId::scspspush:
      execScspspush(di);
      return;

    case InstId::scspspop:
      execScspspop(di);
      return;

    case InstId::mipopret:
      execMipopret(di);
      return;

    case InstId::sipopret:
      execSipopret(di);
      return;

    case InstId::vqwdotau_vv:
      execVqwdotau_vv(di);
      return;

    case InstId::vqwdotas_vv:
      execVqwdotas_vv(di);
      return;

    case InstId::vqwbdotau_vv:
      execVqwbdotau_vv(di);
      return;

    case InstId::vqwbdotas_vv:
      execVqwbdotas_vv(di);
      return;

    case InstId::vfbdota_vv:
      execVfbdota_vv(di);
      return;

    case InstId::vfwdota_vv:
      execVfwdota_vv(di);
      return;

    case InstId::vfqwdota_vv:
      execVfqwdota_vv(di);
      return;

    case InstId::vfqwbdota_vv:
      execVfqwbdota_vv(di);
      return;

    case InstId::vfwbdota_vv:
      execVfwbdota_vv(di);
      return;

    case InstId::endId_:
      assert(0 && "Error: Shouldn't be able to get here");
      return;
    }

  assert(0 && "Error: Shouldn't be able to get here if all cases above returned");
}
// NOLINTEND(readability-function-size)


template <typename URV>
void
Hart<URV>::enableInstructionFrequency(bool b)
{
  instFreq_ = b;
  if (b)
    instProfs_.configure();
}


template <typename URV>
void
Hart<URV>::enterDebugMode_(DebugModeCause cause, URV pc)
{
  if (cancelLrOnDebug_)
    cancelLr(CancelLrCause::ENTER_DEBUG);  // Lose LR reservation.

  if (debugMode_)
    std::cerr << "Warning: Entering debug-mode while in debug-mode\n";
  debugMode_ = true;
  csRegs_.enterDebug(true);
  enteredDebugMode_ = (cause == DebugModeCause::EBREAK) or
                      (cause == DebugModeCause::TRIGGER);

  updateCachedTriggerState();

  URV value = 0;
  if (peekCsr(CsrNumber::DCSR, value))
    {
      DcsrFields<URV> dcsr(value);
      dcsr.bits_.CAUSE = URV(cause);
      dcsr.bits_.PRV = URV(privMode_) & 0x3;
      dcsr.bits_.V = virtMode_;
      if (isRvZicfilp())
        {
          dcsr.bits_.PELP = elp_;
          setElp(false);
        }

      if (nmiPending_)
        dcsr.bits_.NMIP = 1;
      csRegs_.poke(CsrNumber::DCSR, dcsr.value_);
    }

  csRegs_.poke(CsrNumber::DPC, pc);
  setPrivilegeMode(PrivilegeMode::Machine);
  setVirtualMode(false);

  // If hart is configured to jump to a special target on enetering debug mode, then set
  // the pc to that target.
  if (debugParkLoop_ != ~URV(0))
    {
      pc_ = debugParkLoop_;
      inDebugParkLoop_ = true;
    }
}


template <typename URV>
void
Hart<URV>::enterDebugMode(URV pc)
{
  // This method is used by the test-bench to make the simulator follow it into
  // debug-mode. Do nothing if the simulator got into debug-mode on its own.
  if (debugMode_)
    return;   // Already in debug mode.

  enterDebugMode_(DebugModeCause::HALTREQ, pc);
}


template <typename URV>
void
Hart<URV>::exitDebugMode()
{
  if (not debugMode_)
    {
      std::cerr << "Warning: Bench sent exit debug while not in debug mode.\n";
      return;
    }

  if (cancelLrOnDebug_)
    cancelLr(CancelLrCause::EXIT_DEBUG);  // Lose LR reservation.

  pc_ = peekCsr(CsrNumber::DPC);  // Restore PC

  debugMode_ = false;
  inDebugParkLoop_ = false;
  csRegs_.enterDebug(false);

  updateCachedTriggerState();

  URV dcsrVal = 0;
  if (not peekCsr(CsrNumber::DCSR, dcsrVal))
    std::cerr << "Warning: Failed to read DCSR in exit debug.\n";

  DcsrFields<URV> dcsrf(dcsrVal);

  // Restore privilege mode.
  auto pm = PrivilegeMode{dcsrf.bits_.PRV};
  setPrivilegeMode(pm);

  // Restore virtual mode.
  bool vm = dcsrf.bits_.V;
  setVirtualMode(vm);

  // Restore ELP.
  if (isRvZicfilp())
    setElp(isLandingPadEnabled(pm, vm)? dcsrf.bits_.PELP : false);
}


template <typename URV>
void
Hart<URV>::execBlt(const DecodedInst* di)
{
  SRV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 < v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvzca() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execBltu(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 < v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvzca() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execBge(const DecodedInst* di)
{
  SRV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 >= v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvzca() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execBgeu(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op0()),  v2 = intRegs_.read(di->op1());
  if (v1 >= v2)
    {
      URV nextPc = (currPc_ + di->op2As<SRV>()) & ~URV(1);
      if (not isRvzca() and (nextPc & 3))
	{
	  // Target must be word aligned if C is off.
	  initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
	}
      else
	{
	  setPc(nextPc);
	  lastBranchTaken_ = true;
	}
    }
}


template <typename URV>
void
Hart<URV>::execJalr(const DecodedInst* di)
{
  URV temp = pc_;  // pc has the address of the instruction after jalr

  URV nextPc = (intRegs_.read(di->op1()) + di->op2As<SRV>()) & ~URV(1);
  if (not isRvzca() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      setPc(nextPc);
      intRegs_.write(di->op0(), temp);
      lastBranchTaken_ = true;
      if (isRvZicfilp())
        {
          if (isLandingPadEnabled(privMode_, virtMode_))
            setElp((di->op1() != 1) and (di->op1() != 5) and (di->op1() != 7));
        }
    }
}


template <typename URV>
void
Hart<URV>::execJal(const DecodedInst* di)
{
  URV nextPc = (currPc_ + SRV(int32_t(di->op1()))) & ~URV(1);
  if (not isRvzca() and (nextPc & 3))
    {
      // Target must be word aligned if C is off.
      initiateException(ExceptionCause::INST_ADDR_MISAL, currPc_, nextPc);
    }
  else
    {
      intRegs_.write(di->op0(), pc_);
      setPc(nextPc);
      lastBranchTaken_ = true;
    }
}


template <typename URV>
void
Hart<URV>::execAuipc(const DecodedInst* di)
{
  intRegs_.write(di->op0(), currPc_ + SRV(int32_t(di->op1())));
}


template <typename URV>
inline
bool
Hart<URV>::checkShiftImmediate(const DecodedInst* di, URV imm)
{
  bool bad = isRv64()? imm > 63 : imm > 31;

  if (bad)
    {
      illegalInst(di);
      return false;
    }
  return true;
}


template <typename URV>
void
Hart<URV>::execSlli(const DecodedInst* di)
{
  URV amount = di->op2();
  if (not checkShiftImmediate(di, amount))
    return;

  URV v = intRegs_.read(di->op1()) << amount;
  intRegs_.write(di->op0(), v);

  // Start of semi-hosting sequence: See section 2.8 (ebreak) of unprivileged spec.
  if (isSemihostSlli(di))
    {
      endSemihostSeq();  // In case one was active.
      startSemihostSeq(currPc_);
    }
}


template <typename URV>
void
Hart<URV>::execSlti(const DecodedInst* di)
{
  SRV imm = di->op2As<SRV>();
  URV v = SRV(intRegs_.read(di->op1())) < imm ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSltiu(const DecodedInst* di)
{
  URV imm = di->op2As<SRV>();   // We sign extend then use as unsigned.
  URV v = URV(intRegs_.read(di->op1())) < imm ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execXori(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) ^ di->op2As<SRV>();
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSrli(const DecodedInst* di)
{
  URV amount(di->op2());
  if (not checkShiftImmediate(di, amount))
    return;

  URV v = intRegs_.read(di->op1());
  v >>= amount;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSrai(const DecodedInst* di)
{
  uint32_t amount(di->op2());
  if (not checkShiftImmediate(di, amount))
    return;

  URV val = SRV(intRegs_.read(di->op1())) >> amount;

  // End of semi-hosting sequence: See section 2.8 (ebreak) of unprivileged spec.
  if (isSemihostSrai(di))
    {
      URV a0 = peekIntReg(RegA0);
      URV a1 = peekIntReg(RegA1);
      a0 = syscall_.emulateSemihost(hartIx_, a0, a1);
      intRegs_.write(RegA0, a0);
    }
  else
    intRegs_.write(di->op0(), val);

  endSemihostSeq();
}


template <typename URV>
void
Hart<URV>::execOri(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) | di->op2As<SRV>();
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSub(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) - intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSll(const DecodedInst* di)
{
  URV mask = shiftMask();
  URV v = intRegs_.read(di->op1()) << (intRegs_.read(di->op2()) & mask);
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSlt(const DecodedInst* di)
{
  SRV v1 = intRegs_.read(di->op1());
  SRV v2 = intRegs_.read(di->op2());
  URV v = v1 < v2 ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSltu(const DecodedInst* di)
{
  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV v = v1 < v2 ? 1 : 0;
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execXor(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) ^ intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSrl(const DecodedInst* di)
{
  URV mask = shiftMask();
  URV v = intRegs_.read(di->op1());
  v >>= (intRegs_.read(di->op2()) & mask);
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execSra(const DecodedInst* di)
{
  URV mask = shiftMask();
  URV v = SRV(intRegs_.read(di->op1())) >> (intRegs_.read(di->op2()) & mask);
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execOr(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) | intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execAnd(const DecodedInst* di)
{
  URV v = intRegs_.read(di->op1()) & intRegs_.read(di->op2());
  intRegs_.write(di->op0(), v);
}


template <typename URV>
void
Hart<URV>::execFence(const DecodedInst*)
{
}


template <typename URV>
void
Hart<URV>::execFence_tso(const DecodedInst* di)
{
  // Only fence_tso rw,rw is legal.

  if ( di->isFencePredRead() and
       di->isFencePredWrite() and
       di->isFenceSuccRead() and
       di->isFenceSuccWrite() and
       not di->isFencePredInput() and
       not di->isFencePredOutput() and
       not di->isFenceSuccInput() and
       not di->isFenceSuccOutput() )
    return;

  // Spec says that reserved configurations should be treated as normal FENCE.
  // We do not take an exception.
}


template <typename URV>
void
Hart<URV>::execFencei(const DecodedInst* di)
{
  if (not extensionIsEnabled(RvExtension::Zifencei))
    {
      illegalInst(di);
      return;
    }

  if (mcm_ and fetchCache_ and not coherentIcache_)
    fetchCache_->clear();

  if (traceCacheOn_)
    traceCache(0, 0, 0, false, false, false, true, false);

  // invalidateDecodeCache();  // No need for this. We invalidate on each write.
}


template <typename URV>
void
Hart<URV>::execEcall(const DecodedInst*)
{
  if (breakpOrEnterDebugTripped())
    return;

  if (newlib_ or linux_)
    {

      unsigned sysReg = isRve() ? RegT0 : RegA7; // Reg containting syscall number.
      URV sysIx = peekIntReg(sysReg);  // Syscall number.

      URV a0 = peekIntReg(RegA0);  // Syscall params.
      URV a1 = peekIntReg(RegA1);
      URV a2 = peekIntReg(RegA2);
      URV a3 = peekIntReg(RegA3);

      a0 = syscall_.emulate(hartIx_, sysIx, a0, a1, a2, a3);
      intRegs_.write(RegA0, a0);
      return;
    }

  if (privMode_ == PrivilegeMode::Machine)
    initiateException(ExceptionCause::M_ENV_CALL, currPc_, 0);
  else if (privMode_ == PrivilegeMode::Supervisor)
    {
      auto ec = (virtMode_)? ExceptionCause::VS_ENV_CALL : ExceptionCause::S_ENV_CALL;
      initiateException(ec, currPc_, 0);
    }
  else if (privMode_ == PrivilegeMode::User)
    initiateException(ExceptionCause::U_ENV_CALL, currPc_, 0);
  else
    assert(0 and "Invalid privilege mode in execEcall");
}


template <typename URV>
void
Hart<URV>::execEbreak(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  // If semi-hosting was initiated by a special slli, then the ebreak follows that slli,
  // the the ebreak is a part of that sequence and not a debugger break. See section 2.8
  // (ebreak) of unprivileged spec.
  if (isSemihostEbreak(di))
    return;
  endSemihostSeq();

  if (enableGdb_)
    {
      setPc(currPc_);
      handleExceptionForGdb(*this, gdbInputFd_);
      return;
    }

  URV dcsrVal = 0;
  bool hasDcsr = peekCsr(CsrNumber::DCSR, dcsrVal);

  auto dmCause = DebugModeCause::EBREAK;

  if (inDebugParkLoop_)
    {
      pc_ = debugParkLoop_;
      ebreakInstDebug_ = true;         // Avoid incrementing MINSTRET
      return;
      if (hasDcsr)
	{
	  DcsrFields<URV> fields{dcsrVal};
	  fields.bits_.CAUSE = unsigned(DebugModeCause::HALTREQ);
	  csRegs_.poke(CsrNumber::DCSR, fields.value_);
	  csRegs_.recordWrite(CsrNumber::DCSR);
	  pc_ = debugParkLoop_;
	  return;
	}
    }

  // If in M/S/U privilege mode and DCSR bit ebreakm/s/u is set, then enter debug
  // mode. Same if in VS/VU mode and DCSR bit ebreakvs/vu set.
  if (hasDcsr)
    {
      DcsrFields<URV> fields{dcsrVal};
      bool ebm  = fields.bits_.EBREAKM;  // Ebreak in M-privilege enabled.
      bool ebs  = fields.bits_.EBREAKS;
      bool ebu  = fields.bits_.EBREAKU;
      bool ebvs = fields.bits_.EBREAKVS;
      bool ebvu = fields.bits_.EBREAKVU;

      using PM = PrivilegeMode;

      bool hit = ( (ebm and privMode_ == PM::Machine) or
		   (ebs and privMode_ == PM::Supervisor) or
		   (ebu and privMode_ == PM::User) );

      hit = hit or ( virtMode_ and ( (ebvs and privMode_ == PM::Supervisor) or
                                     (ebvu and privMode_ == PM::User) ) );

      // Should we do if we are debug-mode single stepping?
      if (hit)
        {
          // The documentation (RISCV external debug support) does not say whether or not
          // we set EPC and MTVAL.
          enterDebugMode_(dmCause, currPc_);
          ebreakInstDebug_ = true;         // Avoid incrementing MINSTRET
          recordCsrWrite(CsrNumber::DCSR);
          return;
        }
    }

  URV savedPc = currPc_;  // Goes into MEPC.
  URV trapInfo = currPc_;  // Goes into MTVAL.

  if (clearMtvalOnEbreak_)
    trapInfo = 0;

  lastEbreak_ = true;
  initiateException(ExceptionCause::BREAKP, savedPc, trapInfo);
  lastEbreak_ = false;
}


template <typename URV>
void
Hart<URV>::execSfence_vma(const DecodedInst* di)
{
  if (not isRvs())
    {
      illegalInst(di);
      return;
    }

  if (privMode_ == PrivilegeMode::User)
    {
      if (virtMode_)
	virtualInst(di);  // VU mode.
      else
	illegalInst(di);
      return;
    }

  bool tvm = virtMode_ ? hstatus_.bits_.VTVM : mstatus_.bits_.TVM;
  if (tvm and privMode_ == PrivilegeMode::Supervisor)
    {
      if (virtMode_)
	virtualInst(di);
      else
	illegalInst(di);
      return;
    }

  auto& tlb = virtMode_ ? virtMem_.vsTlb_ : virtMem_.tlb_;
  auto vmid = virtMem_.vmid();
  uint32_t wid = steeEnabled_? stee_.secureWorld() : 0;

  if (di->op0() == 0 and di->op1() == 0)
    {
      if (virtMode_)
        tlb.invalidateVmid(vmid, wid);
      else
        tlb.invalidate();
    }
  else if (di->op0() == 0 and di->op1() != 0)
    {
      URV asid = intRegs_.read(di->op1());
      if (virtMode_)
        tlb.invalidateAsidVmid(asid, vmid, wid);
      else
        tlb.invalidateAsid(asid, wid);
    }
  else if (di->op0() != 0 and di->op1() == 0)
    {
      URV addr = intRegs_.read(di->op0());
      uint64_t vpn = virtMem_.pageNumber(addr);
      if (virtMode_)
        tlb.invalidateVirtualPageVmid(vpn, vmid, wid);
      else
        tlb.invalidateVirtualPage(vpn, wid);
    }
  else
    {
      URV addr = intRegs_.read(di->op0());
      uint64_t vpn = virtMem_.pageNumber(addr);
      URV asid = intRegs_.read(di->op1());

      if (virtMode_)
        tlb.invalidateVirtualPageAsidVmid(vpn, asid, vmid, wid);
      else
        tlb.invalidateVirtualPageAsid(vpn, asid, wid);
    }

  // The page-walk PTE cache is a translation cache too: flush it on sfence.vma.
  virtMem_.flushPteCache();

#if 0
  if (mcm_)
    fetchCache_->clear();

  if (di->op0() == 0)
    invalidateDecodeCache();
  else
    {
      uint64_t va = intRegs_.read(di->op0());
      uint64_t pageStart = virtMem_.pageStartAddress(va);
      uint64_t last = pageStart + virtMem_.pageSize();
      for (uint64_t addr = pageStart; addr < last; addr += 4)
        invalidateDecodeCache(addr, 4);
    }
#endif
}


template <typename URV>
void
Hart<URV>::execSinval_vma(const DecodedInst* di)
{
  if (not isRvs() or not isRvsvinval() or privMode_ < PrivilegeMode::Supervisor)
   {
      if (virtMode_ and privMode_ == PrivilegeMode::User)
	virtualInst(di);
      else
	illegalInst(di);
      return;
   }
  execSfence_vma(di);
}


template <typename URV>
void
Hart<URV>::execSfence_w_inval(const DecodedInst* di)
{
  if (not isRvs() or not isRvsvinval() or privMode_ < PrivilegeMode::Supervisor)
    {
      if (virtMode_ and privMode_ == PrivilegeMode::User)
	virtualInst(di);
      else
	illegalInst(di);
      return;
    }
}


template <typename URV>
void
Hart<URV>::execSfence_inval_ir(const DecodedInst* di)
{
  if (not isRvs() or not isRvsvinval() or privMode_ < PrivilegeMode::Supervisor)
    {
      if (virtMode_ and privMode_ == PrivilegeMode::User)
	virtualInst(di);
      else
	illegalInst(di);
      return;
    }
}


namespace WdRiscv
{

  template <>
  void
  Hart<uint64_t>::execMret(const DecodedInst* di)
  {
    if (privMode_ < PrivilegeMode::Machine)
      {
	illegalInst(di);
	return;
      }

    if (breakpOrEnterDebugTripped())
      return;

    if (sdtrigOn_)
      csRegs_.restoreTcontrolMte();

    // 1. Restore privilege mode, interrupt enable, and virtual mode.
    uint64_t value = csRegs_.peekMstatus();

    MstatusFields<uint64_t> fields(value);
    auto savedMode = PrivilegeMode(fields.bits_.MPP);
    bool savedVirt = fields.bits_.MPV;

    // 1.1 Restore MIE.
    fields.bits_.MIE = fields.bits_.MPIE;

    // 1.1. Set MPP to the least privileged mode available.
    if (isRvu())
      fields.bits_.MPP = unsigned(PrivilegeMode::User);
    else if (isRvs())
      fields.bits_.MPP = unsigned(PrivilegeMode::Supervisor);
    else
      fields.bits_.MPP = unsigned(PrivilegeMode::Machine);

    // 1.2. Set MPIE.
    fields.bits_.MPIE = 1;

    // 1.3. Clear MPRV.
    if (savedMode != PrivilegeMode::Machine and clearMprvOnRet_)
      fields.bits_.MPRV = 0;

    // 1.4. Clear virtual (V) mode.
    fields.bits_.MPV = 0;

    // 1.5. Restore ELP.
    if (isRvZicfilp())
      {
        setElp(isLandingPadEnabled(savedMode, savedVirt)? fields.bits_.MPELP : false);
        fields.bits_.MPELP = false;
      }

    // 1.6. Smdbltrp: MRET clears MDT (spec machine.html line 1409-1411:
    // "The MRET and SRET instructions, when executed in M-mode, set the
    //  MDT bit to 0.")
    if (isRvsmdbltrp())
      fields.bits_.MDT = 0;

    // 1.6b. Ssdbltrp: MRET to U-mode (or VS/VU in H-mode) clears sstatus.SDT.
    // Spec supervisor.adoc: "When MRET or SRET are executed in M-mode and the
    // new privilege mode is U, VS, or VU, sstatus.SDT is also set to 0."
    if (isRvssdbltrp() and savedMode < PrivilegeMode::Supervisor)
      fields.bits_.SDT = 0;

    // 1.7. Write back MSTATUS.
    if (not csRegs_.write(CsrNumber::MSTATUS, privMode_, fields.value_))
      assert(0 and "Failed to write MSTATUS register\n");
    updateCachedMstatus();

    // Smnip: on mret, restore mithreshold from mistatus.pithreshprio (9-bit, [16:8]).
    if (extensionIsEnabled(RvExtension::Smnip) and aclic_ and aclic_->isMnipEnabled())
      {
        uint64_t misVal = 0;
        [[maybe_unused]] bool ok = csRegs_.peek(CsrNumber::MISTATUS, misVal);
        assert(ok);
        auto pithresh = static_cast<uint16_t>((misVal >> 8) & 0x1FF);
        aclic_->setMithreshold(pithresh);
        csRegs_.poke(CsrNumber::MITHRESHOLD, uint64_t(aclic_->getMithreshold()));
      }

    // 2. Restore program counter from MEPC.
    uint64_t epc = 0;
    if (not csRegs_.readSignExtend(CsrNumber::MEPC, privMode_, epc))
      assert(0 && "Error: Assertion failed");
    setPc(epc);

    // 3. Update virtual mode.
    if (savedMode != PrivilegeMode::Machine)
      setVirtualMode(savedVirt);

    // 4. Update privilege mode.
    setPrivilegeMode(savedMode);
  }


  // SV32 version of execMret has to contend with MSTATUSH.
  template <>
  void
  Hart<uint32_t>::execMret(const DecodedInst* di)
  {
    if (privMode_ < PrivilegeMode::Machine)
      {
	illegalInst(di);
	return;
      }

    if (breakpOrEnterDebugTripped())
      return;

    // 1. Restore privilege mode, interrupt enable, and virtual mode.
    uint32_t value = csRegs_.peekMstatus();
    uint32_t hvalue = peekCsr(CsrNumber::MSTATUSH);
    bool savedVirt = (hvalue >> 7) & 1;

    MstatusFields<uint32_t> fields(value);
    auto savedMode = PrivilegeMode(fields.bits_.MPP);
    fields.bits_.MIE = fields.bits_.MPIE;

    // 1.1. Set MPP to the least privileged mode available.
    if (isRvu())
      fields.bits_.MPP = unsigned(PrivilegeMode::User);
    else if (isRvs())
      fields.bits_.MPP = unsigned(PrivilegeMode::Supervisor);
    else
      fields.bits_.MPP = unsigned(PrivilegeMode::Machine);

    // 1.2. Enable interrupts.
    fields.bits_.MPIE = 1;

    // 1.3. Clear MPRV.
    if (savedMode != PrivilegeMode::Machine and clearMprvOnRet_)
      fields.bits_.MPRV = 0;

    // 1.4. Clear virtual (V) mode.
    hvalue &= ~ uint32_t(1 << 7);

    // 1.5. Restore ELP.
    if (isRvZicfilp())
      {
        setElp(isLandingPadEnabled(savedMode, savedVirt)? ((hvalue >> 9) & 1) : false);
        hvalue &= ~ uint32_t(1 << 9);
      }

    // 1.6. Smdbltrp: MRET clears MDT (spec machine.html line 1409-1411).
    // For RV32, MDT is bit 10 of mstatush (bit 42 overall = 42 - 32 = 10).
    if (isRvsmdbltrp())
      hvalue &= ~ uint32_t(1 << 10);

    // 1.7. Write back MSTATUS.
    if (not csRegs_.write(CsrNumber::MSTATUS, privMode_, fields.value_))
      assert(0 and "Failed to write MSTATUS register\n");
    if (not csRegs_.write(CsrNumber::MSTATUSH, privMode_, hvalue))
      assert(0 and "Failed to write MSTATUSH register\n");
    updateCachedMstatus();

    // Smnip: on mret, restore mithreshold from mistatus.pithreshprio (9-bit, [16:8]).
    if (extensionIsEnabled(RvExtension::Smnip) and aclic_ and aclic_->isMnipEnabled())
      {
        uint32_t misVal = 0;
        [[maybe_unused]] bool ok = csRegs_.peek(CsrNumber::MISTATUS, misVal);
        assert(ok);
        auto pithresh = static_cast<uint16_t>((misVal >> 8) & 0x1FF);
        aclic_->setMithreshold(pithresh);
        csRegs_.poke(CsrNumber::MITHRESHOLD, uint32_t(aclic_->getMithreshold()));
      }

    // 2. Restore program counter from MEPC.
    uint32_t epc = 0;
    if (not csRegs_.read(CsrNumber::MEPC, privMode_, epc))
      illegalInst(di);
    setPc(epc);

    // 3. Update virtual mode.
    if (savedMode != PrivilegeMode::Machine)
      setVirtualMode(savedVirt);

    // 4. Update privilege mode.
    setPrivilegeMode(savedMode);
  }
}


template <typename URV>
void
Hart<URV>::execSret(const DecodedInst* di)
{
  if (not isRvs())
    {
      illegalInst(di);
      return;
    }

  if (privMode_ < PrivilegeMode::Supervisor)
    {
      if (virtMode_)
	virtualInst(di);
      else
	illegalInst(di);
      return;
    }

  // If MSTATUS.TSR is 1 then sret is illegal in supervisor mode.
  bool tsr = virtMode_? hstatus_.bits_.VTSR : mstatus_.bits_.TSR;
  URV mstatus = csRegs_.peekMstatus();
  MstatusFields<URV> mfields(mstatus);
  if (tsr and privMode_ == PrivilegeMode::Supervisor)
    {
      if (virtMode_)
	virtualInst(di);
      else
	illegalInst(di);
      return;
    }

  if (breakpOrEnterDebugTripped())
    return;

  // Restore privilege mode and interrupt enable by getting
  // current value of SSTATUS, ...
  URV value = 0;
  if (not csRegs_.read(CsrNumber::SSTATUS, privMode_, value))
    {
      illegalInst(di);
      return;
    }

  // ... updating/unpacking its fields,
  MstatusFields<URV> fields(value);
  PrivilegeMode savedMode = fields.bits_.SPP? PrivilegeMode::Supervisor : PrivilegeMode::User;
  bool savedVirt = hstatus_.bits_.SPV;

  // Restore SIE.
  fields.bits_.SIE = fields.bits_.SPIE;

  // Set SPP.
  if (isRvu())
    fields.bits_.SPP = 0; // User mode
  else
    fields.bits_.SPP = 1; // Supervisor mode

  // Set SPIE
  fields.bits_.SPIE = 1;

  // Set ELP.
  if (isRvZicfilp())
    {
      setElp(isLandingPadEnabled(savedMode, savedVirt)? fields.bits_.SPELP : false);
      fields.bits_.SPELP = 0;
    }

  // ... and putting it back
  if (not csRegs_.write(CsrNumber::SSTATUS, privMode_, fields.value_))
    assert(0 && "Error: Assertion failed");

  // Clear MPRV
  if (savedMode != PrivilegeMode::Machine and clearMprvOnRet_)
    csRegs_.poke(CsrNumber::SSTATUS, fields.value_, virtMode_);

  updateCachedSstatus();

  // Ssnip: on sret, restore sithreshold from sistatus.pithreshprio (9-bit, [16:8]).
  if (extensionIsEnabled(RvExtension::Ssnip) and aclic_ and aclic_->isSnipEnabled())
    {
      URV sisVal = 0;
      [[maybe_unused]] bool ok = csRegs_.peek(CsrNumber::SISTATUS, sisVal);
      assert(ok);
      auto pithresh = static_cast<uint16_t>((sisVal >> 8) & 0x1FF);
      aclic_->setSithreshold(pithresh);
      csRegs_.poke(CsrNumber::SITHRESHOLD, URV(aclic_->getSithreshold()));
    }

  // Smdbltrp: SRET when executed in M-mode clears MDT (spec machine.html
  // line 1409-1411: "The MRET and SRET instructions, when executed in
  // M-mode, set the MDT bit to 0.")
  // Note: privMode_ here is the mode BEFORE the SRET (still M-mode if
  // this SRET was executed from M-mode).
  if (isRvsmdbltrp() and privMode_ == PrivilegeMode::Machine)
    {
      mstatus_.bits_.MDT = 0;
      writeMstatus();
    }

  // Ssdbltrp: SRET clears sstatus.SDT (supervisor.adoc §SRET).
  if (isRvssdbltrp())
    {
      mstatus_.bits_.SDT = 0;
      writeMstatus();
    }

  // Clear hstatus.spv if sret executed in M/S modes.
  if (not virtMode_ and savedVirt)
    {
      hstatus_.bits_.SPV = 0;
      if (not csRegs_.write(CsrNumber::HSTATUS, privMode_, hstatus_.value_))
	assert(0 && "Error: Assertion failed");
    }

  // Restore program counter from SEPC.
  URV epc;
  if (not csRegs_.read(CsrNumber::SEPC, privMode_, epc))
    {
      illegalInst(di);
      return;
    }
  setPc(epc);

  // Update virtual mode.
  if (not virtMode_)
    setVirtualMode(savedVirt);

  // Update privilege mode.
  setPrivilegeMode(savedMode);
}


template <typename URV>
void
Hart<URV>::execMnret(const DecodedInst* di)
{
  if (not extensionIsEnabled(RvExtension::Smrnmi) or
      privMode_ < PrivilegeMode::Machine)
    {
      illegalInst(di);
      return;
    }

  if (breakpOrEnterDebugTripped())
    return;

  // Recover privilege mode and virtual mode.
  MnstatusFields mnf{csRegs_.peekMnstatus()};
  auto savedMode = PrivilegeMode{mnf.bits_.MNPP};
  bool savedVirt = mnf.bits_.MNPV;

  mnf.bits_.NMIE = 1;  // Set mnstatus.mnie
  pokeCsr(CsrNumber::MNSTATUS, mnf.value_);
  recordCsrWrite(CsrNumber::MNSTATUS);

  // Restore PC
  URV epc = 0;
  csRegs_.read(CsrNumber::MNEPC, privMode_, epc);
  setPc(epc);

  // Restore virtual mode
  if (savedMode != PrivilegeMode::Machine)
    {
      setVirtualMode(savedVirt);

      // Smdbltrp: MNRET sets MDT to 0 if the new privilege mode is not M
      // (spec machine.html lines 1414-1416: "The MNRET instruction, provided
      //  by the Smrnmi extension, sets the MDT bit to 0 if the new privilege
      //  mode is not M.")
      bool needWrite = false;
      if (isRvsmdbltrp() and mstatus_.bits_.MDT)
	{
	  mstatus_.bits_.MDT = 0;
	  needWrite = true;
	}
      if (mstatus_.bits_.MPRV != 0)
	{
	  mstatus_.bits_.MPRV = 0;
	  needWrite = true;
	}
      if (needWrite)
	writeMstatus();
    }

  // Restore privilege mode
  setPrivilegeMode(savedMode);
}


template <typename URV>
void
Hart<URV>::execWfi(const DecodedInst* di)
{
#if 1

  // Remove when RTL is ready.

  using PM = PrivilegeMode;
  auto pm = privilegeMode();

  if (pm == PM::Machine)
    return;

  if (mstatus_.bits_.TW)
    {
      // TW is 1 and Executing in privilege less than machine: illegal unless
      // complete in bounded time.
      if (wfiTimeout_ == 0)
	illegalInst(di);
      return;
    }

  // TW is 0.
  if (pm == PM::User and isRvs())
    {
      if (virtMode_)
	virtualInst(di);   // VU mode and TW=0. Section 9.6 of privilege spec.
      else if (wfiTimeout_ == 0)
	illegalInst(di);
      return;
    }


  // VS mode, VTW=1 and mstatus.TW=0
  if (virtMode_ and pm == PM::Supervisor and hstatus_.bits_.VTW)
    {
      if (wfiTimeout_ == 0)
	virtualInst(di);
      return;
    }

#else

  // Enable when RTL is ready.

  // If running standalone, we assume that the WFI timeout (if any) has expired. If
  // running with an external agent (e.g. test-bench), we assume that the agent will poke
  // MIP with an interrupt (if any) before we get here so by the time we get here the
  // wfi timeout has expired.

  using PM = PrivilegeMode;

  auto pm = privilegeMode();

  if (pm == PM::Machine)
    return;

  bool tw = mstatus_.bits_.TW;
  bool vtw = hstatus_.bits_.VTW;

  if (not virtMode_)
    {
      if (pm == PM::Supervisor and not tw)
	return;
      illegalInst(di);   // Supervisor or User mode. Timeout expired.
      return;
    }

  if (pm == PM::Supervisor)   // VS mode
    {
      if (not vtw and not tw)
	return;
      if (vtw and not tw)
	virtualInst(di);
      else if (tw)
	illegalInst(di);
      return;
    }

  // VU mode.
  if (tw)
    illegalInst(di);
  else
    virtualInst(di);

#endif
}


template <typename URV>
void
Hart<URV>::execDret(const DecodedInst* di)
{
  // The dret instruction is only valid if debug is on.
  if (not debugMode_)
    {
      illegalInst(di);
      return;
    }

  exitDebugMode();
}


template <typename URV>
bool
Hart<URV>::checkCsrAccess(const DecodedInst* di, CsrNumber csr, bool isWrite)
{
  using PM = PrivilegeMode;
  using CN = CsrNumber;

  auto uMode = privMode_ == PM::User;

  // Sscsrind implements siselect/sireg* without Smaia; Smcdeleg checks must still run.
  if (csRegs_.isAia(csr))
    {
      if (csRegs_.isHypervisor(csr) and not isRvh())
        {
          illegalInst(di);
          return false;
        }

      if (privMode_ != PM::Machine)
        {
          auto mappedCsr = csRegs_.getImplementedCsr(csr, virtMode_);
          auto csrn = mappedCsr? mappedCsr->getNumber() : csr;

          if (virtMode_)
            {
              // Section 5.5 of privileged spec. If CSRIND is 1, VS/VU access to
              // vsiselect/vsireg and VU access to sireg should ignore other stateen bits.
              if (csr == CN::VSIREG or csr == CN::VSISELECT or
                  (uMode and (csr == CN::SIREG or csr == CN::SISELECT)))
                {
                  auto mstateen0 = csRegs_.read64(CN::MSTATEEN0);
                  Mstateen0Fields fields{mstateen0};
                  if (fields.bits_.CSRIND)
                    {
                      virtualInst(di);
                      return false;
                    }
                }

              // Sec 5.5 of priv spec. If the hypervisor extension is implemented, the
              // same bit is defined also in hypervisor CSR hstateen0, but controls access
              // to only siselect and sireg* (really vsiselect and vsireg*), which is the
              // state potentially accessible to a virtual machine executing in VS or
              // VU-mode. When hstateen0[60]=0 and mstateen0[60]=1, all attempts from VS
              // or VU-mode to access siselect or sireg* raise a virtual instruction
              // exception, not an illegal instruction exception, regardless of the value
              // of vsiselect or any other mstateen bit.
              if ((csr == CN::SIREG or csr == CN::SISELECT))
                {
                  auto hse0 = csRegs_.read64(CN::HSTATEEN0);
                  auto mse0 = csRegs_.read64(CN::MSTATEEN0);
                  if (Mstateen0Fields{hse0}.bits_.CSRIND == 0 and Mstateen0Fields{mse0}.bits_.CSRIND == 1)
                    {
                      virtualInst(di);
                      return false;
                    }
                }
            }

          // Section 2.5 of AIA. Check if MSTATEEN disallows access.
          if (not csRegs_.isStateEnabled(csrn, PM::Machine, false /*virtMode_*/))
            {
              illegalInst(di);  // Not enabled in MSTATEEN.
              return false;
            }

          // Smcdeleg.
          // Furthermore, while vsiselect holds a value in the range 0x40-0x5F:
          // ⚫ An attempt to access any vsireg* from M or S mode raises an
          //   illegal-instruction exception.
          // ⚫ An attempt from VS-mode to access any sireg* (really vsireg*) raises an
          //   illegal-instruction exception if menvcfg.CDE = 0, or a virtual-instruction
          //   exception if menvcfg.CDE = 1.
          if (auto vsisel = csRegs_.getImplementedCsr(CN::VSISELECT); vsisel)
            {
              auto sel = vsisel->read();
              if (sel >= 0x40 and sel <= 0x5f)
                {
                  bool isVsireg = csr >= CN::VSIREG and csr <= CN::VSIREG6;
                  bool sorm = privMode_ == PM::Machine or (privMode_ == PM::Supervisor and not virtMode_);
                  if (isVsireg and sorm)
                    {
                      illegalInst(di);
                      return false;
                    }
                  bool isSireg = csr >= CN::SIREG and csr <= CN::SIREG6;
                  bool vs = privMode_ == PM::Supervisor and virtMode_;
                  if (isSireg and vs)
                    {
                      if (csRegs_.menvcfgCde())
                        virtualInst(di);
                      else
                        illegalInst(di);
                      return false;
                    }
                }
            }

          // Section 2.5 of AIA. Check if MSTATEEN/HSTATEEN allow access.
          if (csRegs_.stateenOn_ and virtMode_ and (csr == CN::SIREG or csr == CN::SISELECT))
            {
              auto hstateen0 = csRegs_.read64(CsrNumber::HSTATEEN0);
              Mstateen0Fields fields{hstateen0};
              if (not fields.bits_.CSRIND)
                {
                  virtualInst(di);  // Bit 60 (CSRIND) 1 in MSTATEEN0, 0 in HSTATEEN0
                  return false;
                }
            }
        }
    }

  // Smcdeleg: M-mode access to vsireg* while vsiselect is in 0x40-0x5F must raise
  // illegal-instruction. The inner AIA block above only runs for non-Machine modes.
  if (csRegs_.smcdelegOn())
    {
      bool isVsireg = csr >= CN::VSIREG and csr <= CN::VSIREG6;
      if (isVsireg and privMode_ == PM::Machine)
        {
          if (auto vsisel = csRegs_.getImplementedCsr(CN::VSISELECT); vsisel)
            {
              auto sel = vsisel->read();
              if (sel >= 0x40 and sel <= 0x5f)
                {
                  illegalInst(di);
                  return false;
                }
            }
        }
    }

  // Check if HS qualified (section 9.6.1 of privileged spec).
  bool hsq = isRvs() and csRegs_.isReadable(csr, PM::Supervisor, false /*virtMode*/);
  if (isWrite)
    hsq = hsq and isCsrWriteable(csr, PM::Supervisor, false /*virtMode*/);

  if (virtMode_)
    {
      if (isRvaia() and ((csr == CN::VSIREG or csr == CN::VSISELECT) or
                         (uMode and (csr == CN::SIREG or csr == CN::SISELECT))))
        {
          virtualInst(di);
          return false;  // Section 2.3 of interrupt spec and section 5.4 of of privileged spec.
        }
      if (csr >= CN::CYCLE and csr <= CN::HPMCOUNTER31 and not isWrite)
	{       // Section 9.2.6 of privileged spec.
	  URV hcounteren = 0, mcounteren = 0, scounteren = 0;
	  if (not peekCsr(CN::MCOUNTEREN, mcounteren) or not peekCsr(CN::HCOUNTEREN, hcounteren) or
	      not peekCsr(CN::SCOUNTEREN, scounteren))
	    assert(0 && "Error: Assertion failed");
	  unsigned bitIx = unsigned(csr) - unsigned(CN::CYCLE);
	  URV mask = URV(1) << bitIx;
          if ((mcounteren & mask) == 0)
            {
              illegalInst(di);
              return false;
            }
          if (((hcounteren & mask) == 0) or (uMode and (scounteren & mask) == 0))
	    {
	      virtualInst(di);
	      return false;
	    }
	}
      if (csr == CN::SEED and not isWrite)
        {
          illegalInst(di);
          return false;
        }
      if (csRegs_.isHypervisor(csr) or
          (uMode and not csRegs_.isReadable(csr, PM::User, virtMode_)))
	{
	  assert(not csRegs_.isHighHalf(csr) or sizeof(URV) == 4);
	  if (hsq)
	    virtualInst(di);
	  else
	    illegalInst(di);
	  return false;
        }
    }

  if (isWrite and not isCsrWriteable(csr, privMode_, virtMode_))
    {
      if (virtMode_)
	{
	  if (csr == CsrNumber::SATP)
	    virtualInst(di);
	  else
	    {
	      if (csRegs_.isHighHalf(csr) and sizeof(URV) > 4)
		hsq = false;
	      if (hsq) virtualInst(di); else illegalInst(di);
	    }
	}
      else
	illegalInst(di);
      return false;
    }


  // Section 2.3 of AIA, lower priority than stateen. Doesn't follow normal hs-qualified rules.
  if (isRvaia() and imsicTrap(di, csr, virtMode_))
    return false;

  if (csr == CN::SATP and privMode_ == PM::Supervisor)
    {
      if (mstatus_.bits_.TVM and not virtMode_)
	{
	  illegalInst(di);
	  return false;
	}
      if (hstatus_.bits_.VTVM and virtMode_)
	{
	  virtualInst(di);
	  return false;
	}
    }

  if (csr == CN::HGATP and privMode_ == PM::Supervisor and not virtMode_)
    if (mstatus_.bits_.TVM)
      {
	illegalInst(di);
	return false;
      }

  if (not isFpLegal())
    if (csr == CN::FCSR or csr == CN::FRM or csr == CN::FFLAGS)
      {
	illegalInst(di);
	return false;
      }

  if (not isVecLegal())
    if (csr == CN::VSTART or csr == CN::VXSAT or csr == CN::VXRM or csr == CN::VCSR
	or csr == CN::VL or csr == CN::VTYPE or csr == CN::VLENB)
      {
        illegalInst(di);
        return false;
      }

  if (csr == CN::SEED)
    {
      if (not isWrite)
        {
          illegalInst(di);
          return false;
        }
      if (privMode_ != PM::Machine)
        {
          bool sseed = false, useed = false;
          if (not csRegs_.mseccfgSeed(sseed, useed))
            return false;
          bool avail = (privMode_ == PM::User and not virtMode_)? useed : sseed;
          if (not avail)
            {
              if (virtMode_ and sseed) virtualInst(di); else illegalInst(di);
              return false;
            }
        }
    }
  
  return true;
}


template <typename URV>
bool
Hart<URV>::doCsrRead(const DecodedInst* di, CsrNumber csr, bool isWrite, URV& value)
{
  if (not checkCsrAccess(di, csr, isWrite))
    return false;

  if (csRegs_.read(csr, privMode_, value))
    return true;

  // Check if HS qualified (section 9.6.1 of privileged spec).
  using PM = PrivilegeMode;
  bool hsq = isRvs() and csRegs_.isReadable(csr, PM::Supervisor, false /*virtMode*/);
  if (isWrite)
    hsq = hsq and isCsrWriteable(csr, PM::Supervisor, false /*virtMode*/);

  if (virtMode_ and hsq )
    virtualInst(di);  // HS qualified
  else
    illegalInst(di);

  return false;
}


template <typename URV>
bool
Hart<URV>::imsicTrap(const DecodedInst* di, CsrNumber csr, bool virtMode)
{
  using CN = CsrNumber;
  using PM = PrivilegeMode;

  if (imsic_)
    {
      bool guestTopei = csr == CN::VSTOPEI or (csr == CN::STOPEI and virtMode);
      bool guestIreg  = csr == CN::VSIREG or (csr == CN::SIREG and virtMode);
      bool invalidVgein = not hstatus_.bits_.VGEIN or hstatus_.bits_.VGEIN >= imsic_->guestCount();

      if (guestTopei and invalidVgein)
        {
          if (virtMode)
            virtualInst(di);
          else
            illegalInst(di);
          return true;
	}

      if (csr == CN::MIREG or csr == CN::SIREG or csr == CN::VSIREG)
        {
          if (privMode_ == PM::User and not virtMode_)  // U mode
            {
              illegalInst(di);
              return true;
            }

          CN iselect = CsRegs<URV>::advance(csr, -1);
          if (guestIreg)
            iselect = CN::VSISELECT;

          URV sel = 0;
          if (not peekCsr(iselect, sel))
            {
              std::cerr << "Error: Failed to peek AIA select csr\n";
              return true;
            }

          bool isVs = (privMode_ == PM::Supervisor and virtMode_);  // VS mode
          bool isMhs = (privMode_ != PM::User and not virtMode_);   // M or HS mode

          /// Check if value in xISELECT is for imsic.
          bool imsicSel = csRegs_.isImsicSelectStrict(sel);

          if (TT_IMSIC::Imsic::isFileSelReserved(sel))
            {
              if (imsicSel and iselect == CN::MISELECT and csr == CN::MIREG)
                {
                  illegalInst(di);
                  return true;
                }
              if (imsicSel and iselect == CN::SISELECT and csr == CN::SIREG)
                {
                  illegalInst(di);
                  return true;
                }
              if (imsicSel and iselect == CN::VSISELECT)
                {
                  // Sec 2.3 of interrupt spec: attempts from M-mode or HS-mode to access
                  // vsireg, or from VS-mode to access sireg (really vsireg), should
                  // preferably raise an illegal instruction exception. This was in the
                  // 2023 version but was removed from the 2025 version implying that it
                  // became implementation dependent. We kept it.
                  if ((isMhs and csr == CN::VSIREG) or (isVs and csr == CN::SIREG))
                    illegalInst(di);
                  else
                    virtualInst(di);
                  return true;
                }
            }

          // Sec 2.3, accessing *ireg within a normally valid range with an invalid VGEIN
          // is deemed inaccessible.  The only other ranges are "reserved", which we
          // evaluate above.
          if (not TT_IMSIC::Imsic::isFileSelAccessible<URV>(sel, guestIreg) or
              (guestIreg and invalidVgein))
            {
              if (imsicSel and iselect == CN::MISELECT and csr == CN::MIREG)
                {
                  illegalInst(di);
                  return true;
                }
              if (imsicSel and iselect == CN::SISELECT and csr == CN::SIREG)
                {
                  illegalInst(di);
                  return true;
                }
              if (iselect == CN::VSISELECT)
                {
                  // Sec 2.3 of interrupt spec: attempts from M-mode or HS-mode to access
                  // vsireg raise an illegal instruction exception, and attempts from VS-mode
                  // to access sireg (really vsireg) raise a virtual instruction exception.
                  if (isVs and csr == CN::SIREG)
                    virtualInst(di);
                  else
                    illegalInst(di);  // Everything else including VSIREG in M/HS mode
                  return true;
                }
            }
        }

        // From section 5.3, When mvien.SEIP is set, 0x70-0xFF are reserved and stopei
        // are reserved from S-mode.
        bool isS = privMode_ == PM::Supervisor and not virtMode_;
        if (isS and (csr == CN::STOPEI or csr == CN::SIREG))
          {
            URV mvien = csRegs_.peekMvien();
            if ((mvien >> URV(InterruptCause::S_EXTERNAL)) & 1)
              {
                if (csr == CN::STOPEI)
                  {
                    illegalInst(di);
                    return true;
                  }

                // sireg
                CN iselect = CsRegs<URV>::advance(csr, -1);
                URV sel = 0;
                if (not peekCsr(iselect, sel))
                  {
                    std::cerr << "Error: Failed to peek AIA select csr\n";
                    return true;
                  }

                using EIC = TT_IMSIC::File::ExternalInterruptCsr;
                if (csRegs_.isImsicSelectStrict(sel) and sel >= EIC::DELIVERY and sel <= EIC::E63)
                  {
                    illegalInst(di);
                    return true;
                  }
              }
          }
    }
  else if (aclic_ and (csr == CN::MIREG or csr == CN::MTOPEI or
                        csr == CN::SIREG or csr == CN::STOPEI))
    {
      // No IMSIC, but ACLIC is present.  VSIREG/VSTOPEI are hypervisor CSRs not
      // used by ACLIC and remain illegal.
      if ((csr == CN::SIREG or csr == CN::STOPEI) and not aclic_->hasSupervisorDomain())
        {
          illegalInst(di);
          return true;
        }
      // For xireg, validate the selector is in an ACLIC-defined range.
      if (csr == CN::MIREG or csr == CN::SIREG)
        {
          CN iselect = CsRegs<URV>::advance(csr, -1);
          URV sel = 0;
          if (not peekCsr(iselect, sel))
            { illegalInst(di); return true; }
          bool validSel = (sel >= 0x80 and sel <= 0xFF) or (sel >= 0x1000 and sel <= 0x10FF);
          if (not validSel)
            { illegalInst(di); return true; }
        }
      // Valid ACLIC access — fall through to return true.
    }
  else if (csr == CN::MTOPEI or csr == CN::STOPEI or csr == CN::VSTOPEI or
           csr == CN::MIREG or csr == CN::SIREG or csr == CN::VSIREG)
    {
      illegalInst(di);
      return true;
    }

  return false;
}


template <typename URV>
bool
Hart<URV>::isCsrWriteable(CsrNumber csr, PrivilegeMode privMode, bool virtMode) const
{
  using PM = PrivilegeMode;

  if (virtMode)
    {
      if (csRegs_.isHypervisor(csr) or
	  (privMode == PM::User and not csRegs_.isWriteable(csr, PM::User, virtMode)))
	return false;
    }

  if (not csRegs_.isWriteable(csr, privMode, virtMode))
    return false;

  if (csr == CsrNumber::SATP and privMode == PrivilegeMode::Supervisor)
    {
      if (mstatus_.bits_.TVM and not virtMode)
	return false;
      if (hstatus_.bits_.VTVM and virtMode)
	return false;
      return true;
    }

  if (csr == CsrNumber::HGATP and privMode == PM::Supervisor and not virtMode)
    if (mstatus_.bits_.TVM)
      return false;

  if (not isFpLegal())
    if (csr == CsrNumber::FCSR or csr == CsrNumber::FRM or csr == CsrNumber::FFLAGS)
      return false;

  if (not isVecLegal())
    if (csr == CsrNumber::VSTART or csr == CsrNumber::VXSAT or csr == CsrNumber::VXRM or
	csr == CsrNumber::VCSR or csr == CsrNumber::VL or csr == CsrNumber::VTYPE or
	csr == CsrNumber::VLENB)
      return false;

  if ((csr == CsrNumber::STIMECMP or csr == CsrNumber::STIMECMPH) and virtMode)
    {
      URV val = 0;
      if (peekCsr(CsrNumber::HVICTL, val))
        {
          HvictlFields hvictl(val);
          if (hvictl.bits_.VTI)
            return false;
        }
    }

  if (csr == CsrNumber::SEED and privMode != PM::Machine)
    {
      bool sseed = false, useed = false;
      if (not csRegs_.mseccfgSeed(sseed, useed))
        return false;
      return (privMode == PM::User and not virtMode)? useed : sseed;
    }

  return true;
}


template <typename URV>
void
Hart<URV>::doCsrWrite(const DecodedInst* di, CsrNumber csr, URV val,
                      unsigned intReg, URV intRegVal)
{
  if (not checkCsrAccess(di, csr, true /* isWrite */))
    return;

  // Make auto-increment happen before CSR write for minstret and cycle.
  if (csr == CsrNumber::MINSTRET or csr == CsrNumber::MINSTRETH)
    if (minstretEnabled())
      minstret_++;
  if (csr == CsrNumber::MCYCLE or csr == CsrNumber::MCYCLEH)
    cycleCount_++;

  updatePerformanceCountersForCsr(*di);

  // Avoid updating MISA if update would turn off C and next pc is not 4-byte aligned.
  if (csr == CsrNumber::MISA and (pc_ & 3) != 0)
    {
      auto misa = csRegs_.getImplementedCsr(csr);
      URV cMask = URV(1) << ('c' - 'a');
      if (misa and (misa->getWriteMask() & cMask) and (val & cMask) == 0)
	return;  // Cannot turn-off C-extension if PC is not word aligned.
    }

  // Update integer register.
  intRegs_.write(intReg, intRegVal);

  // Legalize HGATP. We do this here to avoid making CsRegs depend on VirtMem.
  if (csr == CsrNumber::HGATP)
    {
      URV oldVal = 0;
      if (not peekCsr(csr, oldVal))
	oldVal = URV(VirtMem::Mode::Bare);  // Should not happen
      HgatpFields<URV> oldHgatp(oldVal);
      HgatpFields<URV> hgatp(val);
      auto mode = VirtMem::Mode{hgatp.bits_.MODE};
      if (not virtMem_.isModeSupported(mode))
	hgatp.bits_.MODE = oldHgatp.bits_.MODE;  // Preserve MODE field.
      val = hgatp.value_;
    }
  else if (csr == CsrNumber::SATP or csr == CsrNumber::VSATP)
    {
      unsigned modeBits = 0;
      if constexpr (sizeof(URV) == 4)
	modeBits = (val >> 31) & 1;
      else
	modeBits = (val >> 60) & 0xf;
      auto mode = VirtMem::Mode(modeBits);
      if (not virtMem_.isModeSupported(mode))
	return;  // Unsupported mode: Write has no effect.
    }
  else if (csr == CsrNumber::MENVCFG or csr == CsrNumber::SENVCFG or
            csr == CsrNumber::HENVCFG or csr == CsrNumber::MSECCFG)
    {
      if constexpr (sizeof(URV) == 8)
        {
          URV oldVal = 0;
          if (not peekCsr(csr, oldVal))
            oldVal = 0;

          HenvcfgFields<uint64_t> hf{val};
          unsigned pmm = hf.bits_.PMM;
          if (not pmaskManager_.isSupported(PmaskManager::Mode{pmm}))
            {
              pmm = HenvcfgFields<uint64_t>(oldVal).bits_.PMM;
              hf.bits_.PMM = pmm;
              val = hf.value_;
            }
        }
    }
  else if (csr == CsrNumber::HSTATUS)
    {
      if constexpr (sizeof(URV) == 8)
        {
          URV oldVal = 0;
          if (not peekCsr(csr, oldVal))
            oldVal = 0;

          HstatusFields<uint64_t> hf{val};
          unsigned pmm = hf.bits_.HUPMM;
          if (not pmaskManager_.isSupported(PmaskManager::Mode{pmm}))
            {
              pmm = HstatusFields<uint64_t>(oldVal).bits_.HUPMM;
              hf.bits_.HUPMM = pmm;
              val = hf.value_;
            }
        }
    }

  // Update CSR.
  auto lastVal = csRegs_.peek(csr);
  if (not csRegs_.write(csr, privMode_, val))
    {
      // Same HS-qualified illegal/virtual behavior as doCsrRead.
      using PM = PrivilegeMode;
      bool hsq = isRvs() and csRegs_.isReadable(csr, PM::Supervisor, false /*virtMode*/);
      hsq = hsq and isCsrWriteable(csr, PM::Supervisor, false /*virtMode*/);
      if (virtMode_ and hsq)
        virtualInst(di);
      else
        illegalInst(di);
      return;
    }
  postCsrUpdate(csr, val, lastVal);

  // Csr was written. If it was minstret, compensate for
  // auto-increment that will be done by run, runUntilAddress or
  // singleStep method.
  if (csr == CsrNumber::MINSTRET or csr == CsrNumber::MINSTRETH)
    if (minstretEnabled())
      minstret_--;

  // Same for mcycle.
  if (csr == CsrNumber::MCYCLE or csr == CsrNumber::MCYCLEH)
    cycleCount_--;
}


template <typename URV>
void
Hart<URV>::doCsrScWrite(const DecodedInst* di, CsrNumber csrn, URV csrVal,
                        URV scMask, unsigned intReg, URV intVal)
{
  // This meethod is a workaround for CSRs with aliased bits that are still writable even
  // when aliased. Say, we are executing "csrrc t0, mvip, t1" with t1[1]==0, the internal
  // value of MVIP[1] is 1, MVIP[1] is aliased to MIP[1], and MIP[1] is 0. We read MVIP
  // and we get the effective value of MVIP[1] as 0, we and it with ~t1[1] and get 0, we
  // write it back to MVIP[1] changing that to 0. That should not have happened since
  // t1[1] is 0 (bit 1 should not be cleared). So we set the write mask to 0 except where
  // the anded/ored bits are 1 to preserve the non set/cleared bits. This would not be
  // necessary if there is no aliasing.

  using CN = CsrNumber;

  if (csrn != CN::MVIP)
    {
      doCsrWrite(di, csrn, csrVal, intReg, intVal);
    }
  else
    {
      auto csr = csRegs_.getImplementedCsr(csrn);
      auto prevMask = csr->getWriteMask();
      csr->setWriteMask(prevMask & scMask);

      doCsrWrite(di, csrn, csrVal, intReg, intVal);

      csr->setWriteMask(prevMask);
    }
}


// Set control and status register csr (op2) to value of register rs1
// (op1) and save its original value in register rd (op0).
template <typename URV>
void
Hart<URV>::execCsrrw(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  if (not extensionIsEnabled(RvExtension::Zicsr))
    {
      illegalInst(di);
      return;
    }

  auto csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV prev = 0;
  if (di->op0() != 0)
    if (not doCsrRead(di, csr, true /*isWrite*/, prev))
      {
	if (postCsrInst_)
	  postCsrInst_(hartIx_, csr);
	return;
      }

  URV next = intRegs_.read(di->op1());

  // MIP read value is ored with supervisor external interrupt pin. Same for SIP if
  // supervisor external interrupt is delegated.
  using IC = InterruptCause;
  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
          (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithSeiPinAndMvip(prev);

  doCsrWrite(di, csr, next, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrs(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  if (not extensionIsEnabled(RvExtension::Zicsr))
    {
      illegalInst(di);
      return;
    }

  auto csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  // Attempts to access the seed CSR using a read-only CSR-access instruction (CSRRS/CSRRC
  // with rs1=x0 or CSRRSI/CSRRCI with uimm=0) raise an illegal-instruction exception; any
  // other CSR-access instruction may be used to access seed
  if (csr == CsrNumber::SEED and di->op1() == 0)
    {
      illegalInst(di);
      return;
    }

  URV prev = 0;
  bool isWrite = di->op1() != 0;
  if (not doCsrRead(di, csr, isWrite, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  // When determining next value, we check the MVIP bit.
  using IC = InterruptCause;
  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithMvip(prev);

  URV orMask = intRegs_.read(di->op1());
  URV next = prev | orMask;

  // When determining read value, we check both Mvip and SEI pin.
  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithSeiPin(prev);

  if (di->op1() == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrScWrite(di, csr, next, orMask, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrc(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  if (not extensionIsEnabled(RvExtension::Zicsr))
    {
      illegalInst(di);
      return;
    }

  auto csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  if (csr == CsrNumber::SEED and di->op1() == 0)
    {
      illegalInst(di);   // See execCsrrs
      return;
    }

  URV prev = 0;
  bool isWrite = di->op1() != 0;
  if (not doCsrRead(di, csr, isWrite, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  using IC = InterruptCause;
  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithMvip(prev);

  URV andMask = intRegs_.read(di->op1());
  URV next = prev & ~andMask;

  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithSeiPin(prev);

  if (di->op1() == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrScWrite(di, csr, next, andMask, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrwi(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  if (not extensionIsEnabled(RvExtension::Zicsr))
    {
      illegalInst(di);
      return;
    }

  auto csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV prev = 0;
  if (di->op0() != 0)
    if (not doCsrRead(di, csr, true /*isWrite*/, prev))
      {
        if (postCsrInst_)
          postCsrInst_(hartIx_, csr);
        return;
      }

  // MIP read value is ored with supervisor external interrupt pin. Same for SIP if
  // supervisor external interrupt is delegated.
  using IC = InterruptCause;
  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithSeiPinAndMvip(prev);

  doCsrWrite(di, csr, di->op1(), di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrsi(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  if (not extensionIsEnabled(RvExtension::Zicsr))
    {
      illegalInst(di);
      return;
    }

  auto csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV imm = di->op1();
  if (csr == CsrNumber::SEED and imm == 0)
    {
      illegalInst(di);  // See execCsrrs
      return;
    }

  URV prev = 0;
  bool isWrite = imm != 0;
  if (not doCsrRead(di, csr, isWrite, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  // MIP read value is ored with supervisor external interrupt pin. Same for SIP if
  // supervisor external interrupt is delegated.
  using IC = InterruptCause;
  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithMvip(prev);

  URV orMask = imm;
  URV next = prev | orMask;

  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithSeiPin(prev);

  if (imm == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrScWrite(di, csr, next, orMask, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execCsrrci(const DecodedInst* di)
{
  if (breakpOrEnterDebugTripped())
    return;

  if (not extensionIsEnabled(RvExtension::Zicsr))
    {
      illegalInst(di);
      return;
    }

  auto csr = CsrNumber(di->op2());

  if (preCsrInst_)
    preCsrInst_(hartIx_, csr);

  URV imm = di->op1();
  if (csr == CsrNumber::SEED and imm == 0)
    {
      illegalInst(di);  // See execCsrrs
      return;
    }

  URV prev = 0;
  bool isWrite = imm != 0;
  if (not doCsrRead(di, csr, isWrite, prev))
    {
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  using IC = InterruptCause;
  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithMvip(prev);

  URV andMask = imm;
  URV next = prev & ~andMask;

  if (csr == CsrNumber::MIP or
      (not virtMode_ and csr == CsrNumber::SIP and
            (csRegs_.peekMideleg() & (URV(1) << URV(IC::S_EXTERNAL)))))
    prev = csRegs_.overrideWithSeiPin(prev);

  if (imm == 0)
    {
      updatePerformanceCountersForCsr(*di);
      intRegs_.write(di->op0(), prev);
      if (postCsrInst_)
        postCsrInst_(hartIx_, csr);
      return;
    }

  doCsrScWrite(di, csr, next, andMask, di->op0(), prev);

  if (postCsrInst_)
    postCsrInst_(hartIx_, csr);
}


template <typename URV>
void
Hart<URV>::execLb(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<int8_t>(di, virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execLbu(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint8_t>(di, virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
void
Hart<URV>::execLhu(const DecodedInst* di)
{
  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint16_t>(di, virtAddr, data))
    intRegs_.write(di->op0(), data);
}

template <typename URV>
ExceptionCause
Hart<URV>::determineStoreException(uint64_t& addr1, uint64_t& addr2,
                                   uint64_t& gaddr1, uint64_t& gaddr2,
				   unsigned stSize, bool hyper, bool amo)
{
  // NOLINTNEXTLINE(modernize-use-auto)
  uint64_t va1 = URV(addr1); // Virtual address. Truncate to 32-bits in 32-bit mode.
  uint64_t va2 = va1;        // Used if crossing page boundary
  ldStFaultAddr_ = va1;
  addr1 = gaddr1 = va1;
  addr2 = gaddr2 = va2;  // Phys addr of 2nd page when crossing page boundary.

  uint64_t alignMask = stSize - 1;
  bool misal = addr1 & alignMask;
  misalignedLdSt_ = misal;

  using EC = ExceptionCause;
  using PM = PrivilegeMode;

  auto [pm, virt] = effLdStMode(hyper);

  ldStFaultAddr_ = addr1 = gaddr1 = va1 = applyPointerMask(va1, false, hyper);
  addr2 = gaddr2 = va2 = va1;

  // If misaligned exception has priority take exception.
  if (misal)
    {
      if (misalHasPriority_ and not misalDataOk_)
	return ExceptionCause::STORE_ADDR_MISAL;
      va2 = (va1 + stSize - 1) & ~alignMask;
    }

  setMemProtAccIsFetch(false);
  steeInsec1_ = false;
  steeInsec2_ = false;

  auto checkPa = [this, pm, stSize, misal, amo] (uint64_t va, uint64_t& pa, Pma& pma, bool lower) -> EC {
    ldStFaultAddr_ = va;

    if (pmpEnabled_)
      {
        auto pmp = pmpMgr_.accessPmp(pm, pa);
        if (not pmp.isWrite())
          return EC::STORE_ACC_FAULT;
      }

    if (steeEnabled_)
      {
        if (not stee_.isValidAddress(pa))
          return EC::STORE_ACC_FAULT;
        bool& steeInsec = lower? steeInsec1_ : steeInsec2_;
        steeInsec = stee_.isInsecureAccess(pa);
        pa = stee_.clearSecureBits(pa);
      }

    pma = accessPma(pa);
    pma = overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt());
    if (not pma.isWrite())
      return EC::STORE_ACC_FAULT;

    auto size = stSize;

    if (misal)
      {
        bool ok = pma.isMisalignedOk();
        if (amo)
          {
            ok = false;
            if (auto mag = pma.misalAtomicGranule(); mag)
              {
                auto mask = ~uint64_t(mag - 1);
                ok = (pa & mask) == ((pa + stSize - 1) & mask);
              }
          }
        if (not ok)
          return pma.misalOnMisal()? EC::STORE_ADDR_MISAL : EC::STORE_ACC_FAULT;

        // If checking the lower part of a misal address, do not cross alignment boundary.
        // If pa is 0xffc and size is 8, then size becomes 4 (dist to next multiple of 8).
        if (lower)
          size = ((pa + (size - 1)) & ~uint64_t(size - 1)) - pa;
      }

    // In case memory size is less that what the PMA/PMP declares as accessible.
    if (pa > memory_.size() - size)
      return EC::STORE_ACC_FAULT;

    return EC::NONE;
  };

  bool translate = isRvs() and pm != PM::Machine;
  if (translate)
    {
      if (auto cause = virtMem_.translateForStore(va1, pm, virt, gaddr1, addr1);
          cause != EC::NONE)
        {
          ldStFaultAddr_ = addr1;
          return cause;
        }
    }

  gaddr2 = gaddr1;
  addr2 = addr1;
  uint64_t pa1 = addr1;  // We do this because checkPa modifies addr1 (clears STEE bits).

  ldStPma1_ = ldStPma2_ = Pma{};

  if (not misal)
    {
      if (auto cause = checkPa(va1, addr1, ldStPma1_, true); cause != EC::NONE)
        return cause;
      addr2 = addr1;  // checkPa may clear STEE bits of addr1
    }
  else
    {
      if (inSeqnMisaligned_)
        if (auto cause = checkPa(va1, addr1, ldStPma1_, true); cause != EC::NONE)
          return cause;

      bool cross = virtMem_.pageNumber(va1) != virtMem_.pageNumber(va2);
      addr2 = (pa1 + (stSize - 1)) & ~alignMask;

      if (cross and translate)
        {
          auto cause = virtMem_.translateForStore(va2, pm, virt, gaddr2, addr2);
          if (cause != EC::NONE)
            {
              ldStFaultAddr_ = addr2;
              gaddr1 = gaddr2;  // We report faulting GPA in gaddr.
              return cause;
            }
        }

      if (inSeqnMisaligned_)
        if (auto cause = checkPa(va2, addr2, ldStPma2_, false); cause != EC::NONE)
          return cause;

      if (not inSeqnMisaligned_)
        {
          if (auto cause = checkPa(va1, addr1, ldStPma1_, true); cause != EC::NONE)
            return cause;
          if (auto cause = checkPa(va2, addr2, ldStPma2_, false); cause != EC::NONE)
            return cause;
        }

      if (not cross)
        addr2 = addr1; // addr2 is different from addr1 only if we cross page boundary
    }

  return EC::NONE;
}


template <typename URV>
void
Hart<URV>::execSb(const DecodedInst* di)
{
  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  auto value = uint8_t(intRegs_.read(di->op0()));
  store<uint8_t>(di, addr, value);
}


template <typename URV>
void
Hart<URV>::execSh(const DecodedInst* di)
{
  uint32_t rs1 = di->op1();
  URV base = intRegs_.read(rs1);
  URV addr = base + di->op2As<SRV>();
  auto value = uint16_t(intRegs_.read(di->op0()));
  store<uint16_t>(di, addr, value);
}


template<typename URV>
void
Hart<URV>::execMul(const DecodedInst* di)
{
  if (not isRvzmmul() and not isRvm())
    {
      illegalInst(di);
      return;
    }

  SRV a = intRegs_.read(di->op1());
  SRV b = intRegs_.read(di->op2());

  SRV c = a * b;
  intRegs_.write(di->op0(), c);
}


namespace WdRiscv
{

  template<>
  void
  Hart<uint32_t>::execMulh(const DecodedInst* di)
  {
    if (not isRvzmmul() and not isRvm())
      {
	illegalInst(di);
	return;
      }

    int64_t a = int32_t(intRegs_.read(di->op1()));  // sign extend.
    int64_t b = int32_t(intRegs_.read(di->op2()));
    int64_t c = a * b;
    auto high = static_cast<int32_t>(c >> 32);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint32_t>::execMulhsu(const DecodedInst* di)
  {
    if (not isRvzmmul() and not isRvm())
      {
	illegalInst(di);
	return;
      }

    int64_t a = int32_t(intRegs_.read(di->op1()));
    int64_t b = uint32_t(intRegs_.read(di->op2()));
    int64_t c = a * b;
    auto high = static_cast<int32_t>(c >> 32);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint32_t>::execMulhu(const DecodedInst* di)
  {
    if (not isRvzmmul() and not isRvm())
      {
	illegalInst(di);
	return;
      }

    uint64_t a = uint32_t(intRegs_.read(di->op1()));
    uint64_t b = uint32_t(intRegs_.read(di->op2()));
    uint64_t c = a * b;
    auto high = static_cast<uint32_t>(c >> 32);

    intRegs_.write(di->op0(), high);
  }


  template<>
  void
  Hart<uint64_t>::execMulh(const DecodedInst* di)
  {
    if (not isRvzmmul() and not isRvm())
      {
	illegalInst(di);
	return;
      }

    Int128 a = int64_t(intRegs_.read(di->op1()));  // sign extend.
    Int128 b = int64_t(intRegs_.read(di->op2()));
    Int128 c = a * b;
    auto high = static_cast<int64_t>(c >> 64);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint64_t>::execMulhsu(const DecodedInst* di)
  {
    if (not isRvzmmul() and not isRvm())
      {
	illegalInst(di);
	return;
      }

    Int128 a = int64_t(intRegs_.read(di->op1()));
    Int128 b = intRegs_.read(di->op2());
    Int128 c = a * b;
    auto high = static_cast<int64_t>(c >> 64);

    intRegs_.write(di->op0(), high);
  }


  template <>
  void
  Hart<uint64_t>::execMulhu(const DecodedInst* di)
  {
    if (not isRvzmmul() and not isRvm())
      {
	illegalInst(di);
	return;
      }

    Uint128 a = intRegs_.read(di->op1());
    Uint128 b = intRegs_.read(di->op2());
    Uint128 c = a * b;
    auto high = static_cast<uint64_t>(c >> 64);

    intRegs_.write(di->op0(), high);
  }

}


template <typename URV>
void
Hart<URV>::execDiv(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  SRV a = intRegs_.read(di->op1());
  SRV b = intRegs_.read(di->op2());
  SRV c = -1;   // Divide by zero result
  if (b != 0)
    {
      SRV minInt = SRV(1) << (mxlen_ - 1);
      if (a == minInt and b == -1)
	c = a;
      else
	c = a / b;  // Per spec: User-Level ISA, Version 2.3, Section 6.2
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


template <typename URV>
void
Hart<URV>::execDivu(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  URV a = intRegs_.read(di->op1());
  URV b = intRegs_.read(di->op2());
  URV c = ~ URV(0);  // Divide by zero result.
  if (b != 0)
    c = a / b;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


// Remainder instruction.
template <typename URV>
void
Hart<URV>::execRem(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  SRV a = intRegs_.read(di->op1());
  SRV b = intRegs_.read(di->op2());
  SRV c = a;  // Divide by zero remainder.
  if (b != 0)
    {
      SRV minInt = SRV(1) << (mxlen_ - 1);
      if (a == minInt and b == -1)
	c = 0;   // Per spec: User-Level ISA, Version 2.3, Section 6.2
      else
	c = a % b;
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


// Unsigned remainder instruction.
template <typename URV>
void
Hart<URV>::execRemu(const DecodedInst* di)
{
  if (not isRvm())
    {
      illegalInst(di);
      return;
    }

  URV a = intRegs_.read(di->op1());
  URV b = intRegs_.read(di->op2());
  URV c = a;  // Divide by zero remainder.
  if (b != 0)
    c = a % b;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  intRegs_.write(di->op0(), c);
}


template <typename URV>
void
Hart<URV>::execLwu(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  URV base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint32_t>(di, virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <>
void
Hart<uint32_t>::execLd(const DecodedInst* di)
{
  if (not isRvzilsd())
    {
      illegalInst(di);
      return;
    }

  if ((di->op0() % 2) != 0)
    {
      illegalInst(di);   // Destination register number must be even.
      return;
    }

  uint32_t base = intRegs_.read(di->op1());
  uint32_t virtAddr = base + di->op2As<int32_t>();

  // Zilsd decomposes into 32-bit sub-accesses. Addresses that are not
  // 4-byte aligned must be reported as misaligned load exceptions.
  if (virtAddr & 0x3)
    {
      initiateLoadException(di, ExceptionCause::LOAD_ADDR_MISAL, virtAddr, virtAddr);
      return;
    }

  // Perform architected RV32 pair-load as two 32-bit constituent accesses.
  uint64_t low = 0, high = 0;
  if (not load<uint32_t>(di, virtAddr, low))
    return;

  // Allow first-sub-operation effect to be visible if second faults.
  if (di->op0() != 0)
    intRegs_.write(di->op0(), uint32_t(low));

  if (not load<uint32_t>(di, virtAddr + 4, high))
    return;

  if (di->op0() != 0)
    intRegs_.write(di->op0() + 1, uint32_t(high));
}


template <>
void
Hart<uint64_t>::execLd(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }
  uint64_t base = intRegs_.read(di->op1());
  uint64_t virtAddr = base + di->op2As<int32_t>();

  uint64_t data = 0;
  if (load<uint64_t>(di, virtAddr, data))
    intRegs_.write(di->op0(), data);
}


template <typename URV>
inline
void
Hart<URV>::execLq(const DecodedInst* di)
{
  // TODO: implement once RV128 is supported
  illegalInst(di);
}


template <>
void
Hart<uint32_t>::execSd(const DecodedInst* di)
{
  if (not isRvzilsd())
    {
      illegalInst(di);
      return;
    }

  if ((di->op0() % 2) != 0)
    {
      illegalInst(di);   // Stored register number must be even.
      return;
    }

  unsigned rs1 = di->op1();

  uint32_t base = intRegs_.read(rs1);
  uint32_t addr = base + di->op2As<int32_t>();

  // Zilsd decomposes into 32-bit sub-accesses. Addresses that are not
  // 4-byte aligned must be reported as misaligned store exceptions.
  if (addr & 0x3)
    {
      initiateStoreException(di, ExceptionCause::STORE_ADDR_MISAL, addr, addr);
      return;
    }

  // Perform architected RV32 pair-store as two 32-bit constituent accesses.
  auto low = uint32_t(intRegs_.read(di->op0()));
  uint32_t high = (di->op0() == 0) ? 0 : uint32_t(intRegs_.read(di->op0() + 1));

  if (not store<uint32_t>(di, addr, low))
    return;

  // If this second sub-operation traps, first-sub-operation effects remain visible.
  store<uint32_t>(di, addr + 4, high);
}


template <>
void
Hart<uint64_t>::execSd(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  unsigned rs1 = di->op1();

  uint64_t base = intRegs_.read(rs1);
  uint64_t addr = base + di->op2As<int64_t>();
  uint64_t value = intRegs_.read(di->op0());

  store<uint64_t>(di, addr, value);
}


template <typename URV>
void
Hart<URV>::execSq(const DecodedInst* di)
{
  // TODO: implement once RV128 is supported
  illegalInst(di);
}


template <typename URV>
void
Hart<URV>::execSlliw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t amount(di->op2());

  if (amount > 0x1f)
    {
      illegalInst(di);   // Bits 5 and 6 of immeidate must be zero.
      return;
    }

  auto word = int32_t(intRegs_.read(di->op1()));
  word <<= static_cast<int>(amount);

  SRV value = word; // Sign extend to 64-bit.
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSrliw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t amount(di->op2());

  if (amount > 0x1f)
    {
      illegalInst(di);   // Bits 5 and 6 of immediate must be zero.
      return;
    }

  auto word = uint32_t(intRegs_.read(di->op1()));
  word >>= amount;

  SRV value = int32_t(word); // Sign extend to 64-bit.
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSraiw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t amount(di->op2());

  if (amount > 0x1f)
    {
      illegalInst(di);   // Bits 5 and 6 of immeddiate must be zero.
      return;
    }

  auto word = int32_t(intRegs_.read(di->op1()));
  word >>= static_cast<int>(amount);

  SRV value = word; // Sign extend to 64-bit.
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execAddiw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  // Signed overflow is undefined behavior, and Clang optimizes in such a way
  // that the overflow does not occur even if intended.  As a result, do the
  // addition as unsigned to allow the overflow and then convert to signed
  // before sign extending.
  auto word = uint32_t(intRegs_.read(di->op1()));
  word += di->op2As<uint32_t>();
  SRV value = static_cast<int32_t>(word);  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execAddw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  auto word = int32_t(intRegs_.read(di->op1()) + intRegs_.read(di->op2()));
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSubw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  auto word = int32_t(intRegs_.read(di->op1()) - intRegs_.read(di->op2()));
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSllw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t shift = intRegs_.read(di->op2()) & 0x1f;
  auto word = int32_t(intRegs_.read(di->op1()) << shift);
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSrlw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  auto word = uint32_t(intRegs_.read(di->op1()));
  auto shift = uint32_t(intRegs_.read(di->op2()) & 0x1f);
  word >>= shift;
  SRV value = int32_t(word);  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSraw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  auto word = int32_t(intRegs_.read(di->op1()));
  auto shift = int32_t(intRegs_.read(di->op2()) & 0x1f);
  word >>= shift;
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execMulw(const DecodedInst* di)
{
  if (not isRv64() or (not isRvm() and not isRvzmmul()))
    {
      illegalInst(di);
      return;
    }

  auto word1 = int32_t(intRegs_.read(di->op1()));
  auto word2 = int32_t(intRegs_.read(di->op2()));
  auto word = int32_t(word1 * word2);
  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execDivw(const DecodedInst* di)
{
  if (not isRv64() or not isRvm())
    {
      illegalInst(di);
      return;
    }

  auto word1 = int32_t(intRegs_.read(di->op1()));
  auto word2 = int32_t(intRegs_.read(di->op2()));

  int32_t word = -1;  // Divide by zero result
  if (word2 != 0)
    {
      int32_t minInt = int32_t(1) << 31;
      if (word1 == minInt and word2 == -1)
	word = word1;
      else
	word = word1 / word2;
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execDivuw(const DecodedInst* di)
{
  if (not isRv64() or not isRvm())
    {
      illegalInst(di);
      return;
    }

  auto word1 = uint32_t(intRegs_.read(di->op1()));
  auto word2 = uint32_t(intRegs_.read(di->op2()));

  auto word = ~uint32_t(0);  // Divide by zero result.
  if (word2 != 0)
    word = word1 / word2;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  URV value = SRV(int32_t(word));  // Sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execRemw(const DecodedInst* di)
{
  if (not isRv64() or not isRvm())
    {
      illegalInst(di);
      return;
    }

  auto word1 = int32_t(intRegs_.read(di->op1()));
  auto word2 = int32_t(intRegs_.read(di->op2()));

  int32_t word = word1;  // Divide by zero remainder
  if (word2 != 0)
    {
      int32_t minInt = int32_t(1) << 31;
      if (word1 == minInt and word2 == -1)
	word = 0;   // Per spec: User-Level ISA, Version 2.3, Section 6.2
      else
	word = word1 % word2;
    }

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  SRV value = word;  // sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execRemuw(const DecodedInst* di)
{
  if (not isRv64() or not isRvm())
    {
      illegalInst(di);
      return;
    }

  auto word1 = uint32_t(intRegs_.read(di->op1()));
  auto word2 = uint32_t(intRegs_.read(di->op2()));

  uint32_t word = word1;  // Divide by zero remainder
  if (word2 != 0)
    word = word1 % word2;

  recordDivInst(di->op0(), peekIntReg(di->op0()));

  URV value = SRV(int32_t(word));  // Sign extend to 64-bits
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execWrs_nto(const DecodedInst* di)
{
  // Wait for reservation store. No timeout.

  if (not isRvzawrs())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  auto pm = privilegeMode();

  if (mstatus_.bits_.TW and pm != PM::Machine)
    {
      illegalInst(di);
      return;
    }

  // VS/VU mode, VTW=1 and mstatus.TW=0
  if (virtMode_ and (pm == PM::Supervisor or pm == PM::User) and not mstatus_.bits_.TW and hstatus_.bits_.VTW)
    {
      virtualInst(di);
      return;
    }

  // In server/interactive mode, we skip cancelLr. The driver (test-bench) will explicitly
  // cancel-lr at the right time.
  if (wrsCancelsLr_)
    cancelLr(CancelLrCause::WRS_NTO);  // Lose reservation.
}


template <typename URV>
void
Hart<URV>::execWrs_sto(const DecodedInst* di)
{
  // Wait for reservaton store. Short timeout.

  if (not isRvzawrs())
    {
      illegalInst(di);
      return;
    }

  // In server/interactive mode, we skip cancelLr. The driver (test-bench) will explicitly
  // cancel-lr at the right time.
  if (wrsCancelsLr_)
    cancelLr(CancelLrCause::WRS_STO);  // Lose reservation.
}


template <typename URV>
void
Hart<URV>::execCzero_eqz(const DecodedInst* di)
{
  if (not isRvzicond())
    {
      illegalInst(di);
      return;
    }
  URV value = intRegs_.read(di->op1());
  URV condition = intRegs_.read(di->op2());
  URV res = (condition == 0) ? 0 : value;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execCzero_nez(const DecodedInst* di)
{
  if (not isRvzicond())
    {
      illegalInst(di);
      return;
    }
  URV value = intRegs_.read(di->op1());
  URV condition = intRegs_.read(di->op2());
  URV res = (condition != 0) ? 0 : value;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execC_zext_h(const DecodedInst* di)
{
  if (not isRvzcb() or not isRvzbb())
    {
      illegalInst(di);
      return;
    }
  URV value = intRegs_.read(di->op1());
  value &= 0xffff;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execMop_r(const DecodedInst* di)
{
  if (not isRvzimop())
    {
      illegalInst(di);
      return;
    }

  if (isShadowStackEnabled(privMode_, virtMode_))
    {
      if (di->op0() == 0 and (di->op1() == 1 or di->op1() == 5))
        {
          execSspopchk(di, di->op1());
          return;
        }
      if (di->op0() != 0 and di->op1() == 0)
        {
          execSsrdp(di);
          return;
        }
    }

  URV value = 0;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execMop_rr(const DecodedInst* di)
{
  if (not isRvzimop())
    {
      illegalInst(di);
      return;
    }

  if (isShadowStackEnabled(privMode_, virtMode_))
    {
      // SSPUSH is encoded as mop_rr with rd=x0, rs1=x0, rs2 in {x1,x5}.
      // Use rs2 (op2) as the pushed register source.
      if (di->op0() == 0 and di->op1() == 0 and (di->op2() == 1 or di->op2() == 5))
        {
          execSspush(di, di->op2());
          return;
        }
    }

  URV value = 0;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execCmop(const DecodedInst* di)
{
  if (not isRvzcmop() or not isRvzca())
    {
      illegalInst(di);
      return;
    }

  if (isShadowStackEnabled(privMode_, virtMode_))
    {
      if (di->op0() == 5)
        {
          execSspopchk(di, di->op0());
          return;
        }
      if (di->op0() == 1)
        {
          execSspush(di, di->op0());
          return;
        }
    }

  URV value = 0;
  intRegs_.write(RegX0, value);
}


template <typename URV>
void
Hart<URV>::execLpad(const DecodedInst* di)
{
  if (di->instId() != InstId::auipc or
      di->op0() != RegX0 or
      currPc_ & 3) // PC must be word aligned.
    {
      initiateException(ExceptionCause::SOFTWARE_CHECK, currPc_, 2 /* Landing pad */);
      return;
    }

  uint32_t lpl = di->op1();
  URV expected = (intRegs_.read(RegX7) & 0xfffff000); // Must match bits 31:12
  if (lpl != 0 and expected != lpl)
    {
      initiateException(ExceptionCause::SOFTWARE_CHECK, currPc_, 2 /* Landing pad */);
      return;
    }
  setElp(false);
}


template <typename URV>
void
Hart<URV>::applySpmcntrpmf()
{
  if (not isa_.isEnabled(RvExtension::Smcntrpmf))
    return;

  auto minstCfg = csRegs_.read64(CsrNumber::MINSTRETCFG);

  auto fields = MhpmeventFields(minstCfg);  // INH bits are in same bit positions as MHPMEVENT

  using PM = PrivilegeMode;
  bool enable = true;   // Mode enable MINSTRET.

  if (privMode_ == PM::Machine)
    enable = not fields.bits_.MINH;
  else if (privMode_ == PM::Supervisor and virtMode_)
    enable = not fields.bits_.VSINH;
  else if (privMode_ == PM::User and virtMode_ )
    enable = not fields.bits_.VUINH;
  else if (privMode_ == PM::Supervisor and not virtMode_)
    enable = not fields.bits_.SINH;
  else if (privMode_ == PM::User and not virtMode_ )
    enable = not fields.bits_.UINH;

  minstretControl_ = enable? 0x4 : 0;

  auto mcycleCfg = csRegs_.read64(CsrNumber::MCYCLECFG);
  fields = MhpmeventFields(mcycleCfg);
  enable = true;  // Mode enable MCYCLE

  if (privMode_ == PM::Machine)
    enable = not fields.bits_.MINH;
  else if (privMode_ == PM::Supervisor and virtMode_)
    enable = not fields.bits_.VSINH;
  else if (privMode_ == PM::User and virtMode_ )
    enable = not fields.bits_.VUINH;
  else if (privMode_ == PM::Supervisor and not virtMode_)
    enable = not fields.bits_.SINH;
  else if (privMode_ == PM::User and not virtMode_ )
    enable = not fields.bits_.UINH;

  mcycleControl_ = enable? 1 : 0;
}


template
bool
WdRiscv::Hart<uint32_t>::load<uint8_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<int8_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<uint16_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<int16_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<uint32_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<int32_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::load<uint64_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<uint8_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<int8_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<uint16_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<int16_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<uint32_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<int32_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::load<uint64_t>(const DecodedInst*, uint64_t, uint64_t&);


template
bool
WdRiscv::Hart<uint32_t>::store<uint8_t>(const DecodedInst*, uint32_t, uint8_t, bool);

template
bool
WdRiscv::Hart<uint32_t>::store<uint16_t>(const DecodedInst*, uint32_t, uint16_t, bool);

template
bool
WdRiscv::Hart<uint32_t>::store<uint32_t>(const DecodedInst*, uint32_t, uint32_t, bool);

template
bool
WdRiscv::Hart<uint32_t>::store<uint64_t>(const DecodedInst*, uint32_t, uint64_t, bool);

template
bool
WdRiscv::Hart<uint64_t>::store<uint8_t>(const DecodedInst*, uint64_t, uint8_t, bool);

template
bool
WdRiscv::Hart<uint64_t>::store<uint16_t>(const DecodedInst*, uint64_t, uint16_t, bool);

template
bool
WdRiscv::Hart<uint64_t>::store<uint32_t>(const DecodedInst*, uint64_t, uint32_t, bool);

template
bool
WdRiscv::Hart<uint64_t>::store<uint64_t>(const DecodedInst*, uint64_t, uint64_t, bool);


template
bool
WdRiscv::Hart<uint32_t>::readForLoad<uint8_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint32_t>::readForLoad<int8_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint32_t>::readForLoad<uint16_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint32_t>::readForLoad<int16_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint32_t>::readForLoad<uint32_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint32_t>::readForLoad<int32_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint32_t>::readForLoad<uint64_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint64_t>::readForLoad<uint8_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint64_t>::readForLoad<int8_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint64_t>::readForLoad<uint16_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint64_t>::readForLoad<int16_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint64_t>::readForLoad<uint32_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint64_t>::readForLoad<int32_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);

template
bool
WdRiscv::Hart<uint64_t>::readForLoad<uint64_t>(const DecodedInst*, uint64_t, uint64_t, uint64_t, uint64_t&, unsigned, unsigned);


template
bool
WdRiscv::Hart<uint32_t>::writeForStore<uint8_t>(uint64_t, uint64_t, uint64_t, uint8_t);

template
bool
WdRiscv::Hart<uint32_t>::writeForStore<uint16_t>(uint64_t, uint64_t, uint64_t, uint16_t);

template
bool
WdRiscv::Hart<uint32_t>::writeForStore<uint32_t>(uint64_t, uint64_t, uint64_t, uint32_t);

template
bool
WdRiscv::Hart<uint32_t>::writeForStore<uint64_t>(uint64_t, uint64_t, uint64_t, uint64_t);

template
bool
WdRiscv::Hart<uint64_t>::writeForStore<uint8_t>(uint64_t, uint64_t, uint64_t, uint8_t);

template
bool
WdRiscv::Hart<uint64_t>::writeForStore<uint16_t>(uint64_t, uint64_t, uint64_t, uint16_t);

template
bool
WdRiscv::Hart<uint64_t>::writeForStore<uint32_t>(uint64_t, uint64_t, uint64_t, uint32_t);

template
bool
WdRiscv::Hart<uint64_t>::writeForStore<uint64_t>(uint64_t, uint64_t, uint64_t, uint64_t);


template
bool
WdRiscv::Hart<uint32_t>::fastLoad<uint8_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::fastLoad<uint16_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::fastLoad<uint32_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint32_t>::fastLoad<uint64_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::fastLoad<uint8_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::fastLoad<uint16_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::fastLoad<uint32_t>(const DecodedInst*, uint64_t, uint64_t&);

template
bool
WdRiscv::Hart<uint64_t>::fastLoad<uint64_t>(const DecodedInst*, uint64_t, uint64_t&);


template
bool
WdRiscv::Hart<uint32_t>::fastStore<uint8_t>(const DecodedInst*, uint64_t, uint8_t);

template
bool
WdRiscv::Hart<uint32_t>::fastStore<uint16_t>(const DecodedInst*, uint64_t, uint16_t);

template
bool
WdRiscv::Hart<uint32_t>::fastStore<uint32_t>(const DecodedInst*, uint64_t, uint32_t);

template
bool
WdRiscv::Hart<uint32_t>::fastStore<uint64_t>(const DecodedInst*, uint64_t, uint64_t);

template
bool
WdRiscv::Hart<uint64_t>::fastStore<uint8_t>(const DecodedInst*, uint64_t, uint8_t);

template
bool
WdRiscv::Hart<uint64_t>::fastStore<uint16_t>(const DecodedInst*, uint64_t, uint16_t);

template
bool
WdRiscv::Hart<uint64_t>::fastStore<uint32_t>(const DecodedInst*, uint64_t, uint32_t);

template
bool
WdRiscv::Hart<uint64_t>::fastStore<uint64_t>(const DecodedInst*, uint64_t, uint64_t);


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;

