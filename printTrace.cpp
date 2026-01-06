#include <cinttypes>
#include <iostream>
#include <iomanip>
#include <ranges>
#include <sstream>
#include "Hart.hpp"
#include "Trace.hpp"
#include "Stee.hpp"

using namespace WdRiscv;
using TT_STEE::Stee;

//NOLINTNEXTLINE(bugprone-reserved-identifier, cppcoreguidelines-avoid-non-const-global-variables)
void (*__tracerExtension)(void*) = nullptr;

static std::mutex& printInstTraceMutex()
{
  static std::mutex value;
  return value;
}

template <typename URV>
static constexpr
std::string_view
privilegeModeToStr(const Hart<URV>& hart)
{
  bool vm = hart.lastVirtMode();
  auto pm = hart.lastPrivMode();
  bool dm = hart.lastDebugMode();

  bool hyper = hart.extensionIsEnabled(RvExtension::H);

  if (not vm)
    {
      if (pm == PrivilegeMode::Machine)    return dm? "D" : "M";
      if (pm == PrivilegeMode::Supervisor) return hyper ? "HS" : "S";
      if (pm == PrivilegeMode::User)       return "U";
    }
  else
    {
      if (pm == PrivilegeMode::Supervisor) return "VS";
      if (pm == PrivilegeMode::User) return "VU";
    }
  return "?";
}


template <typename URV>
void
formatVecInstTrace(FILE* out, uint64_t tag, const Hart<URV>& hart,
		   std::string_view opcode, unsigned vecReg,
		   std::span<const uint8_t> data, unsigned byteCount,
		   std::string_view assembly);


template <>
void
formatVecInstTrace<uint32_t>(FILE* out, uint64_t tag, const Hart<uint32_t>& hart,
			     std::string_view opcode, unsigned vecReg,
			     std::span<const uint8_t> data, unsigned byteCount,
			     std::string_view assembly)
{
  std::string_view pmStr = privilegeModeToStr(hart);
  unsigned ix = hart.sysHartIndex();
  uint32_t pc = hart.lastPc();

  fprintf(out, "#%jd %d %2s %08x %8s v %02x ",
	  uintmax_t(tag), ix, pmStr.data(), pc, opcode.data(), vecReg);
  for (unsigned i = 0; i < byteCount; ++i)
    fprintf(out, "%02x", data[byteCount - 1 - i]);
  fprintf(out, " %s", assembly.data());
}


template <>
void
formatVecInstTrace<uint64_t>(FILE* out, uint64_t tag, const Hart<uint64_t>& hart,
			     std::string_view opcode, unsigned vecReg,
			     std::span<const uint8_t> data, unsigned byteCount,
			     std::string_view assembly)
{
  std::string_view pmStr = privilegeModeToStr(hart);

  unsigned ix = hart.sysHartIndex();
  uint64_t pc = hart.lastPc();

  fprintf(out, "#%jd %d %2s %016jx %8s v %02x ",
          uintmax_t(tag), ix, pmStr.data(), uintmax_t(pc), opcode.data(), vecReg);
  for (unsigned i = 0; i < byteCount; ++i)
    fprintf(out, "%02x", data[byteCount - 1 - i]);
  fprintf(out, " %s", assembly.data());
}


template <typename URV>
void
formatInstTrace(FILE* out, uint64_t tag, const Hart<URV>& hart,
		std::string_view opcode, char resource, URV addr,
		URV value, std::string_view assembly);

template <>
void
formatInstTrace<uint32_t>(FILE* out, uint64_t tag, const Hart<uint32_t>& hart,
                          std::string_view opcode, char resource, uint32_t addr,
                          uint32_t value, std::string_view assembly)
{
  std::string_view pmStr = privilegeModeToStr(hart);
  unsigned ix = hart.sysHartIndex();
  uint32_t pc = hart.lastPc();

  if (resource == 'r')
    {
      fprintf(out, "#%jd %d %2s %08x %8s r %02x         %08x  %s",
              uintmax_t(tag), ix, pmStr.data(),
	      pc, opcode.data(), addr, value, assembly.data());
    }
  else if (resource == 'c')
    {
      if ((addr >> 16) == 0)
        fprintf(out, "#%jd %d %2s %08x %8s c %04x       %08x  %s",
                uintmax_t(tag), ix, pmStr.data(), pc,
		opcode.data(), addr, value, assembly.data());
      else
        fprintf(out, "#%jd %d %2s %08x %8s c %08x   %08x  %s",
                uintmax_t(tag), ix, pmStr.data(), pc, opcode.data(),
		addr, value, assembly.data());
    }
  else
    {
      fprintf(out, "#%jd %d %2s %08x %8s %c %08x   %08x  %s", uintmax_t(tag),
	      ix, pmStr.data(), pc, opcode.data(),
	      resource, addr, value, assembly.data());
    }
}


template <>
void
formatInstTrace<uint64_t>(FILE* out, uint64_t tag, const Hart<uint64_t>& hart,
                          std::string_view opcode, char resource, uint64_t addr,
                          uint64_t value, std::string_view assembly)
{
  unsigned ix = hart.sysHartIndex();
  uint64_t pc = hart.lastPc();
  std::string_view pmStr = privilegeModeToStr(hart);

  fprintf(out, "#%jd %d %2s %016jx %8s %c %016jx %016jx %s",
          uintmax_t(tag), ix, pmStr.data(), uintmax_t(pc), opcode.data(),
	  resource, uintmax_t(addr), uintmax_t(value), assembly.data());
}


template <typename URV>
void
formatFpInstTrace(FILE* out, uint64_t tag, const Hart<URV>& hart,
		  std::string_view opcode, unsigned fpReg,
		  uint64_t fpVal, unsigned width, std::string_view assembly);

template <>
void
formatFpInstTrace<uint32_t>(FILE* out, uint64_t tag, const Hart<uint32_t>& hart,
			    std::string_view opcode, unsigned fpReg,
			    uint64_t fpVal, unsigned width,
			    std::string_view assembly)
{
  unsigned ix = hart.sysHartIndex();
  uint32_t pc = hart.lastPc();
  std::string_view pmStr = privilegeModeToStr(hart);

  if (width == 64)
    {
      fprintf(out, "#%jd %d %2s %08x %8s f %02x %016jx  %s", uintmax_t(tag), ix,
	      pmStr.data(), pc, opcode.data(), fpReg, uintmax_t(fpVal),
	      assembly.data());
    }
  else
    {
      uint32_t val32 = fpVal;
      fprintf(out, "#%jd %d %2s %08x %8s f %02x         %08x  %s",
	      uintmax_t(tag), ix, pmStr.data(), pc, opcode.data(), fpReg, val32,
	      assembly.data());
    }
}

template <>
void
formatFpInstTrace<uint64_t>(FILE* out, uint64_t tag, const Hart<uint64_t>& hart,
			    std::string_view opcode, unsigned fpReg,
			    uint64_t fpVal, unsigned width,
			    std::string_view assembly)
{
  unsigned ix = hart.sysHartIndex();
  uint64_t pc = hart.lastPc();
  std::string_view pmStr = privilegeModeToStr(hart);

  if (width == 64)
    {
      fprintf(out, "#%jd %d %2s %016jx %8s f %016jx %016jx %s",
	      uintmax_t(tag), ix, pmStr.data(), uintmax_t(pc), opcode.data(),
	      uintmax_t(fpReg), uintmax_t(fpVal), assembly.data());
    }
  else
    {
      uint32_t val32 = fpVal;
      fprintf(out, "#%jd %d %2s %016jx %8s f %016jx         %08x %s",
	      uintmax_t(tag), ix, pmStr.data(), uintmax_t(pc), opcode.data(),
	      uintmax_t(fpReg), val32, assembly.data());
    }
}


template <typename URV>
static
void
printPageTableWalk(FILE* out, const Hart<URV>& hart, const char* tag,
		   const VirtMem::Walk& walk, bool /*steeEnabled*/, const Stee &stee)
{
  if (walk.size() == 0)
    return;

  fputs(tag, out);
  fputs(":", out);
  fputs("  +\n", out);

  if (walk.isTwoStage())
    fputs(walk.isStage2() ? " gpa: " : "gva: ",  out);
  else
    fputs(" gpa: ", out);
  fprintf(out, "0x%" PRIx64, walk.start());

  const char* indent = "  ";
  if (walk.isTwoStage())
    indent = walk.isStage2() ? "  "  :  " ";

  for (size_t i = 0; i < walk.size(); ++i)
    {
      fputs("  +\n", out);
      auto addr = walk.ithPteAddr(i);
      uint64_t effAddr = addr;   // Addr after clearing STEE bit
      effAddr = stee.clearSecureBits(addr);

      fputs(indent, out);
      fprintf(out, "0x%" PRIx64, addr);
      if (walk.isStage1())
        {
          addr = walk.ithPteSpaAddr(i);
          if (i+1 == walk.size() and not walk.stage1TailPteValid())
            continue;
          fputs("  +\n", out);
          fputs(indent, out); fputs(indent, out);
          effAddr = stee.clearSecureBits(addr);
          fprintf(out, "0x%" PRIx64, addr);
        }

      uint64_t pte = walk.ithPte(i);
      fprintf(out, "=0x%" PRIx64, pte);

      Pma pma = hart.getPma(effAddr);
      if (i+1 == walk.size())
        pma = hart.overridePmaWithPbmt(pma, walk.pbmt());
      fprintf(out, ", ma=%s", Pma::attributesToString(pma.attributesToInt()).c_str());
    }

  if (walk.hasException())
    return;  // Do not print translation result if walk was not successful.

  fputs("  +\n", out);

  indent = " ";
  if (walk.isTwoStage())
    indent = walk.isStage2() ? " " : "";
  fputs(indent, out);
  fputs("res:", out);
  fprintf(out, "0x%" PRIx64, walk.result());
}


template <typename URV>
void
Hart<URV>::printInstTrace(uint32_t inst, uint64_t tag, std::string& tmp,
			  FILE* out)
{
  if (not out and not __tracerExtension)
    return;

  uint32_t ix = (currPc_ >> 1) & decodeCacheMask_;
  DecodedInst* dip = &decodeCache_[ix];
  if (dip->isValid() and dip->address() == currPc_)
    printDecodedInstTrace(*dip, tag, tmp, out);
  else
    {
      DecodedInst di;
      uint64_t physPc = currPc_;
      decode(currPc_, physPc, inst, di);
      printDecodedInstTrace(di, tag, tmp, out);
    }
}


template <typename URV>
void
Hart<URV>::printDecodedInstTrace(const DecodedInst& di, uint64_t tag, std::string& tmp,
                                 FILE* out)
{
  if (instCounter_ < logStart_)
    return;

  if (__tracerExtension)
    {
      TraceRecord<URV> tr(this, di);
      __tracerExtension(&tr);
    }

  if (not out)
    return;

  if (not traceOn_)
    return;

  if (csvTrace_)
    {
      printInstCsvTrace(di, out);
      return;
    }

  // Serialize to avoid jumbled output.
  auto lock = (ownTrace_)? std::unique_lock<std::mutex>() : std::unique_lock<std::mutex>(printInstTraceMutex());

  if (logLabelEnabled_)
    {
      // Get the symbol label corresponding to the instruction’s PC:
      std::string label;
      if (memory_.findSymbolByAddress(di.address(), label))
        fprintf(out, "%s:\n", label.c_str());
    }

  disassembleInst(di, tmp);
  if (hasInterrupt_)
    tmp += " (interrupted)";

  if (ldStSize_)
    {
      std::ostringstream oss;
      oss << "0x" << std::hex << ldStAddr_;
      if (ldStPhysAddr1_ != ldStAddr_)
	oss << ":0x" << ldStPhysAddr1_;
      tmp += " [" + oss.str() + "]";
    }
  else
    {
      auto& vecInfo = getLastVectorMemory();
      if (not vecInfo.empty())
	{
          int num_nibbles = vecInfo.elemSize_*2;
	  std::ostringstream oss;
	  auto& elems = vecInfo.elems_;
          const auto* sep = "";
	  for (uint64_t i = 0; i < elems.size(); ++i)
	    {
	      auto& einfo = elems.at(i);
              if (not vecInfo.isLoad_ and einfo.skip_)
                continue;  // Non-active vector store element.
              oss << sep;
              sep = ";";
	      oss << "0x" << std::hex << einfo.va_;
	      if (einfo.pa_ != einfo.va_)
		oss << ":0x" << einfo.pa_;
	      if (not vecInfo.isLoad_)
		oss << '=' << "0x" << std::setfill('0') << std::setw(num_nibbles) << einfo.data_;
	    }
	  tmp += " [" + oss.str() + "]";
	}
    }

  std::array<char, 128> instBuff{};
  int                   instLen = 0;
  if (di.instSize() == 4)
    instLen = snprintf(instBuff.data(), instBuff.size(), "%08x", di.inst());
  else
    instLen = snprintf(instBuff.data(), instBuff.size(), "%04x", di.inst() & 0xffff);
  std::string_view instSV = std::string_view(instBuff.begin(), instLen);

  bool pending = false;  // True if a printed line need to be terminated.

  // Order: rfvmc (int regs, fp regs, vec regs, memory, csr)

  // Process integer register diff.
  int reg = intRegs_.getLastWrittenReg();
  URV value = 0;
  if (reg > 0)
    {
      bool twoRegsUpdated = (
        (di.instId() == InstId::amocas_q) or
        (di.instId() == InstId::amocas_d and sizeof(URV) == 4)
      );
      if (twoRegsUpdated)
	{
	  assert((reg & 1) == 1);
	  value = intRegs_.read(reg - 1);
	  formatInstTrace<URV>(out, tag, *this, instSV, 'r', reg - 1, value, tmp);
	  fprintf(out, " +\n");
	  pending = true;
	}
      value = intRegs_.read(reg);
      formatInstTrace<URV>(out, tag, *this, instSV, 'r', reg, value, tmp);
      pending = true;
    }

  // Process floating point register diff.
  int fpReg = fpRegs_.getLastWrittenReg();
  if (fpReg >= 0)
    {
      uint64_t val = fpRegs_.readBitsRaw(fpReg);
      if (pending) fprintf(out, "  +\n");
      unsigned width = isRvd() ? 64 : 32;
      formatFpInstTrace<URV>(out, tag, *this, instSV, fpReg, val, width, tmp);
      pending = true;
    }

  // Process vector register diff.
  unsigned groupSize = 0;
  int vecReg = lastVecReg(di, groupSize);
  if (vecReg >= 0)
    {
      for (unsigned i = 0; i < groupSize; ++i, ++vecReg)
        {
          if (pending)
            fprintf(out, " +\n");
          formatVecInstTrace<URV>(out, tag, *this, instSV,
                                  vecReg, vecRegs_.getVecData(vecReg),
                                  vecRegs_.bytesPerRegister(),
                                  tmp);
          pending = true;
        }
    }

  // Process memory diff.
  if (di.instId() == InstId::cbo_zero and not hasException_)
    {
      uint64_t addr = cacheLineAlign(ldStAddr_);
      for (unsigned i = 0; i < cacheLineSize_; i += sizeof(URV))
	{
	  if (pending)
	    fprintf(out, "  +\n");
	  formatInstTrace<URV>(out, tag, *this, instSV, 'm',
			       URV(addr + i), URV(0), tmp);
	  pending = true;
	}
    }
  else if (ldStWrite_ and not di.isVector())
    {
      if (pending)
        fprintf(out, "  +\n");
      formatInstTrace<URV>(out, tag, *this, instSV, 'm',
                           URV(ldStAddr_), URV(ldStData_), tmp);
      pending = true;
    }

  // Process CSR diffs.
  std::vector<CsrNumber> csrs;
  std::vector<unsigned> triggers;
  csRegs_.getLastWrittenRegs(csrs, triggers);

  using CVP = std::pair<URV, URV>;  // CSR-value pair
  std::vector< CVP > cvps; // CSR-value pairs
  cvps.reserve(csrs.size() + triggers.size());

  // Collect changed CSRs and their values.
  for (CsrNumber csr : csrs)
    {
      // We always record the real csr number for VS/S mappings
      if (not csRegs_.peek(csr, value, false))
        continue;
      cvps.push_back(CVP(URV(csr), value));
    }

  // Sort by CSR number.
  std::sort(cvps.begin(), cvps.end(), [] (const CVP& a, const CVP& b) {
      return a.first < b.first; });

  for (const auto& cvp : cvps)
    {
      if (pending) fprintf(out, "  +\n");
      formatInstTrace<URV>(out, tag, *this, instSV, 'c',
                           cvp.first, cvp.second, tmp);
      pending = true;
    }

  if (not pending)
    formatInstTrace<URV>(out, tag, *this, instSV, 'r', 0, 0,
                         tmp);  // No change: emit X0 as modified reg.

  if (tracePtw_)
    {
      if (not virtMem_.getFetchWalks().empty() or not virtMem_.getDataWalks().empty())
        {
          fprintf(out, "  +\nsatp mode: %4s", VirtMem::to_string(lastPageMode_).data());
          fprintf(out, "  +\nvsatp mode: %4s", VirtMem::to_string(lastVsPageMode_).data());
          fprintf(out, "  +\nhgatp mode: %4s", VirtMem::to_string(lastPageModeStage2_).data());
        }

      for (const auto& walk : virtMem_.getFetchWalks())
        {
          fputs("  +\n", out);
          printPageTableWalk(out, *this, "iptw", walk, steeEnabled_, stee_);
        }

      for (const auto& walk : virtMem_.getDataWalks())
        {
          fputs("  +\n", out);
          printPageTableWalk(out, *this, "dptw", walk, steeEnabled_, stee_);
        }
    }
  fputs("\n", out);
}


// We use custom code for speed. This makes a big difference in
// runtime for large traces.
namespace Whisper
{
  class PrintBuffer
  {
  public:

    PrintBuffer() = default;

    // Append to buffer hexadecimal string representing given number.
    inline
    PrintBuffer& print(uint64_t num)
    {
      if (num == 0)
	buff_.at(pos_++) = '0';
      else
	{
	  size_t beg = pos_;
	  for ( ; num; num = num >> 4 )
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
	    buff_.at(pos_++) = "0123456789abcdef"[num&0xf];
	  std::reverse(buff_.begin() + beg, buff_.begin() + pos_);
	}
      return *this;
    }

    // Append to buffer copy of given string excluding null char
    inline
    PrintBuffer& print(const char* str)
    {
      if (str)
	while (*str)
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
	  buff_.at(pos_++) = *str++;
      return *this;
    }

    // Append to buffer copy of given string excluding null char
    inline
    PrintBuffer& print(std::string_view str)
    {
      std::size_t amountToCopy = std::min(buff_.size() - pos_, str.size());

      std::copy_n(str.begin(), amountToCopy, std::next(buff_.begin(), static_cast<std::ptrdiff_t>(pos_)));
      pos_ += amountToCopy;
      return *this;
    }

    // Append char to buffer.
    inline
    PrintBuffer& printChar(char c)
    {
      buff_.at(pos_++) = c;
      return *this;
    }

    // Write contents of buffer to given file.
    inline
    void write(FILE* out)
    {
      size_t n = fwrite(buff_.data(), pos_, 1, out);
      (void)n;
      assert(n == 1);
    }

    // Clear contents of buffer.
    inline
    void clear()
    { pos_ = 0; }

  private:
    using Buff = std::array<char, UINT64_C(12)*4096>;

    Buff buff_{};
    size_t pos_ = 0;
  };
}


template <typename URV>
void
Hart<URV>::printInstCsvTrace(const DecodedInst& di, FILE* out)
{
  static Whisper::PrintBuffer sharedBuffer;
  static std::unordered_map<unsigned, Whisper::PrintBuffer> ownedBuffers;

  // Serialize to avoid jumbled output.
  auto lock = (ownTrace_)? std::unique_lock<std::mutex>() : std::unique_lock<std::mutex>(printInstTraceMutex());
  auto buffer = (ownTrace_)? ownedBuffers[sysHartIndex()] : sharedBuffer;

  if (not traceHeaderPrinted_)
    {
      traceHeaderPrinted_ = true;
      fprintf(out, "pc, inst, modified regs, source operands, memory, inst info, privilege, trap, disassembly, hartid");
      if (tracePtw_)
        fprintf(out, ", iptw, dptw");
      fprintf(out, "\n");
    }

  buffer.clear();

  // Program counter.
  uint64_t virtPc = di.address(), physPc = di.physAddress();
  buffer.print(virtPc);
  if (physPc != virtPc)
    buffer.printChar(':').print(physPc);

  // Instruction.
  buffer.printChar(',').print(di.inst()).printChar(',');

  // Changed integer register.
  unsigned regCount = 0;
  int reg = lastIntReg();
  uint64_t val64 = 0;
  if (reg > 0)
    {
      val64 = peekIntReg(reg);
      buffer.print(IntRegs<URV>::regName(reg)).printChar('=').print(val64);
      regCount++;
    }

  // Changed fp register.
  reg = lastFpReg();
  if (reg >= 0)
    {
      peekFpReg(reg, val64);
      if (not isRvd())
        val64 = uint32_t(val64);  // Clear top 32 bits if only F extension.
      if (regCount) buffer.printChar(';');
      buffer.print(FpRegs::regName(reg)).printChar('=').print(val64);
      // Print incremental flags since FRM is sticky.
      unsigned fpFlags = lastFpFlags();
      if (fpFlags != 0)
        buffer.print(";ff=").print(fpFlags);
      regCount++;
    }

  // Changed CSR register(s).
  std::vector<CsrNumber> csrns;
  lastCsr(csrns);
  for (auto csrn : csrns)
    {
      URV val = peekCsr(csrn);
      if (regCount) buffer.printChar(';');
      buffer.printChar('c').print(std::to_string(unsigned(csrn))).printChar('=').print(val);
      regCount++;
    }

  // Changed vector register group.
  unsigned groupSize = 0;
  int vecReg = lastVecReg(di, groupSize);
  if (vecReg >= 0)
    {
      for (unsigned i = 0; i < groupSize; ++i, ++vecReg)
        {
          if (regCount) buffer.printChar(';');
          buffer.printChar('v').print(std::to_string(vecReg)).printChar('=');
          auto data = vecRegs_.getVecData(vecReg);
          unsigned byteCount = vecRegs_.bytesPerRegister();
          for (unsigned i = 0; i < byteCount; ++i)
            {
              unsigned byte = data[byteCount - 1 - i];
              if (byte < 16) buffer.printChar('0');
              buffer.print(byte);
            }
          regCount++;
        }
    }

  // Non sequential PC change.
  const auto* instEntry = di.instEntry();
  bool hasTrap = hasInterrupt_ or hasException_;
  if (not hasTrap and instEntry->isBranch() and lastBranchTaken_)
    {
      if (regCount) buffer.printChar(';');
      buffer.print("pc=").print(pc_);
    }

  // Source operands.
  buffer.printChar(',');
  const char* sep = "";
  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = instEntry->ithOperandMode(i);
      auto type = instEntry->ithOperandType(i);
      if (mode == OperandMode::Read or mode == OperandMode::ReadWrite or
          type == OperandType::Imm)
        {
          unsigned operand = di.ithOperand(i);
          if (type ==  OperandType::IntReg)
            buffer.print(sep).print(IntRegs<URV>::regName(operand));
          else if (type ==  OperandType::FpReg)
            buffer.print(sep).print(FpRegs::regName(operand));
          else if (type == OperandType::CsReg)
            buffer.print(sep).printChar('c').print(std::to_string(operand));
          else if (type == OperandType::VecReg)
            {
              buffer.print(sep).printChar('v').print(std::to_string(operand));
              unsigned emul = i < vecRegs_.opsEmul_.size() ? vecRegs_.opsEmul_.at(i) : 1;
              if (emul >= 2 and emul <= 8)
                buffer.printChar('m').print(emul);
            }
          else if (type == OperandType::Imm)
            buffer.print(sep).printChar('i').print(operand);
          sep = ";";
        }
    }

  // Print rounding mode with source operands.
  if (instEntry->hasRoundingMode())
    {
      RoundingMode rm = effectiveRoundingMode(di.roundingMode());
      buffer.print(sep).print("rm=").print(unsigned(rm));
      sep = ";";
    }

  // Memory
  buffer.printChar(',');
  uint64_t virtDataAddr = 0, physDataAddr = 0;
  auto& vecInfo = getLastVectorMemory();
  if (not vecInfo.empty())
    {
      auto& elems = vecInfo.elems_;
      for (uint64_t i = 0; i < elems.size(); ++i)
        {
	  auto& einfo = elems.at(i);
          if (i > 0)
            buffer.printChar(';');
          buffer.print(einfo.va_);
          if (einfo.pa_ != einfo.va_)
            buffer.printChar(':').print(einfo.pa_);
          if (einfo.skip_)
            buffer.printChar('m');
          if (not vecInfo.isLoad_)
            buffer.printChar('=').print(einfo.data_);
        }
    }
  if (di.instId() == InstId::cbo_zero and not hasException_)
    {
      uint64_t va = cacheLineAlign(ldStAddr_);
      uint64_t pa = cacheLineAlign(ldStPhysAddr1_);
      for (unsigned i = 0; i < cacheLineSize_; i += sizeof(URV))
	{
          if (i > 0)
            buffer.printChar(';');
          buffer.print(va + i);
          if (va != pa)
            {
              buffer.printChar(':');
              buffer.print(pa + i);
            }
          buffer.print("=0");
	}
    }
  else if (lastLdStAddress(virtDataAddr, physDataAddr))
    {
      bool store = ldStWrite_;
      buffer.print(virtDataAddr);
      if (physDataAddr != virtDataAddr)
        buffer.printChar(':').print(physDataAddr);
      if (store)
        buffer.printChar('=').print(ldStData_);
    }

  // Instruction information.
  buffer.printChar(',');
  RvExtension type = instEntry->extension();
  if (type == RvExtension::A)
    buffer.printChar('a');
  else if (instEntry->isLoad())
    buffer.printChar('l');
  else if (instEntry->isStore())
    buffer.printChar('s');
  else if (instEntry->isBranch())
    {
      if (instEntry->isConditionalBranch())
        buffer.print(lastBranchTaken_ ? "t" : "nt");
      else
        {
          if (di.isReturn())
            buffer.printChar('r');
          else if (di.isCall())
            buffer.printChar('c');
          else
            buffer.printChar('j');
        }
    }
  else if (type == RvExtension::F or type == RvExtension::D or type == RvExtension::Zfh or
           type == RvExtension::Zfbfmin)
    buffer.printChar('f');
  else if (instEntry->isVector())
    buffer.printChar('v');

  // Privilege mode.
  if      (lastPriv_ == PrivilegeMode::Machine)    buffer.print((lastDm_)? ",d," : ",m,");
  else if (lastPriv_ == PrivilegeMode::Supervisor) buffer.print((lastVirt_)? ",vs," : ",s,");
  else if (lastPriv_ == PrivilegeMode::User)       buffer.print((lastVirt_)? ",vu," : ",u,");
  else                                             buffer.print(",,");

  // Interrupt/exception cause.
  if (hasTrap)
    {
      URV cause = 0;
      if (nmiPending_)
        cause = peekCsr(CsrNumber::MNCAUSE);
      else if (privilegeMode() == PrivilegeMode::Machine)
        cause = peekCsr(CsrNumber::MCAUSE);
      else if (privilegeMode() == PrivilegeMode::Supervisor)
        cause = peekCsr(CsrNumber::SCAUSE);
      buffer.print(uint64_t(cause));
    }
  buffer.printChar(',');

  // Disassembly.
  std::string tmp;
  disassembleInst(di, tmp);
  std::ranges::replace(tmp, ',', ';');
  buffer.print(tmp);

  // Hart Id.
  buffer.printChar(',').print(sysHartIndex());

  // Page table walk.
  if (tracePtw_)
    {
      buffer.printChar(',');

      auto printWalks = [this, &buffer] (bool isFetch) {
        const auto& walks = isFetch ? getFetchPageTableWalks() : getDataPageTableWalks();
        if (walks.empty())
          return;
        const auto* sep = "";

        for (const auto& walk : walks)
          {
            buffer.print(sep).print(walk.start());
            sep = ";";

            const auto& addrs = walk.pteAddrs();
            const auto& ptes = walk.pteValues();

            for (size_t i = 0; i < addrs.size(); ++i)
              {
                auto addr = addrs.at(i);
                if (steeEnabled_)
                  addr = stee_.clearSecureBits(addr);
                buffer.print(sep).print(addr);
                buffer.printChar('=').print(ptes.at(i));

                Pma pma = getPma(addr);
                if (i+1 == addrs.size())
                  pma = overridePmaWithPbmt(pma, walk.pbmt());
                buffer.print(";ma=").print(pma.attributesToInt());
              }

            buffer.print(sep).print(walk.result());
          }
      };

      printWalks(true);  // Instruction fetch walks.
      buffer.printChar(',');

      printWalks(false); // Data access walks.
    }

  buffer.printChar('\n');
  buffer.write(out);
}


/// Report the number of executed and retired instruction count and the simulation
/// rate.
template <typename URV>
void
Hart<URV>::reportInstsPerSec(uint64_t instCount, uint64_t retInstCount, double elapsed,
                             bool userStop)
{
  std::lock_guard<std::mutex> guard(printInstTraceMutex());

  std::cout.flush();

  std::array<char, 20> secBuf{};
  int                  secLen = snprintf(secBuf.data(), secBuf.size(), "%.2fs", elapsed);
  std::string_view     secStr = std::string_view(secBuf.begin(), secLen);

  if (userStop)
    std::cerr << "Info: User stop\n";
  std::cerr << "Info: Executed " << instCount << " instruction"
	    << (instCount > 1? "s" : "") << " and "
            << "retired " << retInstCount << " instruction"
            << (retInstCount > 1? "s" : "")
            << " in " << secStr;
  if (elapsed > 0)
    std::cerr << " -- " << uint64_t(double(instCount)/elapsed) << " inst/s";
  std::cerr << " hart=" << hartIx_ << '\n';

  if (hasRoiTraceEnabled())
    std::cerr << "Info: Traced " << traceCount_ << " instructions\n";
}


template <typename URV>
bool
Hart<URV>::logStop(const CoreException& ce, uint64_t counter, FILE* traceFile)
{
  bool success = false;
  bool isRetired = false;

  if (ce.type() == CoreException::Stop)
    {
      isRetired = true;
      success = (ce.value() >> 1) == 0;
      setTargetProgramFinished(true);
    }
  else if (ce.type() == CoreException::Exit)
    {
      isRetired = true;
      success = ce.value() == 0;
      setTargetProgramFinished(true);
    }
  else if (ce.type() == CoreException::Snapshot or
           ce.type() == CoreException::RoiEntry)
    {
      isRetired = true;
      success = true;
    }
  else if (ce.type() == CoreException::SnapshotAndStop)
    {
      isRetired = true;
      success = true;
      setTargetProgramFinished(true);
    }

  if (isRetired)
    {
      if (minstretEnabled())
        retiredInsts_++;

      uint32_t inst = 0;
      readInst(currPc_, inst);
      std::string instStr;
      printInstTrace(inst, counter, instStr, traceFile);
    }

  using std::cerr;

  {
    std::lock_guard<std::mutex> guard(printInstTraceMutex());

    cerr << std::dec;
    if (ce.type() == CoreException::Stop)
      cerr << (success? "Successful " : "Error: Failed ")
           << "stop: Hart " << hartIx_ << ": " << ce.what() << "\n";
    else if (ce.type() == CoreException::Exit)
      cerr << "Info: Target program exited with code " << ce.value() << '\n';
    else if (ce.type() == CoreException::Snapshot)
      cerr << "Info: Attempting to snapshot\n";
    else if (ce.type() == CoreException::RoiEntry)
      cerr << "Info: Entering ROI\n";
    else if (ce.type() == CoreException::SnapshotAndStop)
      cerr << "Info: Successful stop: Hart " << hartIx_ << ": attempting to snapshot and stop\n";
    else
      cerr << "Error: Stopped -- unexpected exception\n";
  }

  return success;
}


template <typename URV>
void
Hart<URV>::printInstructions(FILE* file) const
{
  for (unsigned i = 0; i <= unsigned(InstId::maxId); ++i)
    {
      auto id = InstId(i);
      const auto& entry = decoder_.getInstructionEntry(id);
      auto extension = entry.extension();
      if (isa_.isEnabled(extension))
        {
          auto name = entry.name();
          fprintf(file, "%s\n", name.data());
        }
    }
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
