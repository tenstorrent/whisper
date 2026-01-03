#pragma once

#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_streambuf.hpp>

namespace WhisperUtil  {

  // Operand type: Integer-register, floating-point register, control
  // and status register, vector register, or immediate value.
  enum class OperandType { Int, Fp, Csr, Vec, Imm };

  // Processor privilege mod.
  enum class PrivMode { Machine, Supervisor, User };

  class PageTableMaker;

  // Enum for the major columns of the log file.
  enum class HeaderTag { Pc, Inst, DestRegs, SourceOps,
    Memory, InstType, Priv, Trap, Dis, HartId, Iptw, Dptw, Pmp, _Count
  };

  // Model an instruction operand.
  struct Operand
  {
    OperandType type = OperandType::Int;
    unsigned number = 0;                  // Register number.
    unsigned emul = 1;                    // Effective group multiplier for vector register.
    uint64_t value = 0;
    uint64_t prevValue = 0;               // Used for modified registers.
    std::vector<uint8_t> vecValue;        // Used for vector registers
    std::vector<uint8_t> vecPrevValue;    // Used for modified vector registers

    bool operator==(const Operand& rhs) const {
      if (this->type != rhs.type) return false;
      if (this->number != rhs.number) return false;
      if (this->emul != rhs.emul) return false;
      if (number != 256)
        if (this->value != rhs.value) return false;

      return true;
    }

    bool operator!=(const Operand& rhs) const {
      if (this->type != rhs.type) return true;
      if (this->number != rhs.number) return true;
      if (this->emul != rhs.emul) return true;
      if (number != 256)
        if (this->value != rhs.value) return true;

      return false;
    }

    // bool operator<(const TraceRecord &rhs) const {
    //   return number < rhs.number;
    // }
  };

  // Model a record in the log file.
  struct TraceRecord
  {
    uint64_t virtPc = 0, physPc = 0;
    uint64_t takenBranchTarget = 0;
    uint32_t inst = 0;
    uint8_t instSize = 0;
    uint8_t instType = 0;
    uint8_t dataSize = 0;     // For ld/st instructions
    uint8_t fpFlags = 0;
    uint8_t roundingMode = 0;
    std::vector<Operand> modifiedRegs;
    std::vector<Operand> sourceOperands;
    std::vector<std::pair<unsigned, uint64_t>> contextCSRs;
    std::vector<uint64_t> virtAddrs;  // Memory addresses
    std::vector<uint64_t> physAddrs;  // Memory addresses
    std::vector<uint64_t> memVals;    // Correspondign data for store
    std::vector<bool> maskedAddrs;    // Maked addresses (for vector instructions).
    std::unordered_map<uint64_t, std::vector<uint64_t>> dpteAddrs;  // PTE addresses address translation.
    std::unordered_map<uint64_t, std::vector<uint64_t>> ipteAddrs;

    PrivMode priv = PrivMode::Machine;
    bool virt = false;
    bool hasTrap = false;
    uint64_t trap = 0;
    std::string assembly;

    bool operator==(const TraceRecord& rhs) const {
      if (virtPc != rhs.virtPc) return false;
      if (physPc != rhs.physPc) return false;
      if (takenBranchTarget != rhs.takenBranchTarget) return false;
      if (inst != rhs.inst) return false;
      if (instSize != rhs.instSize) return false;
      if (instType != rhs.instType) return false;
      if (dataSize != rhs.dataSize) return false;
      if (fpFlags != rhs.fpFlags) return false;
      if (roundingMode != rhs.roundingMode) return false;

      // compare modified regs, the order may or may not match
      if (modifiedRegs.size() != rhs.modifiedRegs.size()) return false;
      for (const auto &it: modifiedRegs) {
        if (std::find(rhs.modifiedRegs.begin(), rhs.modifiedRegs.end(), it) == rhs.modifiedRegs.end()) {
          return false;
        }
      }

      // compare source operands, the order may or may not match
      if (sourceOperands.size() != rhs.sourceOperands.size()) return false;
      for (const auto &it : sourceOperands) {
        if (std::find(rhs.sourceOperands.begin(), rhs.sourceOperands.end(), it) == rhs.sourceOperands.end()) {
          return false;
        }
      }
      
      // compare contextCSRs, ignoring the value for now
      if (contextCSRs.size() != rhs.contextCSRs.size()) return false;
      for (const auto &it : contextCSRs) {
        auto rhsit = std::find_if(rhs.contextCSRs.begin(), rhs.contextCSRs.end(), [&](const std::pair<unsigned, uint64_t> &rhs) {
          return rhs.first == it.first;
        });
        if (rhsit == rhs.contextCSRs.end()) return false;
      }

      if (virtAddrs != rhs.virtAddrs) return false;
      if (physAddrs != rhs.physAddrs) return false;
      if (maskedAddrs != rhs.maskedAddrs) return false;

      // compare dpteAddrs between the records
      if (dpteAddrs.size() != rhs.dpteAddrs.size()) return false;
      for (const auto &it : dpteAddrs) {
        auto rhsit = std::find(rhs.dpteAddrs.begin(), rhs.dpteAddrs.end(), it);
        if (rhsit == rhs.dpteAddrs.end()) return false;
      }

      if (ipteAddrs != rhs.ipteAddrs) return false;
      if (priv != rhs.priv) return false;
      if (virt != rhs.virt) return false;
      if (hasTrap != rhs.hasTrap) return false;
      if (trap != rhs.trap) return false;
      if (assembly != rhs.assembly) return false;

      return true;
    }

    // bool operator!=(const TraceRecord&) const = default;
    bool operator!=(const TraceRecord& rhs) const {
      if (virtPc != rhs.virtPc) return true;
      if (physPc != rhs.physPc) return true;
      if (takenBranchTarget != rhs.takenBranchTarget) return true;
      if (inst != rhs.inst) return true;
      if (instSize != rhs.instSize) return true;
      if (instType != rhs.instType) return true;
      if (dataSize != rhs.dataSize) return true;
      if (fpFlags != rhs.fpFlags) return true;
      if (roundingMode != rhs.roundingMode) return true;

      // compare modified regs, the order may or may not match
      if (modifiedRegs.size() != rhs.modifiedRegs.size()) return true;
      for (const auto &it: modifiedRegs) {
        if (std::find(rhs.modifiedRegs.begin(), rhs.modifiedRegs.end(), it) == rhs.modifiedRegs.end()) {
          return true;
        }
      }

      // compare source operands, the order may or may not match
      if (sourceOperands.size() != rhs.sourceOperands.size()) return true;
      for (const auto &it : sourceOperands) {
        if (std::find(rhs.sourceOperands.begin(), rhs.sourceOperands.end(), it) == rhs.sourceOperands.end()) {
          return true;
        }
      }

      if (contextCSRs.size() != rhs.contextCSRs.size()) return true;
      for (const auto &it : contextCSRs) {
        auto rhsit = std::find_if(rhs.contextCSRs.begin(), rhs.contextCSRs.end(), [&](const std::pair<unsigned, uint64_t> &rhs) {
          return rhs.first == it.first;
        });
        if (rhsit == rhs.contextCSRs.end()) return true;
      }

      if (virtAddrs != rhs.virtAddrs) return true;
      if (physAddrs != rhs.physAddrs) return true;
      if (maskedAddrs != rhs.maskedAddrs) return true;

      if (dpteAddrs.size() != rhs.dpteAddrs.size()) return true;
      for (const auto &it : dpteAddrs) {
        auto rhsit = std::find(rhs.dpteAddrs.begin(), rhs.dpteAddrs.end(), it);
        if (rhsit == rhs.dpteAddrs.end()) return true;
      }

//       if (ipteAddrs != rhs.ipteAddrs) return true;
      if (priv != rhs.priv) return true;
      if (virt != rhs.virt) return true;
      if (hasTrap != rhs.hasTrap) return true;
      if (trap != rhs.trap) return true;
      if (assembly != rhs.assembly) return true;

      return false;
    }

    // Clear this record.
    void clear();

    // Return true if this is a floating point instruction.
    bool isFp() const
    { return instType == 'f'; }

    // Return true if this is a vector instruction.
    bool isVector() const
    { return instType == 'v'; }

    // Return true if this is an atomic instruction.
    bool isAtomic() const
    { return instType == 'a'; }

    // Return true if this is a scalar load instruction.
    bool isLoad() const
    { return instType == 'l'; }

    // Return true if this is a scalar store instruction.
    bool isStore() const
    { return instType == 's'; }

    // Return true if this is a vector load instruction.
    bool isVecLoad() const
    { return isVector() and (inst & 0x7f) == 0x7; }

    // Return true if this is a vector store instruction.
    bool isVecStore() const
    { return isVector() and (inst & 0x7f) == 0x27; }

    // Return true if this is a vector load/store unit-stride
    // instruction.
    bool isVecUnitStride() const
    { return (isVecLoad() or isVecStore()) and (((inst >> 26) & 0x3) == 0); }

    // Return true if this is a vector load/store indexed
    // unordered instruction.
    bool isVecIndexedUnordered() const
    { return (isVecLoad() or isVecStore()) and (((inst >> 26) & 0x3) == 1); }

    // Return true if this is a vector load/store strided
    // instruction.
    bool isVecStride() const
    { return (isVecLoad() or isVecStore()) and (((inst >> 26) & 0x3) == 2); }

    // Return true if this is a vector load/store indexed
    // ordered instruction.
    bool isVecIndexedOrdered() const
    { return (isVecLoad() or isVecStore()) and (((inst >> 26) & 0x3) == 3); }

    // Return true if this is a call instruction.
    bool isCall() const
    { return instType == 'c'; }

    // Return true if this is a return instruction.
    bool isReturn() const
    { return instType == 'r'; }

    // Return true if this is a jump instruction (excluding call/return).
    bool isJump() const
    { return instType == 'j'; }

    // Return true if this is a conditional branch instruction.
    bool isConditionalBranch() const
    { return instType == 't' or instType == 'n'; }

    // Return true if this is a conditional branch instruction that is taken.
    bool isTakenConditionalBranch() const
    { return instType == 't'; }

    // Return true if this is a conditional branch instruction that is
    // not taken. This will return false if this is a conditional
    // branch that is taken or if the instruction is not a conditional
    // branch.
    bool isNotTakenConditionalBranch() const
    { return instType == 'n'; }

    // Return true if this is a cmo instruction.
    bool isCmo() const
    {
      uint64_t opcode = inst & 0xfff07fff;
      if (opcode == 0x00200f or
          opcode == 0x10200f or
          opcode == 0x20200f or
          opcode == 0x40200f)
        return true;
      return false;
    }

    // Return true if this an illegal instruction.
    bool isIllegal() const
    { return inst == 0 or ~inst == 0; }

    // Return the instruction name.
    std::string instructionName() const
    {
      auto pos = assembly.find(' ');
      if (pos == std::string::npos)
        return assembly;
      return assembly.substr(0, pos);
    }
  };


  // Reader for whisper CSV log file.
  // Samle usage:
  //   TraceRecord rec("log.csv");
  //   TraceReader reader;
  //   while (reader.nextRecord(record))
  //     {
  //        reader.printRecord(std::cout, record);
  //     }
  class TraceReader
  {
  public:

    // Constructor. Open given input file.
    TraceReader(const std::string& inputPath);

    // Constructor with register initialization.
    TraceReader(const std::string& inputPath, const std::string& initPath);

    // Destructor.
    ~TraceReader();

    // Return true if associated input stream if valid (good for
    // input).
    operator bool() const
    { return static_cast<bool> (fileStream_); }

    // Return true if asscoaited input stream is at end of file.
    bool eof() const
    { return fileStream_.eof(); }

    // Read and parse the next record in the input stream opened at
    // construction. Put the read data in the given record. Return
    // true on success and false on failure or end of file.
    bool nextRecord(TraceRecord& record);

    // Read and parse the next record in the input stream opened at
    // construction. Put the read data in the given record. Return
    // true on success and false on failure or end of file. Also store the line
    // that was parsed.
    bool nextRecord(TraceRecord& record, std::string& line);

    // Read and parse the next record in the input stream opened at
    // construction, only populating PC, instruction, and privilege mode.
    // This is a lightweight version of nextRecord for performance-critical
    // use cases. Return true on success and false on failure or end of file.
    bool nextRecordLightweight(TraceRecord& record);

    // Parse given non-header line putting the collected data in the
    // given record.
    bool parseLine(std::string& line, uint64_t lineNum,
		   TraceRecord& record);

    // Parse given non-header line extracting only PC, instruction, and
    // privilege mode into the given record. This is a lightweight version
    // of parseLine for performance-critical use cases.
    bool parseLineLightweight(std::string& line, uint64_t lineNum,
			TraceRecord& record);

    // Define the parameters of a page table generator. Once defined
    // page table walks can be generated by calling genPageTableWalk.
    // Return true on success and false on failure (we fail if addr
    // is not a multiple of the page size, if arenaSize is not
    // a multiple of the page size or is smaller than 1 page).
    template<class Mode>
    bool definePageTableMaker(uint64_t addr,
			      /* PageTableMaker:: */ Mode mode,
			      uint64_t arenaSize);

    // Generate a page table walk that would be suitable for
    // translating the given virtual address to the given physical
    // address. Page table pages are created as necessary in
    // the arena defined by definePageTableMaker.
    bool genPageTableWalk(uint64_t virAddr, uint64_t physAddr,
			  std::vector<uint64_t>& walk);

    // Parse header line setting up the indices corresponding to the
    // header tags. For example, if the header line consists of
    // "pc,inst", then the Pc index would be 0 and the Inst index would
    // be 1.
    bool extractHeaderIndices(const std::string& line, uint64_t lineNum);

    std::string getHeaderLine();

    // Return the current value of the given integer regiser. The
    // given regiser index must be less than 32.
    uint64_t intRegValue(unsigned ix) const
    { return intRegs_.at(ix); }

    // Return the bits of current value of the given floating point
    // regiser.  The given regiser index must be less than 32.
    uint64_t fpRegValue(unsigned ix) const
    { return fpRegs_.at(ix); }

    // Return the current value of the given CSR. Return 0 if the CSR
    // is not in the trace. The given regiser index must be less than
    // 4096.
    uint64_t csrValue(unsigned ix) const
    { return csRegs_.at(ix); }

    // Return the current value of the given vector regiser.  The
    // given regiser index must be less than 32. Note that we return a
    // reference and the referenced data changes with each invocation
    // of nextRecord.
    const std::vector<uint8_t>& vecRegValue(unsigned ix) const
    { return vecRegs_.at(ix); }

    /// Return the current value of the vector start (VL) CSR.
    uint64_t vstartValue() const
    { return csrValue(0x8); }

    /// Return the current value of the vector length (VL) CSR.
    uint64_t vlValue() const
    { return csrValue(0xc20); }

    /// Return the current value of the vector type (VTYPE) CSR.
    uint64_t vtypeValue() const
    { return csrValue(0xc21); }

    /// Retrun the raw vector group multiplier (from the current value of the VTYPE CSR).
    /// Encoding: m1=0, m2=1, m4=2, m8=3, reserved=4, mf8=5, mf4=6, mf2=7
    unsigned rawLmul() const
    { return vtypeValue() & 7; }

    /// Return the group multiplier times 8: mf8=1, mf4=2, mf2=1, m1=8, m2=16, m4=32,
    /// m8=64.  Return zero if lmul value is reserved.
    unsigned groupMultiplierX8() const
    {
      unsigned raw = rawLmul();
      assert(raw < 8);

      if (raw == 4)
        return 0;

      if (raw < 4)
        return (1 << raw)*8;

      return (1 << (raw-5));
    }

    /// Return the raw SEW field  from the current value of the VTYPE CSR.
    unsigned rawSew() const
    { return vtypeValue() >> 3 & 7; }

    /// Return the vector element width (in bytes) from the current value of the VTYPE
    /// CSR.
    unsigned vecElemWidthInBytes() const
    { return 1 << rawSew(); }

    /// Return the tail aganostic (VTA) flag from the current value of the VTYPE CSR.
    bool tailAgnostic() const
    { return (vtypeValue() >> 6) & 1; }

    /// Return the mask aganostic (VMA) flag from the current value of the VTYPE CSR.
    bool maskAgnostic() const
    { return (vtypeValue() >> 7) & 1; }

    /// Return the illegal flag (VILL) from the current value of the VTYPE CSR. This is a
    /// hack. We need to know whether we are in RV32 or RV64 to do this right.
    bool vtypeVill() const
    { return ((vtypeValue() >> 31) & 1) | ((vtypeValue() >> 63) & 1); }

    void printRecord(std::ostream& stream, const TraceRecord& record) const;

  protected:

    /// Read the file containing initial vlues of registers.
    void readInitialState(const std::string& path);

    // Extract a pair of addresses from the given field. Return true on
    // success and false on failure.
    static
    bool extractAddressPair(uint64_t lineNum, const char* tag,
			    const char* pairString,
			    uint64_t& virt, uint64_t& phys,
			    bool& masked);

    // Parse the register value in the given value string into the given
    // operand.  Return true on success and false on failure. Update the
    // current values of regisers maintained by this parser.
    bool parseRegValue(uint64_t lineNum, char* regName,
		       char* valStr, Operand& operand);

    // Parse the given operand string into the given operand. Return true on
    // success and false on failure. Assing a value to the operand from the
    // maintained set of values in this parser.
    bool parseOperand(uint64_t lineNum, char* opStr,
		      Operand& operand);

    // Parse the memory string putting the resuts in the given
    // record. Return true on success and false on failure.
    bool parseMem(uint64_t lineNum, char* memStr,
		  TraceRecord& rec);

    bool splitLine(std::string& line, uint64_t lineNum);

  private:

    using VecReg = std::vector<uint8_t>;

    std::vector<uint64_t> intRegs_;
    std::vector<uint64_t> fpRegs_;
    std::vector<uint64_t> csRegs_;
    std::vector<VecReg>   vecRegs_;

    std::vector<char*> fields_;
    std::vector<char*> subfields_;
    std::vector<char*> keyvals_;

    std::vector<int> indices_; // Map a header tag to an index (field number in line).

    std::string headerLine_; // The original header line
    std::string line_;       // Buffer used to read file records.
    uint64_t lineNum_ = 0;   // Line number in input file.
    unsigned colCount_ = 0;  // Column count.

    PageTableMaker* pageMaker_ = nullptr;

    std::ifstream fileStream_;
    std::istream* input_ = nullptr;
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inStreambuf_;
  };
}

