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

#include <iostream>
#include <vector>
#include "System.hpp"
#include "Hart.hpp"
#include "DecodedInst.hpp"

namespace TT_PERF         // Tenstorrent Whisper Performance Model API
{

  using System64 = WdRiscv::System<uint64_t>;
  using Hart64 = WdRiscv::Hart<uint64_t>;
  using ExceptionCause = WdRiscv::ExceptionCause;
  using OperandType = WdRiscv::OperandType;
  using OperandMode = WdRiscv::OperandMode;
  using Walk = WdRiscv::VirtMem::Walk;

  /// Operand value.
  struct OpVal
  {
    uint64_t scalar = 0;        // For scalar/immediate operands.
    std::vector<uint8_t> vec;   // For vector operands.
  };

  /// Strucuture to recover the source/destination operands of an instruction packet.
  struct Operand
  {
    OperandType type = OperandType::IntReg;
    OperandMode mode = OperandMode::None;
    unsigned number = 0;  // Register number (0 for immediate operands).
    unsigned lmul = 0;    // Effective group multiplier. Valid for vector operand.
    OpVal value;          // Immediate or register value.
    OpVal prevValue;      // Used for modified registers.
  };

  class InstrPac;

  struct OpProducer
  {
    std::shared_ptr<InstrPac> scalar;               // Scalar operand
    std::vector<std::shared_ptr<InstrPac>> vec;     // Vector operand

    void clear()
    {
      scalar =  nullptr;
      vec.clear();
    }
  };

  /// Instruction packet.
  class InstrPac
  {
  public:

    friend class PerfApi;

    /// Constructor: iva/ipa are the instruction virtual/physical addresses.  For
    /// instruction crossing page boundary, ipa2 is the physical address of the other
    /// page. I not crossing page boundary ipa2 is same as ipa.
    InstrPac(uint64_t tag, uint64_t iva, uint64_t ipa, uint64_t ipa2)
      : tag_(tag), iva_(iva), ipa_(ipa), ipa2_(ipa2)
    { }

    ~InstrPac() = default;

    /// This supports PerfApi::shouldFlush. It is not meant to be called directly.
    bool shouldFlush() const
    { return shouldFlush_; }

    /// Return the instruction virtual address.
    uint64_t instrVa() const
    { return iva_; }

    /// Return the instruction physical address.
    uint64_t instrPa() const
    { return ipa_; }

    /// Set the instruction virtual address.
    void setInstrVa(uint64_t iva)
    { iva_ = iva; }

    /// For non-page crossing fetch return the same value as instrPa. For page-crossing
    /// return the physical address of the other page.
    uint64_t instrPa2() const
    { return ipa2_; }

    /// Return the data virtual address of a load/store instruction. Return 0 if
    /// instruction is not load/store.
    uint64_t dataVa() const
    { return dva_; }

    /// Return the data physical address of a load/store instruction. Return 0 if
    /// instruction is not load/store.
    uint64_t dataPa() const
    { return dpa_; }

    /// For non-page corssing load/store return the same value as dataPa. Return 0 if
    /// instruction is not load/store
    uint64_t dataPa2() const
    { return dpa2_; }

    /// Return the scalar store value
    uint64_t stData() const
    { return stData_; }

    /// Return a vector of pa/va/skip triplet corresponding to the
    /// virtual-addr/physical-addr/skip of the elements of the vector load/store
    /// instruction of this packet. The skip flag will be true if the corresponding element
    /// was skipped because it was masked-off or it was a tail element.  The return
    /// vector will be empty if the instruction is not a vector load/store or if no
    /// memory was accessed by the instruction.
    using VaPaSkip = std::tuple<uint64_t, uint64_t, bool>;
    const std::vector<VaPaSkip>& vecDataAddrs() const
    { return vecAddrs_; }

    /// Return the size of the instruction (2 or 4 bytes). Instruction must be fetched.
    uint64_t instrSize() const
    { assert(fetched_); return di_.instSize(); }

    /// Return the data size of a load/store instruction. Return 0 if instruction
    /// is not load/store. For vector load/store this will be the element size.
    uint64_t dataSize() const
    { return dsize_; }

    /// Return true if this is a branch instruction. Will return false if instruction is
    /// not decoded.
    bool isBranch() const
    { return di_.isBranch(); }

    /// Return true if this is a branch instruction where the target
    /// address is in a register.
    bool isBranchToRegister() const
    { return di_.isBranchToRegister(); }

    /// Return true if this branch instruction is taken. Will return false if
    /// instruction has not executed or is not a branch.
    bool isTakenBranch() const
    { return taken_; }

    /// Return branch target as determined by decode, even if the branch is not taken.
    /// Return 0 if the instruction is not decoded, not a branch, or is an indirect branch.
    uint64_t branchTargetFromDecode() const;

    /// Record the branch prediction made by the performance model. Return false if
    /// instruction is not a branch or is not decoded.
    bool predictBranch(bool taken, uint64_t target)
    {
      if (not decoded_ or not isBranch())
	return false;
      predicted_ = true;
      prTaken_ = taken;
      prTarget_ = target;
      return true;
    }

    /// Return true if this instruction depends on the given instruction.
    bool dependsOn(const InstrPac& other) const
    {
      assert(decoded_);
      for (unsigned i = 0; i < di_.operandCount(); ++i)
	{
	  using OM = WdRiscv::OperandMode;

	  auto mode = di_.ithOperandMode(i);
	  if (mode == OM::Read or mode == OM::ReadWrite)
	    {
	      const auto& producer = opProducers_.at(i);
	      if (producer.scalar and producer.scalar->tag_ == other.tag_)
		return true;
              for (const auto& entry : producer.vec)
                if (entry and entry->tag_ == other.tag_)
                  return true;
	    }
	}
      return false;
    }

    /// Return the decoded instruction object associated with this packet.
    const WdRiscv::DecodedInst& decodedInst() const
    { return di_; }

    /// Return the PC of the instruction following this instruction in program order.
    /// Only valid if instruction is executed.
    uint64_t nextPc() const
    { assert(executed_); return nextIva_; }

    /// Return true if instruction fetch or execute encountered a trap (exception or
    /// interrupt).
    bool trapped() const
    { return trap_; }

    /// Return true if instruction encountered an interrupt.
    bool interupted() const
    { return interrupt_; }

    /// Return the trap cause. Valid only if instruction was trapped. Returned value will
    /// have its most significant bit set if the trap was due to an interrupt. Basically
    /// this is the value of MCASUE/SCAUSE/VSCAUSE after a trap.
    uint64_t trapCause() const
    { return trapCause_; }

    /// Return true if a branch prediction was made for this instruction.
    bool predicted() const
    { return predicted_; }

    /// Return true if instruction is decoded.
    bool decoded() const
    { return decoded_; }

    /// Return true if instruction is executed.
    bool executed() const
    { return executed_; }

    /// Return true if instruction is retired.
    bool retired() const
    { return retired_; }

    /// Return true if this is a store instruction and the store is drained.
    bool drained() const
    { return drained_; }

    /// Return the tag of this instruction.
    uint64_t tag() const
    { return tag_; }

    /// Return true if this is a load instruction. Packet must be decoded.
    bool isLoad() const
    { return di_.isLoad(); }

    /// Return true if this is a store instruction. This includes FP store and store
    /// conditional but not AMOS. Packet must be decoded.
    bool isStore() const
    { return di_.isStore(); }

    /// Return true if this is a vector store instruction. Pakced must be decoed.
    bool isVectorStore() const
    { return di_.isVectorStore(); }

    /// Return true if this is a vector load instruction. Packet must be decoded.
    bool isVectorLoad() const
    { return di_.isVectorLoad(); }

    /// Return true if this is a vector instruction.
    bool isVector() const
    { return di_.isVector(); }

    /// Return true if this is a cbo_zero instruction. Pakced must be decoed.
    bool isCbo_zero() const
    { return di_.isCbo_zero(); }

    /// Return true if this an AMO instruction (does not include lr/sc).  Packet must be
    /// decoded.
    bool isAmo() const
    { return di_.isAmo(); }

    /// Return true if this a store conditional (sc) instruction.  Packet must be decoded.
    bool isSc() const
    { return di_.isSc(); }

    /// Return true if this a load reserve (lr) instruction.  Packet must be decoded.
    bool isLr() const
    { return di_.isLr(); }

    /// Return true if this is a fence instruction. Packet must be decoded
    bool isFence() const
    { return di_.isFence(); }

    bool isDeviceLdSt() const
    { return deviceAccess_; }

    /// Return true if this is a vsetvli/vstivli/vsetvl instruction.
    bool isVset() const
    { return di_.isVsetvli() || di_.isVsetivli() || di_.isVsetvl(); }

    /// Return true if this is a privileged instruction (ebreak/ecall/mret)
    bool isPrivileged() const
    {
      if (di_.instEntry())
        {
          auto instId = di_.instEntry()->instId();
          return (instId == WdRiscv::InstId::ebreak or 
                  instId == WdRiscv::InstId::ecall or
                  instId == WdRiscv::InstId::mret or
                  instId == WdRiscv::InstId::sret);
        }
      return false;
    }

    /// Return the privilege mode right before this instruction was instruction. Valid
    /// only after instruction is executed. This will return Machine, Supervisor, or
    /// User (M, S, or U). You need to look at virtMode to determine whether the effective
    /// mode is VS or VU.
    WdRiscv::PrivilegeMode privlegeMode() const
    { return privMode_; }

    /// Return the virtual mode right before this instruction was executed. Valid only
    /// after instruction is executed. Returned value is true if machine was in virtual
    /// mode.
    bool virtMode() const
    { return virtMode_; }

    /// Fill the given array with the source operands of this instruction (which must be
    /// decoded). Value of each operand will be zero unless the instruction is executed.
    /// Return the number of operands written into the array. This excludes implicit
    /// operands.
    unsigned getSourceOperands(std::array<Operand, 3>& ops) const;

    /// Fill the given array with the destination operands of this instruction (which must
    /// be decoded). Value of each operand will be zero unless the instruction is
    /// executed. Return the number of operands written into the array. This excludes
    /// implicit operands.
    unsigned getDestOperands(std::array<Operand, 2>& ops) const;

    /// Fill the given array with the implicit destination operands of this instruction
    /// (which must be decoded). Value of each operand will be zero unless the instruction
    /// is executed. Return the number of operands written into the array.
    unsigned getImplicitDestOperands(std::array<Operand, 4>& ops) const;

    /// Fill the given array with the implicit source operands of this instruction (which
    /// must be decoded). Value of each operand will be zero unless the instruction is
    /// executed. Return the number of operands written into the array.
    unsigned getImplicitSrcOperands(std::array<Operand, 8>& ops) const;

    /// Fill the given array with the csrs that changed as a side effect to a trap or to
    /// an MRET/SRET instruction. Return the count of such CSRS.
    unsigned getChangedCsrs(std::array<Operand, 8>& ops) const;

    /// Return a reference to the page table walks for the instruction address translation
    /// step (or steps for a page crosser) associated with this instruction. Valid only
    /// after the execution step. Each walk is a vector of walk entries (WalkEntry). The
    /// addr_ field in a WalkEntry is the final physical address if the type_ is RE
    /// otherwise, it is a PTE address. The PTE address is a guest virtual address, a
    /// guest physical address, or a physical address depending on the type_ field: GVA,
    /// GPA, or PA. Only populated when tracing is enabled.
    const std::vector<Walk>& getFetchPageTableWalks() const
    { return fetchWalks_; }

    /// Return a reference to the page table walks for the data address translation step
    /// (or steps for a page crosser or for a vector instruction) associated with this
    /// instruction. Valid only after the execution step. Only populated when tracing is
    /// enabled.
    const std::vector<Walk>& getDataPageTableWalks() const
    { return dataWalks_; }

    /// Return the size of the instruction packet.
    size_t getPacketSize() const;

  protected:

    /// Return the value of the destination register of the instruction of this packet
    /// which must be the instruction currently being retired. The instruction must be
    /// executed. The element index and field are used for vector instructions. If
    /// instruction produces more than one register (e.g. f0 and fcsr), this returns the
    /// value of the first register.
    uint64_t executedDestVal(const Hart64& hart, unsigned size, unsigned elemIx,
                             unsigned field) const;

  private:

    uint64_t tag_ = 0;
    uint64_t iva_ = 0;        // Instruction virtual address (from performance model)
    uint64_t ipa_ = 0;        // Instruction physical address
    uint64_t ipa2_ = 0;       // Instruction physical address on other page
    uint64_t nextIva_ = 0;    // Virtual address of subsequent instruction in prog order

    uint64_t dva_ = 0;        // ld/st data virtual address
    uint64_t dpa_ = 0;        // ld/st data physical address
    uint64_t dpa2_ = 0;       // ld/st data 2nd physical address for page crossing access
    uint64_t dsize_ = 0;      // ld/st data size (total)

    uint64_t stData_ = 0;     // Store data: Used for committing scalar io store.

    // Used for commiting vector store and for forwarding.
    std::unordered_map<uint64_t, uint8_t> stDataMap_;

    // Vector of va/pa/masked of vector load/store instruction. The bool (skip) is set
    // if the element is skipped (maksed-off or tail-element).
    std::vector<VaPaSkip> vecAddrs_;

    // Instruction and data page table walks associated with instruction.
    std::vector<Walk> fetchWalks_;
    std::vector<Walk> dataWalks_;

    uint64_t flushVa_ = 0;    // Redirect PC for packets that should be flushed.

    WdRiscv::DecodedInst di_; // decoded instruction.

    uint64_t execTime_ = 0;   // Execution time
    uint64_t prTarget_ = 0;   // Predicted branch target
    uint64_t trapCause_ = 0;

    // Up to 4 explicit operands and 6 implicit ones (V0, VTYPE, VL, VSTART, FCSR, FRM).
    static constexpr unsigned maxOpCount = 11;
    std::array<Operand, maxOpCount> operands_{};
    unsigned operandCount_ = 0;

    // Entry i is the in-flight producer of the ith operand.
    std::array<OpProducer, maxOpCount> opProducers_;

    // Global register index of a destination register and its corresponding value.
    using DestValue = std::pair<unsigned, OpVal>;

    // One explicit destination register and up to 4 implicit ones (FCSR, VL, VTYPE, VSTART)
    std::array<DestValue, 5> destValues_;

    std::array<Operand, 8> changedCsrs_;
    unsigned changedCsrCount_ = 0;

    uint32_t opcode_ = 0;

    WdRiscv::PrivilegeMode privMode_ = WdRiscv::PrivilegeMode::Machine;  // Privilege mode before execution

    // Following applicable if instruction is a branch
    bool predicted_    : 1 = false;  // true if predicted to be a branch
    bool prTaken_      : 1 = false;  // true if predicted branch/jump is taken
    bool taken_        : 1 = false;  // true if branch/jump is actually taken
    bool mispredicted_ : 1 = false;
    bool shouldFlush_  : 1 = false;

    bool fetched_      : 1 = false;  // true if instruction fetched
    bool decoded_      : 1 = false;  // true if instruction decoded
    bool executed_     : 1 = false;  // true if instruction executed
    bool retired_      : 1 = false;  // true if instruction retired (committed)
    bool drained_      : 1 = false;  // true if a store that has been drained
    bool trap_         : 1 = false;  // true if instruction trapped
    bool interrupt_    : 1 = false;  // true if instruction interrupted
    bool virtMode_     : 1 = false;  // Virtual mode before execution

    bool deviceAccess_ : 1 = false;  // true if access is to device
  };


  class PerfApi
  {
  public:

    PerfApi(System64& system);

    /// Called by the performance model to affect a fetch in whisper. The
    /// instruction tag must be monotonically increasing for a particular
    /// hart. Time must be monotonically non-decreasing. If a trap is taken, this
    /// will set trap to true and, cause to the trap cause, and trapPc to the trap
    /// handler pc; otherwise, trap will be false, and cause and trapPc will be
    /// left unmodified. If a trap happens, whisper will expect the trapPc as the
    /// the vpc on the subsequent call to fetch.
    /// Return true on success and false if given tag has already been fetched.
    /// The tag is a sequence number. It must be monotonically increasing and
    /// must not be zero. Tags of flushed instructions may be reused.
    bool fetch(unsigned hartIx, uint64_t time, uint64_t tag, uint64_t vpc,
	       bool& trap, ExceptionCause& cause, uint64_t& trapPC);

    /// Called by the performance model to affect a decode in whisper. Whisper
    /// will return false if the instruction has not been fetched; otherwise, it
    /// will return true after decoding the instruction, updating the di field of
    /// the corresponding packet, and marking the packed as decoded.
    bool decode(unsigned hartIx, uint64_t time, uint64_t tag);

    /// Optionally called by performance model after decode to inform Whisper of branch
    /// prediction. Returns true on success and false on error (tag was never decoded).
    bool predictBranch(unsigned hart, uint64_t tag, bool prTaken, uint64_t prTarget)
    {
      auto packet = getInstructionPacket(hart, tag);
      if (not packet)
	return false;
      return packet->predictBranch(prTaken, prTarget);
    }

    /// Execute instruction with the given tag in the given hart. Return true on success
    /// marking the instruction as executed and collect its operand values. Return false
    /// if the given tag has not yet been fetched or if it has been flushed.
    bool execute(unsigned hart, uint64_t time, uint64_t tag);

    /// Helper to above execute: Execute packet instruction without changing hart state.
    /// Poke packet source register values into hart, execute, collect destination
    /// values. Restore hart state.
    bool execute(unsigned hartIx, InstrPac& packet);

    /// Retire given instruction at the given hart. Commit all related state
    /// changes. SC/AMO instructions are executed at this stage and write memory
    /// without going through the store/merge buffer. Return true on success and
    /// false on failure (instruction was not executed or was flushed).
    bool retire(unsigned hart, uint64_t time, uint64_t tag);

    /// Return a pointer to the instruction packet with the given tag in the given
    /// hart. Return a null pointer if the given tag has not yet been fetched.
    std::shared_ptr<InstrPac> getInstructionPacket(unsigned hartIx, uint64_t tag)
    {
      const auto& packetMap = hartPacketMaps_.at(hartIx);
      auto iter = packetMap.find(tag);
      if (iter != packetMap.end())
	return iter->second;
      return nullptr;
    }

    /// Return number of instruction packets in the given hart.
    size_t getInstructionPacketCount(unsigned hartIx) const
    {
      const auto& packetMap = hartPacketMaps_.at(hartIx);
      return packetMap.size();
    }

    /// Flush an instruction and all younger instructions in the given hart. Restore
    /// dependency chain (register renaming) to the way it was before the flushed
    /// instructions were decoded.
    bool flush(unsigned hartIx, uint64_t time, uint64_t tag);

    /// Called by performance model to determine whether or not it should redirect
    /// fetch. Return true on success and false on failure. If successful set flush to
    /// true if flush is required and false otherwise. If flush is required, the new fetch
    /// address will be in addr. Note: This is not being used, we should deprecate it.
    bool shouldFlush(unsigned hartIx, uint64_t time, uint64_t tag, bool& flush,
		     uint64_t& addr);

    /// Translate an instruction virtual address into a physical address. Return
    /// ExceptionCause::NONE on success and actual cause on failure.
    WdRiscv::ExceptionCause translateInstrAddr(unsigned hart, uint64_t iva, uint64_t& ipa);

    /// Translate a load data virtual address into a physical address.  Return
    /// ExceptionCause::NONE on success and actual cause on failure.
    WdRiscv::ExceptionCause translateLoadAddr(unsigned hart, uint64_t va, uint64_t& pa);

    /// Translate a store data virtual address into a physical address.  Return
    /// ExceptionCause::NONE on success and actual cause on failure.
    WdRiscv::ExceptionCause translateStoreAddr(unsigned hart, uint64_t va, uint64_t& pa);

    /// Similar to preceding translateInstrAddr, provide page table walk information.
    WdRiscv::ExceptionCause translateInstrAddr(unsigned hart, uint64_t iva, uint64_t& ipa,
                                               std::vector<Walk>& walks);

    /// Similar to preceding translateLoadAddr, provide page table walk information.
    WdRiscv::ExceptionCause translateLoadAddr(unsigned hart, uint64_t va, uint64_t& pa,
                                              std::vector<Walk>& walks);

    /// Similar to preceding translateInstrAddr, provide page table walk information.
    WdRiscv::ExceptionCause translateStoreAddr(unsigned hart, uint64_t va, uint64_t& pa,
                                               std::vector<Walk>& walks);

    /// Called by performance model to request that whisper updates its own memory with
    /// the data of a store instruction. Return true on success and false on error. It is
    /// an error for a store to be drained before it is retired.
    bool drainStore(unsigned hart, uint64_t time, uint64_t tag);

    /// Set data to the load value of the given instruction tag and return true on
    /// success. Return false on failure leaving data unmodified: tag is not valid or
    /// corresponding instruction is not a load.
    bool getLoadData(unsigned hart, uint64_t tag, uint64_t va, uint64_t pa1,
		     uint64_t pa2, unsigned size, uint64_t& data,
                     unsigned elemIx, unsigned field);

    /// Set the data value of a store/amo instruction to be committed to memory
    /// at drain/retire time.
    bool setStoreData(unsigned hart, uint64_t tag, uint64_t pa1, uint64_t pa2,
                      unsigned size, uint64_t value);

    /// Return a pointer of the hart having the given index or null if no such hart.
    std::shared_ptr<Hart64> getHart(unsigned hartIx)
    { return system_.ithHart(hartIx); }

    /// Return number of harts int the system.
    unsigned hartCount() const
    { return system_.hartCount(); }

    /// Enable command log: Log API calls for replay.
    void enableCommandLog(FILE* log)
    { commandLog_ = log; }

    /// Enable instruction tracing to the log file(s).
    void enableTraceLog(std::vector<FILE*>& files)
    { traceFiles_ = files; }

    /// Flatten a vector operand into individual vectors putting the results into the flat
    /// vector. For example, if operand is v4 and LMUL is 2, then the flattened operand
    /// will consist of v4 and v5. If operand is not a vector operand or if LMUL is <= 1
    /// then the flattened operand will be a copy of the original.
    void flattenOperand(const Operand& op, std::vector<Operand>& flat) const;

  protected:

    /// Helper to retire method. Destination registers of the given packet that renamed to
    /// that packet get changed to point to the register file (they are no longer
    /// renamed).
    void undoDestRegRename(unsigned hartIx, const InstrPac& packet);

    /// Collect the register operand values for the instruction in the given packet. The
    /// values are either obtained from the register files or from the instructions in
    /// flight (which models register renaming).  Return true on success and false if we
    /// fail to read (peek) the value of a register in Whisper.
    bool collectOperandValues(Hart64& hart, InstrPac& packet);

    /// Determine the effective group multiplier of each vector operand of the instruction
    /// associated with the given packet. Put results in packet.operands_. This is a helper
    /// to collectOperandValues.
    void getVectorOperandsLmul(Hart64& hart, InstrPac& packet);

    /// Helper to getVectorOperandsLmul
    static void getVecOpsLmul(Hart64& hart, InstrPac& packet);

    /// Return the page number corresponding to the given address
    uint64_t pageNum(uint64_t addr) const
    { return addr >> pageShift_; }

    /// Return the address of the page with the given page number.
    uint64_t pageAddress(uint64_t pageNum) const
    { return pageNum << pageShift_; }

    /// Return the difference between the next page boundary and the current
    /// address. Return 0 if address is on a page boundary.
    unsigned offsetToNextPage(uint64_t addr) const
    { return pageSize_ - (addr & (pageSize_ - 1)); }

    bool commitMemoryWrite(Hart64& hart, uint64_t pa1, uint64_t pa2, unsigned size, uint64_t value);

    bool commitMemoryWrite(Hart64& hart, const InstrPac& packet);

    void insertPacket(unsigned hartIx, uint64_t tag, const std::shared_ptr<InstrPac>& ptr)
    {
      auto& packetMap = hartPacketMaps_.at(hartIx);
      if (not packetMap.empty() and packetMap.rbegin()->first >= tag)
	assert(0 and "Inserted packet with tag newer than oldest tag.");
      packetMap[tag] = ptr;
    }

    std::shared_ptr<Hart64> checkHart(const char* caller, unsigned hartIx);

    std::shared_ptr<InstrPac> checkTag(const char* caller, unsigned HartIx, uint64_t tag);

    bool checkTime(const char* caller, uint64_t time);

    /// Return the global register index for the local (within regiser file) inxex of the
    /// given type (INT, FP, CSR, ...)  and the given relative register number.
    unsigned globalRegIx(WdRiscv::OperandType type, unsigned regNum) const
    {
      using OT = WdRiscv::OperandType;
      switch(type)
	{
	case OT::IntReg: return regNum + intRegOffset_;
	case OT::FpReg:  return regNum + fpRegOffset_;
	case OT::CsReg:  return regNum + csRegOffset_;
	case OT::VecReg: return regNum + vecRegOffset_;
	case OT::Imm:
	case OT::None:   assert(0 && "Error: Assertion failed"); return ~unsigned(0);
	}
      assert(0 && "Error: Assertion failed");
      return ~unsigned(0);
    }

    static bool peekRegister(Hart64& hart, WdRiscv::OperandType type, unsigned regNum,
                             OpVal& value);

    static bool pokeRegister(Hart64& hart, WdRiscv::OperandType type, unsigned regNum,
                             const OpVal& value);

    static bool peekVecRegGroup(Hart64& hart, unsigned regNum, unsigned lmul, OpVal& value);

    /// Get from the producer packet, the value of the register of the given type and
    /// number. Put the value in val returning true on success. Leave val unmodified and
    /// return false on failure: the instruction of the given packet does not produce a
    /// value for the given register.  The regType must not be OperandType::VecReg,
    /// for vector registers, use getVecDestValue.
    bool getDestValue(const InstrPac& producer, WdRiscv::OperandType regType,
                      unsigned regNum, OpVal& val) const;

    /// Get from the producing packet, the value of the given vector register.  Return
    /// true on success and false if given producer does not produce the requested
    /// register in which case val is left unmodified. The number of bytes per vector
    /// register is passed in vecRegSize.
    bool getVecDestValue(const InstrPac& producer, unsigned regNum, unsigned vecRegSize,
                         OpVal& val) const
    {
      assert(producer.executed());

      auto gri = globalRegIx(WdRiscv::OperandType::VecReg, regNum);

      // Producer should have exactly one vector destination which may be a non-trivial
      // group (LMUL > 1).
      for (const auto& pdv : producer.destValues_)
        {
          const auto& vec = pdv.second.vec;  // Produced vector data.
          if (not vec.empty())  // If vector destination
            {
              unsigned group = vec.size() / vecRegSize;
              if (group == 0)
                group = 1;
              assert(group <= maxEffLmul_ and gri >= pdv.first and gri < pdv.first + group);
              unsigned offset = (gri - pdv.first) * vecRegSize;

              auto& result = val.vec;
              result.clear();
              result.insert(result.end(), vec.begin() + offset, vec.begin() + offset + vecRegSize);
              return true;
            }
        }

      return false;
    }

    /// Save hart register values corresponding to packet operands in prevVal.  Return
    /// true on success. Return false if any of the required hart registers cannot be
    /// read.
    static bool saveHartValues(Hart64& hart, const InstrPac& packet,
                               std::array<OpVal, 9>& prevVal);

    /// Install packet operand values (some obtained from previous in-flight instructions)
    /// into the hart registers. Return true on success. Return false if any of the
    /// required hart registers cannot be written.
    static bool setHartValues(Hart64& hart, const InstrPac& packet);

    /// Restore the hart registers corresponding to the packet operands to the values in
    /// the prevVal array.
    static void restoreHartValues(Hart64& hart, const InstrPac& packet,
                                  const std::array<OpVal, 9>& prevVal);

    /// Helper to execute. Restore IMSIC top interrupt if csrn is one of M/S/VS TOPEI.
    static void restoreImsicTopei(Hart64& hart, WdRiscv::CsrNumber csrn, unsigned id, unsigned guest);

    /// Helper to execute. Save IMSIC top interupt if csrn is one of M/S/VS TOPEI.
    static void saveImsicTopei(Hart64& hart, WdRiscv::CsrNumber csrn, unsigned& id, unsigned& guest);

    /// Record the results (register values) corresponding to the operands of the packet
    /// after the execution of the instruction of that packet.
    void recordExecutionResults(Hart64& hart, InstrPac& packet);

    //// Helper to recordExecutionResults.
    void updatePacketDataAddress(Hart64& hart, InstrPac& packet);

    /// Check execute stage results versus retire stage. Return true on match and false on
    /// mismatch.
    static bool checkExecVsRetire(const Hart64& hart, const InstrPac& packet);

    /// Helper to decode method.
    static void determineExplicitOperands(InstrPac& packet);

    /// Helper to decode method.
    static void determineImplicitOperands(InstrPac& packet);

  private:

    /// Map an instruction tag to corresponding packet.
    using PacketMap = std::map<uint64_t, std::shared_ptr<InstrPac>>;

    /// Map a global register index to the in-flight instruction producing that
    /// register. This is register renaming.
    using RegProducers = std::vector<std::shared_ptr<InstrPac>>;

    System64& system_;
    std::shared_ptr<InstrPac> prevFetch_;

    /// Per-hart map of in flight instruction packets.
    std::vector<PacketMap> hartPacketMaps_;

    /// Per-hart map of in flight executed store packets.
    std::vector<PacketMap> hartStoreMaps_;

    /// Per-hart index tag the last retired instruction.
    std::vector<uint64_t> hartLastRetired_;

    /// Per-hart register renaming table (indexed by global register index).
    std::vector<RegProducers> hartRegProducers_;

    uint64_t time_ = 0;

    bool skipIoLoad_ = false;  // Avoid speculative execute of load from IO space.

    FILE* commandLog_ = nullptr;
    std::vector<FILE*> traceFiles_;   // One per hart.

    unsigned pageSize_ = 4096;  // Page size in bytes
    unsigned pageShift_ = 12;   // log2(pageSize_) : number of bits to represent offset in page.

    /// Global indexing for all registers.
    const unsigned intRegOffset_  = 0;
    const unsigned fpRegOffset_   = intRegOffset_ + 32;
    const unsigned vecRegOffset_  = fpRegOffset_  + 32;

    // The vector register index may go beyond 32 for speculated vector instruction with
    // an invalid register-index/lmul combination. We reserve 512: 8*8*8
    // max lmul = 8, max eew/ew = 8/1 = 8, max field count = 8
    const unsigned maxEffLmul_ = 512;
    const unsigned csRegOffset_   = vecRegOffset_ + 32 + maxEffLmul_;
    const unsigned totalRegCount_ = csRegOffset_  + 4096; // 4096: max CSR count.

    static constexpr uint64_t haltPc = ~uint64_t(1);  // value assigned to InstPac->nextIva_ when program termination is encountered

    const uint64_t initHartLastRetired = -1;  // default value for the hartLastRetired_ map
  };

}
