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

#include <deque>
#include <vector>
#include "System.hpp"
#include "Hart.hpp"
#include "DecodedInst.hpp"

namespace TT_PERF         // Tenstorrent Whisper Performance Model API
{

  //============================================================================
  // POINTER OWNERSHIP POLICY
  //============================================================================
  //
  // This API uses a clear separation between ownership and non-owning access:
  //
  // OWNERSHIP (shared_ptr<InstrPac>):
  //   - FastPacketMap: owns all instruction packets
  //   - Instruction::mPacPtr: model takes shared ownership to extend packet lifetime
  //   - OpProducer/RegProducers: dependency chains require shared ownership
  //   - StoreMap: store forwarding requires shared ownership
  //
  // NON-OWNING ACCESS (raw InstrPac*):
  //   - Internal hot-path operations (fetch, decode, execute, retire)
  //   - Read-only access where ownership isn't needed
  //   - Iterators and lookups
  //
  // API FUNCTIONS:
  //   - getInstructionPacket(): returns shared_ptr (for model to take ownership)
  //   - getInstructionPacketRaw(): returns raw pointer (for hot-path access)
  //   - checkTag/checkTagRaw: similar pattern for validation
  //
  // POINTER VALIDITY:
  //   - Raw pointers remain valid as long as packet is in FastPacketMap
  //   - Packets are removed on retire (non-store) or drain (store)
  //   - Flush removes all packets with tag >= flushed tag
  //
  //============================================================================

  class InstrPac;  // Forward declaration for FastPacketMap

  /// Result codes for retire operation
  enum class RetireResult
  {
    Success,              ///< Instruction retired successfully
    InvalidTime,          ///< Invalid time parameter
    InvalidHart,          ///< Invalid hart index
    UnknownTag,           ///< Tag was never fetched
    OutOfOrder,           ///< Out of order retire detected
    AlreadyRetired,       ///< Tag was already retired
    WrongPc,              ///< PC mismatch at retire time
    ExecRetireMismatch,   ///< Execute vs retire results differ
    ScMispredict,         ///< SC outcome at retire differs from the execute-time prediction
    CsrMispredict,        ///< CSR write outcome at retire differs from the execute-time prediction
    InterruptPreempted    ///< Async interrupt taken at commit instead of this instruction; it did not retire
  };

  /// Convert RetireResult to string for error messages
  inline const char* retireResultToString(RetireResult result)
  {
    switch (result)
    {
      case RetireResult::Success:            return "Success";
      case RetireResult::InvalidTime:        return "Invalid time";
      case RetireResult::InvalidHart:        return "Invalid hart";
      case RetireResult::UnknownTag:         return "Unknown tag (never fetched)";
      case RetireResult::OutOfOrder:         return "Out of order retire";
      case RetireResult::AlreadyRetired:     return "Tag already retired";
      case RetireResult::WrongPc:            return "Wrong PC at retire";
      case RetireResult::ExecRetireMismatch: return "Execute vs retire mismatch";
      case RetireResult::ScMispredict:       return "SC outcome mispredict (execute vs retire differ)";
      case RetireResult::CsrMispredict:      return "CSR outcome mispredict (execute vs retire differ)";
      case RetireResult::InterruptPreempted: return "Async interrupt preempted this instruction";
    }
    return "Unknown error";
  }

  /// Fast O(1) packet map using deque with tag offset.
  /// Exploits the fact that instruction tags are monotonically increasing and dense.
  class FastPacketMap
  {
  public:

    using PacketPtr = std::shared_ptr<InstrPac>;

    FastPacketMap() = default;

    /// O(1) lookup by tag. Returns nullptr if not found.
    InstrPac* find(uint64_t tag) const
    {
      if (tag < baseTag_ or tag >= baseTag_ + packets_.size())
        return nullptr;
      return packets_[tag - baseTag_].get();
    }

    /// O(1) lookup returning shared_ptr (for cases where ownership is needed).
    PacketPtr findShared(uint64_t tag) const
    {
      if (tag < baseTag_ or tag >= baseTag_ + packets_.size())
        return nullptr;
      return packets_[tag - baseTag_];
    }

    /// O(1) amortized insert. Tag must be exactly baseTag_ + size() for sequential insertion.
    void insert(uint64_t tag, PacketPtr packet)
    {
      if (packets_.empty())
        baseTag_ = tag;
      assert(tag == baseTag_ + packets_.size() and "Tags must be inserted in sequential order.");
      packets_.push_back(std::move(packet));
      ++activeCount_;
    }

    /// O(1) erase by tag. Marks slot as nullptr and compacts front.
    void erase(uint64_t tag)
    {
      if (tag < baseTag_ or tag >= baseTag_ + packets_.size())
        return;
      auto& slot = packets_[tag - baseTag_];
      if (slot)
        {
          slot = nullptr;
          --activeCount_;
        }
      // Compact front: remove leading nullptrs
      while (not packets_.empty() and not packets_.front())
        {
          packets_.pop_front();
          ++baseTag_;
        }
    }

    /// Erase all packets with tag >= given tag (for flush).
    /// Returns count of erased packets.
    size_t eraseFrom(uint64_t tag)
    {
      tag = std::max(tag, baseTag_);
      if (tag >= baseTag_ + packets_.size())
        return 0;

      size_t erased = 0;
      size_t startIdx = tag - baseTag_;
      for (size_t i = startIdx; i < packets_.size(); ++i)
        {
          if (packets_[i])
            {
              ++erased;
              --activeCount_;
            }
        }
      packets_.resize(startIdx);
      return erased;
    }

    bool empty() const
    { return activeCount_ == 0; }

    size_t size() const
    { return activeCount_; }

    /// Get the maximum tag (tag of last non-null packet).
    /// Returns 0 if empty.
    uint64_t maxTag() const
    {
      if (packets_.empty())
        return 0;
      // Scan backwards to find last non-null
      for (size_t i = packets_.size(); i > 0; --i)
        {
          if (packets_[i - 1])
            return baseTag_ + i - 1;
        }
      return 0;
    }

    /// Get the minimum tag (baseTag, which is always the first slot).
    uint64_t minTag() const
    { return baseTag_; }

    /// Iterator support for forward iteration over active packets.
    /// Yields (tag, InstrPac*) pairs, skipping nulls.
    class Iterator
    {
    public:

      using PacketEntry = std::pair<uint64_t, InstrPac*>;

      Iterator(const FastPacketMap* map, size_t idx) : map_(map), idx_(idx)
      { skipNulls(); }

      PacketEntry operator*() const
      { return {map_->baseTag_ + idx_, map_->packets_[idx_].get()}; }

      Iterator& operator++()
      {
        ++idx_;
        skipNulls();
        return *this;
      }

      bool operator!=(const Iterator& other) const
      { return idx_ != other.idx_ or map_ != other.map_; }

      bool operator==(const Iterator& other) const
      { return idx_ == other.idx_ and map_ == other.map_; }

      /// Access the raw packet pointer
      InstrPac* packet() const
      { return map_->packets_[idx_].get(); }

      uint64_t tag() const
      { return map_->baseTag_ + idx_; }

    private:

      void skipNulls()
      {
        while (idx_ < map_->packets_.size() and not map_->packets_[idx_])
          ++idx_;
      }

      const FastPacketMap* map_;
      size_t idx_;
    };

    Iterator begin() const
    { return {this, 0}; }

    Iterator end() const
    { return {this, packets_.size()}; }

    /// Find iterator starting at given tag (or next valid if tag's slot is null).
    Iterator iteratorAt(uint64_t tag) const
    {
      if (tag < baseTag_)
        return begin();
      if (tag >= baseTag_ + packets_.size())
        return end();
      return {this, tag - baseTag_};
    }

    /// Reverse iterator
    class ReverseIterator
    {
    public:

      ReverseIterator(const FastPacketMap* map, int64_t idx) : map_(map), idx_(idx)
      { skipNulls(); }

      std::pair<uint64_t, InstrPac*> operator*() const
      { return {map_->baseTag_ + idx_, map_->packets_[idx_].get()}; }

      ReverseIterator& operator++()
      {
        --idx_;
        skipNulls();
        return *this;
      }

      bool operator!=(const ReverseIterator& other) const
      { return idx_ != other.idx_ or map_ != other.map_; }

      InstrPac* packet() const
      { return map_->packets_[idx_].get(); }

      uint64_t tag() const
      { return map_->baseTag_ + idx_; }

    private:

      void skipNulls()
      {
        while (idx_ >= 0 and static_cast<size_t>(idx_) < map_->packets_.size() and not map_->packets_[idx_])
          --idx_;
      }

      const FastPacketMap* map_;
      int64_t idx_;
    };

    ReverseIterator rbegin() const
    { return {this, static_cast<int64_t>(packets_.size()) - 1}; }

    ReverseIterator rend() const
    { return {this, -1}; }

    /// For compatibility: check if a tag would be out of order
    bool wouldBeOutOfOrder(uint64_t tag) const
    { return not packets_.empty() and tag <= maxTag(); }

    /// Clear all packets
    void clear()
    {
      packets_.clear();
      activeCount_ = 0;
      baseTag_ = 1;
    }

  private:

    std::deque<PacketPtr> packets_;
    uint64_t baseTag_ = 1;      // Tag of the first element in deque
    size_t activeCount_ = 0;    // Count of non-null entries
  };

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

    template <typename URV>
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
      const unsigned opCount = di_.operandCount();
      const uint64_t otherTag = other.tag_;
      // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
      for (unsigned i = 0; i < opCount; ++i)
        {
          using OM = WdRiscv::OperandMode;

          auto mode = di_.ithOperandMode(i);
          if (mode == OM::Read or mode == OM::ReadWrite)
            {
              const auto& producer = opProducers_[i];
              if (producer.scalar and producer.scalar->tag_ == otherTag)
                return true;
              for (const auto& entry : producer.vec)
                if (entry and entry->tag_ == otherTag)
                  return true;
            }
        }
      // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
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
    { return di_.isVsetvli() or di_.isVsetivli() or di_.isVsetvl(); }

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
    template <typename URV>
    uint64_t executedDestVal(const WdRiscv::Hart<URV>& hart, unsigned size, unsigned elemIx,
                             unsigned field) const;

  private:

    // ==================== HOT DATA - First Cache Line (~64 bytes) ====================
    // Frequently accessed fields grouped together for cache locality

    uint64_t tag_ = 0;
    uint64_t iva_ = 0;        // Instruction virtual address (from performance model)
    uint64_t ipa_ = 0;        // Instruction physical address
    uint64_t ipa2_ = 0;       // Instruction physical address on other page
    uint64_t nextIva_ = 0;    // Virtual address of subsequent instruction in prog order

    // State flags - frequently checked, packed into ~2 bytes
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

    uint32_t opcode_ = 0;

    // ==================== WARM DATA - Second Cache Line ====================
    // Moderately accessed fields

    uint64_t dva_ = 0;        // ld/st data virtual address
    uint64_t dpa_ = 0;        // ld/st data physical address
    uint64_t dpa2_ = 0;       // ld/st data 2nd physical address for page crossing access
    uint64_t dsize_ = 0;      // ld/st data size (total)

    uint64_t stData_ = 0;     // Store data: Used for committing scalar io store.
    uint64_t flushVa_ = 0;    // Redirect PC for packets that should be flushed.
    uint64_t execTime_ = 0;   // Execution time
    uint64_t prTarget_ = 0;   // Predicted branch target

    // ==================== INSTRUCTION DATA ====================

    WdRiscv::DecodedInst di_; // decoded instruction.

    uint64_t trapCause_ = 0;
    unsigned operandCount_ = 0;
    unsigned changedCsrCount_ = 0;
    WdRiscv::PrivilegeMode privMode_ = WdRiscv::PrivilegeMode::Machine;  // Privilege mode before execution

    // ==================== OPERAND ARRAYS ====================
    // Up to 4 explicit operands and 6 implicit ones (V0, VTYPE, VL, VSTART, FCSR/VCSR, FRM).
    static constexpr unsigned maxOpCount = 11;
    std::array<Operand, maxOpCount> operands_{};

    // Entry i is the in-flight producer of the ith operand.
    std::array<OpProducer, maxOpCount> opProducers_;

    // Global register index of a destination register and its corresponding value.
    using DestValue = std::pair<unsigned, OpVal>;

    // One explicit destination register and up to 4 implicit ones (FCSR, VL, VTYPE, VSTART)
    std::array<DestValue, 5> destValues_;

    std::array<Operand, 8> changedCsrs_;

    // ==================== COLD DATA - Infrequently accessed ====================

    // Reservation state saved before this LR instruction executed, used to restore
    // the prior reservation when this speculative LR is flushed.
    uint64_t savedLrAddr_ = 0;
    unsigned savedLrSize_ = 0;
    bool hadPriorLr_ = false;

    // SC speculative state. Populated by execute() and updated to retire-time values
    // by retire(). scExecuted_ guards the rest; the address/size/value fields are valid
    // only when scExecSuccess_ is true. drainStore() commits the recorded store on a
    // successful SC.
    bool scExecuted_    : 1 = false;
    bool scExecSuccess_ : 1 = false;
    uint64_t scStoreVal_ = 0;
    uint64_t scPa1_      = 0;
    uint64_t scPa2_      = 0;
    unsigned scSize_     = 0;

    // CSR-write speculative state. Populated unconditionally for every non-trapping
    // CSR op at execute(): csrNum_ is the CSR address (di.op2()), csrExecNewVal_
    // is the post-execute CSR value, csrExecRd_ is the captured rd. retire()
    // reconciles these against the architectural retire-time outcome and signals
    // RetireResult::CsrMispredict on divergence. csrExecuted_ is the validity bit
    // and is also used by checkExecVsRetire to bypass the strict dest compare.
    bool csrExecuted_   : 1 = false;
    unsigned csrNum_       = 0;
    uint64_t csrExecRd_    = 0;
    uint64_t csrExecNewVal_ = 0;

    // Large containers placed at the end

    // Used for committing vector store and for forwarding.
    std::unordered_map<uint64_t, uint8_t> stDataMap_;

    // Vector of va/pa/masked of vector load/store instruction. The bool (skip) is set
    // if the element is skipped (masked-off or tail-element).
    std::vector<VaPaSkip> vecAddrs_;

    // Instruction and data page table walks associated with instruction.
    std::vector<Walk> fetchWalks_;
    std::vector<Walk> dataWalks_;
  };


  template <typename URV>
  class PerfApi
  {
  public:

    using SystemType = WdRiscv::System<URV>;
    using HartType = WdRiscv::Hart<URV>;

    PerfApi(SystemType& system);

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
      auto* packet = getInstructionPacketRaw(hart, tag);
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
    /// false on failure. If status pointer is provided, it will be set to the
    /// specific failure reason. An instruction that was never executed is executed here,
    /// so a client that does not speculate can skip execute and step each instruction once.
    bool retire(unsigned hart, uint64_t time, uint64_t tag,
                RetireResult* status = nullptr);

    /// Return a shared_ptr to the instruction packet with the given tag.
    /// Return nullptr if the given tag has not yet been fetched.
    std::shared_ptr<InstrPac> getInstructionPacket(unsigned hartIx, uint64_t tag)
    { return hartPacketMaps_[hartIx].findShared(tag); }

    /// Return a raw pointer to the instruction packet (avoids atomic ref-count operations).
    /// Use this for read-only access in performance-critical paths.
    InstrPac* getInstructionPacketRaw(unsigned hartIx, uint64_t tag)
    { return hartPacketMaps_[hartIx].find(tag); }

    /// Result of a packet lookup. With FastPacketMap, no iterator is needed
    /// since all operations are O(1) by tag.
    struct PacketLookupResult
    {
      InstrPac* packet = nullptr;
      uint64_t tag = 0;
      bool found = false;

      explicit operator bool() const { return found; }
    };

    /// Find packet and return raw pointer with tag.
    PacketLookupResult findPacket(unsigned hartIx, uint64_t tag)
    {
      auto* packet = hartPacketMaps_[hartIx].find(tag);
      if (packet) [[likely]]
        return {.packet = packet, .tag = tag, .found = true};
      return {.packet = nullptr, .tag = 0, .found = false};
    }

    /// Return number of instruction packets in the given hart.
    size_t getInstructionPacketCount(unsigned hartIx) const
    { return hartPacketMaps_[hartIx].size(); }

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

    /// Similar to preceding translateStoreAddr, provide page table walk information.
    WdRiscv::ExceptionCause translateStoreAddr(unsigned hart, uint64_t va, uint64_t& pa,
                                               std::vector<Walk>& walks);

    /// Variants of the preceding translate methods that walk as the instruction with the
    /// given tag would have: speculative CSR writes older than tag are forwarded for the
    /// walk, and the tag's execute-time privilege/virtual mode is used instead of the hart's.
    /// Use these when redoing the walk of an instruction that has executed but not retired:
    /// execute() undoes every speculative CSR write before returning, so a plain translate*
    /// afterwards runs against architectural state and a load that singleStep accepted under
    /// a speculative MSTATUS/SSTATUS (SUM, MXR) faults here instead.
    ///
    /// The mode comes from the in-flight packet, so if there is no packet for tag or it has
    /// not executed these fall back to the hart's mode and behave like the untagged
    /// overloads. Callers wanting the tagged behavior must call after execute().
    ///
    /// walks is cleared first, so it is empty rather than stale when the translation is a
    /// no-op (machine mode, or no supervisor mode). Like the untagged overloads, these leave
    /// the hart's Hart::get{Fetch,Data}PageTableWalks() holding this walk.
    WdRiscv::ExceptionCause translateInstrAddrForTag(unsigned hart, uint64_t tag, uint64_t iva,
                                                     uint64_t& ipa, std::vector<Walk>& walks);

    /// See translateInstrAddrForTag.
    WdRiscv::ExceptionCause translateLoadAddrForTag(unsigned hart, uint64_t tag, uint64_t va,
                                                    uint64_t& pa, std::vector<Walk>& walks);

    /// See translateInstrAddrForTag.
    WdRiscv::ExceptionCause translateStoreAddrForTag(unsigned hart, uint64_t tag, uint64_t va,
                                                     uint64_t& pa, std::vector<Walk>& walks);

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
    std::shared_ptr<HartType> getHart(unsigned hartIx)
    { return system_.ithHart(hartIx); }

    /// Return cached raw Hart pointer. No bounds checking - caller must ensure valid index.
    HartType* getHartRaw(unsigned hartIx)
    { return hartRawPtrs_[hartIx]; }

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
    static bool collectOperandValues(HartType& hart, InstrPac& packet);

    /// Determine the effective group multiplier of each vector operand of the instruction
    /// associated with the given packet. Put results in packet.operands_. This is a helper
    /// to collectOperandValues.
    void getVectorOperandsLmul(HartType& hart, InstrPac& packet);

    /// Helper to getVectorOperandsLmul
    static void getVecOpsLmul(HartType& hart, InstrPac& packet);

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

    bool commitMemoryWrite(HartType& hart, uint64_t pa1, uint64_t pa2, unsigned size, uint64_t value);

    bool commitMemoryWrite(HartType& hart, const InstrPac& packet);

    void insertPacket(unsigned hartIx, uint64_t tag, std::shared_ptr<InstrPac> ptr)
    {
      hartPacketMaps_[hartIx].insert(tag, std::move(ptr));
    }

    std::shared_ptr<HartType> checkHart(const char* caller, unsigned hartIx);

    HartType* checkHartRaw(const char* caller, unsigned hartIx);

    std::shared_ptr<InstrPac> checkTag(const char* caller, unsigned hartIx, uint64_t tag);

    InstrPac* checkTagRaw(const char* caller, unsigned hartIx, uint64_t tag);

    bool checkTime(const char* caller, uint64_t time);

    /// Return the global register index for the local (within regiser file) inxex of the
    /// given type (INT, FP, CSR, ...)  and the given relative register number.
    static unsigned globalRegIx(WdRiscv::OperandType type, unsigned regNum)
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

    static bool peekRegister(HartType& hart, WdRiscv::OperandType type, unsigned regNum,
                             OpVal& value);

    static bool pokeRegister(HartType& hart, WdRiscv::OperandType type, unsigned regNum,
                             const OpVal& value);

    static bool peekVecRegGroup(HartType& hart, unsigned regNum, unsigned lmul, OpVal& value);

    /// Get from the producer packet, the value of the register of the given type and
    /// number. Put the value in val returning true on success. Leave val unmodified and
    /// return false on failure: the instruction of the given packet does not produce a
    /// value for the given register.  The regType must not be OperandType::VecReg,
    /// for vector registers, use getVecDestValue.
    static bool getDestValue(const InstrPac& producer, WdRiscv::OperandType regType,
                             unsigned regNum, OpVal& val);

    /// Get from the producing packet, the value of the given vector register.  Return
    /// true on success and false if given producer does not produce the requested
    /// register in which case val is left unmodified. The number of bytes per vector
    /// register is passed in vecRegSize.
    static bool getVecDestValue(const InstrPac& producer, unsigned regNum, unsigned vecRegSize,
                                OpVal& val)
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
              // A trapped producer with an illegal EMUL is recorded for more registers
              // than exist; its value covers only those that do.
              if (producer.trapped() and gri >= pdv.first + group)
                return false;
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
    static bool saveHartValues(HartType& hart, const InstrPac& packet,
                               std::array<OpVal, 9>& prevVal);

    /// Install packet operand values (some obtained from previous in-flight instructions)
    /// into the hart registers. Return true on success. Return false if any of the
    /// required hart registers cannot be written.
    static bool setHartValues(HartType& hart, const InstrPac& packet);

    /// Restore the hart registers corresponding to the packet operands to the values in
    /// the prevVal array.
    static void restoreHartValues(HartType& hart, const InstrPac& packet,
                                  const std::array<OpVal, 9>& prevVal);

    /// One CSR clobbered by pushSpecCsrContext, with what to put back. ok is false when the CSR
    /// could not be read, and so was never poked; popSpecCsrContext then leaves it alone.
    struct CsrFwdSave { unsigned csrNum; URV prev; bool ok; };

    /// Poke onto the hart the predicted new values of the in-flight speculative CSR writes
    /// older than tag, so a setHartValues, singleStep or walk done next sees the CSR state this
    /// point in the speculative stream would. Poked in hartSpecCsrs_ order, which is tag order,
    /// so the youngest write to a CSR wins. Skips non-forwardable CSRs (see isNonForwardableCsr)
    /// and packet's own CSR operands, which belong to set/restoreHartValues. Overlay before
    /// setHartValues: pokeCsr ignores VTYPE/VL/VSTART while VS is Off.
    ///
    /// An S-level CSR peeks/pokes its VS counterpart per the hart's virtual mode, so the hart
    /// must be in the mode the forwarded values were produced under. See VirtModeGuard.
    ///
    /// Returns what was overwritten; hand it to popSpecCsrContext (or SpecCsrContext) to undo.
    std::vector<CsrFwdSave> pushSpecCsrContext(HartType& hart, unsigned hartIx, uint64_t tag,
                                               const InstrPac* packet);

    /// Undo pushSpecCsrContext, leaving only architectural CSR state on the hart. Reverse
    /// order, so where several writes to one CSR were forwarded the oldest value lands.
    static void popSpecCsrContext(HartType& hart, const std::vector<CsrFwdSave>& saves);

    /// Scoped pushSpecCsrContext: pops at end of scope, so no path out of the block -- early
    /// return or throw included -- can strand the poked values. pop() undoes it sooner; it is
    /// idempotent and disarms the destructor.
    class SpecCsrContext
    {
    public:

      SpecCsrContext(HartType& hart, std::vector<CsrFwdSave>&& saves)
        : hart_(&hart), saves_(std::move(saves))
      { }

      SpecCsrContext(const SpecCsrContext&) = delete;
      SpecCsrContext& operator=(const SpecCsrContext&) = delete;

      ~SpecCsrContext()
      { pop(); }

      void pop()
      {
        if (hart_)
          {
            popSpecCsrContext(*hart_, saves_);
            hart_ = nullptr;
          }
      }

    private:

      HartType* hart_ = nullptr;
      std::vector<CsrFwdSave> saves_;
    };

    /// Scoped enable of VirtMem walk tracing. A null VirtMem is a no-op that never touches the
    /// trace flag -- restoring a stand-in prior value would clobber tracing the caller had on.
    class WalkTraceGuard
    {
    public:

      explicit WalkTraceGuard(WdRiscv::VirtMem* virtMem)
        : virtMem_(virtMem), prev_(virtMem? virtMem->enableTrace(true) : false)
      { }

      WalkTraceGuard(const WalkTraceGuard&) = delete;
      WalkTraceGuard& operator=(const WalkTraceGuard&) = delete;

      ~WalkTraceGuard()
      { if (virtMem_) virtMem_->enableTrace(prev_); }

    private:

      WdRiscv::VirtMem* virtMem_ = nullptr;
      bool prev_ = false;
    };

    /// Scoped swap of the hart's virtual (V) mode. A null hart is a no-op, so a caller can say
    /// "only if it differs" without branching around the guard.
    class VirtModeGuard
    {
    public:

      VirtModeGuard(HartType* hart, bool virtMode)
        : hart_(hart), prev_(hart? hart->virtMode() : false)
      { if (hart_) hart_->setVirtualMode(virtMode); }

      VirtModeGuard(const VirtModeGuard&) = delete;
      VirtModeGuard& operator=(const VirtModeGuard&) = delete;

      ~VirtModeGuard()
      { if (hart_) hart_->setVirtualMode(prev_); }

    private:

      HartType* hart_ = nullptr;
      bool prev_ = false;
    };

    /// The access a translateAddr call is checking. One value per access, so read and write
    /// cannot both be requested and the reported name is never ambiguous.
    enum class TransAccess { Fetch, Load, Store };

    /// Name of the given access, for error reporting.
    static const char* transAccessName(TransAccess access);

    /// Shared body of the translate* entry points. walks, when non-null, is cleared and then
    /// receives the walk; it distinguishes the walk-reporting overloads. tag scopes the
    /// speculative CSR context and the privilege/virtual mode (see translateInstrAddrForTag);
    /// pass noSpecContext for plain architectural state.
    WdRiscv::ExceptionCause translateAddr(unsigned hartIx, uint64_t tag, uint64_t va, uint64_t& pa,
                                          TransAccess access, std::vector<Walk>* walks);

    /// Helper to execute. Restore IMSIC top interrupt if csrn is one of M/S/VS TOPEI.
    static void restoreImsicTopei(HartType& hart, WdRiscv::CsrNumber csrn, unsigned id, unsigned guest);

    /// Helper to execute. Save IMSIC top interupt if csrn is one of M/S/VS TOPEI.
    static void saveImsicTopei(HartType& hart, WdRiscv::CsrNumber csrn, unsigned& id, unsigned& guest);

    /// Record the results (register values) corresponding to the operands of the packet
    /// after the execution of the instruction of that packet.
    void recordExecutionResults(HartType& hart, InstrPac& packet);

    /// Helper to execute and retire. Record the outcome of an executed, non-trapping SC.
    static void recordScOutcome(HartType& hart, InstrPac& packet);

    /// Helper to execute and retire. Record the current value of csrn as changed by the packet.
    static void recordChangedCsr(HartType& hart, InstrPac& packet, WdRiscv::CsrNumber csrn);

    //// Helper to recordExecutionResults.
    void updatePacketDataAddress(HartType& hart, InstrPac& packet);

    /// Check execute stage results versus retire stage. Return true on match and false on
    /// mismatch.
    static bool checkExecVsRetire(const HartType& hart, const InstrPac& packet);

    /// Helper to decode method.
    static void determineExplicitOperands(InstrPac& packet);

    /// Helper to decode method.
    static void determineImplicitOperands(InstrPac& packet);

  private:

    /// Map an instruction tag to corresponding packet (for store forwarding).
    using StoreMap = std::map<uint64_t, std::shared_ptr<InstrPac>>;

    /// Map a global register index to the in-flight instruction producing that
    /// register. This is register renaming.
    using RegProducers = std::vector<std::shared_ptr<InstrPac>>;

    /// Reservation created by a speculatively executed LR instruction.
    struct SpecLrEntry
    {
      uint64_t tag = 0;
      uint64_t addr = 0;
      unsigned size = 0;
    };

    /// Predicted CSR new value captured by a speculatively executed CSR write.
    /// Used to forward the speculative outcome to younger speculative consumers
    /// (e.g. mret reading MEPC) before they are reconciled architecturally at retire.
    struct SpecCsrEntry
    {
      uint64_t tag = 0;
      unsigned csrNum = 0;
      uint64_t newVal = 0;
    };

    SystemType& system_;
    std::shared_ptr<InstrPac> prevFetch_;

    /// Per-hart map of in flight instruction packets. Uses FastPacketMap for O(1) access.
    std::vector<FastPacketMap> hartPacketMaps_;

    /// Per-hart map of in flight executed store packets (uses std::map for lower_bound support).
    std::vector<StoreMap> hartStoreMaps_;

    /// Per-hart index tag the last retired instruction.
    std::vector<uint64_t> hartLastRetired_;

    /// Per-hart register renaming table (indexed by global register index).
    std::vector<RegProducers> hartRegProducers_;

    /// Per-hart list of reservations from speculatively executed LR instructions,
    /// ordered by tag. Used to set the correct reservation before speculative SC
    /// execution and to restore reservations on flush.
    std::vector<std::vector<SpecLrEntry>> hartSpecLrs_;

    /// Per-hart list of in-flight speculative CSR writes' predicted new values, kept sorted by
    /// tag -- entries are added at execute, which is out of order, so insertion is at the upper
    /// bound; pushSpecCsrContext needs the order to make the youngest write to a CSR win.
    /// Walked before every speculative singleStep to forward younger consumers (implicit ones
    /// like mret/sret/trap entry included) to the predicted post-write value; restored after.
    /// Pruned on retire of the CSR write and on flush. Mirror of hartSpecLrs_.
    std::vector<std::vector<SpecCsrEntry>> hartSpecCsrs_;

    /// Cached raw Hart pointers.
    std::vector<HartType*> hartRawPtrs_;

    uint64_t time_ = 0;

    bool skipIoLoad_ = false;  // Avoid speculative execute of load from IO space.

    FILE* commandLog_ = nullptr;
    std::vector<FILE*> traceFiles_;   // One per hart.

    unsigned pageSize_ = 4096;  // Page size in bytes
    unsigned pageShift_ = 12;   // log2(pageSize_) : number of bits to represent offset in page.

    /// Global indexing for all registers.
    static constexpr unsigned intRegOffset_  = 0;
    static constexpr unsigned fpRegOffset_   = intRegOffset_ + 32;
    static constexpr unsigned vecRegOffset_  = fpRegOffset_  + 32;

    // The vector register index may go beyond 32 for speculated vector instruction with
    // an invalid register-index/lmul combination. We reserve 512: 8*8*8
    // max lmul = 8, max eew/ew = 8/1 = 8, max field count = 8
    static constexpr unsigned maxEffLmul_ = 512;
    static constexpr unsigned csRegOffset_   = vecRegOffset_ + 32 + maxEffLmul_;
    static constexpr unsigned totalRegCount_ = csRegOffset_  + 4096; // 4096: max CSR count.

    static constexpr uint64_t haltPc = ~uint64_t(1);  // value assigned to InstPac->nextIva_ when program termination is encountered

    const uint64_t initHartLastRetired = -1;  // default value for the hartLastRetired_ map

    /// tag value meaning "no speculative context": translate against architectural state.
    static constexpr uint64_t noSpecContext = ~uint64_t(0);
  };

  // Type aliases for RV32 and RV64
  using PerfApi32 = PerfApi<uint32_t>;
  using PerfApi64 = PerfApi<uint64_t>;

}

