#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <set>
#include <map>
#include <unordered_set>
#include "DecodedInst.hpp"
#include "Hart.hpp"


namespace WdRiscv
{
  template <typename URV>
  class Hart;

  using McmInstrIx = uint32_t;
  using MemoryOpIx = uint32_t;

  struct MemoryOp
  {
    uint64_t   time_           = 0;
    uint64_t   pa_             = 0;  // Physical address.
    uint64_t   data_           = 0;  // Model (Whisper) data for ld/st instructions.
    uint64_t   rtlData_        = 0;  // RTL data.
    McmInstrIx tag_            = 0;  // Instruction tag.

    // Per byte forward time as offset from read time (time_). A forwarded byte effective
    // time is time + offset. This would not be needed if RTL/test-bech would break a read
    // op at forward boundary.
    std::array<uint16_t, 8> fwOffset_  = { 0, 0, 0, 0, 0, 0, 0, 0 };

    // This would not be needed if the RTL always writes vector elements in order. In some
    // cases (e.g. element ops crossing middle of cache line) we do get the writes out of
    // order.
    uint64_t   insertTime_     = 0;  // Time of merge buffer insert (if applicable).

    uint16_t   elemIx_         = 0;  // Vector element index.
    uint16_t   field_          = 0;  // Vector element field (for segment load).
    uint8_t    hartIx_    : 8  = 0;
    uint8_t    size_      : 8  = 0;
    bool       isRead_    : 1  = false;
    bool       failRead_  : 1  = false;
    bool       canceled_  : 1  = false;
    bool       bypass_    : 1  = false;   // True if a bypass operation.
    bool       isIo_      : 1  = false;   // True if in IO region.

    /// Return true if address range of this operation overlaps that of the given one.
    bool overlaps(const MemoryOp& other) const
    { return pa_ + size_ > other.pa_ and pa_ < other.pa_ + other.size_; }

    /// Return true if address range of this operation overlaps given address.
    bool overlaps(uint64_t addr) const
    { return addr >= pa_ and addr < pa_ + size_; }

    bool isCanceled() const { return canceled_; }
    void cancel() { canceled_ = true; }

    /// Set value to the model (whisper) byte data for the given physical byte address
    /// returning true on success. Return false if given address is not covered by this
    /// operation or if this is not a read operation.
    bool getModelReadOpByte(uint64_t pa, uint8_t& byte) const
    {
      if (not isRead_ or pa < pa_ or pa >= pa_ + size_)
	return false;
      byte = data_ >> ((pa - pa_) * 8);
      return true;
    }

    /// Return the minimum forward time of this operation.
    uint64_t minForwardTime() const
    {
      if (not isRead_)
        return time_;

      uint64_t mt = time_ + fwOffset_.at(0);
      for (unsigned i = 1; i < size_; ++i)
        mt = std::min(mt, time_ + fwOffset_.at(i));
      return mt;
    }

    /// Return the maxnimum forward time of this operation.
    uint64_t maxForwardTime() const
    {
      if (not isRead_)
        return time_;

      uint64_t mt = time_;
      for (unsigned i = 0; i < size_; ++i)
        mt = std::max(mt, time_ + fwOffset_.at(i));
      return mt;
    }

  };



  struct McmInstr
  {
    // memOps contains indices into an array of MemoryOp items.
    std::vector<MemoryOpIx> memOps_;
    uint64_t virtAddr_ = 0;   // Virtual data address for ld/st instructions.
    uint64_t physAddr_ = 0;   // Physical data address for ld/st instruction.
    uint64_t physAddr2_ = 0;  // Additional data address for page crossing stores.
    uint64_t storeData_ = 0;  // Model (whisper) Data for sore instructions.

    uint64_t addrTime_ = 0;   // Time address register was produced (for ld/st/amo).
    uint64_t dataTime_ = 0;   // Time data register was produced (for st/amo).
    uint64_t retireTime_ = 0; // Time instruction was retired.
    McmInstrIx addrProducer_ = 0;  // Producer of addr register (for ld/st/amo).
    McmInstrIx dataProducer_ = 0;  // Producer of data register (for st/amo).

    // Producer and time of the data register of a vector ld/st instuction.
    struct VecProdTime
    {
      unsigned regIx_ = 0;
      McmInstrIx tag_ = 0;
      uint64_t time_ = 0;
    };

    // Time ld/st intruction vector data register(s) were produced.
    std::array<VecProdTime, 8> vecProdTimes_;

    // Time ld/st instruction vector index register(s) were produced,
    std::array<VecProdTime, 8> ixProdTimes_;

    DecodedInst di_;
    McmInstrIx tag_ = 0;
    uint8_t hartIx_     : 8 = 0;
    uint8_t size_       : 8 = 0;        // Data size for load/store instructions.
    PrivilegeMode priv_ : 2 = PrivilegeMode::Machine;
    bool virt_          : 1 = false;    // Virtual mode.

    bool retired_       : 1 = false;
    bool canceled_      : 1 = false;
    bool isLoad_        : 1 = false;
    bool isStore_       : 1 = false;
    bool complete_      : 1 = false;
    bool hasOverlap_    : 1 = false;   // For vector load and store instructions.

    /// Return true if this a load/store instruction.
    bool isMemory() const { return isLoad_ or isStore_; }

    /// Return true if this instruction is retired.
    bool isRetired() const { return retired_; }

    /// Return true if this instruction is canceled.
    bool isCanceled() const { return canceled_; }

    /// Mark instruction as canceled.
    void cancel() { canceled_ = true; }

    /// Associated given memory operation index with this instruction.
    void addMemOp(MemoryOpIx memOpIx)
    {
      if (std::find(memOps_.begin(), memOps_.end(), memOpIx) != memOps_.end())
	{
	  std::cerr << "McmInstr::addMemOp: Error: Op already added\n";
	  assert(0 && "Error: McmInstr::addMemOp: Op ix already present");
	}
      memOps_.push_back(memOpIx);
    }

    /// Return true if data memory referenced by this instruction overlaps that
    /// of the given other instruction. Both instructions must be memory
    /// instructions.  Both instructions must be retired.
    bool overlaps(const McmInstr& other) const
    {
      // A non-successful store conditional (zero size) does not overlap anything.
      if ((di_.isSc() and size_ == 0) or (other.di_.isSc() and other.size_ == 0))
	return false;

      if (size_ == 0 or other.size_ == 0)
	std::cerr << "McmInstr::overlaps: Error: tag1=" << tag_
		  << " tag2=" << other.tag_ << " zero data size\n";

      if (not retired_ or not other.retired_)
	std::cerr << "McmInstr::overlaps: Error: tag1=" << tag_
		  << " tag2=" << other.tag_ << " non-retired instruction\n";

      if (virtAddr_ == other.virtAddr_)
	return true;
      if (virtAddr_ < other.virtAddr_)
	return other.virtAddr_ - virtAddr_ < size_;
      return virtAddr_ - other.virtAddr_ < other.size_;
    }

    /// Return true if address of the data memory referenced by this instruction is
    /// aligned.
    bool isAligned() const
    { return (physAddr_ & (size_ - 1)) == 0; }

  };


  template <typename URV>
  class Mcm
  {
  public:

    enum PpoRule { R1 = 1, R2, R3, R4, R5, R6, R7, R8, R9, R10, R11, R12,
		   R13, Io, Limit };

    /// Constructor.
    Mcm(unsigned hartCount, unsigned pageSize, unsigned mergeBufferSize);

    /// Destructor.
    ~Mcm();

    /// Initiate an out of order read for a load instruction. If a preceding overlapping
    /// store has not yet left the merge/store buffer then forward data from that store to
    /// the read operation; otherwise, get the data from global memory. Return true on
    /// success and false if global memory is not readable (in the case where we do not
    /// forward). For vector load elemIx is the element index and field is the segment
    /// field number (0 if non segment).
    bool readOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag, uint64_t physAddr,
		unsigned size, uint64_t rtlData, unsigned elemIx, unsigned field);

    /// This is a write operation bypassing the merge buffer. The cache parameter indicates
    /// whether the write operation should go to cache or memory.
    bool bypassOp(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t pa,
                  unsigned size, uint64_t rtlData, unsigned elem, unsigned field, bool cache);

    /// Initiate a merge buffer write.  All associated store write transactions are marked
    /// completed. Write instructions where all writes are complete are marked
    /// complete. Return true on success.  The given physical address must be a multiple
    /// of the merge buffer line size (which is also the cache line size). The rtlData
    /// vector must be of size n or larger where n is the merge buffer line size. The
    /// rtlData bytes will be placed in memory in consecutive locations starting with
    /// physAddr. If skipCheck is true, do not check RTL data versus Whisper data and do
    /// not update Whisper memory (this is used by test-bench for non correctable checksum
    /// errors).
    bool mergeBufferWrite(Hart<URV>& hart, uint64_t time, uint64_t physAddr,
			  const std::vector<uint8_t>& rtlData,
			  const std::vector<bool>& mask,
                          bool skipCheck = false);

    /// Insert a write operation for the given instruction into the merge buffer removing
    /// it from the store buffer. Return true on success. Size is expected to be less than
    /// or equal to 8. Larger inserts must be split by the caller.
    bool mergeBufferInsert(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t pa,
                           unsigned size, uint64_t rtlData, unsigned elem, unsigned field);

    /// Cancel all the memory operations associated with the given tag. This is
    /// done when a speculative instruction is canceled or when an instruction
    /// is trapped.
    void cancelInstruction(Hart<URV>& hart, uint64_t instrTag);

    /// This is called when an instruction is retired.
    bool retire(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		const DecodedInst& di, bool trapped);

    /// Skip read check on containing address. This is a workaround for cbo.inval
    /// operations.
    void skipReadDataCheck(uint64_t addr, unsigned size, bool enable)
    {
      skipReadCheckAddr_ = addr;
      skipReadCheckSize_ = size;
      skipReadCheckEnable_ = enable;
    }

    bool isReadDataCheckEnabled(uint64_t addr) const
    {
      if (skipReadCheckEnable_ and
          addr >= skipReadCheckAddr_ and
          addr < (skipReadCheckAddr_ + skipReadCheckSize_))
        return false;
      return true;
    }

    /// Perform PPO checks (e.g. rule 4) on pending instructions.
    bool finalChecks(Hart<URV>& hart);

    /// Return the load value of the current target instruction which must be a load
    /// instruction. Pa1 is the physical address of the loaded data. Pa2 is the same as
    /// paddr1 except for page crossing loads where pa2 is the physical address of the
    /// second page. Va is the virtual address of the load data.
    bool getCurrentLoadValue(Hart<URV>& hart, uint64_t tag, uint64_t va, uint64_t pa1,
			     uint64_t pa2, unsigned size, bool isVec, uint64_t& value,
			     unsigned elemIx = 0, unsigned field = 0);

    /// Return the merge buffer line size in bytes.
    unsigned mergeBufferLineSize() const
    { return lineSize_; }

    /// Enable/disable total-store-order.
    void enableTso(bool flag)
    { isTso_ = flag; }

    /// Return the earliest memory time for the byte at the given address. Return 0 if
    /// address is not covered by given instruction.
    uint64_t earliestByteTime(const McmInstr& instr, uint64_t addr) const;

    /// Return the earliest memory time for the byte at the given address and given element
    /// index.  Return 0 if address is not covered by given instruction.
    uint64_t earliestByteTime(const McmInstr& instr, uint64_t addr, unsigned ix) const;

    /// Return the latest memory time for the byte at the given address. Return max value
    /// if address is not covered by given instruction.
    uint64_t latestByteTime(const McmInstr& instr, uint64_t addr) const;

    /// Return the latest memory time for the byte at the given address and given element
    /// index. Return max value if address is not covered by given instruction.
    uint64_t latestByteTime(const McmInstr& instr, uint64_t addr, unsigned ix) const;

    /// Return the effective earliest memory time for the byte at the given
    /// address. Return 0 if address is not covered by given instruction.
    /// The effective time it the max of the mread-time and the forward-time.
    uint64_t effectiveMinByteTime(const McmInstr& instr, uint64_t addr) const;

    /// Return the effective latest memory time for the byte at the given address. Return
    /// max value if address is not covered by given instruction. The effective time it
    /// the max of the mread-time and the forward-time.
    uint64_t effectiveMaxByteTime(const McmInstr& instr, uint64_t addr) const;

    /// Skip checking preserve program order (PPO) rules if flag is false.
    void enablePpo(bool flag)
    { std::fill(ppoEnabled_.begin(), ppoEnabled_.end(), flag); }

    /// Skip checking given preserve program order (PPO) rule if flag is false.
    void enablePpo(PpoRule rule, bool flag)
    { ppoEnabled_.at(rule) = flag; }

    /// Return true if given rule is enabled.
    bool isEnabled(PpoRule rule) const
    { return ppoEnabled_.at(unsigned(rule)); }

    /// Check PPO rule1 for the given instruction as instruction B, all relevant
    /// preceding instructions (in program order) are considered as instruction A.
    /// Return true if no violation of rule1 is found and false otherwise.
    bool ppoRule1(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule2. See ppoRule1.
    bool ppoRule2(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule3. See ppoRule1.
    bool ppoRule3(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule4. See ppoRule1.
    bool ppoRule4(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule5. See ppoRule1.
    bool ppoRule5(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule6. See ppoRule1.
    bool ppoRule6(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule7. See ppoRule1.
    bool ppoRule7(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule8. See ppoRule1.
    bool ppoRule8(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule9. See ppoRule1.
    bool ppoRule9(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule10. See ppoRule1.
    bool ppoRule10(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule11. See ppoRule1.
    bool ppoRule11(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule12. See ppoRule1.
    bool ppoRule12(Hart<URV>& hart, const McmInstr& instr) const;

    /// Check PPO rule13. See ppoRule1.
    bool ppoRule13(Hart<URV>& hart, const McmInstr& instr) const;

    bool ioPpoChecks(Hart<URV>& hart, const McmInstr& instr) const;

    /// Helper to main ppoRule1. Check A against B.
    bool ppoRule1(unsigned hartId, const McmInstr& instrA, const McmInstr& instrB) const;

    /// Helper to main ppoRule1. Check memory operation of A against B.
    bool ppoRule1(unsigned hartId, const McmInstr& instrA, const MemoryOp& opA,
		  const McmInstr& instrB) const;

    /// Helper to above ppoRule5.
    bool ppoRule5(Hart<URV>&, const McmInstr& instrA, const McmInstr& instrB) const;

    /// Helper to above ppoRule6.
    bool ppoRule6(Hart<URV>&, const McmInstr& instrA, const McmInstr& instrB) const;

    /// Helper to above ppoRule7.
    bool ppoRule7(const McmInstr& instrA, const McmInstr& instrB) const;

    /// Check that all preceeding stores preceeding fence in program order have drained
    /// given that the fence has predecessor write. Reutrn true on success and fail
    /// if any preceeding store have not drained. This is stronger than what is requied
    /// by ppo rule4 and it simplifies that rule.
    bool checkFence(Hart<URV>& hart, const McmInstr& fence) const;

    /// If B is a load and A is cbo.flush/clean instruction that overlaps B and precedes
    /// it in program order, then B cannot have a read operation prior to to A's retire
    /// time.  Return true if this rule followed and false otherwise.
    bool checkLoadVsPriorCmo(Hart<URV>& hart, const McmInstr& instrB) const;

    /// Check that previous SINVAL.VMA instructions are executed before subsequent
    /// implicit references to the memory-management data structures. Return true on
    /// success and false if there are translations between the the current
    /// SFENCE.INVAL.IR and the most recent SINVAL.VMA instruction.
    bool checkSfenceInvalIr(Hart<URV>& hart, const McmInstr& instr) const;

    /// The SFENCE.W.INVAL instruction guarantees that any previous stores already visible
    /// to the current RISC-V hart are ordered before subsequent SINVAL.VMA instructions
    /// executed by the same hart.
    bool checkSfenceWInval(Hart<URV>& hart, const McmInstr& instr) const;

    bool checkCmo(Hart<URV>& bart, const McmInstr& instr) const;

    uint64_t latestOpTime(const McmInstr& instr) const
    {
      if (not instr.complete_)
	{
	  std::cerr << "Error: Mcm::latestOpTime: Called on an incomplete instruction\n";
	  assert(0 && "Error: Mcm::lasestOpTime: Incomplete instr");
	}
      uint64_t time = 0;
      for (auto opIx : instr.memOps_)
	if (opIx < sysMemOps_.size())
	  time = std::max(time, sysMemOps_.at(opIx).time_);
      return time;
    }

    /// Return the smallest time of the memory operations of given instruction.
    uint64_t earliestOpTime(const McmInstr& instr) const
    {
      if (instr.memOps_.empty())
	return time_;

      uint64_t mt = ~uint64_t(0);
      for (auto opIx : instr.memOps_)
	if (opIx < sysMemOps_.size())
	  mt = std::min(mt, sysMemOps_.at(opIx).time_);
      return mt;
    }

    /// Return the smallest time of the memory operations of the given instruction. Adjust
    /// read-operation times to account for forwarding.
    uint64_t effectiveMinTime(Hart<URV>& hart, const McmInstr& instr) const;

    /// Return the largest time of the memory operations of the given instruction. Adjust
    /// read-operation times to account for forwarding.
    uint64_t effectiveMaxTime(const McmInstr& instr) const;

    /// Return true if instruction a is before b in memory time.
    bool isBeforeInMemoryTime(const McmInstr& a, const McmInstr& b) const
    {
      // if (a.complete_ and not b.complete_)
      // return true;
      if (not a.complete_ and b.complete_)
	return false;
      if (not a.complete_ and not b.complete_)
	{
	  std::cerr << "Error: Mcm::isBeforeInMemoryTime: Both instructions incomplete\n";
	  assert(0 && "Error: Mcm::isBeforeInMemoryTime: Both instructions incomplete");
	  return false;
	}
      uint64_t aTime = latestOpTime(a);
      uint64_t bTime = earliestOpTime(b);
      if (a.isStore_ and b.isStore_ and aTime == bTime)
	return a.tag_ < b.tag_;
      return aTime < bTime;
    }

    /// Configure checking whole merge buffer line (versus checking bytes covered by
    /// stores).
    void setCheckWholeMbLine(bool flag)
    { checkWholeLine_ = flag; }

    /// Return the smallest tag of a load/amo instruction preceeding the given instruction
    /// and having a memory time larger than that of the given instruction. Return the tag
    /// of the given instruction if no smaller tag can be found. This supports ppo rules
    /// 12 and 13. We only check read tags because a write cannot generate a data or
    /// address dependency.
    McmInstrIx getMinReadTagWithLargerTime(unsigned hartIx, const McmInstr& instr) const;

    /// Issue a warning about an instruction opcode missing in the fetch cache (not
    /// brought in with an mcm-ifetch operation).
    void reportMissingFetch(const Hart<URV>& hart, uint64_t tag, uint64_t pa) const;

  protected:

    /// Helper to public readOp which splits line crossing ops into two calling this
    /// method for each.
    bool readOp_(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t pa, unsigned size,
                 uint64_t rtlData, unsigned elemIx, unsigned field);

    using MemoryOpVec = std::vector<MemoryOp>;

    enum VecKind
      {
	Skip,     // No active elements, all elements have agnostic mask/tail policy.
	Active,   // Vector has at least one active element.
	Preserve  // Vector has at least one element with preserve mask/tail policy.
      };

    /// Helper to ppoRule1.
    void printPpo1Error(unsigned hartId, McmInstrIx tag1, McmInstrIx tag2, uint64_t t1,
			uint64_t t2, uint64_t pa) const;

    /// Helper to read-commit methods: commitReadOps & commitVecReadOPs.
    void printReadMismatch(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
                           unsigned size, uint64_t rtlData, uint64_t refData) const;

    /// Read up to a double word (size <= 8) from the reference model memory.
    bool referenceModelRead(Hart<URV>& hart, uint64_t pa, unsigned size, uint64_t& val);

    /// Return true if the physical adresses of the given read operation, op, overlaps the
    /// the range of addresses associated with a memory access for the given vector
    /// element. Size is the size of the memory access, and elemSize is the element size.
    bool vecReadOpOverlapsElem(const MemoryOp& op, uint64_t pa1, uint64_t pa2,
			       unsigned size, unsigned elemIx, bool unitStride,
			       unsigned elemSize) const;

    /// Heler to vecReadOpOverlapsElem.
    bool vecReadOpOverlapsElemByte(const MemoryOp& op, uint64_t addr, unsigned elemIx,
				   bool unitStride, unsigned elemSize) const;

    /// Return true if given instruction is an indexed load/store and it has an index
    /// register with a value produced after the instruction has used that index
    /// register. If out of order, set producer to the tag of the instruction producing
    /// the value of the ooo index register, also set dataTime to the time of the
    /// corrsponding data register instr. This works only for the instruction being
    /// retired.
    bool isVecIndexOutOfOrder(Hart<URV>& hart, const McmInstr& instr, unsigned& ixReg,
			      McmInstrIx& producer, uint64_t& produerTime,
			      uint64_t& dataTime) const;

    /// Return the earliest memory time for the given vector register.
    uint64_t getVecRegEarlyTime(Hart<URV>& hart, const McmInstr& instr, unsigned regNum) const;

    /// Trim read operations to match reference (whisper). Mark replay read ops as
    /// canceled. Remove cancled ops.
    bool commitReadOps(Hart<URV>& hart, McmInstr& instr);

    /// Similar to above but for vector instructions.
    bool commitVecReadOps(Hart<URV>& hart, McmInstr& instr);

    /// Helper to comitVecReadOps for strided with stride 0.
    bool commitVecReadOpsStride0(Hart<URV>& hart, McmInstr& instr);

    /// Helper to comitVecReadOps for unit stride.
    bool commitVecReadOpsUnitStride(Hart<URV>& hart, McmInstr& instr);

    /// Helper to getCurrentLoadValue. For stride-zero vector loads, sometimes the
    /// test-bench will send fewer reads than elements (worst case, it will send one read
    /// for all the elements). Return the highest element index sent by the test-bench
    /// that is less than or equal to the given element index.
    unsigned effectiveStride0ElemIx(const Hart<URV>&, const McmInstr&, unsigned elemIx) const;

    /// Helper to commitVecReadOps: Collect the reference (Whisper) element info:
    /// address, index, field, data-reg, index-reg. Determine the number of
    /// active (non-masked) elements.
    void collectVecRefElems(Hart<URV>& hart, McmInstr& instr, unsigned& activeCount);

    /// Compute a mask of the instruction data bytes covered by the
    /// given memory operation. Return 0 if the operation does not
    /// overlap the given instruction.
    unsigned determineOpMask(const McmInstr& instr, const MemoryOp& op) const;

    /// If op overlaps instruction then trim high end of op to the
    /// boundary of the instruction.
    void trimMemoryOp(const McmInstr& instr, MemoryOp& op);

    /// Helper to retire method: Capture parameters of store instruction and
    /// commit its data to memory. Return true on success and false on failure.
    /// Return true if instruction is not a a store.
    bool retireStore(Hart<URV>& hart, McmInstr& instr);

    /// Helper to retire method: Retire a CMO instruction. Return false on
    /// failure. Return true on success or if given instruction is not CMO.
    bool retireCmo(Hart<URV>& hart, McmInstr& instr);

    /// Return the page number corresponding to the given address.
    uint64_t pageNum(uint64_t addr) const
    { return addr >> pageShift_; }

    /// Return the address of the page with the given page number.
    uint64_t pageAddress(uint64_t pageNum) const
    { return pageNum << pageShift_; }

    /// Return the line number corresponding to the given address.
    uint64_t lineNum(uint64_t addr) const
    { return addr >> lineShift_; }

    /// Return the closest line boundary less than or equal to the given address.
    uint64_t lineAlign(uint64_t addr) const
    { return (addr >> lineShift_) << lineShift_; }

    /// Set the tag of the instruction producing the latest data of the given vector
    /// register.
    void setVecRegProducer(unsigned hartIx, unsigned vecReg, McmInstrIx tag)
    {
      auto& regProducer = hartData_.at(hartIx).regProducer_;
      regProducer.at(vecReg + vecRegOffset_) = tag;
    }

    /// Set the time the data of the given vector register was produced.
    void setVecRegTime(unsigned hartIx, unsigned vecReg, uint64_t time)
    {
      auto& regTime = hartData_.at(hartIx).regTime_;
      regTime.at(vecReg + vecRegOffset_) = time;
    }

    /// Return the time the data of the given vector register was produced.
    uint64_t vecRegTime(unsigned hartIx, unsigned vecReg) const
    {
      const auto& regTime = hartData_.at(hartIx).regTime_;
      return regTime.at(vecReg + vecRegOffset_);
    }

    /// Return the tag of the instruction producing the latest data in data of the given
    /// vector register. Return 0 if no such instruction.
    McmInstrIx vecRegProducer(unsigned hartIx, unsigned vecReg) const
    {
      const auto& regProducer = hartData_.at(hartIx).regProducer_;
      return regProducer.at(vecReg + vecRegOffset_);
    }

    /// Remove from hartPendingWrites_ the write ops falling with the given RTL
    /// line and masked by rtlMask (rtlMask bit is on for op bytes) and place
    /// them sorted by instr tag in coveredWrites. Write ops may not straddle
    /// line boundary. Write ops may not be partially masked.
    bool collectCoveredWrites(Hart<URV>& hart, uint64_t time, uint64_t lineBegin,
			      uint64_t lineSize, const std::vector<bool>& rtlMask,
			      MemoryOpVec& coveredWrites);

    /// Forward to the given read op from the stores ahead of the instruction of the read
    /// op in program order.
    bool forwardToRead(Hart<URV>& hart, const std::set<McmInstrIx>& stores, MemoryOp& op) const;

    /// Forward from a write to a read op. Return true on success.  Mask is the mask of
    /// bits of op to be updated by the forward operation and is updated (bits cleared)
    /// if some parts of op are successfully updated. This a helper to forwardToRead.
    bool writeToReadForward(const MemoryOp& writOp, MemoryOp& readOp, uint64_t& mask);

    /// Helper to getCurrentLoadValue. Collect overlapping stores preceding instr in
    /// program order and with write times after those of instr reads (write times before
    /// instr reads imply a drained write that can no longer forward to instr).
    void collectForwardingStores(Hart<URV>& hart, const McmInstr& instr,
				 std::set<McmInstrIx>& stores) const;

    /// Forward from a store instruction to a read-operation. Mask is the mast of bits of
    /// op to be updated by the forward operation and is updated (bits cleared) if some
    /// parts of op are successfully updated. This is a helper to to forwardToRead.
    /// Addr/data/size are the physical-address/data-value/data-size of the store
    /// instruction.
    bool storeToReadForward(const McmInstr& store, MemoryOp& readOp, uint64_t& mask,
			    uint64_t addr, uint64_t data, unsigned size) const;

    bool vecStoreToReadForward(const McmInstr& store, MemoryOp& readOp, uint64_t& mask) const;

    /// Determine the source and destination registers of the given instruction.
    void identifyRegisters(const Hart<URV>& hart,
                           const DecodedInst& di,
			   std::vector<unsigned>& sourceRegs,
			   std::vector<unsigned>& destRegs);

    /// Return true if any of the physical data addresses of the given vector instruction
    /// overlap the physical address range of the given memory operation.
    bool vecOverlaps(const McmInstr& instr, const MemoryOp& other) const
    {
      if (not instr.di_.isVector())
	return false;

      const auto& vecRefMap = hartData_.at(instr.hartIx_).vecRefMap_;
      auto iter = vecRefMap.find(instr.tag_);
      if (iter == vecRefMap.end())
        assert(false);

      auto& vecRefs = iter->second;
      if (vecRefs.isOutOfBounds(other))
	return false;

      for (auto& vecRef : vecRefs.refs_)
        if (rangesOverlap(vecRef.pa_, vecRef.size_, other.pa_, other.size_))
	  return true;

      return false;
    }

    /// Return true if the physical data address range referenced by given instruction
    /// overlaps any of the addresses in the given set.
    bool overlaps(const McmInstr& instr, const std::unordered_set<uint64_t>& addrSet) const;

    /// Return true if the physical data address range referenced by given instruction
    /// overlaps that of the given memory operation.
    bool overlaps(const McmInstr& instr, const MemoryOp& op) const
    {
      if (instr.size_ == 0)
	std::cerr << "Mcm::overlaps: Error: hart-ix=" << unsigned(instr.hartIx_) << " instr-tag="
                  << instr.tag_ << " zero data size (not a memory instruction)\n";
      if (op.size_ == 0)
	std::cerr << "Mcm::overlaps: Error: hart-ix=" << unsigned(op.hartIx_) << " op-tag="
                  << op.tag_ << " zero data size\n";

      if (instr.di_.isVector())
	return vecOverlaps(instr, op);

      if (instr.physAddr_ == instr.physAddr2_)   // Non-page-crossing
	return rangesOverlap(instr.physAddr_, instr.size_, op.pa_, op.size_);

      // Page crossing.
      unsigned size1 = offsetToNextPage(instr.physAddr_);
      if (rangesOverlap(instr.physAddr_, size1, op.pa_, op.size_))
	return true;
      unsigned size2 = instr.size_ - size1;
      return rangesOverlap(instr.physAddr2_, size2, op.pa_, op.size_);
    }

    /// Return true if the given address ranges overlap one another.
    bool rangesOverlap(uint64_t addr1, unsigned size1, uint64_t addr2, unsigned size2) const
    {
      if (addr1 <= addr2)
	return addr2 - addr1 < size1;
      return addr1 - addr2 < size2;
    }

    /// Return true if the given vector instruction has one or more reference (Whisper)
    /// physical address. Reference addresses may be missing if all the elements are
    /// masked off, or if VL is 0.
    bool vecHasRefPhysAddr(const McmInstr& instr) const
    {
      assert(instr.di_.isVector());

      auto& vecRefMap = hartData_.at(instr.hartIx_).vecRefMap_;
      auto iter = vecRefMap.find(instr.tag_);
      if (iter == vecRefMap.end())
	return false;

      auto& vecRefs = iter->second;
      return not vecRefs.empty();
    }

    /// Return true if any of the physical addresses associated with the given instruction
    /// overlap the given address.
    bool vecOverlapsRefPhysAddr(const McmInstr& instr, uint64_t addr) const;

    /// Return true if any of the RTL physical addresses associated with the given
    /// instruction overlap the given address.
    bool vecOverlapsRtlPhysAddr(const McmInstr& instr, uint64_t addr) const
    {
      for (auto opIx : instr.memOps_)
	{
	  auto& op = sysMemOps_.at(opIx);
	  if (op.overlaps(addr))
	    return true;
	}
      return false;
    }

    /// Return true if given instruction data addresses overlap the given address. Return
    /// false if instruction is not a memory instruction. Instruction must be retired.
    bool overlapsRefPhysAddr(const McmInstr& instr, uint64_t addr) const
    {
      if (not instr.isMemory())
	return false;
      assert(instr.isRetired());

      if (instr.di_.isVector())
	return vecOverlapsRefPhysAddr(instr, addr);

      if (instr.physAddr_ == instr.physAddr2_)
	return instr.physAddr_ <= addr and addr - instr.physAddr_ < instr.size_;

      unsigned size1 = offsetToNextPage(instr.physAddr_);
      if (pageNum(instr.physAddr_) == pageNum(addr))
	return instr.physAddr_ <= addr and addr - instr.physAddr_ < size1;
      unsigned size2 = instr.size_ - size1;
      return instr.physAddr2_ <= addr and addr - instr.physAddr2_ < size2;
    }

    /// Return true if the instruction have overlapping data address ranges.
    bool overlaps(const McmInstr& i1, const McmInstr& i2) const;

    bool instrHasRead(const McmInstr& instr) const;

    bool instrHasWrite(const McmInstr& instr) const;

    bool checkStoreComplete(unsigned hartIx, const McmInstr& instr) const;

    /// Check reference (Whipser) data aginst RTL data for given instruction.
    bool checkStoreData(Hart<URV>& hart, const McmInstr& instr) const;

    /// Helper to check sore data. Applies to vector store.
    bool checkVecStoreData(Hart<URV>& hart, const McmInstr& instr) const;

    bool checkLoadComplete(const McmInstr& instr) const;

    /// Put in the given set the physical addresses of the target instruction that are
    /// written by the store instruction.
    void identifyWrittenBytes(const McmInstr& storeInstr, const McmInstr& target,
			      std::unordered_set<uint64_t>& written) const
    {
      if (not storeInstr.isStore_)
	return;
      for (auto opIx : target.memOps_)
	{
	  const auto& op = sysMemOps_.at(opIx);
	  for (unsigned i = 0; i < op.size_; ++i)
	    {
	      uint64_t addr = op.pa_ + i;
	      if (overlapsRefPhysAddr(storeInstr, addr))
		written.insert(addr);
	    }
	}
    }

    /// Return true if given trapped instruction is a partially executed vector load/store.
    bool isPartialVecLdSt(Hart<URV>& hart, const DecodedInst& di) const;

    void cancelNonRetired(Hart<URV>& hart, uint64_t instrTag);

    bool checkRtlWrite(unsigned hartId, const McmInstr& instr,
		       const MemoryOp& op) const;

    bool checkRtlRead(Hart<URV>&hart, const McmInstr& instr,
		      const MemoryOp& op) const;

    bool updateTime(const char* method, uint64_t time);

    McmInstr* findInstr(unsigned hartIx, McmInstrIx tag);

    McmInstr* findOrAddInstr(unsigned hartIx, McmInstrIx tag);

    /// Helper to cancelInstruction.
    void cancelInstr(Hart<URV>& hart, McmInstr& instr);

    /// Propagate depenencies from source to dest registers. Set the time data of dest
    /// regs was produced.
    void updateDependencies(const Hart<URV>& hart, const McmInstr& instr);

    /// Propagate data times from source to dest register for vector instructions.
    /// This is a helper to updateDependencies.
    void updateVecRegTimes(const Hart<URV>& hart, const McmInstr& instr);

    /// Put in the given array the indices of the vector data registers of the currently
    /// retiring vector load/store instruction, each index is associated with a VecKind
    /// indicating whether or not the vector is active (has one or more active element),
    /// is preserved (has one or more masked/tail element with a preserve policy), or
    /// skipped (all elements masked/tail with an agnostic policy). Return 0 if no vector
    /// register is referenced or if the instruction is not a vector ld/st.
    unsigned getLdStDataVectors(const Hart<URV>& hart, const McmInstr& instr,
				std::array<std::pair<unsigned,VecKind>, 32>& vecs) const;

    /// Put in the given array the indices of the vector index registers of the currently
    /// retiring vector load/store indexed instruction, each index is associated with a
    /// VecKind (see previous method). Return 0 if no vector register is not referenced or
    /// if the instruction is not a vector ld/st.
    unsigned getLdStIndexVectors(const Hart<URV>& hart, const McmInstr& instr,
				 std::array<std::pair<unsigned,VecKind>, 32>& vecs) const;

    /// Set the memory time of the given branch instruction to the latest time (data was
    /// produced) of its registers. This branch memory time is used for checking
    /// the control dependency rule (rule 11). No-op if instr is not a branch.
    void setBranchMemTime(const Hart<URV>& hart, const McmInstr& instr);

    /// Get the latest read time for each vector register that is in the destination
    /// register group of the given load instruction. Return the overall latest time.
    /// The times vector will be resized to the number of registers in the group and
    /// will have one value for each register.
    void updateVecLoadDependencies(const Hart<URV>& hart, const McmInstr& instr);

    /// Set the tag of the instruction producing the data/address used by the given
    /// instruction (data for store/amo instrs, address for load/amo instrs). Set the
    /// corresponding time. No-op if instruction is not a memory instruction.
    void setProducerTime(const Hart<URV>& hart, McmInstr& instr);

    /// Map register number of operand opIx to a unique integer by adding an offset:
    /// integer register have 0 offset, fp regs have 32, vector regs have 64, and csr regs
    /// have 96.
    unsigned effectiveRegIx(const DecodedInst& di, unsigned opIx) const;

    /// Return the difference between the next page boundary and the current
    /// address. Return 0 if address is on a page boundary.
    unsigned offsetToNextPage(uint64_t addr) const
    { return pageSize_ - (addr & (pageSize_ - 1)); }

    /// Return the difference between the next line boundary and the current
    /// address. Return 0 if address is on a line boundary.
    unsigned offsetToNextLine(uint64_t addr) const
    { return lineSize_ - (addr & (lineSize_ - 1)); }

    /// Return true if given memory operation belong to an instrction from the Zicbom
    /// extension (cbo.clean/flush/inval instructions). Such instructions are not marked
    /// as stores (they do not forward).
    bool isCbomOp(const MemoryOp& op) const
    {
      if (op.size_ != 0)
        return false;
      auto& instrVec = hartData_.at(op.hartIx_).instrVec_;
      if (op.tag_ >= instrVec.size())
        return false;
      auto& instr = instrVec.at(op.tag_);
      return instr.di_.extension() == RvExtension::Zicbom;
    }

  private:

    const unsigned intRegOffset_ = 0;
    const unsigned fpRegOffset_ = 32;
    const unsigned vecRegOffset_ = 64;
    const unsigned csRegOffset_ = 96;
    const unsigned totalRegCount_ = csRegOffset_ + 4096; // 4096: max csr count.

    using McmInstrVec = std::vector<McmInstr>;

    using RegTimeVec = std::vector<uint64_t>;    // Map reg index to time.
    using RegProducerVec = std::vector<McmInstrIx>;   // Map reg index to instr tag.

    /// Vector reference (produced by Whisper) load/store physical addresses and
    /// corresponding data.
    struct VecRef
    {
      VecRef(unsigned ix, uint64_t pa = 0, uint64_t data = 0, unsigned size = 0,
	     unsigned dataReg = 0, unsigned ixReg = 0, unsigned field = 0)
	: pa_(pa), data_(data), ix_(ix), size_(size), reg_(dataReg), ixReg_(ixReg),
	  field_(field)
      { }

      bool overlaps(uint64_t addr) const
      { return addr >= pa_ && addr < pa_ + size_; }

      uint64_t pa_     = 0;   // Physical address.
      uint64_t data_   = 0;   // Reference value.
      uint8_t ix_      = 0;   // Index of element in regiser group.
      uint8_t size_    = 0;   // Number of bytes of element covered by this object.
      uint8_t reg_     = 0;   // Number of data register.
      uint8_t ixReg_   = 0;   // Number of index register (if indexed).
      uint8_t field_   = 0;   // Field (if segmented load).
    };

    /// Collection of vector load/store reference (Whisper) addresses for a single
    /// instruction and the associated address bounds. The bounds are there to avoid
    /// the cost of searching for an address that is not in the refs.
    struct VecRefs
    {
      bool empty() const
      { return refs_.empty(); }

      bool isOutOfBounds(uint64_t addr) const
      {
	if (empty())
	  return true;
	return addr < low_ or addr > high_;
      }

      bool isOutOfBounds(uint64_t low, uint64_t high) const
      {
	if (empty())
	  return true;
	return low > high_ or high < low_;
      }

      bool isOutOfBounds(const MemoryOp& op) const
      {
	if (empty())
	  return true;
	assert(op.size_ > 0);
	return isOutOfBounds(op.pa_, op.pa_ + op.size_ - 1);
      }

      bool isOutOfBounds(const VecRef& ref) const
      {
	if (empty())
	  return true;
	assert(ref.size_ > 0);
	return isOutOfBounds(ref.pa_, ref.pa_ + ref.size_ - 1);
      }

      void add(unsigned ix, uint64_t addr, uint64_t data, unsigned size, unsigned vecReg,
	       unsigned ixReg, unsigned field)
      {
	assert(size > 0);

	uint64_t l = addr, h = addr + size - 1;
	if (empty())
	  {
	    low_ = l;
	    high_ = h;
	  }
	else
	  {
	    low_ = std::min(low_, l);
	    high_ = std::max(high_, h);
	  }
	refs_.push_back(VecRef(ix, addr, data, size, vecReg, ixReg, field));
      }

      std::vector<VecRef> refs_;
      uint64_t low_  = 0;   // Low address in refs
      uint64_t high_ = 0;   // High address in refs
    };

    // Per hart information related to MCM.
    struct HartData
    {
      McmInstrVec instrVec_;
      MemoryOpVec pendingWrites_;
      RegTimeVec regTime_;
      RegProducerVec regProducer_;

      // Retired but not yet drained stores. Candidates for forwarding.
      std::set<McmInstrIx> undrainedStores_;

      // Set of stores that may affect (through forwarding) the currently executing load
      // instruction.
      std::set<McmInstrIx> forwardingStores_;

      McmInstrIx currentLoadTag_ = 0;  // Currently executing load instruction.

      // Reference vec ld/st store data produced by whisper.
      std::map<McmInstrIx, VecRefs> vecRefMap_;

      // Dependency time of most recent branch in program order or 0 if branch does not
      // depend on a prior memory instruction.
      uint64_t branchTime_ = 0;
      uint64_t branchProducer_;

      // Dependency time of most recent vsetvl or vsetvli in program order or
      // 0 if vset does not depend on prior memory instruction.
      uint64_t vlTime_ = 0;
      uint64_t vlProducer_ = 0;

      McmInstrIx currentInstrTag_ = 0;
      uint64_t sinvalVmaTime_ = 0;
      uint64_t sinvalVmaTag_ = 0;
    };

    std::vector<HartData> hartData_;    // One entry per hart.

    MemoryOpVec sysMemOps_;             // Memory ops of all harts ordered by time.

    uint64_t time_ = 0;

    unsigned pageSize_ = 4096;
    unsigned pageShift_ = 12;   // Log2(pageSize_);
    unsigned lineSize_ = 64;    // Merge buffer line size.
    unsigned lineShift_ = 6;    // log2(lineSize_);

    bool writeOnInsert_ = false;

    // Check whole merge buffer line if true otherwise check bytes covered by store
    // instructions.
    bool checkWholeLine_ = false;

    // When enabled, skip read data checks on containing addresses.
    uint64_t skipReadCheckAddr_ = 0;
    unsigned skipReadCheckSize_ = 0;
    bool skipReadCheckEnable_ = false;

    std::vector<bool> ppoEnabled_;

    bool isTso_ = false;  // True if total-store-ordering model.
  };

}
