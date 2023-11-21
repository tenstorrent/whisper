#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <unordered_set>
#include "DecodedInst.hpp"


namespace WdRiscv
{
  template <typename URV>
  class Hart;

  using McmInstrIx = uint32_t;
  using MemoryOpIx = uint32_t;

  struct MemoryOp
  {
    uint64_t time_ = 0;
    uint64_t physAddr_ = 0;
    uint64_t data_ = 0;
    uint64_t rtlData_ = 0;
    McmInstrIx instrTag_ = 0;
    uint8_t hartIx_ = 0;
    uint8_t size_ = 0;
    bool isRead_ = false;
    bool failRead_ = false;
    bool canceled_ = false;

    bool isCanceled() const { return canceled_; }
    void cancel() { canceled_ = true; }
  };


  struct McmInstr
  {
    // memOps contains indices into an array of MemoryOp items.
    std::vector<MemoryOpIx> memOps_;
    uint64_t physAddr_ = 0;   // Data address for ld/store instruction.
    uint64_t data_ = 0;       // Data for load/sore instructions.
    uint64_t addrTime_ = 0;   // Time address register was produced (for ld/st/amo).
    uint64_t dataTime_ = 0;   // Time data register was produced (for st/amo).
    McmInstrIx addrProducer_ = 0;
    McmInstrIx dataProducer_ = 0;
    DecodedInst di_;
    McmInstrIx tag_ = 0;
    uint8_t size_ = 0;        // Data size for load/store insructions.
    bool retired_ = false;
    bool canceled_ = false;
    bool isLoad_ = false;
    bool isStore_ = false;
    bool complete_ = false;

    bool isMemory() const { return isLoad_ or isStore_; }

    bool isRetired() const { return retired_; }

    bool isCanceled() const { return canceled_; }

    void cancel() { canceled_ = true; }

    void addMemOp(MemoryOpIx memOpIx)
    {
      if (std::find(memOps_.begin(), memOps_.end(), memOpIx) != memOps_.end())
	{
	  std::cerr << "McmInstr::addMemOp: Error: Op already added\n";
	  assert(0 && "McmInstr::addMemOp: Op ix already present");
	}
      memOps_.push_back(memOpIx);
    }

    bool overlaps(const McmInstr& other) const
    {
      if (size_ == 0 or other.size_ == 0)
	{
	  std::cerr << "McmInstr::overlaps: Error: tag1=" << tag_
		    << " tag2=" << other.tag_ << " zero data size\n";
	  // assert(0 && "McmInstr::overlaps: zero data size\n");
	}
      if (physAddr_ == other.physAddr_)
	return true;
      if (physAddr_ < other.physAddr_)
	return other.physAddr_ - physAddr_ < size_;
      return physAddr_ - other.physAddr_ < other.size_;
    }

    bool overlaps(const MemoryOp& op) const
    {
      if (size_ == 0 or op.size_ == 0)
	{
	  std::cerr << "McmInstr::overlaps: Error: tag1=" << tag_
		    << " tag2=" << op.instrTag_ << " zero data size\n";
	  // assert(0 && "McmInstr::overlaps: zero data size\n");
	}
      if (physAddr_ == op.physAddr_)
	return true;
      if (physAddr_ < op.physAddr_)
	return op.physAddr_ - physAddr_ < size_;
      return physAddr_ - op.physAddr_ < op.size_;
    }
  };


  template <typename URV>
  class Mcm
  {
  public:

    Mcm(unsigned hartCount, unsigned mergeBufferSize);

    ~Mcm();

    /// Initiate an out of order read for a load instruction. If a
    /// preceding overlapping store has not yet left the merge/store
    /// buffer then forward data from that store to the read operation;
    /// otherwise, get the data from glbal memory. Return true on
    /// success and false if global memory is not readable (in the
    /// case where we do not forward).
    bool readOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		uint64_t physAddr, unsigned size, uint64_t rtlData);
    
    /// This is a write operation bypassing the merge buffer.
    bool bypassOp(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		  uint64_t physAddr, unsigned size, uint64_t rtlData);

    /// Initiate a merge buffer write.  All associated store write
    /// transactions are marked completed. Write instructions where
    /// all writes are complete are marked complete. Return true on
    /// success.  The given physical address must be a multiple of the
    /// merge buffer line size (which is also the cache line
    /// size). The rtlData vector must be of size n or larger where n
    /// is the merge buffer line size. The rtlData bytes will be
    /// placed in memory in consecutive locations starting with
    /// physAddr.
    bool mergeBufferWrite(Hart<URV>& hart, uint64_t time, uint64_t physAddr,
			  const std::vector<uint8_t>& rtlData,
			  const std::vector<bool>& mask);

    /// Insert a write operation for the given instruction into the
    /// merge buffer removing it from the store buffer. Return true on
    /// success. Return false if no such operation is in the store
    /// buffer.
    bool mergeBufferInsert(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
			   uint64_t physAddr, unsigned size,
			   uint64_t rtlData);

    /// Cancel all the memory operations associted with the given tag. This is
    /// done when a speculative instruction is canceled or when an instruction
    /// is trapped.
    void cancelInstruction(Hart<URV>& hart, uint64_t instrTag);

    /// This is called when an instruction is retired.
    bool retire(Hart<URV>& hart, uint64_t time, uint64_t instrTag,
		const DecodedInst& di);

    bool setCurrentInstruction(Hart<URV>& hart, uint64_t instrTag);

    /// Return the load value of the current target instruction
    /// (set with setCurrentInstruction).
    bool getCurrentLoadValue(Hart<URV>& hart, uint64_t addr, unsigned size,
			     uint64_t& value);

    /// Return the merge buffer line size in bytes.
    unsigned mergeBufferLineSize() const
    { return lineSize_; }

    /// Skip checking RTL read-op values against this model. This is used
    /// for items like the CLINT timer where we cannot match the RTL.
    void skipReadCheck(uint64_t addr)
    { skipReadCheck_.insert(addr); }

    /// Enable/disable total-store-order.
    void enableTso(bool flag)
    { isTso_ = flag; }

    bool ppoRule1(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule2(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule3(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule4(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule5(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule6(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule7(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule8(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule9(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule10(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule11(Hart<URV>& hart, const McmInstr& instr) const;

    bool ppoRule12(Hart<URV>& hart, const McmInstr& instr) const;

    uint64_t latestOpTime(const McmInstr& instr) const
    {
      if (not instr.complete_)
	{
	  std::cerr << "Mcm::latestOpTime: Called on an incomplete instruction\n";
	  assert(0 && "Mcm::lasestOpTime: Incomplete instr");
	}
      uint64_t time = 0;
      for (auto opIx : instr.memOps_)
	if (opIx < sysMemOps_.size())
	  time = std::max(time, sysMemOps_.at(opIx).time_);
      return time;
    }

    uint64_t earliestOpTime(const McmInstr& instr) const
    {
      if (not instr.complete_ and instr.memOps_.empty())
	{
	  std::cerr << "Mcm::earliestOpTime: Called on an incomplete instruction\n";
	  assert(0 && "Mcm::earliestOpTime: Incomplete instr");
	}
      uint64_t time = ~uint64_t(0);
      for (auto opIx : instr.memOps_)
	if (opIx < sysMemOps_.size())
	  time = std::min(time, sysMemOps_.at(opIx).time_);
      return time;
    }

    bool isBeforeInMemoryTime(const McmInstr& a, const McmInstr& b) const
    {
      // if (a.complete_ and not b.complete_)
      // return true;
      if (not a.complete_ and b.complete_)
	return false;
      if (not a.complete_ and not b.complete_)
	{
	  std::cerr << "Mcm::isBeforeInMemoryTime: Both instructions incomplete\n";
	  assert(0 && "Mcm::isBeforeInMemoryTime: Both instructions incomplete");
	  return false;
	}
      uint64_t aTime = latestOpTime(a);
      uint64_t bTime = earliestOpTime(b);
      if (a.isStore_ and b.isStore_ and aTime == bTime)
	return a.tag_ < b.tag_;
      return aTime < bTime;
    }

    /// Configure checking whole merge buffer line (versus checking
    /// bytes covered by stores).
    void setCheckWholeMbLine(bool flag)
    { checkWholeLine_ = flag; }

  protected:

    using MemoryOpVec = std::vector<MemoryOp>;

    void cancelReplayedReads(McmInstr*);

    /// Remove from hartPendingWrites_ the write ops falling with the given RTL
    /// line and masked by rtlMask (rtlMask bit is on for op bytes) and place
    /// them sorted by instr tag in coveredWrites. Write ops may not straddle
    /// line boundary. Write ops may not be partially masked.
    bool collectCoveredWrites(Hart<URV>& hart, uint64_t time, uint64_t lineBegin,
			      uint64_t lineSize, const std::vector<bool>& rtlMask,
			      MemoryOpVec& coveredWrites);

    /// Forward from a store to a read op. Return true on success.
    /// Return false if instr is not retired, is canceled, is not a
    /// store (amo couts as store), or does not match range address of
    /// op.  Mask is the mask of bits of op to be updated by the
    /// forward (bypass) operartion and is updated (bits cleared) if
    /// some parts of op, covered by the mask, are successfully
    /// updated.
    bool forwardTo(const McmInstr& instr, MemoryOp& op, uint64_t& mask);

    /// Forward to the given read op from the stores of the retired
    /// instructions ahead of tag.
    bool forwardToRead(Hart<URV>& hart, uint64_t tag, MemoryOp& op);

    /// Determine the source and destination registers of the given
    /// instruction.
    void identifyRegisters(const DecodedInst& di,
			   std::vector<unsigned>& sourceRegs,
			   std::vector<unsigned>& destRegs);

    bool instrHasRead(const McmInstr& instr) const;
    bool instrHasWrite(const McmInstr& instr) const;

    bool checkStoreComplete(const McmInstr& instr) const;

    bool checkStoreData(unsigned hartId, const McmInstr& insrt) const;

    bool checkLoadComplete(const McmInstr& instr) const;

    void clearMaskBitsForWrite(const McmInstr& storeInstr,
			       const McmInstr& target, uint64_t& mask) const;

    void cancelNonRetired(unsigned hartIx, uint64_t instrTag);

    bool checkRtlWrite(unsigned hartId, const McmInstr& instr,
		       const MemoryOp& op) const;

    bool checkRtlRead(unsigned hartId, const McmInstr& instr,
		      const MemoryOp& op) const;

    bool updateTime(const char* method, uint64_t time);

    McmInstr* findInstr(unsigned hartIx, McmInstrIx tag);

    McmInstr* findOrAddInstr(unsigned hartIx, McmInstrIx tag);

    void cancelInstr(McmInstr& instr)
    {
      if (instr.isCanceled())
	std::cerr << "Instr with tag=" << instr.tag_ << " already canceled\n";
      instr.cancel();
      for (auto memIx : instr.memOps_)
	{
	  if (sysMemOps_.at(memIx).isCanceled())
	    std::cerr << "Mcm::cancelInstr: Error: op already canceled\n";
	  sysMemOps_.at(memIx).cancel();
	}
    }

    void updateDependencies(const Hart<URV>& hart, const McmInstr& instr);

    void setProducerTime(unsigned hartIx, McmInstr& instr);

    /// Map register number of operand opIx to a unique integer by adding
    /// an offset: integer register have 0 offset, fp regs have 32, and
    /// csr regs have 64.
    unsigned effectiveRegIx(const DecodedInst& di, unsigned opIx) const;

  private:

    const unsigned intRegOffset_ = 0;
    const unsigned fpRegOffset_ = 32;
    const unsigned csRegOffset_ = 64;
    const unsigned totalRegCount_ = csRegOffset_ + 4096; // 4096: max csr count.


    using McmInstrVec = std::vector<McmInstr>;

    using RegTimeVec = std::vector<uint64_t>; // Map reg index to time.
    using RegProducer = std::vector<uint64_t>; // Map reg index to instr tag.

    MemoryOpVec sysMemOps_;  // Memory ops of all cores.
    std::vector<McmInstrVec> hartInstrVecs_; // One vector per hart.
    std::vector<MemoryOpVec> hartPendingWrites_; // One vector per hart.

    uint64_t time_ = 0;
    unsigned lineSize_ = 64; // Merge buffer line size.

    bool writeOnInsert_ = false;

    // Check whole merge buffer line if true otherwise check bytes
    // covered by store instructions.
    bool checkWholeLine_ = false;

    bool isTso_ = false;  // True if total-store-ordering model.

    std::vector<McmInstrIx> currentInstrTag_;

    std::vector<RegTimeVec> hartRegTimes_;  // One vector per hart.
    std::vector<RegProducer> hartRegProducers_;  // One vector per hart.

    // Dependency time of most recent branch in program order or 0 if
    // branch does not depend on a prior memory instruction.
    std::vector<uint64_t> hartBranchTimes_;
    std::vector<uint64_t> hartBranchProducers_;

    std::unordered_set<uint64_t> skipReadCheck_;
  };

}
