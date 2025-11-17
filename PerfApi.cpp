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

#include <cinttypes>
#include <iomanip>
#include "PerfApi.hpp"

using namespace TT_PERF;

using CSRN = WdRiscv::CsrNumber;
using std::cerr;

PerfApi::PerfApi(System64& system)
  : system_(system)
{
  unsigned n = system.hartCount();

  traceFiles_.resize(n);

  hartPacketMaps_.resize(n);
  hartStoreMaps_.resize(n);
  hartLastRetired_.resize(n, initHartLastRetired);
  hartRegProducers_.resize(n);

  for (auto& producers : hartRegProducers_)
    producers.resize(totalRegCount_);
}


std::shared_ptr<Hart64>
PerfApi::checkHart(const char* caller, unsigned hartIx)
{
  auto hart = getHart(hartIx);
  if (not hart)
    {
      std::cerr << caller << ": Bad hart index: " << hartIx << '\n';
      assert(0 && "Error: Assertion failed -- bad hart index");
    }
  return hart;
}


std::shared_ptr<InstrPac>
PerfApi::checkTag(const char* caller, unsigned hartIx, uint64_t tag)
{
  auto& packetMap = hartPacketMaps_.at(hartIx);
  auto iter = packetMap.find(tag);
  if (iter != packetMap.end())
    return iter->second;
  std::cerr << caller << ": Unknown tag (never fetched): " << tag << '\n';
  assert(0 && "Error: Assertion failed -- unknown tag");
  return nullptr;
}


bool
PerfApi::checkTime(const char* caller, uint64_t time)
{
  if (time < time_)
    {
      std::cerr << caller << ": Bad time: " << time << '\n';
      assert(0 && "Error: Assertion failed -- bad time value");
      return false;
    }
  time_ = time;
  return true;
}


bool
PerfApi::fetch(unsigned hartIx, uint64_t time, uint64_t tag, uint64_t vpc,
	       bool& trap, ExceptionCause& cause, uint64_t& trapPc)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_fetch %" PRIu64 " 0x%" PRIx64 "\n",
            hartIx, time, tag, vpc);

  auto hart = checkHart("Fetch", hartIx);
  if (not hart)
    return false;

  if (not checkTime("Fetch", time))
    return false;

  if (tag == 0)
    {
      std::cerr << "Error: PerfApi::fetch: Hart-ix=" << hartIx << " tag=" << tag
		<< " zero tag is reserved.\n";
      assert(0 && "Error: Assertion failed -- zero tag");
      return false;
    }

  auto& packetMap = hartPacketMaps_.at(hartIx);
  if (not packetMap.empty() and packetMap.rbegin()->first >= tag)
    {
      std::cerr << "Error: PerfApi::fetch: Hart-ix=" << hartIx << "tag=" << tag
		<< " tag is not in increasing order.\n";
      assert(0 && "Error: Assertion failed -- tag out of order");
      return false;
    }

  auto packet = getInstructionPacket(hartIx, tag);
  if (packet)
    {
      std::cerr << "Error: PerfApi::fetch: Hart-ix=" << hartIx << "tag=" << tag
		<< " tag is already fetched.\n";
      return false;   // Tag already fetched.
    }

  auto prev = this->prevFetch_;
  if (prev and not prev->predicted() and not prev->trapped() and not prev->executed())
    {
      bool sequential = prev->instrVa() + prev->decodedInst().instSize() == vpc;
      if (not prev->decodedInst().isBranch())
	{
	  if (not sequential)
	    prev->predictBranch(true, vpc);
	}
      else
	{
	  if (sequential)
	    prev->predictBranch(false, vpc);
	}
    }

  uint64_t ppc = 0, ppc2 = 0;  // Physical pc.
  uint32_t opcode = 0;
  uint64_t gpc = 0; // Guest physical pc.
  cause = hart->fetchInstNoTrap(vpc, ppc, ppc2, gpc, opcode);

  packet = std::make_shared<InstrPac>(tag, vpc, ppc, ppc2);
  assert(packet);
  packet->fetched_ = true;
  packet->opcode_= opcode;
  insertPacket(hartIx, tag, packet);
  if (not decode(hartIx, time, tag))
    assert(0 && "Error: Assertion failed -- failed to decode");
  prevFetch_ = packet;

  trap = cause != ExceptionCause::NONE;
  packet->trap_ = trap;
  packet->trapCause_ = uint64_t(cause);

  if (prev and not prev->trapped() and prev->executed() and prev->nextIva_ != vpc)
    {
      packet->shouldFlush_ = true;
      packet->flushVa_ = prev->nextIva_;
    }

  if (trap)
    {
      prevFetch_ = nullptr;
      trapPc = 0;
    }

  return true;
}


bool
PerfApi::decode(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_decode %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Decode", time))
    return false;

  auto hartPtr = checkHart("Decode", hartIx);
  if (not hartPtr)
    return false;
  auto& hart = *hartPtr;

  auto packPtr = checkTag("Decode", hartIx, tag);
  if (not packPtr)
    return false;
  auto& packet = *packPtr;

  if (packet.decoded())
    return true;

  hart.decode(packet.instrVa(), packet.instrPa(), packet.opcode_, packet.di_);
  packet.decoded_ = true;

  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  determineExplicitOperands(packet);

  // Determine effective group multiplier of vector operands. We do this before adding
  // explicit operands as we may be producing vtype which affects LMUL.
  if (packet.di_.isVector())
    getVectorOperandsLmul(hart, packet);

  determineImplicitOperands(packet);

  // Collect producers of operands of this instruction.
  auto& producers = hartRegProducers_.at(hartIx);
  for (unsigned i = 0; i < packet.operandCount_; ++i)
    {
      auto& op = packet.operands_.at(i);
      auto mode = op.mode;
      auto type = op.type;

      if (type == OT::Imm)
        continue;

      assert(type != OT::None and mode != OM::None);
      if (mode == OM::None)
        continue;

      unsigned regNum = op.number;
      unsigned gri = globalRegIx(type, regNum);

      if (type != OT::VecReg)
        packet.opProducers_.at(i).scalar = producers.at(gri);
      else
        {
          auto lmul = op.lmul;
          assert(lmul != 0 and lmul <= maxEffLmul_);
          for (unsigned n = 0; n < lmul; ++n)
            {
              unsigned vgri = gri + n;
              packet.opProducers_.at(i).vec.push_back(producers.at(vgri));
            }
	}
    }

  // Mark this insruction as the producer of each of its destination registers.
  for (unsigned i = 0; i < packet.operandCount_; ++i)
    {
      auto& op = packet.operands_.at(i);
      auto mode = op.mode;
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  unsigned regNum = op.number;
          auto type = op.type;
	  unsigned gri = globalRegIx(type, regNum);
	  if (regNum == 0 and type == OT::IntReg)
	    continue;  // Reg X0 has no producer

          if (type != OT::VecReg)
            producers.at(gri) = packPtr;
          else
            {
              auto lmul = op.lmul;
              assert(lmul <= maxEffLmul_);
              for (unsigned n = 0; n < lmul; ++n)
                {
                  unsigned vgri = gri + n;
                  producers.at(vgri) = packPtr;
                }
            }
	}
    }

  return true;
}


bool
PerfApi::execute(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_execute %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Execute", time))
    return false;

  auto hartPtr = checkHart("Execute", hartIx);
  if (not hartPtr)
    return false;

  auto& hart = *hartPtr;

  auto pacPtr = checkTag("execute", hartIx, tag);
  if (not pacPtr)
    return false;

  auto& packet = *pacPtr;
  const auto& di = packet.decodedInst();

#if 0
  if (di.isLr())
    return true;   // LR is executed and retired at PerApi::retire.
#endif

  auto& packetMap = hartPacketMaps_.at(hartIx);

  if (packet.executed())
    {
      // Instruction is being re-executed. Must be load/store. Every instruction that
      // depends on it must be re-executed.
      if (not di.isLoad() and not di.isStore())
	assert(0 && "Error: Assertion failed -- re-executed instruction is not ld/store");
      auto iter = packetMap.find(packet.tag());
      assert(iter != packetMap.end());
      for ( ; iter != packetMap.end(); ++iter)
	{
	  auto succ = iter->second;   // Successor packet
	  if (succ->dependsOn(packet))
	    succ->executed_ = false;
	}
    }

  // Collect register operand values. Some values come from in-flight instructions
  // (register renaming).
  bool peekOk = collectOperandValues(hart, packet);

  // Execute the instruction: Poke source register values, execute, recover destination
  // register values.
  if (not execute(hartIx, packet))
    assert(0 && "Error: Assertion failed -- failed to execute isntruction");

  // We should not fail to read an operand value unless there is an exception.
  if (not peekOk)
    assert(packet.trap_);

  packet.executed_ = true;
  packet.execTime_ = time;
  if (packet.predicted_)
    packet.mispredicted_ = packet.prTarget_ != packet.nextIva_;

  if (packet.isBranch())
    {
      // Check if the next instruction in program order is at the right PC.
      auto iter = packetMap.find(packet.tag());
      ++iter;
      if (iter != packetMap.end())
	{
	  auto& next = *(iter->second);
	  if (next.iva_ != packet.nextIva_)
	    {
	      next.shouldFlush_ = true;
	      next.flushVa_ = packet.nextIva_;
	    }
	  if (next.executed_)
	    {
	      packet.shouldFlush_ = true;
	      packet.flushVa_ = packet.iva_;
	    }
	}
    }

  return true;
}


bool
PerfApi::execute(unsigned hartIx, InstrPac& packet)
{
  assert(packet.decoded());

  auto hartPtr = getHart(hartIx);
  assert(hartPtr);
  auto& hart = *hartPtr;

  uint64_t prevPc = hart.peekPc();
  uint64_t prevInstrCount = hart.getInstructionCount();

  hart.pokePc(packet.instrVa());
  hart.setInstructionCount(packet.tag_ - 1);

  uint64_t prevMstatus = 0;
  if (not hart.peekCsr(CSRN::MSTATUS, prevMstatus))
    assert(0 && "Error: Assertion failed");

  // Save hart register values corresponding to packet operands in prevVal.
  std::array<OpVal, 8> prevVal;
  bool saveOk = saveHartValues(hart, packet, prevVal);

  // Install packet operand values (some obtained from previous in-flight instructions)
  // into the hart registers.
  bool setOk = setHartValues(hart, packet);

  assert(hartLastRetired_.at(hartIx) == initHartLastRetired ||  packet.tag_ > hartLastRetired_.at(hartIx));

  // Only make the time adjustment when the hart has retired at least one instruction. 
  if (hartLastRetired_.at(hartIx) != initHartLastRetired)
    {
      auto itag = std::bit_cast<int64_t>(packet.tag_);
      auto ilast = std::bit_cast<int64_t>(hartLastRetired_.at(hartIx));
      assert(itag >= 0 and ilast >= 0);  // Check overflow
      hart.adjustTime(itag - ilast - 1);
    }

  const auto& di = packet.decodedInst();

  unsigned imsicId = 0, imsicGuest = 0;
  if (di.isCsr())
    saveImsicTopei(hart, CSRN(di.ithOperand(2)), imsicId, imsicGuest);

  packet.privMode_ = hart.privilegeMode();
  packet.virtMode_ = hart.virtMode();

  // Execute
  skipIoLoad_ = true;   // Load from IO space takes effect at retire.
  hart.singleStep();
  skipIoLoad_ = false;

  bool trap = hart.lastInstructionTrapped();
  packet.trap_ = trap;
  packet.interrupt_ = hart.lastInstructionInterrupted();
  if (trap)
    packet.trapCause_ = hart.lastTrapCause();

  // If save fails or set fails, there must be a trap.
  if (not saveOk or not setOk)
    assert(trap);

  // Record PC of subsequent packet.
  packet.nextIva_ = hart.peekPc();

  recordExecutionResults(hart, packet);

  // Undo changes to the hart.

  if (hartLastRetired_.at(hartIx) != initHartLastRetired)
    hart.adjustTime(-(int64_t)(packet.tag_ - hartLastRetired_.at(hartIx)));  // Restore timer value.
  else
    hart.adjustTime(-1); // untick one cycle for the first instruction

  // Restore CSRs modified by the instruction or trap. TODO: For vector ld/st we have to
  // restore partially modified vectors.
  if (di.isXRet() and not trap)
    {   // For an MRET/SRET/... Privilege may have been lowered. Restore it before restoring CSRs.
      hart.setVirtualMode(hart.lastVirtMode());
      hart.setPrivilegeMode(hart.lastPrivMode());
    }

  // Restore CSR changes due to a trap or to mret/sret or to side effects from
  // vector/fp instructions (MSTATUS.VS, MSTATUS.FS, FCSR, etc...).
  std::vector<CSRN> csrns;
  hart.lastCsr(csrns);
  for (auto csrn : csrns)
    {
      uint64_t value = hart.lastCsrValue(csrn);
      if (not hart.pokeCsr(csrn, value))
        assert(0);
      if (trap or di.isXRet())
        {
          Operand op;
          op.type = OperandType::CsReg;
          op.mode = OperandMode::Write; // Arbitrary.
          op.number = unsigned(csrn);
          if (not hart.peekCsr(csrn, op.value.scalar))
            assert(0);
          packet.changedCsrs_.at(packet.changedCsrCount_++) = op;
        }
    }

  if (trap)
    {  // Privilege raised.  Restore it after restoring CSRs.
      hart.setVirtualMode(hart.lastVirtMode());
      hart.setPrivilegeMode(hart.lastPrivMode());
    }

  // Restore hart registers that we changed before single step.
  restoreHartValues(hart, packet, prevVal);

  uint64_t mstatus = 0;
  if (not hart.peekCsr(CSRN::MSTATUS, mstatus))
    assert(0 && "Error: Assertion failed");
  if (mstatus != prevMstatus)
    hart.pokeCsr(CSRN::MSTATUS, prevMstatus);

  if (di.isCsr())
    restoreImsicTopei(hart, CSRN(di.ithOperand(2)), imsicId, imsicGuest);

  hart.setTargetProgramFinished(false);
  hart.pokePc(prevPc);
  hart.setInstructionCount(prevInstrCount);

  // Collect the page table walks.
  packet.fetchWalks_ = hart.virtMem().getFetchWalks();
  packet.dataWalks_ = hart.virtMem().getDataWalks();

  hart.clearTraceData();

  return true;
}


bool
PerfApi::retire(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_retire %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Retire", time))
    return false;

  auto hartPtr = checkHart("Retire", hartIx);
  if (not hartPtr)
    return false;

  auto pacPtr = checkTag("Retire", hartIx, tag);
  if (not pacPtr)
    return false;

  auto& packet = *pacPtr;
  auto& hart = *hartPtr;

  if (hartLastRetired_.at(hartIx) != initHartLastRetired && tag <= hartLastRetired_.at(hartIx))
    {
      std::cerr << "Error: Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Out of order retire\n";
      return false;
    }
  hartLastRetired_.at(hartIx) = tag;

  if (packet.retired())
    {
      std::cerr << "Error: Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Tag retired more than once\n";
      return false;
    }

  if (packet.instrVa() != hart.peekPc())
    {
      std::cerr << "Error: Hart=" << hartIx << " time=" << time << " tag=" << tag << std::hex
		<< " Wrong pc at retire: 0x" << packet.instrVa() << " expecting 0x"
		<< hart.peekPc() << '\n' << std::dec;
      return false;
    }

  hart.setInstructionCount(tag - 1);
  auto* traceFile = traceFiles_.at(hartIx);

  hart.singleStep(nullptr);

  const auto& di = packet.decodedInst();

  using CN = WdRiscv::CsrNumber;
  if (di.isCsr() and di.op2() == unsigned(CN::MCYCLE) and di.op0() != 0)
    {
      // CSR instr using MCYCLE. Force the value we saw at exec to aoid exec/retire
      // mismatch since the cycle counter keeps incrementing in-between.
      hart.pokeIntReg(di.op0(), packet.destValues_.at(0).second.scalar);
    }

  if (traceFile)
    {
      hart.virtMem().setFetchWalks(packet.fetchWalks_);  // We print the walk from execute.
      hart.virtMem().setDataWalks(packet.dataWalks_);
      hart.printInstCsvTrace(packet.di_, traceFile);
    }

  // Sanity check. Results at execute and retire must match.
#if 1
  if (not checkExecVsRetire(hart, packet))
    assert(0 && "Error: Assertion failed");
#endif

  // Undo renaming of destination registers.
  undoDestRegRename(hartIx, packet);

  bool trap = hart.lastInstructionTrapped();
  packet.trap_ = packet.trap_ or trap;
  packet.interrupt_ = hart.lastInstructionInterrupted();
  if (packet.trap_)
    packet.trapCause_ = hart.lastTrapCause();

#if 0
  if (di.isLr())
    {
      // Record PC of subsequent packet.
      packet.nextIva_ = hart.peekPc();

      recordExecutionResults(hart, packet);
      packet.executed_ = true;
    }
#endif

  packet.retired_ = true;

  // AMO/SC drain here (at retire) if more than 1 hart; otherwise, they drain at drain
  // stage.
  bool amoSc = packet.isSc() or packet.isAmo();
  bool drained = amoSc and system_.hartCount() > 1;  // True if drained in this method.
  if (drained)
    {
      uint64_t sva = 0, spa1 = 0, spa2 = 0, sval = 0;
      unsigned size = hart.lastStore(sva, spa1, spa2, sval);
      if (size != 0)   // Could be zero for a failed sc
	if (not commitMemoryWrite(hart, spa1, spa2, size, packet.stData_))
	  assert(0 && "Error: Assertion failed -- could not commit SC/AMO data to memory");
      if (packet.isSc())
	hart.cancelLr(WdRiscv::CancelLrCause::SC);
      auto& storeMap = hartStoreMaps_.at(hartIx);
      packet.drained_ = true;
      storeMap.erase(packet.tag());
    }

  // Clear dependency on other packets to expedite release of packet memory.
  for (auto& producer : packet.opProducers_)
    producer.clear();

  // Erase packet from packet map. Stores erased at drain time.
  auto& pacMap = hartPacketMaps_.at(hartIx);

  if (drained)
    pacMap.erase(packet.tag());
  else
    {
      bool store = packet.isStore() or di.isVectorStore() or di.isCbo_zero() or di.isAmo();
      if (not store)
        pacMap.erase(packet.tag());   // Stores erased at drain stage.
    }

  return true;
}


bool
PerfApi::checkExecVsRetire(const Hart64& hart, const InstrPac& packet)
{
  unsigned hartIx = hart.sysHartIndex();
  auto tag = packet.tag_;

  bool retireTrap = hart.lastInstructionTrapped();
  if (packet.trap_ != retireTrap)
    {
      cerr << "Error: Hart=" << hartIx << " tag=" << tag << " trap on execute/retire "
           << "differ: " << packet.trap_ << '/' << retireTrap << '\n';
      return false;
    }

  if (packet.nextPc() != hart.peekPc())
    {
      cerr << "Hart=" << hartIx << " tag=" << tag << " execute and retire PCs differ:"
           << " 0x" << std::hex << packet.nextPc() << " & 0x" << hart.peekPc() << std::dec << '\n';
      return false;
    }

  if (packet.trap_)
     return true;

  if (int reg = hart.lastIntReg(); reg > 0)
    {
      uint64_t val = hart.peekIntReg(reg);
      uint64_t execVal = packet.destValues_.at(0).second.scalar;
      if (val != execVal)
        cerr << "Error: Hart=" << hartIx << " tag=" << tag << " retire & exec vals differ:"
             << " 0x" << std::hex << val << " & 0x" << execVal << std::dec << '\n';
      return val == execVal;
    }

  if (int reg = hart.lastFpReg(); reg >= 0)
    {
      uint64_t val = 0;
      if (not hart.peekFpReg(reg, val))
        assert(0 && "Error: Assertion failed");
      uint64_t execVal = packet.destValues_.at(0).second.scalar;
      if (val != execVal)
        cerr << "Error: Hart=" << hartIx << " tag=" << tag << " exec & retire vals differ:"
             << " 0x" << std::hex << val << " & 0x" << execVal << std::dec << '\n';
      return val == execVal;
    }


  unsigned group = 0;
  int vr = hart.lastVecReg(packet.di_, group);
  if (vr >= 0)
    {
      std::vector<uint8_t> retire;
      hart.peekVecRegLsb(vr, retire);
      const auto& exec = packet.destValues_.at(0).second.vec;
      assert(retire.size() <= exec.size());
      for (unsigned i = 0; i < retire.size(); ++i)
        if (retire.at(i) != exec.at(i))
          {
            cerr << "Error: Hart=" << hartIx << " tag=" << tag << " lmul=" << group
                 << " vd=" << vr << " byte-ccount=" << retire.size()
                 << " exec & retire vec vals differ\n";
            cerr << "  retire: 0x" << std::hex;
            cerr << std::hex;
            unsigned count = retire.size();
            for (unsigned j = 0; j < count; ++j)
              cerr << std::setw(2) << std::setfill('0') << unsigned(retire.at(count-1-j));
            cerr << "\n";
            cerr << "  exec:   0x" << std::hex;
            for (unsigned j = 0; j < count; ++j)
              cerr << std::setw(2) << std::setfill('0') << unsigned(exec.at(count-1-j));
            cerr << "\n";
            return false;
          }
    }

  return true;
}


WdRiscv::ExceptionCause
PerfApi::translateInstrAddr(unsigned hartIx, uint64_t va, uint64_t& pa)
{
  auto hart = checkHart("Translate-instr-addr", hartIx);
  hart->clearPageTableWalk();
  bool r = false, w = false, x = true;
  auto pm = hart->privilegeMode();
  pa = va;
  if (pm == WdRiscv::PrivilegeMode::Machine or not hart->isRvs())
    return WdRiscv::ExceptionCause::NONE;
  return  hart->transAddrNoUpdate(va, pm, hart->virtMode(), r, w, x, pa);
}


WdRiscv::ExceptionCause
PerfApi::translateLoadAddr(unsigned hartIx, uint64_t va, uint64_t& pa)
{
  auto hart = checkHart("translate-load-addr", hartIx);
  hart->clearPageTableWalk();
  bool r = true, w = false, x = false;
  auto pm = hart->privilegeMode();
  pa = va;
  if (pm == WdRiscv::PrivilegeMode::Machine or not hart->isRvs())
    return WdRiscv::ExceptionCause::NONE;
  return  hart->transAddrNoUpdate(va, pm, hart->virtMode(), r, w, x, pa);
}


WdRiscv::ExceptionCause
PerfApi::translateStoreAddr(unsigned hartIx, uint64_t va, uint64_t& pa)
{
  auto hart = checkHart("translate-store-addr", hartIx);
  hart->clearPageTableWalk();
  bool r = false, w = true, x = false;
  auto pm = hart->privilegeMode();
  pa = va;
  if (pm == WdRiscv::PrivilegeMode::Machine or not hart->isRvs())
    return WdRiscv::ExceptionCause::NONE;
  return  hart->transAddrNoUpdate(va, pm, hart->virtMode(), r, w, x, pa);
}


WdRiscv::ExceptionCause
PerfApi::translateInstrAddr(unsigned hartIx, uint64_t va, uint64_t& pa,
                            std::vector<std::vector<WalkEntry>>& walks)
{
  auto hart = checkHart("translate-instr-addr", hartIx);

  pa = va;
  auto pm = hart->privilegeMode();
  if (pm == WdRiscv::PrivilegeMode::Machine or not hart->isRvs())
    return WdRiscv::ExceptionCause::NONE;

  auto virtmem = hart->virtMem();
  auto prevTrace = virtmem.enableTrace(true);
  auto ec = translateInstrAddr(hartIx, va, pa);
  virtmem.enableTrace(prevTrace);
  walks = hart->virtMem().getFetchWalks();
  return ec;
}


WdRiscv::ExceptionCause
PerfApi::translateLoadAddr(unsigned hartIx, uint64_t va, uint64_t& pa,
                           std::vector<std::vector<WalkEntry>>& walks)
{
  auto hart = checkHart("translate-load-addr", hartIx);

  pa = va;
  auto pm = hart->privilegeMode();
  if (pm == WdRiscv::PrivilegeMode::Machine or not hart->isRvs())
    return WdRiscv::ExceptionCause::NONE;

  auto virtmem = hart->virtMem();
  auto prevTrace = virtmem.enableTrace(true);
  auto ec = translateLoadAddr(hartIx, va, pa);
  virtmem.enableTrace(prevTrace);
  walks = hart->virtMem().getDataWalks();
  return ec;
}


WdRiscv::ExceptionCause
PerfApi::translateStoreAddr(unsigned hartIx, uint64_t va, uint64_t& pa,
                            std::vector<std::vector<WalkEntry>>& walks)
{
  auto hart = checkHart("translate-store-addr", hartIx);

  pa = va;
  auto pm = hart->privilegeMode();
  if (pm == WdRiscv::PrivilegeMode::Machine or not hart->isRvs())
    return WdRiscv::ExceptionCause::NONE;

  auto virtmem = hart->virtMem();
  auto prevTrace = virtmem.enableTrace(true);
  auto ec = translateStoreAddr(hartIx, va, pa);
  virtmem.enableTrace(prevTrace);
  walks = hart->virtMem().getDataWalks();
  return ec;
}


bool
PerfApi::drainStore(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_drain_store %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Drain-store", time))
    return false;

  auto hartPtr = checkHart("Drain-store", hartIx);
  auto pacPtr = checkTag("Drain-store", hartIx, tag);

  if (not hartPtr or not pacPtr or not pacPtr->retired())
    {
      assert(0 && "Error: Assertion failed");
      return false;
    }

  auto& hart = *hartPtr;
  auto& packet = *pacPtr;

  auto& di = packet.di_;
  if (not di.isStore() and not di.isVectorStore() and not di.isCbo_zero() and not di.isAmo() and not di.isSc())
    {
      std::cerr << "Error: Hart=" << hartIx << " time=" << time << " tag=" << tag
		<< " Draining a non-store instruction\n";
      return false;
    }

  // AMO/SC drained at retire if more than 1 hart.
  bool skipDrain = (packet.isSc() or packet.isAmo()) and system_.hartCount() > 1;

  if (skipDrain)
    assert(packet.drained());   // AMO/SC must be already retired.
  else
    {
      if (packet.drained())
	{
	  std::cerr << "Error: Hart=" << hartIx << " time=" << time << " tag=" << tag
		    << " Instruction drained more than once\n";
	  assert(0 && "Error: Assertion failed");
	}

      if (packet.dsize_ and not commitMemoryWrite(hart, packet))
	assert(0 && "Error: Assertion failed");

      if (packet.isSc())
	hart.cancelLr(WdRiscv::CancelLrCause::SC);

      packet.drained_ = true;
    }

  // Clear dependency on other packets to expedite release of packet memory.
  for (auto& producer : packet.opProducers_)
    producer.clear();

  auto& packetMap = hartPacketMaps_.at(hartIx);
  packetMap.erase(packet.tag());

  auto& storeMap = hartStoreMaps_.at(hartIx);
  storeMap.erase(packet.tag());

  return true;
}


bool
PerfApi::getLoadData(unsigned hartIx, uint64_t tag, uint64_t va, uint64_t pa1,
		     uint64_t pa2, unsigned size, uint64_t& data, unsigned elemIx,
                     unsigned field)
{
  auto hart = checkHart("Get-load-data", hartIx);
  auto packet = checkTag("Get-load-Data", hartIx, tag);

  bool isLoad = ( packet->di_.isLoad() or packet->di_.isAmo() or
                  packet->di_.isVectorLoad() );
  if (not isLoad)
    assert(0 && "Error: Assertion failed");

  // Scalar instruction should not have trapped. Vector may trap on a later element.
  if (not packet->di_.isVectorLoad())
    assert(not packet->trapped());

  // If AMO destination register is x0, we lose the loaded value: redo the read for AMOs
  // to avoid that case. AMOs should not have a discrepancy between early read and read at
  // retire, so redoing the read is ok.
  bool amoRedo = packet->isAmo() and packet->di_.op0() == 0;

  if (packet->executed() and not amoRedo)
    {
      data = packet->executedDestVal(*hart, size, elemIx, field);
      return true;
    }

  data = 0;
  if (hart->isDeviceAddr(pa1))
    {      
      if (skipIoLoad_)
        return true;  // Load from IO space happens at execute.
      hart->deviceRead(pa1, size, data);

      return true;
    }

  if (uint64_t toHost = 0; hart->getToHostAddress(toHost) && toHost == pa1)
    return true;  // Reading from toHost yields 0.

  auto& storeMap =  hartStoreMaps_.at(hartIx);

  unsigned mask = (1 << size) - 1;  // One bit ber byte of load data.
  unsigned forwarded = 0;            // One bit per forwarded byte.

  unsigned size1 = size;
  if (pa1 != pa2 and pageNum(pa1) != pageNum(pa2))
    size1 = offsetToNextPage(pa1);

  for (auto& kv : storeMap)
    {
      auto stTag = kv.first;
      if (stTag >= tag)
	break;

      auto& stPac = *(kv.second);
      if (not stPac.executed())
	continue;

      uint64_t stAddr = stPac.dataVa();
      unsigned stSize = stPac.dataSize();
      if (not stPac.isVectorStore() and not stPac.isCbo_zero())
        if ((stAddr + stSize < va or va + size < stAddr))
          continue;  // No overlap.

      auto& stMap = stPac.stDataMap_;

      for (unsigned i = 0; i < size; ++i)
	{
	  unsigned byteMask = 1 << i;
	  uint64_t byteAddr = i < size1 ? pa1 + i : pa2 + (i - size1);

          auto iter = stMap.find(byteAddr);
          if (iter == stMap.end())
            continue;

          data &= ~(0xffLL << (i * 8));     // Clear byte location in data
          uint8_t byte = iter->second;
          data |= uint64_t(byte) << (i*8);  // Insert forwarded value instead
          forwarded |= byteMask;
	}
    }

  if (forwarded == mask)
    return true;

  // Non-forward bytes are read from memory.
  for (unsigned i = 0; i < size; ++i)
    if (not (forwarded & (1 << i)))
      {
	uint8_t byte = 0;
        uint64_t pa = i < size1 ? pa1 + i : pa2 + (i - size1);
        if (not hart->peekMemory(pa, byte, true))
          assert(0 && "Error: Assertion failed");
	data |= uint64_t(byte) << (i*8);
      }

  return true;
}


bool
PerfApi::setStoreData(unsigned hartIx, uint64_t tag, uint64_t pa1, uint64_t pa2,
                      unsigned size, uint64_t value)
{
  auto hartPtr = checkHart("Set-store-data", hartIx);
  auto packetPtr = checkTag("Set-store-Data", hartIx, tag);
  if (not packetPtr or not hartPtr)
    {
      assert(0 && "Error: Assertion failed");
      return false;
    }

  auto& packet = *packetPtr;
  if (not (packet.isStore() or packet.isAmo() or packet.isVectorStore() or packet.isCbo_zero()))
    {
      assert(0 && "Error: Assertion failed");
      return false;
    }

  auto& hart = *hartPtr;

  if (pa1 != pa2)
    {
      bool isDev = hart.isDeviceAddr(pa1);
      if (isDev)
        assert(0 && "Error: Assertion failed");

      isDev = hart.isDeviceAddr(pa2);
      if (isDev)
        assert(0 && "Error: Assertion failed");
    }

  packet.dpa_ = pa1;
  packet.dpa2_ = pa2;
  packet.stData_ = value;
  packet.dsize_ = size;

  unsigned size1 = size;
  if (pa1 != pa2 and pageNum(pa1) != pageNum(pa2))
    size1 = offsetToNextPage(pa1);

  for (unsigned i = 0; i < size; ++i, value >>= 8)
    {
      uint64_t pa = i < size1 ? pa1 + i : pa2 + (i - size1);
      uint8_t byte = value;
      packet.stDataMap_[pa] = byte;
    }      

  return true;
}


bool
PerfApi::commitMemoryWrite(Hart64& hart, uint64_t pa1, uint64_t pa2, unsigned size, uint64_t value)
{
  if (hart.isToHostAddr(pa1))
    {
      hart.handleStoreToHost(pa1, value);
      return true;
    }

  auto commit = [] (Hart64& hart, uint64_t pa, unsigned sz, uint64_t val) -> bool {
    switch (sz)
      {
      case 1:  return hart.pokeMemory(pa, uint8_t(val), true);
      case 2:  return hart.pokeMemory(pa, uint16_t(val), true);
      case 4:  return hart.pokeMemory(pa, uint32_t(val), true);
      case 8:  return hart.pokeMemory(pa, val, true);
      default:
        {
          bool ok = true;
          for (unsigned i = 0; i < sz; ++i, val >>= 8)
            {
              uint8_t byte = val;
              ok = hart.pokeMemory(pa + i, byte, true) and ok;
            }
          return ok;
        }
      }
    return true;
  };

  if (pa1 == pa2 or pageNum(pa1) == pageNum(pa2))
    return commit(hart, pa1, size, value);

  unsigned size1 = offsetToNextPage(pa1);
  bool ok = commit(hart, pa1, size1, value);

  value = value >> size1*8;
  ok = commit(hart, pa2, size - size1, value) and ok;
  return ok;
}


bool
PerfApi::commitMemoryWrite(Hart64& hart, const InstrPac& packet)
{
  if (not packet.isVectorStore() and not packet.isCbo_zero())
    return commitMemoryWrite(hart, packet.dpa_, packet.dpa2_, packet.dsize_, packet.stData_);

  auto ok = true;

  for (auto [addr, val] : packet.stDataMap_)
    ok = hart.pokeMemory(addr, val, true) and ok;

  return ok;
}


bool
PerfApi::flush(unsigned hartIx, uint64_t time, uint64_t tag)
{
  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_flush %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Flush", time))
    return false;

  if (not checkHart("Flush", hartIx))
    return false;

  auto& packetMap = hartPacketMaps_.at(hartIx);
  auto& storeMap =  hartStoreMaps_.at(hartIx);
  auto& producers = hartRegProducers_.at(hartIx);

  // Flush tag and all older packets. Flush in reverse order to undo register renaming.
  for (auto iter = packetMap.rbegin(); iter != packetMap.rend(); )
    {
      auto pacPtr = iter->second;
      if (pacPtr->tag_ < tag)
	break;

      if (pacPtr->retired())
        {
          assert(0 && "Error: Assertion failed: Flushing retired instruction");
          return false;
        }

      auto& packet = *pacPtr;
      for (size_t i = 0; i < packet.operandCount_; ++i)
        {
          using OM = WdRiscv::OperandMode;
          using OT = WdRiscv::OperandType;

          auto& op = packet.operands_.at(i);
          if (op.mode == OM::Write or op.mode == OM::ReadWrite)
            {
              unsigned regNum = op.number;
              unsigned gri = globalRegIx(op.type, regNum);
	      if (gri != 0)
		assert(producers.at(gri)->tag_ == packet.tag_);

              auto& iop = packet.opProducers_.at(i);  // ith op producer

              if (op.type != OT::VecReg)
                {
                  auto prev = iop.scalar;
                  if (prev and prev->retired_)
                    prev = nullptr;
                  producers.at(gri) = prev;
                }
              else
                {
                  assert(iop.vec.size() <= maxEffLmul_);
                  for (unsigned n = 0; n < iop.vec.size(); ++n)
                    {
                      auto prev = iop.vec.at(n);
                      if (prev and prev->retired_)
                        prev = nullptr;
                      producers.at(gri+n) = prev;
                    }
                }
            }
        }

      ++iter;
    }

  // delete input tag and all newer instructions
  std::erase_if(packetMap, [&tag](const auto &packet)
  {
    auto const& [_, pacPtr] = packet;
    return pacPtr->tag_ >= tag;
  });

  std::erase_if(storeMap, [&tag](const auto &packet)
  {
    auto const& [_, pacPtr] = packet;
    return pacPtr->tag_ >= tag;
  });

  if (prevFetch_ && prevFetch_->tag_ > tag)
    prevFetch_ = nullptr;

  return true;
}


bool
PerfApi::shouldFlush(unsigned hartIx, uint64_t time, uint64_t tag, bool& flush,
		     uint64_t& addr)
{
  flush = false;
  addr = 0;

  if (commandLog_)
    fprintf(commandLog_, "hart=%" PRIu32 " time=%" PRIu64 " perf_model_should_flush %" PRIu64 "\n",
            hartIx, time, tag);

  if (not checkTime("Flush", time))
    return false;

  if (not checkHart("Flush", hartIx))
    return false;

  auto pacPtr = checkTag("Retire", hartIx, tag);
  if (not pacPtr)
    return false;
  auto& packet = *pacPtr;

  if (packet.shouldFlush())
    {
      flush = true;
      addr = packet.flushVa_;
    }
  else
    {
      // If on the wrong path after a branch, then we wshould flush.
      auto& packetMap = hartPacketMaps_.at(hartIx);
      auto iter = packetMap.find(tag); --iter;
      for ( ; iter != packetMap.end(); --iter)
	{
	  auto& packet = *iter->second;
	  if (packet.mispredicted_)
	    {
	      flush = true;
	      if (packet.di_.isBranch())
		addr = packet.nextIva_;
	      else
		addr = packet.iva_;
	    }
	}
    }

  return true;
}


unsigned
InstrPac::getSourceOperands(std::array<Operand, 3>& ops) const
{
  assert(decoded_);
  if (not decoded_)
    return 0;

  // Return explicit operands. Skip implicit.
  unsigned limit = std::min(di_.operandCount(), operandCount_);
  unsigned count = 0;

  using OM = WdRiscv::OperandMode;

  for (unsigned i = 0; i < limit; ++i)
    {
      const auto& op = operands_.at(i);
      if (op.mode== OM::Read or op.mode == OM::ReadWrite or op.type == OperandType::Imm)
        ops.at(count++) = op;
    }

  return count;
}


unsigned
InstrPac::getDestOperands(std::array<Operand, 2>& ops) const
{
  assert(decoded_);
  if (not decoded_)
    return 0;

  // Return explicit operands. Skip implicit.
  unsigned limit = std::min(di_.operandCount(), operandCount_);
  unsigned count = 0;

  using OM = WdRiscv::OperandMode;

  for (unsigned i = 0; i < limit; ++i)
    {
      const auto& op = operands_.at(i);
      if (op.mode == OM::Write or op.mode == OM::ReadWrite)
        {
          auto& tgt = ops.at(count);
          tgt = op;
          tgt.value = destValues_.at(count).second;
          ++count;
        }
    }

  return count;
}


unsigned
InstrPac::getImplicitDestOperands(std::array<Operand, 4>& ops) const
{
  assert(decoded_);
  if (not decoded_)
    return 0;

  assert(di_.operandCount() <= operandCount_);

  using OM = WdRiscv::OperandMode;

  unsigned explicitDests = 0;

  for (unsigned i = 0; i < di_.operandCount(); ++i)
    {
      const auto& op = operands_.at(i);
      if (op.mode == OM::Write or op.mode == OM::ReadWrite)
        ++explicitDests;
    }

  unsigned count = 0;  // Count of implicit destinations.

  unsigned start = di_.operandCount();

  for (unsigned i = start; i < operandCount_; ++i)
    {
      const auto& op = operands_.at(i);
      if (op.mode == OM::Write or op.mode == OM::ReadWrite)
        {
          auto& tgt = ops.at(count);
          tgt = op;
          tgt.value = destValues_.at(explicitDests + count).second;
          ++count;
        }
    }

  return count;
}


unsigned
InstrPac::getImplicitSrcOperands(std::array<Operand, 8>& impOps) const
{
  assert(decoded_);
  if (not decoded_)
    return 0;

  assert(di_.operandCount() <= operandCount_);

  using OM = WdRiscv::OperandMode;

  unsigned count = 0;  // Count of implicit sources.

  unsigned start = di_.operandCount();

  for (unsigned i = start; i < operandCount_; ++i)
    {
      const auto& op = operands_.at(i);
      if (op.mode == OM::Read or op.mode == OM::ReadWrite)
        impOps.at(count++) = op;
    }

  return count;
}


unsigned
InstrPac::getChangedCsrs(std::array<Operand, 8>& ops) const
{
  for (unsigned i = 0; i < changedCsrCount_; ++i)
    ops.at(i) = changedCsrs_.at(i);
  return changedCsrCount_;
}


uint64_t
InstrPac::branchTargetFromDecode() const
{
  if (!isBranch()) return 0;

  using WdRiscv::InstId;
  switch (di_.instEntry()->instId())
    {
    case InstId::jal:
    case InstId::c_jal:
    case InstId::c_j:
      return instrVa() + di_.op1As<int64_t>();

    case InstId::beq:
    case InstId::bne:
    case InstId::blt:
    case InstId::bge:
    case InstId::bltu:
    case InstId::bgeu:
    case InstId::c_beqz:
    case InstId::c_bnez:
      return instrVa() + di_.op2As<int64_t>();

    default:
      return 0;
    }
}


uint64_t
InstrPac::executedDestVal(const Hart64& hart, unsigned size, unsigned elemIx, unsigned field) const
{
  assert(executed());

  const OpVal& destVal = destValues_.at(0).second;

  if (operands_.at(0).type != WdRiscv::OperandType::VecReg)
    {
      assert(size == dataSize());
      return destVal.scalar;
    }

  const std::vector<uint8_t>& vec = destVal.vec;   // Vector register value.

  const auto& info = hart.getLastVectorMemory();
  unsigned elemSize = info.elemSize_;

  unsigned offset = elemSize * elemIx;
  if (info.isSegmented_)
    {
      if (info.fields_ > 0)
        {
          assert(field < info.fields_);
          unsigned bytesPerReg = hart.vecRegs().bytesPerRegister();
          assert(info.group_ > 0);
          offset += field*bytesPerReg*info.group_;
        }
      else
        assert(field == 0);
    }

  assert(offset + size <= vec.size());

  uint64_t val = 0;
  for (unsigned i = 0; i < size; ++i)
    {
      uint64_t byte = vec.at(offset + i);
      val |= byte << i*8;
    }
  return val;
}


size_t InstrPac::getPacketSize() const
{
  size_t totalSize = 0;

  totalSize += sizeof(*this); // Get the size of the object itself

  // stDataMap_: unordered_map<uint64_t, uint8_t>
  totalSize += stDataMap_.size() * (sizeof(uint64_t) + sizeof(uint8_t));  // key-value pairs

  // vecAddrs_: vector<VaPaSkip> where VaPaSkip = tuple<uint64_t, uint64_t, bool>
  totalSize += vecAddrs_.size() * (sizeof(uint64_t) + sizeof(uint64_t) + sizeof(bool));  // tuple elements

  // fetchWalks_: vector<vector<WalkEntry>>
  for (const auto& walk : fetchWalks_) {
      totalSize += sizeof(std::vector<WdRiscv::VirtMem::WalkEntry>);   // inner vector overhead
      totalSize += walk.size() * sizeof(WdRiscv::VirtMem::WalkEntry);  // WalkEntry objects
  }

  // dataWalks_: vector<vector<WalkEntry>>
  for (const auto& walk : dataWalks_) {
      totalSize += sizeof(std::vector<WdRiscv::VirtMem::WalkEntry>);   // inner vector overhead
      totalSize += walk.size() * sizeof(WdRiscv::VirtMem::WalkEntry);  // WalkEntry objects
  }

  return totalSize;
}


bool
PerfApi::saveHartValues(Hart64& hart, const InstrPac& packet,
                        std::array<OpVal, 8>& prevVal)
{
  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  bool ok = true;

  for (unsigned i = 0; i < packet.operandCount_; ++i)
    {
      const auto& op = packet.operands_.at(i);
      auto mode = op.mode;
      if (mode == OM::None)
	continue;

      auto type = op.type;
      uint32_t number = op.number;

      switch (type)
	{
	case OT::IntReg:
	  if (not hart.peekIntReg(number, prevVal.at(i).scalar))
	    assert(0 && "Error: Assertion failed");
	  break;

	case OT::FpReg:
	  ok = hart.peekFpReg(number, prevVal.at(i).scalar) and ok;
	  break;

	case OT::CsReg:
	  ok = hart.peekCsr(CSRN(number), prevVal.at(i).scalar) and ok;
	  break;

	case OT::VecReg:
          ok = peekVecRegGroup(hart, number, op.lmul, prevVal.at(i)) and ok;
	  break;

	case OT::Imm:
	  break;

	default:
	  assert(0 && "Error: Assertion failed");
	  break;
	}
    }

  return ok;
}


void
PerfApi::saveImsicTopei(Hart64& hart, CSRN csrn, unsigned& id, unsigned& guest)
{
  id = 0;
  guest = 0;

  auto imsic = hart.imsic();
  if (not imsic)
    return;

  if (csrn == CSRN::MTOPEI)
    {
      id = imsic->machineTopId();
    }
  else if (csrn == CSRN::STOPEI)
    {
      id = imsic->supervisorTopId();
    }
  else if (csrn == CSRN::VSTOPEI)
    {
      uint64_t hs = 0;
      if (hart.peekCsr(CSRN::HSTATUS, hs))
	{
	  WdRiscv::HstatusFields<uint64_t> hsf(hs);
	  unsigned gg = hsf.bits_.VGEIN;
	  if (gg > 0 and gg < imsic->guestCount())
	    {
	      guest = gg;
	      imsic->guestTopId(gg);
	    }
	}
    }
}



void
PerfApi::restoreImsicTopei(Hart64& hart, CSRN csrn, unsigned id, unsigned guest)
{
  auto imsic = hart.imsic();
  if (not imsic)
    return;

  if (id == 0)
    return;

  if (csrn == CSRN::MTOPEI)
    {
      imsic->setMachinePending(id, true);
    }
  else if (csrn == CSRN::STOPEI)
    {
      imsic->setSupervisorPending(id, true);
    }
  else if (csrn == CSRN::VSTOPEI)
    {
      if (guest > 0 and guest < imsic->guestCount())
	imsic->setGuestPending(guest, id, true);
    }
}


void
PerfApi::restoreHartValues(Hart64& hart, const InstrPac& packet,
			   const std::array<OpVal, 8>& prevVal)
{
  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  for (unsigned i = 0; i < packet.operandCount_; ++i)
    {
      const auto& op = packet.operands_.at(i);
      auto mode = op.mode;
      auto type = op.type;
      uint32_t number = op.number;
      uint64_t prev = prevVal.at(i).scalar;
      const std::vector<uint8_t>& vec = prevVal.at(i).vec;
      if (mode == OM::None)
	continue;

      switch (type)
	{
	case OT::IntReg:
	  if (not hart.pokeIntReg(number, prev))
	    assert(0 && "Error: Assertion failed");
	  break;

	case OT::FpReg:
	  if (not hart.pokeFpReg(number, prev))
	    assert(0 && "Error: Assertion failed");
	  break;

	case OT::CsReg:
          {
            auto csrn = CSRN(number);
            hart.pokeCsr(csrn, prev);  // May fail because of privilege. It's ok: handled at caller.
          }
	  break;

	case OT::VecReg:
          {
            size_t bytesPerReg = hart.vecRegs().bytesPerRegister();
            size_t count = vec.size() / bytesPerReg;
            assert(count * bytesPerReg == vec.size());
            for (unsigned i = 0; i < count; ++i)
              {
                const auto* pokeData = &vec.at(i*bytesPerReg);
                if (not hart.pokeVecRegLsb(number + i, std::span(pokeData, bytesPerReg)))
                  assert(packet.trap_);
              }
          }
	  break;

	default:
	  assert(0 && "Error: Assertion failed");
	  break;
	}
    }
}


bool
PerfApi::setHartValues(Hart64& hart, const InstrPac& packet)
{
  bool ok = true;

  for (unsigned i = 0; i < packet.operandCount_; ++i)
    {
      const auto& op = packet.operands_.at(i);
      if (op.mode == WdRiscv::OperandMode::None)
 	continue;

      auto type = op.type;
      uint32_t regNum = op.number;
      ok = pokeRegister(hart, type, regNum, op.value) and ok;
    }

  return ok;
}


bool
PerfApi::peekRegister(Hart64& hart, WdRiscv::OperandType type, unsigned regNum,
                      OpVal& value)
{
  using OT = WdRiscv::OperandType;
  switch(type)
    {
    case OT::IntReg: return hart.peekIntReg(regNum, value.scalar);
    case OT::FpReg:  return hart.peekFpReg(regNum, value.scalar);
    case OT::CsReg:  return hart.peekCsr(WdRiscv::CsrNumber(regNum), value.scalar);
    case OT::VecReg: return hart.peekVecRegLsb(regNum, value.vec);
    case OT::Imm:
    case OT::None:   assert(0 && "Error: Assertion failed"); return false;
    }
  return false;
}


bool
PerfApi::pokeRegister(Hart64& hart, WdRiscv::OperandType type, unsigned regNum,
		      const OpVal& value)
{
  using OT = WdRiscv::OperandType;

  uint64_t scalar = value.scalar;
  const std::vector<uint8_t>& vecVal = value.vec;

  switch (type)
    {
    case OT::IntReg:
      if (hart.pokeIntReg(regNum, scalar))
        return true;
      assert(0 && "Error: Assertion failed");
      return false;

    case OT::FpReg:
      return hart.pokeFpReg(regNum, scalar);

    case OT::CsReg:
      return hart.pokeCsr(CSRN(regNum), scalar);
      
    case OT::VecReg:
      {
        bool ok = true;

        size_t bytesPerReg = hart.vecRegs().bytesPerRegister();
        size_t count = vecVal.size() / bytesPerReg;
        assert(count * bytesPerReg == vecVal.size());

        for (unsigned i = 0; i < count; ++i)
          {
            const auto* pokeData = &vecVal.at(i*bytesPerReg);
            ok = hart.pokeVecRegLsb(regNum + i, std::span(pokeData, bytesPerReg)) and ok;
          }

        return ok;
      }

    case OT::Imm:
    default:
      break;
    }

  assert(0 && "Error: Assertion failed");
  return false;
}


bool
PerfApi::peekVecRegGroup(Hart64& hart, unsigned regNum, unsigned lmul, OpVal& value)
{
  std::vector<uint8_t>& data = value.vec;
  std::vector<uint8_t> vecVal;  // Single vector value.

  bool ok = true;

  for (unsigned n = 0; n < lmul; ++n)
    {
      ok = hart.peekVecRegLsb(regNum + n, vecVal) and ok;

      // Append single vector value to data.
      data.insert(data.end(), vecVal.begin(), vecVal.end());
    }

  return ok;
}


void
PerfApi::updatePacketDataAddress(Hart64& hart, InstrPac& packet)
{
  const auto& di = packet.decodedInst();

  auto hartIx = hart.sysHartIndex();

  if (di.isLoad())
    {
      hart.lastLdStAddress(packet.dva_, packet.dpa_, packet.dpa2_);
      packet.dsize_ = di.loadSize();
      uint64_t dpa = packet.dpa_;
      packet.deviceAccess_ = ( hart.isAclintMtimeAddr(dpa) or hart.isImsicAddr(dpa) or
#if PCI
                               hart.isPciAddr(dpa) or
#endif
                               hart.isHtifAddr(dpa) );
    }
  else if (di.isVectorLoad())
    {
      const auto& info = hart.getLastVectorMemory();
      for (const auto& elem : info.elems_)
        {
          InstrPac::VaPaSkip skip{elem.va_, elem.pa_, elem.skip_};
          packet.vecAddrs_.emplace_back(skip);
        }
      packet.dsize_ = info.elemSize_;
    }
  else if (di.isStore() or di.isAmo() or di.isVectorStore() or di.isCbo_zero() or di.isCmo())
    {
      uint64_t sva = 0, spa1 = 0, spa2 = 0, sval = 0;
      unsigned ssize = hart.lastStore(sva, spa1, spa2, sval);
      if (ssize == 0 and (di.isStore() or di.isAmo()) and not di.isSc())
	{    // sc or vec-store may have 0 size
	  std::cerr << "Error: Hart=" << hartIx << " tag=" << packet.tag_
		    << " store/AMO with zero size\n";
	  assert(0 && "Error: Assertion failed");
	}

      packet.dva_ = sva;
      packet.dpa_ = spa1;  // FIX TODO : handle page crossing
      packet.dpa2_ = spa2;
      packet.dsize_ = ssize;
      assert(ssize == packet.dsize_);
    
      auto& storeMap =  hartStoreMaps_.at(hartIx);
      auto tag = packet.tag();

      bool skipStoreMap = (packet.isSc() or packet.isAmo()) and system_.hartCount() > 1;

      if (di.isCbo_zero())
        {
          storeMap[tag] = getInstructionPacket(hartIx, tag);
        }
      else if (di.isVectorStore())
        {
          const auto& info = hart.getLastVectorMemory();
          for (const auto& elem : info.elems_)
            {
              InstrPac::VaPaSkip skip{elem.va_, elem.pa_, elem.skip_};
              packet.vecAddrs_.emplace_back(skip);
            }

          packet.dsize_ = info.elemSize_;

          storeMap[tag] = getInstructionPacket(hartIx, tag);
          // FIX What to do about device access? Do we allow mixed device/non-device
          // access?
        }
      else if ((di.isStore() or di.isAmo()) and not skipStoreMap)
	{
	  storeMap[tag] = getInstructionPacket(hartIx, tag);
          uint64_t dpa = packet.dpa_;
	  packet.deviceAccess_ = ( hart.isAclintMtimeAddr(dpa) or hart.isImsicAddr(dpa) or
#if PCI
                                   hart.isPciAddr(dpa) or
#endif
                                   hart.isHtifAddr(dpa) );
	}
    }
}


void
PerfApi::recordExecutionResults(Hart64& hart, InstrPac& packet)
{
  if (not packet.trap_)
    updatePacketDataAddress(hart, packet);

  if (hart.hasTargetProgramFinished())
    packet.nextIva_ = haltPc;

  if (packet.di_.isBranch())
    packet.taken_ = hart.lastBranchTaken();

  // Record the values of the destination register.
  unsigned destIx = 0;
  for (unsigned i = 0; i < packet.operandCount_; ++i)
    {
      using OM = WdRiscv::OperandMode;
      using OT = WdRiscv::OperandType;

      auto& op = packet.operands_.at(i);

      if (op.mode == OM::Write or op.mode == OM::ReadWrite)
	{
	  unsigned regNum = op.number;
	  unsigned gri = globalRegIx(op.type, regNum);
	  OpVal destVal;
          if (op.type != OT::VecReg)
            {
              if (not peekRegister(hart, op.type, regNum, destVal))
                assert(packet.trap_);
            }
          else
            {
              if (not peekVecRegGroup(hart, regNum, op.lmul, destVal))
                assert(packet.trap_);
            }
	  packet.destValues_.at(destIx) = InstrPac::DestValue(gri, destVal);
	  destIx++;
	}
    }

  // Memory should not have changed.
}


void
PerfApi::getVectorOperandsLmul(Hart64& hart, InstrPac& packet)
{
  auto di = packet.decodedInst();
  if (not di.isVector())
    return;

  using CN = WdRiscv::CsrNumber;
  using OT = WdRiscv::OperandType;

  // 1. Set vtype value if it is in-flight.
  auto hartIx = hart.sysHartIndex();
  auto& producers = hartRegProducers_.at(hartIx);
  auto vtypeGri = globalRegIx(OT::CsReg, unsigned(CN::VTYPE));
  auto producer = producers.at(vtypeGri);  // Producer of vtype

  uint64_t prevVal = 0;
  if (producer)
    {
      if (not hart.peekCsr(CN::VTYPE, prevVal))
        assert(0 && "Error: Assertion failed");

      OpVal vtypeVal;
      getDestValue(*producer, vtypeGri, vtypeVal);
      hart.pokeCsr(CN::VTYPE, vtypeVal.scalar);
    }

  // 2. Determine the operands LMUL
  getVecOpsLmul(hart, packet);

  // 3. Restore vtype if it was set.
  if (producer)
    hart.pokeCsr(CN::VTYPE, prevVal);
}


void
PerfApi::getVecOpsLmul(Hart64& hart, InstrPac& packet)
{
  const auto& vecRegs = hart.vecRegs();

  auto groupX8 = vecRegs.groupMultiplierX8();
  unsigned effLmul = groupX8 <= 8 ? 1 : groupX8 / 8;

  auto wideX8 = 2*groupX8;
  unsigned effWideLmul = wideX8 <= 8 ? 1 : wideX8 / 8;

  for (unsigned i = 0; i < 3; ++i)
    packet.operands_.at(i).lmul = effLmul;

  using InstId = WdRiscv::InstId;

  auto di = packet.decodedInst();

  if (di.isVectorLoad() or di.isVectorStore())
    {
      unsigned fields = di.vecFieldCount();

      if (di.isVectorLoadIndexed() or di.isVectorStoreIndexed())
        {
          auto ig8 = groupX8 * hart.vecLdStIndexElemSize(di) / vecRegs.elemWidthInBytes();
          auto dg8 = groupX8;

          unsigned dmul = dg8 <= 8 ? 1 : dg8 / 8;   // Data reg effective lmul
          unsigned imul = ig8 <= 8 ? 1 : ig8 / 8;   // Index reg effective lmul

          if (fields > 0)
            dmul *= fields;

          packet.operands_.at(0).lmul = dmul;
          packet.operands_.at(2).lmul = imul;
        }
      else
        {
          auto id = di.instId();
          if ( (id >= InstId::vlre8_v and id <= InstId::vlre64_v) or
               (id >= InstId::vs1r_v and id <= InstId::vs8r_v) )
            packet.operands_.at(0).lmul = fields;
          else
            {
              auto dg8 = groupX8 * hart.vecLdStElemSize(di) / vecRegs.elemWidthInBytes();
              unsigned dmul = dg8 <= 8 ? 1 : dg8 / 8;   // Data reg effective lmul
              if (fields)
                dmul *= fields;             // Segment load/store
              packet.operands_.at(0).lmul = dmul;
            }
        }

      return;
    }

  switch(di.instId())
    {
    case InstId::vmv1r_v:
      packet.operands_.at(0).lmul = packet.operands_.at(1).lmul = 1;
      break;
    case InstId::vmv2r_v:
      packet.operands_.at(0).lmul = packet.operands_.at(1).lmul = 2;
      break;
    case InstId::vmv4r_v:
      packet.operands_.at(0).lmul = packet.operands_.at(1).lmul = 4;
      break;
    case InstId::vmv8r_v:
      packet.operands_.at(0).lmul = packet.operands_.at(1).lmul = 8;
      break;

    case InstId::vmv_x_s:
    case InstId::vmv_s_x:
      packet.operands_.at(0).lmul = packet.operands_.at(1).lmul = 1;
      break;

    case InstId::vwaddu_vv:
    case InstId::vwaddu_vx:
    case InstId::vwsubu_vv:
    case InstId::vwsubu_vx:
    case InstId::vwadd_vv:
    case InstId::vwadd_vx:
    case InstId::vwsub_vv:
    case InstId::vwsub_vx:
    case InstId::vwmulu_vv:
    case InstId::vwmulu_vx:
    case InstId::vwmul_vv:
    case InstId::vwmul_vx:
    case InstId::vwmulsu_vv:
    case InstId::vwmulsu_vx:
    case InstId::vwmaccu_vv:
    case InstId::vwmaccu_vx:
    case InstId::vwmacc_vv:
    case InstId::vwmacc_vx:
    case InstId::vwmaccsu_vv:
    case InstId::vwmaccsu_vx:
    case InstId::vwmaccus_vx:
    case InstId::vwsll_vv:
    case InstId::vwsll_vx:
    case InstId::vwsll_vi:
    case InstId::vfwcvt_xu_f_v:
    case InstId::vfwcvt_x_f_v:
    case InstId::vfwcvt_rtz_xu_f_v:
    case InstId::vfwcvt_rtz_x_f_v:
    case InstId::vfwcvt_f_xu_v:
    case InstId::vfwcvt_f_x_v:
    case InstId::vfwcvt_f_f_v:
    case InstId::vfwcvtbf16_f_f_v:
    case InstId::vfwmaccbf16_vv:
    case InstId::vfwmaccbf16_vf:
      packet.operands_.at(0).lmul = effWideLmul;
      break;

    case InstId::vwaddu_wv:
    case InstId::vwaddu_wx:
    case InstId::vwsubu_wv:
    case InstId::vwsubu_wx:
    case InstId::vwadd_wv:
    case InstId::vwadd_wx:
    case InstId::vwsub_wv:
    case InstId::vwsub_wx:
    case InstId::vfwadd_wv:
    case InstId::vfwadd_wf:
    case InstId::vfwsub_wv:
    case InstId::vfwsub_wf:
      packet.operands_.at(0).lmul = effWideLmul;
      packet.operands_.at(1).lmul = effWideLmul;
      break;

    case InstId::vnsrl_wv:
    case InstId::vnsrl_wx:
    case InstId::vnsrl_wi:
    case InstId::vnsra_wv:
    case InstId::vnsra_wx:
    case InstId::vnsra_wi:
    case InstId::vnclipu_wv:
    case InstId::vnclipu_wx:
    case InstId::vnclipu_wi:
    case InstId::vnclip_wv:
    case InstId::vnclip_wx:
    case InstId::vnclip_wi:

    case InstId::vfncvt_xu_f_w:
    case InstId::vfncvt_x_f_w:
    case InstId::vfncvt_rtz_xu_f_w:
    case InstId::vfncvt_rtz_x_f_w :
    case InstId::vfncvt_f_xu_w:
    case InstId::vfncvt_f_x_w :
    case InstId::vfncvt_f_f_w:
    case InstId::vfncvt_rod_f_f_w:
    case InstId::vfncvtbf16_f_f_w:
      packet.operands_.at(1).lmul = effWideLmul;
      break;

    case InstId::vsext_vf2:
    case InstId::vzext_vf2:
      packet.operands_.at(1).lmul = effLmul < 2 ? 1 : effLmul / 2;
      break;

    case InstId::vsext_vf4:
    case InstId::vzext_vf4:
      packet.operands_.at(1).lmul = effLmul < 4 ? 1 : effLmul / 4;
      break;

    case InstId::vsext_vf8:
    case InstId::vzext_vf8:
      packet.operands_.at(1).lmul = effLmul < 8 ? 1 : effLmul / 8;
      break;

    case InstId::vmseq_vv:
    case InstId::vmseq_vx:
    case InstId::vmseq_vi:
    case InstId::vmsne_vv:
    case InstId::vmsne_vx:
    case InstId::vmsne_vi:
    case InstId::vmsltu_vv:
    case InstId::vmsltu_vx:
    case InstId::vmslt_vv:
    case InstId::vmslt_vx:
    case InstId::vmsleu_vv:
    case InstId::vmsleu_vx:
    case InstId::vmsleu_vi:
    case InstId::vmsle_vv:
    case InstId::vmsle_vx:
    case InstId::vmsle_vi:
    case InstId::vmsgtu_vx:
    case InstId::vmsgtu_vi:
    case InstId::vmsgt_vx:
    case InstId::vmsgt_vi:
      packet.operands_.at(0).lmul = 1;
      break;

    case InstId::vredsum_vs:
    case InstId::vredand_vs:
    case InstId::vredor_vs:
    case InstId::vredxor_vs:
    case InstId::vredminu_vs:
    case InstId::vredmin_vs:
    case InstId::vredmaxu_vs:
    case InstId::vredmax_vs:

    case InstId::vwredsumu_vs:
    case InstId::vwredsum_vs:
    case InstId::vfwredusum_vs:
    case InstId::vfwredosum_vs:
      packet.operands_.at(0).lmul = 1;  // Destination vec operand has lmul of 1.
      packet.operands_.at(2).lmul = 1;  // Third vec operand has lmul of 1.
      break;

    case InstId::vmand_mm:
    case InstId::vmnand_mm:
    case InstId::vmandn_mm:
    case InstId::vmxor_mm:
    case InstId::vmor_mm:
    case InstId::vmnor_mm:
    case InstId::vmorn_mm:
    case InstId::vmxnor_mm:
    case InstId::vcpop_m:
    case InstId::vfirst_m:
    case InstId::vmsbf_m:
    case InstId::vmsif_m:
    case InstId::vmsof_m:
    case InstId::viota_m:
      packet.operands_.at(0).lmul = packet.operands_.at(1).lmul = packet.operands_.at(2).lmul = 1;
      break;

    case InstId::vrgatherei16_vv:
      {
        unsigned op2g8 = (16*groupX8) / vecRegs.elemWidthInBits();
        unsigned op2Lmul = op2g8 <= 8 ? 1 : op2g8 / 8;
        packet.operands_.at(2).lmul = op2Lmul;
      }
      break;

    default:
      break;
    }
}


void
PerfApi::undoDestRegRename(unsigned hartIx, const InstrPac& packet)
{
  auto& producers = hartRegProducers_.at(hartIx);
  for (size_t i = 0; i < packet.operandCount_; ++i)
    {
      using OM = WdRiscv::OperandMode;
      using OT = WdRiscv::OperandType;

      const auto& op = packet.operands_.at(i);

      auto mode = op.mode;
      if (mode == OM::Write or mode == OM::ReadWrite)
	{
	  unsigned regNum = op.number;
          auto type = op.type;
	  unsigned gri = globalRegIx(type, regNum);
          if (type != OT::VecReg)
            {
              auto& producer = producers.at(gri);
              if (producer and producer->tag() == packet.tag())
                producer = nullptr;
            }
          else
            {
              assert(op.lmul <= maxEffLmul_);
              for (unsigned n = 0; n < op.lmul; ++n)
                {
                  auto& producer = producers.at(gri + n);
                  if (producer and producer->tag() == packet.tag())
                    producer = nullptr;
                }
            }
	}
    }
}


bool
PerfApi::collectOperandValues(Hart64& hart, InstrPac& packet)
{
  bool peekOk = true;

  auto hartIx = hart.sysHartIndex();
  auto tag = packet.tag();

  using OT = WdRiscv::OperandType;

  unsigned vecRegSize = hart.vecRegSize();

  for (unsigned i = 0; i < packet.operandCount_; ++i)
    {
      auto& op = packet.operands_.at(i);

      auto type = op.type;
      if (type == OT::Imm)
        continue;  // Value obtained at decode.

      assert(op.mode != WdRiscv::OperandMode::None);

      unsigned regNum = op.number;
      unsigned gri = globalRegIx(type, regNum);
      OpVal opVal;

      auto& iop = packet.opProducers_.at(i);   // Ith operand producer

      if (type != OT::VecReg)
        {
          auto& producer = iop.scalar;
          if (producer)
            {
              if (not producer->executed())
                {
                  std::cerr << "Error: PerfApi::execute: Hart-ix=" << hartIx << "tag=" << tag
                            << " depends on tag=" << producer->tag_ << " which is not yet executed.\n";
                  assert(0 && "Error: Assertion failed");
                  return false;
                }
              getDestValue(*producer, gri, opVal);
            }
          else
            peekOk = peekRegister(hart, type, regNum, opVal) and peekOk;
        }
      else
        {
          assert(iop.vec.size() <= maxEffLmul_);
          for (unsigned n = 0; n < iop.vec.size(); ++n)
            {
              OpVal val;  // Single register value
              auto& producer = iop.vec.at(n);
              if (producer)
                {
                  if (not producer->executed())
                    {
                      std::cerr << "Error: PerfApi::execute: Hart-ix=" << hartIx << "tag=" << tag
                                << " depends on tag=" << producer->tag_ << " which is not yet executed.\n";
                      assert(0 && "Error: Assertion failed");
                      return false;
                    }
                  peekOk = getVecDestValue(*producer, gri + n, vecRegSize, val) and peekOk;
                }
              else
                peekOk = peekRegister(hart, type, regNum+n, val) and peekOk;

              // Append val to opVal.
              opVal.vec.insert(opVal.vec.end(), val.vec.begin(), val.vec.end());
            }
        }

      op.value = opVal;
    }

  return peekOk;
}


void
PerfApi::determineExplicitOperands(InstrPac& packet)
{
  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  const auto& di = packet.decodedInst();

  packet.operandCount_ = 0;

  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      auto mode = di.effectiveIthOperandMode(i);
      auto type = di.ithOperandType(i);

      if (mode == OM::None)
        assert(type == OT::Imm);

      auto& op = packet.operands_.at(packet.operandCount_++);
      op.type = type;
      op.mode = mode;
      op.number = di.ithOperand(i);     // Irrelevant for immediate ops.
      if (op.type == OT::Imm)
        op.value.scalar = di.ithOperand(i);
    }
}


void
PerfApi::determineImplicitOperands(InstrPac& packet)
{
  using OM = WdRiscv::OperandMode;
  using OT = WdRiscv::OperandType;

  const auto& di = packet.decodedInst();

  // Determine implicit operands. Vtype is an implicit source for all vector instructions.
  // It is an implicit destination for the vsetvl/vsetvli/vsetivli. Same for VL.
  if (di.isVector())
    {
      using WdRiscv::InstId;
      auto id = di.instId();
      bool isVset = (id == InstId::vsetvl or id == InstId::vsetvli or id == InstId::vsetivli);

      if (di.isMasked() and not isVset)
        {
          auto& v0Op = packet.operands_.at(packet.operandCount_++);
          v0Op.type = OT::VecReg;
          v0Op.mode = OM::Read;
          v0Op.number = 0;
          v0Op.lmul = 1;
        }

      auto& vtOp = packet.operands_.at(packet.operandCount_++);
      vtOp.type = OT::CsReg;
      vtOp.mode = isVset? OM::Write : OM::Read;
      vtOp.number = unsigned(CSRN::VTYPE);
      
      auto& vlOp = packet.operands_.at(packet.operandCount_++);
      vlOp.type = OT::CsReg;
      if (di.isVectorLoadFaultFirst())
        vlOp.mode = OM::ReadWrite;
      else
        vlOp.mode = isVset ? OM::Write : OM::Read;
      vlOp.number = unsigned(CSRN::VL);

      auto& vsOp = packet.operands_.at(packet.operandCount_++);
      vsOp.type = OT::CsReg;
      vsOp.mode = OM::ReadWrite;
      vsOp.number = unsigned(CSRN::VSTART);

      // We currently don't keep track of vector instructions that use FCSR. Assume all do.
      auto& fcsrOp = packet.operands_.at(packet.operandCount_++);
      fcsrOp.type = OT::CsReg;
      fcsrOp.mode = OM::ReadWrite;
      fcsrOp.number = unsigned(CSRN::FCSR);
    }
  else if (di.isFp() and (di.modifiesFflags() or di.hasDynamicRoundingMode()))
    {
      auto& op = packet.operands_.at(packet.operandCount_++);
      op.type = OT::CsReg;
      op.mode = OM::Read;
      op.number = unsigned(CSRN::FCSR);
      if (di.modifiesFflags())
        op.mode = di.hasDynamicRoundingMode() ? OM::ReadWrite : OM::Write;
    }      
}


void
PerfApi::flattenOperand(const Operand& op, std::vector<Operand>& flat) const
{
  flat.clear();

  auto hart0 = system_.ithHart(0);
  unsigned vecSize = hart0->vecRegs().bytesPerRegister();

  if (op.type != OperandType::VecReg or op.value.vec.size() <= vecSize)
    {
      flat.push_back(op);
      return;
    }

  unsigned remains = op.value.vec.size();  // Count of remaining bytes to flatten.
  unsigned vecIx = op.number;
  unsigned offset = 0;  // Offset into data of original operand.

  while (remains > 0)
    {
      Operand flatOp;
      flatOp.type = op.type;
      flatOp.mode = op.mode;
      flatOp.number = vecIx++;
      flatOp.lmul = 1;

      unsigned chunk = std::min(remains, vecSize);
      remains -= chunk;

      auto& dest = flatOp.value.vec;
      const auto& src = op.value.vec;
      dest.insert(dest.end(),  src.begin() + offset,  src.begin() + offset + chunk);
      offset += chunk;

      flat.push_back(flatOp);
    }
}
