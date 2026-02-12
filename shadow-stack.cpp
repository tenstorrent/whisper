#include "Hart.hpp"
#include "DecodedInst.hpp"

using namespace WdRiscv;

template <typename URV>
ExceptionCause
Hart<URV>::determineSsException(uint64_t& addr, uint64_t& gaddr, uint64_t size, Pma::Attrib attrib)
{
  using PM = PrivilegeMode;
  using EC = ExceptionCause;
  ldStAddr_ = addr;
  ldStPhysAddr1_ = ldStAddr_;
  ldStPhysAddr2_ = ldStAddr_;
  ldStSize_ = size;
  bool load = attrib == Pma::Attrib::Read;

#ifndef FAST_SLOPPY

  bool misal = (addr & (size - 1)) != 0;
  if (misal and misalHasPriority_)
    {
      addr = applyPointerMask(addr, false);
      return misalAtomicCauseAccessFault_? EC::STORE_ACC_FAULT : EC::STORE_ADDR_MISAL;
    }

  if (hasActiveTrigger())
    {
      if (ldStAddrTriggerHit(addr, size, TriggerTiming::Before, load))
	{
	  dataAddrTrig_ = not triggerTripped_;  // Mark data unless instruction already tripped.
	  triggerTripped_ = true;
	}
    }

  uint64_t va = addr;
  auto [pm, virt] = effLdStMode();
  ldStFaultAddr_ = addr = gaddr = va = applyPointerMask(va, true, false);

  // If effective priv mode is M and we perform an amoswap, we must take
  // an exception.
  if ((pm == PM::Machine) and (attrib == Pma::Attrib::AmoSwap))
    return EC::STORE_ACC_FAULT;

  setMemProtAccIsFetch(false);

  if (isRvs())
    {
      if (pm != PM::Machine)
        {
	  auto cause = virtMem_.translateForSs(va, pm, virt, load, gaddr, addr);
          if (cause != EC::NONE)
            {
              ldStFaultAddr_ = addr;
              return cause;
            }
        }
    }

  // Physical memory protection. Assuming grain size is >= 8.
  if (pmpEnabled_)
    {
      auto effPm = effectivePrivilege();
      const Pmp& pmp = pmpManager_.accessPmp(addr);
      if (not pmp.isRead(effPm)  or not pmp.isWrite(effPm))
	return EC::STORE_ACC_FAULT;
    }

  steeInsec1_ = false;
  if (steeEnabled_)
    {
      if (not stee_.isValidAddress(addr))
	return EC::STORE_ACC_FAULT;
      steeInsec1_ = stee_.isInsecureAccess(addr);
      addr = stee_.clearSecureBits(addr);
    }

  Pma pma = accessPma(addr);
  pma = overridePmaWithPbmt(pma, virtMem_.lastEffectivePbmt());
  if (not pma.hasAttrib(attrib) or not pma.hasAttrib(Pma::Idempotent))
    return EC::STORE_ACC_FAULT;

  if (misal)
    return misalAtomicCauseAccessFault_? EC::STORE_ACC_FAULT : EC::STORE_ADDR_MISAL;

#endif

  return EC::NONE;
}


template <typename URV>
void
Hart<URV>::execSspush(const DecodedInst* di)
{
  uint64_t addr = ssp_ - sizeof(URV);
  uint64_t gaddr = addr;

  // We assume address must be XLEN-aligned.
  assert((addr & (sizeof(URV) - 1)) == 0);

  auto cause = determineSsException(addr, gaddr, sizeof(URV), Pma::Attrib::Write);
  if (not memory_.checkWrite(addr, sizeof(URV)))
    cause = ExceptionCause::STORE_ACC_FAULT;

  if (cause != ExceptionCause::NONE)
    {
      if (triggerTripped_)
        return;
      initiateStoreException(di, cause, ldStFaultAddr_, gaddr);
      return;
    }

  // write value
  URV data = intRegs_.read(di->op2());
#ifdef FAST_SLOPPY
  fastStore<URV>(di, ldStAddr_, data);
#else
  writeForStore<URV>(ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data);
#endif
  ssp_ -= sizeof(URV);
}


template <typename URV>
void
Hart<URV>::execSspopchk(const DecodedInst* di)
{
  uint64_t addr = ssp_;
  uint64_t gaddr = addr;

  // We assume address must be XLEN-aligned.
  assert((addr & (sizeof(URV) - 1)) == 0);

  auto cause = determineSsException(addr, gaddr, sizeof(URV), Pma::Attrib::Read);
  if (not memory_.checkRead(addr, sizeof(URV)))
    cause = ExceptionCause::STORE_ACC_FAULT;

  if (cause != ExceptionCause::NONE)
    {
      if (triggerTripped_)
        return;
      initiateStoreException(di, cause, ldStFaultAddr_, gaddr);
      return;
    }

  uint64_t data = 0;
#ifdef FAST_SLOPPY
  fastLoad<URV>(di, ldStAddr_, data);
#else
  readForLoad<URV>(di, ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data, 0, 0);
#endif

  URV expected = intRegs_.read(di->op1());
  if (URV(data) != expected)
    {
      initiateException(ExceptionCause::SOFTWARE_CHECK, currPc_, 3 /* shadow stack */);
      return;
    }
  ssp_ += sizeof(URV);
}


template <typename URV>
void
Hart<URV>::execSsrdp(const DecodedInst* di)
{
  intRegs_.write(di->op0(), ssp_);
}


template <typename URV>
void
Hart<URV>::execSsamoswap_w(const DecodedInst* di)
{
  if (not isRva() or not isRvZicfiss())
    {
      illegalInst(di);
      return;
    }
  if  (not isShadowStackEnabled(privMode_, virtMode_))
    {
      if (virtMode_)
        virtualInst(di);
      else
        illegalInst(di);
      return;
    }

  std::unique_lock lock(memory_.amoMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  uint64_t addr = intRegs_.read(rs1);
  uint64_t gaddr = addr;

  auto cause = determineSsException(addr, gaddr, sizeof(uint32_t), Pma::Attrib::AmoSwap);
  if (not memory_.checkRead(addr, sizeof(uint32_t)) or not memory_.checkWrite(addr, sizeof(uint32_t)))
    cause = ExceptionCause::STORE_ACC_FAULT;

  if (cause != ExceptionCause::NONE)
    {
      if (triggerTripped_)
        return;
      initiateStoreException(di, cause, ldStFaultAddr_, gaddr);
      return;
    }

  uint64_t data = 0;
#ifdef FAST_SLOPPY
  fastLoad<uint32_t>(di, ldStAddr_, data);
  // RV64 SSAMOSWAP.W returns sign-extended 32-bit loaded value in rd.
  intRegs_.write(rd, SRV(int32_t(data)));

  URV rs2Val = intRegs_.read(rs2);
  fastStore<uint32_t>(di, ldStAddr_, uint32_t(rs2Val));
#else
  readForLoad<uint32_t>(di, ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data, 0, 0);
  // RV64 SSAMOSWAP.W returns sign-extended 32-bit loaded value in rd.
  intRegs_.write(rd, SRV(int32_t(data)));

  URV rs2Val = intRegs_.read(rs2);
  writeForStore<uint32_t>(ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, uint32_t(rs2Val));
#endif
}


template <typename URV>
void
Hart<URV>::execSsamoswap_d(const DecodedInst* di)
{
  if (not isRva() or not isRvZicfiss() or not isRv64())
    {
      illegalInst(di);
      return;
    }
  if (not isShadowStackEnabled(privMode_, virtMode_))
    {
      if (virtMode_)
        virtualInst(di);
      else
        illegalInst(di);
      return;
    }

  std::unique_lock lock(memory_.amoMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  uint64_t addr = intRegs_.read(rs1);
  uint64_t gaddr = addr;

  auto cause = determineSsException(addr, gaddr, sizeof(uint64_t), Pma::Attrib::AmoSwap);
  if (not memory_.checkRead(addr, sizeof(uint64_t)) or not memory_.checkWrite(addr, sizeof(uint64_t)))
    cause = ExceptionCause::STORE_ACC_FAULT;

  if (cause != ExceptionCause::NONE)
    {
      if (triggerTripped_)
        return;
      initiateStoreException(di, cause, ldStFaultAddr_, gaddr);
      return;
    }


  uint64_t data = 0;
#ifdef FAST_SLOPPY
  fastLoad<uint64_t>(di, ldStAddr_, data);
  intRegs_.write(rd, data);

  URV rs2Val = intRegs_.read(rs2);
  fastStore<uint64_t>(di, ldStAddr_, rs2Val);
#else
  readForLoad<uint64_t>(di, ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data, 0, 0);
  intRegs_.write(rd, data);

  URV rs2Val = intRegs_.read(rs2);
  writeForStore<uint64_t>(ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, rs2Val);
#endif
}

template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
