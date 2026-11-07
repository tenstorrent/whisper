#include "Hart.hpp"
#include "DecodedInst.hpp"

using namespace WdRiscv;

template <typename URV>
bool
Hart<URV>::ssAccessAllowed(const DecodedInst* di, uint64_t& addr, uint64_t size,
                           Pma::Attrib attrib)
{
  ldStAddr_ = ldStFaultAddr_ = addr;
  ldStPhysAddr1_ = ldStPhysAddr2_ = addr;
  ldStSize_ = size;

  bool isLoad = attrib != Pma::Attrib::Write;
  bool isStore = attrib != Pma::Attrib::Read;

  using EC = ExceptionCause;
  auto cause = EC::NONE;
  uint64_t gaddr = addr;

#ifndef FAST_SLOPPY
  uint64_t va = applyPointerMask(addr, true, false);

  // An address trigger has priority over every exception of the access.
  if (hasActiveTrigger())
    {
      ldStAddrTriggerHit(va, size, TriggerTiming::Before, isLoad, isStore);
      if (breakpOrEnterDebugTripped())
        return false;
    }

  ldStFaultAddr_ = addr = gaddr = va;
  cause = determineSsException(addr, gaddr, size, attrib);
#endif

  if (cause == EC::NONE and ((isLoad and not pmaMgr_.checkRead(addr, size)) or
                             (isStore and not pmaMgr_.checkWrite(addr, size))))
    cause = EC::STORE_ACC_FAULT;

  if (cause != EC::NONE)
    {
      initiateStoreException(di, cause, ldStFaultAddr_, gaddr);
      return false;
    }

  ldStPhysAddr1_ = ldStPhysAddr2_ = addr;
  return true;
}


template <typename URV>
ExceptionCause
Hart<URV>::determineSsException(uint64_t& addr, uint64_t& gaddr, uint64_t size, Pma::Attrib attrib)
{
  using EC = ExceptionCause;
  using PM = PrivilegeMode;

  uint64_t va = addr;

  bool misal = (va & (size - 1)) != 0;
  if (misal and misalHasPriority_)
    return misalAtomicCauseAccessFault_? EC::STORE_ACC_FAULT : EC::STORE_ADDR_MISAL;

  auto [pm, virt] = effLdStMode();

  if ((pm == PM::Machine) and (attrib == Pma::Attrib::AmoSwap))
    return EC::STORE_ACC_FAULT;

  setMemProtAccIsFetch(false);

  if (isRvs())
    {
      if (pm != PM::Machine)
        {
          bool translateAsRead = attrib == Pma::Attrib::Read;
	  auto cause = virtMem_.translateForSs(va, pm, virt, translateAsRead, gaddr, addr);
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
      const Pmp& pmp = pmpMgr_.accessPmp(effPm, addr, size);
      if (not pmp.isRead()  or not pmp.isWrite())
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

  return EC::NONE;
}


template <typename URV>
void
Hart<URV>::execSspush(const DecodedInst* di, unsigned regNum)
{
  uint64_t addr = ssp_ - sizeof(URV);
  assert((addr & (sizeof(URV) - 1)) == 0);

  if (not ssAccessAllowed(di, addr, sizeof(URV), Pma::Attrib::Write))
    return;

  URV data = intRegs_.read(regNum);
#ifdef FAST_SLOPPY
  fastStore<URV>(di, ldStAddr_, data);
#else
  writeForStore<URV>(ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data);
#endif
  ssp_ -= sizeof(URV);
}


template <typename URV>
void
Hart<URV>::execSspopchk(const DecodedInst* di, unsigned regNum)
{
  uint64_t addr = ssp_;
  assert((addr & (sizeof(URV) - 1)) == 0);

  if (not ssAccessAllowed(di, addr, sizeof(URV), Pma::Attrib::Read))
    return;

  uint64_t data = 0;
#ifdef FAST_SLOPPY
  fastLoad<URV>(di, ldStAddr_, data);
#else
  if (not readForLoad<URV>(di, ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data, 0, 0))
    return;
#endif

  URV expected = intRegs_.read(regNum);
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
  if (not isRvZaamo() or not isRvZicfiss())
    {
      illegalInst(di);
      return;
    }

  // Given the privilege and virtual mode, determines if shadow stack instruction
  // ssamoswap is legal to execute.
  using enum PrivilegeMode;
  auto mode = privMode_;

  if (mode != Machine and not csRegs_.menvcfgSse())
    { illegalInst(di); return; }

  if (not isRvs())
    { illegalInst(di); return; }

  if (mode == User and not virtMode_ and not csRegs_.senvcfgSse())
    { illegalInst(di); return; }

  if (mode == Supervisor and virtMode_ and not csRegs_.henvcfgSse())
    { virtualInst(di); return; }

  if (mode == User  and virtMode_ and not csRegs_.senvcfgSse())
    { virtualInst(di); return; }

  std::unique_lock lock(memory_.amoMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  uint64_t addr = intRegs_.read(rs1);

  if (not ssAccessAllowed(di, addr, sizeof(uint32_t), Pma::Attrib::AmoSwap))
    return;

  uint64_t data = 0;
#ifdef FAST_SLOPPY
  fastLoad<uint32_t>(di, ldStAddr_, data);
  // RV64 SSAMOSWAP.W returns sign-extended 32-bit loaded value in rd.
  intRegs_.write(rd, SRV(int32_t(data)));

  URV rs2Val = intRegs_.read(rs2);
  fastStore<uint32_t>(di, ldStAddr_, uint32_t(rs2Val));
#else
  if (not readForLoad<uint32_t>(di, ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data, 0, 0))
    return;
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
  if (not isRvZaamo() or not isRvZicfiss() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  // Given the privilege and virtual mode, determines if shadow stack instruction
  // ssamoswap is legal to execute.
  using enum PrivilegeMode;
  auto mode = privMode_;

  if (mode != Machine and not csRegs_.menvcfgSse())
    { illegalInst(di); return; }

  if (not isRvs())
    { illegalInst(di); return; }

  if (mode == User and not virtMode_ and not csRegs_.senvcfgSse())
    { illegalInst(di); return; }

  if (mode == Supervisor and virtMode_ and not csRegs_.henvcfgSse())
    { virtualInst(di); return; }

  if (mode == User  and virtMode_ and not csRegs_.senvcfgSse())
    { virtualInst(di); return; }

  std::unique_lock lock(memory_.amoMutex_);

  uint32_t rd = di->op0(), rs1 = di->op1(), rs2 = di->op2();
  uint64_t addr = intRegs_.read(rs1);

  if (not ssAccessAllowed(di, addr, sizeof(uint64_t), Pma::Attrib::AmoSwap))
    return;

  uint64_t data = 0;
#ifdef FAST_SLOPPY
  fastLoad<uint64_t>(di, ldStAddr_, data);
  intRegs_.write(rd, data);

  URV rs2Val = intRegs_.read(rs2);
  fastStore<uint64_t>(di, ldStAddr_, rs2Val);
#else
  if (not readForLoad<uint64_t>(di, ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, data, 0, 0))
    return;
  intRegs_.write(rd, data);

  URV rs2Val = intRegs_.read(rs2);
  writeForStore<uint64_t>(ldStAddr_, ldStPhysAddr1_, ldStPhysAddr2_, rs2Val);
#endif
}

template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
