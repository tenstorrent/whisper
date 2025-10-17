// Copyright 2022 Tenstorrent Corporation.
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

#include <iostream>
#include <cassert>

#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "PerfApi.hpp"

using namespace WdRiscv;


template <typename URV>
ExceptionCause
Hart<URV>::determineCboException(uint64_t& addr, uint64_t& gpa, uint64_t& pa, bool isZero)
{
  addr = URV(addr);   // Truncate to 32 bits in 32-bit mode.

  using EC = ExceptionCause;

  EC cause = EC::NONE;

  // Address translation
  auto [pm, virt] = effLdStMode();

  gpa = pa = addr;

  setMemProtAccIsFetch(false);

  if (isRvs())
    {
      if (pm != PrivilegeMode::Machine)
        {
	  if (isZero)
	    {
	      bool read = false, write = true, exec = false;
	      cause = virtMem_.translate(addr, pm, virt, read, write, exec, gpa, pa);
	      if (cause != EC::NONE)
		return cause;
	    }
	  else
	    {
	      // If load or store is allowed CBO is allowed.
	      bool read = true, write = false, exec = false;
	      cause = virtMem_.translate(addr, pm, virt, read, write, exec, gpa, pa);
	      if (cause != EC::NONE)
		{
		  if (cause == EC::LOAD_ACC_FAULT)
		    return EC::STORE_ACC_FAULT;
		  if (cause == EC::LOAD_PAGE_FAULT)
		    return EC::STORE_PAGE_FAULT;
		  if (cause == EC::LOAD_GUEST_PAGE_FAULT)
		    return EC::STORE_GUEST_PAGE_FAULT;
		  return cause;
		}
	    }
        }
    }

  // Physical memory protection.
  if (pmpEnabled_)
    {
      assert((cacheLineSize_ % 8) == 0);
      auto ep = effectivePrivilege();

      for (uint64_t offset = 0; offset < cacheLineSize_; offset += 8)
	{
	  uint64_t dwa = pa + offset;  // Double word address
	  Pmp pmp = pmpManager_.accessPmp(dwa);
	  if (isZero)
	    {
	      if (not pmp.isWrite(ep))
		return EC::STORE_ACC_FAULT;
	    }
	  else if (not pmp.isRead(ep) and not pmp.isWrite(ep))
	    return EC::STORE_ACC_FAULT;
	}
    }

  steeInsec1_ = false;
  steeInsec2_ = false;

  if (steeEnabled_)
    {
      if (not stee_.isValidAddress(pa))
	return EC::STORE_ACC_FAULT;
      pa = stee_.clearSecureBits(pa);
    }

  for (uint64_t offset = 0; offset < cacheLineSize_; offset += 8)
    {
      Pma pma = accessPma(pa + offset);
      if (isZero)
        {
          if (not pma.isWrite())
            return EC::STORE_ACC_FAULT;
        }
      else if (not pma.isRead() and not pma.isWrite())
        return EC::STORE_ACC_FAULT;
    }

  return EC::NONE;
}


template <typename URV>
void
Hart<URV>::execCbo_clean(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  using CN = CsrNumber;
  MenvcfgFields<uint64_t> menvf(csRegs_.read64(CN::MENVCFG));
  SenvcfgFields<uint64_t> senvf(isRvs()? csRegs_.read64(CN::SENVCFG) : 0);
  HenvcfgFields<uint64_t> henvf(isRvh()? csRegs_.read64(CN::HENVCFG) : 0);

  if ( (pm != PM::Machine and not menvf.bits_.CBCFE) or
       (not virtMode_ and pm == PM::User and not senvf.bits_.CBCFE) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and not henvf.bits_.CBCFE) or
	   (pm == PM::User and not (henvf.bits_.CBCFE and senvf.bits_.CBCFE)) )
	{
	  virtualInst(di);
	  return;
	}
    }

  uint64_t virtAddr = intRegs_.read(di->op0());
  if (alignCboAddr_)
    virtAddr = cacheLineAlign(virtAddr);
  uint64_t gpa = virtAddr;      // Guest physical address
  uint64_t physAddr = virtAddr;
  uint64_t pmva = applyPointerMask(virtAddr, false /*isLoad*/);

  ldStAddr_ = ldStFaultAddr_ = ldStPhysAddr1_ = ldStPhysAddr2_ = virtAddr;
  ldStSize_ = cacheLineSize_;

#ifndef FAST_SLOPPY
  if (hasActiveTrigger())
    ldStAddrTriggerHit(pmva, cacheLineSize_, TriggerTiming::Before, false /* isLoad */);

  if (triggerTripped_)
    return;
#endif

  bool isZero = false;
  auto cause = determineCboException(pmva, gpa, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(di, cause, pmva, gpa);
      return;
    }

  ldStPhysAddr1_ = ldStPhysAddr2_ = physAddr;

  if (cacheBuffer_.max_size() and not cacheTraceFile_.empty())
    traceCache(virtAddr, physAddr, physAddr, false, false, false, false, true);
}


template <typename URV>
void
Hart<URV>::execCbo_flush(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  using CN = CsrNumber;
  MenvcfgFields<uint64_t> menvf(csRegs_.read64(CN::MENVCFG));
  SenvcfgFields<uint64_t> senvf(isRvs()? csRegs_.read64(CN::SENVCFG) : 0);
  HenvcfgFields<uint64_t> henvf(isRvh()? csRegs_.read64(CN::HENVCFG) : 0);

  if ( (pm != PM::Machine and not menvf.bits_.CBCFE) or
       (not virtMode_ and pm == PM::User and not senvf.bits_.CBCFE) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and not henvf.bits_.CBCFE) or
	   (pm == PM::User and not (henvf.bits_.CBCFE and senvf.bits_.CBCFE)) )
	{
	  virtualInst(di);
	  return;
	}
    }

  uint64_t virtAddr = intRegs_.read(di->op0());
  if (alignCboAddr_)
    virtAddr = cacheLineAlign(virtAddr);
  uint64_t gpa = virtAddr;   // Guest physical address
  uint64_t physAddr = virtAddr;
  uint64_t pmva = applyPointerMask(virtAddr, false /*isLoad*/);

  ldStAddr_ = ldStFaultAddr_ = ldStPhysAddr1_ = ldStPhysAddr2_ = virtAddr;
  ldStSize_ = cacheLineSize_;

#ifndef FAST_SLOPPY
  if (hasActiveTrigger())
    ldStAddrTriggerHit(pmva, cacheLineSize_, TriggerTiming::Before, false /* isLoad */);

  if (triggerTripped_)
    return;
#endif

  bool isZero = false;
  auto cause = determineCboException(pmva, gpa, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(di, cause, pmva, gpa);
      return;
    }

  ldStPhysAddr1_ = ldStPhysAddr2_ = physAddr;
}


template <typename URV>
void
Hart<URV>::execCbo_inval(const DecodedInst* di)
{
  if (not isRvzicbom())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  using CN = CsrNumber;
  MenvcfgFields<uint64_t> menvf(csRegs_.read64(CN::MENVCFG));
  SenvcfgFields<uint64_t> senvf(isRvs()? csRegs_.read64(CN::SENVCFG) : 0);
  HenvcfgFields<uint64_t> henvf(isRvh()? csRegs_.read64(CN::HENVCFG) : 0);

  if ( (pm != PM::Machine and menvf.bits_.CBIE == 0) or
       (not virtMode_ and pm == PM::User and senvf.bits_.CBIE == 0) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and henvf.bits_.CBIE == 0) or
	   (pm == PM::User and (henvf.bits_.CBIE == 0 or senvf.bits_.CBIE == 0)) )
	{
	  virtualInst(di);
	  return;
	}
    }

  bool isZero = false;

  uint64_t virtAddr = intRegs_.read(di->op0());
  if (alignCboAddr_)
    virtAddr = cacheLineAlign(virtAddr);
  uint64_t gpa = virtAddr;    // Guest physical addres
  uint64_t physAddr = virtAddr;
  uint64_t pmva = applyPointerMask(virtAddr, false /*isLoad*/);

  ldStAddr_ = ldStFaultAddr_ = ldStPhysAddr1_ = ldStPhysAddr2_ = virtAddr;
  ldStSize_ = cacheLineSize_;

#ifndef FAST_SLOPPY
  if (hasActiveTrigger())
    ldStAddrTriggerHit(pmva, cacheLineSize_, TriggerTiming::Before, false /* isLoad */);
  if (triggerTripped_)
    return;
#endif

  auto cause = determineCboException(pmva, gpa, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(di, cause, pmva, gpa);
      return;
    }

  ldStPhysAddr1_ = ldStPhysAddr2_ = physAddr;

  if (cacheBuffer_.max_size() and not cacheTraceFile_.empty())
    {
      // FIXME: check CBIE bits.
      traceCache(virtAddr, physAddr, physAddr, false, false, false, false, true);
    }
}


template <typename URV>
void
Hart<URV>::execCbo_zero(const DecodedInst* di)
{
  if (not isRvzicboz())
    {
      illegalInst(di);
      return;
    }

  using PM = PrivilegeMode;
  PM pm = privilegeMode();

  using CN = CsrNumber;
  MenvcfgFields<uint64_t> menvf(csRegs_.read64(CN::MENVCFG));
  SenvcfgFields<uint64_t> senvf(isRvs()? csRegs_.read64(CN::SENVCFG) : 0);
  HenvcfgFields<uint64_t> henvf(isRvh()? csRegs_.read64(CN::HENVCFG) : 0);

  if ( (pm != PM::Machine and not menvf.bits_.CBZE) or
       (not virtMode_ and pm == PM::User and not senvf.bits_.CBZE) )
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      if ( (pm == PM::Supervisor and not henvf.bits_.CBZE) or
	   (pm == PM::User and not (henvf.bits_.CBZE and senvf.bits_.CBZE)) )
	{
	  virtualInst(di);
	  return;
	}
    }

  // Translate virtual addr and check for exception.
  uint64_t virtAddr = intRegs_.read(di->op0());
  if (alignCboAddr_)
    virtAddr = cacheLineAlign(virtAddr);  // To report aligned address in xTVAL.
  uint64_t gpa = virtAddr;     // Guest physical address
  uint64_t physAddr = virtAddr;
  uint64_t pmva = applyPointerMask(virtAddr, false /*isLoad*/);

  ldStAddr_ = ldStFaultAddr_ = ldStPhysAddr1_ = ldStPhysAddr2_ = virtAddr;
  ldStSize_ = cacheLineSize_;

#ifndef FAST_SLOPPY
  if (hasActiveTrigger())
    ldStAddrTriggerHit(pmva, cacheLineSize_, TriggerTiming::Before, false /* isLoad */);
  if (triggerTripped_)
    return;
#endif

  bool isZero = true;
  auto cause = determineCboException(pmva, gpa, physAddr, isZero);
  if (cause != ExceptionCause::NONE)
    {
      initiateStoreException(di, cause, pmva, gpa);
      return;
    }

  ldStWrite_ = true;
  ldStPhysAddr1_ = ldStPhysAddr2_ = physAddr;

  if (ooo_)
    {
      if (perfApi_)
        {
          uint64_t val = 0;
          unsigned size = sizeof(val);   // Chunk size.
          for (unsigned i = 0; i < cacheLineSize_; i += size)
            {
              uint64_t pa = physAddr + i;
              perfApi_->setStoreData(hartIx_, instCounter_, pa, pa, size, val);
            }
        }

      // For MCM: We update memory when we get bypass message from test bench.

      return;
    }

  uint64_t pa = cacheLineAlign(physAddr);
  for (unsigned i = 0; i < cacheLineSize_; i += 8, pa += 8)
    memWrite(pa, pa, uint64_t(0));

  if (cacheBuffer_.max_size() and not cacheTraceFile_.empty())
    traceCache(virtAddr, pa, pa, false, true, false, false, false);
}


template <typename URV>
void
Hart<URV>::execPrefetch_i(const DecodedInst* di)
{
  if (not isRvzicbop())
    {
      illegalInst(di);
      return;
    }
}


template <typename URV>
void
Hart<URV>::execPrefetch_r(const DecodedInst* di)
{
  if (not isRvzicbop())
    {
      illegalInst(di);
      return;
    }
}


template <typename URV>
void
Hart<URV>::execPrefetch_w(const DecodedInst* di)
{
  if (not isRvzicbop())
    {
      illegalInst(di);
      return;
    }
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
