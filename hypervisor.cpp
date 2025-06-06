// Copyright 2022 Tenstorretn Corporation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//stributed under the License isstributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <climits>
#include <cassert>

#include "instforms.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "Mcm.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execHfence_vvma(const DecodedInst* di)
{
  using PM = PrivilegeMode;

  if (not isRvh())
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);
      return;
    }

  if (privMode_ == PM::User)
    {
      illegalInst(di);
      return;
    }

  auto& vsTlb = virtMem_.vsTlb_;
  auto& stage2Tlb = virtMem_.stage2Tlb_;

  auto vmid = virtMem_.vmid();
  uint32_t wid = steeEnabled_? stee_.secureWorld() : 0;

  if (di->op0() == 0 and di->op1() == 0)
    {
      vsTlb.invalidateVmid(vmid, wid);
      stage2Tlb.invalidateVmid(vmid, wid);
    }
  else if (di->op0() == 0 and di->op1() != 0)
    {
      URV asid = intRegs_.read(di->op1());
      vsTlb.invalidateAsidVmid(asid, vmid, wid);
      stage2Tlb.invalidateAsidVmid(asid, vmid, wid);
    }
  else if (di->op0() != 0 and di->op1() == 0)
    {
      URV addr = intRegs_.read(di->op0());
      uint64_t vpn = virtMem_.pageNumber(addr);
      vsTlb.invalidateVirtualPageVmid(vpn, vmid, wid);
      stage2Tlb.invalidateVmid(vmid, wid);
    }
  else
    {
      URV addr = intRegs_.read(di->op0());
      uint64_t vpn = virtMem_.pageNumber(addr);
      URV asid = intRegs_.read(di->op1());
      vsTlb.invalidateVirtualPageAsid(vpn, asid, wid);
      stage2Tlb.invalidateAsidVmid(asid, vmid, wid);
    }
}


template <typename URV>
void
Hart<URV>::execHfence_gvma(const DecodedInst* di)
{
  using PM = PrivilegeMode;

  if (not isRvh())
    {
      illegalInst(di);
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);
      return;
    }

  if (privMode_ == PM::User or (privMode_ == PM::Supervisor and mstatus_.bits_.TVM == 1))
    {
      illegalInst(di);
      return;
    }

  auto& stage2Tlb = virtMem_.stage2Tlb_;
  auto& vsTlb = virtMem_.vsTlb_;

  auto vmid = virtMem_.vmid();
  uint32_t wid = steeEnabled_? stee_.secureWorld() : 0;

  // Some implementations do not store guest-physical-addresses in the TLB. For those, we
  // over-invalidate.
  bool useGpa = not hfenceGvmaIgnoresGpa_;

  // Invalidate whole VS TLB. This is overkill.
  if (di->op0() == 0 and di->op1() == 0)
    {
      stage2Tlb.invalidateVmid(vmid, wid);
      vsTlb.invalidateVmid(vmid, wid);
    }
  else if (di->op0() == 0 and di->op1() != 0)
    {
      URV vmid = intRegs_.read(di->op1());
      stage2Tlb.invalidateVmid(vmid, wid);
      vsTlb.invalidateVmid(vmid, wid);
    }
  else if (di->op0() != 0 and di->op1() == 0)
    {
      URV addr = intRegs_.read(di->op0()) << 2;
      uint64_t vpn = virtMem_.pageNumber(addr);
      if (useGpa)
	stage2Tlb.invalidateVirtualPageVmid(vpn, vmid, wid);
      else
        stage2Tlb.invalidateVmid(vmid, wid);
      vsTlb.invalidateVmid(vmid, wid);
    }
  else
    {
      URV addr = intRegs_.read(di->op0()) << 2;
      uint64_t vpn = virtMem_.pageNumber(addr);
      URV vmid = intRegs_.read(di->op1());
      if (useGpa)
        stage2Tlb.invalidateVirtualPageVmid(vpn, vmid, wid);
      else
        stage2Tlb.invalidateVmid(vmid, wid);

      vsTlb.invalidateVmid(vmid, wid);
    }
}


template <typename URV>
template <typename LOAD_TYPE>
void
Hart<URV>::hyperLoad(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }

  if (privMode_ == PrivilegeMode::User and not hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  // Use VS mode big-endian/make-exec-readbale for translation.
  bool prevTbe = virtMem_.bigEndian();  // Previous translation big endian.
  bool prevMxr = virtMem_.stage1ExecReadable();  // Previous stage1 MXR.
  bool prevVsSum = virtMem_.vsSum();

  PmaskManager::Mode prevPmm = pmaskManager_.getMode(PrivilegeMode::User, true /* twoStage */);
  virtMem_.setBigEndian(hstatus_.bits_.VSBE);
  virtMem_.setStage1ExecReadable(vsstatus_.bits_.MXR);
  virtMem_.setVsSum(vsstatus_.bits_.SUM);
  pmaskManager_.setStage1ExecReadable(vsstatus_.bits_.MXR);

  if constexpr (isRv64())
    if (privMode_ == PrivilegeMode::User and not virtMode_)
      pmaskManager_.enablePointerMasking(PmaskManager::Mode(hstatus_.bits_.HUPMM),
                                         PrivilegeMode::User, true);
  hyperLs_ = true;

  URV virtAddr = intRegs_.read(di->op1());
  uint64_t data = 0;
  auto savedPrivMode = privMode_;
  auto savedVirtMode = virtMode_;
  if (load<LOAD_TYPE>(di, virtAddr, true /*hyper*/, data))
    intRegs_.write(di->op0(), data);

  hyperLs_ = false;
  virtMem_.setBigEndian(prevTbe);            // Restore big endian mod.
  virtMem_.setStage1ExecReadable(prevMxr);   // Restore stage1 MXR.
  virtMem_.setVsSum(prevVsSum);
  pmaskManager_.setStage1ExecReadable(prevMxr);   // Restore stage1 MXR.

  if constexpr (isRv64())
    if (savedPrivMode == PrivilegeMode::User and not savedVirtMode)
      pmaskManager_.enablePointerMasking(prevPmm, PrivilegeMode::User, true);
}


template <typename URV>
void
Hart<URV>::execHlv_b(const DecodedInst* di)
{
  hyperLoad<int8_t>(di);
}


template <typename URV>
void
Hart<URV>::execHlv_bu(const DecodedInst* di)
{
  hyperLoad<uint8_t>(di);
}


template <typename URV>
void
Hart<URV>::execHlv_h(const DecodedInst* di)
{
  hyperLoad<int16_t>(di);
}


template <typename URV>
void
Hart<URV>::execHlv_hu(const DecodedInst* di)
{
  hyperLoad<uint16_t>(di);
}


template <typename URV>
void
Hart<URV>::execHlv_w(const DecodedInst* di)
{
  hyperLoad<int32_t>(di);
}


template <typename URV>
void
Hart<URV>::execHlv_wu(const DecodedInst* di)
{
  hyperLoad<uint32_t>(di);
}


template <typename URV>
void
Hart<URV>::execHlvx_hu(const DecodedInst* di)
{
  virtMem_.useExecForRead(true);
  pmaskManager_.useExecForRead(true);

  hyperLoad<uint16_t>(di);

  virtMem_.useExecForRead(false);
  pmaskManager_.useExecForRead(false);
}


template <typename URV>
void
Hart<URV>::execHlvx_wu(const DecodedInst* di)
{
  virtMem_.useExecForRead(true);
  pmaskManager_.useExecForRead(true);

  hyperLoad<uint32_t>(di);

  virtMem_.useExecForRead(false);
  pmaskManager_.useExecForRead(false);
}


template <typename URV>
void
Hart<URV>::execHlv_d(const DecodedInst* di)
{
  hyperLoad<uint64_t>(di);
}


template <typename URV>
template <typename STORE_TYPE>
void
Hart<URV>::hyperStore(const DecodedInst* di)
{
  if (not isRvh())
    {
      illegalInst(di);    // H extension must be enabled.
      return;
    }

  if (virtMode_)
    {
      virtualInst(di);    // Must not be in V mode.
      return;
    }

  if (privMode_ == PrivilegeMode::User and not hstatus_.bits_.HU)
    {
      illegalInst(di);    // Must not be in User mode unless HSTATUS.HU
      return;
    }

  // Use VS mode big-endian for translation.
  bool prevTbe = virtMem_.bigEndian();  // Previous translation big endian.
  bool prevVsSum = virtMem_.vsSum();

  auto prevPmm = pmaskManager_.getMode(PrivilegeMode::User, true /* twoStage */);
  virtMem_.setBigEndian(hstatus_.bits_.VSBE);
  virtMem_.setVsSum(vsstatus_.bits_.SUM);

  if constexpr (isRv64())
    if (privMode_ == PrivilegeMode::User and not virtMode_)
      pmaskManager_.enablePointerMasking(PmaskManager::Mode(hstatus_.bits_.HUPMM),
                                         PrivilegeMode::User, true);
  hyperLs_ = true;

  uint32_t rs1 = di->op1();
  URV virtAddr = intRegs_.read(rs1);
  STORE_TYPE value = STORE_TYPE(intRegs_.read(di->op0()));
  auto savedPrivMode = privMode_;
  auto savedVirtMode = virtMode_;
  store<STORE_TYPE>(di, virtAddr, true /*hyper*/, value);

  hyperLs_ = false;
  virtMem_.setBigEndian(prevTbe);            // Restore big endian mod.
  virtMem_.setVsSum(prevVsSum);

  if constexpr (isRv64())
    if (savedPrivMode == PrivilegeMode::User and not savedVirtMode)
      pmaskManager_.enablePointerMasking(prevPmm, PrivilegeMode::User, true);
}


template <typename URV>
void
Hart<URV>::execHsv_b(const DecodedInst* di)
{
  hyperStore<uint8_t>(di);
}


template <typename URV>
void
Hart<URV>::execHsv_h(const DecodedInst* di)
{
  hyperStore<uint16_t>(di);
}


template <typename URV>
void
Hart<URV>::execHsv_w(const DecodedInst* di)
{
  hyperStore<uint32_t>(di);
}


template <typename URV>
void
Hart<URV>::execHsv_d(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }
  hyperStore<uint64_t>(di);
}


template <typename URV>
void
Hart<URV>::execHinval_vvma(const DecodedInst* di)
{
  if (not isRvsvinval())
    illegalInst(di);
  else
    execHfence_vvma(di);
}


template <typename URV>
void
Hart<URV>::execHinval_gvma(const DecodedInst* di)
{
  if (not isRvsvinval())
    illegalInst(di);
  else
    execHfence_gvma(di);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
