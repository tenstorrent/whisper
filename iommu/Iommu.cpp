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

#include "Iommu.hpp"
#include "MsiPte.hpp"
#include <iostream>
#include <algorithm>
#include <cstdarg>

using namespace TT_IOMMU;

void
dbg_fprintf(FILE * fp, const char *fmt, ...)
{
#if DEBUG_IOMMU
  valist ap;
  va_start(ap, fmt);
  vfprintf(stdout, fmt, ap);
  va_end(ap);
#else
  (void) fp;
  (void) fmt;
#endif
}

bool
Iommu::read(uint64_t addr, unsigned size, uint64_t& data) const
{
  // Size must be 4 or 8. Address must be aligned.
  if (size != 4 and size != 8 and (addr & (size-1)) != 0)
    return false;

  uint64_t offset = addr - addr_;
  if (offset < 1024)
    return readCsr(offset, size, data);

  // For PMPCFG/PMADDR access, size must be 8 and address must double-word aligned
  if (pmpEnabled_)
    {
      if (isPmpcfgAddr(addr))
        {
          const unsigned pmpcfgSize = 8;
          if (size != pmpcfgSize or (addr & (pmpcfgSize - 1)) != 0)
            return false;
          unsigned ix = (addr - pmpcfgAddr_) / pmpcfgSize;
          data = pmpcfg_.at(ix);
          return true;
        }

      if (isPmpaddrAddr(addr))
        {
          const unsigned pmpaddrSize = 8;
          if (size != pmpaddrSize or (addr & (pmpaddrSize - 1)) != 0)
            return false;
          unsigned ix = (addr - pmpaddrAddr_) / pmpaddrSize;
          data = pmpaddr_.at(ix);
          assert(0 && "adjust PMPADDR value according to type");
          return true;
        }

      // Not a PMP address. Check if PMA.
    }

  return false;
}

// NOLINTBEGIN(bugprone-branch-clone)
bool
Iommu::readCsr(uint64_t offset, unsigned size, uint64_t& data) const
{
  //auto offset = addr - base_;
  if (size != 4 and size != 8)
    return false;
  if (offset % 4 != 0)
    return false;
  if (size == 8 and offset % 8 != 0)
    return false;
  if (size == 8 and sizeAtWordOffset_.at(offset/4) == 4)
    return false;

  switch (offset)
    {
      case 0:  case 4:      data = readCapabilities();  break;
      case 8:               data = readFctl();          break;
      case 12:              return false; // reserved
      case 16: case 20:     data = readDdtp();          break;
      case 24: case 28:     data = readCqb();           break;
      case 32:              data = readCqh();           break;
      case 36:              data = readCqt();           break;
      case 40: case 44:     data = readFqb();           break;
      case 48:              data = readFqh();           break;
      case 52:              data = readFqt();           break;
      case 56: case 60:     data = readPqb();           break;
      case 64:              data = readPqh();           break;
      case 68:              data = readPqt();           break;
      case 72:              data = readCqcsr();         break;
      case 76:              data = readFqcsr();         break;
      case 80:              data = readPqcsr();         break;
      case 84:              data = readIpsr();          break;
      case 88:              data = readIocountovf();    break;
      case 92:              data = readIocountinh();    break;
      case 96:  case 100:   data = readIohpmcycles();   break;
      case 600: case 604:   data = readTrReqIova();     break;
      case 608: case 612:   data = readTrReqCtl();      break;
      case 616: case 620:   data = readTrResponse();    break;
      case 624:             data = readIommuQosid();    break;
      case 760: case 764:   data = readIcvec();         break;
      default: break;
    }

  if (offset >= 104 and offset < 352)
    {
      unsigned index = (offset - 104)/8 + 1;
      data = readIohpmctr(index);
    }
  else if (offset >= 352 and offset < 600)
    {
      unsigned index = (offset - 352)/8 + 1;
      data = readIohpmevt(index);
    }
  else if (offset >= 628 and offset < 760)
    return false; // reserved and custom
  else if (offset >= 768 and offset < 1024)
    {
      unsigned index = (offset - 768)/16;
      unsigned reg = offset % 16;
      if      (reg < 8)   data = readMsiAddr(index);
      else if (reg == 8)  data = readMsiData(index);
      else if (reg == 12) data = readMsiVecCtl(index);
    }
  else if (offset >= 1024)
    return false; // reserved

  unsigned regSize = sizeAtWordOffset_.at((offset & ~7)/4);
  if (size == 4 and regSize == 8)
    {
      if (offset % 8 == 4)
        data >>= 32;
      data &= 0xffffffff;
    }

  return true;
}
// NOLINTEND(bugprone-branch-clone)


bool
Iommu::write(uint64_t addr, unsigned size, uint64_t data)
{
  // Size must be 4 or 8. Address must be aligned.
  if (size != 4 and size != 8 and (addr & (size-1)) != 0)
    return false;

  uint64_t offset = addr - addr_;
  if (offset < 1024)
    return writeCsr(offset, size, data);

  // For PMPCFG/PMADDR access, size must be 8 and address must double-word aligned
  if (pmpEnabled_)
    {
      if (isPmpcfgAddr(addr))
        {
          const unsigned pmpcfgSize = 8;
          if (size != pmpcfgSize or (addr & (pmpcfgSize - 1)) != 0)
            return false;
          unsigned ix = (addr - pmpcfgAddr_) / pmpcfgSize;
          uint64_t prev = pmpcfg_.at(ix);
          data = pmpMgr_.legalizePmpcfg(prev, data);
          pmpcfg_.at(ix) = data;
          updateMemoryProtection();
          return true;
        }

      if (isPmpaddrAddr(addr))
        {
          const unsigned pmpaddrSize = 8;
          if (size != pmpaddrSize or (addr & (pmpaddrSize - 1)) != 0)
            return false;
          unsigned ix = (addr - pmpaddrAddr_) / pmpaddrSize;
          pmpaddr_.at(ix) = data;

          uint8_t cfgByte =  getPmpcfgByte(ix);
          if (((cfgByte >> 3) & 3) != 0)   // If type != Off
            updateMemoryProtection();
          return true;
        }
    }

  if (pmaEnabled_ and isPmacfgAddr(addr))
    {
      const unsigned pmacfgSize = 8;
      if (size != pmacfgSize or (addr & (pmacfgSize - 1)) != 0)
        return false;

      unsigned ix = (addr - pmacfgAddr_) / pmacfgSize;
      uint64_t prev = pmacfg_.at(ix);
      data = PmaManager::legalizePmacfg(prev, data);
      pmacfg_.at(ix) = data;
      updateMemoryAttributes(ix);
      return true;
    }

  return false;
}

// NOLINTBEGIN(bugprone-branch-clone, readability-else-after-return)
bool
Iommu::writeCsr(uint64_t offset, unsigned size, uint64_t data)
{
  if (size != 4 and size != 8)
    return false;
  if (offset % 4 != 0)
    return false;
  if (size == 8 and offset % 8 != 0)
    return false;
  if (size == 8 and sizeAtWordOffset_.at(offset/4) == 4)
    return false;

  unsigned regSize = sizeAtWordOffset_.at((offset & ~7)/4);
  unsigned wordMask = 3;
  if (size == 4)
    wordMask = (offset % 8 == 0) ? 1 : 2;
  if (regSize == 8 and wordMask == 2)
    data <<= 32;

  switch (offset)
    {
      case 0:   case 4:     /* capabilities is RO */            return true;
      case 8:               writeFctl(data);                    return true;
      case 16:  case 20:    writeDdtp(data, wordMask);          return true;
      case 24:  case 28:    writeCqb(data, wordMask);           return true;
      case 32:              /* cqh is RO */                     return true;
      case 36:              writeCqt(data);                     return true;
      case 40:  case 44:    writeFqb(data, wordMask);           return true;
      case 48:              writeFqh(data);                     return true;
      case 52:              /* fqt is RO */                     return true;
      case 56:  case 60:    writePqb(data, wordMask);           return true;
      case 64:              writePqh(data);                     return true;
      case 68:              /* pqt is RO */                     return true;
      case 72:              writeCqcsr(data);                   return true;
      case 76:              writeFqcsr(data);                   return true;
      case 80:              writePqcsr(data);                   return true;
      case 84:              writeIpsr(data);                    return true;
      case 88:              /* iocountovf is RO */              return true;
      case 92:              writeIocountinh(data);              return true;
      case 96:  case 100:   writeIohpmcycles(data, wordMask);   return true;
      case 600: case 604:   writeTrReqIova(data, wordMask);     return true;
      case 608: case 612:   writeTrReqCtl(data, wordMask);      return true;
      case 616: case 620:   /* tr_response is RO */             return true;
      case 624:             writeIommuQosid(data);              return true;
      case 760: case 764:   writeIcvec(data);                   return true;
      default: break;
    }

  if (offset >= 104 and offset < 352)
    {
      unsigned index = (offset - 104)/8 + 1;
      writeIohpmctr(index, data, wordMask);
      return true;
    }
  else if (offset >= 352 and offset < 600)
    {
      unsigned index = (offset - 352)/8 + 1;
      writeIohpmevt(index, data, wordMask);
      return true;
    }
  else if (offset >= 768 and offset < 1024)
    {
      unsigned index = (offset - 768)/16;
      unsigned reg = offset % 16;
      if      (reg < 8)   writeMsiAddr(index, data, wordMask);
      else if (reg == 8)  writeMsiData(index, data);
      else if (reg == 12) writeMsiVecCtl(index, data);
      return true;
    }

  return false;
}
// NOLINTEND(bugprone-branch-clone, readability-else-after-return)


void Iommu::writeFctl(uint32_t data)
{
  Fctl new_fctl { .value = data };
  if (beWritable_)  fctl_.fields.be  = new_fctl.fields.be;
  if (wsiWritable_) fctl_.fields.wsi = new_fctl.fields.wsi;
  if (gxlWritable_) fctl_.fields.gxl = new_fctl.fields.gxl;
}


void Iommu::writeDdtp(uint64_t data, unsigned wordMask)
{
  Ddtp new_ddtp { .value = data };
  if (int(new_ddtp.fields.iommu_mode) > 4)
    new_ddtp.fields.iommu_mode = ddtp_.fields.iommu_mode;
  new_ddtp.fields.busy = ddtp_.fields.busy;
  new_ddtp.fields.reserved0 = 0;
  new_ddtp.fields.reserved1 = 0;
  if (wordMask & 1) ddtp_.words[0] = new_ddtp.words[0];
  if (wordMask & 2) ddtp_.words[1] = new_ddtp.words[1];
}


void Iommu::writeCqb(uint64_t data, unsigned wordMask)
{
  Cqb new_cqb { .value = data };
  // clear 31:LOG2SZ in cqt
  cqt_ &= (1u << (new_cqb.fields.log2szm1+1)) - 1u;
  if (wordMask & 1) cqb_.words[0] = new_cqb.words[0];
  if (wordMask & 2) cqb_.words[1] = new_cqb.words[1];
}


void Iommu::writeCqt(uint32_t data)
{
  // only LOG2SZ-1:0 bits are writable
  uint32_t mask = (1u << (cqb_.fields.log2szm1+1)) - 1u;
  cqt_ = data & mask;
  processCommandQueue();
}


void Iommu::writeFqb(uint64_t data, unsigned wordMask)
{
  Fqb new_fqb { .value = data };
  // clear 31:LOG2SZ in fqh
  fqh_ &= (1u << (new_fqb.fields.log2szm1+1)) - 1u;
  if (wordMask & 1) fqb_.words[0] = new_fqb.words[0];
  if (wordMask & 2) fqb_.words[1] = new_fqb.words[1];
}


void Iommu::writeFqh(uint32_t data)
{
  // only LOG2SZ-1:0 bits are writable
  uint32_t mask = (1u << (fqb_.fields.log2szm1+1)) - 1u;
  fqh_ = data & mask;
}


void Iommu::writePqb(uint64_t data, unsigned wordMask)
{
  if (capabilities_.fields.ats == 0)
    return;
  Pqb new_pqb { .value = data };
  // clear 31:LOG2SZ in pqh
  pqh_ &= (1u << (new_pqb.fields.log2szm1+1)) - 1u;
  if (wordMask & 1) pqb_.words[0] = new_pqb.words[0];
  if (wordMask & 2) pqb_.words[1] = new_pqb.words[1];
}


void Iommu::writePqh(uint32_t data)
{
  if (capabilities_.fields.ats == 0)
    return;
  // only LOG2SZ-1:0 bits are writable
  uint32_t mask = (1u << (pqb_.fields.log2szm1+1)) - 1u;
  pqh_ = data & mask;
}


void Iommu::writeCqcsr(uint32_t data)
{
  if (cqcsr_.fields.busy)
    return;

  Cqcsr new_cqcsr = { .value = data };
  bool cqen_posedge = not cqcsr_.fields.cqen and new_cqcsr.fields.cqen;
  bool cqen_negedge = cqcsr_.fields.cqen and not new_cqcsr.fields.cqen;

  if (cqen_posedge) {
      cqh_ = 0;
      cqcsr_.fields.cmd_ill = 0;
      cqcsr_.fields.cmd_to = 0;
      cqcsr_.fields.cqmf = 0;
      cqcsr_.fields.fence_w_ip = 0;
      cqcsr_.fields.cqon = 1;
  } else if (cqen_negedge) {
      cqcsr_.fields.cqon = 0;
  }

  cqcsr_.fields.cqen = new_cqcsr.fields.cqen;
  cqcsr_.fields.cie = new_cqcsr.fields.cie;
  if (new_cqcsr.fields.cqmf)
    cqcsr_.fields.cqmf = 0;
  if (new_cqcsr.fields.cmd_to)
    cqcsr_.fields.cmd_to = 0;
  if (new_cqcsr.fields.cmd_ill)
    cqcsr_.fields.cmd_ill = 0;
  if (new_cqcsr.fields.fence_w_ip)
    cqcsr_.fields.fence_w_ip = 0;
}


void Iommu::writeFqcsr(uint32_t data)
{
  if (fqcsr_.fields.busy)
    return;

  Fqcsr new_fqcsr = { .value = data };
  bool fqen_posedge = not fqcsr_.fields.fqen and new_fqcsr.fields.fqen;
  bool fqen_negedge = fqcsr_.fields.fqen and not new_fqcsr.fields.fqen;

  if (fqen_posedge) {
      fqt_ = 0;
      fqcsr_.fields.fqof = 0;
      fqcsr_.fields.fqmf = 0;
      fqcsr_.fields.fqon = 1;
  } else if (fqen_negedge) {
      fqcsr_.fields.fqon = 0;
  }

  fqcsr_.fields.fqen = new_fqcsr.fields.fqen;
  fqcsr_.fields.fie = new_fqcsr.fields.fie;
  if (new_fqcsr.fields.fqmf)
    fqcsr_.fields.fqmf = 0;
  if (new_fqcsr.fields.fqof)
    fqcsr_.fields.fqof = 0;
}


void Iommu::writePqcsr(uint32_t data)
{
  if (capabilities_.fields.ats == 0)
    return;
  if (pqcsr_.fields.busy)
    return;

  Pqcsr new_pqcsr = { .value = data };
  bool pqen_posedge = not pqcsr_.fields.pqen and new_pqcsr.fields.pqen;
  bool pqen_negedge = pqcsr_.fields.pqen and not new_pqcsr.fields.pqen;

  if (pqen_posedge) {
      pqt_ = 0;
      pqcsr_.fields.pqof = 0;
      pqcsr_.fields.pqmf = 0;
      pqcsr_.fields.pqon = 1;
  } else if (pqen_negedge) {
      pqcsr_.fields.pqon = 0;
  }

  pqcsr_.fields.pqen = new_pqcsr.fields.pqen;
  pqcsr_.fields.pie = new_pqcsr.fields.pie;
  if (new_pqcsr.fields.pqmf)
    pqcsr_.fields.pqmf = 0;
  if (new_pqcsr.fields.pqof)
    pqcsr_.fields.pqof = 0;
}


void Iommu::writeIpsr(uint32_t data)
{
  Ipsr new_ipsr { .value = data };
  
  // For WSI mode, deassert interrupts when pending bits are cleared
  if (wiredInterrupts() && signalWiredInterrupt_)
    {
      if (new_ipsr.fields.cip && ipsr_.fields.cip)
        signalWiredInterrupt_(icvec_.fields.civ, false);  // Deassert command queue interrupt
      if (new_ipsr.fields.fip && ipsr_.fields.fip)
        signalWiredInterrupt_(icvec_.fields.fiv, false);  // Deassert fault queue interrupt
      if (new_ipsr.fields.pip && ipsr_.fields.pip)
        signalWiredInterrupt_(icvec_.fields.piv, false);  // Deassert page request interrupt
      if (new_ipsr.fields.pmip && ipsr_.fields.pmip)
        signalWiredInterrupt_(icvec_.fields.pmiv, false); // Deassert performance monitor interrupt
    }
  
  // Clear the pending bits (RW1C - write 1 to clear)
  if (new_ipsr.fields.cip)
    ipsr_.fields.cip = 0;
  if (new_ipsr.fields.fip)
    ipsr_.fields.fip = 0;
  if (new_ipsr.fields.pmip)
    ipsr_.fields.pmip = 0;
  if (new_ipsr.fields.pip)
    ipsr_.fields.pip = 0;
  
  updateIpsr(); // may need to reassert interrupt pending bits
}


void Iommu::writeIocountinh(uint32_t data)
{
  if (capabilities_.fields.hpm == 0)
    return;
  iocountinh_.value = data;
}


void Iommu::writeIohpmcycles(uint64_t data, unsigned wordMask)
{
  if (capabilities_.fields.hpm == 0)
    return;
  Iohpmcycles new_iohpmcycles { .value = data };
  if (wordMask & 1) iohpmcycles_.words[0] = new_iohpmcycles.words[0];
  if (wordMask & 2) iohpmcycles_.words[1] = new_iohpmcycles.words[1];
}


void Iommu::writeTrReqIova(uint64_t data, unsigned wordMask)
{
  if (capabilities_.fields.dbg == 0)
    return;

  // Behavior is unspecified if tr_req_iova is modified when go_busy is 1
  // for now, ignore writes when busy
  if (tr_req_ctl_.fields.go_busy == 1)
    return;

  TrReqIova new_tr_req_iova { .value = data };
  new_tr_req_iova.fields.reserved = 0;
  if (wordMask & 1) tr_req_iova_.words[0] = new_tr_req_iova.words[0];
  if (wordMask & 2) tr_req_iova_.words[1] = new_tr_req_iova.words[1];
}


void Iommu::writeTrReqCtl(uint64_t data, unsigned wordMask)
{
  if (capabilities_.fields.dbg == 0)
    return;

  // Only allow writes when go_busy is 0 (not busy)
  if (tr_req_ctl_.fields.go_busy == 1)
    return;

  TrReqCtl new_tr_req_ctl { .value = data };
  new_tr_req_ctl.fields.reserved0 = 0;
  new_tr_req_ctl.fields.reserved1 = 0;
  new_tr_req_ctl.fields.custom = 0;

  // Check if this is a 0→1 transition on go_busy
  bool go_busy_transition = (tr_req_ctl_.fields.go_busy == 0) &&
                            (new_tr_req_ctl.fields.go_busy == 1);

  if (wordMask & 1) tr_req_ctl_.words[0] = new_tr_req_ctl.words[0];
  if (wordMask & 2) tr_req_ctl_.words[1] = new_tr_req_ctl.words[1];

  // Process debug translation on go_busy 0→1 transition
  if (go_busy_transition)
    processDebugTranslation();
}


void Iommu::processDebugTranslation()
{

  IommuRequest req;
  req.devId = tr_req_ctl_.fields.did;
  req.hasProcId = tr_req_ctl_.fields.pv;
  req.procId = tr_req_ctl_.fields.pid;
  req.iova = tr_req_iova_.value;
  req.size = 1;  // Single byte access

  // NW=1 means READ, NW=0 means WRITE
  if (tr_req_ctl_.fields.nw)
    req.type = Ttype::UntransRead;
  else
    req.type = Ttype::UntransWrite;

  // Set privilege mode based on Priv bit
  req.privMode = tr_req_ctl_.fields.priv ?
                 PrivilegeMode::Supervisor : PrivilegeMode::User;

  // Note: Exe bit is present in tr_req_ctl but we use NW to determine
  // read vs write. The Exe bit could be used for execute-specific checks.

  // Perform the translation
  uint64_t pa = 0;
  unsigned cause = 0;
  bool success = translate(req, pa, cause);

  // Build the response
  tr_response_.value = 0;  // Clear all fields
  tr_response_.fields.reserved0 = 0;
  tr_response_.fields.reserved1 = 0;
  tr_response_.fields.custom = 0;

  if (success)
    {
      tr_response_.fields.fault = 0;
      tr_response_.fields.ppn = pa >> 12;

      tr_response_.fields.pbmt = 0;
      tr_response_.fields.s = 0;
    }
  else
    {
      // Translation failed
      tr_response_.fields.fault = 1;
      // Per spec, PBMT, S, and PPN are UNSPECIFIED on fault
      // We set them to 0
      tr_response_.fields.pbmt = 0;
      tr_response_.fields.s = 0;
      tr_response_.fields.ppn = 0;
    }

  // Clear the go_busy bit to indicate completion
  tr_req_ctl_.fields.go_busy = 0;
}


void Iommu::writeIommuQosid(uint32_t data)
{
  if (capabilities_.fields.qosid == 0)
    return;
  IommuQosid new_iommu_qosid { .value = data };
  iommu_qosid_.fields.rcid = new_iommu_qosid.fields.rcid;
  iommu_qosid_.fields.mcid = new_iommu_qosid.fields.mcid;
}


void Iommu::writeIcvec(uint32_t data)
{
  Icvec new_icvec { .value = data };
  icvec_.fields.civ = new_icvec.fields.civ;
  icvec_.fields.fiv = new_icvec.fields.fiv;
  icvec_.fields.pmiv = new_icvec.fields.pmiv;
  icvec_.fields.piv = new_icvec.fields.piv;
}


void Iommu::writeIohpmctr(unsigned index, uint64_t data, unsigned wordMask)
{
  if (capabilities_.fields.hpm == 0)
    return;
  assert(index >= 1 and index <= 31);
  uint64_t mask = 0;
  if (wordMask & 1) mask |= 0x00000000ffffffffULL;
  if (wordMask & 2) mask |= 0xffffffff00000000ULL;
  uint64_t &iohpmctr = iohpmctr_.at(index-1);
  iohpmctr = (iohpmctr & ~mask) | (data & mask);
}


void Iommu::writeIohpmevt(unsigned index, uint64_t data, unsigned wordMask)
{
  if (capabilities_.fields.hpm == 0)
    return;
  assert(index >= 1 and index <= 31);
  Iohpmevt new_iohpmevt { .value = data };
  Iohpmevt &iohpmevt = iohpmevt_.at(index-1);
  if (new_iohpmevt.fields.eventId > 8)
      new_iohpmevt.fields.eventId = iohpmevt.fields.eventId;
  if (wordMask & 1) iohpmevt.words[0] = new_iohpmevt.words[0];
  if (wordMask & 2) iohpmevt.words[1] = new_iohpmevt.words[1];
}


void Iommu::writeMsiAddr(unsigned index, uint64_t data, unsigned wordMask)
{
  if (capabilities_.fields.igs == unsigned(IgsMode::Wsi))
    return;
  MsiCfgTbl new_msi_cfg_tbl {};
  new_msi_cfg_tbl.regs.msi_addr = data & 0x00fffffffffffffc;
  if (wordMask & 1) msi_cfg_tbl_.at(index).words[0] = new_msi_cfg_tbl.words[0];
  if (wordMask & 2) msi_cfg_tbl_.at(index).words[1] = new_msi_cfg_tbl.words[1];
}


void Iommu::writeMsiData(unsigned index, uint64_t data)
{
  if (capabilities_.fields.igs == unsigned(IgsMode::Wsi))
    return;
  msi_cfg_tbl_.at(index).regs.msi_data = data;
}


void Iommu::writeMsiVecCtl(unsigned index, uint64_t data)
{
  if (capabilities_.fields.igs == unsigned(IgsMode::Wsi))
    return;
  msi_cfg_tbl_.at(index).regs.msi_vec_ctl = data & 1;
}


void
Iommu::signalInterrupt(unsigned vector)
{
  if (wiredInterrupts())
    {
      // Wired interrupts (WSI mode): Signal via callback to APLIC/Hart
      if (signalWiredInterrupt_)
        signalWiredInterrupt_(vector, true);  // Assert the interrupt
    }
  else
    {
      uint64_t addr = readMsiAddr(vector);
      uint32_t data = readMsiData(vector);
      uint32_t control = readMsiVecCtl(vector);

      if (control & 1)
        return;  // Interrupt is currently masked.

      bool bigEnd = faultQueueBigEnd();
      if (not memWrite(addr, sizeof(data), bigEnd, data))
        {
          FaultRecord record;
          record.cause = 273;
          record.iotval = addr;
          record.ttyp = unsigned(Ttype::None);
          writeFaultRecord(record); // TODO: prevent infinite loop?
        }
    }
}


void Iommu::updateIpsr(IpsrEvent event)
{
  if (cqcsr_.fields.cie and (
      cqcsr_.fields.fence_w_ip or
      cqcsr_.fields.cmd_ill or
      cqcsr_.fields.cmd_to or
      cqcsr_.fields.cqmf) and !ipsr_.fields.cip)
    {
      ipsr_.fields.cip = 1;
      signalInterrupt(icvec_.fields.civ);
    }

  if (fqcsr_.fields.fie and (
      fqcsr_.fields.fqof or
      fqcsr_.fields.fqmf or
      event == IpsrEvent::NewFault) and !ipsr_.fields.fip)
    {
      ipsr_.fields.fip = 1;
      signalInterrupt(icvec_.fields.fiv);
    }

  if (pqcsr_.fields.pie and (
      pqcsr_.fields.pqof or
      pqcsr_.fields.pqmf or
      event == IpsrEvent::NewPageRequest) and !ipsr_.fields.pip)
    {
      ipsr_.fields.pip = 1;
      signalInterrupt(icvec_.fields.piv);
    }

  // Check for HPM counter overflow interrupt
  if (event == IpsrEvent::HpmOverflow and !ipsr_.fields.pmip)
    {
      ipsr_.fields.pmip = 1;
      signalInterrupt(icvec_.fields.pmiv);
    }
}


void
Iommu::incrementIohpmcycles()
{
  // Only increment if HPM is supported
  if (capabilities_.fields.hpm == 0)
    return;

  // Check if counting is inhibited (bit 0 of iocountinh)
  if (iocountinh_.fields.cy)
    return;

  // Increment the counter (63-bit counter, bit 62:0)
  iohpmcycles_.fields.counter = (iohpmcycles_.fields.counter + 1) & 0x7FFFFFFFFFFFFFFFULL;

  // Check for overflow (wrapped to 0) and set OF bit if not already set
  if (iohpmcycles_.fields.counter == 0 and iohpmcycles_.fields.of == 0)
    {
      iohpmcycles_.fields.of = 1;
      updateIpsr(IpsrEvent::HpmOverflow);
    }
}


uint32_t
Iommu::readIocountovf() const
{
  // The iocountovf register is read-only and reflects the overflow status
  // of all performance monitoring counters.
  // Bit 0 (cy): reflects iohpmcycles.of
  // Bits 31:1 (hpm): reflect iohpmctr[1-31] overflow status via iohpmevt[0-30].of

  Iocountovf temp{};
  temp.value = 0;

  if (capabilities_.fields.hpm == 0)
    return 0;

  // Bit 0: iohpmcycles overflow flag
  temp.fields.cy = iohpmcycles_.fields.of;

  // Bits 31:1: iohpmctr[1-31] overflow flags from iohpmevt[0-30].of
  uint32_t hpm_ovf = 0;
  for (unsigned i = 0; i < 31; ++i)
    {
      if (iohpmevt_.at(i).fields.of)
        hpm_ovf |= (1U << i);
    }
  temp.fields.hpm = hpm_ovf;

  return temp.value;
}


void
Iommu::countEvent(HpmEventId eventId, bool pv, uint32_t pid,
                  bool pscv, uint32_t pscid, uint32_t did,
                  bool gscv, uint32_t gscid)
{
  // Only count if HPM is supported
  if (capabilities_.fields.hpm == 0)
    return;

  // Iterate through all 31 event counters (iohpmctr[1-31] via iohpmevt[0-30])
  for (unsigned i = 0; i < 31; ++i)
    {
      // Check if this counter is inhibited (bit i+1 of iocountinh.hpm)
      if ((iocountinh_.fields.hpm >> i) & 1)
        continue;

      const auto& evt = iohpmevt_.at(i);

      // Check if event ID matches
      if (evt.fields.eventId != static_cast<uint16_t>(eventId))
        continue;

      // Apply filtering based on IDT (Filter ID Type)
      // IDT=0: Filter by DID/PID (untranslated requests)
      // IDT=1: Filter by GSCID/PSCID (translated requests)
      bool idt = evt.fields.idt;

      // Select appropriate filter values based on IDT
      bool processIdValid = idt ? pscv : pv;
      uint32_t processIdValue = idt ? pscid : pid;
      bool deviceIdValid = idt ? gscv : true;  // DID always valid for untranslated
      uint32_t deviceIdValue = idt ? gscid : did;

      // Check process ID filter (PV_PSCV bit enables this filter)
      if (evt.fields.pv_pscv)
        {
          if (!processIdValid || evt.fields.pid_pscid != processIdValue)
            continue;
        }

      // Check device ID filter (DV_GSCV bit enables this filter)
      if (evt.fields.dv_gscv)
        {
          if (!deviceIdValid)
            continue;

          // Calculate mask for device ID matching with DMASK support
          uint32_t mask = 0xFFFFFF;  // Default: match all 24 bits
          if (evt.fields.dmask)
            {
              // DMASK=1: Use evt.fields.did_gscid to compute range mask
              // mask = ~((did_gscid + 1) ^ did_gscid)
              mask = evt.fields.did_gscid + 1;
              mask = ~(mask ^ evt.fields.did_gscid);
            }

          if ((evt.fields.did_gscid & mask) != (deviceIdValue & mask))
            continue;
        }

      // All filters passed - increment the counter
      iohpmctr_.at(i)++;

      // Check for overflow (wrapped to 0) and set OF bit if not already set
      if (iohpmctr_.at(i) == 0 and iohpmevt_.at(i).fields.of == 0)
        {
          iohpmevt_.at(i).fields.of = 1;
          updateIpsr(IpsrEvent::HpmOverflow);
        }
    }
}


bool
Iommu::loadDeviceContext(unsigned devId, DeviceContext& dc, unsigned& cause)
{
  deviceDirWalk_.clear();

  DdtCacheEntry* cacheEntry = findDdtCacheEntry(devId);
  if (cacheEntry)
  {
    dc = cacheEntry->deviceContext;
    return true;
  }

  cause = 0;
  bool extended = capabilities_.fields.msi_flat; // Extended or base format
  bool bigEnd = fctl_.fields.be;

  // 1. Identify device tree address and levels.
  uint64_t addr = ddtp_.fields.ppn * pageSize_;
  unsigned levels = ddtp_.levels();
  if (levels == 0)
    return false;
  unsigned ii = levels - 1;

  Devid idFields(devId);

  // 2. If i == 0 go to step 8.
  while (ii != 0)
    {
      // 3. Let ddte be the value of the eight bytes at address a + DDI[i] x 8. If
      //    accessing ddte violates a PMA or PMP check, then stop and report "DDT entry
      //    load access fault" (cause = 257).
      uint64_t ddteVal = 0;
      uint64_t ddteAddr = addr + idFields.ithDdi(ii, extended)*size_t(8);
      if (not memReadDouble(ddteAddr, bigEnd, ddteVal))
        {
          cause = 257;
          return false;
        }

      auto walkEntry = std::pair<uint64_t, uint64_t>(ddteAddr, ddteVal);
      deviceDirWalk_.push_back(walkEntry);

      // PMA and PMP should be covered by the memory read callback.

      // 4. If ddte access detects a data corruption (a.k.a. poisoned data), then stop and
      //    report "DDT data corruption" (cause = 268).
      if (false)
        {
          cause = 268;
          return false;
        }

      // 5. If ddte.V == 0, stop and report "DDT entry not valid" (cause = 258).
      Ddte ddte(ddteVal);
      if (ddte.bits_.v_ == 0)
      {
        cause = 258;
        return false;
      }

      // 6. If any bits or encoding that are reserved for future standard use are set
      //    within ddte, stop and report "DDT entry misconfigured" (cause = 259).
      if (ddte.bits_.reserved_ != 0 or ddte.bits_.reserved2_ != 0)
        {
          cause = 259;
          return false;
        }

      // 7. Let i = i - 1 and let a = ddte.PPN x pageSize. Go to step 2.
      --ii;
      addr = ddte.bits_.ppn_ * pageSize_;
    }

  // 8. Let DC be the value of DC_SIZE bytes at address a + DDI[0] * DC_SIZE. If
  //    capabilities.MSI_FLAT is 1 then DC_SIZE is 64-bytes else it is 32-bytes. If
  //    accessing DC violates a PMA or PMP check, then stop and report "DDT entry load
  //    access fault" (cause = 257). If DC access detects a data corruption
  //    (a.k.a. poisoned data), then stop and report "DDT data corruption" (cause = 268).
  uint64_t dcSize = extended? 64 : 32;
  uint64_t dcAddr = addr + idFields.ithDdi(0, extended) * dcSize;
  unsigned dwordCount = dcSize / 8;  // Double word count.
  std::vector<uint64_t> dcd(dwordCount);  // Device context data.
  for (size_t i = 0; i < dwordCount; ++i)
    if (not memReadDouble(dcAddr + i*8, bigEnd, dcd.at(i)))
      {
        cause = 257;
        return false;
      }

  // Check for poisoned data. This would require a test-bench API.

  if (dwordCount == 4)
    dc = DeviceContext(dcd.at(0), dcd.at(1), dcd.at(2), dcd.at(3));
  else if (dwordCount == 8)
    dc = DeviceContext(dcd.at(0), dcd.at(1), dcd.at(2), dcd.at(3),
                       dcd.at(4), dcd.at(5), dcd.at(6), dcd.at(7));
  else
    assert(0);

  // 9. If DC.tc.V == 0, stop and report "DDT entry not valid" (cause = 258).
  if (not dc.valid())
    {
      cause = 258;
      return false;
    }

  // 10. If the DC is misconfigured as determined by rules outlined in Section 2.1.4 then
  //     stop and report "DDT entry misconfigured" (cause = 259).
  if (misconfiguredDc(dc))
    {
      cause = 259;
      return false;
    }

  // 11. The device-context has been successfully located.
  updateDdtCache(devId, dc);
  return true;
}


bool
Iommu::loadProcessContext(const DeviceContext& dc, uint32_t pid,
                          ProcessContext& pc, unsigned& cause)
{
  // Call the overloaded version with deviceId = 0 (unknown)
  return loadProcessContext(dc, 0, pid, pc, cause);
}

bool
Iommu::loadProcessContext(const DeviceContext& dc, unsigned devId, uint32_t pid,
                          ProcessContext& pc, unsigned& cause)
{
  cause = 0;
  bool bigEnd = dc.sbe();
  Procid procid(pid);

  processDirWalk_.clear();

  PdtCacheEntry* cacheEntry = findPdtCacheEntry(devId, pid);
  if (cacheEntry)
  {
    pc = cacheEntry->processContext;
    return true;
  }

  // 1. Let a be pdtp.PPN x pageSize and let i = LEVELS-1. When pdtp.MODE is PD20, LEVELS is
  //    three. When pdtp.MODE is PD17, LEVELS is two. When pdtp.MODE is PD8, LEVELS is
  //    one.
  uint64_t aa = dc.pdtpPpn() * pageSize_;
  unsigned levels = dc.processTableLevels();
  if (levels == 0)
    return false;
  unsigned ii = levels - 1;

  while (true)
    {
      // 2. If DC.iohgatp.mode != Bare, then A (here aa) is a GPA. Invoke the process to
      //    translate A to an SPA as an implicit memory access. If faults occur during
      //    second-stage address translation of a then stop and report the fault detected
      //    by the second-stage address translation process. The translated A is used in
      //    subsequent steps.
      if (dc.iohgatpMode() != IohgatpMode::Bare)
        {
          // FIX double check that the privilege mode is User. Should it be the mode of
          // the initiating IommuRequest?
          uint64_t pa = 0;
          if (not stage2Translate(dc.iohgatp(), PrivilegeMode::User,  true, false, false,
                                  aa, dc.gade(), pa, cause))
            return false;
          aa = pa;
        }

      // 3. If i == 0 go to step 9.
      if (ii == 0)
        break;

      // 4. Let pdte be the value of the eight bytes at address a + PDI[i] x 8. If
      //    accessing pdte violates a PMA or PMP check, then stop and report "PDT entry
      //    load access fault" (cause = 265).
      uint64_t pdte = 0;
      uint64_t pdteAddr = aa + procid.ithPdi(ii) * uint64_t(8);
      if (not memReadDouble(pdteAddr, bigEnd, pdte))
        {
          cause = 265;
          return false;
        }

      auto walkEntry = std::pair<uint64_t, uint64_t>(pdteAddr, pdte);
      processDirWalk_.emplace_back(walkEntry);

      // 5. If pdte access detects a data corruption (a.k.a. poisoned data), then stop and
      //    report "PDT data corruption" (cause = 269).

      // 6. If pdte.V == 0, stop and report "PDT entry not valid" (cause = 266).
      if (Pdte{pdte}.bits_.v_ == 0)
        {
          cause = 266;
          return false;
        }

      // 7. If any bits or encoding that are reserved for future standard use are set
      //    within pdte, stop and report "PDT entry misconfigured" (cause = 267).
      uint64_t reserved = pdte & 0xff00'0000'0000'03feLL;
      if (reserved != 0)
        {
          cause = 267;
          return false;
        }

      // 8. Let i = i - 1 and let a = pdte.PPN x pageSize. Go to step 2.
      --ii;
      aa = Pdte{pdte}.bits_.ppn_ * pageSize_;
    }

  // 9. Let PC be the value of the 16-bytes at address a + PDI[0] x 16. If accessing PC
  //    violates a PMA or PMP check, then stop and report "PDT entry load access fault"
  //    (cause = 265). If PC access detects a data corruption (a.k.a. poisoned data), then
  //    stop and report "PDT data corruption" (cause = 269).
  uint64_t pca = aa + procid.ithPdi(0) * uint64_t(16);
  if (not readProcessContext(dc, pca, pc))
    {
      cause = 265;
      return false;
    }

  // Check for poisoned data. This would require a test-bench API.

  // 10. If PC.ta.V == 0, stop and report "PDT entry not valid" (cause = 266).
  if (not pc.valid())
    {
      cause = 266;
      return false;
    }

  // 11. If the PC is misconfigured as determined by rules outlined in Section 2.2.4 then
  //     stop and report "PDT entry misconfigured" (cause = 267).
  if (misconfiguredPc(pc, dc.sxl()))
    return false;

  // 12. The Process-context has been successfully located.
  updatePdtCache(devId, pid, pc);
  return true;
}


bool
Iommu::misconfiguredDc(const DeviceContext& dc) const
{
  bool extended = capabilities_.fields.msi_flat; // Extended or base format

  // 1. If any bits or encodings that are reserved for future standard use are set.
  if (dc.nonZeroReservedBits(extended, capabilities_.fields.qosid)){
    return true;
  }


  // 2. capabilities.ATS is 0 and DC.tc.EN_ATS, or DC.tc.EN_PRI, or DC.tc.PRPR is 1
  if (capabilities_.fields.ats == 0 and (dc.ats() or dc.pri() or dc.prpr())){
    return true;
  }


  // 3. DC.tc.EN_ATS is 0 and DC.tc.T2GPA is 1
  // 4. DC.tc.EN_ATS is 0 and DC.tc.EN_PRI is 1
  if (not dc.ats() and (dc.t2gpa() or dc.pri())){
    return true;
  }

  // 5. DC.tc.EN_PRI is 0 and DC.tc.PRPR is 1
  if (not dc.pri() and dc.prpr()){
    return true;
  }

  // 6. capabilities.T2GPA is 0 and DC.tc.T2GPA is 1
  if (not capabilities_.fields.t2gpa and dc.t2gpa()){
    return true;
  }

  // 7. DC.tc.T2GPA is 1 and DC.iohgatp.MODE is Bare
  if (dc.t2gpa() and dc.iohgatpMode() == IohgatpMode::Bare){
    return true;
  }

  // 8. DC.tc.PDTV is 1 and DC.fsc.pdtp.MODE is not a supported mode
  //    a. capabilities.PD20 is 0 and DC.fsc.pdtp.MODE is PD20
  //    b. capabilities.PD17 is 0 and DC.fsc.pdtp.MODE is PD17
  //    c. capabilities.PD8 is 0 and DC.fsc.pdtp.MODE is PD8
  if (dc.pdtv())
    {
      auto mode = dc.pdtpMode();
      if (mode != PdtpMode::Bare and mode != PdtpMode::Pd8 and mode != PdtpMode::Pd17 and
          mode != PdtpMode::Pd20){
            return true;
          }
      if (not capabilities_.fields.pd20 and mode == PdtpMode::Pd20){
        return true;
      }
      if (not capabilities_.fields.pd17 and mode == PdtpMode::Pd17){
        return true;
      }
      if (not capabilities_.fields.pd8 and mode == PdtpMode::Pd8){
        return true;
      }
    }

  // 9. DC.tc.PDTV is 0 and DC.fsc.iosatp.MODE encoding is not a valid encoding as
  //    determined by Table 3
  if (not dc.pdtv())
    {
      auto mode = dc.iosatpMode();

      if (dc.sxl())
        {
          if (mode != IosatpMode::Bare and mode != IosatpMode::Sv32){
            return true;
          }
        }
      else
        if (mode != IosatpMode::Bare and mode != IosatpMode::Sv39 and
            mode != IosatpMode::Sv48 and mode != IosatpMode::Sv57){
        return true;
      }
    }

  // 10. DC.tc.PDTV is 0 and DC.tc.SXL is 0 DC.fsc.iosatp.MODE is not one of the supported
  //     modes
  //     a. capabilities.Sv39 is 0 and DC.fsc.iosatp.MODE is Sv39
  //     b. capabilities.Sv48 is 0 and DC.fsc.iosatp.MODE is Sv48
  //     c. capabilities.Sv57 is 0 and DC.fsc.iosatp.MODE is Sv57
  if (not dc.pdtv() and not dc.sxl())
    {
      auto mode = dc.iosatpMode();
      if (not capabilities_.fields.sv39 and mode == IosatpMode::Sv39){
        return true;
      }

      if (not capabilities_.fields.sv48 and mode == IosatpMode::Sv48){
        return true;
      }
      if (not capabilities_.fields.sv57 and mode == IosatpMode::Sv57){
        return true;
      }
    }

  // 11. DC.tc.PDTV is 0 and DC.tc.SXL is 1 DC.fsc.iosatp.MODE is not one of the supported
  //     modes
  //     a. capabilities.Sv32 is 0 and DC.fsc.iosatp.MODE is Sv32
  if (not dc.pdtv() and dc.sxl())
    {
      auto mode = dc.iosatpMode();

      if (not capabilities_.fields.sv32 and mode == IosatpMode::Sv32){
        return true;
      }
    }

  // 12. DC.tc.PDTV is 0 and DC.tc.DPE is 1
  if (not dc.pdtv() and dc.dpe()){
    return true;
  }

  // 13. DC.iohgatp.MODE encoding is not a valid encoding as determined by Table 2
  auto gmode = dc.iohgatpMode();

  // Check valid IOHGATP modes based on fctl.GXL
  if (fctl_.fields.gxl)
    {
      // When GXL=1, only Bare and Sv32x4 are valid
      if (gmode != IohgatpMode::Bare && gmode != IohgatpMode::Sv32x4){
        return true;
      }
    }
  else
    {
      // When GXL=0, only Bare, Sv39x4, Sv48x4, and Sv57x4 are valid
      if (gmode != IohgatpMode::Bare && gmode != IohgatpMode::Sv39x4 &&
          gmode != IohgatpMode::Sv48x4 && gmode != IohgatpMode::Sv57x4){
        return true;
      }
    }

  // 14. fctl.GXL is 0 and DC.iohgatp.MODE is not a supported mode
  //     a. capabilities.Sv39x4 is 0 and DC.iohgatp.MODE is Sv39x4
  //     b. capabilities.Sv48x4 is 0 and DC.iohgatp.MODE is Sv48x4
  //     c. capabilities.Sv57x4 is 0 and DC.iohgatp.MODE is Sv57x4
  if (not fctl_.fields.gxl)
    {
      if (not capabilities_.fields.sv39x4 and gmode == IohgatpMode::Sv39x4){
        return true;
      }
      if (not capabilities_.fields.sv48x4 and gmode == IohgatpMode::Sv48x4){
        return true;
      }
      if (not capabilities_.fields.sv57x4 and gmode == IohgatpMode::Sv57x4){
        return true;
      }
    }

  // 15. fctl.GXL is 1 and DC.iohgatp.MODE is not a supported mode
  //     a. capabilities.Sv32x4 is 0 and DC.iohgatp.MODE is Sv32x4
  if (fctl_.fields.gxl)
    if (not capabilities_.fields.sv32x4 and gmode == IohgatpMode::Sv32x4){
      return true;
    }

  // 16. capabilities.MSI_FLAT is 1 and DC.msiptp.MODE is not Off and not Flat
  bool msiFlat = capabilities_.fields.msi_flat;
  auto msiMode = dc.msiMode();
  if (msiFlat)
    {
      if (msiMode != MsiptpMode::Off and msiMode != MsiptpMode::Flat){
        return true;
    }

  // 17. DC.iohgatp.MODE is not Bare and the root page table determined by DC.iohgatp.PPN
  // is not aligned to a 16-KiB boundary.
  if (gmode != IohgatpMode::Bare and (dc.iohgatpPpn() & 0x3) != 0)
    return true;
  }

  // 18. capabilities.AMO_HWAD is 0 and DC.tc.SADE or DC.tc.GADE is 1
  if (not capabilities_.fields.amo_hwad and (dc.sade() or dc.gade())){
    return true;
  }

  // 19. capabilities.END is 0 and fctl.BE != DC.tc.SBE
  if (not capabilities_.fields.end and fctl_.fields.be != dc.sbe()){
    return true;
  }

  // 20. DC.tc.SXL value is not a legal value. If fctl.GXL is 1, then DC.tc.SXL must be
  // 1. If fctl.GXL is 0 and is writable, then DC.tc.SXL may be 0 or 1. If fctl.GXL is 0
  // and is not writable then DC.tc.SXL must be 0.
  if (fctl_.fields.gxl and not dc.sxl()){
    return true;
  }
  if (not fctl_.fields.gxl) {
    if (not gxlWritable_)  // fctl.GXL not writable
      if (dc.sxl() != 0){
        return true;
      }
  }

  // 21. DC.tc.SBE value is not a legal value. If fctl.BE is writable then DC.tc.SBE may
  // be 0 or 1. If fctl.BE is not writable then DC.tc.SBE must be the same as fctl.BE.
  if (not beWritable_)   // fctl.BE not writable
    if (dc.sbe() != fctl_.fields.be)
      return true;

  // 22. capabilities.QOSID is 1 and DC.ta.RCID or DC.ta.MCID values are wider
  // than that supported by the IOMMU.
  if ( (capabilities_.fields.qosid == 1) &&
       ((dc.transAttrib().bits_.rcid_ >> rcidWidth_ != 0) ||
        (dc.transAttrib().bits_.mcid_ >> mcidWidth_ != 0)) )
    return true;

  // When DC.iohgatp.MODE is Bare, DC.msiptp.MODE must be set to Off by
  // software. All other settings are reserved. Implementations are recommended
  // to stop and report "DDT entry misconfigured" (cause = 259) if a reserved
  // setting is detected.
  if ( (gmode == IohgatpMode::Bare) && (msiMode != MsiptpMode::Off) )
    return true;

  return false;
}


bool
Iommu::misconfiguredPc(const ProcessContext& pc, bool sxl) const
{
  // 1. If any bits or encoding that are reserved for future standard use are set
  if (pc.nonZeroReservedBits())
    return true;

  // 2. PC.fsc.MODE encoding is not valid as determined by Table 3
  auto mode = pc.iosatpMode();
  if (sxl)
    {
      if (mode != IosatpMode::Bare and mode != IosatpMode::Sv39)
        return true;
    }
  else
    if (mode != IosatpMode::Bare and mode != IosatpMode::Sv39 and
        mode != IosatpMode::Sv48 and mode != IosatpMode::Sv57)
      return true;


  // 3. DC.tc.SXL is 0 and PC.fsc.MODE is not one of the supported modes
  //    a. capabilities.Sv39 is 0 and PC.fsc.MODE is Sv39
  //    b. capabilities.Sv48 is 0 and PC.fsc.MODE is Sv48
  //    c. capabilities.Sv57 is 0 and PC.fsc.MODE is Sv57
  if (not sxl)
    {
      if (not capabilities_.fields.sv39 and mode == IosatpMode::Sv39)
        return true;
      if (not capabilities_.fields.sv48 and mode == IosatpMode::Sv48)
        return true;
      if (not capabilities_.fields.sv57 and mode == IosatpMode::Sv57)
        return true;
    }

  // 4. DC.tc.SXL is 1 and PC.fsc.MODE is not one of the supported modes
  //    a. capabilities.Sv32 is 0 and PC.fsc.MODE is Sv32
  if (sxl)
    if (not capabilities_.fields.sv32 and mode == IosatpMode::Sv32)
      return true;

  return false;
}


bool
Iommu::translate(const IommuRequest& req, uint64_t& pa, unsigned& cause)
{
  cause = 0;

  bool repFault = true;  // Should fault be reported?

  if (translate_(req, pa, cause, repFault))
    return true;

  if (repFault)
    {
      FaultRecord record;
      record.cause = cause;
      record.ttyp = unsigned(req.type);

      switch (req.type)
        {
          case Ttype::None:
            // Spec says that the values of iotval and iotval2 are "as defined by the CAUSE".
            // Spec does not say how the CAUSE defineds the values.
            assert(0);
          case Ttype::Reserved:
            assert(0);
          case Ttype::PcieAts:
            assert(0);
          case Ttype::PcieMessage:
            std::cerr << "PCIE message requests not yet supported\n";
            assert(0);
          case Ttype::UntransExec: case Ttype::UntransRead: case Ttype::UntransWrite:
          case Ttype::TransExec:   case Ttype::TransRead:   case Ttype::TransWrite:
            // Section 4.2
            record.did = req.devId;
            record.pv = req.hasProcId;   // Process id valid.
            if (record.pv)
              {
                record.pid = req.procId;
                record.priv = req.privMode == PrivilegeMode::Supervisor? 1 : 0;
              }
            record.iotval = req.iova;
            if (cause == 20 or cause == 21 or cause == 23)   // Guest page fault
              {
                uint64_t gpa = 0;
                bool implicit = false, write = false;
                stage2TrapInfo_(gpa, implicit, write);
                uint64_t iotval2 = (gpa >> 2) << 2;  // Clear least sig 2 bits.
                if (implicit)
                  {
                    iotval2 |= 1;     // Set bit 0
                    if (write)
                      iotval2 |= 2;   // Set bit 1
                  }
                record.iotval2 = iotval2;
              }
            break;
          default: assert(0);
        }

      writeFaultRecord(record);
    }

  return false;
}


bool
Iommu::readForDevice(const IommuRequest& req, uint64_t& data, unsigned& cause)
{
  deviceDirWalk_.clear();
  processDirWalk_.clear();

  cause = 0;
  if (not req.isRead())
    return false;  // Request misconfigured.

  uint64_t pa = 0;
  if (not translate(req,  pa, cause))
    return false;

  // FIX Should we consider device endianness?
  return memRead(pa, req.size, data);
}


bool
Iommu::writeForDevice(const IommuRequest& req, uint64_t data, unsigned& cause)
{
  deviceDirWalk_.clear();
  processDirWalk_.clear();

  cause = 0;
  if (not req.isWrite())
    return false;  // Request misconfigured.

  uint64_t pa = 0;
  if (not translate(req,  pa, cause))
    return false;

  // FIX Should we consider device endianness?
  return memWrite(pa, req.size, data);
}


bool
Iommu::translate_(const IommuRequest& req, uint64_t& pa, unsigned& cause, bool& repFault)
{
  deviceDirWalk_.clear();
  processDirWalk_.clear();

  cause = 0;

  // By default all faults are reported (assume DTF is 0 until we determine DTF).
  // Section 4.2. of spec.
  repFault = true;

  unsigned processId = req.procId;

  // Count request type event (Translated vs Untranslated)
  if (req.isTranslated())
    countEvent(HpmEventId::TranslatedReq, req.hasProcId, req.procId, false, 0, req.devId, false, 0);
  else if (!req.isAts())
    countEvent(HpmEventId::UntranslatedReq, req.hasProcId, req.procId, false, 0, req.devId, false, 0);

  // 1.  If ddtp.iommu_mode == Off then stop and report "All inbound
  // transactions disallowed" (cause = 256).
  if (ddtp_.fields.iommu_mode == Ddtp::Mode::Off)
    {
      cause = 256;
      return false;
    }

  // 2. If ddtp.iommu_mode == Bare and any of the following conditions hold then stop and
  //    report "Transaction type disallowed" (cause = 260); else go to step 20 with
  //    translated address same as the IOVA.
  //    a. Transaction type is a Translated request (read, write/AMO, read-for-execute) or
  //       is a PCIe ATS Translation request.
  if (ddtp_.fields.iommu_mode == Ddtp::Mode::Bare)
    {
      if (req.isTranslated() or req.isAts())
        {
          cause = 260;
          return false;
        }
      pa = req.iova;
      return true;
    }

  // 3. If capabilities.MSI_FLAT is 0 then the IOMMU uses base-format device context. Let
  //    DDI[0] be device_id[6:0], DDI[1] be device_id[15:7], and DDI[2] be
  //    device_id[23:16].
  bool extended = capabilities_.fields.msi_flat; // Extended or base format

  // 4. If capabilities.MSI_FLAT is 1 then the IOMMU uses extended-format device
  //    context. Let DDI[0] be device_id[5:0], DDI[1] be device_id[14:6], and DDI[2] be
  //    device_id[23:15].
  Devid devid(req.devId);
  unsigned ddi1 = devid.ithDdi(1, extended);
  unsigned ddi2 = devid.ithDdi(2, extended);

  // 5. If the device_id is wider than that supported by the IOMMU mode, as determined by
  //    the following checks then stop and report "Transaction type disallowed" (cause =
  //    260).
  //    a. ddtp.iommu_mode is 2LVL and DDI[2] is not 0
  //    b. ddtp.iommu_mode is 1LVL and either DDI[2] is not 0 or DDI[1] is not 0
  Ddtp::Mode ddtpMode = ddtp_.fields.iommu_mode;
  if ((ddtpMode == Ddtp::Mode::Level2 and ddi2 != 0) or
      (ddtpMode == Ddtp::Mode::Level1 and (ddi2 != 0 or ddi1 != 0)))
    {
      cause = 260;
      return false;
    }
  // 6. Use device_id to then locate the device-context (DC) as specified in Section
  //    2.3.1.
  DeviceContext dc;
  if (not loadDeviceContext(req.devId, dc, cause))
    return false;

  // Count DDT walk event (only if not from cache)
  if (!deviceDirWalk_.empty())
    {
      // Extract GSCID and PSCID from device context for event filtering
      bool gscv = (dc.iohgatpMode() != IohgatpMode::Bare);
      uint32_t gscid = dc.iohgatpGscid();
      bool pscv = (dc.pscid() != 0);  // PSCID valid if non-zero in device context
      uint32_t pscid = dc.pscid();
      countEvent(HpmEventId::DdtWalk, req.hasProcId, req.procId, pscv, pscid, req.devId, gscv, gscid);
    }

  bool dtf = dc.dtf();  // Disable translation fault reporting.
  // 7. If any of the following conditions hold then stop and report "Transaction type
  //    disallowed" (cause = 260).
  //    a. Transaction type is a Translated request (read, write/AMO, read-for-execute) or
  //       is a PCIe ATS Translation request and DC.tc.EN_ATS is 0.
  //    b. Transaction has a valid process_id and DC.tc.PDTV is 0.
  //    c. Transaction has a valid process_id and DC.tc.PDTV is 1 and the process_id is
  //       wider than that supported by pdtp.MODE.
  //    d. Transaction type is not supported by the IOMMU.
  if (((req.isTranslated() or req.isAts()) and not dc.ats()) or  // a
      (req.hasProcId and not dc.pdtv()))                 // b
    {
      repFault = not dtf;   // Sec 4.2, table 11.
      cause = 260;
      return false;
    }
  if (req.hasProcId and dc.pdtv())                       // c
    {
      Procid procid(req.procId);
      unsigned pdi1 = procid.ithPdi(1);
      unsigned pdi2 = procid.ithPdi(2);
      Pdtp pdtp(dc.pdtp());
      PdtpMode pdtpMode = pdtp.mode();
      if ((pdtpMode == PdtpMode::Pd17 and pdi2 != 0) or
          (pdtpMode == PdtpMode::Pd8 and (pdi2 != 0 or pdi1 != 0)))
        {
          repFault = not dtf;   // Sec 4.2, table 11.
          cause = 260;
          return false;
        }
    }
  // 8. If request is a Translated request and DC.tc.T2GPA is 0 then the translation
  //    process is complete. Go to step 20.
  if (req.isTranslated() and not dc.t2gpa())
    {
      pa = req.iova;  // Not explicitly in the spec. Implied.
      return true;
    }

  unsigned pscid = 0;   // dc.pscid();
  bool sum = false;  // Supervisor has access to user pages.

  // 9. If request is a Translated request and DC.tc.T2GPA is 1 then the IOVA is a GPA. Go
  //    to step 17 with following page table information:
  //    a. Let A be the IOVA (the IOVA is a GPA).
  //    b. Let iosatp.MODE be Bare
  //       i. The PSCID value is not used when first-stage is Bare.
  //    c. Let iohgatp be the value in the DC.iohgatp field
  uint64_t iohgatp = dc.iohgatp();
  uint64_t iosatp = not dc.pdtv() ? dc.iosatp() : uint64_t(IosatpMode::Bare) << 60;
  if (req.isTranslated() and dc.t2gpa())
    pscid = 0;
  else if (not dc.pdtv())
    {
      // 10. If DC.tc.PDTV is set to 0 then go to step 17 with the following page table
      //     information:
      //     a. Let iosatp.MODE be the value in the DC.fsc.MODE field
      //     b. Let iosatp.PPN be the value in the DC.fsc.PPN field
      //     c. Let PSCID be the value in the DC.ta.PSCID field
      //     d. Let iohgatp be the value in the DC.iohgatp field
      pscid = dc.pscid();
    }
  else
    {
      // 11. If DPE is 1 and there is no process_id associated with the transaction then
      //     let process_id be the default value of 0.
      if (dc.dpe() and not req.hasProcId)
        processId = 0;

      // 12. If DPE is 0 and there is no process_id associated with the transaction then
      //     then go to step 17 with the following page table information:
      //     a. Let iosatp.MODE be Bare
      //        i. The PSCID value is not used when first-stage is Bare.
      //     b. Let iohgatp be the value in the DC.iohgatp field
      if (not dc.dpe() and not req.hasProcId)
        {
          iosatp = uint64_t(IosatpMode::Bare) << 60;
          pscid = 0;
        }
      else
        {
          // 13. If DC.fsc.pdtp.MODE = Bare then go to step 17 with the following page
          //     table information:
          //     a. Let iosatp.MODE be Bare
          //        i. The PSCID value is not used when first-stage is Bare.
          //     b. Let iohgatp be value in DC.iohgatp field
          if (dc.pdtpMode() == PdtpMode::Bare)
            {
              iosatp = uint64_t(IosatpMode::Bare) << 60;
              pscid = 0;
            }
          else
            {
              // 14. Locate the process-context (PC) as specified in Section 2.3.2.
              ProcessContext pc;
              if (not loadProcessContext(dc, req.devId, processId, pc, cause))
                {
                  // All causes produced by load-process-context are subject to DC.DTF.
                  repFault = not dc.dtf();  // Sec 4.2, table 11.
                  return false;
                }

              // Count PDT walk event (only if not from cache)
              if (!processDirWalk_.empty())
                {
                  // Extract GSCID from device context and PSCID from process context for event filtering
                  bool gscv = (dc.iohgatpMode() != IohgatpMode::Bare);
                  uint32_t gscid = dc.iohgatpGscid();
                  bool pscv = pc.valid();  // PSCID valid if process context is valid
                  uint32_t pscid = pc.pscid();
                  countEvent(HpmEventId::PdtWalk, req.hasProcId, req.procId, pscv, pscid, req.devId, gscv, gscid);
                }

              // 15. if any of the following conditions hold then stop and report
              //     "Transaction type disallowed" (cause = 260).
              //     a. The transaction requests supervisor privilege but PC.ta.ENS is not
              //        set.
              if (req.privMode == PrivilegeMode::Supervisor and not pc.ens())
                {
                  repFault = not dtf;   // Sec 4.2, table 11.
                  cause = 260;
                  return false;
                }

              // 16. Go to step 17 with the following page table information:
              //     a. Let iosatp.MODE be the value in the PC.fsc.MODE field
              //     b. Let iosatp.PPN be the value in the PC.fsc.PPN field
              //     c. Let PSCID be the value in the PC.ta.PSCID field
              //     d. Let iohgatp be the value in the DC.iohgatp field
              iosatp = pc.fsc();
              pscid = pc.pscid();
              sum = pc.sum();
            }
        }
    }
  // 17. Use the process specified in Section "Two-Stage Address Translation" of the
  //     RISC-V Privileged specification [3] to determine the GPA accessed by the
  //     transaction. If a fault is detected by the first stage address translation
  //     process then stop and report the fault. If the translation process is completed
  //     successfully then let A be the translated GPA.
  uint64_t gpa = req.iova;
  if (not stage1Translate(iosatp, iohgatp, req.privMode, pscid, req.isRead(), req.isWrite(),
                          req.isExec(), sum, req.iova, dc.gade(), dc.sade(), gpa, cause))
    {
      repFault = not dtf;   // Sec 4.2, table 11. Cause range is 1 to 23.
      return false;
    }

  // 18. If MSI address translations using MSI page tables is enabled (i.e.,
  //     DC.msiptp.MODE != Off) then the MSI address translation process specified in
  //     Section 2.3.3 is invoked. If the GPA A is not determined to be the address of a
  //     virtual interrupt file then the process continues at step 19. If a fault is
  //     detected by the MSI address translation process then stop and report the fault
  //     else the process continues at step 20.s
  if (extended and dc.msiMode() != MsiptpMode::Off)
    {
      bool isMrif = false;
      uint64_t mrif = 0;
      uint64_t nppn = 0;
      unsigned nid = 0;
      if (msiTranslate(dc, req, gpa, pa, isMrif, mrif, nppn, nid, cause))
        return true;  // A is address of virtual file and MSI translation successful
      if (cause != 0)
        {
          // All causes produced by MSI translate are subject to DC.DTF.
          repFault = not dtf;  // Sec 4.2, table 11.
          return false;  // A is address of virtual file and MSI translation failed
        }
    }

  // 19. Use the second-stage address translation process specified in Section "Two-Stage
  //     Address Translation" of the RISC-V Privileged specification [3] to translate the
  //     GPA A to determine the SPA accessed by the transaction. If a fault is detected by
  //     the address translation process then stop and report the fault.
  if (not stage2Translate(iohgatp, req.privMode, req.isRead(), req.isWrite(),
                          req.isExec(), gpa, dc.gade(), pa, cause))
    {
      repFault = not dtf;   // Sec 4.2, table 11. Cause range is 1 to 23.
      return false;
    }

  // 20. Translation process is complete
  return true;
}


bool
Iommu::msiTranslate(const DeviceContext& dc, const IommuRequest& req,
                    uint64_t gpa, uint64_t& pa, bool& isMrif, uint64_t& mrif,
                    uint64_t& nnpn, unsigned& nid, unsigned& cause)
{
  if (not isDcExtended())
    return false;

  cause = 0;

  bool bigEnd = fctl_.fields.be;

  // 1. Let A be the GPA
  uint64_t aa = gpa;

  // 2. Let DC be the device-context located using the device_id of the device using the
  //     process outlined in Section 2.3.1.

  // Step 2 is already done by the caller.

  // 3. Determine if the address A is an access to a virtual interrupt file as specified
  //    in Section 3.1.3.6.
  if (not dc.isMsiAddress(gpa))
    return false;   // MSI translation does not apply.

  // 4. If the address is not determined to be that of a virtual interrupt file then stop
  //    this process and instead use the regular translation data structures to do the
  //    address translation.

  // Step 4 will be handled by the caller when they see cause == 0.

  // 5. Extract an interrupt file number I from A as I = extract(A >> 12,
  //    DC.msi_addr_mask). The bit extract function extract(x, y) discards all bits from x
  //    whose matching bits in the same positions in the mask y are zeros, and packs the
  //    remaining bits from x contiguously at the least- significant end of the result,
  //    keeping the same bit order as x and filling any other bits at the most-significant
  //    end of the result with zeros. For example, if the bits of x and y are:
  //      x = abcdefgh
  //      y = 10100110
  //    then the value of extract(x, y) has bits 0000acfg.
  uint64_t ii = DeviceContext::extractMsiBits(aa >> 12, dc.msiMask());

  // 6. Let m be (DC.msiptp.PPN x pageSize).
  uint64_t mm = dc.msiPpn() * pageSize_;

  // 7. Let msipte be the value of sixteen bytes at address (m | (I x 16)).  If accessing
  //    msipte violates a PMA or PMP check, then stop and report "MSI PTE load access
  //    fault" (cause = 261).
  uint64_t pteAddr = mm | (ii * 16);
  uint64_t pte0 = 0, pte1 = 0;
  if (not memReadDouble(pteAddr, bigEnd, pte0) or not memReadDouble(pteAddr+8, bigEnd, pte1))
    {
      cause = 261;
      return false;
    }

  // 8. If msipte access detects a data corruption (a.k.a. poisoned data), then stop and
  //    report "MSI PT data corruption" (cause = 270).
  MsiPte0 msiPte0(pte0);

  // 9. If msipte.V == 0, then stop and report "MSI PTE not valid" (cause = 262).
  if (not msiPte0.bits_.v_)
    {
      cause = 262;
      return false;
    }

  // 10. If msipte.C == 1, then further processing to interpret the PTE is implementation
  //     defined.
  if (msiPte0.bits_.c_)
    {
      cause = 263;
      return false;
    }

  // 11. If msipte.C == 0 then the process is outlined in subsequent steps.

  // 12. If msipte.M == 0 or msipte.M == 2, then stop and report "MSI PTE misconfigured"
  //     (cause = 263).
  if (msiPte0.bits_.m_ == 0 or msiPte0.bits_.m_ == 2)
    {
      cause = 263;
      return false;
    }

  // 13. If msipte.M == 3 the PTE is in basic translate mode and the translation process
  //     is as follows:
  //     a. If any bits or encoding that are reserved for future standard use are set
  //        within msipte, stop and report "MSI PTE misconfigured" (cause = 263).
  //     b. Compute the translated address as msipte.PPN << 12 | A[11:0].
  if (msiPte0.bits_.m_ == 3)
    {
      if (msiPte0.bits_.rsrv0_ or msiPte0.bits_.rsrv1_ or pte1)
        {
          cause = 263;
          return false;
        }
      pa = (msiPte0.bits_.ppn_ << 12) | (aa & 0xfff);
    }

  // 14. If msipte.M == 1 the PTE is in MRIF mode and the translation process is as
  //     follows:
  //     a. If capabilities.MSI_MRIF == 0, stop and report "MSI PTE misconfigured" (cause
  //        = 263).
  //     b. If any bits or encoding that are reserved for future standard use are set
  //        within msipte, stop and report "MSI PTE misconfigured" (cause = 263).
  //     c. The address of the destination MRIF is msipte.MRIF_Address[55:9] * 512.
  //     d. The destination address of the notice MSI is msipte.NPPN << 12.  e. Let NID be
  //        (msipte.N10 << 10) | msipte.N[9:0]. The data value for notice MSI is the
  //        11-bit NID value zero-extended to 32-bits.
  //     e. Let NID be (msipte.N10 << 10) | msipte.N[9:0]. The data value for notice MSI
  //        is the 11-bit NID value zero-extended to 32-bits.
  if (msiPte0.bits_.m_ == 1)
    {
      if (capabilities_.fields.msi_mrif == 0)    // a.
        {
          cause = 263;
          return false;
        }

      MsiMrifPte0 mpte0(pte0);    // b.
      MsiMrifPte1 mpte1(pte1);
      if (mpte0.bits_.reserved0_ or mpte0.bits_.reserved1_ or
          mpte1.bits_.reserved0_ or mpte1.bits_.reserved1_)
        {
          cause = 263;
          return false;
        }

      mrif = mpte0.bits_.addr_ * 512;  // c.
      nnpn = mpte1.bits_.nppn_ << 12;  // d.
      nid = (mpte1.bits_.nidh_ << 10) | (mpte1.bits_.nidl_);  // e.
      isMrif = true;
    }

  // 15. The access permissions associated with the translation determined through this
  //     process are equivalent to that of a regular RISC-V second-stage PTE with R=W=U=1
  //     and X=0. Similar to a second-stage PTE, when checking the U bit, the transaction
  //     is treated as not requesting supervisor privilege.
  //     a. If the transaction is an Untranslated or Translated read-for-execute then stop
  //        and report "Instruction access fault" (cause = 1).
  if (req.isExec())
    {
      cause = 1;
      return false;
    }

  // 16. MSI address translation process is complete.
  return true;
}


bool
Iommu::stage1Translate(uint64_t satpVal, uint64_t hgatpVal, PrivilegeMode pm, unsigned procId,
                       bool r, bool w, bool x, bool sum,
                       uint64_t va, bool gade, bool sade, uint64_t& gpa, unsigned& cause)
{
  Iosatp satp{satpVal};
  auto privMode = unsigned(pm);
  auto transMode = unsigned(satp.bits_.mode_);   // Sv39, Sv48, ...
  uint64_t ppn = satp.bits_.ppn_;
  stage1Config_(transMode, procId, ppn, sum);
  setFaultOnFirstAccess_(0, not sade);
  setFaultOnFirstAccess_(1, not sade);

  Iohgatp hgatp{hgatpVal};
  transMode = unsigned(hgatp.bits_.mode_);  // Sv39x4, Sv48x4, ...
  unsigned gcsid = hgatp.bits_.gcsid_;
  ppn = hgatp.bits_.ppn_;
  stage2Config_(transMode, gcsid, ppn);
  setFaultOnFirstAccess_(2, not gade);

  return stage1_(va, privMode, r, w, x, gpa, cause);
}


bool
Iommu::stage2Translate(uint64_t hgatpVal, PrivilegeMode pm, bool r, bool w, bool x,
                       uint64_t gpa, bool gade, uint64_t& pa, unsigned& cause)
{
  Iohgatp hgatp{hgatpVal};

  auto privMode = unsigned(pm);
  auto transMode = unsigned(hgatp.bits_.mode_);  // Sv39x4, Sv48x4, ...
  unsigned gcsid = hgatp.bits_.gcsid_;
  uint64_t ppn = hgatp.bits_.ppn_;

  stage2Config_(transMode, gcsid, ppn);
  setFaultOnFirstAccess_(2, not gade);
  return stage2_(gpa, privMode, r, w, x, pa, cause);
}


void
Iommu::configureCapabilities(uint64_t value)
{
  capabilities_.value = value;

  // If capabilities.ATS == 0, set pqb, pqh, pqt, and pqcsr to 0
  if (capabilities_.fields.ats == 0) {
      pqb_.value = 0;
      pqh_ = 0;
      pqt_ = 0;
      pqcsr_.value = 0;
  }

  // If capabilities.HPM == 0, set iocountinh, iohpmcycles, iohpmctr1-31, iohpmevt1-31 to 0
  if (capabilities_.fields.hpm == 0) {
      iocountinh_.value = 0;
      iohpmcycles_.value = 0;
      for (unsigned i = 0; i < 31; ++i) {
          iohpmctr_.at(i) = 0;
          iohpmevt_.at(i).value = 0;
      }
  }

  // If capabilities.DBG == 0, set tr_req_iova, tr_req_ctl, and tr_response to 0
  if (capabilities_.fields.dbg == 0) {
      tr_req_iova_.value = 0;
      tr_req_ctl_.value = 0;
      tr_response_.value = 0;
  }

  // If capabilities.QOSID == 0, set iommu_qosid to 0
  if (capabilities_.fields.qosid == 0) {
      iommu_qosid_.value = 0;
  }

  // Configure fctl.wsi writability based on IGS mode
  // Per RISC-V IOMMU spec:
  // - IGS=MSI: fctl.wsi is hardwired to 0 (not writable)
  // - IGS=WSI: fctl.wsi is hardwired to 1 (not writable)
  // - IGS=Both: fctl.wsi is writable by software
  if (capabilities_.fields.igs == unsigned(IgsMode::Msi) ||
      capabilities_.fields.igs == unsigned(IgsMode::Wsi)) {
    wsiWritable_ = false;  // Hardwired in MSI-only or WSI-only mode
  } else if (capabilities_.fields.igs == unsigned(IgsMode::Both)) {
    wsiWritable_ = true;   // Software can choose between MSI and WSI
  }

  // If capabilities.IGS == WSI, set msi_cfg_tbl to 0
  if (capabilities_.fields.igs == unsigned(IgsMode::Wsi)) {
    for (unsigned i = 0; i < 16; ++i) {
        msi_cfg_tbl_.at(i).regs.msi_addr = 0;
        msi_cfg_tbl_.at(i).regs.msi_data = 0;
        msi_cfg_tbl_.at(i).regs.msi_vec_ctl = 0;
    }
  }
}


void
Iommu::reset()
{
  // Initialize fctl based on capabilities.igs mode
  // if IGS=WSI, fctl.wsi must be 1 (wired interrupts only)
  fctl_.value = 0;
  if (capabilities_.fields.igs == unsigned(IgsMode::Wsi))
    fctl_.fields.wsi = 1;  // WSI-only mode requires wsi=1
  // For IGS=MSI, wsi remains 0 (MSI-only mode)
  // For IGS=Both, wsi defaults to 0 (MSI mode, can be changed by software)
  
  ddtp_.value = 0;
  cqb_.value = 0;
  cqh_ = 0;
  cqt_ = 0;
  fqb_.value = 0;
  fqh_ = 0;
  fqt_ = 0;
  pqb_.value = 0;
  pqh_ = 0;
  pqt_ = 0;
  cqcsr_.value = 0;
  fqcsr_.value = 0;
  pqcsr_.value = 0;
  ipsr_.value = 0;
  iocountinh_.value = 0;
  iohpmcycles_.value = 0;
  iohpmctr_.fill(0);
  iohpmevt_.fill({});
  tr_req_iova_.value = 0;
  tr_req_ctl_.value = 0;
  tr_response_.value = 0;
  iommu_qosid_.value = 0;
  icvec_.value = 0;
  msi_cfg_tbl_.fill({});

  // Reset directory caches
  for (auto& entry : ddtCache_)
    entry.valid = false;
  for (auto& entry : pdtCache_)
    entry.valid = false;
  cacheTimestamp_ = 0;
}


void
Iommu::writeFaultRecord(const FaultRecord& record)
{
  if (not fqcsr_.fields.fqon)
    return;

  if (fqFull())
    {
      fqcsr_.fields.fqof = 1;
      updateIpsr();
      return;
    }

  assert(fqt_ < fqb_.capacity());

  uint64_t slotAddr = (fqb_.fields.ppn << 12) + fqt_ * sizeof(record);

  // Interpret FaultRecord as a an array of double words.
  FaultRecDwords recDwords;
  recDwords.rec = record;
  const auto& dwords = recDwords.dwords;

  bool bigEnd = faultQueueBigEnd();

  for (unsigned i = 0; i < dwords.size(); ++i, slotAddr += 8)
    {
      if (not memWriteDouble(slotAddr, bigEnd, dwords.at(i)))
        {
          fqcsr_.fields.fqmf = 1;
          updateIpsr();
          return;
        }
    }

  // Move tail.
  fqt_ = (fqt_ + 1) % fqb_.capacity();
  updateIpsr(IpsrEvent::NewFault);
}


void
Iommu::writePageRequest(const PageRequest& req)
{
  // Check if page request queue is active
  if (!pqcsr_.fields.pqon)
    {
      // TODO: refer to section 3.7. For now, we silently drop the request when queue is off
      return;
    }

  // Check for error conditions - discard all messages until software clears these bits
  if (pqcsr_.fields.pqmf or pqcsr_.fields.pqof)
    {
      // Discard this message. IOMMU may respond per Section 3.7.
      return;
    }

  // Check if queue is full
  if (pqFull())
    {
      pqcsr_.fields.pqof = 1;
      updateIpsr();
      return;
    }

  // Add page request at tail.
  assert(pqt_ < pqb_.capacity());

  uint64_t slotAddr = (pqb_.fields.ppn << 12) + pqt_ * sizeof(req);

  bool bigEnd = faultQueueBigEnd();

  // Write page request to queue memory
  bool writeOk = true;
  for (unsigned i = 0; i < req.value_.size(); ++i, slotAddr += 8)
    {
      if (!memWriteDouble(slotAddr, bigEnd, req.value_.at(i)))
        {
          writeOk = false;
          break;
        }
    }

  // Check for memory fault during write
  if (!writeOk)
    {
      pqcsr_.fields.pqmf = 1;
      updateIpsr();
      return;
    }

  pqt_ = (pqt_ + 1) % pqb_.capacity();
  updateIpsr(IpsrEvent::NewPageRequest);
}


bool
Iommu::wiredInterrupts() const
{
  if (capabilities_.fields.igs == unsigned(IgsMode::Wsi))
    return true;

  if (capabilities_.fields.igs == unsigned(IgsMode::Both))
    return fctl_.fields.wsi;

  if (capabilities_.fields.igs == unsigned(IgsMode::Msi))
    return false;

  assert(0);
  return false;
}


bool
Iommu::processCommand()
{
  if (cqStalledForItag_)
    return false;
  if (iofenceWaitingForInvals_)
    return false;
  if (not cqcsr_.fields.cqon)
    return false;
  if (cqcsr_.fields.cmd_ill)
    return false;
  if (cqcsr_.fields.cqmf)
    return false;
  if (cqEmpty())
    return false;

  if (cqh_ >= cqb_.capacity())
    return false; // Invalid head pointer

  // Read command from queue
  uint64_t cmdAddr = (cqb_.fields.ppn << 12) + cqh_ * 16ull; // Commands are 16 bytes
  AtsCommandData cmdData;

  bool bigEnd = false; // Command queue endianness (typically little endian)
  if (!memReadDouble(cmdAddr, bigEnd, cmdData.dw0) ||
      !memReadDouble(cmdAddr + 8, bigEnd, cmdData.dw1))
    {
      cqcsr_.fields.cqmf = 1;
      updateIpsr();
      return false;
    }

  // Convert to Command for type checking
  AtsCommand cmd(cmdData);

  // Process the command based on its type
  bool shouldAdvanceHead = true;

  if (isAtsInvalCommand(cmd))
  {
    shouldAdvanceHead = executeAtsInvalCommand(cmd);
  }
  else if (isAtsPrgrCommand(cmd))
  {
    shouldAdvanceHead = executeAtsPrgrCommand(cmd);
  }
  else if (isIodirCommand(cmd))
  {
    executeIodirCommand(cmd);
  }
  else if (isIofenceCCommand(cmd))
  {
    shouldAdvanceHead = executeIofenceCCommand(cmd);
  }
  else if (isIotinvalVmaCommand(cmd) || isIotinvalGvmaCommand(cmd))
  {
    executeIotinvalCommand(cmd);
  }
  else
  {
    shouldAdvanceHead = false;
    cqcsr_.fields.cmd_ill = 1;
    updateIpsr();
  }

  // Advance head pointer
  if (shouldAdvanceHead)
    cqh_ = (cqh_ + 1) % cqb_.capacity();
  return shouldAdvanceHead;
}

void
Iommu::processCommandQueue()
{
  while (processCommand())
    ;
}

bool
Iommu::executeAtsInvalCommand(const AtsCommand& atsCmd)
{
  // Parse ATS.INVAL command
  const auto& cmd = atsCmd.inval;  // Reinterpret generic command as an AtsInvalCommand.

  // Check if ATS capability is enabled
  if (!capabilities_.fields.ats)
  {
    // ATS not supported - command is illegal
    cqcsr_.fields.cmd_ill = 1;
    updateIpsr();
    return false; // Don't advance head, command is illegal
  }

  // Extract command fields
  uint32_t rid = cmd.RID;
  uint32_t pid = cmd.PID;
  bool pv = cmd.PV;
  bool dsv = cmd.DSV;
  uint32_t dseg = cmd.DSEG;
  uint64_t address = cmd.address;
  bool global = cmd.G;

  uint32_t devId = dsv ? ((dseg << 16) | rid) : rid;
  InvalidationScope scope = InvalidationScope::GlobalDevice;
  if (!global)
    {
      if (pv && address != 0)
        scope = InvalidationScope::ProcessAndAddress;
      else if (pv)
        scope = InvalidationScope::ProcessSpecific;
      else if (address != 0)
        scope = InvalidationScope::AddressSpecific;
    }

  uint8_t itag = 0;
  if (!allocateItag(devId, dsv, dseg, rid, pv, pid, address, global, scope, itag))
  {
    blockedAtsInval_ = BlockedAtsInval{
      .devId = devId,
      .pid = pid,
      .pv = pv,
      .dsv = dsv,
      .dseg = static_cast<uint8_t>(dseg),
      .rid = static_cast<uint16_t>(rid),
      .address = address,
      .global = global,
      .scope = scope
    };
    cqStalledForItag_ = true;
    return false;
  }

  if (sendInvalReq_)
    sendInvalReq_(devId, pid, pv, address, global, scope, itag);

  return true;
}

bool
Iommu::executeAtsPrgrCommand(const AtsCommand& atsCmd)
{
  const auto& cmd = atsCmd.prgr;

  if (!capabilities_.fields.ats)
  {
    // ATS not supported - command is illegal
    cqcsr_.fields.cmd_ill = 1;
    updateIpsr();
    return false; // Don't advance head, command is illegal
  }

  uint32_t rid = cmd.RID;
  uint32_t pid = cmd.PID;
  uint32_t prgi = cmd.prgi;
  uint32_t resp_code = cmd.responsecode;
  bool pv = cmd.PV;
  bool dsv = cmd.DSV;
  uint32_t dseg = cmd.DSEG;
  uint32_t devId = dsv ? ((dseg << 16) | rid) : rid;

  if (sendPrgr_)
    sendPrgr_(devId, pid, pv, prgi, resp_code, dsv, dseg);
  
  return true; // Command completed, advance head
}

void
Iommu::executeIodirCommand(const AtsCommand& atsCmd)
{
  const auto& cmd = atsCmd.iodir;
  uint32_t pid = cmd.PID;
  bool dv = cmd.DV;
  uint32_t did = cmd.DID;
  IodirFunc func = cmd.func3;

  if (func == IodirFunc::INVAL_DDT)
  {
    if (dv)
    {
      bool extended = capabilities_.fields.msi_flat;
      Devid devid(did);
      unsigned ddi1 = devid.ithDdi(1, extended);
      unsigned ddi2 = devid.ithDdi(2, extended);

      Ddtp::Mode ddtpMode = ddtp_.fields.iommu_mode;
      if ((ddtpMode == Ddtp::Mode::Level2 and ddi2 != 0) or
          (ddtpMode == Ddtp::Mode::Level1 and (ddi2 != 0 or ddi1 != 0)))
        return;
    }

    (void)pid;
    invalidateDdtCache(did, dv);
  }
  else if (func == IodirFunc::INVAL_PDT)
  {
    if (!dv)
      return;

    bool extended = capabilities_.fields.msi_flat;
    Devid devid(did);
    unsigned ddi1 = devid.ithDdi(1, extended);
    unsigned ddi2 = devid.ithDdi(2, extended);

    Ddtp::Mode ddtpMode = ddtp_.fields.iommu_mode;
    if ((ddtpMode == Ddtp::Mode::Level2 and ddi2 != 0) or
        (ddtpMode == Ddtp::Mode::Level1 and (ddi2 != 0 or ddi1 != 0)))
      return;

    DeviceContext dc;
    unsigned cause = 0;
    if (loadDeviceContext(did, dc, cause))
    {
      if (dc.pdtv())
      {
        Procid procid(pid);
        unsigned pdi1 = procid.ithPdi(1);
        unsigned pdi2 = procid.ithPdi(2);
        PdtpMode pdtpMode = dc.pdtpMode();

        if ((pdtpMode == PdtpMode::Pd17 and pdi2 != 0) or
            (pdtpMode == PdtpMode::Pd8 and (pdi2 != 0 or pdi1 != 0)))
          return;
      }
      else
        return;
    }
    else
      return;

    invalidatePdtCache(did, pid);
  }
}

bool
Iommu::executeIofenceCCore(bool pr, bool pw, bool av, bool wsi, uint64_t addr, uint32_t data)
{
  // Check and report timeout if needed
  if (atsInvalTimeout_)
  {
    if (cqcsr_.fields.cmd_to == 0)
      {
        cqcsr_.fields.cmd_to = 1;
        updateIpsr();
        dbg_fprintf(stdout, "IOFENCE.C: Reporting ATS.INVAL timeout via cmd_to bit\n");
        return false; // Do not advance head pointer while reporting timeout
      }
    // Timeout has been reported and acknowledged, clear it
    atsInvalTimeout_ = false;
  }

  // Execute memory ordering (PR/PW bits)
  if (pr || pw)
  {
    // TODO: Implement memory ordering guarantees
    // For now, assume ordering is handled by the memory system
  }

  // Execute memory write if AV=1
  if (av)
  {
    if (!memWrite(addr, 4, data))
    {
      cqcsr_.fields.cqmf = 1;
      updateIpsr();
      return false; // Do not advance head pointer on memory fault
    }
  }

  // Generate interrupt if WSI=1
  // Per spec: fence_w_ip should only be set when wired interrupts are active
  if (wsi && wiredInterrupts())
    {
      cqcsr_.fields.fence_w_ip = 1;
      updateIpsr();
    }

  return true; // Command completed successfully
}

bool
Iommu::executeIofenceCCommand(const AtsCommand& atsCmd)
{
  // Parse IOFENCE.C command
  const auto& cmd = atsCmd.iofence;

  // Extract command fields
  bool av = cmd.AV;
  bool wsi = cmd.WSI;
  bool pr = cmd.PR;
  bool pw = cmd.PW;
  uint64_t addr = cmd.ADDR << 2; // Convert from ADDR[63:2] to full address
  uint32_t data = cmd.DATA;

  if (cmd.reserved0 or cmd.reserved1 or (cmd.func3 != IofenceFunc::C) or (cmd.WSI and not fctl_.fields.wsi))
    {
      cqcsr_.fields.cmd_ill = 1;
      updateIpsr();
      return false;
    }

  dbg_fprintf(stdout, "IOFENCE.C: AV=%d, WSI=%d, PR=%d, PW=%d, addr=0x%lx, data=0x%x\n", av, wsi, pr, pw, addr, data);

  // Check if waiting for invalidation requests to complete
  if (anyItagBusy())
  {
    dbg_fprintf(stdout, "IOFENCE.C: Waiting for %zu pending ATS.INVAL commands (ITAGs busy)\n", countBusyItags());

    pendingIofence_ = PendingIofence{
      .pr = pr,
      .pw = pw,
      .av = av,
      .wsi = wsi,
      .addr = addr,
      .data = data
    };

    iofenceWaitingForInvals_ = true;
    return false; // Do not advance head pointer
  }

  // Execute the core IOFENCE logic
  return executeIofenceCCore(pr, pw, av, wsi, addr, data);
}

bool
Iommu::retryPendingIofence()
{
  if (!pendingIofence_.has_value())
    return true; // No pending fence, consider it successful

  const auto& fence = pendingIofence_.value();

  dbg_fprintf(stdout, "IOFENCE.C: Retrying after ITAGs freed\n");

  // Execute the core IOFENCE logic
  if (!executeIofenceCCore(fence.pr, fence.pw, fence.av, fence.wsi, fence.addr, fence.data))
  {
    // Failed (timeout reporting or memory fault) - keep fence pending
    return false;
  }

  // Success - clear the stall condition and advance head pointer
  iofenceWaitingForInvals_ = false;
  pendingIofence_.reset();

  cqh_ = (cqh_ + 1) % cqb_.capacity();

  return true; // Successfully completed
}

void
Iommu::executeIotinvalCommand(const AtsCommand& atsCmd)
{
  // Parse IOTINVAL command (handles both VMA and GVMA)
  const auto& cmd = atsCmd.iotinval; // Reinterpret genric command as IotinvalCommand

  // Extract command fields
  bool AV = cmd.AV;        // Address Valid
  bool PSCV = cmd.PSCV;    // Process Soft-Context Valid
  bool GV = cmd.GV;        // Guest Soft-Context Valid
  uint32_t PSCID = cmd.PSCID;  // Process Soft-Context ID
  uint32_t GSCID = cmd.GSCID;  // Guest Soft-Context ID
  uint64_t addr = cmd.ADDR << 12; // Convert from ADDR[63:12] to full address (page-aligned)
  bool isVma = (cmd.func3 == IotinvalFunc::VMA);
  bool isGvma = (cmd.func3 == IotinvalFunc::GVMA);

  const char* cmdName = isVma ? "IOTINVAL.VMA" : "IOTINVAL.GVMA";

  dbg_fprintf(stdout, "%s: AV=%d, PSCV=%d, GV=%d, PSCID=0x%x, GSCID=0x%x, addr=0x%lx\n", cmdName, AV, PSCV, GV, PSCID, GSCID, addr);

  // ========================================================================
  // IOTINVAL.VMA - First-stage page table cache invalidation
  // ========================================================================
  if (isVma) {
    // Validate VMA-specific parameters
    if (PSCV && !AV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalid combination - PSCV=1 requires AV=1\n");
      return;
    }

    // Table 9: IOTINVAL.VMA operands and operations (8 combinations)
    if (!GV && !AV && !PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating all first-stage page table cache entries for all host address spaces\n");
    }
    else if (!GV && !AV && PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating first-stage entries for host address space with PSCID=0x%x\n", PSCID);
    }
    else if (!GV && AV && !PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating first-stage entries for address 0x%lx in all host address spaces\n", addr);
    }
    else if (!GV && AV && PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating first-stage entries for address 0x%lx in host address space PSCID=0x%x\n", addr, PSCID);
    }
    else if (GV && !AV && !PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating all first-stage entries for VM address spaces with GSCID=0x%x\n", GSCID);
    }
    else if (GV && !AV && PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating first-stage entries for VM address space PSCID=0x%x, GSCID=0x%x\n", PSCID, GSCID);
    }
    else if (GV && AV && !PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating first-stage entries for address 0x%lx in all VM address spaces with GSCID=0x%x\n", addr, GSCID);
    }
    else if (GV && AV && PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.VMA: Invalidating first-stage entries for address 0x%lx in VM address space PSCID=0x%x, GSCID=0x%x\n", addr, PSCID, GSCID);
    }
  }
  // ========================================================================
  // IOTINVAL.GVMA - Second-stage page table cache invalidation
  // ========================================================================
  else if (isGvma) {
    // Validate GVMA-specific parameters
    if (PSCV) {
      dbg_fprintf(stdout, "IOTINVAL.GVMA: Invalid command - PSCV must be 0 for GVMA commands\n");
      return;
    }

    // Table 10: IOTINVAL.GVMA operands and operations (3 combinations)
    if (!GV) {
      // When GV=0, AV is ignored per Table 10
      dbg_fprintf(stdout, "IOTINVAL.GVMA: Invalidating all second-stage page table cache entries for all VM address spaces (AV ignored)\n");
    }
    else if (GV && !AV) {
      dbg_fprintf(stdout, "IOTINVAL.GVMA: Invalidating all second-stage entries for VM address spaces with GSCID=0x%x\n", GSCID);
    }
    else if (GV && AV) {
      dbg_fprintf(stdout, "IOTINVAL.GVMA: Invalidating second-stage leaf entries for address 0x%lx in VM address space GSCID=0x%x\n", addr, GSCID);
    }
  }

  // TODO: Implement actual IOATC (IOMMU Address Translation Cache) invalidation
  // This is a stub implementation - actual invalidation logic would:
  // 1. Identify matching IOATC entries based on the invalidation scope
  // 2. Remove/invalidate those entries from the translation cache
  // 3. Ensure ordering with respect to previous memory operations per specification

  dbg_fprintf(stdout, "%s: Command completed (stub implementation)\n", cmdName);

  addr_ = addr_ + 0;
}


bool
Iommu::atsTranslate(const IommuRequest& req, AtsResponse& response, unsigned& cause)
{
  response = AtsResponse{};
  uint64_t pa = 0;
  response.success = translate(req, pa, cause);
  if (!response.success) {
    response.isCompleterAbort = (
      cause == 1 or cause == 5 or cause == 7 or  // Access faults
      cause == 261 or cause == 263 or            // MSI PTE faults
      cause == 265 or cause == 267               // PDT entry faults
    );
  }
  response.translatedAddr = pa;
  response.readPerm = false; // TODO: get this from PTE
  response.writePerm = req.isWrite(); // TODO: get this from PTE
  response.execPerm = req.isExec(); // TODO: get this from PTE
  response.privMode = req.hasProcId and (req.privMode == PrivilegeMode::Supervisor);
  response.noSnoop = false; // TODO
  response.cxlIo = false; // TODO
  response.global = false; // TODO
  response.ama = 0; // TODO
  response.untranslatedOnly = false; // TODO
  return response.success;
}


void
Iommu::atsPageRequest(const PageRequest& req)
{
  uint32_t devId = req.bits_.did_;
  uint32_t pid = req.bits_.pid_;
  bool pv = req.bits_.pv_;
  bool priv = req.bits_.priv_;
  bool R = req.bits_.r_;
  bool W = req.bits_.w_;
  bool L = req.bits_.l_;
  uint32_t prgi = req.bits_.prgi_;

  PrgrResponseCode responseCode = PrgrResponseCode::FAILURE;
  uint32_t rid = devId & 0xffff;
  uint32_t dseg = (devId >> 16) & 0xff;
  bool dsv = dsv_;

  unsigned cause = 0;
  DeviceContext dc;

  FaultRecord faultRecord;
  faultRecord.pid = pid;
  faultRecord.pv = pv;
  faultRecord.priv = priv;
  faultRecord.ttyp = unsigned(Ttype::PcieMessage);
  faultRecord.did = devId;
  faultRecord.iotval = unsigned(PcieMsgCode::PAGE_REQ);
  faultRecord.iotval2 = 0;

  bool extended = capabilities_.fields.msi_flat;
  Devid devid(devId);
  unsigned ddi1 = devid.ithDdi(1, extended);
  unsigned ddi2 = devid.ithDdi(2, extended);

  bool send = false;

  if (ddtp_.fields.iommu_mode == Ddtp::Mode::Off)
  {
    faultRecord.cause = 256;
    writeFaultRecord(faultRecord);
    responseCode = PrgrResponseCode::FAILURE;
    send = true;
  }
  else if (ddtp_.fields.iommu_mode == Ddtp::Mode::Bare ||
           (ddtp_.fields.iommu_mode == Ddtp::Mode::Level2 && ddi2 != 0) ||
           (ddtp_.fields.iommu_mode == Ddtp::Mode::Level1 && (ddi2 != 0 || ddi1 != 0)))
  {
    faultRecord.cause = 260;
    writeFaultRecord(faultRecord);
    responseCode = PrgrResponseCode::INVALID;
    send = true;
  }
  else if (!loadDeviceContext(devId, dc, cause))
  {
    faultRecord.cause = cause;
    writeFaultRecord(faultRecord);
    responseCode = PrgrResponseCode::FAILURE;
    send = true;
  }

  bool prpr = send? false : dc.prpr();

  if (not send)
    {
      if (!dc.pri())
        {
          faultRecord.cause = 260;
          writeFaultRecord(faultRecord);
          responseCode = PrgrResponseCode::INVALID;
          send = true;
        }
      else
        {
          if (!pqcsr_.fields.pqon || !pqcsr_.fields.pqen || pqcsr_.fields.pqmf)
            {
              responseCode = PrgrResponseCode::FAILURE;
              send = true;
            }
          else if (pqcsr_.fields.pqof)
            {
              responseCode = PrgrResponseCode::SUCCESS;
              send = true;
            }
        }
    }

  if (not send)
  {
    Pqcsr pqcsrBefore = pqcsr_;
    writePageRequest(req);
    Pqcsr pqcsrAfter = pqcsr_;

    if (pqcsrAfter.fields.pqof && !pqcsrBefore.fields.pqof)
    {
      responseCode = PrgrResponseCode::SUCCESS;
      send = true;
    }
    else if (pqcsrAfter.fields.pqmf && !pqcsrBefore.fields.pqmf)
    {
      responseCode = PrgrResponseCode::FAILURE;
      send = true;
    }
  }

  if (not send)
    return;

  if (!L || (L && !R && !W))
    return;

  if (!sendPrgr_)
    return;

  bool includePasid = false;
  if (responseCode == PrgrResponseCode::INVALID || responseCode == PrgrResponseCode::SUCCESS)
    includePasid = (prpr && pv);
  else
    includePasid = pv;

  sendPrgr_(rid, includePasid ? pid : 0, includePasid, prgi, uint32_t(responseCode), dsv, dseg);
}


bool
Iommu::allocateItag(uint32_t devId, bool dsv, uint8_t dseg, uint16_t rid,
                    bool pv, uint32_t pid, uint64_t address, bool global,
                    InvalidationScope scope, uint8_t& itag)
{
  for (uint8_t i = 0; i < MAX_ITAGS; i++)
  {
    if (!itagTrackers_.at(i).busy)
    {
      itagTrackers_.at(i).busy = true;
      itagTrackers_.at(i).dsv = dsv;
      itagTrackers_.at(i).dseg = dseg;
      itagTrackers_.at(i).rid = rid;
      itagTrackers_.at(i).devId = devId;
      itagTrackers_.at(i).pv = pv;
      itagTrackers_.at(i).pid = pid;
      itagTrackers_.at(i).address = address;
      itagTrackers_.at(i).global = global;
      itagTrackers_.at(i).scope = scope;
      itagTrackers_.at(i).numRspRcvd = 0;

      itag = i;
      return true;
    }
  }

  return false;
}


bool
Iommu::anyItagBusy() const
{
  return std::any_of(itagTrackers_.begin(), itagTrackers_.end(),
                     [](const auto& tracker) { return tracker.busy; });
}


size_t
Iommu::countBusyItags() const
{
  size_t count = 0;
  for (const auto& tracker : itagTrackers_)
    if (tracker.busy)
      count++;
  return count;
}


void
Iommu::retryBlockedAtsInval()
{
  if (!blockedAtsInval_.has_value())
    return;

  uint8_t itag = 0;
  const auto& blocked = blockedAtsInval_.value();

  if (allocateItag(blocked.devId, blocked.dsv, blocked.dseg, blocked.rid,
                   blocked.pv, blocked.pid, blocked.address, blocked.global,
                   blocked.scope, itag))
  {
    dbg_fprintf(stdout, "ATS.INVAL: Retried blocked request with ITAG=%u, devId=0x%x\n", itag, blocked.devId);

    if (sendInvalReq_)
      sendInvalReq_(blocked.devId, blocked.pid, blocked.pv,
                   blocked.address, blocked.global, blocked.scope, itag);

    blockedAtsInval_.reset();
    cqStalledForItag_ = false;

    // Advance the command queue head pointer
    cqh_ = (cqh_ + 1) % cqb_.capacity();
  }
}


void
Iommu::atsInvalidationCompletion(uint32_t devId, uint32_t itagVector,
                                 uint8_t completionCount)
{
  dbg_fprintf(stdout, "ATS.INVAL Completion: devId=0x%x, itagVector=0x%x, cc=%u\n", devId, itagVector, completionCount);

  for (uint8_t i = 0; i < MAX_ITAGS; i++)
  {
    if (itagVector & (1 << i))
    {
      if (!itagTrackers_.at(i).busy)
      {
        dbg_fprintf(stdout, "WARNING: Unexpected completion for ITAG=%u (not busy)\n", i);
        continue;
      }

      if (itagTrackers_.at(i).devId != devId)
      {
        dbg_fprintf(stdout, "ERROR: Device ID mismatch for ITAG=%u (expected 0x%x, got 0x%x)\n", i, itagTrackers_.at(i).devId, devId);
        continue;
      }

      itagTrackers_.at(i).numRspRcvd++;

      dbg_fprintf(stdout, "ATS.INVAL: ITAG=%u received completion %u/%u\n", i, itagTrackers_.at(i).numRspRcvd, completionCount);

      if (itagTrackers_.at(i).numRspRcvd == completionCount)
      {
        dbg_fprintf(stdout, "ATS.INVAL: ITAG=%u complete, freeing\n", i);
        itagTrackers_.at(i).busy = false;

        retryBlockedAtsInval();

        if (iofenceWaitingForInvals_ && !anyItagBusy())
          retryPendingIofence();
      }
    }
  }
}

void
Iommu::atsInvalidationTimeout(uint32_t itagVector)
{
  dbg_fprintf(stdout, "ATS.INVAL Timeout: itagVector=0x%x\n", itagVector);

  for (uint8_t i = 0; i < MAX_ITAGS; i++)
  {
    if (itagVector & (1 << i))
    {
      if (itagTrackers_.at(i).busy)
      {
        dbg_fprintf(stdout, "ATS.INVAL: ITAG=%u timed out, freeing\n", i);
        itagTrackers_.at(i).busy = false;
      }
    }
  }

  atsInvalTimeout_ = true;
  retryBlockedAtsInval();

  if (iofenceWaitingForInvals_ && !anyItagBusy())
    retryPendingIofence();
}


void
Iommu::waitForPendingAtsInvals()
{
  if (!anyItagBusy())
    return;

  dbg_fprintf(stdout, "IOFENCE.C: Waiting for %zu pending ATS.INVAL requests to complete\n", countBusyItags());

  if (anyItagBusy())
  {
    dbg_fprintf(stdout, "IOFENCE.C: Clearing %zu pending ITAGs (assuming completion or timeout)\n", countBusyItags());

    for (auto& tracker : itagTrackers_)
      if (tracker.busy)
      {
        tracker.busy = false;
        atsInvalTimeout_ = true;
      }
  }

  dbg_fprintf(stdout, "IOFENCE.C: All prior ATS.INVAL commands complete\n");
}


bool
Iommu::t2gpaTranslate(const IommuRequest& req, uint64_t& gpa, unsigned& cause)
{
  deviceDirWalk_.clear();
  processDirWalk_.clear();

  cause = 0;

  // Check IOMMU mode
  if (ddtp_.fields.iommu_mode == Ddtp::Mode::Off)
  {
    cause = 256; // All inbound transactions disallowed
    return false;
  }

  if (ddtp_.fields.iommu_mode == Ddtp::Mode::Bare)
  {
    // In bare mode, no translation - IOVA is the GPA
    gpa = req.iova;
    return true;
  }

  // Load device context
  DeviceContext dc;
  if (not loadDeviceContext(req.devId, dc, cause))
    return false;

  // T2GPA mode requires ATS to be enabled
  if (not dc.ats() or not dc.t2gpa())
  {
    cause = 260; // Transaction type disallowed
    return false;
  }

  // Determine access permissions
  bool r = req.isRead() or req.isExec();
  bool w = req.isWrite();
  bool x = req.isExec();

  // Perform first-stage translation to get GPA
  if (dc.pdtv())
  {
    // Process directory table mode
    ProcessContext pc;
    unsigned procId = req.hasProcId ? req.procId : 0;

    if (req.hasProcId or dc.dpe())
    {
      if (not loadProcessContext(dc, req.devId, procId, pc, cause))
        return false;

      // Use process context for first-stage translation
      uint64_t iosatp = pc.fsc(); // FSC field contains the IOSATP value
      uint64_t iohgatp = dc.iohgatp();
      bool sum = pc.sum();

      if (not stage1Translate(iosatp, iohgatp, req.privMode, procId, r, w, x, sum, dc.gade(), dc.sade(), req.iova, gpa, cause))
        return false;
    }
    else
    {
      // No valid process ID and DPE not enabled
      cause = 260; // Transaction type disallowed
      return false;
    }
  }
  else
  {
    // Direct IOSATP mode
    uint64_t iosatp = dc.iosatp();
    uint64_t iohgatp = dc.iohgatp();
    bool sum = false;

    if (not stage1Translate(iosatp, iohgatp, req.privMode, 0, r, w, x, sum, req.iova, dc.gade(), dc.sade(), gpa, cause))
      return false;
  }

  // In T2GPA mode, we stop here and return the GPA
  // The device will use this GPA in subsequent translated requests
  // which will then undergo second-stage translation

  return true;
}


bool
Iommu::definePmpRegs(uint64_t cfgAddr, unsigned cfgCount,
                     uint64_t addrAddr, unsigned addrCount)
{
  if (cfgCount == 0 and addrCount == 0)
    {
      pmpcfgCount_ = cfgCount;
      pmpaddrCount_ = addrCount;
      pmpEnabled_ = false;
      return true;
    }

  if (addrCount != 8 and addrCount != 16 and addrCount != 64)
    {
      dbg_fprintf(stderr, "Invalid IOMMU PMPADDR count: %d -- expecting 8, 16, or 64\n", addrCount);
      return false;
    }

  if ((addrCount / 8) != cfgCount)
    {
      dbg_fprintf(stderr, "Invalid IOMMU PMPCFG count: %d -- expecting %d\n", cfgCount, (addrCount / 8));
      return false;
    }

  if ((cfgAddr & 7) != 0)
    {
      dbg_fprintf(stderr, "Invalid IOMMU PMPCFG address: 0x%llx: must be double-word aligned\n", cfgAddr);
      return false;
    }

  if ((addrAddr & 7) != 0)
    {
      dbg_fprintf(stderr, "Invalid IOMMU PMPADDR address: 0x%llx: must be double-word aligned\n", addrAddr);
      return false;
    }

  pmpcfgCount_ = cfgCount;
  pmpaddrCount_ = addrCount;
  pmpcfgAddr_ = cfgAddr;
  pmpaddrAddr_ = addrAddr;

  pmacfg_.clear();
  pmpcfg_.resize(pmpcfgCount_);

  pmpaddr_.clear();
  pmpaddr_.resize(pmpaddrCount_);

  pmpEnabled_ = true;
  return true;
}


void
Iommu::updateMemoryProtection()
{
  pmpMgr_.reset();

  for (unsigned ix = 0; ix < pmpaddrCount_; ++ix)
    {
      uint64_t low = 0, high = 0;
      Pmp::Type type = Pmp::Type::Off;
      Pmp::Mode mode = Pmp::Mode::None;
      bool locked = false;

      uint8_t cfgByte = getPmpcfgByte(ix);
      uint64_t val = pmpaddr_.at(ix);
      uint64_t precVal =  (ix == 0) ? 0 : pmpaddr_.at(ix - 1);  // Preceding PMPADDR reg.

      pmpMgr_.unpackMemoryProtection(cfgByte, val, precVal, false /*rv32*/, mode,
                                     type, locked, low, high);

      pmpMgr_.defineRegion(low, high, type, mode, ix, locked);
    }
}


bool
Iommu::definePmaRegs(uint64_t cfgAddr, unsigned cfgCount)
{
  if (cfgCount == 0)
    {
      pmacfgCount_ = cfgCount;
      pmaEnabled_ = false;
      return true;
    }

  if ((cfgAddr & 7) != 0)
    {
      dbg_fprintf(stderr, "Invalid IOMMU PMACFG address: %llx: must be double-word aligned\n", cfgAddr);
      return false;
    }

  pmacfgCount_ = cfgCount;
  pmacfgAddr_ = cfgAddr;

  pmacfg_.clear();
  pmacfg_.resize(pmacfgCount_);

  pmaEnabled_ = true;
  return true;
}


void
Iommu::updateMemoryAttributes(unsigned pmacfgIx)
{
  uint64_t val = pmacfg_.at(pmacfgIx);

  uint64_t low = 0, high = 0;
  Pma pma;
  bool valid = false;

  PmaManager::unpackPmacfg(val, valid, low, high, pma);
  if (valid)
    {
      if (not pmaMgr_.defineRegion(pmacfgIx, low, high, pma))
        assert(0);
    }
}


void
Iommu::invalidateDdtCache(uint32_t deviceId, bool dv)
{
  if (!dv)
  {
    for (auto& entry : ddtCache_)
      entry.valid = false;
  }
  else
  {
    for (auto& entry : ddtCache_)
      if (entry.valid && entry.deviceId == deviceId)
        entry.valid = false;
  }
}


void
Iommu::invalidatePdtCache(uint32_t deviceId, uint32_t processId)
{
  for (auto& entry : pdtCache_)
    if (entry.valid && entry.deviceId == deviceId && entry.processId == processId)
      entry.valid = false;
}


Iommu::DdtCacheEntry*
Iommu::findDdtCacheEntry(uint32_t deviceId)
{
  for (auto& entry : ddtCache_)
    if (entry.valid && entry.deviceId == deviceId)
    {
      entry.timestamp = cacheTimestamp_++;
      return &entry;
    }
  return nullptr;
}


Iommu::PdtCacheEntry*
Iommu::findPdtCacheEntry(uint32_t deviceId, uint32_t processId)
{
  for (auto& entry : pdtCache_)
    if (entry.valid && entry.deviceId == deviceId && entry.processId == processId)
    {
      entry.timestamp = cacheTimestamp_++;
      return &entry;
    }
  return nullptr;
}


void
Iommu::updateDdtCache(uint32_t deviceId, const DeviceContext& dc)
{
  for (auto& entry : ddtCache_)
    if (entry.valid && entry.deviceId == deviceId)
    {
      entry.deviceContext = dc;
      entry.timestamp = cacheTimestamp_++;
      return;
    }

  for (auto& entry : ddtCache_)
    if (!entry.valid)
    {
      entry.deviceId = deviceId;
      entry.deviceContext = dc;
      entry.timestamp = cacheTimestamp_++;
      entry.valid = true;
      return;
    }

  assert(!ddtCache_.empty());
  auto lruIt = ddtCache_.begin();
  for (auto it = ddtCache_.begin(); it != ddtCache_.end(); ++it)
    if (it->timestamp < lruIt->timestamp)
      lruIt = it;

  lruIt->deviceId = deviceId;
  lruIt->deviceContext = dc;
  lruIt->timestamp = cacheTimestamp_++;
  lruIt->valid = true;
}


void
Iommu::updatePdtCache(uint32_t deviceId, uint32_t processId, const ProcessContext& pc)
{
  for (auto& entry : pdtCache_)
    if (entry.valid && entry.deviceId == deviceId && entry.processId == processId)
    {
      entry.processContext = pc;
      entry.timestamp = cacheTimestamp_++;
      return;
    }

  for (auto& entry : pdtCache_)
    if (!entry.valid)
    {
      entry.deviceId = deviceId;
      entry.processId = processId;
      entry.processContext = pc;
      entry.timestamp = cacheTimestamp_++;
      entry.valid = true;
      return;
    }

  auto lruIt = pdtCache_.begin();
  for (auto it = pdtCache_.begin(); it != pdtCache_.end(); ++it)
    if (it->timestamp < lruIt->timestamp)
      lruIt = it;

  lruIt->deviceId = deviceId;
  lruIt->processId = processId;
  lruIt->processContext = pc;
  lruIt->timestamp = cacheTimestamp_++;
  lruIt->valid = true;
}
