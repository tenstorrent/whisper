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

#include <string>
#include <vector>
#include "DeviceContext.hpp"
#include "ProcessContext.hpp"
#include "FaultQueue.hpp"
#include "Ats.hpp"
#include "IommuPmpManager.hpp"
#include "IommuPmaManager.hpp"


namespace TT_IOMMU
{
  enum class IgsMode : uint32_t
    {
      Msi, Wsi, Both, Reserved
    };

  union Capabilities {
    struct {
      uint64_t version        : 8;
      uint64_t sv32           : 1;
      uint64_t sv39           : 1;
      uint64_t sv48           : 1;
      uint64_t sv57           : 1;
      uint64_t reserved0      : 2;
      uint64_t svrsw60t59b    : 1;
      uint64_t svpbmt         : 1;
      uint64_t sv32x4         : 1;
      uint64_t sv39x4         : 1;
      uint64_t sv48x4         : 1;
      uint64_t sv57x4         : 1;
      uint64_t reserved1      : 1;
      uint64_t amo_mrif       : 1;
      uint64_t msi_flat       : 1;
      uint64_t msi_mrif       : 1;
      uint64_t amo_hwad       : 1;
      uint64_t ats            : 1;
      uint64_t t2gpa          : 1;
      uint64_t end            : 1;
      uint64_t igs            : 2;
      uint64_t hpm            : 1;
      uint64_t dbg            : 1;
      uint64_t pas            : 6;
      uint64_t pd8            : 1;
      uint64_t pd17           : 1;
      uint64_t pd20           : 1;
      uint64_t qosid          : 1;
      uint64_t nl             : 1;
      uint64_t s              : 1;
      uint64_t reserved2      : 12;
      uint64_t custom         : 8;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;
  };

  union Fctl {
    struct {
      uint32_t be         : 1;
      uint32_t wsi        : 1;
      uint32_t gxl        : 1;
      uint32_t reserved   : 13;
      uint32_t custom     : 16;
    } fields;
    uint32_t value;
  };

  union Ddtp {
    enum class Mode : uint32_t {
      Off = 0, Bare = 1, Level1 = 2, Level2 = 3, Level3 = 4
    };

    struct {
      Mode     iommu_mode : 4;
      uint64_t busy       : 1;
      uint64_t reserved0  : 5;
      uint64_t ppn        : 44;
      uint64_t reserved1  : 10;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;

    /// Return the number of levels encoded in this DDTP or 0 if no valid number of
    /// levels.
    unsigned levels() const
    {
      switch (fields.iommu_mode)
      {
      case Mode::Level1: return 1;
      case Mode::Level2: return 2;
      case Mode::Level3: return 3;
      default: return 0;
      }
      return 0;
    }
  };
  static_assert(sizeof(Ddtp) == 8, "Ddtp not 8 bytes in size");

  union Xqb {
    struct {
      uint64_t log2szm1   : 5;
      uint64_t reserved0  : 5;
      uint64_t ppn        : 44;
      uint64_t reserved1  : 10;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;

    unsigned capacity() const {
        return 1u << (fields.log2szm1+1);
    }
  };

  using Cqb = Xqb;
  using Fqb = Xqb;
  using Pqb = Xqb;

  union Cqcsr {
    struct {
      uint32_t cqen       : 1;
      uint32_t cie        : 1;
      uint32_t reserved0  : 6;
      uint32_t cqmf       : 1;
      uint32_t cmd_to     : 1;
      uint32_t cmd_ill    : 1;
      uint32_t fence_w_ip : 1;
      uint32_t reserved1  : 4;
      uint32_t cqon       : 1;
      uint32_t busy       : 1;
      uint32_t reserved2  : 10;
      uint32_t custom     : 4;
    } fields;
    uint32_t value;
  };

  union Fqcsr {
    struct {
      uint32_t fqen       : 1;
      uint32_t fie        : 1;
      uint32_t reserved0  : 6;
      uint32_t fqmf       : 1;
      uint32_t fqof       : 1;
      uint32_t reserved1  : 6;
      uint32_t fqon       : 1;
      uint32_t busy       : 1;
      uint32_t reserved2  : 10;
      uint32_t custom     : 4;
    } fields;
    uint32_t value;
  };

  union Pqcsr {
    struct {
      uint32_t pqen       : 1;
      uint32_t pie        : 1;
      uint32_t reserved0  : 6;
      uint32_t pqmf       : 1;
      uint32_t pqof       : 1;
      uint32_t reserved1  : 6;
      uint32_t pqon       : 1;
      uint32_t busy       : 1;
      uint32_t reserved2  : 10;
      uint32_t custom     : 4;
    } fields;
    uint32_t value;
  };

  union Ipsr {
    struct {
      uint32_t cip        : 1;
      uint32_t fip        : 1;
      uint32_t pmip       : 1;
      uint32_t pip        : 1;
      uint32_t reserved0  : 4;
      uint32_t custom     : 8;
      uint32_t reserved1  : 16;
    } fields;
    uint32_t value;
  };

  union Iocountovf {
    struct {
      uint32_t cy     : 1;
      uint32_t hpm    : 31;
    } fields;
    uint32_t value;
  };

  union Iocountinh {
    struct {
      uint32_t cy     : 1;
      uint32_t hpm    : 31;
    } fields;
    uint32_t value;
  };

  union Iohpmcycles {
    struct {
      uint64_t counter    : 63;
      uint64_t of         : 1;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;
  };

  union Iohpmevt {
    struct {
      uint64_t eventId    : 15;
      uint64_t dmask      : 1;
      uint64_t pid_pscid  : 20;
      uint64_t did_gscid  : 24;
      uint64_t pv_pscv    : 1;
      uint64_t dv_gscv    : 1;
      uint64_t idt        : 1;
      uint64_t of         : 1;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;
  };

  union TrReqIova {
    struct {
      uint64_t reserved   : 12;
      uint64_t vpn        : 52;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;
  };

  union TrReqCtl {
    struct {
      uint64_t go_busy    : 1;
      uint64_t priv       : 1;
      uint64_t exe        : 1;
      uint64_t nw         : 1;
      uint64_t reserved0  : 8;
      uint64_t pid        : 20;
      uint64_t pv         : 1;
      uint64_t reserved1  : 3;
      uint64_t custom     : 4;
      uint64_t did        : 24;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;
  };

  union TrResponse {
    struct {
      uint64_t fault      : 1;
      uint64_t reserved0  : 6;
      uint64_t pbmt       : 2;
      uint64_t s          : 1;
      uint64_t ppn        : 44;
      uint64_t reserved1  : 6;
      uint64_t custom     : 4;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;
  };

  union IommuQosid {
    struct {
      uint32_t rcid       : 12;
      uint32_t reserved0  : 4;
      uint32_t mcid       : 12;
      uint32_t reserved1  : 4;
    } fields;
    uint32_t value;
  };

  union Icvec {
    struct {
      uint64_t civ        : 4;
      uint64_t fiv        : 4;
      uint64_t pmiv       : 4;
      uint64_t piv        : 4;
      uint64_t reserved   : 16;
      uint64_t custom     : 32;
    } fields;
    std::array<uint32_t, 2> words;
    uint64_t value;
  };

  union MsiCfgTbl {
    struct {
      uint64_t msi_addr;
      uint32_t msi_data;
      uint32_t msi_vec_ctl;
    } regs;
    std::array<uint32_t, 4> words;
  };


  enum class InvalidationScope {
    GlobalDevice,      // G=1: All entries for this device
    ProcessSpecific,   // PV=1: Entries for specific PID
    AddressSpecific,   // address != 0: Specific page/range
    ProcessAndAddress  // PV=1 && address != 0: Process-specific address
  };

  union PageRequest {

    PageRequest(std::array<uint64_t, 2> value = {})
    : value_(value)
    { }

    std::array<uint64_t, 2> value_;

    struct {
      // value_[0]:
      uint64_t reserved0_ : 12;
      uint64_t pid_       : 20;
      uint64_t pv_        : 1;
      uint64_t priv_      : 1;
      uint64_t exec_      : 1;
      uint64_t reserved1_ : 4;
      uint64_t did_       : 24;
      // value_[1]:
      uint64_t r_         : 1;
      uint64_t w_         : 1;
      uint64_t l_         : 1;
      uint64_t prgi_      : 9;
      uint64_t address_   : 52;
    } bits_;
  };
  static_assert(sizeof(PageRequest) == 16, "PageRequest not 16 bytes in size");

  /// Iommu request: Translation request sent to the IOMMU from a device. Exactly one of
  /// read/write/exec must be true.
  struct IommuRequest
  {
    using PM = PrivilegeMode;

    unsigned devId = 0;       // Device id.
    bool hasProcId = false;   // True if request has a valid process id
    unsigned procId = 0;      // Process Id
    uint64_t iova = 0;        // IO virtual address.

    Ttype type = Ttype::None; // Inbound transaction type. Ttype defined in FaultQueue.hpp.
    PM privMode = PM::User;   // Privilege mode
    unsigned size = 0;        // Size of access in bytes

    /// Return true if this is a translated request: iova is an SPA that is already
    /// translated and need no further translation. Return false if this an untranslated
    /// request (iova needs to be translated). Note that if dc.t2gpa is 1, the iova is a
    /// GPA that would be translated using stage2 translation even though this method
    /// returns true (in this case is-translated means is partially translated).
    bool isTranslated() const  // Translated request
    { return type == Ttype::TransRead or type == Ttype::TransWrite or type == Ttype::TransExec; }

    /// Return true if the request is for a read.
    bool isRead() const
    { return type == Ttype::TransRead or type == Ttype::UntransRead; }

    /// Return true if the request is for a write.
    bool isWrite() const
    { return type == Ttype::TransWrite or type == Ttype::UntransWrite; }

    /// Return true if the request is for a read-for-exec.
    bool isExec() const
    { return type == Ttype::TransExec or type == Ttype::UntransExec; }

    /// Return true if the request is for a PCIE address translation service.
    bool isAts() const
    { return type == Ttype::PcieAts; }

    /// Return true if the request is for a PCIE message request.
    bool isMessage() const
    { return type == Ttype::PcieMessage; }

  };


  /// Model an IOMMU.
  class Iommu
  {
  public:

    /// Constructor: Define an IOMMU with memory mapped registers at the given memory
    /// address covering the memory address range [addr, addr + size - 1]. The
    /// capabilities CSR is set to the given value and the Iommu reset according to the
    /// given capabilities. The constructed object is not usable until the callbacks for
    /// memory access and address translation defined using the callback related methods
    /// below.
    Iommu(uint64_t addr, uint64_t size, uint64_t memorySize, uint64_t capabilities = 0)
      : addr_(addr), size_(size), pmaMgr_(memorySize)
    {
      ddtCache_.resize(DDT_CACHE_SIZE);
      pdtCache_.resize(PDT_CACHE_SIZE);
      capabilities_.value = capabilities;
      reset();
    }

    /// Return true if the memory region of this IOMMU contains the given address.
    bool containsAddr(uint64_t addr) const
    {
      if (addr >= addr_ and addr < addr_ + size_)
        return true;
      return isPmpRegAddr(addr) or isPmaRegAddr(addr);
    }

    /// Return true if the given address is in the physical memory protection (PMP) memory
    /// mapped registers associated with this IOMMU.
    bool isPmpRegAddr(uint64_t addr) const
    { return isPmpcfgAddr(addr) or isPmpaddrAddr(addr); }

    /// Return true if given address in the region associated with the physical memory
    /// protection configuration registers (PMPCFG).
    bool isPmpcfgAddr(uint64_t addr) const
    {
      if (pmpEnabled_)
        return addr >= pmpcfgAddr_ and addr < pmpcfgAddr_ + pmpcfgCount_ * 8;
      return false;
    }

    /// Return true if given address in the region associated with the physical memory
    /// protection address registers (PMPADDR).
    bool isPmpaddrAddr(uint64_t addr) const
    {
      if (pmpEnabled_)
        return addr >= pmpaddrAddr_ and addr < pmpaddrAddr_ + pmpaddrCount_ * 8;
      return false;
    }

    /// Return true if the given address is in the physical memory attribute (PMA) memory
    /// mapped registers associated with this IOMMU.
    bool isPmaRegAddr(uint64_t addr) const
    { return isPmacfgAddr(addr); }

    /// Return true if given address in the region associated with the physical memory
    /// attribute configuration registers (PMACFG).
    bool isPmacfgAddr(uint64_t addr) const
    {
      if (pmaEnabled_)
        return addr >= pmacfgAddr_ and addr < pmacfgAddr_ + pmacfgCount_ * 8;
      return false;
    }

    /// Read a memory mapped register associated with this IOMMU. Return true on
    /// success. Return false leaving value unmodified if addr is not in the range of this
    /// IOMMU or if size/alignment is not valid. For example, if this IOMMMU is mapped at
    /// address x, then calling read(x, 8, value) will set value to that of the
    /// CAPABILITES CSR; and calling read(x+8, 4, value) will set value to that of the
    /// FCTL CSR.
    bool read(uint64_t addr, unsigned size, uint64_t& value) const;

    bool readCsr(uint64_t offset, unsigned size, uint64_t &value) const;

    uint64_t readCapabilities() const               { return capabilities_.value; }
    uint32_t readFctl() const                       { return fctl_.value; }
    uint64_t readDdtp() const                       { return ddtp_.value; }
    uint64_t readCqb() const                        { return cqb_.value; }
    uint32_t readCqh() const                        { return cqh_; }
    uint32_t readCqt() const                        { return cqt_; }
    uint64_t readFqb() const                        { return fqb_.value; }
    uint32_t readFqh() const                        { return fqh_; }
    uint32_t readFqt() const                        { return fqt_; }
    uint64_t readPqb() const                        { return pqb_.value; }
    uint32_t readPqh() const                        { return pqh_; }
    uint32_t readPqt() const                        { return pqt_; }
    uint32_t readCqcsr() const                      { return cqcsr_.value; }
    uint32_t readFqcsr() const                      { return fqcsr_.value; }
    uint32_t readPqcsr() const                      { return pqcsr_.value; }
    uint32_t readIpsr() const                       { return ipsr_.value; }
    uint32_t readIocountovf() const                 { return iocountovf_.value; }
    uint32_t readIocountinh() const                 { return iocountinh_.value; }
    uint64_t readIohpmcycles() const                { return iohpmcycles_.value; }
    uint32_t readTrReqIova() const                  { return tr_req_iova_.value; }
    uint32_t readTrReqCtl() const                   { return tr_req_ctl_.value; }
    uint32_t readTrResponse() const                 { return tr_response_.value; }
    uint32_t readIommuQosid() const                 { return iommu_qosid_.value; }
    uint32_t readIcvec() const                      { return icvec_.value; }
    uint64_t readIohpmctr(unsigned index) const     { return iohpmctr_.at(index-1); }
    uint64_t readIohpmevt(unsigned index) const     { return iohpmevt_.at(index-1).value; }
    uint64_t readMsiAddr(unsigned index) const      { return msi_cfg_tbl_.at(index).regs.msi_addr; }
    uint32_t readMsiData(unsigned index) const      { return msi_cfg_tbl_.at(index).regs.msi_data; }
    uint32_t readMsiVecCtl(unsigned index) const    { return msi_cfg_tbl_.at(index).regs.msi_vec_ctl; }

    /// Write a memory mapped register associated with this IOMMU. Return true on
    /// success. Return false if addr is not in the range of this IOMMU or if
    /// size/alignment is not valid. See the read method for info about addr.
    bool write(uint64_t addr, unsigned size, uint64_t value);

    bool writeCsr(uint64_t offset, unsigned size, uint64_t value);

    /// The following methods write 32- or 64-bit values into the specified CSR according to the
    /// rules outlined in the IOMMU specification. For 64-bit CSRs, either the entire 64-bit value
    /// may be written or just the upper or lower 32 bits. This is determined by the wordMask
    /// parameter: when wordMask is 1, only the least-significant word is written, when 2, only the
    /// upper, and when 3, both. A few of the CSRs have an index parameter which specifies which CSR
    /// in the array to access. For iohpmctr and iohpmevt, these indices are 1-based.
    void writeFctl(uint32_t data);
    void writeDdtp(uint64_t data, unsigned wordMask);
    void writeCqb(uint64_t data, unsigned wordMask);
    void writeCqt(uint32_t data);
    void writeFqb(uint64_t data, unsigned wordMask);
    void writeFqh(uint32_t data);
    void writePqb(uint64_t data, unsigned wordMask);
    void writePqh(uint32_t data);
    void writeCqcsr(uint32_t data);
    void writeFqcsr(uint32_t data);
    void writePqcsr(uint32_t data);
    void writeIpsr(uint32_t data);
    void writeIocountinh(uint32_t data);
    void writeIohpmcycles(uint64_t data, unsigned wordMask);
    void writeTrReqIova(uint64_t data, unsigned wordMask);
    void writeTrReqCtl(uint64_t data, unsigned wordMask);
    void writeIommuQosid(uint32_t data);
    void writeIcvec(uint32_t data);
    void writeIohpmctr(unsigned index, uint64_t data, unsigned wordMask);
    void writeIohpmevt(unsigned index, uint64_t data, unsigned wordMask);
    void writeMsiAddr(unsigned index, uint64_t data, unsigned wordMask);
    void writeMsiData(unsigned index, uint64_t data);
    void writeMsiVecCtl(unsigned index, uint64_t data);

    void signalInterrupt(unsigned vector);
    void updateIpsr(bool newFault = false, bool newPageRequest = false);

    bool processCommand();

    /// Process pending commands in the command queue. This should be called periodically
    /// or when the command queue tail pointer is updated.
    void processCommandQueue();

    /// Perform an address translation request. Return true on success and false on fail.
    /// Report fault cause on fail.
    bool translate(const IommuRequest& req, uint64_t& pa, unsigned& cause);

    /// Perform an ATS (Address Translation Services) translation request. This method
    /// handles PCIe ATS Translation Requests according to RISC-V IOMMU spec section 3.6.
    /// Returns true on success with translated address, false on failure with appropriate
    /// response code. The response parameter contains the ATS completion response fields.
    struct AtsResponse {
      bool success = false;        // True for Success response, false for UR/CA
      bool isCompleterAbort = false; // True for CA, false for UR (when success=false)
      uint64_t translatedAddr = 0; // Translated address (SPA or GPA based on T2GPA)
      bool readPerm = false;       // R bit in ATS completion
      bool writePerm = false;      // W bit in ATS completion
      bool execPerm = false;       // X bit in ATS completion
      bool privMode = false;       // Priv bit in ATS completion
      bool noSnoop = false;        // N bit in ATS completion (always 0 per spec)
      bool cxlIo = false;          // CXL.io bit in ATS completion
      bool global = false;         // Global bit in ATS completion
      uint32_t ama = 0;            // AMA field in ATS completion (default 000b)
      bool untranslatedOnly = false; // U bit - for MRIF mode MSI addresses
    };
    bool atsTranslate(const IommuRequest& req, AtsResponse& response, unsigned& cause);

    void atsPageRequest(const PageRequest& req);

    /// Device calls this when it completes an ATS invalidation request
    /// Per spec: ATS.INVAL command doesn't complete until this is called (or timeout)
    /// @param devId Device ID that completed the invalidation
    /// @param itagVector Bitmap of ITAGs being completed (bit i = 1 means ITAG i completed)
    /// @param completionCount Expected number of completion messages (per PCIe ATS spec)
    void atsInvalidationCompletion(uint32_t devId, uint32_t itagVector, uint8_t completionCount);

    /// Device calls this when an ATS invalidation times out
    /// @param itagVector Bitmap of ITAGs that timed out
    void atsInvalidationTimeout(uint32_t itagVector);

    /// Check if there are pending ATS invalidation requests
    bool hasPendingAtsInvals() const { return anyItagBusy(); }

    /// Perform T2GPA (Two-stage to Guest Physical Address) translation. This method
    /// performs two-stage translation but returns GPA instead of SPA for hypervisor
    /// containment. Used when device context has T2GPA=1.
    bool t2gpaTranslate(const IommuRequest& req, uint64_t& gpa, unsigned& cause);

    /// Perform a memory read operation on behalf of a device. The request is used to
    /// perform address translation and if the translation is successful the system
    /// physical memory is read and the value placed in data. Return true on success and
    /// false on failure. This interface belongs in the bridge but we don't have a bridge
    /// model.
    bool readForDevice(const IommuRequest& req, uint64_t& data, unsigned& cause);

    /// Perform a memory write operation on behalf of a device. The request is used to
    /// perform address translation and if the translation is successful the system
    /// physical memory is written with the provided data. Return true on success and
    /// false on failure. This interface belongs in the bridge but we don't have a bridge
    /// model.
    bool writeForDevice(const IommuRequest& req, uint64_t data, unsigned& cause);

    /// Define a callback to be used by this object to configure the stage1 address
    /// translation step. The callback is invoked by the translate method.
    void setStage1ConfigCb(const std::function<
                           void(unsigned mode, unsigned asid, uint64_t ppn, bool sum)>& cb)
    { stage1Config_ = cb; }

    /// Define a callback to be used by this object to configure the stage2 address
    /// translation step. The callback is invoked by the translate method.
    void setStage2ConfigCb( const std::function<
                            void(unsigned mode, unsigned asid, uint64_t ppn) >& cb)
    { stage2Config_ = cb; }

    /// Define a callback to be used by this object to perform stage1 address translation.
    /// The callback is invoked by the translate method and is expected to return true on
    /// success setting gpa to the translated address and return false on failure setting
    /// cause to the RISCV exception cause (e.g. 1 for load access fault, 13 for load page
    /// fault, etc.) See section 11.3.2. of the RISCV privileged spec.
    void setStage1Cb(const std::function<
                     bool(uint64_t va, unsigned privMode, bool r, bool w, bool x, uint64_t& gpa,
                     unsigned& cause)>& cb)
    { stage1_ = cb; }

    /// Define a callback to be used by this object to perform stage1 address translation.
    /// The callback is invoked by the translate method.
    void setStage2Cb(const std::function<
                     bool(uint64_t gpa, unsigned privMode, bool r, bool w, bool x, uint64_t& pa,
                     unsigned& cause)>& cb)
    { stage2_ = cb; }

    /// Define a callback to be used by this object to read physical memory. The callback
    /// should perform PMA/PMP checks and return true on success (setting data to the read
    /// value) and false on failure.
    void setMemReadCb(const std::function<bool(uint64_t addr, unsigned size, uint64_t& data)>& cb)
    { memRead_ = cb; }

    /// Define a callback to be used by this object to write physical memory. The callback
    /// should perform PMA/PMP checks and return true on success and false on failure.
    void setMemWriteCb(const std::function<bool(uint64_t addr, unsigned size, uint64_t data)>& cb)
    { memWrite_ = cb; }

    /// Define a callback to be used by this object to determine whether or not an
    /// address is readable. The callback is responsible for checking PMA/PMP.
    void setIsReadableCb(const std::function<bool(uint64_t addr, PrivilegeMode mode)>& cb)
    { isReadable_ = cb; }

    /// Define a callback to be used by this object to determine whether or not an
    /// address is writable. The callback is responsible for checking PMA/PMP.
    void setIsWritableCb(const std::function<bool(uint64_t addr, PrivilegeMode mode)>& cb)
    { isWritable_ = cb; }

    void setSendInvalReqCb(const std::function<void(uint32_t devId, uint32_t pid, bool pv, uint64_t address, bool global, InvalidationScope scope, uint8_t itag)> & cb)
    { sendInvalReq_ = cb; }

    void setSendPrgrCb(const std::function<void(uint32_t devId, uint32_t pid, bool pv, uint32_t prgi, uint32_t resp_code, bool dsv, uint32_t dseg)> & cb)
    { sendPrgr_ = cb; }

    /// Configure the capabilities register using a mask.
    void configureCapabilities(uint64_t value);

    /// Reset the IOMMU by resetting all CSRs to their default values.
    void reset();

    /// Define a callback to be used by this object to obtain information about a second
    /// stage address translation trap.
    void setStage2TrapInfoCb(const std::function<void(uint64_t& gpa, bool& implicit, bool& write)>& cb)
    { stage2TrapInfo_ = cb; }

    /// Load device context given a device id. Return true on success and false on
    /// failure. Set cause to failure cause on failure.
    bool loadDeviceContext(unsigned devId, DeviceContext& dc, unsigned& cause);

    /// Load process context given a device context and a process id. Return true on
    /// success and false on failure. Set cause to failure cause on failure.
    bool loadProcessContext(const DeviceContext& dc, unsigned pid,
                            ProcessContext& pc, unsigned& cause);

    /// Overloaded version with device ID for PDT cache support
    bool loadProcessContext(const DeviceContext& dc, unsigned devId, unsigned pid,
                            ProcessContext& pc, unsigned& cause);

    /// Return true if this IOMMU uses wired interrupts. Return false it it uses message
    /// signaled interrupts (MSI). This is for interrupting the core in case of a fault.
    bool wiredInterrupts() const;

    /// Return the device directory table leaf entry size.
    static unsigned devDirTableLeafSize(bool extended)
    { return extended ? sizeof(ExtendedDeviceContext) : sizeof(BaseDeviceContext); }

    /// Return the pagesize.
    unsigned pageSize() const
    { return pageSize_; }

    /// Read physical memory. Byte swap if bigEnd is true. Return true on success. Return
    /// false on failure (Failed PMA/PMP check).
    bool memRead(uint64_t addr, unsigned size, bool bigEnd, uint64_t& data)
    {
      if (size == 0 or size > 8)
        return false;

      if ( ((size - 1) & size) != 0 )
        return false;    // Not a power of 2.

      if (not isPmpReadable(addr, PrivilegeMode::Machine) or not isPmaReadable(addr))
        return false;

      uint64_t val = 0;
      if (not memRead_(addr, size, val))
        return false;

      if (bigEnd)
        {
          val = __builtin_bswap64(val);
          val = val >> ((8 - size)*8);
        }

      data = val;
      return true;
    }

    /// Write physical memory byte-swapping first if bigEnd it true. Return true on
    /// success. Return false on failure (Failed PMA/PMP check).
    bool memWrite(uint64_t addr, unsigned size, bool bigEnd, uint64_t data)
    {
      if (size == 0 or size > 8)
        return false;

      if ( ((size - 1) & size) != 0 )
        return false;    // Not a power of 2.

      if (not isPmpWritable(addr, PrivilegeMode::Machine) or not isPmaWritable(addr))
        return false;

      if (bigEnd)
        {
          data = __builtin_bswap64(data);
          data = data >> ((8 - size)*8);
        }

      return memWrite_(addr, size, data);
    }

    /// Read physical memory. Return true on success. Return false on failure (Failed
    /// PMA/PMP check).
    bool memRead(uint64_t addr, unsigned size, uint64_t& data)
    {
      if (size == 0 or size > 8)
        return false;

      if ( ((size - 1) & size) != 0 )
        return false;    // Not a power of 2.

      if (not isPmpReadable(addr, PrivilegeMode::Machine) or not isPmaReadable(addr))
        return false;

      uint64_t val = 0;
      if (not memRead_(addr, size, val))
        return false;

      data = val;
      return true;
    }

    /// Write physical memory. Return true on success. Return false on failure (Failed
    /// PMA/PMP check).
    bool memWrite(uint64_t addr, unsigned size, uint64_t data)
    {
      if (size == 0 or size > 8)
        return false;

      if ( ((size - 1) & size) != 0 )
        return false;    // Not a power of 2.

      if (not isPmpWritable(addr, PrivilegeMode::Machine) or not isPmaWritable(addr))
        return false;

      return memWrite_(addr, size, data);
    }

    /// If physical memory protection is not enabled, return true; otherwise, return true
    /// if the PMP grants read access for the given address and privilege mode.
    bool isPmpReadable(uint64_t addr, PrivilegeMode mode) const
    {
      if (not pmpEnabled_)
        return true;
      const Pmp& pmp = pmpMgr_.getPmp(addr);
      return pmp.isRead(mode);
    }

    /// If physical memory protection is not enabled, return true; otherwise, return true
    /// if the PMP grants read access for the given address and privilege mode.
    bool isPmpWritable(uint64_t addr, PrivilegeMode mode) const
    {
      if (not pmpEnabled_)
        return true;
      const Pmp& pmp = pmpMgr_.getPmp(addr);
      return pmp.isWrite(mode);
    }

    /// If physical memory attribute is not enabled, return true; otherwise, return true
    /// if the PMA grants read access for the given address.
    bool isPmaReadable(uint64_t addr) const
    {
      if (not pmaEnabled_)
        return true;
      auto pma = pmaMgr_.getPma(addr);
      return pma.isRead();
    }

    /// If physical memory attribute is not enabled, return true; otherwise, return true
    /// if the PMA grants read access for the given address.
    bool isPmaWritable(uint64_t addr) const
    {
      if (not pmaEnabled_)
        return true;
      auto pma = pmaMgr_.getPma(addr);
      return pma.isWrite();
    }

    /// Return true if device context has extended format.
    bool isDcExtended() const
    { return capabilities_.fields.msi_flat; }

    /// Return true if the device directory table is big endian.
    bool devDirTableBe() const
    { return fctl_.fields.be; }  // Cached FCTL.BE

    /// Return true if the device directory table is big endian.
    bool devDirBigEnd() const
    { return fctl_.fields.be; }  // Cached FCTL.BE

    /// Return true if the second-stage page table is big endian.
    bool stage2BigEnd() const
    { return fctl_.fields.be; }

    /// Return true if the MIS page table is big endian.
    bool msiBigEnd() const
    { return fctl_.fields.be; }

    /// Return true if the fault-queue is big endina.
    bool faultQueueBigEnd() const
    { return fctl_.fields.be; }

    /// Read the process context at the given address following the endianness specified
    /// by the given device context. Return true on success and false on failure.
    bool readProcessContext(const DeviceContext& dc, uint64_t addr, ProcessContext& pc)
    {
      uint64_t ta = 0, fsc = 0;
      bool bigEnd = dc.sbe();
      if (not memReadDouble(addr, bigEnd, ta) or not memReadDouble(addr+8, bigEnd, fsc))
        return false;
      pc.set(ta, fsc);
      return true;
    }

    /// Write the given process directory table entry the given address following the
    /// endianness specified by the given device context. Return true on success and false
    /// on failure.
    bool writeProcDirTableEntry(const DeviceContext& dc, uint64_t addr, uint64_t pdte)
    {
      bool bigEnd = dc.sbe();
      return memWriteDouble(addr, bigEnd, pdte);
    }

    /// Write the given process context to the given address following the endianness
    /// specified by the given device context. Return true on success and false on
    /// failure.
    bool writeProcessContext(const DeviceContext& dc, uint64_t addr, const ProcessContext& pc)
    {
      bool bigEnd = dc.sbe();
      return ( memWriteDouble(addr, bigEnd, pc.ta()) and
               memWriteDouble(addr+8, bigEnd, pc.fsc()) );
    }

    /// Write the given device directory table entry to the given address honoring the
    /// endinaness of the device directory table. Return true on success and false on
    /// failure.
    bool writeDevDirTableEntry(uint64_t addr, uint64_t ddte)
    {
      bool bigEnd = devDirTableBe();
      return memWriteDouble(addr, bigEnd, ddte);
    }

    /// Write to memory, at the given address, he base/extended part of the given device
    /// context based on whether or not the device is extended. Honor the endianness of the
    /// device directory table. Return true on success and false on failure.
    bool writeDeviceContext(uint64_t addr, const DeviceContext& dc)
    {
      bool bigEnd = devDirTableBe();

      bool ok = memWriteDouble(addr, bigEnd, dc.transControl().value_);
      addr += 8;
      ok = memWriteDouble(addr, bigEnd, dc.iohgatp()) and ok;
      addr += 8;
      ok = memWriteDouble(addr, bigEnd, dc.transAttrib().value_) and ok;
      addr += 8;
      ok = memWriteDouble(addr, bigEnd, dc.firstStageContext()) and ok;
      addr += 8;

      if (isDcExtended())
        {
          ok = memWriteDouble(addr, bigEnd, dc.msiTablePointer()) and ok;
          addr += 8;
          ok = memWriteDouble(addr, bigEnd, dc.fullMsiMask()) and ok;
          addr += 8;
          ok = memWriteDouble(addr, bigEnd, dc.fullMsiPattern()) and ok;
          addr += 8;
          ok = memWriteDouble(addr, bigEnd, 0) and ok; // Reserved field.
        }
      return ok;
    }

    /// Fill the given vector with the address/value pairs corresponding to the ddte
    /// entries visited in the last device directory walk (loadDeviceContext).
    void lastDeviceDirectoryWalk(std::vector<std::pair<uint64_t, uint64_t>> &walk) const
    { walk = deviceDirWalk_; }

    /// Fill the given vector with the address/value pairs corresponding to the pdte
    /// entries visited in the last process directory walk (loadDeviceContext).
    void lastProcessDirectoryWalk(std::vector<std::pair<uint64_t, uint64_t>> &walk) const
    { walk = processDirWalk_; }

    /// Return true if the given command is an ATS command (has the correct opcode).
    static bool isAtsCommand(const AtsCommand& cmd)
    { return cmd.isAts(); }

    static bool isAtsInvalCommand(const AtsCommand& cmd)
    { return cmd.isInval(); }

    static bool isAtsPrgrCommand(const AtsCommand& cmd)
    { return cmd.isPrgr(); }

    /// Return true if the given command is an IODIR command (has the correct opcode).
    static bool isIodirCommand(const AtsCommand& cmd)
    { return cmd.isIodir(); }

    /// Return true if the given command is an IOFENCE command (has the correct opcode).
    static bool isIofenceCommand(const AtsCommand& cmd)
    { return cmd.isIofence(); }

    static bool isIofenceCCommand(const AtsCommand& cmd)
    { return cmd.isIofenceC(); }

    /// Return true if the given command is an IOTINVAL command (has the correct opcode).
    static bool isIotinvalCommand(const AtsCommand& cmd)
    { return cmd.isIotinval(); }

    static bool isIotinvalVmaCommand(const AtsCommand& cmd)
    { return cmd.isIotinvalVma(); }

    static bool isIotinvalGvmaCommand(const AtsCommand& cmd)
    { return cmd.isIotinvalGvma(); }

    /// Execute an ATS.INVAL command for address translation cache invalidation.
    /// Returns true if the command completed and the queue head should advance.
    /// Returns false if blocked waiting for ITAG availability.
    bool executeAtsInvalCommand(const AtsCommand& cmd);

    /// Execute an ATS.PRGR command for page request group response
    void executeAtsPrgrCommand(const AtsCommand& cmd);

    /// Execute an IODIR command
    void executeIodirCommand(const AtsCommand& cmdData);

    /// Execute an IOFENCE.C command for command queue fence.
    /// Returns true if the command completed and the queue head should advance.
    /// Returns false if waiting for invalidations, reporting timeout, or memory fault.
    bool executeIofenceCCommand(const AtsCommand& cmdData);

    /// Retry a pending IOFENCE.C command after ATS invalidations complete.
    /// Returns true if the IOFENCE completed successfully, false if still waiting or failed.
    bool retryPendingIofence();

    /// Helper function to execute the core IOFENCE.C logic (timeout check, memory ops, interrupt).
    /// Returns true if completed successfully, false if timeout reporting or memory fault.
    bool executeIofenceCCore(bool pr, bool pw, bool av, bool wsi, uint64_t addr, uint32_t data);

    /// Wait for all pending ATS invalidation requests to complete (legacy, for compatibility)
    /// Called by IOFENCE.C per spec requirement
    void waitForPendingAtsInvals();

    /// Execute an IOTINVAL command for page table cache invalidation (VMA or GVMA)
    void executeIotinvalCommand(const AtsCommand& cmdData);


    /// Define the physical memory protection registers (pmp-config regs and pmp-addr
    /// regs). The registers are memory mapped at the given addresses.
    /// Return true on success and false on failure (addresses not double word aligned,
    /// counts are too large, counts are not consistent...).
    bool definePmpRegs(uint64_t pmpcfgAddr, unsigned pmpcfgCount,
                       uint64_t pmpaddrAddr, unsigned pmpaddrCount);

    /// Define the physical memory attribute registers (PMACFG).  The registers are memory
    /// mapped at the given address.  Return true on success and false on failure (address
    /// is not double word aligned, count too large...).
    bool definePmaRegs(uint64_t pmacfgAddr, unsigned pmacfgCount);

    bool dsv_ = false;

  protected:

    /// Helper to translate. Does translation but does not report fault cause on fail,
    /// instead, it sets repFault to true if a fault should be reported.
    bool translate_(const IommuRequest& req, uint64_t& pa, unsigned& cause,
                    bool& repFault);

    /// Return true if given device context is mis-configured. See section 2.1.4 of IOMMMU
    /// spec.
    bool misconfiguredDc(const DeviceContext& dc) const;

    /// Return true if given process context is mis-configured. See section 2.2.4 of
    /// IOMMMU spec.
    bool misconfiguredPc(const ProcessContext& pc, bool sxl) const;

    /// Define the constrol and status registers associated with this IOMMU
    void defineCsrs();

    /// Translate guest physical address gpa into host address pa using the MSI address
    /// translation process.
    bool msiTranslate(const DeviceContext& dc, const IommuRequest& req, uint64_t gpa,
                      uint64_t& pa, bool& isMrif, uint64_t& mrif, uint64_t& nnpn,
                      unsigned& nid, unsigned& cause);

    /// Riscv stage 1 address translation.
    bool stage1Translate(uint64_t iosatp, uint64_t iohgatp, PrivilegeMode pm, unsigned procId,
                         bool r, bool w, bool x, bool sum,
                         uint64_t va, uint64_t& gpa, unsigned& cause);

    /// Riscv stage 2 address translation.
    bool stage2Translate(uint64_t iohgatp, PrivilegeMode pm, bool r, bool w, bool x,
                         uint64_t gpa, uint64_t& pa, unsigned& cause);

    /// Read a double word from physical memory. Byte swap if bigEnd is true. Return true
    /// on success. Return false on failure (failed PMA/PMP check).
    bool memReadDouble(uint64_t addr, bool bigEnd, uint64_t& data)
    {
      uint64_t val = 0;
      if (not memRead(addr, 8, val))
        return false;
      data = bigEnd ? __builtin_bswap64(val) : val;
      return true;
    }

    /// Write a double word from physical memory. Byte swap if bigEnd is true. Return true
    /// on success. Return false on failure (failed PMA/PMP check).
    bool memWriteDouble(uint64_t addr, bool bigEnd, uint64_t data)
    {
      uint64_t val = bigEnd ? __builtin_bswap64(data) : data;
      return memWrite(addr, 8, val);
    }

    bool cqFull() const { return (cqt_ + 1) % cqb_.capacity() == cqh_; }
    bool fqFull() const { return (fqt_ + 1) % fqb_.capacity() == fqh_; }
    bool pqFull() const { return (pqt_ + 1) % pqb_.capacity() == pqh_; }

    bool cqEmpty() const { return cqt_ == cqh_; }
    bool fqEmpty() const { return fqt_ == fqh_; }
    bool pqEmpty() const { return pqt_ == pqh_; }

    /// Write given fault record to the fault queue which must not be full.
    void writeFaultRecord(const FaultRecord& record);

    void writePageRequest(const PageRequest& req);

    /// Called after a PMPCFG/PMPADDR CSR is changed to update the cached memory
    /// protection in PmpManager.
    void updateMemoryProtection();

    /// Called after a PMACFG CSR is changed to update the cached memory attributes in
    /// PmaManager.
    void updateMemoryAttributes(unsigned pmacfgIx);

    /// Check if CIP should be set based on CQCSR
    /// conditions. Returns true if cie=1 and any error condition is present.
    bool shouldSetCip() const;

    /// Set CIP bit in IPSR if conditions are met (called when CQCSR error bits change).
    void updateCip();

    /// Check if FIP should be set based on FQCSR
    /// conditions. Returns true if fie=1 and any error condition is present or new record added.
    bool shouldSetFip() const;

    /// Set FIP bit in IPSR if conditions are met (called when FQCSR error bits change or record added).
    void updateFip();

    /// Check if PIP should be set based on PQCSR conditions.
    bool shouldSetPip() const;

    /// Set PIP bit in IPSR if conditions are met.
    void updatePip();

    /// Return the configuration byte of a PMPCFG register corresponding to the PMPADDR
    /// register having the given index (index 0 corresponds to PMPADDR0). Given index
    /// must not be out of bouds.
    uint8_t getPmpcfgByte(unsigned pmpaddrIx) const
    {
      assert(pmpaddrIx < pmpaddrCount_);
      unsigned cfgIx = pmpaddrIx / 8;  // 1 PMPCFG reg for 8 PMPADDR regs.
      uint64_t cfgVal = pmpcfg_.at(cfgIx);
      unsigned cfgByteIx = pmpaddrIx % 8;
      uint8_t cfgByte = cfgVal >> (8*cfgByteIx);
      return cfgByte;
    }

  private:

    bool beWritable_ = true;
    bool wsiWritable_ = true;
    bool gxlWritable_ = true;

    Capabilities    capabilities_{};
    Fctl            fctl_{};
    Ddtp            ddtp_{};
    Cqb             cqb_{};
    uint32_t        cqh_{};
    uint32_t        cqt_{};
    Fqb             fqb_{};
    uint32_t        fqh_{};
    uint32_t        fqt_{};
    Pqb             pqb_{};
    uint32_t        pqh_{};
    uint32_t        pqt_{};
    Cqcsr           cqcsr_{};
    Fqcsr           fqcsr_{};
    Pqcsr           pqcsr_{};
    Ipsr            ipsr_{};
    Iocountovf      iocountovf_{};
    Iocountinh      iocountinh_{};
    Iohpmcycles     iohpmcycles_{};
    std::array<uint64_t, 31> iohpmctr_{};
    std::array<Iohpmevt, 31> iohpmevt_{};
    TrReqIova       tr_req_iova_{};
    TrReqCtl        tr_req_ctl_{};
    TrResponse      tr_response_{};
    IommuQosid      iommu_qosid_{};
    Icvec           icvec_{};
    std::array<MsiCfgTbl, 16> msi_cfg_tbl_{};

    // This array says at which word offsets 4 and 8 byte accesses may be performed. A 4 byte access
    // may be performed to any offset at which an 8 byte access may be performed but the reverse is
    // not true. Reserved and custome offsets are indicated with 0.
    static constexpr std::array<unsigned, 1024/4> sizeAtWordOffset_ = {
//    0     8     16    24    32    40    48    56    64    72    80    88    96    104   112   120
      8, 4, 4, 0, 8, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 4, 8, 4, 8, 4, 8, 4, //   0 - 127
      8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, // 128 - 255
      8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, // 256 - 383
      8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, // 384 - 511
      8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 8, 4, 4, 0, 0, 0, // 512 - 639
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 4, // 640 - 767
      8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, // 768 - 895
      8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, 8, 4, 4, 4, // 896 - 1023
    };

    uint64_t addr_;      // Address of this IOMMU in memory
    uint64_t size_;      // Size in bytes of IOMMU memory region

    const unsigned pageSize_ = 4096;

    // Address/ddte-value pairs of last device directory walk (loadDeviceContext).
    std::vector<std::pair<uint64_t, uint64_t>> deviceDirWalk_;

    // Address/pdte-value pairs of last process directory walk (loadDeviceContext).
    std::vector<std::pair<uint64_t, uint64_t>> processDirWalk_;

    std::function<bool(uint64_t addr, unsigned size, uint64_t& data)> memRead_ = nullptr;
    std::function<bool(uint64_t addr, unsigned size, uint64_t data)> memWrite_ = nullptr;

    std::function<bool(uint64_t addr, PrivilegeMode mode)> isReadable_ = nullptr;
    std::function<bool(uint64_t addr, PrivilegeMode mode)> isWritable_ = nullptr;

    std::function<void(unsigned mode, unsigned asid, uint64_t ppn, bool sum)> stage1Config_ = nullptr;
    std::function<void(unsigned mode, unsigned asid, uint64_t ppn)> stage2Config_ = nullptr;

    std::function<bool(uint64_t va, unsigned privMode, bool r, bool w, bool x, uint64_t& gpa,
      unsigned& cause)> stage1_ = nullptr;

    std::function<bool(uint64_t gpa, unsigned privMode, bool r, bool w, bool x, uint64_t& pa,
      unsigned& cause)> stage2_ = nullptr;

    std::function<void(uint64_t& gpa, bool& implicit, bool& write)> stage2TrapInfo_ = nullptr;

    std::function<void(uint32_t devId, uint32_t pid, bool pv, uint64_t address, bool global, InvalidationScope scope, uint8_t itag)> sendInvalReq_ = nullptr;
    std::function<void(uint32_t devId, uint32_t pid, bool pv, uint32_t prgi, uint32_t resp_code, bool dsv, uint32_t dseg)> sendPrgr_ = nullptr;


    bool pmpEnabled_ = false;        // Physical memory protection (PMP)
    uint64_t pmpcfgCount_ = 0;       // Number of PMPCFG registers
    uint64_t pmpaddrCount_ = 0;      // Number of PMPADDR registers
    uint64_t pmpcfgAddr_ = 0;        // Address of first PMPCFG register
    uint64_t pmpaddrAddr_ = 0;       // Address of first PMPADDR register

    std::vector<uint64_t> pmpcfg_;   // Cached values of PMPCFG registers
    std::vector<uint64_t> pmpaddr_;  // Cached values of PMPADDR registers

    PmpManager pmpMgr_;

    bool pmaEnabled_ = false;        // Physical memory attributes (PMA)
    uint64_t pmacfgCount_ = 0;       // Count of PMACFG registers
    uint64_t pmacfgAddr_ = 0;        // Address of first PMACFG register

    std::vector<uint64_t> pmacfg_;   // Cached values of PMACFG registers

    PmaManager pmaMgr_;

    // IOMMU Directory Cache structures for DDT and PDT caching
    struct DdtCacheEntry {
      uint32_t deviceId{0};
      DeviceContext deviceContext;
      uint64_t timestamp{0};  // For LRU eviction
      bool valid{false};

      DdtCacheEntry() = default;
    };

    struct PdtCacheEntry {
      uint32_t deviceId{0};
      uint32_t processId{0};
      ProcessContext processContext;
      uint64_t timestamp{0};  // For LRU eviction
      bool valid{false};

      PdtCacheEntry() = default;
    };

    // Directory caches - configurable size, default to reasonable values
    static const size_t DDT_CACHE_SIZE = 64;  // Number of DDT entries to cache
    static const size_t PDT_CACHE_SIZE = 128; // Number of PDT entries to cache

    std::vector<DdtCacheEntry> ddtCache_;
    std::vector<PdtCacheEntry> pdtCache_;
    uint64_t cacheTimestamp_ = 0;  // Global timestamp for LRU

    /// Invalidate DDT cache entries based on device ID and DV flag
    void invalidateDdtCache(uint32_t deviceId, bool dv);

    /// Invalidate PDT cache entries based on device ID and process ID
    void invalidatePdtCache(uint32_t deviceId, uint32_t processId);

    /// Find DDT cache entry for given device ID
    DdtCacheEntry* findDdtCacheEntry(uint32_t deviceId);

    /// Find PDT cache entry for given device ID and process ID
    PdtCacheEntry* findPdtCacheEntry(uint32_t deviceId, uint32_t processId);

    /// Add or update DDT cache entry
    void updateDdtCache(uint32_t deviceId, const DeviceContext& dc);

    /// Add or update PDT cache entry
    void updatePdtCache(uint32_t deviceId, uint32_t processId, const ProcessContext& pc);

    // ATS Invalidation tracking using ITAGs (per spec: commands don't complete until device responds)
    // ITAG = Invalidation Tag, a hardware resource for tracking outstanding ATS invalidation requests
    struct ITagTracker {
      bool busy = false;              // Is this ITAG currently tracking a request?
      bool dsv = false;               // Destination segment valid
      uint8_t dseg = 0;               // Destination segment number
      uint16_t rid = 0;               // Requester ID (device function)
      uint32_t devId = 0;             // Full device ID (dseg << 16 | rid)
      bool pv = false;                // PASID valid
      uint32_t pid = 0;               // Process ID (PASID)
      uint64_t address = 0;           // Address being invalidated
      bool global = false;            // Global invalidation flag
      InvalidationScope scope = InvalidationScope::GlobalDevice;  // Invalidation scope
      uint8_t numRspRcvd = 0;         // Number of completion responses received
    };

    // Maximum number of ITAGs (matches riscv-iommu reference implementation)
    static constexpr size_t MAX_ITAGS = 2;
    std::array<ITagTracker, MAX_ITAGS> itagTrackers_;

    // Command queue stall state
    bool cqStalledForItag_ = false;           // CQ stalled waiting for free ITAG
    bool iofenceWaitingForInvals_ = false;    // IOFENCE waiting for ITAGs to complete
    bool atsInvalTimeout_ = false;            // At least one ATS.INVAL timed out

    // Blocked request storage (when no ITAG available)
    struct BlockedAtsInval {
      uint32_t devId;
      uint32_t pid;
      bool pv;
      bool dsv;
      uint8_t dseg;
      uint16_t rid;
      uint64_t address;
      bool global;
      InvalidationScope scope;
    };
    std::optional<BlockedAtsInval> blockedAtsInval_;

    // IOFENCE parameters (saved when IOFENCE needs to wait for invalidations)
    struct PendingIofence {
      bool pr, pw, av, wsi;
      uint64_t addr;
      uint32_t data;
    };
    std::optional<PendingIofence> pendingIofence_;

    // ITAG helper functions
    /// Allocate an ITAG for a new invalidation request
    /// Returns true if allocation succeeded, false if no ITAGs available
    bool allocateItag(uint32_t devId, bool dsv, uint8_t dseg, uint16_t rid,
                      bool pv, uint32_t pid, uint64_t address, bool global,
                      InvalidationScope scope, uint8_t& itag);

    /// Check if any ITAGs are currently busy (tracking outstanding requests)
    bool anyItagBusy() const;

    /// Count how many ITAGs are currently busy
    size_t countBusyItags() const;

    /// Try to retry a blocked ATS.INVAL command if an ITAG becomes available
    void retryBlockedAtsInval();
  };

}
