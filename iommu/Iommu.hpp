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
#include "IommuCsr.hpp"
#include "DeviceContext.hpp"
#include "ProcessContext.hpp"
#include "FaultQueue.hpp"
#include "Ats.hpp"
#include "IommuPmpManager.hpp"
#include "IommuPmaManager.hpp"


namespace TT_IOMMU
{

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
    /// address covering the memory address range [addr, addr + size - 1]. The constructed
    /// object is not usable until the callbacks for memory access and address translation
    /// are defined using the callback related methods below. After this object is
    /// constructed, the capabilities CSR should be configured by calling the
    /// configureCapabilites method and then the reset method should be called to reset
    /// this object according to the configured capabilities.
    Iommu(uint64_t addr, uint64_t size, uint64_t memorySize)
      : addr_(addr), size_(size), pmaMgr_(memorySize)
    { 
      wordToCsr_.resize(size / 4, nullptr);
      ddtCache_.resize(DDT_CACHE_SIZE);
      pdtCache_.resize(PDT_CACHE_SIZE);
      defineCsrs();
    }

    /// Constructor: Define an IOMMU with memory mapped registers at the given memory
    /// address covering the memory address range [addr, addr + size - 1]. The
    /// capabilities CSR is set to the given value and the Iommu reset according to the
    /// given capabilities. The constructed object is not usable until the callbacks for
    /// memory access and address translation defined using the callback related methods
    /// below.
    Iommu(uint64_t addr, uint64_t size, uint64_t memorySize, uint64_t capabilities)
      : addr_(addr), size_(size), pmaMgr_(memorySize)
    {
      wordToCsr_.resize(size / 4, nullptr);
      ddtCache_.resize(DDT_CACHE_SIZE);
      pdtCache_.resize(PDT_CACHE_SIZE);
      defineCsrs();
      configureCapabilities(capabilities);
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

    /// Write a memory mapped register associated with this IOMMU. Return true on
    /// success. Return false if addr is not in the range of this IOMMU or if
    /// size/alignment is not valid. See the read method for info about addr.
    bool write(uint64_t addr, unsigned size, uint64_t value);

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

    void applyCapabilityRestrictions();

    /// Define a callback to be used by this object to obtain information about a second
    /// stage address translation trap.
    void setStage2TrapInfoCb(const std::function<void(uint64_t& gpa, bool& implicit, bool& write)>& cb)
    { stage2TrapInfo_ = cb; }

    /// Return the memory address associated with the given CSR number. This is useful for
    /// testing.
    uint64_t getCsrAddress(CsrNumber csrn) const
    { return addr_ + csrs_.at(size_t(csrn)).offset_; }

    /// Return the base address of the queue associated with the given queue base CSR.
    /// Public wrapper for testing.
    uint64_t getQueueAddress(CsrNumber qbase) const
    { return queueAddress(qbase); }

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

    /// Read the given CSR. If the CSR is of size 4, the top 32 bits of the result
    /// will be zero.
    uint64_t readCsr(CsrNumber csrn) const
    {
      const auto& csr = csrs_.at(unsigned(csrn));
      uint64_t val = csr.read();
      if (csr.size() == 4)
        val = (val << 32) >> 32;
      return val;
    }

    /// Write the given CSR. If the CSR is of size 4, the top 32 bits of data are ignored.
    /// This method honors the RW1C/RW1S field attributes (read-write-1-clear and
    /// read-write-1-set).
    void writeCsr(CsrNumber csrn, uint64_t data);

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
    { return Capabilities{readCsr(CsrNumber::Capabilities)}.bits_.msiFlat_; }

    /// Return true if the device directory table is big endian.
    bool devDirTableBe() const
    { return fctlBe_; }  // Cached FCTL.BE

    /// Return true if the device directory table is big endian.
    bool devDirBigEnd() const
    { return fctlBe_; }  // Cached FCTL.BE

    /// Return true if the second-stage page table is big endian.
    bool stage2BigEnd() const
    { return fctlBe_; }

    /// Return true if the MIS page table is big endian.
    bool msiBigEnd() const
    { return fctlBe_; }

    /// Return true if the fault-queue is big endina.
    bool faultQueueBigEnd() const
    { return fctlBe_; }

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

    /// Execute an ATS.INVAL command for address translation cache invalidation
    void executeAtsInvalCommand(const AtsCommand& cmd);

    /// Execute an ATS.PRGR command for page request group response
    void executeAtsPrgrCommand(const AtsCommand& cmd);

    /// Execute an IODIR command
    void executeIodirCommand(const AtsCommand& cmdData);

    /// Execute an IOFENCE.C command for command queue fence
    void executeIofenceCCommand(const AtsCommand& cmdData);

    /// Retry a pending IOFENCE.C command after ATS invalidations complete
    void retryPendingIofence();

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

    /// Return a pointer the the CSR covering the given address. Return nullptr if no such
    /// CSR or if the address is not word aligned.
    IommuCsr* findCsrByAddr(uint64_t addr)
    {
      if ((addr & 3) != 0 or addr < addr_ or addr - addr_ >= size_)
	return nullptr;

      uint64_t wordIx = (addr - addr_) / 4;
      return wordToCsr_.at(wordIx);
    }

    /// Return a pointer the the CSR covering the given address. Return nullptr if no such
    /// CSR or if the address is not word aligned.
    const IommuCsr* findCsrByAddr(uint64_t addr) const
    {
      if ((addr & 3) != 0 or addr < addr_ or addr - addr_ >= size_)
	return nullptr;

      uint64_t wordIx = (addr - addr_) / 4;
      return wordToCsr_.at(wordIx);
    }

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

    /// Return CSR having the given number n.
    IommuCsr& csrAt(CsrNumber n)
    { return csrs_.at(unsigned(n)); }

    /// Return CSR having the given number n.
    const IommuCsr& csrAt(CsrNumber n) const
    { return csrs_.at(unsigned(n)); }

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

    /// Return the queue capacity (buffer size) associated with the given queue base CSR
    /// (cqb, fqb, or pqb). Return value is the total number of entries (not bytes) in the
    /// queue buffer (currently used or otherwise).
    uint64_t queueCapacity(CsrNumber qbase) const;

    /// Return the base address of the queue associated with the given queue base CSR
    /// (cqb, fqb, or pqb).
    uint64_t queueAddress(CsrNumber qbase) const;

    /// Return true if the queue associated with the given queue base/head/tail CSRs is
    /// full.
    bool queueFull(CsrNumber qbase, CsrNumber qhead, CsrNumber qtail) const;

    /// Return true if the queu associated with the given queue base/head/tail CSR is
    /// full.
    bool queueEmpty(CsrNumber qbase, CsrNumber qhead, CsrNumber qtail) const;

    /// Similar to writeCsr but is not affected by RW1C/RW1S.
    void pokeCsr(CsrNumber csrn, uint64_t data);

    /// Special case of pokeCsr for IPSR.
    void pokeIpsr(uint64_t value);

    /// Special case of writeCsr for IPSR.
    void writeIpsr(uint64_t value);

    /// Write given fault record to the fault queue which must not be full.
    void writeFaultRecord(const FaultRecord& record);

    void writePageRequest(const PageRequest& req);

    /// Called after a PMPCFG/PMPADDR CSR is changed to update the cached memory
    /// protection in PmpManager.
    void updateMemoryProtection();

    /// Called after a PMACFG CSR is changed to update the cached memory attributes in
    /// PmaManager.
    void updateMemoryAttributes(unsigned pmacfgIx);

  

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

    uint64_t addr_;      // Address of this IOMMU in memory
    uint64_t size_;      // Size in bytes of IOMMU memory region
    std::vector<IommuCsr> csrs_;
    bool bigEnd_ = false;   // True if big-endian set in cabalities.

    bool fctlBe_ = false;   // Big endian control for dev dir table and 2nd stage table

    const unsigned pageSize_ = 4096;

    // Address/ddte-value pairs of last device directory walk (loadDeviceContext).
    std::vector<std::pair<uint64_t, uint64_t>> deviceDirWalk_;

    // Address/pdte-value pairs of last process directory walk (loadDeviceContext).
    std::vector<std::pair<uint64_t, uint64_t>> processDirWalk_;

    // Map a word (4 bytes) in the IOMMU memory region to the index of the CSR covering at
    // word.  Words 0 and 1 would map to the capabilities CSR, word 2 to the FCNTL CSR,
    // ...
    std::vector<IommuCsr*> wordToCsr_;

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
    
    mutable std::vector<DdtCacheEntry> ddtCache_;
    mutable std::vector<PdtCacheEntry> pdtCache_;
    mutable uint64_t cacheTimestamp_ = 0;  // Global timestamp for LRU

    /// Invalidate DDT cache entries based on device ID and DV flag
    void invalidateDdtCache(uint32_t deviceId, bool dv);
    
    /// Invalidate PDT cache entries based on device ID and process ID
    void invalidatePdtCache(uint32_t deviceId, uint32_t processId);
    
    /// Find DDT cache entry for given device ID
    DdtCacheEntry* findDdtCacheEntry(uint32_t deviceId) const;
    
    /// Find PDT cache entry for given device ID and process ID
    PdtCacheEntry* findPdtCacheEntry(uint32_t deviceId, uint32_t processId) const;
    
    /// Add or update DDT cache entry
    void updateDdtCache(uint32_t deviceId, const DeviceContext& dc) const;
    
    /// Add or update PDT cache entry
    void updatePdtCache(uint32_t deviceId, uint32_t processId, const ProcessContext& pc) const;

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
    mutable std::array<ITagTracker, MAX_ITAGS> itagTrackers_;

    // Command queue stall state
    mutable bool cqStalledForItag_ = false;           // CQ stalled waiting for free ITAG
    mutable bool iofenceWaitingForInvals_ = false;    // IOFENCE waiting for ITAGs to complete
    mutable bool atsInvalTimeout_ = false;            // At least one ATS.INVAL timed out

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
    mutable std::optional<BlockedAtsInval> blockedAtsInval_;

    // IOFENCE parameters (saved when IOFENCE needs to wait for invalidations)
    struct PendingIofence {
      bool pr, pw, av, wsi;
      uint64_t addr;
      uint32_t data;
    };
    mutable std::optional<PendingIofence> pendingIofence_;

    // ITAG helper functions
    /// Allocate an ITAG for a new invalidation request
    /// Returns true if allocation succeeded, false if no ITAGs available
    bool allocateItag(uint32_t devId, bool dsv, uint8_t dseg, uint16_t rid,
                      bool pv, uint32_t pid, uint64_t address, bool global,
                      InvalidationScope scope, uint8_t& itag) const;

    /// Check if any ITAGs are currently busy (tracking outstanding requests)
    bool anyItagBusy() const;

    /// Count how many ITAGs are currently busy
    size_t countBusyItags() const;

    /// Try to retry a blocked ATS.INVAL command if an ITAG becomes available
    void retryBlockedAtsInval() const;
  };

}
