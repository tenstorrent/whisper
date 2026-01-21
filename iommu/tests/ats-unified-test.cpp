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
#include "DeviceContext.hpp"
#include "ProcessContext.hpp"
#include "Ats.hpp"
#include "MemoryModel.hpp"
#include "IommuStructures.hpp"
#include "MemoryManager.hpp"
#include "TableBuilder.hpp"
#include <iostream>
#include <cassert>
#include <cstring>
#include <map>
#include <memory>

using namespace TT_IOMMU;
using namespace IOMMU;

class AtsTestHelper
{
public:
  AtsTestHelper() : mem_(size_t(256) * 1024 * 1024), // 256MB memory
                    readFunc_([this](uint64_t addr, unsigned size, uint64_t& data, bool& corrupted) {
                        corrupted = false;
                        return mem_.read(addr, size, data);
                    }),
                    writeFunc_([this](uint64_t addr, unsigned size, uint64_t data) {
                        return mem_.write(addr, size, data);
                    }),
                    tableBuilder_(memMgr_, readFunc_, writeFunc_)
  {
    setupIommu();
  }

  Iommu& getIommu() { return *iommu_; }
  MemoryModel& getMemory() { return mem_; }
  TableBuilder& getTableBuilder() { return tableBuilder_; }
  MemoryManager& getMemoryManager() { return memMgr_; }

  // Create a device context using TableBuilder
  void setupDeviceContextWithBuilder(uint32_t devId, bool enableAts = true, bool enableT2gpa = false)
  {
    std::cout << "[ATS_HELPER] Setting up device context for ID 0x" << std::hex << devId
              << " with ATS=" << (enableAts ? "enabled" : "disabled") << std::dec << '\n';

    // Check if DDTP is already configured; if so, reuse it instead of creating a new root
    uint64_t ddtpValue = iommu_->readDdtp();
    Ddtp ddtp{ .value = ddtpValue };

    if (ddtp.fields.iommu_mode == Ddtp::Mode::Off || ddtp.fields.iommu_mode == Ddtp::Mode::Bare) {
      // DDTP not yet configured, set up a new 2-level DDT root
      ddtp.fields.iommu_mode = Ddtp::Mode::Level2;
      ddtp.fields.ppn = memMgr_.getFreePhysicalPages(1);
      iommu_->writeDdtp(ddtp.value, 3);
      std::cout << "[DEBUG] Created new DDTP: 0x" << std::hex << ddtp.value << std::dec << '\n';
    } else {
      // DDTP already configured, reuse existing root
      std::cout << "[DEBUG] Reusing existing DDTP: 0x" << std::hex << ddtp.value
                << ", mode: " << static_cast<int>(ddtp.fields.iommu_mode) << std::dec << '\n';
    }

    // Create device context with ATS configuration
    ExtendedDeviceContext dc = {};
    dc.tc_ = 0x1; // Valid device context

    // Configure ATS
    if (enableAts) {
        dc.tc_ |= 0x2; // Enable ATS bit
    }

    // Configure T2GPA
    if (enableT2gpa) {
        dc.tc_ |= 0x8; // Enable T2GPA bit

        // Set up IOHGATP for G-stage translation
        TT_IOMMU::Iohgatp iohgatp;
        iohgatp.bits_.mode_ = TT_IOMMU::IohgatpMode::Sv39x4;
        iohgatp.bits_.gcsid_ = 0;
        iohgatp.bits_.ppn_ = memMgr_.getFreePhysicalPages(4);
        dc.iohgatp_ = iohgatp.value_;
    } else {
        // Bare mode - no G-stage translation
        dc.iohgatp_ = 0;
    }

    // Set up first-stage context - direct IOSATP mode (PDTV=0)
    // FSC holds an IOSATP when PDTV=0
    TT_IOMMU::Fsc fsc;
    fsc.bits_.mode_ = static_cast<uint32_t>(TT_IOMMU::IosatpMode::Sv39);
    fsc.bits_.ppn_ = memMgr_.getFreePhysicalPages(1);
    dc.fsc_ = fsc.value_;

    // Use TableBuilder to create the device context
    bool msi_flat = iommu_->isDcExtended();
    uint64_t dc_addr = tableBuilder_.addDeviceContext(dc, devId, ddtp, msi_flat);

    if (dc_addr == 0) {
        std::cerr << "[ATS_HELPER] Failed to create device context!" << '\n';
        return;
    }

    // Store the device context address for later use
    deviceContextAddrs_[devId] = dc_addr;

    std::cout << "[ATS_HELPER] Device context created at address 0x" << std::hex << dc_addr << std::dec << '\n';

    // Create process context and page table entries for common IOVA addresses
    setupPageTablesForDevice(devId, dc);
  }

  // Create an ATS request
  static IommuRequest createAtsRequest(uint32_t devId, uint64_t iova, Ttype type = Ttype::PcieAts)
  {
    IommuRequest req;
    req.devId = devId;
    req.type = type;
    req.iova = iova;
    req.size = 4;
    req.privMode = PrivilegeMode::User;
    req.hasProcId = false;
    return req;
  }

  // Setup command queue for command testing
  void setupCommandQueue()
  {
    uint64_t cqbAddr = 0x1000000;

    // Configure CQB using Qbase
    Cqb cqb{0};
    cqb.fields.ppn = cqbAddr >> 12;
    cqb.fields.log2szm1 = 11; // 4KB
    iommu_->writeCqb(cqb.value, 3);
    iommu_->writeCqt(0);
    iommu_->writeCqcsr(1);

    std::cout << "[ATS_HELPER] Command queue configured at PPN 0x" << std::hex << (cqbAddr >> 12) << std::dec << '\n';
  }

  // Check if device context exists
  bool hasDeviceContext(uint32_t devId) const {
    return deviceContextAddrs_.find(devId) != deviceContextAddrs_.end();
  }

  // Get device context address
  uint64_t getDeviceContextAddr(uint32_t devId) const {
    auto it = deviceContextAddrs_.find(devId);
    return (it != deviceContextAddrs_.end()) ? it->second : 0;
  }

  // Setup page tables for common IOVA addresses used in tests
  void setupPageTablesForDevice(uint32_t devId, const ExtendedDeviceContext& dc) {
    // For direct IOSATP mode (PDTV=0), we directly create S-stage page table entries
    // using the IOSATP from the device context's first-stage context

    // Create S-stage page table entries for common test IOVA addresses
    std::vector<uint64_t> testIovas = {
        0x1000,           // Basic test
        0x2000,           // Multiple devices test
        0x10000000,       // T2GPA test
        0x2000 + (devId << 12)  // Device-specific IOVA
    };

    for (uint64_t iova : testIovas) {
        // Create a leaf PTE for this IOVA
        pte_t pte;
        pte.V = 1;        // Valid
        pte.R = 1;        // Readable
        pte.W = 1;        // Writable
        pte.X = 0;        // Not executable
        pte.U = 1;        // User accessible
        pte.G = 0;        // Not global
        pte.A = 1;        // Accessed
        pte.D = 0;        // Not dirty
        pte.PPN = memMgr_.getFreePhysicalPages(1); // Map to physical page

        // Add S-stage page table entry directly using the IOSATP from device context
        // FSC holds an IOSATP when PDTV=0
        Iosatp iosatp(dc.fsc_);
        bool success = tableBuilder_.addSStagePageTableEntry(iosatp, iova, pte, 0);
        if (!success) {
            std::cerr << "[ATS_HELPER] Failed to create S-stage PTE for IOVA 0x"
                      << std::hex << iova << " device 0x" << devId << std::dec << '\n';
        }
    }

    std::cout << "[ATS_HELPER] Created page tables for device 0x" << std::hex << devId
              << " with " << testIovas.size() << " IOVA mappings" << std::dec << '\n';
  }

private:
  MemoryModel mem_;
  MemoryManager memMgr_{};
  std::function<bool(uint64_t,unsigned,uint64_t&,bool&)> readFunc_;
  std::function<bool(uint64_t,unsigned,uint64_t)> writeFunc_;
  TableBuilder tableBuilder_;
  std::unique_ptr<Iommu> iommu_;
  std::map<uint32_t, uint64_t> deviceContextAddrs_; // devId -> context address

  void setupIommu()
  {
    iommu_ = std::make_unique<Iommu>(0x1000, 0x800, mem_.size());

    // Install memory callbacks
    iommu_->setMemReadCb(readFunc_);
    iommu_->setMemWriteCb(writeFunc_);

    // Install stage1 translation callback (identity translation)
    std::function<bool(uint64_t, unsigned, bool, bool, bool, uint64_t&, unsigned&)> stage1_cb =
      [](uint64_t va, unsigned /*privMode*/, bool, bool, bool, uint64_t& gpa, unsigned& cause) {
        gpa = va; // Identity translation
        cause = 0;
        return true;
      };
    iommu_->setStage1Cb(stage1_cb);

    // Install stage2 translation callback (identity translation)
    std::function<bool(uint64_t, unsigned, bool, bool, bool, uint64_t&, unsigned&)> stage2_cb =
      [](uint64_t gpa, unsigned /*privMode*/, bool, bool, bool, uint64_t& pa, unsigned& cause) {
        pa = gpa; // Identity translation
        cause = 0;
        return true;
      };
    iommu_->setStage2Cb(stage2_cb);

    // Install stage2 trap info callback
    std::function<void(uint64_t&, bool&, bool&)> trap_cb =
      [](uint64_t& /*gpa*/, bool& /*implicit*/, bool& /*write*/) {
        // Do nothing
      };
    iommu_->setStage2TrapInfoCb(trap_cb);

    // Install stage1 configuration callback
    std::function<void(unsigned, unsigned, uint64_t, bool)> stage1_config_cb =
      [](unsigned /*mode*/, unsigned /*asid*/, uint64_t /*ppn*/, bool /*sum*/) {
        // Do nothing
      };
    iommu_->setStage1ConfigCb(stage1_config_cb);

    // Install stage2 configuration callback
    std::function<void(unsigned, unsigned, uint64_t)> stage2_config_cb =
      [](unsigned /*mode*/, unsigned /*vmid*/, uint64_t /*ppn*/) {
        // Do nothing
      };
    iommu_->setStage2ConfigCb(stage2_config_cb);

    iommu_->setSetFaultOnFirstAccess([](unsigned /* stage */ , bool /* flag */) {});

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 0);  // version 1.0
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 10); // Sv48
    caps |= (1ULL << 16); // Sv32x4
    caps |= (1ULL << 17); // Sv39x4
    caps |= (1ULL << 18); // Sv48x4
    caps |= (1ULL << 19); // Sv57x4
    caps |= (1ULL << 25); // ATS (correct bit position)
    caps |= (1ULL << 26); // T2GPA (correct bit position)
    caps |= (1ULL << 38); // PD8 (correct bit position)
    caps |= (1ULL << 39); // PD17 (correct bit position)
    caps |= (1ULL << 40); // PD20 (correct bit position)

    iommu_->configureCapabilities(caps);

    // Configure FCTL for little-endian operation
    iommu_->writeFctl(0);

    std::cout << "[ATS_HELPER] IOMMU configured with capabilities 0x" << std::hex << caps << std::dec << '\n';
  }
};

void testBasicAtsTranslation() {
    std::cout << "\n=== Basic ATS Translation Test (using TableBuilder) ===\n";

  AtsTestHelper helper;

    // Test device ID
    constexpr uint32_t devId = 0x123;

    // Set up device context with ATS enabled
    helper.setupDeviceContextWithBuilder(devId, true, false);

    // Verify device context was created
    bool contextExists = helper.hasDeviceContext(devId);
    std::cout << "[TEST] Device context creation: " << (contextExists ? "PASS" : "FAIL") << '\n';

    if (!contextExists) {
        return;
    }

    // Create ATS translation request
    IommuRequest atsReq = AtsTestHelper::createAtsRequest(devId, 0x1000);

    // Process the ATS request
    auto& iommu = helper.getIommu();

    // Debug: Check DDTP before ATS request
    uint64_t ddtpBefore = iommu.readDdtp();
    Ddtp ddtpBeforeObj{.value = ddtpBefore};
    std::cout << "[DEBUG] DDTP before ATS: 0x" << std::hex << ddtpBefore
              << ", mode: " << static_cast<int>(ddtpBeforeObj.fields.iommu_mode) << std::dec << '\n';
    Iommu::AtsResponse resp;
  unsigned cause = 0;

    bool success = iommu.atsTranslate(atsReq, resp, cause);
    std::cout << "[TEST] ATS translation request: " << (success ? "PASS" : "FAIL") << '\n';

    if (!success) {
        std::cout << "[DEBUG] ATS translation failed with cause: " << cause << '\n';
        std::cout << "[DEBUG] Response status: " << unsigned(resp.status) << '\n';
    }

    if (success && resp.successful()) {
        std::cout << "[RESULT] ATS translation: IOVA 0x" << std::hex << atsReq.iova
                  << " -> PA 0x" << resp.translatedAddr << std::dec << '\n';
    }
}

void testAtsWithT2gpa() {
    std::cout << "\n=== ATS with T2GPA Test (using TableBuilder) ===\n";

    AtsTestHelper helper;

    constexpr uint32_t devId = 0x456;

    // Set up device context with both ATS and T2GPA enabled
    helper.setupDeviceContextWithBuilder(devId, true, true);

    bool contextExists = helper.hasDeviceContext(devId);
    std::cout << "[TEST] Device context with T2GPA creation: " << (contextExists ? "PASS" : "FAIL") << '\n';

    if (!contextExists) {
        return;
    }

    // Create ATS request for a higher address that will go through G-stage translation
    IommuRequest atsReq = AtsTestHelper::createAtsRequest(devId, 0x10000000);

    auto& iommu = helper.getIommu();
    Iommu::AtsResponse resp;
  unsigned cause = 0;

    bool success = iommu.atsTranslate(atsReq, resp, cause);
    std::cout << "[TEST] ATS with T2GPA request: " << (success ? "PASS" : "FAIL") << '\n';

    if (!success) {
        std::cout << "[DEBUG] ATS+T2GPA translation failed with cause: " << cause << '\n';
        std::cout << "[DEBUG] Response success: " << unsigned(resp.status) << '\n';
    }

    if (success && resp.successful()) {
        std::cout << "[RESULT] ATS+T2GPA translation: IOVA 0x" << std::hex << atsReq.iova
                  << " -> PA 0x" << resp.translatedAddr << std::dec << '\n';
    }
}

void testMultipleDevicesAts() {
    std::cout << "\n=== Multiple Devices ATS Test (using TableBuilder) ===\n";

  AtsTestHelper helper;

    // Set up multiple devices with different configurations
    std::vector<std::pair<uint32_t, bool>> devices = {
        {0x100, true},   // ATS enabled
        {0x200, false},  // ATS disabled
        {0x300, true},   // ATS enabled
        {0x400, true}    // ATS enabled
    };

    // Create device contexts for all devices
    for (auto [devId, atsEnabled] : devices) {
        helper.setupDeviceContextWithBuilder(devId, atsEnabled, false);

        bool contextExists = helper.hasDeviceContext(devId);
        std::cout << "[SETUP] Device 0x" << std::hex << devId << std::dec
                  << " context: " << (contextExists ? "PASS" : "FAIL") << '\n';
    }

    // Test ATS requests for ATS-enabled devices
    auto& iommu = helper.getIommu();
    int successCount = 0;
    int totalAtsDevices = 0;

    for (auto [devId, atsEnabled] : devices) {
        if (!atsEnabled) continue; // Skip non-ATS devices

        totalAtsDevices++;
        IommuRequest atsReq = AtsTestHelper::createAtsRequest(devId, 0x2000 + (devId << 12));
        Iommu::AtsResponse resp;
        unsigned cause = 0;

        std::cout << "[DEBUG] Testing device 0x" << std::hex << devId << " with IOVA 0x" << atsReq.iova << std::dec << '\n';

        bool success = iommu.atsTranslate(atsReq, resp, cause);
        if (success && resp.successful()) {
            successCount++;
            std::cout << "[ATS] Device 0x" << std::hex << devId << ": IOVA 0x" << atsReq.iova
                      << " -> PA 0x" << resp.translatedAddr << std::dec << '\n';
        } else {
            std::cout << "[ATS] Device 0x" << std::hex << devId << ": FAILED - success=" << success
                      << ", resp.status=" << unsigned(resp.status) << ", cause=" << std::dec << cause << '\n';
        }
    }

    std::cout << "[TEST] Multiple devices ATS: " << successCount << "/" << totalAtsDevices
              << " successful (" << (successCount == totalAtsDevices ? "PASS" : "FAIL") << ")" << '\n';
}

void testAtsCommandQueue() {
    std::cout << "\n=== ATS Command Queue Test (using TableBuilder) ===\n";

  AtsTestHelper helper;

    // Set up command queue
    helper.setupCommandQueue();

    constexpr uint32_t devId = 0x789;

    // Set up device context with ATS
    helper.setupDeviceContextWithBuilder(devId, true, false);

    bool contextExists = helper.hasDeviceContext(devId);
    std::cout << "[TEST] ATS command queue setup: " << (contextExists ? "PASS" : "FAIL") << '\n';

    // For a more comprehensive test, we would issue ATS invalidation commands
    // through the command queue, but that requires more complex setup

    auto& iommu = helper.getIommu();

    // Check that command queue is enabled
    uint64_t cqcsr = iommu.readCqcsr();
    bool cqEnabled = (cqcsr & 0x1) != 0;

    std::cout << "[TEST] Command queue enabled: " << (cqEnabled ? "PASS" : "FAIL") << '\n';
}

void testTableBuilderStats() {
    std::cout << "\n=== TableBuilder Memory Statistics ===\n";

  AtsTestHelper helper;

    // Set up several devices to show memory allocation
    for (uint32_t devId = 0x1000; devId <= 0x1005; devId++) {
        helper.setupDeviceContextWithBuilder(devId, true, devId % 2 == 0); // Alternate T2GPA
    }

    // Print allocation statistics
    helper.getMemoryManager().printStats();
}

// -----------------------------------------------------------------------------
// Main function
// -----------------------------------------------------------------------------

int main() {
    std::cout << "=== IOMMU ATS Unified Tests (Refactored with TableBuilder) ===\n";

  try {
    testBasicAtsTranslation();
        testAtsWithT2gpa();
        testMultipleDevicesAts();
        testAtsCommandQueue();
        testTableBuilderStats();

        std::cout << "\n=== All ATS tests completed! ===\n";
        return 0;

  } catch (const std::exception& e) {
        std::cerr << "ATS test failed with exception: " << e.what() << '\n';
    return 1;
  } catch (...) {
        std::cerr << "ATS test failed with unknown exception" << '\n';
    return 1;
  }
}
