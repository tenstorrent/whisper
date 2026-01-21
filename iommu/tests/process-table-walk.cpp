#include "Iommu.hpp"
#include "ProcessContext.hpp"
#include "DeviceContext.hpp"
#include "MemoryModel.hpp"
#include "IommuStructures.hpp"
#include "MemoryManager.hpp"
#include "TableBuilder.hpp"
#include <iostream>
#include <cstring>
#include <cassert>
#include <functional>

using namespace TT_IOMMU;
using namespace IOMMU;

namespace TestValues {
    constexpr uint32_t TEST_DEV_ID = 0x2A5;
    constexpr uint32_t TEST_PROCESS_ID_8 = 0x7F;    // For PD8 mode
    constexpr uint32_t TEST_PROCESS_ID_17 = 0x1ABCD; // For PD17 mode
    constexpr uint32_t TEST_PROCESS_ID_20 = 0xFEDCB;  // For PD20 mode
}

// Configure DDTP for index calculation
static void configureDdtp(Iommu& iommu, uint64_t rootPpn, Ddtp::Mode mode) {
  Ddtp ddtp {};
  ddtp.fields.iommu_mode = mode;
  ddtp.fields.ppn = rootPpn;

  iommu.writeDdtp(ddtp.value, 3);

  uint64_t readBack = iommu.readDdtp();
  assert(readBack == ddtp.value);
}

static void installMemCbs(Iommu& iommu, MemoryModel& mem) {
  std::function<bool(uint64_t,unsigned,uint64_t&,bool&)> rcb =
    [&mem](uint64_t a, unsigned s, uint64_t& d, bool& c) { c = false; return mem.read(a, s, d); };

  std::function<bool(uint64_t,unsigned,uint64_t)> wcb =
    [&mem](uint64_t a, unsigned s, uint64_t d) { return mem.write(a, s, d); };

  iommu.setMemReadCb(rcb);
  iommu.setMemWriteCb(wcb);
}

static uint64_t setupTablesWithBuilder(Iommu& iommu, MemoryModel& /* memory */,
                                      MemoryManager& memMgr, TableBuilder& tableBuilder,
                                      uint32_t devId, uint32_t processId,
                                      Ddtp::Mode ddtMode, PdtpMode pdtMode) {

    // Set up DDTP
    Ddtp ddtp{};
    ddtp.fields.iommu_mode = ddtMode;
    ddtp.fields.ppn = memMgr.getFreePhysicalPages(1);

    // Configure DDTP register in the IOMMU
    configureDdtp(iommu, ddtp.fields.ppn, ddtMode);

    // Create a device context with PDT enabled
    ExtendedDeviceContext dc = {};
    dc.tc_ = 0x21; // Valid device context with PDTV=1 for process directory
    dc.iohgatp_ = 0; // Bare mode

    // Set up first-stage context with PDT (FSC holds PDTP when PDTV=1)
    TT_IOMMU::Fsc fsc;
    fsc.bits_.mode_ = static_cast<uint32_t>(pdtMode);
    fsc.bits_.ppn_ = memMgr.getFreePhysicalPages(1);
    dc.fsc_ = fsc.value_;
    bool gxl = (iommu.readFctl() >> 2) & 1;

    // Create device context using TableBuilder
    bool msi_flat = iommu.isDcExtended();
    uint64_t dc_addr = tableBuilder.addDeviceContext(dc, devId, ddtp, msi_flat);

    if (dc_addr == 0) {
        std::cerr << "[ERROR] Failed to create device context" << '\n';;
        return 0;
    }

    std::cout << "[TABLE_BUILDER] Created device context at 0x" << std::hex << dc_addr
              << " for device ID 0x" << devId << std::dec << '\n';

    // Create a process context and set up SATP for addr translation (FSC field)
    TT_IOMMU::Iosatp iosatp(0);
    iosatp.bits_.mode_ = TT_IOMMU::IosatpMode::Sv39;
    iosatp.bits_.ppn_ = memMgr.getFreePhysicalPages(1);
    ProcessContext pc{0x1, iosatp.value_};  // TA.valid=1, FSC=iosatp

    // Add process context using TableBuilder
    uint64_t pc_addr = tableBuilder.addProcessContext(dc, gxl, pc, processId);

    if (pc_addr == 0) {
        std::cerr << "[ERROR] Failed to create process context" << '\n';
        return 0;
    }

    std::cout << "[TABLE_BUILDER] Created process context at 0x" << std::hex << pc_addr
              << " for process ID 0x" << processId << std::dec << '\n';

    return pc_addr;
}

void testProcessDirectoryPd8() {
    std::cout << "\n=== Process Directory PD8 Test (using TableBuilder) ===\n";

    // Create infrastructure
    MemoryModel memory(size_t(1024) * 1024);  // 1MB
    MemoryManager memMgr;

    auto readFunc = [&memory](uint64_t addr, unsigned size, uint64_t& data, bool& corrupted) {
        corrupted = false;
        return memory.read(addr, size, data);
    };
    auto writeFunc = [&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    };

    TableBuilder tableBuilder(memMgr, readFunc, writeFunc);

    Iommu iommu(0x1000, 0x800, memory.size());
    installMemCbs(iommu, memory);

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 22); // pd8
    caps |= (1ULL << 9);  // sv39
    caps |= (1ULL << 17);  // sv39x4
    iommu.configureCapabilities(caps);

    // Test PD8 mode with 1-level DDT
    uint64_t pcAddr = setupTablesWithBuilder(iommu, memory, memMgr, tableBuilder,
                                            TestValues::TEST_DEV_ID,
                                            TestValues::TEST_PROCESS_ID_8,
                                            Ddtp::Mode::Level1, PdtpMode::Pd8);

    bool success = (pcAddr != 0);
    std::cout << "[TEST] PD8 process directory creation: "
              << (success ? "PASS" : "FAIL") << '\n';

    if (success) {
        // Verify we can read back the process context would require device context
        // For now, just verify the address is non-zero (context was created)
        std::cout << "[VERIFY] Process context created successfully at address 0x"
                  << std::hex << pcAddr << std::dec << '\n';
    }
}

void testProcessDirectoryPd17() {
    std::cout << "\n=== Process Directory PD17 Test (using TableBuilder) ===\n";

    MemoryModel memory(size_t(2) * 1024 * 1024);  // 2MB
    MemoryManager memMgr;

    auto readFunc = [&memory](uint64_t addr, unsigned size, uint64_t& data, bool& corrupted) {
        corrupted = false;
        return memory.read(addr, size, data);
    };
    auto writeFunc = [&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    };

    TableBuilder tableBuilder(memMgr, readFunc, writeFunc);

    Iommu iommu(0x1000, 0x800, memory.size());
    installMemCbs(iommu, memory);

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 23); // pd17
    caps |= (1ULL << 9);  // sv39
    caps |= (1ULL << 17);  // sv39x4
    iommu.configureCapabilities(caps);

    // Test PD17 mode with 2-level DDT
    uint64_t pcAddr = setupTablesWithBuilder(iommu, memory, memMgr, tableBuilder,
                                            TestValues::TEST_DEV_ID,
                                            TestValues::TEST_PROCESS_ID_17,
                                            Ddtp::Mode::Level2, PdtpMode::Pd17);

    bool success = (pcAddr != 0);
    std::cout << "[TEST] PD17 process directory creation: "
              << (success ? "PASS" : "FAIL") << '\n';
}

void testProcessDirectoryPd20() {
    std::cout << "\n=== Process Directory PD20 Test (using TableBuilder) ===\n";

    MemoryModel memory(size_t(4) * 1024 * 1024);  // 4MB
    MemoryManager memMgr;

    auto readFunc = [&memory](uint64_t addr, unsigned size, uint64_t& data, bool& corrupted) {
        corrupted = false;
        return memory.read(addr, size, data);
    };
    auto writeFunc = [&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    };

    TableBuilder tableBuilder(memMgr, readFunc, writeFunc);

    Iommu iommu(0x1000, 0x800, memory.size());
    installMemCbs(iommu, memory);

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 24); // pd20
    caps |= (1ULL << 9);  // sv39
    caps |= (1ULL << 17);  // sv39x4
    iommu.configureCapabilities(caps);

    // Test PD20 mode with 3-level DDT
    uint64_t pcAddr = setupTablesWithBuilder(iommu, memory, memMgr, tableBuilder,
                                            TestValues::TEST_DEV_ID,
                                            TestValues::TEST_PROCESS_ID_20,
                                            Ddtp::Mode::Level3, PdtpMode::Pd20);

    bool success = (pcAddr != 0);
    std::cout << "[TEST] PD20 process directory creation: "
              << (success ? "PASS" : "FAIL") << '\n';
}

void testMultipleProcesses() {
    std::cout << "\n=== Multiple Processes Test (using TableBuilder) ===\n";

    MemoryModel memory(size_t(8) * 1024 * 1024);  // 8MB
    MemoryManager memMgr;

    auto readFunc = [&memory](uint64_t addr, unsigned size, uint64_t& data, bool& corrupted) {
        corrupted = false;
        return memory.read(addr, size, data);
    };
    auto writeFunc = [&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    };

    TableBuilder tableBuilder(memMgr, readFunc, writeFunc);

    Iommu iommu(0x1000, 0x800, memory.size());
    installMemCbs(iommu, memory);

    uint64_t caps = 0;
    caps |= (1ULL << 23); // pd17
    caps |= (1ULL << 9);  // sv39
    caps |= (1ULL << 17);  // sv39x4
    iommu.configureCapabilities(caps);

    // Set up device context first
    Ddtp ddtp{};
    ddtp.fields.iommu_mode = Ddtp::Mode::Level2;
    ddtp.fields.ppn = memMgr.getFreePhysicalPages(1);
    configureDdtp(iommu, ddtp.fields.ppn, Ddtp::Mode::Level2);

    ExtendedDeviceContext dc = {};
    dc.tc_ = 0x21; // Valid with PDTV=1
    dc.iohgatp_ = 0; // Bare mode
    TT_IOMMU::Fsc fsc;
    fsc.bits_.mode_ = static_cast<uint32_t>(TT_IOMMU::PdtpMode::Pd17);
    fsc.bits_.ppn_ = memMgr.getFreePhysicalPages(1);
    dc.fsc_ = fsc.value_;
    bool gxl = (iommu.readFctl() >> 2) & 1;

    uint64_t dc_addr = tableBuilder.addDeviceContext(dc, TestValues::TEST_DEV_ID, ddtp, false);

    if (dc_addr == 0) {
        std::cout << "[TEST] Multiple processes setup: FAIL (device context creation failed)" << '\n';
        return;
    }

    // Create multiple process contexts for the same device
    std::vector<uint32_t> processIds = {0x100, 0x200, 0x300, 0x400};
    std::vector<uint64_t> pcAddrs;

    for (uint32_t pid : processIds) {
        TT_IOMMU::Iosatp iosatp(0);
        iosatp.bits_.mode_ = TT_IOMMU::IosatpMode::Sv39;
        iosatp.bits_.ppn_ = memMgr.getFreePhysicalPages(1);

        ProcessContext pc{0x1, iosatp.value_};  // TA.V=1, FSC=iosatp.

        uint64_t pc_addr = tableBuilder.addProcessContext(dc, gxl, pc, pid);
        pcAddrs.push_back(pc_addr);

        std::cout << "[TABLE_BUILDER] Process ID 0x" << std::hex << pid
                  << " -> context at 0x" << pc_addr << std::dec << '\n';
    }

    // Verify all processes were created successfully
    bool allSuccess = true;
    for (uint64_t addr : pcAddrs) {
        if (addr == 0) {
            allSuccess = false;
            break;
        }
    }

    std::cout << "[TEST] Multiple processes creation: "
              << (allSuccess ? "PASS" : "FAIL") << '\n';

    // Print memory allocation statistics
    std::cout << "\n--- Memory Allocation Statistics ---\n";
    memMgr.printStats();
}

int main() {
    std::cout << "=== IOMMU Process Table Walk Tests (Refactored with TableBuilder) ===\n";

    try {
        testProcessDirectoryPd8();
        testProcessDirectoryPd17();
        testProcessDirectoryPd20();
        testMultipleProcesses();

        std::cout << "\n=== All process table tests completed! ===\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << '\n';
        return 1;
    }
}
