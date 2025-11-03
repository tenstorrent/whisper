#include "Iommu.hpp"
#include "DeviceContext.hpp"
#include "MemoryModel.hpp"
#include "IommuStructures.hpp"
#include "MemoryManager.hpp" 
#include "TableBuilder.hpp"
#include <cassert>
#include <iostream>
#include <vector>
#include <functional>

using namespace TT_IOMMU;
using namespace IOMMU;

namespace TestValues {
    // Common test device IDs
    constexpr uint32_t SIMPLE_DEV_ID = 0x2A;
    constexpr uint32_t TWO_LEVEL_DEV_ID = 0x1FFF;
    constexpr uint32_t THREE_LEVEL_DEV_ID = 0xABCDEF;
    
    // MSI values
    constexpr uint64_t MSI_ADDR_MASK = 0xFFFFF000ULL;
    constexpr uint64_t MSI_ADDR_PATTERN = 0xFEDC1000ULL;
}

static void installMemCbs(TT_IOMMU::Iommu& iommu, MemoryModel& mem) {
    std::function<bool(uint64_t,unsigned,uint64_t&)> rcb =
        [&mem](uint64_t a, unsigned s, uint64_t& d) { return mem.read(a, s, d); };
    std::function<bool(uint64_t,unsigned,uint64_t)> wcb =
        [&mem](uint64_t a, unsigned s, uint64_t d) { return mem.write(a, s, d); };

    iommu.setMemReadCb(rcb);
    iommu.setMemWriteCb(wcb);
}

static void configureCapabilities(TT_IOMMU::Iommu& iommu) {
    Capabilities caps{};
    caps.fields.pd8 = 1;
    caps.fields.pd17 = 1;
    caps.fields.pd20 = 1;
    caps.fields.sv32 = 1;
    caps.fields.sv39 = 1;
    caps.fields.sv48 = 1; 
    caps.fields.sv57= 1;
    caps.fields.sv32x4 = 1;
    caps.fields.sv39x4 = 1;
    caps.fields.sv48x4 = 1;
    caps.fields.sv57x4 = 1;
    caps.fields.amo_hwad = 1;
    caps.fields.msi_flat = 1;  // For extended format tests
    caps.fields.end = 1;      // Support for endianness control

    iommu.configureCapabilities(caps.value);
}

// Configure FCTL register - critical for SXL tests
static void configureFctl(TT_IOMMU::Iommu& iommu, bool gxl = false, bool be = false, bool wsi = false) {
    uint32_t fctlVal = 0;
    if (gxl) fctlVal |= (1 << 2);  // GXL bit
    if (be) fctlVal |= (1 << 0);   // BE bit
    if (wsi) fctlVal |= (1 << 1);  // WSI bit
    
    iommu.writeFctl(fctlVal);
    
    // Verify
    uint64_t readback = iommu.readFctl();
    std::cout << "[CONFIG] FCTL configured: GXL=" << (gxl ? "1" : "0")
              << ", BE=" << (be ? "1" : "0")
              << ", WSI=" << (wsi ? "1" : "0")
              << ", readback=0x" << std::hex << readback << std::dec << "\n";
}

/// Returns the address of the leaf device context entry
static uint64_t setupDeviceTableWithBuilder(TT_IOMMU::Iommu& iommu, MemoryModel& /* memory */,
                                           MemoryManager& memMgr, TableBuilder& tableBuilder,
                                           uint32_t devId, Ddtp::Mode mode) {
    // Set up DDTP
    Ddtp ddtp{};
    ddtp.fields.iommu_mode = mode;
    ddtp.fields.ppn = memMgr.getFreePhysicalPages(1);
    
    // Configure DDTP register in the IOMMU
    iommu.writeDdtp(ddtp.value, 3);
    
    // Create a basic device context
    ExtendedDeviceContext dc = {};
    dc.tc_ = 0x1; // Valid device context
    dc.ta_ = 0;   // Translation Attributes (no PSCID)
    dc.iohgatp_ = 0; // Bare mode
    
    // Set up first-stage context - FSC holds IOSATP when PDTV=0
    TT_IOMMU::Fsc fsc;
    fsc.bits_.mode_ = static_cast<uint32_t>(TT_IOMMU::IosatpMode::Sv39);
    fsc.bits_.ppn_ = memMgr.getFreePhysicalPages(1);
    dc.fsc_ = fsc.value_;
    
    // Use TableBuilder to create the DDT structure
    bool msi_flat = iommu.isDcExtended();
    uint64_t dc_addr = tableBuilder.addDeviceContext(dc, devId, ddtp, msi_flat);
    
    Ddtp ddtpObj{.value = ddtp.value};
    std::cout << "[TABLE_BUILDER] Created DDT structure for device ID 0x" 
              << std::hex << devId << " using " << ddtpObj.levels() << "-level mode" 
              << ", device context at 0x" << dc_addr << std::dec << '\n';
    
    return dc_addr;
}

// Creates a device context with the specified configuration (simplified version)
static DeviceContext createDeviceContext(
    bool valid = true, 
    bool enable_ats = false,
    bool enable_pri = false,
    bool t2gpa = false,
    bool dtf = false,
    bool pdtv = false,
    bool prpr = false,
    bool gade = false,
    bool sade = false,
    bool dpe = false,
    bool sbe = false,
    bool sxl = false,
    IohgatpMode iohgatp_mode = IohgatpMode::Bare,
    uint16_t gscid = 0,
    uint64_t iohgatp_ppn = 0,
    uint32_t pscid = 0,
    IosatpMode iosatp_mode = IosatpMode::Bare,
    uint64_t iosatp_ppn = 0,
    PdtpMode pdtp_mode = PdtpMode::Bare,
    uint64_t pdtp_ppn = 0,
    MsiptpMode msi_mode = MsiptpMode::Off,
    uint64_t msi_ppn = 0,
    uint64_t msi_addr_mask = 0,
    uint64_t msi_addr_pattern = 0)
{
    // Create the Translation Control field
    TransControl tc;
    tc.bits_.v_ = valid ? 1 : 0;
    tc.bits_.ats_ = enable_ats ? 1 : 0;
    tc.bits_.pri_ = enable_pri ? 1 : 0;
    tc.bits_.t2gpa_ = t2gpa ? 1 : 0;
    tc.bits_.dtf_ = dtf ? 1 : 0;
    tc.bits_.pdtv_ = pdtv ? 1 : 0;
    tc.bits_.prpr_ = prpr ? 1 : 0;
    tc.bits_.gade_ = gade ? 1 : 0;
    tc.bits_.sade_ = sade ? 1 : 0;
    tc.bits_.dpe_ = dpe ? 1 : 0;
    tc.bits_.sbe_ = sbe ? 1 : 0;
    tc.bits_.sxl_ = sxl ? 1 : 0;
    
    // Create IOHGATP field
    Iohgatp iohgatp;
    iohgatp.bits_.mode_ = iohgatp_mode;
    iohgatp.bits_.gcsid_ = gscid;
    iohgatp.bits_.ppn_ = iohgatp_ppn;
    uint64_t iohgatpVal = iohgatp.value_;
    
    // Create TA field with PSCID
    uint64_t ta = pscid << 12; // PSCID is in bits 12-31
    
    // Create FSC field based on PDTV
    Fsc fsc;
    if (pdtv) {
      // If PDTV is set, FSC holds PDTP
      fsc.bits_.ppn_ = pdtp_ppn; fsc.bits_.mode_ = uint32_t(pdtp_mode);
    } else {
      // Otherwise, FSC holds IOSATP
      fsc.bits_.ppn_ = iosatp_ppn; fsc.bits_.mode_ = uint32_t(iosatp_mode);
    }
    
    // Create the base DeviceContext
    DeviceContext dc(tc.value_, iohgatpVal, ta, fsc.value_);
    
    // If MSI fields are needed, create extended DeviceContext
    if (msi_mode != MsiptpMode::Off || msi_ppn != 0 || 
        msi_addr_mask != 0 || msi_addr_pattern != 0) {
        uint64_t msiptp = (uint64_t(msi_mode) << 60) | msi_ppn;
        dc = DeviceContext{tc.value_, iohgatpVal, ta, fsc.value_, 
                           msiptp, msi_addr_mask, msi_addr_pattern};
    }
    
    return dc;
}

// Helper function to print test results  
static void printTestResult(const std::string& testName, bool success) {
    std::cout << "[TEST] " << testName << ": " 
              << (success ? "PASS" : "FAIL") << '\n';
}

void testBasicDeviceTableWalk() {
    std::cout << "\n=== Basic Device Table Walk Test (using TableBuilder) ===\n";
    
    // Create infrastructure
    MemoryModel memory(size_t(1024) * 1024);  // 1MB
    MemoryManager memMgr;
    
    // Create table builder with memory callbacks
    auto readFunc = [&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    };
    auto writeFunc = [&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    };
    
    TableBuilder tableBuilder(memMgr, readFunc, writeFunc);
    
    // Create IOMMU instance
    Iommu iommu(0x1000, 0x800, memory.size());
    installMemCbs(iommu, memory);
    configureCapabilities(iommu);
    configureFctl(iommu);
    
    // Test 1-level DDT
    std::cout << "\n--- Testing 1-level DDT ---\n";
    uint64_t leafAddr1 = setupDeviceTableWithBuilder(iommu, memory, memMgr, tableBuilder,
                                                    TestValues::SIMPLE_DEV_ID, Ddtp::Mode::Level1);
    
    // Test 2-level DDT  
    std::cout << "\n--- Testing 2-level DDT ---\n";
    uint64_t leafAddr2 = setupDeviceTableWithBuilder(iommu, memory, memMgr, tableBuilder,
                                                    TestValues::TWO_LEVEL_DEV_ID, Ddtp::Mode::Level2);
    
    // Test 3-level DDT
    std::cout << "\n--- Testing 3-level DDT ---\n";
    uint64_t leafAddr3 = setupDeviceTableWithBuilder(iommu, memory, memMgr, tableBuilder,
                                                    TestValues::THREE_LEVEL_DEV_ID, Ddtp::Mode::Level3);
    
    // Verify addresses are valid
    printTestResult("1-level DDT creation", leafAddr1 != 0);
    printTestResult("2-level DDT creation", leafAddr2 != 0);  
    printTestResult("3-level DDT creation", leafAddr3 != 0);
    
    // Print memory allocation statistics
    std::cout << "\n--- Memory Allocation Statistics ---\n";
    memMgr.printStats();
}

void testDeviceContextTranslation() {
    std::cout << "\n=== Device Context Translation Test ===\n";
    
    // Create infrastructure 
    MemoryModel memory(size_t(2) * 1024 * 1024);  // 2MB
    MemoryManager memMgr;
    
    auto readFunc = [&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    };
    auto writeFunc = [&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    };
    
    TableBuilder tableBuilder(memMgr, readFunc, writeFunc);
    
    Iommu iommu(0x1000, 0x800, memory.size());
    installMemCbs(iommu, memory);
    configureCapabilities(iommu);
    configureFctl(iommu);
    
    // Create device context with translation enabled
    uint64_t leafAddr = setupDeviceTableWithBuilder(iommu, memory, memMgr, tableBuilder,
                                                   TestValues::SIMPLE_DEV_ID, Ddtp::Mode::Level2);
    
    // Write a device context with more complex settings
    DeviceContext dc = createDeviceContext(
        true,    // valid
        false,   // enable_ats (disabled since ATS capability not set)
        false,   // enable_pri
        false,   // t2gpa
        false,   // dtf
        false,   // pdtv
        false,   // prpr
        false,   // gade 
        false,   // sade
        false,   // dpe
        false,   // sbe
        false,   // sxl
        IohgatpMode::Bare,  // iohgatp_mode
        0,       // gscid
        0,       // iohgatp_ppn
        0x123,   // pscid
        IosatpMode::Sv39,   // iosatp_mode
        memMgr.getFreePhysicalPages(1)  // iosatp_ppn
    );
    
    // Write the device context using the IOMMU
    iommu.writeDeviceContext(leafAddr, dc);
    
    // Try to read it back for verification
    DeviceContext readDc;
    unsigned cause = 0;
    bool readSuccess = iommu.loadDeviceContext(TestValues::SIMPLE_DEV_ID, readDc, cause);
    
    if (!readSuccess) {
        std::cout << "[ERROR] loadDeviceContext failed with cause: " << cause << '\n';
    }
    
    printTestResult("Device context write/read", readSuccess);
    
    if (readSuccess) {
        std::cout << "[VERIFY] Device context valid: " << (readDc.valid() ? "true" : "false") << '\n';
        std::cout << "[VERIFY] Device context ATS enabled: " << (readDc.ats() ? "true" : "false") << '\n';
        std::cout << "[VERIFY] Device context IOHGATP: 0x" << std::hex << readDc.iohgatp() << std::dec << '\n';
        std::cout << "[VERIFY] Device context IOHGATP mode: " << static_cast<int>(readDc.iohgatpMode()) << '\n';
    }
}

int main() {
    std::cout << "=== IOMMU Device Table Walk Tests (Refactored with TableBuilder) ===\n";
    
    try {
        testBasicDeviceTableWalk();
        testDeviceContextTranslation();
        
        std::cout << "\n=== All tests completed! ===\n";
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << '\n';
        return 1;
    }
}
