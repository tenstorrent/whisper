#include "Iommu.hpp"
#include "DeviceContext.hpp"
#include "MemoryModel.hpp"
#include "FaultQueue.hpp"
#include <cassert>
#include <iostream>
#include <vector>
#include <set>
#include <functional>

using namespace TT_IOMMU;

// Simple memory model for testing
class TestMemory {
public:
    TestMemory(size_t size) : memory(size, 0) {
        std::cout << "Created test memory of size " << size << " bytes\n";
    }

    bool read(uint64_t addr, unsigned size, uint64_t& data) {
        if (addr + size > memory.size()) {
            std::cout << "Memory read error: address 0x" << std::hex << addr
                      << std::dec << " + size " << size << " exceeds memory size "
                      << memory.size() << "\n";
            return false;
        }

        data = 0;
        for (unsigned i = 0; i < size; i++) {
            data |= static_cast<uint64_t>(memory[addr + i]) << (i * 8);
        }
        return true;
    }

    bool write(uint64_t addr, unsigned size, uint64_t data) {
        if (addr + size > memory.size()) {
            std::cout << "Memory write error: address 0x" << std::hex << addr
                      << std::dec << " + size " << size << " exceeds memory size "
                      << memory.size() << "\n";
            return false;
        }

        for (unsigned i = 0; i < size; i++) {
            memory[addr + i] = (data >> (i * 8)) & 0xFF;
        }
        return true;
    }

    // Helper to dump memory
    void dump(uint64_t addr, unsigned size) {
        std::cout << "Memory dump at 0x" << std::hex << addr << ":\n";
        for (unsigned i = 0; i < size; i += 16) {
            std::cout << std::hex << (addr + i) << ": ";
            for (unsigned j = 0; j < 16 && (i + j) < size; j++) {
                std::cout << std::hex << static_cast<int>(memory[addr + i + j]) << " ";
            }
            std::cout << std::dec << "\n";
        }
    }

    uint64_t size() const {
        return memory.size();
    }

private:
    std::vector<uint8_t> memory;
};

// Simple test for fault queue
void testSimpleFaultQueue() {
    std::cout << "=== Simple Fault Queue Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    });

    // Configure basic capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);

    // Configure FCTL
    iommu.writeFctl(0); // little-endian, no WSI

    // Set DDTP to Off mode (this will cause a known fault)
    iommu.writeDdtp(0, 3); // Off mode

    // Set up a small fault queue (4 entries)
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 4 * sizeof(FaultRecord); i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue base with 4 entries (LOG2SZ-1 = 1)
    uint64_t fqb = (1ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0); // Head at entry 0

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
        std::cout << "Waiting for fault queue to activate..." << '\n';
    }

    if (!fqActive) {
        testPassed = false;
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
    }

    // Get initial state
    uint64_t fqhBefore = iommu.readFqh();
    uint64_t fqtBefore = iommu.readFqt();
    uint32_t ipsrBefore = iommu.readIpsr();

    std::cout << "Initial: FQH=" << fqhBefore << ", FQT=" << fqtBefore
              << ", IPSR=0x" << std::hex << ipsrBefore << std::dec << "\n";

    // Set up a tracking flag to verify callback is called
    bool stage1Called = false;

    // Setup translation stubs - the stage 1 should always fail
    iommu.setStage1Cb([&stage1Called](uint64_t va, unsigned /*privMode*/, bool r, bool w, bool x,
                                      uint64_t& /*gpa*/, unsigned& cause) {
        std::cout << "Stage1 callback called: va=0x" << std::hex << va << std::dec
                  << ", r=" << r << ", w=" << w << ", x=" << x << '\n';
        stage1Called = true;
        cause = 5; // Read access fault
        return false; // Return false to indicate failure
    });

    iommu.setStage2Cb([](uint64_t gpa, unsigned /*privMode*/, bool r, bool w, bool x,
                         uint64_t& /*pa*/, unsigned& cause) {
        std::cout << "Stage2 callback called: gpa=0x" << std::hex << gpa << std::dec
                  << ", r=" << r << ", w=" << w << ", x=" << x << '\n';
        cause = 5; // Read access fault
        return false; // Return false to indicate failure
    });

    iommu.setSetFaultOnFirstAccess([](unsigned /* stage */, bool /* flag */) {});

    iommu.setStage2TrapInfoCb([](uint64_t& gpa, bool& implicit, bool& write) {
        gpa = 0x1000;
        implicit = false;
        write = false;
    });

    // Create a simple request
    IommuRequest req;
    req.devId = 0x1;        // Simple device ID
    req.hasProcId = false;  // No process ID to keep it simple
    req.iova = 0x1000;      // Simple IOVA
    req.type = Ttype::UntransRead;
    req.privMode = PrivilegeMode::User;
    req.size = 4;

    // Perform translation (should fail with cause = 256 because DDTP is Off)
    uint64_t pa = 0;
    unsigned cause = 0;
    bool result = iommu.translate(req, pa, cause);

    std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
              << ", cause=" << cause << "\n";

    // Verify translation failed correctly
    if (result || cause != 256) {
        testPassed = false;
        std::cout << "ERROR: Expected translation to fail with cause 256 but got "
                  << "result=" << result << ", cause=" << cause << '\n';
    }

    // Check fault queue state
    uint64_t fqhAfter = iommu.readFqh();
    uint64_t fqtAfter = iommu.readFqt();
    uint32_t ipsrAfter = iommu.readIpsr();

    std::cout << "After: FQH=" << fqhAfter << ", FQT=" << fqtAfter
              << ", IPSR=0x" << std::hex << ipsrAfter << std::dec << "\n";

    // Check if fault was queued (FQT should have advanced)
    bool faultQueued = (fqtBefore != fqtAfter);
    std::cout << "Fault queued: " << (faultQueued ? "YES" : "NO") << "\n";

    if (!faultQueued) {
        testPassed = false;
        std::cout << "ERROR: Fault was not queued (FQT didn't advance)" << '\n';
    }

    // Check if FIP bit is set in IPSR
    bool fipSet = ((ipsrAfter & 0x2) != 0);
    std::cout << "FIP bit set: " << (fipSet ? "YES" : "NO") << "\n";

    if (!fipSet) {
        testPassed = false;
        std::cout << "ERROR: FIP bit was not set in IPSR" << '\n';
    }

    // If fault wasn't queued, dump the fqcsr register to see if there were errors
    if (!faultQueued) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        std::cout << "FQCSR value: 0x" << std::hex << fqcsrVal << std::dec << "\n";
        bool fqof = (fqcsrVal & 0x200) != 0; // Check overflow bit
        bool fqmf = (fqcsrVal & 0x100) != 0; // Check memory fault bit

        std::cout << "FQCSR.fqof (overflow): " << (fqof ? "YES" : "NO") << "\n";
        std::cout << "FQCSR.fqmf (memory fault): " << (fqmf ? "YES" : "NO") << "\n";

        if (fqmf) {
            std::cout << "ERROR: Memory fault bit set in FQCSR" << '\n';
        }
    }

    // Dump the memory where the fault record should be
    if (faultQueued) {
        std::cout << "Dumping fault record in memory:\n";
        memory.dump(fqAddr, sizeof(FaultRecord));

        // Read the first 8 bytes to check cause and other fields
        uint64_t recordData = 0;
        memory.read(fqAddr, 8, recordData);

        unsigned recordCause = recordData & 0xFFF;
        unsigned recordTtyp = (recordData >> 34) & 0x3F;

        std::cout << "Record cause: " << recordCause << "\n";
        std::cout << "Record TTYP: " << recordTtyp << "\n";

        // Verify fields
        bool causeMatch = (recordCause == cause);
        bool ttypMatch = (recordTtyp == static_cast<unsigned>(req.type));

        std::cout << "Cause matches: " << (causeMatch ? "YES" : "NO") << "\n";
        std::cout << "TTYP matches: " << (ttypMatch ? "YES" : "NO") << "\n";

        if (!causeMatch) {
            testPassed = false;
            std::cout << "ERROR: Fault record cause doesn't match expected value" << '\n';
        }

        if (!ttypMatch) {
            testPassed = false;
            std::cout << "ERROR: Fault record TTYP doesn't match expected value" << '\n';
        }

    // Read device ID from first 8 bytes (bits 40-63)
    memory.read(fqAddr, 8, recordData);
    unsigned recordDid = (recordData >> 40) & 0xFFFFFF;

        std::cout << "Record device ID: 0x" << std::hex << recordDid << std::dec << "\n";
        bool didMatch = (recordDid == req.devId);
        std::cout << "Device ID matches: " << (didMatch ? "YES" : "NO") << "\n";

        if (!didMatch) {
            testPassed = false;
            std::cout << "ERROR: Fault record device ID doesn't match expected value" << '\n';
        }
    }

    // Make sure callbacks were called if needed
    // Note: With DDTP.mode = Off, we expect an early fault before callbacks are called
    if (stage1Called) {
        std::cout << "INFO: Stage1 callback was called, but not expected with DDTP.mode = Off" << '\n';
    }

    std::cout << "=== Simple Fault Queue Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testFaultQueueInitialization() {
    std::cout << "=== Fault Queue Initialization Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        bool result = memory.read(addr, size, data);
        std::cout << "Memory read: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << ", result="
                 << (result ? "success" : "fail") << std::dec << "\n";
        return result;
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        std::cout << "Memory write: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << std::dec << "\n";
        return memory.write(addr, size, data);
    });

    // Configure basic capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);
    iommu.reset();

    // Set up a small fault queue (4 entries)
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 4 * sizeof(FaultRecord); i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue base with 4 entries (LOG2SZ-1 = 1)
    uint64_t fqb = (1ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);

    // Read back to verify
    uint64_t fqbRead = iommu.readFqb();
    std::cout << "FQB written: 0x" << std::hex << fqb << ", read back: 0x"
              << fqbRead << std::dec << "\n";
    if (fqb != fqbRead) {
        testPassed = false;
        std::cout << "ERROR: FQB read back value doesn't match written value\n";
    }

    // Initialize head to 0
    iommu.writeFqh(0);
    uint64_t fqhRead = iommu.readFqh();
    if (fqhRead != 0) {
        testPassed = false;
        std::cout << "ERROR: FQH read back value is not 0\n";
    }

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // fqen=1, fie=1
    iommu.writeFqcsr(fqcsr);

    // Check if fqon bit gets set (bit 16)
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
    }

    if (!fqActive) {
        testPassed = false;
        std::cout << "ERROR: Fault queue did not activate!\n";
    }

    std::cout << "=== Fault Queue Initialization Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testFaultQueueOverflow() {
    std::cout << "=== Fault Queue Overflow Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    });

    // Configure basic capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);

    // Set DDTP to Off mode (this will cause a known fault)
    iommu.writeDdtp(0, 3); // Off mode

    // Set up a tiny fault queue (2 entries)
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 2 * sizeof(FaultRecord); i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue base with 2 entries (LOG2SZ-1 = 0)
    uint64_t fqb = (0ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0); // Head at entry 0

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== Fault Queue Overflow Test: FAILED ===\n\n";
        return;
    }

    // Calculate queue capacity from our configuration
    const uint64_t queueCapacity = 2;
    bool overflowDetected = false;

    for (int i = 0; i < 5; ++i) {
        std::cout << "Translation Attempt " << i << ":\n";

        // Create request
        IommuRequest req;
        req.devId = 0x1;        // Simple device ID
        req.hasProcId = false;  // No process ID to keep it simple
        req.iova = 0x1000 + (i * 0x1000);  // Different IOVA each time
        req.type = Ttype::UntransRead;
        req.privMode = PrivilegeMode::User;
        req.size = 4;

        // Perform translation (should fail)
        uint64_t pa = 0;
        unsigned cause = 0;
        bool result = iommu.translate(req, pa, cause);

        std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
                  << ", cause=" << cause << "\n";

        if (result) {
            testPassed = false;
            std::cout << "ERROR: Translation unexpectedly succeeded" << '\n';
        }

        uint64_t fqh = iommu.readFqh();
        uint64_t fqt = iommu.readFqt();
        uint32_t fqcsr = iommu.readFqcsr();

        std::cout << "FQH: " << fqh << ", FQT: " << fqt
                  << ", FQCSR: 0x" << std::hex << fqcsr << std::dec << "\n";

        // Calculate if queue is full and print
        bool isFull = ((fqt + 1) % queueCapacity) == fqh;
        std::cout << "Queue Full: " << (isFull ? "YES" : "NO")
                  << ", Capacity: " << queueCapacity << "\n";

        // Check for overflow flag (bit 9 in FQCSR) - 0x200
        bool fqof = (fqcsr & 0x200) != 0;
        std::cout << "FQOF set: " << (fqof ? "YES" : "NO") << "\n\n";

        if (fqof) {
            overflowDetected = true;
        }

        // Check for memory fault
        bool fqmf = (fqcsr & (1 << 8)) != 0;
        if (fqmf) {
            testPassed = false;
            std::cout << "ERROR: Unexpected memory fault (FQMF) bit set" << '\n';
        }
    }

    // We expect to see the overflow bit set after the first entry
    if (!overflowDetected) {
        testPassed = false;
        std::cout << "ERROR: Overflow bit (FQOF) was never set" << '\n';
    }

    // Get tail after 5 translations
    uint64_t finalFqt = iommu.readFqt();

    // Check that tail doesn't advance after overflow
    if (finalFqt != 1) {
        testPassed = false;
        std::cout << "ERROR: FQT advanced beyond 1 after overflow" << '\n';
    }

    std::cout << "=== Fault Queue Overflow Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

// Updated testMultipleFaultCauses function
void testMultipleFaultCauses() {
    std::cout << "=== Multiple Fault Causes Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks with detailed logging
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        bool result = memory.read(addr, size, data);
        std::cout << "Memory read: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << ", result="
                 << (result ? "success" : "fail") << std::dec << "\n";
        return result;
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        std::cout << "Memory write: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << std::dec << "\n";
        return memory.write(addr, size, data);
    });

    // Configure basic capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);

    // Set DDTP to Off mode (this will cause a known fault)
    iommu.writeDdtp(0, 3); // Off mode (cause 256)

    // Set up fault queue with proper size
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue - make it larger to accommodate potential differences
    for (uint64_t i = 0; i < 4096; i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue with more entries (16 entries, LOG2SZ-1 = 3)
    uint64_t fqb = (3ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0); // Head at entry 0

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== Multiple Fault Causes Test: FAILED ===\n\n";
        return;
    }

    // Test two different transaction types and verify records
    std::vector<Ttype> transactionTypes = {
        Ttype::UntransRead,
        Ttype::UntransWrite
    };

    // Start with a clean FQH/FQT state
    iommu.writeFqh(0);
    std::cout << "Reset FQH=0, FQT=0 at start of test" << '\n';

    // Store the addresses where records are written for later verification
    std::vector<uint64_t> recordAddresses;

    for (size_t i = 0; i < transactionTypes.size(); i++) {
        // Clear any overflow condition from previous test
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 9)) { // FQOF bit
            iommu.writeFqcsr((1 << 9)); // Clear FQOF
            std::cout << "Cleared FQOF bit before test " << i << '\n';
        }

        // Clear IPSR FIP bit
        iommu.writeIpsr((1 << 1)); // Clear FIP
        std::cout << "Cleared FIP bit before test " << i << '\n';

        auto txType = transactionTypes[i];
        std::cout << "Testing Transaction Type: " << static_cast<int>(txType) << "\n";

        // Create request with complex device ID
        IommuRequest req;
        req.devId = 0x1234 + i;  // Use different device ID for each test
        req.hasProcId = false;
        req.iova = 0x1000 + (i * 0x1000);
        req.type = txType;
        req.privMode = PrivilegeMode::User;
        req.size = 4;

        // Check queue state before translation
        uint64_t fqhBefore = iommu.readFqh();
        uint64_t fqtBefore = iommu.readFqt();
        std::cout << "Before translation: FQH=" << fqhBefore << ", FQT=" << fqtBefore << '\n';

        // Run translation (should fail with DDTP = Off)
        uint64_t pa = 0;
        unsigned cause = 0;
        bool result = iommu.translate(req, pa, cause);

        std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
                  << ", cause=" << cause << "\n";

        if (result) {
            testPassed = false;
            std::cout << "ERROR: Translation unexpectedly succeeded" << '\n';
        }

        // Get fault queue state
        uint64_t fqhAfter = iommu.readFqh();
        uint64_t fqtAfter = iommu.readFqt();
        uint32_t ipsr = iommu.readIpsr();

        std::cout << "After translation: FQH=" << fqhAfter << ", FQT=" << fqtAfter << '\n';
        std::cout << "IPSR: 0x" << std::hex << ipsr << std::dec << "\n";

        // Check if the fault record was written
        if (fqtBefore == fqtAfter) {
            testPassed = false;
            std::cout << "ERROR: FQT did not advance after fault" << '\n';
            continue;
        }

        // Find where the record was written by examining the memory writes
        // Note: This is a debug-focused approach to find the correct addresses
        uint64_t startAddress = 0;

        // Dump memory in this range to find the written record
        std::cout << "Scanning memory region to locate fault record:" << '\n';
        for (uint64_t scanAddr = fqAddr + (fqtBefore * 32);
             scanAddr < fqAddr + (fqtBefore * 32) + 64;
             scanAddr += 8)
        {
            uint64_t value = 0;
            memory.read(scanAddr, 8, value);

            // Look for record with expected cause and ttyp pattern
            if ((value & 0xFFF) == cause) {
                startAddress = scanAddr;
                std::cout << "  Found record at 0x" << std::hex << startAddress
                          << " with cause=" << cause << std::dec << '\n';
                break;
            }
        }

        if (startAddress == 0) {
            testPassed = false;
            std::cout << "ERROR: Could not locate fault record in memory" << '\n';
            continue;
        }

        // Store the address for later testing
        recordAddresses.push_back(startAddress);

        // Read record fields from the found address
        uint64_t recordData0 = 0, recordData1 = 0;
        memory.read(startAddress, 8, recordData0);
        memory.read(startAddress + 8, 8, recordData1);

        unsigned recordCause = recordData0 & 0xFFF;
        unsigned recordTtyp = (recordData0 >> 34) & 0x3F;
        unsigned recordDid = recordData1 & 0xFFFFFF;

        std::cout << "Record Cause: " << recordCause << "\n";
        std::cout << "Record TTYP: " << recordTtyp << "\n";
        std::cout << "Record Device ID: 0x" << std::hex << recordDid << std::dec << "\n";

        // Verify fault record contents
        if (recordCause != cause) {
            testPassed = false;
            std::cout << "ERROR: Recorded cause (" << recordCause
                     << ") doesn't match expected cause (" << cause << ")" << '\n';
        }

        if (recordTtyp != static_cast<unsigned>(txType)) {
            testPassed = false;
            std::cout << "ERROR: Recorded TTYP (" << recordTtyp
                     << ") doesn't match expected TTYP ("
                     << static_cast<unsigned>(txType) << ")" << '\n';
        }

        if (recordDid != req.devId) {
            testPassed = false;
            std::cout << "ERROR: Recorded device ID (0x" << std::hex << recordDid
                     << ") doesn't match expected device ID (0x"
                     << req.devId << std::dec << ")" << '\n';
        }

        // Clear FIP bit after checking it
        iommu.writeIpsr((1 << 1));
        std::cout << "Cleared FIP bit after test" << '\n';
    }

    // If we processed two fault records, check the distance between them
    if (recordAddresses.size() == 2) {
        uint64_t recordSize = recordAddresses[1] - recordAddresses[0];
        std::cout << "Actual fault record size appears to be: " << recordSize << " bytes" << '\n';
    }

    // Verify we processed multiple faults
    if (recordAddresses.size() < 2) {
        testPassed = false;
        std::cout << "ERROR: Not enough faults were processed" << '\n';
    }

    std::cout << "=== Multiple Fault Causes Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testMultipleFaultTypes() {
    std::cout << "=== Multiple Fault Types Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    });

    // Configure basic capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);

    // Set up fault queue
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;
    const unsigned recordSize = 40; // Each fault record is 40 bytes

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 8UL * recordSize; i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue (8 entries, LOG2SZ-1 = 2)
    uint64_t fqb = (2ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0); // Head at entry 0

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== Multiple Fault Types Test: FAILED ===\n\n";
        return;
    }

    // Set up different stages to generate different fault types
    bool stage1Called = false;
    iommu.setStage1Cb([&stage1Called](uint64_t /*va*/, unsigned /*privMode*/, bool r, bool w, bool x,
                                      uint64_t& /*gpa*/, unsigned& cause) {
        stage1Called = true;
        if (x) {
            // Instruction page fault
            cause = 12;
        } else if (r) {
            // Read page fault
            cause = 13;
        } else if (w) {
            // Write page fault
            cause = 15;
        }
        return false; // Return false to indicate failure
    });

    bool stage2Called = false;
    iommu.setStage2Cb([&stage2Called](uint64_t /*gpa*/, unsigned /*privMode*/, bool r, bool w, bool x,
                                     uint64_t& /*pa*/, unsigned& cause) {
        stage2Called = true;
        if (x) {
            // Instruction guest page fault
            cause = 20;
        } else if (r) {
            // Read guest page fault
            cause = 21;
        } else if (w) {
            // Write guest page fault
            cause = 23;
        }
        return false; // Return false to indicate failure
    });

    // Set up test configurations for different fault types
    struct FaultTest {
        const char* name;
        Ttype type;
        unsigned expectedCause;
        PrivilegeMode privMode;
        bool useProcessId;
    };

    std::vector<FaultTest> tests = {
        {"Read Page Fault", Ttype::UntransRead, 13, PrivilegeMode::User, false},
        {"Write Page Fault", Ttype::UntransWrite, 15, PrivilegeMode::User, false},
        {"Exec Page Fault", Ttype::UntransExec, 12, PrivilegeMode::User, false},
        {"Read Page Fault (Supervisor)", Ttype::UntransRead, 13, PrivilegeMode::Supervisor, true}
    };

    // Run tests for different fault types
    for (size_t i = 0; i < tests.size(); i++) {
        const auto& test = tests[i];
        std::cout << "\nTest " << i+1 << ": " << test.name << '\n';

        // Clear stage callback flags
        stage1Called = false;
        stage2Called = false;

        // Set DDTP to an appropriate mode for this test
        if (i == 0) {
            // First test: Set the DDTP to a mode that will use stage1 translation
            iommu.writeDdtp(3, 3); // 2LVL mode
        }

        // Create request
        IommuRequest req;
        req.devId = 0x1000 + i;
        req.hasProcId = test.useProcessId;
        req.procId = 0x54321;
        req.iova = 0x2000 + (i * 0x1000);
        req.type = test.type;
        req.privMode = test.privMode;
        req.size = 4;

        // Check queue state before translation
        uint64_t fqhBefore = iommu.readFqh();
        uint64_t fqtBefore = iommu.readFqt();
        std::cout << "Before translation: FQH=" << fqhBefore << ", FQT=" << fqtBefore << '\n';

        // Run translation (should fail)
        uint64_t pa = 0;
        unsigned cause = 0;
        bool result = iommu.translate(req, pa, cause);

        std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
                  << ", cause=" << cause << "\n";

        if (result) {
            testPassed = false;
            std::cout << "ERROR: Translation unexpectedly succeeded" << '\n';
            continue;
        }

        // Check fault queue state
        uint64_t fqhAfter = iommu.readFqh();
        uint64_t fqtAfter = iommu.readFqt();

        std::cout << "After translation: FQH=" << fqhAfter << ", FQT=" << fqtAfter << '\n';
        std::cout << "Stage1 called: " << (stage1Called ? "YES" : "NO") << '\n';
        std::cout << "Stage2 called: " << (stage2Called ? "YES" : "NO") << '\n';

        // Check if appropriate stage callback was called
        if (i == 0 && !stage1Called) {
            std::cout << "WARNING: Stage1 callback wasn't called as expected" << '\n';
        }

        // Check FQT advanced
        if (fqtBefore == fqtAfter) {
            testPassed = false;
            std::cout << "ERROR: FQT did not advance after fault" << '\n';
            continue;
        }

        // Read the fault record
        uint64_t recordAddr = fqAddr + fqtBefore * recordSize;
        uint64_t recordData0 = 0;
        memory.read(recordAddr, 8, recordData0);

        unsigned recordCause = recordData0 & 0xFFF;
        unsigned recordTtyp = (recordData0 >> 34) & 0x3F;

        std::cout << "Record Cause: " << recordCause << "\n";
        std::cout << "Record TTYP: " << recordTtyp << "\n";

        // For page faults, verify the cause matches expected value
        if (recordCause != cause) {
            testPassed = false;
            std::cout << "ERROR: Recorded cause (" << recordCause
                     << ") doesn't match expected cause (" << cause << ")" << '\n';
        }

        if (recordTtyp != static_cast<unsigned>(test.type)) {
            testPassed = false;
            std::cout << "ERROR: Recorded TTYP (" << recordTtyp
                     << ") doesn't match expected TTYP ("
                     << static_cast<unsigned>(test.type) << ")" << '\n';
        }

        // Clear IPSR
        iommu.writeIpsr((1 << 1)); // Clear FIP
    }

    std::cout << "=== Multiple Fault Types Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

// Debug function to examine FaultRecord structure layout
void debugFaultRecordStructure() {
    std::cout << "=== Fault Record Layout Debug ===\n";

    // Create a sample fault record
    FaultRecord record;
    record.cause = 0x102;  // 258
    record.ttyp = 2;
    record.did = 0x4321;
    record.pv = 1;
    record.pid = 0x98765;
    record.priv = 1;
    record.iotval = 0x2000;
    record.iotval2 = 0;

    // Print struct layout
    std::cout << "Struct FaultRecord size: " << sizeof(FaultRecord) << " bytes\n";

    // Print memory representation

    // Interpret FaultRecord as a an array of double words.
    FaultRecDwords recDwords{};
    recDwords.rec = record;
    const auto& dwords = recDwords.dwords;

    for (size_t i = 0; i < dwords.size(); i++)
        std::cout << "Doubleword " << i << ": 0x" << std::hex << dwords.at(i) << std::dec << "\n";

    // Analyze first doubleword
    uint64_t d0 = dwords.at(0);
    std::cout << "D0 breakdown:\n";
    std::cout << "  Cause: 0x" << std::hex << (d0 & 0xFFF) << std::dec << "\n";
    std::cout << "  TTYP: " << ((d0 >> 34) & 0x3F) << "\n";

    // Analyze second doubleword
    uint64_t d1 = dwords.at(1);
    std::cout << "D1 breakdown:\n";
    std::cout << "  DID: 0x" << std::hex << (d1 & 0xFFFFFF) << std::dec << "\n";
    std::cout << "  PV bit position check:" << "\n";
    for (int bit = 24; bit < 40; bit++) {
        std::cout << "    Bit " << bit << ": " << ((d1 >> bit) & 0x1) << "\n";
    }

    // Try different possible PV, PID, PRIV locations
    std::cout << "Possible field locations:\n";
    // Try doubleword 0, different possible bit positions
    std::cout << "  D0 - PV(12): " << ((d0 >> 12) & 0x1) << "\n";
    std::cout << "  D0 - PID(13-32): 0x" << std::hex << ((d0 >> 13) & 0xFFFFF) << std::dec << "\n";
    std::cout << "  D0 - PRIV(33): " << ((d0 >> 33) & 0x1) << "\n";

    // Try doubleword 1, various bit positions
    std::cout << "  D1 - PV(24): " << ((d1 >> 24) & 0x1) << "\n";
    std::cout << "  D1 - PID(25-44): 0x" << std::hex << ((d1 >> 25) & 0xFFFFF) << std::dec << "\n";
    std::cout << "  D1 - PRIV(45): " << ((d1 >> 45) & 0x1) << "\n";

    std::cout << "  D1 - PV(32): " << ((d1 >> 32) & 0x1) << "\n";
    std::cout << "  D1 - PID(33-52): 0x" << std::hex << ((d1 >> 33) & 0xFFFFF) << std::dec << "\n";
    std::cout << "  D1 - PRIV(53): " << ((d1 >> 53) & 0x1) << "\n";

    std::cout << "=== End Debug ===\n\n";
}

void testFaultQueueWithProcessId() {
    std::cout << "=== Fault Queue with Process ID Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    });

    // Configure basic capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    caps |= (1ULL << 38); // PD8 - Process context support
    iommu.configureCapabilities(caps);

    // Set DDTP to an appropriate mode that supports process IDs
    iommu.writeDdtp(3, 3); // 2LVL mode

    // Set up fault queue
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 4 * sizeof(FaultRecord); i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue
    uint64_t fqb = (1ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0);

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== Fault Queue with Process ID Test: FAILED ===\n\n";
        return;
    }

    // Test with a request that has Process ID
    IommuRequest req;
    req.devId = 0x4321;
    req.hasProcId = true;
    req.procId = 0x98765;
    req.iova = 0x2000;
    req.type = Ttype::UntransRead;
    req.privMode = PrivilegeMode::Supervisor;
    req.size = 4;

    // Create a sample FaultRecord with the same values to determine expected PID encoding
    FaultRecord sampleRecord;
    sampleRecord.pv = 1;
    sampleRecord.pid = 0x98765;
    sampleRecord.priv = 1;

    FaultRecDwords frd;   // To reinterpret fault record as an array of double words.
    frd.rec = sampleRecord;
    uint64_t expectedPid = (frd.dwords.at(0) >> 13) & 0xFFFFF;

    std::cout << "Expected encoded PID value: 0x" << std::hex << expectedPid << std::dec << "\n";

    // Run translation (should fail)
    uint64_t pa = 0;
    unsigned cause = 0;
    iommu.translate(req, pa, cause);

    // Read the fault record
    uint64_t fqt = iommu.readFqt();
    if (fqt == 0) {
        std::cout << "ERROR: No fault record was generated" << '\n';
        return;
    }

    uint64_t recordAddr = fqAddr;

    // Read the fault record fields using the CORRECT bit positions
    uint64_t recordData0 = 0, recordData1 = 0;
    memory.read(recordAddr, 8, recordData0);
    memory.read(recordAddr + 8, 8, recordData1);

    unsigned recordCause = recordData0 & 0xFFF;
    unsigned recordTtyp = (recordData0 >> 34) & 0x3F;
    unsigned recordDid = recordData1 & 0xFFFFFF;

    // IMPORTANT: These are the correct bit positions based on our debug findings
    bool recordPv = (recordData0 >> 12) & 0x1;
    unsigned recordPid = (recordData0 >> 13) & 0xFFFFF;
    bool recordPriv = (recordData0 >> 33) & 0x1;

    std::cout << "Record Cause: " << recordCause << "\n";
    std::cout << "Record TTYP: " << recordTtyp << "\n";
    std::cout << "Record Device ID: 0x" << std::hex << recordDid << std::dec << "\n";
    std::cout << "Record PV: " << recordPv << "\n";
    std::cout << "Record Process ID: 0x" << std::hex << recordPid << std::dec << "\n";
    std::cout << "Record PRIV: " << recordPriv << "\n";

    // Verify fault record fields
    if (!recordPv) {
        testPassed = false;
        std::cout << "ERROR: PV bit not set in fault record" << '\n';
    }

    // Test against the expected encoded value, not the original value
    if (recordPid != expectedPid) {
        testPassed = false;
        std::cout << "ERROR: Recorded Process ID 0x" << std::hex << recordPid
                 << " doesn't match expected encoded Process ID 0x"
                 << expectedPid << std::dec << '\n';
    } else {
        std::cout << "SUCCESS: Process ID is correctly encoded!" << '\n';
    }

    if (!recordPriv) {
        testPassed = false;
        std::cout << "ERROR: Recorded PRIV bit doesn't reflect Supervisor mode" << '\n';
    }

    std::cout << "=== Fault Queue with Process ID Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testProcessIdFaultRecord() {
    std::cout << "=== Process ID Fault Record Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Create a sample FaultRecord to determine expected encoding
    FaultRecord sampleRecord;
    sampleRecord.pv = 1;
    sampleRecord.pid = 0x98765;
    sampleRecord.priv = 1;

    FaultRecDwords frd{};   // To reinterpret fault record as an array of double words.
    frd.rec = sampleRecord;
    uint64_t expectedPid = (frd.dwords.at(0) >> 13) & 0xFFFFF;

    std::cout << "Expected encoded PID value: 0x" << std::hex << expectedPid << std::dec << "\n";

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    });

    // Configure basic capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    caps |= (1ULL << 38); // PD8 - Process context support
    iommu.configureCapabilities(caps);

    // Set up fault queue
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 256; i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue (4 entries, LOG2SZ-1 = 1)
    uint64_t fqb = (1ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0);

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Set up a simple request that uses process ID
    IommuRequest req;
    req.devId = 0x1;
    req.hasProcId = true;
    req.procId = 0x98765;
    req.iova = 0x2000;
    req.type = Ttype::UntransRead;
    req.privMode = PrivilegeMode::Supervisor;
    req.size = 4;

    // Run translation (should fail)
    uint64_t pa = 0;
    unsigned cause = 0;
    iommu.translate(req, pa, cause);

    // Read the fault record with detailed analysis
    uint64_t fqt = iommu.readFqt();
    if (fqt == 0) {
        std::cout << "ERROR: No fault record was generated" << '\n';
        std::cout << "=== Process ID Fault Record Test: FAILED ===\n\n";
        return;
    }

    // Dump the entire fault record for thorough debugging
    // const unsigned recordSize = 32; // Try different sizes to confirm actual size
    uint64_t recordAddr = fqAddr;
    std::cout << "Dumping fault record at address 0x" << std::hex << recordAddr << std::dec << ":" << '\n';
    for (unsigned offset = 0; offset < 64; offset += 8) { // Dump 64 bytes to be safe
        uint64_t value = 0;
        memory.read(recordAddr + offset, 8, value);
        std::cout << "  Offset +" << offset << ": 0x" << std::hex << value << std::dec << '\n';
    }

    // Read the fault record fields - try different layouts
    uint64_t recordData0 = 0, recordData1 = 0;
    memory.read(recordAddr, 8, recordData0);
    memory.read(recordAddr + 8, 8, recordData1);

    // Use the correct bit positions
    unsigned recordCause = recordData0 & 0xFFF;
    unsigned recordTtyp = (recordData0 >> 34) & 0x3F;
    unsigned recordDid = (recordData0 >> 40) & 0xFFFFFF;
    bool recordPv = (recordData0 >> 12) & 0x1;
    unsigned recordPid = (recordData0 >> 13) & 0xFFFFF;
    bool recordPriv = (recordData0 >> 33) & 0x1;

    std::cout << "Record Cause: " << recordCause << "\n";
    std::cout << "Record TTYP: " << recordTtyp << "\n";
    std::cout << "Record Device ID: 0x" << std::hex << recordDid << std::dec << "\n";
    std::cout << "Record PV: " << recordPv << "\n";
    std::cout << "Record Process ID: 0x" << std::hex << recordPid << std::dec << "\n";
    std::cout << "Record PRIV: " << recordPriv << "\n";

    // Verify fault record fields
    if (!recordPv) {
        testPassed = false;
        std::cout << "ERROR: PV bit not set in fault record" << '\n';
    }

    // Test against the expected encoded value
    if (recordPid != expectedPid) {
        testPassed = false;
        std::cout << "ERROR: Recorded Process ID 0x" << std::hex << recordPid
                 << " doesn't match expected encoded Process ID 0x"
                 << expectedPid << std::dec << '\n';
    } else {
        std::cout << "SUCCESS: Process ID is correctly encoded!" << '\n';
    }

    if (!recordPriv) {
        testPassed = false;
        std::cout << "ERROR: Recorded PRIV bit doesn't reflect Supervisor mode" << '\n';
    }

    std::cout << "=== Process ID Fault Record Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testDtfBitWithDdtErrors() {
    std::cout << "=== DTF Bit With DDT Errors Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        return memory.read(addr, size, data);
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        return memory.write(addr, size, data);
    });

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);

    // Set up fault queue
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 8 * sizeof(FaultRecord); i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue
    uint64_t fqb = (2ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0);

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== DTF Bit With DDT Errors Test: FAILED ===\n\n";
        return;
    }

    // Set up two device directory tables - one valid, one invalid
    const uint64_t validDdtAddr = 0x30000;
    const uint64_t validDdtPpn = validDdtAddr / 4096;

    const uint64_t invalidDdtAddr = 0x40000;
    const uint64_t invalidDdtPpn = invalidDdtAddr / 4096;

    // Valid DDT - Create device contexts with DTF=0 and DTF=1
    const uint64_t dcSize = 32; // Base format

    // Write DC with DTF=0 (device_id = 0)
    uint64_t tc0 = 1ULL; // V=1, DTF=0
    memory.write(validDdtAddr + 0 * dcSize, 8, tc0);
    memory.write(validDdtAddr + 0 * dcSize + 8, 8, 0);  // iohgatp
    memory.write(validDdtAddr + 0 * dcSize + 16, 8, 0); // ta
    memory.write(validDdtAddr + 0 * dcSize + 24, 8, 0); // fsc

    // Write DC with DTF=1 (device_id = 1)
    uint64_t tc1 = 1ULL | (1ULL << 6); // V=1, DTF=1
    memory.write(validDdtAddr + 1 * dcSize, 8, tc1);
    memory.write(validDdtAddr + 1 * dcSize + 8, 8, 0);  // iohgatp
    memory.write(validDdtAddr + 1 * dcSize + 16, 8, 0); // ta
    memory.write(validDdtAddr + 1 * dcSize + 24, 8, 0); // fsc

    // Invalid DDT - Create invalid device context (V=0) for both device IDs
    memory.write(invalidDdtAddr + 0 * dcSize, 8, 0);  // V=0
    memory.write(invalidDdtAddr + 1 * dcSize, 8, 0);  // V=0

    // Test cases to verify DTF behavior
    struct DtfTestCase {
        const char* name;
        uint64_t ddtPpn;
        unsigned deviceId;
        unsigned expectedCause;
        bool shouldRespectDtf;
    };

    std::vector<DtfTestCase> testCases = {
        // Valid DDT, DTF=0, should see fault reported
        {"Valid DDT, DTF=0", validDdtPpn, 0, 0, true},

        // Valid DDT, DTF=1, should NOT see fault reported for faults that respect DTF
        {"Valid DDT, DTF=1", validDdtPpn, 1, 0, true},

        // Invalid DDT, device_id=0, should see "DDT entry not valid" (cause=258)
        {"Invalid DDT, device_id=0", invalidDdtPpn, 0, 258, true},

        // Invalid DDT, device_id=1, should NOT see fault if DTF=1 is respected
        {"Invalid DDT, device_id=1", invalidDdtPpn, 1, 258, true}
    };

    for (const auto& test : testCases) {
        std::cout << "\nTesting: " << test.name << "\n";

        // Set DDTP to 1LVL mode with the appropriate DDT base
        uint64_t ddtp = (1ULL << 0) | (test.ddtPpn << 10); // 1 for 1LVL mode
        iommu.writeDdtp(ddtp, 3);

        // Get current fault queue state
        uint64_t fqhBefore = iommu.readFqh();
        uint64_t fqtBefore = iommu.readFqt();
        std::cout << "Before translation: FQH=" << fqhBefore << ", FQT=" << fqtBefore << "\n";

        // Create request
        IommuRequest req;
        req.devId = test.deviceId;
        req.hasProcId = false;
        req.iova = 0x2000;
        req.type = Ttype::UntransRead;
        req.privMode = PrivilegeMode::User;
        req.size = 4;

        // Run translation
        uint64_t pa = 0;
        unsigned cause = 0;
        bool result = iommu.translate(req, pa, cause);

        std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
                  << ", cause=" << cause << "\n";

        // Only for tests that should fail, check the cause
        if (!result && test.expectedCause != 0) {
            if (cause != test.expectedCause) {
                testPassed = false;
                std::cout << "ERROR: Cause code " << cause
                         << " doesn't match expected " << test.expectedCause << '\n';
            }
        }

        // Check fault queue state
        uint64_t fqhAfter = iommu.readFqh();
        uint64_t fqtAfter = iommu.readFqt();
        std::cout << "After translation: FQH=" << fqhAfter << ", FQT=" << fqtAfter << "\n";

        bool faultReported = (fqtBefore != fqtAfter);
        std::cout << "Fault reported: " << (faultReported ? "YES" : "NO") << "\n";

        // For DTF=1 and faults that should respect DTF, no fault should be reported
        if (test.deviceId == 1 && test.shouldRespectDtf && faultReported) {
            testPassed = false;
            std::cout << "ERROR: DTF=1 but fault was still reported (FQT advanced)" << '\n';
        }

        // For DTF=0 and failures, fault should be reported
        if (test.deviceId == 0 && !result && !faultReported) {
            testPassed = false;
            std::cout << "ERROR: DTF=0 and translation failed but fault was not reported" << '\n';
        }

        // If a fault was reported, check that the record contains the correct cause
        if (faultReported) {
            uint64_t recordAddr = fqAddr + fqtBefore * sizeof(FaultRecord);
            uint64_t recordData = 0;
            memory.read(recordAddr, 8, recordData);
            unsigned recordCause = recordData & 0xFFF;

            if (recordCause != cause) {
                testPassed = false;
                std::cout << "ERROR: Recorded cause (" << recordCause
                         << ") doesn't match expected cause (" << cause << ")" << '\n';
            }
        }
    }

    std::cout << "=== DTF Bit With DDT Errors Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testEndiannessSbeField() {
    std::cout << "=== Device Context SBE Field Endianness Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        bool result = memory.read(addr, size, data);
        std::cout << "Memory read: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << ", result="
                 << (result ? "success" : "fail") << std::dec << "\n";
        return result;
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        std::cout << "Memory write: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << std::dec << "\n";
        return memory.write(addr, size, data);
    });

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    caps |= (1ULL << 27); // END - support for both endianness
    caps |= (1ULL << 38); // PD8 - Process context support
    iommu.configureCapabilities(caps);

    // Set up fault queue
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Configure fault queue
    uint64_t fqb = (2ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0);

    // Enable fault queue
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Set up device directory table and process tables
    const uint64_t ddtAddr = 0x30000;
    const uint64_t ddtPpn = ddtAddr / 4096;

    // Setup two different PDTs for different endianness tests
    const uint64_t pdtLeAddr = 0x40000;  // Little-endian PDT
    const uint64_t pdtBeAddr = 0x50000;  // Big-endian PDT
    const uint64_t pdtLePpn = pdtLeAddr / 4096;
    const uint64_t pdtBePpn = pdtBeAddr / 4096;

    // Set DDTP to 1LVL mode
    uint64_t ddtp = (1ULL << 0) | (ddtPpn << 10); // 1 for 1LVL mode
    iommu.writeDdtp(ddtp, 3);

    // Set FCTL to support endianness
    iommu.writeFctl(0); // Little-endian for main IOMMU

    // Create device contexts with different SBE settings
    const uint64_t dcSize = 32; // Base format

    // Write DC with SBE=0 (Little-endian, device_id = 0)
    uint64_t tc0 = 1ULL | (1ULL << 10); // V=1, PDTV=1, SBE=0
    uint64_t pdtp0 = (1ULL << 60) | pdtLePpn; // MODE=1 (PD8), PPN points to LE PDT

    memory.write(ddtAddr + 0 * dcSize, 8, tc0);
    memory.write(ddtAddr + 0 * dcSize + 8, 8, 0);   // iohgatp
    memory.write(ddtAddr + 0 * dcSize + 16, 8, 0);  // ta
    memory.write(ddtAddr + 0 * dcSize + 24, 8, pdtp0); // fsc with PD8 mode

    // Write DC with SBE=1 (Big-endian, device_id = 1)
    uint64_t tc1 = 1ULL | (1ULL << 10) | (1ULL << 8); // V=1, PDTV=1, SBE=1
    uint64_t pdtp1 = (1ULL << 60) | pdtBePpn; // MODE=1 (PD8), PPN points to BE PDT

    memory.write(ddtAddr + 1 * dcSize, 8, tc1);
    memory.write(ddtAddr + 1 * dcSize + 8, 8, 0);   // iohgatp
    memory.write(ddtAddr + 1 * dcSize + 16, 8, 0);  // ta
    memory.write(ddtAddr + 1 * dcSize + 24, 8, pdtp1); // fsc with PD8 mode

    // Create Little-endian Process Context (for device_id=0)
    // First, create PDT with valid entry
    memory.write(pdtLeAddr, 8, 1); // V=1, PPN not relevant for this test

    // Process context for process_id=0 (little-endian format - less significant bits first)
    const uint64_t leProcessId = 0x12345;
    const uint64_t leProcessCtxAddr = pdtLeAddr + leProcessId * 16;

    // Little-endian PC - Write in little-endian format
    uint64_t lePcVal0 = 0x0000000000000001; // V=1, other fields 0
    uint64_t lePcVal1 = 0x0000000000000000; // All fields 0

    // Write LE process context
    memory.write(leProcessCtxAddr, 8, lePcVal0);
    memory.write(leProcessCtxAddr + 8, 8, lePcVal1);

    // Create Big-endian Process Context (for device_id=1)
    // First, create PDT with valid entry
    memory.write(pdtBeAddr, 8, 1); // V=1, PPN not relevant for this test

    // Process context for process_id=0 (big-endian format - most significant bits first)
    const uint64_t beProcessId = 0x12345;
    const uint64_t beProcessCtxAddr = pdtBeAddr + beProcessId * 16;

    // Big-endian PC - Need to byte-swap the values before writing
    uint64_t bePcVal0 = 0x0100000000000000; // Byte-swapped V=1
    uint64_t bePcVal1 = 0x0000000000000000; // All fields 0

    // Write BE process context
    memory.write(beProcessCtxAddr, 8, bePcVal0);
    memory.write(beProcessCtxAddr + 8, 8, bePcVal1);

    // Test cases - one for each endianness
    for (int devId = 0; devId < 2; devId++) {
        bool isBigEndian = (devId == 1);
        std::cout << "\nTesting device_id=" << devId << " (SBE=" << isBigEndian << ")\n";

        IommuRequest req;
        req.devId = devId;
        req.hasProcId = true;
        req.procId = isBigEndian ? beProcessId : leProcessId;
        req.iova = 0x2000;
        req.type = Ttype::UntransRead;
        req.privMode = PrivilegeMode::User;
        req.size = 4;

        // Get current fault queue state
        uint64_t fqtBefore = iommu.readFqt();
        std::cout << "Before translation: FQT=" << fqtBefore << "\n";

        // Run translation
        uint64_t pa = 0;
        unsigned cause = 0;
        bool result = iommu.translate(req, pa, cause);

        std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
                  << ", cause=" << cause << "\n";

        // Get fault queue state after
        uint64_t fqtAfter = iommu.readFqt();
        std::cout << "After translation: FQT=" << fqtAfter << "\n";

        // Translation should succeed for both cases if SBE is correctly handled
        if (!result) {
            testPassed = false;
            std::cout << "ERROR: Translation failed for device_id=" << devId
                     << " (SBE=" << isBigEndian << ")" << '\n';

            // If a fault was recorded, check its cause
            if (fqtBefore != fqtAfter) {
                uint64_t faultAddr = fqAddr + fqtBefore * sizeof(FaultRecord);
                uint64_t recordData = 0;
                memory.read(faultAddr, 8, recordData);
                unsigned recordCause = recordData & 0xFFF;

                std::cout << "  Fault cause: " << recordCause << '\n';

                // If fault is PDT-related, it may be an endianness issue
                if (recordCause == 265 || recordCause == 266 || recordCause == 267 || recordCause == 269) {
                    std::cout << "  Suspected endianness issue with PDT/PC access" << '\n';
                }
            }
        }
    }

    std::cout << "=== Device Context SBE Field Endianness Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testSbeFieldEndianness() {
    std::cout << "=== Revised SBE Field Endianness Test ===\n";
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks with more verbose logging
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        bool result = memory.read(addr, size, data);
        if (!result) {
            std::cout << "READ ERROR: addr=0x" << std::hex << addr << std::dec
                      << " size=" << size << " (exceeds memory bounds)" << '\n';
        }
        return result;
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        bool result = memory.write(addr, size, data);
        if (!result) {
            std::cout << "WRITE ERROR: addr=0x" << std::hex << addr << std::dec
                      << " size=" << size << " data=0x" << std::hex << data
                      << std::dec << " (exceeds memory bounds)" << '\n';
        }
        return result;
    });

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    caps |= (1ULL << 27); // END - support for both endianness
    caps |= (1ULL << 38); // PD8 - Process context support
    iommu.configureCapabilities(caps);

    // Set up fault queue
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 8 * sizeof(FaultRecord); i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue
    uint64_t fqb = (2ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0);

    // Enable fault queue
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== Revised SBE Field Endianness Test: FAILED ===\n\n";
        return;
    }

    // Set up device directory table and process tables
    const uint64_t ddtAddr = 0x30000;
    const uint64_t ddtPpn = ddtAddr / 4096;

    // Setup two different PDTs for different endianness tests
    const uint64_t pdtLeAddr = 0x40000;  // Little-endian PDT
    const uint64_t pdtBeAddr = 0x50000;  // Big-endian PDT
    const uint64_t pdtLePpn = pdtLeAddr / 4096;
    const uint64_t pdtBePpn = pdtBeAddr / 4096;

    // Set DDTP to 1LVL mode
    uint64_t ddtp = (1ULL << 0) | (ddtPpn << 10); // 1 for 1LVL mode
    iommu.writeDdtp(ddtp, 3);

    // Set FCTL to support endianness
    iommu.writeFctl(0); // Little-endian for main IOMMU

    // Create device contexts with different SBE settings
    const uint64_t dcSize = 32; // Base format

    // Write DC with SBE=0 (Little-endian, device_id = 0)
    uint64_t tc0 = 1ULL | (1ULL << 10); // V=1, PDTV=1, SBE=0
    uint64_t pdtp0 = (1ULL << 60) | pdtLePpn; // MODE=1 (PD8), PPN points to LE PDT

    memory.write(ddtAddr + 0 * dcSize, 8, tc0);
    memory.write(ddtAddr + 0 * dcSize + 8, 8, 0);   // iohgatp
    memory.write(ddtAddr + 0 * dcSize + 16, 8, 0);  // ta
    memory.write(ddtAddr + 0 * dcSize + 24, 8, pdtp0); // fsc with PD8 mode

    // Write DC with SBE=1 (Big-endian, device_id = 1)
    uint64_t tc1 = 1ULL | (1ULL << 10) | (1ULL << 8); // V=1, PDTV=1, SBE=1
    uint64_t pdtp1 = (1ULL << 60) | pdtBePpn; // MODE=1 (PD8), PPN points to BE PDT

    memory.write(ddtAddr + 1 * dcSize, 8, tc1);
    memory.write(ddtAddr + 1 * dcSize + 8, 8, 0);   // iohgatp
    memory.write(ddtAddr + 1 * dcSize + 16, 8, 0);  // ta
    memory.write(ddtAddr + 1 * dcSize + 24, 8, pdtp1); // fsc with PD8 mode

    // Use small process_id values to avoid memory access issues
    const uint64_t leProcessId = 0x5; // Small value to avoid memory issues
    const uint64_t beProcessId = 0x5;

    // Clear memory for PDTs
    for (uint64_t i = 0; i < 4096; i += 8) {
        memory.write(pdtLeAddr + i, 8, 0);
        memory.write(pdtBeAddr + i, 8, 0);
    }

    // Create Little-endian Process Context (for device_id=0)
    // First, create valid PDT entries
    memory.write(pdtLeAddr, 8, 1); // V=1 for all PDT entries to simplify testing

    // Process context for process_id=5 (little-endian format - less significant bits first)
    const uint64_t leProcessCtxAddr = pdtLeAddr + 0x100; // Fixed offset for PC

    // Little-endian PC - Write in little-endian format
    uint64_t lePcVal0 = 0x0000000000000001; // V=1, other fields 0
    uint64_t lePcVal1 = 0x0000000012345678; // Distinctive value for testing

    // Write LE process context and create a mapping in PDT
    memory.write(leProcessCtxAddr, 8, lePcVal0);
    memory.write(leProcessCtxAddr + 8, 8, lePcVal1);
    memory.write(pdtLeAddr + leProcessId * 8, 8, 0x1 | ((leProcessCtxAddr / 4096) << 10)); // V=1, PPN points to PC

    // Create Big-endian Process Context (for device_id=1)
    memory.write(pdtBeAddr, 8, 1); // V=1 for all PDT entries

    // Process context for process_id=5 (big-endian format - most significant bits first)
    const uint64_t beProcessCtxAddr = pdtBeAddr + 0x100; // Fixed offset for PC

    // Big-endian PC - Need to byte-swap the values
    uint64_t bePcVal0 = 0x0100000000000000; // Byte-swapped V=1
    uint64_t bePcVal1 = 0x7856341200000000; // Byte-swapped distinctive value

    // Write BE process context and create a mapping in PDT
    memory.write(beProcessCtxAddr, 8, bePcVal0);
    memory.write(beProcessCtxAddr + 8, 8, bePcVal1);
    memory.write(pdtBeAddr + beProcessId * 8, 8, 0x1 | ((beProcessCtxAddr / 4096) << 10)); // V=1, PPN points to PC

    std::cout << "Memory setup complete - PDT and PC entries created\n";
    std::cout << "LE PC at 0x" << std::hex << leProcessCtxAddr << std::dec << "\n";
    std::cout << "BE PC at 0x" << std::hex << beProcessCtxAddr << std::dec << "\n";

    // Test cases - one for each endianness
    for (int devId = 0; devId < 2; devId++) {
        bool isBigEndian = (devId == 1);
        std::cout << "\nTesting device_id=" << devId << " (SBE=" << isBigEndian << ")\n";

        IommuRequest req;
        req.devId = devId;
        req.hasProcId = true;
        req.procId = isBigEndian ? beProcessId : leProcessId;
        req.iova = 0x2000;
        req.type = Ttype::UntransRead;
        req.privMode = PrivilegeMode::User;
        req.size = 4;

        // Get current fault queue state
        uint64_t fqtBefore = iommu.readFqt();
        std::cout << "Before translation: FQT=" << fqtBefore << "\n";

        // Run translation
        uint64_t pa = 0;
        unsigned cause = 0;
        bool result = iommu.translate(req, pa, cause);

        std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
                  << ", cause=" << cause << "\n";

        // Get fault queue state after
        uint64_t fqtAfter = iommu.readFqt();
        std::cout << "After translation: FQT=" << fqtAfter << "\n";

        // Check if a fault was reported
        bool faultReported = (fqtBefore != fqtAfter);

        // If this is a big-endian test and translation failed with PDT-related errors,
        // it might indicate an endianness issue
        if (isBigEndian && !result) {
            if (cause == 265 || cause == 266 || cause == 267 || cause == 269) {
                testPassed = false;
                std::cout << "ERROR: Big-endian PDT/PC access failed - likely endianness issue\n";
            }
        }

        // If translation should succeed but failed
        if (!result) {
            testPassed = false;
            std::cout << "ERROR: Translation failed for device_id=" << devId
                     << " (SBE=" << isBigEndian << ")" << '\n';

            // If a fault was recorded, check its cause
            if (faultReported) {
                uint64_t faultAddr = fqAddr + fqtBefore * sizeof(FaultRecord);
                uint64_t recordData = 0;
                memory.read(faultAddr, 8, recordData);
                unsigned recordCause = recordData & 0xFFF;

                std::cout << "  Fault cause: " << recordCause << '\n';

                // If fault is PDT-related, it may be an endianness issue
                if (recordCause == 265 || recordCause == 266 || recordCause == 267 || recordCause == 269) {
                    std::cout << "  Suspected endianness issue with PDT/PC access" << '\n';
                }
            }
        }
    }

    std::cout << "=== Revised SBE Field Endianness Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testTranslateFailFaultQueueRecord() {
    std::cout << "=== Translate Fail Fault Queue Record Test ===" << '\n';
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks with detailed logging
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        bool result = memory.read(addr, size, data);
        std::cout << "Memory read: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << ", result="
                 << (result ? "success" : "fail") << std::dec << "\n";
        return result;
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        std::cout << "Memory write: addr=0x" << std::hex << addr << ", size=" << std::dec
                 << size << ", data=0x" << std::hex << data << std::dec << "\n";
        return memory.write(addr, size, data);
    });

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);
    iommu.reset(); // Make sure IOMMU is in a clean state

    // Set up fault queue
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 4 * sizeof(FaultRecord); i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue with 4 entries (LOG2SZ-1 = 1)
    uint64_t fqb = (1ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0); // Head at entry 0

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
        std::cout << "Waiting for fault queue to activate..." << '\n';
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== Translate Fail Fault Queue Record Test: FAILED ===\n\n";
        return;
    }

    // Set DDTP to Off mode - this should cause all transactions to be disallowed (cause 256)
    iommu.writeDdtp(0, 3); // Off mode

    // Get initial state
    uint64_t fqhBefore = iommu.readFqh();
    uint64_t fqtBefore = iommu.readFqt();
    uint32_t ipsrBefore = iommu.readIpsr();

    std::cout << "Initial: FQH=" << fqhBefore << ", FQT=" << fqtBefore
              << ", IPSR=0x" << std::hex << ipsrBefore << std::dec << "\n";

    // Setup translation stubs - these shouldn't be called since DDTP is Off
    iommu.setStage1Cb([](uint64_t /*va*/, unsigned /*privMode*/, bool , bool , bool ,
                         uint64_t& /*gpa*/, unsigned& /*cause*/) {
        std::cout << "Stage1 callback called unexpectedly" << '\n';
        return true;
    });

    iommu.setStage2Cb([](uint64_t /*gpa*/, unsigned /*privMode*/, bool , bool , bool ,
                         uint64_t& /*pa*/, unsigned& /*cause*/) {
        std::cout << "Stage2 callback called unexpectedly" << '\n';
        return true;
    });

    // Create a simple request
    IommuRequest req;
    req.devId = 0x123;        // Device ID
    req.hasProcId = false;    // No process ID
    req.iova = 0x2000;        // IOVA
    req.type = Ttype::UntransRead;
    req.privMode = PrivilegeMode::User;
    req.size = 4;

    // Run translation (should fail with cause = 256)
    uint64_t pa = 0;
    unsigned cause = 0;
    bool result = iommu.translate(req, pa, cause);

    std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
              << ", cause=" << cause << "\n";

    // Verify translation failed correctly
    if (result || cause != 256) {
        testPassed = false;
        std::cout << "ERROR: Expected translation to fail with cause 256 but got "
                  << "result=" << result << ", cause=" << cause << '\n';
    }

    // Check fault queue state
    uint64_t fqhAfter = iommu.readFqh();
    uint64_t fqtAfter = iommu.readFqt();
    uint32_t ipsrAfter = iommu.readIpsr();

    std::cout << "After: FQH=" << fqhAfter << ", FQT=" << fqtAfter
              << ", IPSR=0x" << std::hex << ipsrAfter << std::dec << "\n";

    // Check if FQT advanced
    if (fqtBefore == fqtAfter) {
        testPassed = false;
        std::cout << "ERROR: FQT did not advance after fault" << '\n';
    }

    // Check if interrupt was signaled
    bool fipSet = ((ipsrAfter & 0x2) != 0);
    std::cout << "FIP bit set: " << (fipSet ? "YES" : "NO") << "\n";

    if (!fipSet) {
        testPassed = false;
        std::cout << "ERROR: FIP bit was not set in IPSR" << '\n';
    }

    // Read the fault record from memory
    uint64_t recordAddr = fqAddr + fqtBefore * sizeof(FaultRecord);
    std::cout << "Reading fault record from address 0x" << std::hex << recordAddr << std::dec << "\n";

    // Dump memory in this range to help debug
    std::cout << "Dumping memory region to find fault record:" << '\n';
    memory.dump(recordAddr, sizeof(FaultRecord));

    // Read the first 16 bytes (2 doublewords) of the fault record
    uint64_t recordData0 = 0, recordData1 = 0;
    memory.read(recordAddr, 8, recordData0);
    memory.read(recordAddr + 8, 8, recordData1);

    std::cout << "Record data0: 0x" << std::hex << recordData0 << std::dec << "\n";
    std::cout << "Record data1: 0x" << std::hex << recordData1 << std::dec << "\n";

    // Extract the fields from the fault record
    unsigned recordCause = recordData0 & 0xFFF;
    unsigned recordTtyp = (recordData0 >> 34) & 0x3F;
    unsigned recordDid = (recordData0 >> 40) & 0xFFFFFF;

    std::cout << "Record cause: " << recordCause << "\n";
    std::cout << "Record TTYP: " << recordTtyp << "\n";
    std::cout << "Record device ID: 0x" << std::hex << recordDid << std::dec << "\n";

    // Validate fault record
    bool causeMatch = (recordCause == cause);
    bool ttypMatch = (recordTtyp == static_cast<unsigned>(req.type));
    bool didMatch = (recordDid == req.devId);

    std::cout << "Cause matches: " << (causeMatch ? "YES" : "NO") << "\n";
    std::cout << "TTYP matches: " << (ttypMatch ? "YES" : "NO") << "\n";
    std::cout << "DID matches: " << (didMatch ? "YES" : "NO") << "\n";

    if (!causeMatch) {
        testPassed = false;
        std::cout << "ERROR: Fault record cause " << recordCause
                  << " doesn't match expected cause " << cause << '\n';
    }

    if (!ttypMatch) {
        testPassed = false;
        std::cout << "ERROR: Fault record TTYP doesn't match request TTYP" << '\n';
    }

    if (!didMatch) {
        testPassed = false;
        std::cout << "ERROR: Fault record DID doesn't match request device ID" << '\n';
    }

    std::cout << "=== Translate Fail Fault Queue Record Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

void testFaultQueueOverflow1() {
    std::cout << "=== Fault Queue Overflow Test ===" << '\n';
    bool testPassed = true;

    // Create a memory model and IOMMU
    const uint64_t memorySize = 1024 * 1024UL; // 1MB
    TestMemory memory(memorySize);
    Iommu iommu(0x1000, 0x800, memory.size());

    // Configure memory callbacks
    iommu.setMemReadCb([&memory](uint64_t addr, unsigned size, uint64_t& data) {
        bool result = memory.read(addr, size, data);
        // For debugging, uncomment:
        // std::cout << "Memory read: addr=0x" << std::hex << addr << ", size=" << std::dec
        //          << size << ", data=0x" << std::hex << data << std::dec << "\n";
        return result;
    });

    iommu.setMemWriteCb([&memory](uint64_t addr, unsigned size, uint64_t data) {
        // For debugging, uncomment:
        // std::cout << "Memory write: addr=0x" << std::hex << addr << ", size=" << std::dec
        //          << size << ", data=0x" << std::hex << data << std::dec << "\n";
        return memory.write(addr, size, data);
    });

    // Configure capabilities
    uint64_t caps = 0;
    caps |= (1ULL << 8);  // Sv32
    caps |= (1ULL << 9);  // Sv39
    caps |= (1ULL << 16); // Sv32x4
    iommu.configureCapabilities(caps);
    iommu.reset(); // Make sure IOMMU is in a clean state

    // Set up a TINY fault queue (2 entries) to easily test overflow
    const uint64_t fqAddr = 0x10000;
    const uint64_t fqPpn = fqAddr / 4096;

    // Clear memory for fault queue
    for (uint64_t i = 0; i < 256; i += 8) {
        memory.write(fqAddr + i, 8, 0);
    }

    // Configure fault queue with only 2 entries (LOG2SZ-1 = 0)
    uint64_t fqb = (0ULL << 0) | (fqPpn << 10);
    iommu.writeFqb(fqb, 3);
    iommu.writeFqh(0); // Head at entry 0

    // Enable fault queue with interrupts
    uint32_t fqcsr = 0x3; // enable and interrupt enable
    iommu.writeFqcsr(fqcsr);

    // Wait for fault queue to be active
    bool fqActive = false;
    for (int i = 0; i < 10; i++) {
        uint32_t fqcsrVal = iommu.readFqcsr();
        if (fqcsrVal & (1 << 16)) { // fqon bit
            fqActive = true;
            break;
        }
        std::cout << "Waiting for fault queue to activate..." << '\n';
    }

    if (!fqActive) {
        std::cout << "ERROR: Fault queue did not activate!" << '\n';
        std::cout << "=== Fault Queue Overflow Test: FAILED ===\n\n";
        return;
    }

    // Set DDTP to Off mode - this should cause all transactions to be disallowed (cause 256)
    iommu.writeDdtp(0, 3);

    // Calculate queue capacity from our configuration
    const uint64_t queueCapacity = 2;
    std::cout << "Fault queue capacity: " << queueCapacity << " entries\n";

    bool overflowDetected = false;

    // Generate more faults than the queue can hold (3 for a 2-entry queue)
    for (int i = 0; i < 3; i++) {
        std::cout << "\nTranslation Attempt " << (i+1) << ":\n";

        // Get current fault queue state
        uint64_t fqhBefore = iommu.readFqh();
        uint64_t fqtBefore = iommu.readFqt();
        uint32_t fqcsrBefore = iommu.readFqcsr();

        std::cout << "Before translation: FQH=" << fqhBefore << ", FQT=" << fqtBefore
                  << ", FQCSR=0x" << std::hex << fqcsrBefore << std::dec << "\n";

        // Check if queue is full
        bool isFull = ((fqtBefore + 1) % queueCapacity) == fqhBefore;
        std::cout << "Queue Full Before: " << (isFull ? "YES" : "NO") << "\n";

        // Check for overflow flag before
        bool fqofBefore = (fqcsrBefore & 0x200) != 0;
        std::cout << "FQOF Before: " << (fqofBefore ? "SET" : "NOT SET") << "\n";

        // Create request with different device ID for each attempt
        IommuRequest req;
        req.devId = 0x100 + i;  // Different device ID each time
        req.hasProcId = false;
        req.iova = 0x1000 + (i * 0x1000);  // Different IOVA each time
        req.type = Ttype::UntransRead;
        req.privMode = PrivilegeMode::User;
        req.size = 4;

        // Perform translation (should fail)
        uint64_t pa = 0;
        unsigned cause = 0;
        bool result = iommu.translate(req, pa, cause);

        std::cout << "Translation result: " << (result ? "SUCCESS" : "FAILED")
                  << ", cause=" << cause << "\n";

        if (result || cause != 256) {
            testPassed = false;
            std::cout << "ERROR: Expected translation to fail with cause 256" << '\n';
        }

        // Check fault queue state after
        uint64_t fqhAfter = iommu.readFqh();
        uint64_t fqtAfter = iommu.readFqt();
        uint32_t fqcsrAfter = iommu.readFqcsr();

        std::cout << "After translation: FQH=" << fqhAfter << ", FQT=" << fqtAfter
                  << ", FQCSR=0x" << std::hex << fqcsrAfter << std::dec << "\n";

        // Check for overflow flag after
        bool fqofAfter = (fqcsrAfter & 0x200) != 0;
        std::cout << "FQOF After: " << (fqofAfter ? "SET" : "NOT SET") << "\n";

        // On the third iteration, we should see the overflow flag set
        if (i == 2 && !fqofAfter) {
            testPassed = false;
            std::cout << "ERROR: Expected FQOF to be set on the third request" << '\n';
        }

        if (fqofAfter) {
            overflowDetected = true;
            break;  // No need to continue once overflow is detected
        }

        // For the first two requests, FQT should advance
        if (i < 2 && fqtBefore == fqtAfter) {
            testPassed = false;
            std::cout << "ERROR: FQT did not advance for request " << (i+1) << '\n';
        }
    }

    // Final state
    uint64_t finalFqh = iommu.readFqh();
    uint64_t finalFqt = iommu.readFqt();
    uint32_t finalFqcsr = iommu.readFqcsr();
    uint32_t finalIpsr = iommu.readIpsr();

    std::cout << "\nFinal state: FQH=" << finalFqh << ", FQT=" << finalFqt
              << ", FQCSR=0x" << std::hex << finalFqcsr
              << ", IPSR=0x" << finalIpsr << std::dec << "\n";

    // We should have detected overflow
    if (!overflowDetected) {
        testPassed = false;
        std::cout << "ERROR: Overflow condition not detected!" << '\n';
    }

    // Check if FIP bit was set
    bool fipSet = (finalIpsr & 0x2) != 0;
    std::cout << "FIP bit: " << (fipSet ? "SET" : "NOT SET") << "\n";

    if (!fipSet) {
        testPassed = false;
        std::cout << "ERROR: FIP bit not set after overflow" << '\n';
    }

    std::cout << "=== Fault Queue Overflow Test: "
              << (testPassed ? "PASSED" : "FAILED") << " ===\n\n";
}

int main() {
    // testFaultQueueInitialization();
    // testSimpleFaultQueue();
    // testFaultQueueOverflow();
    // testMultipleFaultCauses();
    // testMultipleFaultTypes();
    // testFaultQueueWithProcessId();
    // testProcessIdFaultRecord();
    // testDtfBitWithDdtErrors();
    // testEndiannessSbeField();
    // testSbeFieldEndianness();
    testTranslateFailFaultQueueRecord();
    testFaultQueueOverflow1();

    return 0;
}
