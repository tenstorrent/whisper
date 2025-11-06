#include <cassert>
#include <iostream>
#include <memory>
#include "Iommu.hpp"
#include "Ats.hpp"
#include "MemoryModel.hpp"

using namespace TT_IOMMU;

class IofenceTestHelper {
private:
    static constexpr uint64_t IOMMU_ADDR = 0x10000;
    static constexpr uint64_t IOMMU_SIZE = 0x10000;
    static constexpr uint64_t MEMORY_SIZE = 0x10000000; // 256MB

    std::unique_ptr<Iommu> iommu_;
    std::unique_ptr<MemoryModel> memory_;

public:
    IofenceTestHelper() {
        memory_ = std::make_unique<MemoryModel>(MEMORY_SIZE);

        // Set IOMMU capabilities for ATS support
        Capabilities caps{};
        caps.fields.ats = 1;        // Enable ATS support
        caps.fields.msi_flat = 1;   // Enable MSI support
        caps.fields.amo_hwad = 1;   // Enable AMO support
        caps.fields.pd17 = 1;       // Support 17-bit process directory
        caps.fields.pd8 = 1;        // Support 8-bit process directory

        iommu_ = std::make_unique<Iommu>(IOMMU_ADDR, IOMMU_SIZE, MEMORY_SIZE, caps.value);

        // Set up memory callbacks
        iommu_->setMemReadCb([this](uint64_t addr, unsigned size, uint64_t& data) {
            return memory_->read(addr, size, data);
        });

        iommu_->setMemWriteCb([this](uint64_t addr, unsigned size, uint64_t data) {
            return memory_->write(addr, size, data);
        });
    }

    Iommu& getIommu() { return *iommu_; }
    MemoryModel& getMemory() { return *memory_; }

    void setupCommandQueue() {
        // Set up command queue at 0x1000000
        uint64_t cqbAddr = 0x1000000;

        // Configure command queue base register
        // Format: PPN[53:12] (42 bits) | LOG2SZ_1[4:0] (5 bits) | ENABLE (1 bit)
        uint64_t cqbValue = ((cqbAddr >> 12) << 10) | (10 << 1) | 1; // 2^10 = 1024 entries, enabled
        iommu_->writeCqb(cqbValue, 3);

        // Initialize head and tail to 0
        iommu_->writeCqt(0);

        // Enable command queue
        uint64_t cqcsr = (1ULL << 16) | 1; // cqon bit and cqen bit
        iommu_->writeCqcsr(cqcsr);
    }
};

void testBasicIofence()
{
    std::cout << "\n=== Test 1: Basic IOFENCE.C (No Pending ATS) ===" << '\n';

    IofenceTestHelper helper;
    helper.setupCommandQueue();

    // Create IOFENCE.C command
    IofenceCCommand iofenceCmd;
    iofenceCmd.opcode = CommandOpcode::IOFENCE;
    iofenceCmd.func3 = IofenceFunc::C;
    iofenceCmd.AV = 0;     // No address/data write
    iofenceCmd.WSI = 0;    // No wire-signaled interrupt
    iofenceCmd.PR = 0;     // No previous read ordering
    iofenceCmd.PW = 0;     // No previous write ordering
    iofenceCmd.DATA = 0;
    iofenceCmd.ADDR = 0;

    // Write command to queue
    uint64_t cqbAddr = 0x1000000;
    Command cmd(iofenceCmd);
    helper.getMemory().write(cqbAddr, 8, cmd.dw0());
    helper.getMemory().write(cqbAddr + 8, 8, cmd.dw1());

    // Update tail to trigger processing
    helper.getIommu().writeCqt(1);

    // Verify command was processed (head advanced)
    uint64_t newHead = helper.getIommu().readCqh();
    assert(newHead == 1);
    std::cout << "✓ Basic IOFENCE.C processed successfully (no pending ATS)" << '\n';
}

void testIofenceWithMemoryWrite()
{
    std::cout << "\n=== Test 2: IOFENCE.C with Memory Write (AV=1) ===" << '\n';

    IofenceTestHelper helper;
    helper.setupCommandQueue();

    uint64_t targetAddr = 0x2000000;
    uint32_t targetData = 0xDEADBEEF;

    // Create IOFENCE.C command with memory write
    IofenceCCommand iofenceCmd;
    iofenceCmd.opcode = CommandOpcode::IOFENCE;
    iofenceCmd.func3 = IofenceFunc::C;
    iofenceCmd.AV = 1;     // Enable address/data write
    iofenceCmd.WSI = 0;    // No wire-signaled interrupt
    iofenceCmd.PR = 0;     // No previous read ordering
    iofenceCmd.PW = 0;     // No previous write ordering
    iofenceCmd.DATA = targetData;
    iofenceCmd.ADDR = targetAddr >> 2; // ADDR[63:2] for 4-byte aligned address

    // Write command to queue
    uint64_t cqbAddr = 0x1000000;
    Command cmd(iofenceCmd);
    helper.getMemory().write(cqbAddr, 8, cmd.dw0());
    helper.getMemory().write(cqbAddr + 8, 8, cmd.dw1());

    // Update tail to trigger processing
    helper.getIommu().writeCqt(1);

    // Verify command was processed (head advanced)
    uint64_t newHead = helper.getIommu().readCqh();
    assert(newHead == 1);

    // Verify memory was written correctly
    uint64_t readData = 0;
    bool readSuccess = helper.getMemory().read(targetAddr, 4, readData);
    assert(readSuccess);
    assert((uint32_t)readData == targetData);

    std::cout << "✓ IOFENCE.C with memory write successful (wrote 0x"
              << std::hex << targetData << " to 0x" << targetAddr << ")" << '\n';
}

void testIofenceCommandDetection()
{
    std::cout << "\n=== Test 3: IOFENCE Command Detection ===" << '\n';

    IofenceTestHelper helper;

    // Test IOFENCE.C command detection
    IofenceCCommand iofenceCmd;
    iofenceCmd.opcode = CommandOpcode::IOFENCE;
    iofenceCmd.func3 = IofenceFunc::C;

    Command cmd(iofenceCmd);

    assert(helper.getIommu().isIofenceCommand(cmd));
    assert(helper.getIommu().isIofenceCCommand(cmd));
    assert(!helper.getIommu().isAtsCommand(cmd));

    std::cout << "✓ IOFENCE command detection works correctly" << '\n';
}

int main()
{
    std::cout << "Running IOFENCE Tests..." << '\n';
    std::cout << "=============================" << '\n';

    try {
        testBasicIofence();
        testIofenceWithMemoryWrite();
        testIofenceCommandDetection();

        std::cout << "\nAll IOFENCE tests passed!" << '\n';
    } catch (const std::exception& e) {
        std::cout << "\nTest failed: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
