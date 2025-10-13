// Test program for IODIR directory cache invalidation commands
// Copyright 2024 Tenstorrent Corporation.

#include "../Iommu.hpp"
#include "../Ats.hpp"
#include <iostream>
#include <cassert>

using namespace TT_IOMMU;

// Simple test callbacks
bool testMemRead(uint64_t addr, unsigned size, uint64_t& data) {
    // Simple test implementation - return dummy data
    data = 0x1234567890abcdefULL;
    return true;
}

bool testMemWrite(uint64_t addr, unsigned size, uint64_t data) {
    // Simple test implementation
    return true;
}

bool testIsReadable(uint64_t addr, PrivilegeMode mode) {
    return true;
}

bool testIsWritable(uint64_t addr, PrivilegeMode mode) {
    return true;
}

void testStage1Config(unsigned mode, unsigned asid, uint64_t ppn, bool sum) {
    // Test implementation
}

void testStage2Config(unsigned mode, unsigned asid, uint64_t ppn) {
    // Test implementation
}

bool testStage1(uint64_t va, unsigned privMode, bool r, bool w, bool x, uint64_t& gpa, unsigned& cause) {
    gpa = va; // Identity mapping for test
    cause = 0;
    return true;
}

bool testStage2(uint64_t gpa, unsigned privMode, bool r, bool w, bool x, uint64_t& pa, unsigned& cause) {
    pa = gpa; // Identity mapping for test
    cause = 0;
    return true;
}

void testStage2TrapInfo(uint64_t& gpa, bool& implicit, bool& write) {
    gpa = 0;
    implicit = false;
    write = false;
}

int main() {
    std::cout << "Testing IODIR Directory Cache Invalidation Commands\n";
    std::cout << "===================================================\n\n";

    // Create IOMMU instance
    uint64_t iommuAddr = 0x10000000;
    uint64_t iommuSize = 0x1000;
    uint64_t memorySize = 0x100000000ULL; // 4GB
    uint64_t capabilities = 0x0000000000000001ULL; // Basic capabilities
    
    Iommu iommu(iommuAddr, iommuSize, memorySize, capabilities);
    
    // Set up callbacks
    iommu.setMemReadCb(testMemRead);
    iommu.setMemWriteCb(testMemWrite);
    iommu.setIsReadableCb(testIsReadable);
    iommu.setIsWritableCb(testIsWritable);
    iommu.setStage1ConfigCb(testStage1Config);
    iommu.setStage2ConfigCb(testStage2Config);
    iommu.setStage1Cb(testStage1);
    iommu.setStage2Cb(testStage2);
    iommu.setStage2TrapInfoCb(testStage2TrapInfo);

    std::cout << "1. Testing IODIR.INVAL_DDT command\n";
    std::cout << "-----------------------------------\n";
    
    // Create IODIR.INVAL_DDT command
    IodirCommand invalDdtCmd;
    invalDdtCmd.func3 = IodirFunc::INVAL_DDT;
    invalDdtCmd.DV = 1;
    invalDdtCmd.DID = 0x123;
    invalDdtCmd.PID = 0; // Reserved for INVAL_DDT
    
    Command cmd(invalDdtCmd);
    
    std::cout << "Executing IODIR.INVAL_DDT with DV=1, DID=0x123\n";
    iommu.executeIodirCommand(cmd);
    std::cout << "\n";
    
    // Test with DV=0 (invalidate all)
    invalDdtCmd.DV = 0;
    invalDdtCmd.DID = 0; // Ignored when DV=0
    Command cmd2(invalDdtCmd);
    
    std::cout << "Executing IODIR.INVAL_DDT with DV=0 (invalidate all)\n";
    iommu.executeIodirCommand(cmd2);
    std::cout << "\n";

    std::cout << "2. Testing IODIR.INVAL_PDT command\n";
    std::cout << "-----------------------------------\n";
    
    // Create IODIR.INVAL_PDT command
    IodirCommand invalPdtCmd;
    invalPdtCmd.func3 = IodirFunc::INVAL_PDT;
    invalPdtCmd.DV = 1; // Must be 1 for INVAL_PDT
    invalPdtCmd.DID = 0x456;
    invalPdtCmd.PID = 0x789;
    
    Command cmd3(invalPdtCmd);
    
    std::cout << "Executing IODIR.INVAL_PDT with DV=1, DID=0x456, PID=0x789\n";
    iommu.executeIodirCommand(cmd3);
    std::cout << "\n";
    
    // Test invalid command (DV=0 for INVAL_PDT)
    invalPdtCmd.DV = 0;
    Command cmd4(invalPdtCmd);
    
    std::cout << "Executing IODIR.INVAL_PDT with DV=0 (should be illegal)\n";
    iommu.executeIodirCommand(cmd4);
    std::cout << "\n";

    std::cout << "3. Testing cache behavior\n";
    std::cout << "--------------------------\n";
    
    // Test cache miss and hit behavior
    DeviceContext dc;
    unsigned cause = 0;
    
    std::cout << "Loading device context for device 0x100 (should be cache miss)\n";
    bool result = iommu.loadDeviceContext(0x100, dc, cause);
    std::cout << "Result: " << (result ? "Success" : "Failed") << ", Cause: " << cause << "\n\n";
    
    std::cout << "Loading device context for device 0x100 again (should be cache hit)\n";
    result = iommu.loadDeviceContext(0x100, dc, cause);
    std::cout << "Result: " << (result ? "Success" : "Failed") << ", Cause: " << cause << "\n\n";
    
    // Test cache invalidation
    std::cout << "Invalidating DDT cache for device 0x100\n";
    invalDdtCmd.func3 = IodirFunc::INVAL_DDT;
    invalDdtCmd.DV = 1;
    invalDdtCmd.DID = 0x100;
    Command cmd5(invalDdtCmd);
    iommu.executeIodirCommand(cmd5);
    std::cout << "\n";
    
    std::cout << "Loading device context for device 0x100 after invalidation (should be cache miss)\n";
    result = iommu.loadDeviceContext(0x100, dc, cause);
    std::cout << "Result: " << (result ? "Success" : "Failed") << ", Cause: " << cause << "\n\n";

    std::cout << "IODIR Directory Cache Invalidation Test Completed Successfully!\n";
    return 0;
}
