// IODIR directory cache invalidation test
// Copyright 2024 Tenstorrent Corporation.

#include "../Iommu.hpp"
#include "../Ats.hpp"
#include <iostream>

using namespace TT_IOMMU;

bool testMemRead(uint64_t, unsigned, uint64_t& data) {
    data = 0x1234567890abcdefULL;
    return true;
}

bool testMemWrite(uint64_t, unsigned, uint64_t) { return true; }
bool testIsReadable(uint64_t, PrivilegeMode) { return true; }
bool testIsWritable(uint64_t, PrivilegeMode) { return true; }
void testStage1Config(unsigned, unsigned, uint64_t, bool) {}
void testStage2Config(unsigned, unsigned, uint64_t) {}

bool testStage1(uint64_t va, unsigned, bool, bool, bool, uint64_t& gpa, unsigned& cause) {
    gpa = va;
    cause = 0;
    return true;
}

bool testStage2(uint64_t gpa, unsigned, bool, bool, bool, uint64_t& pa, unsigned& cause) {
    pa = gpa;
    cause = 0;
    return true;
}

void testStage2TrapInfo(uint64_t& gpa, bool& implicit, bool& write) {
    gpa = 0;
    implicit = false;
    write = false;
}

void testCommandStructures() {
    std::cout << "Testing command structures\n";
    std::cout << "==========================\n\n";

    IodirCommand invalDdtCmd;
    invalDdtCmd.func3 = IodirFunc::INVAL_DDT;
    invalDdtCmd.DV = 1;
    invalDdtCmd.DID = 0x123456;
    invalDdtCmd.PID = 0;

    std::cout << "INVAL_DDT: Opcode=" << static_cast<uint32_t>(invalDdtCmd.opcode)
              << " Function=" << static_cast<uint32_t>(invalDdtCmd.func3)
              << " DV=" << invalDdtCmd.DV
              << " DID=0x" << std::hex << invalDdtCmd.DID << std::dec << "\n";

    Command cmd(invalDdtCmd);
    std::cout << "Is IODIR: " << (cmd.isIodir() ? "Yes" : "No")
              << ", Is INVAL_DDT: " << (cmd.isIodirInvalDdt() ? "Yes" : "No")
              << ", Is INVAL_PDT: " << (cmd.isIodirInvalPdt() ? "Yes" : "No") << "\n\n";

    IodirCommand invalPdtCmd;
    invalPdtCmd.func3 = IodirFunc::INVAL_PDT;
    invalPdtCmd.DV = 1;
    invalPdtCmd.DID = 0x789ABC;
    invalPdtCmd.PID = 0x12345;

    std::cout << "INVAL_PDT: Opcode=" << static_cast<uint32_t>(invalPdtCmd.opcode)
              << " Function=" << static_cast<uint32_t>(invalPdtCmd.func3)
              << " DV=" << invalPdtCmd.DV
              << " DID=0x" << std::hex << invalPdtCmd.DID
              << " PID=0x" << invalPdtCmd.PID << std::dec << "\n";

    Command cmd2(invalPdtCmd);
    std::cout << "Is IODIR: " << (cmd2.isIodir() ? "Yes" : "No")
              << ", Is INVAL_DDT: " << (cmd2.isIodirInvalDdt() ? "Yes" : "No")
              << ", Is INVAL_PDT: " << (cmd2.isIodirInvalPdt() ? "Yes" : "No") << "\n\n";

    std::cout << "CommandOpcode::IODIR=" << static_cast<uint32_t>(CommandOpcode::IODIR)
              << ", IodirFunc::INVAL_DDT=" << static_cast<uint32_t>(IodirFunc::INVAL_DDT)
              << ", IodirFunc::INVAL_PDT=" << static_cast<uint32_t>(IodirFunc::INVAL_PDT) << "\n\n";

    AtsCommandData data = cmd.data;
    std::cout << "INVAL_DDT data: DW0=0x" << std::hex << data.dw0
              << " DW1=0x" << data.dw1 << std::dec << "\n";
    data = cmd2.data;
    std::cout << "INVAL_PDT data: DW0=0x" << std::hex << data.dw0
              << " DW1=0x" << data.dw1 << std::dec << "\n\n";
}

void testCommandValidation() {
    std::cout << "Testing command validation\n";
    std::cout << "==========================\n\n";

    IodirCommand cmd1;
    cmd1.func3 = IodirFunc::INVAL_DDT;
    cmd1.DV = 0;
    cmd1.DID = 0;
    std::cout << "INVAL_DDT: DV=0 DID=0 (invalidate all) - Valid\n";

    IodirCommand cmd2;
    cmd2.func3 = IodirFunc::INVAL_DDT;
    cmd2.DV = 1;
    cmd2.DID = 0x123;
    std::cout << "INVAL_DDT: DV=1 DID=0x123 (specific device) - Valid\n";

    IodirCommand cmd3;
    cmd3.func3 = IodirFunc::INVAL_PDT;
    cmd3.DV = 1;
    cmd3.DID = 0x456;
    cmd3.PID = 0x789;
    std::cout << "INVAL_PDT: DV=1 DID=0x456 PID=0x789 - Valid\n";

    IodirCommand cmd4;
    cmd4.func3 = IodirFunc::INVAL_PDT;
    cmd4.DV = 0;
    cmd4.DID = 0x456;
    cmd4.PID = 0x789;
    std::cout << "INVAL_PDT: DV=0 DID=0x456 PID=0x789 - Invalid (DV must be 1)\n\n";
}

void testIommuExecution(Iommu& iommu) {
    std::cout << "Testing IOMMU execution\n";
    std::cout << "=======================\n\n";

    IodirCommand invalDdtCmd;
    invalDdtCmd.func3 = IodirFunc::INVAL_DDT;
    invalDdtCmd.DV = 1;
    invalDdtCmd.DID = 0x123;
    invalDdtCmd.PID = 0;

    Command cmd(invalDdtCmd);
    std::cout << "Executing INVAL_DDT: DV=1 DID=0x123\n";
    iommu.executeIodirCommand(cmd);

    invalDdtCmd.DV = 0;
    invalDdtCmd.DID = 0;
    Command cmd2(invalDdtCmd);
    std::cout << "Executing INVAL_DDT: DV=0 (invalidate all)\n";
    iommu.executeIodirCommand(cmd2);

    IodirCommand invalPdtCmd;
    invalPdtCmd.func3 = IodirFunc::INVAL_PDT;
    invalPdtCmd.DV = 1;
    invalPdtCmd.DID = 0x456;
    invalPdtCmd.PID = 0x789;

    Command cmd3(invalPdtCmd);
    std::cout << "Executing INVAL_PDT: DV=1 DID=0x456 PID=0x789\n";
    iommu.executeIodirCommand(cmd3);

    invalPdtCmd.DV = 0;
    Command cmd4(invalPdtCmd);
    std::cout << "Executing INVAL_PDT: DV=0 (illegal)\n";
    iommu.executeIodirCommand(cmd4);
    std::cout << "\n";
}

void testCacheBehavior(Iommu& iommu) {
    std::cout << "Testing cache behavior\n";
    std::cout << "======================\n\n";

    DeviceContext dc;
    unsigned cause = 0;

    std::cout << "Load device 0x100 (cache miss)\n";
    bool result = iommu.loadDeviceContext(0x100, dc, cause);
    std::cout << "Result: " << (result ? "Success" : "Failed") << " Cause: " << cause << "\n";

    std::cout << "Load device 0x100 again (cache hit)\n";
    result = iommu.loadDeviceContext(0x100, dc, cause);
    std::cout << "Result: " << (result ? "Success" : "Failed") << " Cause: " << cause << "\n";

    std::cout << "Invalidate DDT cache for device 0x100\n";
    IodirCommand invalDdtCmd;
    invalDdtCmd.func3 = IodirFunc::INVAL_DDT;
    invalDdtCmd.DV = 1;
    invalDdtCmd.DID = 0x100;
    Command cmd(invalDdtCmd);
    iommu.executeIodirCommand(cmd);

    std::cout << "Load device 0x100 after invalidation (cache miss)\n";
    result = iommu.loadDeviceContext(0x100, dc, cause);
    std::cout << "Result: " << (result ? "Success" : "Failed") << " Cause: " << cause << "\n\n";
}

int main() {
    std::cout << "IODIR Test\n";
    std::cout << "==========\n\n";

    testCommandStructures();
    testCommandValidation();

    Iommu iommu(0x10000000, 0x1000, 0x100000000ULL, 0x0000000000000001ULL);
    iommu.setMemReadCb(testMemRead);
    iommu.setMemWriteCb(testMemWrite);
    iommu.setIsReadableCb(testIsReadable);
    iommu.setIsWritableCb(testIsWritable);
    iommu.setStage1ConfigCb(testStage1Config);
    iommu.setStage2ConfigCb(testStage2Config);
    iommu.setStage1Cb(testStage1);
    iommu.setStage2Cb(testStage2);
    iommu.setStage2TrapInfoCb(testStage2TrapInfo);

    testIommuExecution(iommu);
    testCacheBehavior(iommu);

    std::cout << "All tests passed\n";
    return 0;
}
