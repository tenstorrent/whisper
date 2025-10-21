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
#include <iostream>
#include <cassert>
#include <vector>
#include <string>

// Helper function to test a single CSR
void testCsr(TT_IOMMU::Iommu& iommu, TT_IOMMU::CsrNumber csr, uint64_t writeValue, uint64_t expectedValue, const std::string& csrName, int accessSize = 4) {
    uint64_t addr = iommu.getCsrAddress(csr);
    uint64_t value = 0;

    if (!iommu.write(addr, accessSize, writeValue)) {
        std::cerr << "Write to " << csrName << " failed\n";
        assert(0);
    }

    if (!iommu.read(addr, accessSize, value)) {
        std::cerr << "Read of " << csrName << " failed\n";
    } else {
        std::cerr << csrName << " read: 0x" << std::hex << value << std::dec << '\n';
    }

    assert(value == expectedValue);
}

int main() {
    // Define the IOMMU memory-mapped region
    uint64_t iommuAddr = 0x10000000;
    uint64_t iommuSize = 0x800;
    uint64_t memSize = 4UL*1024*1024;

    TT_IOMMU::Iommu iommu(iommuAddr, iommuSize, memSize);

    // uint64_t ones = ~uint64_t(0);
    uint64_t allBitsSet = (1ULL << 25) | (1ULL << 30) | (1ULL << 31) | (1ULL << 41) | (1ULL << 28) | (1ULL << 29);
    uint32_t writeValue = 0x12345678;

    // Enable all capabilities
    iommu.configureCapabilities(allBitsSet);
    iommu.reset();

    // Test enabled state for pqcsr, pqb, pqh, pqt, iocountovf, iocountinh, iohpmcycles
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqcsr, writeValue, writeValue, "pqcsr");
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqb, writeValue, writeValue, "pqb");
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqh, writeValue, writeValue, "pqh");
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqt, writeValue, writeValue, "pqt");
    testCsr(iommu, TT_IOMMU::CsrNumber::Iocntovf, writeValue, writeValue, "iocountovf");
    testCsr(iommu, TT_IOMMU::CsrNumber::Iocntinh, writeValue, writeValue, "iocountinh");
    testCsr(iommu, TT_IOMMU::CsrNumber::Iohpmcycles, writeValue, writeValue, "iohpmcycles", 8);

    // Test enabled state for tr_req_iova, tr_req_ctl, tr_response, and iommu_qosid
    testCsr(iommu, TT_IOMMU::CsrNumber::TrReqIova, writeValue, writeValue, "tr_req_iova");
    testCsr(iommu, TT_IOMMU::CsrNumber::TrReqCtl, writeValue, writeValue, "tr_req_ctl");
    testCsr(iommu, TT_IOMMU::CsrNumber::TrResponse, writeValue, writeValue, "tr_response");

    testCsr(iommu, TT_IOMMU::CsrNumber::IommuQosid, writeValue, writeValue, "iommu_qosid");


    // Test iohpmctr1-31
    std::cout << "iohpmctr1-31 read: ";
    for (unsigned i = 0; i < 31; ++i) {
        uint64_t addr = iommu.getCsrAddress(static_cast<TT_IOMMU::CsrNumber>(static_cast<uint32_t>(TT_IOMMU::CsrNumber::Iohpmctr1) + i));
        uint64_t value = 0;
        iommu.write(addr, 8, writeValue);
        iommu.read(addr, 8, value);
        std::cout << "0x" << std::hex << value << (i < 31 ? " " : "");
    }
    std::cout << std::dec << '\n';

    // Test iohpmevt1-31
    std::cout << "iohpmevt1-31 read: ";
    for (unsigned i = 0; i < 31; ++i) {
        uint64_t addr = iommu.getCsrAddress(static_cast<TT_IOMMU::CsrNumber>(static_cast<uint32_t>(TT_IOMMU::CsrNumber::Iohpmevt1) + i));
        uint64_t value = 0;
        iommu.write(addr, 8, writeValue);
        iommu.read(addr, 8, value);
        std::cout << "0x" << std::hex << value << (i < 31 ? " " : "");
    }
    std::cout << std::dec << '\n';

    // Test msi_cfg_tbl0-31
    std::cout << "msi_cfg_tbl0-31 read: ";
    for (unsigned i = 0; i < 32; ++i) {
        uint64_t addr = iommu.getCsrAddress(static_cast<TT_IOMMU::CsrNumber>(static_cast<uint32_t>(TT_IOMMU::CsrNumber::MsiAddr0) + i));
        uint64_t value = 0;
        iommu.write(addr, 8, writeValue);
        iommu.read(addr, 8, value);
        std::cout << "0x" << std::hex << value << (i < 31 ? " " : "");
    }
    std::cout << std::dec << '\n';

    // Disable all capabilities
    iommu.configureCapabilities(0);
    iommu.reset();

    // Test disabled state for pqcsr, pqb, pqh, pqt, iocountovf, iocountinh, iohpmcycles
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqcsr, writeValue, 0, "pqcsr");
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqb, writeValue, 0, "pqb");
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqh, writeValue, 0, "pqh");
    testCsr(iommu, TT_IOMMU::CsrNumber::Pqt, writeValue, 0, "pqt");
    testCsr(iommu, TT_IOMMU::CsrNumber::Iocntovf, writeValue, 0, "iocountovf");
    testCsr(iommu, TT_IOMMU::CsrNumber::Iocntinh, writeValue, 0, "iocountinh");
    testCsr(iommu, TT_IOMMU::CsrNumber::Iohpmcycles, writeValue, 0, "iohpmcycles", 8);

    // Test disabled state for tr_req_iova, tr_req_ctl, tr_response, and iommu_qosid
    testCsr(iommu, TT_IOMMU::CsrNumber::TrReqIova, writeValue, 0, "tr_req_iova");
    testCsr(iommu, TT_IOMMU::CsrNumber::TrReqCtl, writeValue, 0, "tr_req_ctl");
    testCsr(iommu, TT_IOMMU::CsrNumber::TrResponse, writeValue, 0, "tr_response");

    testCsr(iommu, TT_IOMMU::CsrNumber::IommuQosid, writeValue, 0, "iommu_qosid");

    // Test disabled iohpmctr1-31
    std::cout << "Disabled iohpmctr1-31 read: ";
    for (unsigned i = 0; i < 31; ++i) {
      uint64_t addr = iommu.getCsrAddress(static_cast<TT_IOMMU::CsrNumber>(static_cast<uint32_t>(TT_IOMMU::CsrNumber::Iohpmctr1) + i));
      uint64_t value = 0;
      iommu.write(addr, 8, writeValue);
      iommu.read(addr, 8, value);
      assert(value == 0);
      std::cout << "0x" << std::hex << value << " ";
    }
    std::cout << std::dec << '\n';

    // Test disabled iohpmevt1-31
    std::cout << "Disabled iohpmevt1-31 read: ";
    for (unsigned i = 0; i < 31; ++i) {
        uint64_t addr = iommu.getCsrAddress(static_cast<TT_IOMMU::CsrNumber>(static_cast<uint32_t>(TT_IOMMU::CsrNumber::Iohpmevt1) + i));
        uint64_t value = 0;
        iommu.write(addr, 8, writeValue);
        iommu.read(addr, 8, value);
        assert(value == 0);
        std::cout << "0x" << std::hex << value << " ";
    }
    std::cout << std::dec << '\n';

    // Test msi_cfg_tbl0-31
    std::cout << "Disabled msi_cfg_tbl0-31 read: ";
    for (unsigned i = 0; i < 16*3; ++i) {
        uint64_t addr = iommu.getCsrAddress(static_cast<TT_IOMMU::CsrNumber>(static_cast<uint32_t>(TT_IOMMU::CsrNumber::MsiAddr0) + i));
        uint64_t value = 0;
        unsigned size = (i % 3 == 0) ? 8 : 4;
        iommu.write(addr, size, writeValue);
        iommu.read(addr, 8, value);
        assert(value == 0);
        std::cout << "0x" << std::hex << value << (i < 31 ? " " : "");
    }
    std::cout << std::dec << '\n';

    return 0;
}

