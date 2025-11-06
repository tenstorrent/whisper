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

#include "Ats.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>

using namespace TT_IOMMU;

void testIotinvalVmaCommand()
{
  std::cout << "Testing IOTINVAL.VMA command structure..." << '\n';

  // Test creating IOTINVAL.VMA command with different combinations

  // Test 1: Global invalidation (GV=0, AV=0, PSCV=0)
  IotinvalCommand vmaCmd1(IotinvalFunc::VMA);
  vmaCmd1.GV = 0;
  vmaCmd1.AV = 0;
  vmaCmd1.PSCV = 0;

  Command cmd1(vmaCmd1);
  assert(cmd1.isIotinval() == true);
  assert(cmd1.isIotinvalVma() == true);
  assert(cmd1.isIotinvalGvma() == false);


  // Test 2: Address-specific invalidation (GV=0, AV=1, PSCV=0)
  IotinvalCommand vmaCmd2(IotinvalFunc::VMA);
  vmaCmd2.GV = 0;
  vmaCmd2.AV = 1;
  vmaCmd2.PSCV = 0;
  vmaCmd2.ADDR = 0x12345;  // Page address

  Command cmd2(vmaCmd2);
  assert(cmd2.isIotinvalVma() == true);


  // Test 3: Process and address specific (GV=0, AV=1, PSCV=1)
  IotinvalCommand vmaCmd3(IotinvalFunc::VMA);
  vmaCmd3.GV = 0;
  vmaCmd3.AV = 1;
  vmaCmd3.PSCV = 1;
  vmaCmd3.PSCID = 0x1234;
  vmaCmd3.ADDR = 0x56789;

  Command cmd3(vmaCmd3);
  assert(cmd3.isIotinvalVma() == true);


  std::cout << "  ✓ IOTINVAL.VMA command structure test PASSED!" << '\n' << '\n';
}

void testIotinvalGvmaCommand()
{
  std::cout << "Testing IOTINVAL.GVMA command structure..." << '\n';

  // Test creating IOTINVAL.GVMA command

  // Test 1: Global second-stage invalidation (GV=0, AV=0, PSCV=0)
  IotinvalCommand gvmaCmd1(IotinvalFunc::GVMA);
  gvmaCmd1.GV = 0;
  gvmaCmd1.AV = 0;
  // PSCV is already 0 by constructor for GVMA

  Command cmd1(gvmaCmd1);
  assert(cmd1.isIotinval() == true);
  assert(cmd1.isIotinvalGvma() == true);
  assert(cmd1.isIotinvalVma() == false);


  // Test 2: Guest-specific invalidation (GV=1, AV=0, PSCV=0)
  IotinvalCommand gvmaCmd2(IotinvalFunc::GVMA);
  gvmaCmd2.GV = 1;
  gvmaCmd2.AV = 0;
  // PSCV remains 0 for GVMA
  gvmaCmd2.GSCID = 0x5678;

  Command cmd2(gvmaCmd2);
  assert(cmd2.isIotinvalGvma() == true);


  std::cout << "  ✓ IOTINVAL.GVMA command structure test PASSED!" << '\n' << '\n';
}

void testCommandOpcodes()
{
  std::cout << "Testing IOTINVAL command opcode assignments..." << '\n';

  // Verify opcode values match specification
  assert(static_cast<uint32_t>(CommandOpcode::IOTINVAL) == 1);
  assert(static_cast<uint32_t>(CommandOpcode::IOFENCE) == 2);
  assert(static_cast<uint32_t>(CommandOpcode::ATS) == 4);

  // Verify function codes
  assert(static_cast<uint32_t>(IotinvalFunc::VMA) == 0);
  assert(static_cast<uint32_t>(IotinvalFunc::GVMA) == 1);


  std::cout << "  ✓ Command opcode assignments are correct!" << '\n' << '\n';
}


int main()
{
  std::cout << "RISC-V IOMMU IOTINVAL Command Test" << '\n';
  std::cout << "===================================" << '\n' << '\n';

  testCommandOpcodes();
  testIotinvalVmaCommand();
  testIotinvalGvmaCommand();

  std::cout << "All IOTINVAL command structure tests PASSED!" << '\n';

  return 0;
}
