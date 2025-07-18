// Copyright 2020 Western Digital Corporation or its affiliates.
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

#include <memory>
#include <string>
#include <string_view>
#include <nlohmann/json_fwd.hpp>
#include "aplic/Aplic.hpp"


namespace WdRiscv
{

  template <typename URV>
  class System;

  template <typename URV>
  class Hart;


  /// Manage loading of configuration file and applying it to a core.
  class HartConfig
  {
  public:

    /// Constructor.
    HartConfig();

    /// Destructor.
    ~HartConfig();

    HartConfig(const HartConfig&) = delete;
    void operator= (const HartConfig&) = delete;

    /// Load given configuration file (JSON file) into this object.
    /// Return true on success and false if file cannot be opened or if the file
    /// does not contain a valid JSON object.
    bool loadConfigFile(const std::string& filePath);

    /// Apply the configurations in this object (as loaded by
    /// loadConfigFile) to the given hart. Return true on success and
    /// false on failure. URV stands for unsigned register type and is
    /// the type associated with the integer registers of a hart. Use
    /// uint32_t for 32-bit harts and uint64_t for 64-bit harts.
    template<typename URV>
    bool applyConfig(Hart<URV>&, bool userMode, bool verbose) const;

    /// Apply the configurations in this object to all the given
    /// harts. Finalize CSR configuration by defining callbacks for
    /// non-standard CSRs.
    template<typename URV>
    bool configHarts(System<URV>& system, bool userMode, bool verbose) const;

    /// Configure memory.
    template<typename URV>
    bool configMemory(System<URV>& system, bool unmappedElfOk) const;

    /// Apply the memory configuration in this object. Helper to configMemory.
    template<typename URV>
    bool applyMemoryConfig(Hart<URV>&) const;

    /// Confgure a core-local-interrupt (CLINT) device in the given
    /// address range: clintStart to  clintEnd - 1. If siOnReset
    /// is true then a software interrupt for hart0 is injected
    /// in the clint at reset.
    template<typename URV>
    bool configAclint(System<URV>&, Hart<URV>&, uint64_t aclintStart, uint64_t aclintSize,
                      uint64_t mswiOffset, bool hasMswi,
                      uint64_t mtimeCmpOffset, uint64_t mtimeOffset, bool hasMtimer,
		      bool siOnReset = false, bool deliverInterrupts = true) const;

    template<typename URV>
    bool applyImsicConfig(System<URV>&) const;

    template <typename URV>
    bool applyAplicConfig(System<URV>& system) const;

    template<typename URV>
    bool applyPciConfig(System<URV>&) const;

    /// Set xeln to the register width configuration held in this
    /// object returning true on success and false if this object does
    /// not contain a register width (xlen) configuration.
    bool getXlen(unsigned& xlen) const;

    /// Set count to the core-count configuration held in this object
    /// returning true on success and false if this object does not
    /// contain any such config.
    bool getCoreCount(unsigned& count) const;

    /// Set count to the hats-per-cor configuration held in this
    /// object returning true on success and false if this object does
    /// not contain any such config.
    bool getHartsPerCore(unsigned& xlen) const;

    /// Set pageSize to the page size configuration held in this
    /// object returning true on success and false if this object does
    /// not contain a page size configuration.
    bool getPageSize(size_t& pageSize) const;

    /// Set memSize to the memory size configuration held in this
    /// object returning true on success and false if this object does
    /// not contain a memory size configuration.
    bool getMemorySize(size_t& memSize) const;

    /// Set lineSize to the merge buffer line size returning true on
    /// success and false on failure (config is missing).
    bool getMcmLineSize(unsigned& lineSize) const;

    /// Set checkAll to the merge buffer check all flag returning true
    /// on success and false on failure (config is missing).
    bool getMcmCheckAll(bool& checkAll) const;

    /// Set enableCache to the mcm cache enable flag returning true
    /// on success and false on failure (config is missing).
    bool getMcmEnableCache(bool& enableCache) const;

    /// Set hartIdOffset to the hart-id offset configuration field
    /// returning true on success and false on failure. Cores will be
    /// associated with start ids 0, hartIdOffset, 2*hartIdOffset, ...
    /// Harts within a core will get consecutive MHARTID values
    /// starting with the start id associated with that core.
    bool getHartIdOffset(unsigned& offset) const;

    /// Set isa to the value of the isa tag (instruction set archietcture)
    /// in the JSON config file. Example: "isa" : "rv32im"
    bool getIsa(std::string& isa) const;

    /// Recover value(s) of "enable_ppo" tag. If missing or set to true then all rules are
    /// enabled. Return true on success and false on failure.
    bool getEnabledPpos(std::vector<unsigned>& enabledPpos) const;

    /// Return true if the reset value of the MISA CSR has the user
    /// extension enabled. Return false if MISA CSR is not present in
    /// this configuration or if user extension is not enabled.
    bool userModeEnabled() const;

    /// Return true if the reset value of the MISA CSR has the
    /// supervisor extension enabled. Return false if MISA CSR is not
    /// present in this configuration or if user extension is not
    /// enabled.
    bool supervisorModeEnabled() const;

    /// Clear (make empty) the set of configurations held in this object.
    void clear();

    /// Configure actions of non-standard CSRs. Configure shared CSRs
    /// in multi-hart configurations.
    template<typename URV>
    bool finalizeCsrConfig(System<URV>& system) const;

    /// Return true if this object has a configuration for the
    /// given CSR.
    bool hasCsrConfig(std::string_view csrName) const;

  protected:

    /// Helper to configHarts.
    template<typename URV>
    bool applyClintConfig(System<URV>& system, Hart<URV>& hart) const;

    template<typename URV>
    bool applyAclintConfig(System<URV>& system, Hart<URV>& hart) const;

    /// Set val to the reset value of the MISA CSR returning true on
    /// success and false if no such entry in this config or if entry
    /// has the wrong value type. This is a helper to userModeEnabled
    /// and supervisorModeEnabeld.
    bool getMisaReset(uint64_t& val) const;

  private:

    std::unique_ptr<nlohmann::json> config_;
  };

}
