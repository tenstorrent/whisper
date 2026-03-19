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

// Copyright 2024 Tenstorrent Corporation or its affiliates.
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

#include <memory>               // For shared_ptr
#include "System.hpp"
#include "Args.hpp"
#include "util.hpp"


namespace WdRiscv
{

  /// Manage a whisper session.
  template <typename URV>
  class Session
  {
  public:

    /// Constructor.
    Session() = default;

    /// Define a system. Basic system parameters (like number of harts) are taken form
    /// configuration object (likely populated from a JSON config file) and the command
    /// line arguments. Command line arguments take precedence (they are applied last).
    /// Return pointer to the defined system on success and a nullptr on failure.
    std::shared_ptr<System<URV>>
    defineSystem(const Args& args, const HartConfig& config);

    /// Configure the system.
    bool configureSystem(const Args& args, const HartConfig& config);

    /// Run the target program on the system.
    bool run(const Args& args);

    /// End of run cleanup.
    bool cleanup(const Args& args);

    /// Obtain integer-register width (xlen). Command line has top priority, then config
    /// file, then ELF file.
    static
    unsigned determineRegisterWidth(const Args& args, const HartConfig& config);

  protected:

    bool getPrimaryConfigParameters(const Args& args, const HartConfig& config,
				    unsigned& hartsPerCore, unsigned& coreCount,
				    size_t& pageSize, size_t& memorySize);

    /// Sanitize memory parameters. Page/region sizes must be greater and equal to 4 and
    /// must be powers of 2. Region size must be a multiple of page size.  Memory size
    /// must be a multiple of region size. Return true if given parameters are
    /// good. False if any parameters is changed to meet expectation.
    bool checkAndRepairMemoryParams(size_t& memSize, size_t& pageSize);

    /// Open the trace-file, command-log and console-output files specified on the command
    /// line. Return true if successful or false if any specified file fails to open.
    bool openUserFiles(const Args& args);

    /// Based on the command line arguments and symbols in the elf file, decide which
    /// emulation to enable: linux, newlib, semihosting, or none.
    ///
    /// If --raw is used, then no emulation: linux/newlib/semihost are all set to false.
    ///
    /// If one or more of --linux/--newlib/--semihosting is used, set to true one of the
    /// variables linux/newlib/semihost in the decreasing priority: linux, newlib,
    /// semihost.
    ///
    /// Otherwise, determine linux or newlib emulation from the target ELF files.
    void checkForNewlibOrLinux(const Args& args, bool& linux, bool& newlib,
                               bool& semihost) const;

    /// Check if running an app that uses openMp.
    bool checkForOpenMp(const Args& args);

    /// Dump memory ranges to given file.
    bool eorMemDump(const std::string& file, const std::vector<uint64_t>& addrs);

    bool determineIsa(const HartConfig& config, const Args& args, bool clib,
		      std::string& isa);

    bool getElfFilesIsaString(const Args& args, std::string& isaString);

    bool applyCmdLineArgs(const Args& args, Hart<URV>& hart, const HartConfig& config,
			  bool clib);

    /// Open a server socket and put opened socket information (hostname and port number)
    /// in the given server file. Wait for one connection. Service connection. Return true
    /// on success and false on failure.
    bool runServer(const std::string& serverFile);

    /// Open a shared memory region and write name to given server file. Return true on
    /// success and false on failure.
    bool runServerShm(const std::string& serverFile);

    /// Run an interactive session with interactive command output going to the given
    /// ostream.
    bool runInteractive(std::ostream& out);

  private:

    std::vector<util::file::SharedFile> traceFiles_;
    util::file::SharedFile commandLog_;
    util::file::SharedFile consoleOut_;
    util::file::SharedFile bblockFile_;
    util::file::SharedFile initStateFile_;

    bool doGzip_ = false;

    std::shared_ptr<System<URV>> system_;
  };

}
