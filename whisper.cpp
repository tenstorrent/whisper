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

#include <iostream>
#include <span>
#include "numa.hpp"
#include "HartConfig.hpp"
#include "Args.hpp"
#include "Session.hpp"


using namespace WdRiscv;


#include <termios.h>


int
main(int argc, char* argv[])
{
  // Used to restore terminal state via RAII
  class TerminalStateRAII
  {
  public:
    TerminalStateRAII()
    { tcgetattr(STDIN_FILENO, &term); }

    ~TerminalStateRAII()
    { tcsetattr(STDIN_FILENO, 0, &term); }  // Restore terminal state.

  private:
    struct termios term{};
  };

  bool ok = true;
  try
    {
      Args args;
      if (not args.parseCmdLineArgs(std::span(argv, argc)))
        return 1;
      if (args.help or args.version)
        return 0;

      // Load configuration files.
      HartConfig config;
      std::vector<std::string> configFiles;
      boost::split(configFiles, args.configFile, boost::is_any_of(","));
      for (const auto& configFile : configFiles)
        {
          if (configFile.empty())
            continue;
          if (not config.loadConfigFile(configFile))
            return 1;
        }

      // Try to use numactl to improve performance
      // If this succeeds, we'll call exec() and never return
      // If this function returns, it's because numactl could not be used for
      // some reason and we must proceed as normal
      if (args.use_numactl)
        {
          unsigned hartsPerCore = 1;
          unsigned coreCount = 1;
          config.getHartsPerCore(hartsPerCore);
          if (args.harts)
            hartsPerCore = *args.harts;
          config.getCoreCount(coreCount);
          if (args.cores)
            coreCount = *args.cores;
          unsigned numa_cores = hartsPerCore * coreCount;
          attempt_numactl(argc, argv, numa_cores);
        }

      TerminalStateRAII term;  // Save/restore terminal state.

      unsigned regWidth = Session<uint32_t>::determineRegisterWidth(args, config);

      if (regWidth == 32)
	{
	  Session<uint32_t> session{};
	  ok = session.defineSystem(args, config) != nullptr;
	  ok = ok and session.configureSystem(args, config);
	  ok = ok and session.run(args);
          ok = ok and session.cleanup(args);
	}
      else if (regWidth == 64)
	{
	  Session<uint64_t> session{};
	  ok = session.defineSystem(args, config) != nullptr;
	  ok = ok and session.configureSystem(args, config);
          ok = ok and session.run(args);
          ok = ok and session.cleanup(args);
	}
      else
        {
          std::cerr << "Error: Invalid register width: " << regWidth;
          std::cerr << "Error:  -- expecting 32 or 64\n";
          ok = false;
        }
    }
  catch (std::exception& e)
    {
      std::cerr << e.what() << '\n';
      ok = false;
    }

  return ok? 0 : 1;
}
