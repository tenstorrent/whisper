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
#include <boost/algorithm/string.hpp>
#include "Args.hpp"


using namespace WdRiscv;


static
void
printVersion()
{
  unsigned version = 1;
  unsigned subversion = 854;
  std::cout << "Version " << version << "." << subversion << " compiled on "
	    << __DATE__ << " at " << __TIME__ << '\n';
#ifdef GIT_SHA
  #define xstr(x) str(x)
  #define str(x) #x
  std::cout << "Git SHA: " << xstr(GIT_SHA) << '\n';
  #undef str
  #undef xstr
#endif
  std::cout << "Compile options: \n";
#if SOFT_FLOAT
  std::cout << "SOFT_FLOAT\n";
#endif
#ifdef MEM_CALLBACKS
  std::cout << "MEM_CALLBACKS\n";
#endif
#if PCI
  std::cout << "PCI\n";
#endif
#if FAST_SLOPPY
  std::cout << "FAST_SLOPPY\n";
#endif
#if LZ4_COMPRESS
  std::cout << "LZ4_COMPRESS\n";
#endif
}


void
Args::expandTargets()
{
  this->expandedTargets.clear();
  for (const auto& target : this->targets)
    {
      StringVec tokens;
      boost::split(tokens, target, boost::is_any_of(this->targetSep),
		   boost::token_compress_on);
      this->expandedTargets.push_back(tokens);
    }
}


bool
Args::collectCommandLineValues(const boost::program_options::variables_map& varMap)
{
  bool ok = true;

  if (varMap.count("startpc"))
    {
      auto numStr = varMap["startpc"].as<std::string>();
      if (not parseCmdLineNumber("startpc", numStr, this->startPc))
	ok = false;
    }

  if (varMap.count("endpc"))
    {
      auto numStr = varMap["endpc"].as<std::string>();
      if (not parseCmdLineNumber("endpc", numStr, this->endPc))
	ok = false;
    }

  if (varMap.count("tohost"))
    {
      auto numStr = varMap["tohost"].as<std::string>();
      if (not parseCmdLineNumber("tohost", numStr, this->toHost))
	ok = false;
    }

  if (varMap.count("fromhost"))
    {
      auto numStr = varMap["fromhost"].as<std::string>();
      if (not parseCmdLineNumber("fromhost", numStr, this->fromHost))
	ok = false;
    }

  if (varMap.count("consoleio"))
    {
      auto numStr = varMap["consoleio"].as<std::string>();
      if (not parseCmdLineNumber("consoleio", numStr, this->consoleIo))
	ok = false;
    }

  if (varMap.count("maxinst"))
    {
      auto numStr = varMap["maxinst"].as<std::string>();
      if (not parseCmdLineNumber("maxinst", numStr, this->instCountLim))
	ok = false;
      this->relativeInstCount = not numStr.empty() and numStr.at(0) == '+';
    }

  if (varMap.count("maxretinst"))
    {
      auto numStr = varMap["maxretinst"].as<std::string>();
      if (not parseCmdLineNumber("maxretinst", numStr, this->retInstCountLim))
	ok = false;
      // TODO: use a separate flag here
      this->relativeInstCount = not numStr.empty() and numStr.at(0) == '+';
    }

  if (varMap.count("memorysize"))
    {
      auto numStr = varMap["memorysize"].as<std::string>();
      if (not parseCmdLineNumber("memorysize", numStr, this->memorySize))
        ok = false;
    }

  if (varMap.count("tlbsize"))
    {
      auto numStr = varMap["tlbsize"].as<std::string>();
      if (not parseCmdLineNumber("tlbsize", numStr, this->tlbSize))
        ok = false;
    }

  if (varMap.count("nmivec"))
    {
      auto numStr = varMap["nmivec"].as<std::string>();
      if (not parseCmdLineNumber("nmivec", numStr, this->nmiVec))
        ok = false;
    }

  if (varMap.count("nmevec"))
    {
      auto numStr = varMap["nmevec"].as<std::string>();
      if (not parseCmdLineNumber("nmevec", numStr, this->nmeVec))
        ok = false;
    }

  if (varMap.count("tohostsym"))
    this->toHostSym = varMap["tohostsym"].as<std::string>();

  if (varMap.count("consoleiosym"))
    this->consoleIoSym = varMap["consoleiosym"].as<std::string>();

  if (varMap.count("alarm"))
    {
      auto numStr = varMap["alarm"].as<std::string>();
      if (not parseCmdLineNumber("alarm", numStr, this->alarmInterval))
        ok = false;
      else if (this->alarmInterval.has_value() and *this->alarmInterval == 0)
        std::cerr << "Warning: Zero alarm period ignored.\n";
    }

  if (varMap.count("branchwindow"))
    {
      auto numStr = varMap["branchwindow"].as<std::string>();
      if (not parseCmdLineNumber("branchwindow", numStr, this->branchWindow))
        ok = false;
    }

  if (varMap.count("clint"))
    {
      auto numStr = varMap["clint"].as<std::string>();
      if (not parseCmdLineNumber("clint", numStr, this->clint))
        ok = false;
      else if (this->clint.has_value() and (*this->clint & 7) != 0)
        {
          std::cerr << "Error: clint address must be a multiple of 8\n";
          ok = false;
        }
    }

  if (varMap.count("mcmls"))
    {
      auto numStr = varMap["mcmls"].as<std::string>();
      if (not parseCmdLineNumber("mcmls", numStr, this->mcmls))
        ok = false;
    }

  if (varMap.count("harts"))
    {
      auto numStr = varMap["harts"].as<std::string>();
      if (not parseCmdLineNumber("harts", numStr, this->harts))
        ok = false;
    }

  if (varMap.count("cores"))
    {
      auto numStr = varMap["cores"].as<std::string>();
      if (not parseCmdLineNumber("cores", numStr, this->cores))
        ok = false;
    }

  if (varMap.count("xlen"))
    {
      auto numStr = varMap["xlen"].as<std::string>();
      if (not parseCmdLineNumber("xlen", numStr, this->xlen))
        ok = false;
      std::cerr << "Warning: Command line option --xlen is deprecated.\n";
    }

  if (varMap.count("noppo"))
    this->noPpo = true;

  if (varMap.count("triggers"))
    this->triggers = true;

  if (varMap.count("notriggers"))
    this->triggers = false;

  if (varMap.count("triggers") and varMap.count("notriggers"))
    {
      std::cerr << "Error: Cannot specify both --triggers and --notriggers.\n";
      ok = false;
    }

  if (varMap.count("steesr"))
    {
      auto rangeStr = varMap["steesr"].as<std::string>();
      StringVec tokens;
      boost::split(tokens, rangeStr, boost::is_any_of(":"));
      if (tokens.size() != 2)
	{
	  std::cerr << "Error: Bad value for --steesr: \"" << rangeStr << "\". "
		    << "Expecting a colon separated pair of numbers.\n";
	  ok = false;
	}
      else
	{
	  uint64_t s0 = 0, s1 = 0;
	  if (not parseCmdLineNumber("steesr", tokens.at(0), s0) or
	      not parseCmdLineNumber("steesr", tokens.at(1), s1))
	    ok = false;
	  else
	    {
	      this->steesr.clear();
	      this->steesr.push_back(s0);
	      this->steesr.push_back(s1);
	    }
	}
    }

  if (varMap.count("instcounter"))
    {
      auto numStr = varMap["instcounter"].as<std::string>();
      if (not parseCmdLineNumber("instcounter", numStr, this->instCounter))
        ok = false;
    }

  if (varMap.count("logstart"))
    {
      auto numStr = varMap["logstart"].as<std::string>();
      if (not parseCmdLineNumber("logstart", numStr, this->logStart))
        ok = false;
    }

  if (varMap.count("deterministic"))
    {
      auto rangeStr = varMap["deterministic"].as<std::string>();
      StringVec tokens;
      boost::split(tokens, rangeStr, boost::is_any_of(":"));
      if (tokens.size() == 1)
	{
	  uint64_t s1 = 0;
	  if (not parseCmdLineNumber("deterministic", tokens.at(0), s1))
	    ok = false;
	  else
	    {
	      this->deterministic.clear();
	      this->deterministic.push_back(1);
	      this->deterministic.push_back(s1);
	    }
	}
      else if (tokens.size() == 2)
        {
	  uint64_t s0 = 0, s1 = 0;
	  if (not parseCmdLineNumber("deterministic", tokens.at(0), s0) or
	      not parseCmdLineNumber("deterministic", tokens.at(1), s1))
	    ok = false;
	  else
	    {
	      this->deterministic.clear();
	      this->deterministic.push_back(s0);
	      this->deterministic.push_back(s1);
	    }
        }
      else
	{
	  std::cerr << "Error: Bad value for --determinstic: \"" << rangeStr << "\". "
		    << "Expecting a number or a colon separated pair of numbers.\n";
	  ok = false;
	}
    }

  if (varMap.count("seed"))
    {
      auto numStr = varMap["seed"].as<std::string>();
      ok = parseCmdLineNumber("seed", numStr, this->seed) and ok;
    }

  if (this->interactive)
    this->trace = true;  // Enable instruction tracing in interactive mode.

  if (varMap.count("dumpmem"))
    ok = parseDumpMem(varMap["dumpmem"].as<std::string>()) and ok;

  return ok;
}


bool
Args::parseCmdLineArgs(std::span<char*> argv)
{
  try
    {
      // Define command line options.
      namespace po = boost::program_options;
      po::options_description desc("options");
      desc.add_options()
	("help,h", po::bool_switch(&this->help),
	 "Produce this message.")
	("numa", po::bool_switch(&this->use_numactl),
	 "Use numactl.")
	("log,l", po::bool_switch(&this->trace),
	 "Enable tracing to standard output of executed instructions.")
	("isa", po::value(&this->isa),
	 "Specify instruction set extensions to enable. Supported extensions "
	 "are a, c, d, f, i, m, s and u. Default is imc.")
	("xlen", po::value<std::string>(),
	 "Specify register width (32 or 64), defaults to 32")
	("harts", po::value<std::string>(),
	 "Specify number of hardware threads per core (default=1).")
	("cores", po::value<std::string>(),
	 "Specify number of core per system (default=1).")
	("pagesize", po::value(&this->pageSize),
	 "Specify memory page size.")
	("target,t", po::value(&this->targets)->multitoken(),
	 "Target program (ELF file) to load into simulator memory. In "
	 "newlib/Linux emulation mode, program options may follow program name.")
	("targetsep", po::value(&this->targetSep),
	 "Target program argument separator.")
	("hex,x", po::value(&this->hexFiles)->multitoken(),
	 "HEX file to load into simulator memory.")
	("binary,b", po::value(&this->binaryFiles)->multitoken(),
	 "Binary file to load into simulator memory. File path may be suffixed with a colon followed "
	 "by an address (integer) in which case data will be loaded at address as opposed to zero. "
	 "An additional suffix of :u may be added to write back the file with the contents of memory "
	 "at the end of the run. "
	 "Example: -b file1 -b file2:0x1040 -b file3:0x20000:u")
#if LZ4_COMPRESS
	("lz4", po::value(&this->lz4Files)->multitoken(),
	 "LZ4 file to load into simulator memory.")
#endif
        ("kernel", po::value(&this->kernelFile),
         "Kernel binary file to load into simulator memory. File will be loaded at 0x400000 for "
        "rv32 or 0x200000 for rv64 unless an explicit address is specified after a colon suffix "
        "to the file path.")
        ("testsignature", po::value(&this->testSignatureFile),
         "Produce a signature file used to score tests provided by the riscv-arch-test project.")
	("logfile,f", po::value(&this->traceFile),
	 "Enable tracing to given file of executed instructions. Output is compressed (with "
         "/usr/bin/gzip) if file name ends with \".gz\".")
	("csvlog", po::bool_switch(&this->csv),
	 "Enable CSV format for log file.")
	("consoleoutfile", po::value(&this->consoleOutFile),
	 "Redirect console output to given file.")
	("commandlog", po::value(&this->commandLogFile),
	 "Enable logging of interactive/socket commands to the given file.")
	("server", po::value(&this->serverFile),
	 "Interactive server mode. Put server hostname and port in file. If shared memory "
         "is enabled, file is memory mapped filename")
        ("shm", po::bool_switch(&this->shm),
         "Enable shared memory IPC for server mode (default mode uses socket).")
	("startpc,s", po::value<std::string>(),
	 "Set program entry point. If not specified, use entry point of the "
	 "most recently loaded ELF file.")
	("endpc,e", po::value<std::string>(),
	 "Set stop program counter. Simulator will stop once instruction at "
	 "the stop program counter is executed.")
	("tohost", po::value<std::string>(),
	 "Memory address for host target interface (HTIF).")
	("tohostsym", po::value<std::string>(),
	 "ELF symbol to use for setting tohost from ELF file (in the case "
	 "where tohost is not specified on the command line). Default: "
	 "\"tohost\".")
	("fromhost", po::value<std::string>(),
	 "Memory address for host target interface (HTIF).")
	("consoleio", po::value<std::string>(),
	 "Memory address corresponding to console io. Reading/writing "
	 "(lw/lh/lb sw/sh/sb) from given address reads/writes a byte from the "
         "console.")
	("consoleiosym", po::value<std::string>(),
	 "ELF symbol to use as console-io address (in the case where "
         "consoleio is not specified on the command line). Default: "
         "\"__whisper_console_io\".")
	("maxinst,m", po::value<std::string>(),
	 "Limit executed instruction count to arg. With a leading plus sign interpret the count as relative to the loaded (from a snapshot) instruction count.")
        ("maxretinst,r", po::value<std::string>(),
         "Limit retired instruction count to arg. With a leading plus sign interpret the count as relative to the loaded (from a snapshot) retired instruction count.")
	("memorysize", po::value<std::string>(),
	 "Memory size (must be a multiple of 4096).")
	("tlbsize", po::value<std::string>(),
	 "Memory size (must be a power of 2).")
	("nmivec", po::value<std::string>(),
	 "PC value after a non-maskable interrupt.")
	("nmevec", po::value<std::string>(),
	 "PC value after an exception in the non-maskable interrupt handler.")
	("interactive,i", po::bool_switch(&this->interactive),
	 "Enable interactive mode.")
	("traceload", po::bool_switch(&this->traceLdSt),
	 "Enable tracing of load/store instruction data address (deprecated -- now always on).")
	("traceptw", po::bool_switch(&this->tracePtw),
	 "Enable printing of page table walk information in log.")
	("semihosting", po::bool_switch(&this->semiHosting),
	 "enable semihosting capabilities on Whisper")
	("triggers",
	 "Enable debug triggers (triggers are on in interactive and server modes)")
	("notriggers",
	 "Disable debug triggers (triggers are on in interactive and server modes)")
	("counters", po::bool_switch(&this->counters),
	 "Enable performance counters")
	("gdb", po::bool_switch(&this->gdb),
	 "Run in gdb mode enabling remote debugging from gdb (this requires gdb version"
         "8.2 or higher).")
	("gdb-tcp-port", po::value(&this->gdbTcpPort)->multitoken(),
	 	 "TCP port number for gdb; If port num is negative,"
			" gdb will work with stdio (default -1).")
	("profileinst", po::value(&this->instFreqFile),
	 "Report instruction frequency to file.")
        ("tracebranch", po::value(&this->branchTraceFile),
         "Trace branch instructions to the given file.")
        ("branchwindow", po::value<std::string>(),
         "Trace branches in the last n instructions.")
        ("tracerlib", po::value(&this->tracerLib),
         "Path to tracer extension shared library which should provide C symbol tracerExtension32 or tracerExtension64."
         "Optionally include arguments after a colon to be exposed to the shared library "
         "as C symbol tracerExtensionArgs (ex. tracer.so or tracer.so:hello42).")
	("logstart", po::value<std::string>(),
	 "Start logging at given instruction rank.")
	("logperhart", po::bool_switch(&this->logPerHart),
	 "Use a separate log per hart. This allows a faster trace by reducing lock contention on the logfile.")
	("setreg", po::value(&this->regInits)->multitoken(),
	 "Initialize registers. Apply to all harts unless specific prefix "
	 "present (hart is 1 in 1:x3=0xabc). Example: --setreg x1=4 x2=0xff "
	 "1:x3=0xabc")
	("configfile", po::value(&this->configFile),
	 "Configuration file (JSON file defining system features).")
	("bblockfile", po::value(&this->bblockFile),
	 "Basic blocks output stats file.")
	("bblockinterval", po::value(&this->bblockInsts),
	 "Basic block stats are reported even multiples of given instruction counts and once at end of run.")
	("snapshotdir", po::value(&this->snapshotDir),
	 "Directory prefix for saving snapshots.")
	("snapshotperiod", po::value(&this->snapshotPeriods)->multitoken(),
	 "Snapshot period: Save snapshot using snapshotdir every so many instructions. "
         "Specifying multiple periods will only save a snapshot on first instance (not periodic).")
	("loadfrom", po::value(&this->loadFrom),
	 "Snapshot directory from which to restore a previously saved (snapshot) state.")
        ("loadfromtrace", po::bool_switch(&this->loadFromTrace),
         "If true, also restore data-lines/instr-lines/branch-trace from a snapshot "
         "directory. This needs to be used in conjunction with --loadfrom.")
        ("snapcompressiontype", po::value(&this->compressionType),
         "Compression type for snapshots. Supported types are: lz4, gzip [default].")
        ("snapdecompressiontype", po::value(&this->decompressionType),
         "Decompression type for snapshots. Supported types are: lz4, gzip [default].")
	("stdout", po::value(&this->stdoutFile),
	 "Redirect standard output of newlib/Linux target program to this.")
	("stderr", po::value(&this->stderrFile),
	 "Redirect standard error of newlib/Linux target program to this.")
	("stdin", po::value(&this->stdinFile),
	 "Redirect standard input of newlib/Linux target program to this.")
	("datalines", po::value(&this->dataLines),
	 "Generate data line address trace to the given file with format <vl>:<pl> where "
         "<vl>/<pl> stands for virtual/physical line number. A line number is an address "
         "divided by the cache line size.")
	("instrlines", po::value(&this->instrLines),
	 "Generate instruction line address trace to the given file. See --datalines for file "
         "format.")
	("initstate", po::value(&this->initStateFile),
	 "Generate to given file the initial state of accessed memory lines.")
	("dumpmem", po::value<std::string>(),
	 "At end of run, write the contents of a list of memory address ranges to a file "
         "in hex format. The argument is a string of the form 'file_name:b1:e1:b2:e2...', "
         "where b1 is the beginning address of the first range and e1 is its end address. "
         "Example: '--dumpmem xyz:0:100:0x200:0x300'. This will dump to the file xyz the "
         "contents of the memory ranges [0,100] and [0x200, 0x300]. The count of the "
         "colon separated addresses after the file name must be even and must not be zero.")
	("abinames", po::bool_switch(&this->abiNames),
	 "Use ABI register names (e.g. sp instead of x2) in instruction dis-assembly.")
	("newlib", po::bool_switch(&this->newlib),
	 "Emulate (some) newlib system calls. Done automatically if newlib "
         "symbols are detected in the target ELF file.")
	("linux", po::bool_switch(&this->linux),
	 "Emulate (some) Linux system calls. Done automatically if Linux "
         "symbols are detected in the target ELF file.")
	("raw", po::bool_switch(&this->raw),
	 "Bare metal mode: Disable emulation of Linux/newlib system call emulation "
         "even if Linux/newlib symbols detected in the target ELF file.")
        ("envvar", po::value(&this->envVars)->multitoken(),
         "Pass environment variable to newlib/Linux target program "
         "(e.g. ENV_VAR_NAME=4)")
	("elfisa", po::bool_switch(&this->elfisa),
	 "Configure reset value of MISA according to the RISCV architecture tag(s) "
         "encoded into the loaded ELF file(s) if any.")
	("unmappedelfok", po::bool_switch(&this->unmappedElfOk),
	 "Do not flag as error ELF file sections targeting unmapped memory.")
	("alarm", po::value<std::string>(),
	 "External interrupt period in micro-seconds: Convert arg to an "
         "instruction count, n, assuming a 1ghz clock, and force an external "
         " interrupt every n instructions. No-op if arg is zero.")
        ("clint", po::value<std::string>(),
         "Define address, a, of memory mapped area for clint (core local "
         "interruptor). In an n-hart system, words at addresses a, a+4, ... "
         "a+(n-1)*4, are  associated with the n harts. Store a 0/1 to one of "
         "these locations clears/sets the software interrupt bit in the MIP CSR "
         "of the corresponding hart. Similarly, addresses b, b+8, ... b+(n-1)*8, "
         "where b is a+0x4000, are associated with the n harts. Writing to one "
         "of these double words sets the timer-limit of the corresponding hart. "
         "A timer interrupt in such a hart becomes pending when the timer value "
         "equals or exceeds the timer limit.")
	("mcm", po::bool_switch(&this->mcm),
	 "Enable memory consistency checks. This is meaningful in server/interactive mode.")
	("noppo",
	 "Skip preserve program order rule check in MCM when this is used.")
	("mcmca", po::bool_switch(&this->mcmca),
	 "Check all bytes of the memory consistency check merge buffer. If not used "
	 "we only check the bytes inserted into the merge buffer.")
	("mcmls", po::value<std::string>(),
	 "Memory consistency checker merge buffer line size. If set to zero then "
	 "write operations are not buffered and will happen as soon a received.")
        ("dismcmcache", po::bool_switch(&this->dismc),
         "Disables memory consistency checker cache model.")
	("steesr", po::value<std::string>(),
	 "Static trusted execution environment secure range: A colon separated pair of numbers"
	 " defining the range of memory addresses considered secure. Secure access bit must"
	 " be zero in each address of the pair.")
	("perfapi", po::bool_switch(&this->perfApi),
	 "Enable performance mode API.")
#ifdef MEM_CALLBACKS
        ("reportusedblocks", po::bool_switch(&this->reportub),
         "Report used blocks with sparse memory. Useful for finding memory footprint of program")
#endif
        ("pcidev", po::value(&this->pciDevs)->multitoken(),
         "Add PCI device to simulation. Format is <device>:<bus>:<slot>:<device-specific>. "
         "This should be combined with the pci option to declare a memory region for these devices. "
         "Currently only supports virtio-blk, which requires a file")
        ("deterministic", po::value<std::string>(),
         "Used for deterministic multi-hart runs. Define a window range [x:y] for the amount of instructions "
         "a hart will execute before switching to the next hart. A range of 0:0 turns this off. The "
         "actual amount of instructions is determined by corresponding seed value.")
        ("seed", po::value<std::string>(),
         "Corresponding seed for deterministic runs. If this is not specified, but 'deterministic' is, whisper will "
         "generate a seed value based on current time.")
	("instcounter", po::value<std::string>(),
	 "Set instruction counter to given value.")
        ("quitany", po::bool_switch(&this->quitOnAnyHart),
         "Terminate multi-threaded run when any hart finishes (default is to wait "
         "for all harts.)")
        ("noconinput", po::bool_switch(&this->noConInput),
         "Do not use console IO address for input. Loads from the console io address "
         "simply return last value stored there.")
        ("instlist", po::bool_switch(&this->instList),
         "List the instructions of the extensions specified by --isa or the \"isa\" configuration "
         "tag")
        ("hintops", po::bool_switch(&this->hintOps),
         "Enable whisper HINT ops.")
	("verbose,v", po::bool_switch(&this->verbose),
	 "Be verbose.")
	("version", po::bool_switch(&this->version),
	 "Print version.")
        ("loglabel,ll", po::bool_switch(&this->logLabel)->default_value(false),
         "When enabled, prepend ELF symbol label (if any) to text log output");

      // Define positional options.
      po::positional_options_description pdesc;
      pdesc.add("target", -1);

      // Parse command line options.
      po::variables_map varMap;
      po::command_line_parser parser(static_cast<int>(argv.size()), argv.data());
      auto parsed = parser.options(desc).positional(pdesc).run();
      po::store(parsed, varMap);
      po::notify(varMap);

      bool earlyExit = false;
      if (this->version)
	{
	  printVersion();
	  earlyExit = true;
	}

      if (this->help)
	{
	  std::cout <<
	    "Simulate a RISCV system running the program specified by the given ELF\n"
	    "and/or HEX file. With --newlib/--linux, the ELF file is a newlib/linux linked\n"
	    "program and may be followed by corresponding command line arguments.\n"
	    "All numeric arguments are interpreted as hexadecimal numbers when prefixed"
	    " with 0x."
	    "Examples:\n"
	    "  whisper --target prog --log\n"
	    "  whisper --target prog --setreg sp=0xffffff00\n"
	    "  whisper --newlib --log --target \"prog -x -y\"\n"
	    "  whisper --linux --log --targetsep ':' --target \"prog:-x:-y\"\n\n";
	  std::cout << desc;
          earlyExit = true;
	}

      if (earlyExit)
        return true;

      if (not this->collectCommandLineValues(varMap))
	return false;

      // Expand each target program string into program name and args.
      this->expandTargets();
    }

  catch (std::exception& exp)
    {
      std::cerr << "Error: Failed to parse command line args: " << exp.what() << '\n';
      return false;
    }

  return true;
}


template <typename TYPE>
bool
Args::parseCmdLineNumber(const std::string& option, const std::string& numberStr,
			 TYPE& number)
{
  std::string str = numberStr;
  bool good = not str.empty();
  uint64_t scale = 1;
  if (good)
    {
      char suffix = std::tolower(str.back());
      if (suffix == 'k')
        scale = 1024;
      else if (suffix == 'm')
        scale = UINT64_C(1024)*1024;
      else if (suffix == 'g')
        scale = UINT64_C(1024)*1024*1024;
      else if (suffix == 't')
        scale = UINT64_C(1024)*1024*1024*1024;
      if (scale != 1)
        {
          str = str.substr(0, str.length() - 1);
          if (str.empty())
            good = false;
        }
    }

  if (good)
    {
      using STYPE = typename std::make_signed_t<TYPE>;

      char* end = nullptr;

      bool bad = false;

      if (std::is_same<TYPE, STYPE>::value)
        {
          int64_t val = strtoll(str.c_str(), &end, 0) * static_cast<int64_t>(scale);
          number = static_cast<TYPE>(val);
          bad = val != number;
        }
      else
        {
          uint64_t val = strtoull(str.c_str(), &end, 0) * scale;
          number = static_cast<TYPE>(val);
          bad = val != number;
        }

      if (bad)
	{
	  std::cerr << "Error: parseCmdLineNumber: Number too large: " << numberStr
		    << '\n';
	  return false;
	}
      if (end and *end)
	good = false;  // Part of the string are non parseable.
    }

  if (not good)
    std::cerr << "Error: Invalid command line " << option << " value: " << numberStr
	      << '\n';
  return good;
}


template <typename TYPE>
bool
Args::parseCmdLineNumber(const std::string& option, const std::string& numberStr,
			 std::optional<TYPE>& number)
{
  TYPE n;
  if (not parseCmdLineNumber(option, numberStr, n))
    return false;
  number = n;
  return true;
}


bool
Args::parseDumpMem(const std::string& arg)
{
  eorMemDump.clear();
  eorMemDumpRanges.clear();

  if (arg.empty())
    {
      std::cerr << "Error: Arg of --dumpmem cannot be an empty string.\n";
      return false;
    }

  if (arg.starts_with(":") or arg.ends_with(":"))
    {
      std::cerr << "Error: Arg of --dumpmem cannot start or end with a colon.\n";
      return false;
    }

  StringVec tokens;
  boost::split(tokens, arg, boost::is_any_of(":"));

  this->eorMemDump = tokens[0];
  if ((tokens.size() %2) != 1 or tokens.size() == 1)
    {
      std::cerr << "Error: For --dumpmem, count of addresses after file name must be "
                << "even and non-zero.\n";
      return false;
    }

  for (size_t i = 1; i < tokens.size(); i += 2)
    {
      uint64_t start = 0, end = 0;

      if (not parseCmdLineNumber("dumpmem-range-start", tokens.at(i), start))
        return false;

      if (not parseCmdLineNumber("dumpmem-range-start", tokens.at(i+1), end))
        return false;

      if (start > end)
        {
          std::cerr << "Error: Invalid address range for --memdump (start > end): "
                    << start << ':' << end << '\n';
          return false;
        }

      this->eorMemDumpRanges.push_back(start);
      this->eorMemDumpRanges.push_back(end);
    }

  return true;
}
