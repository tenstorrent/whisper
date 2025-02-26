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

#include <tuple>
#include <memory>               // For shared_ptr
#include <functional>
#include <string_view>
#include <unordered_map>
#include "Memory.hpp"
#include "Imsic.hpp"
#include "Syscall.hpp"
#include "pci/Pci.hpp"
#include "pci/virtio/Blk.hpp"
#include "aplic/Aplic.hpp"


namespace TT_PERF
{
  class PerfApi;
}


namespace WdRiscv
{

  template <typename URV>
  class Hart;

  template <typename URV>
  class Core;

  template <typename URV>
  class Mcm;

  class DecodedInst;
  class IoDevice;
  class SparseMem;


  /// Model a system consisting of n cores with m-harts per core and a
  /// memory. The harts in the system are indexed from 0 to n*m -
  /// 1. The type URV (unsigned register value) is that of the integer
  /// register and is either uint32_t or uint64_t.
  template <typename URV>
  class System
  {
  public:

    using HartClass = Hart<URV>;
    using CoreClass = Core<URV>;

    /// Constructor: Construct a system with n (coreCount) cores each
    /// consisting of m (hartsPerCore) harts. The harts in this system
    /// are indexed with 0 to n*m - 1.  Each core is assigned a
    /// hart-id start from the sequence 0, hartIdOffset,
    /// 2*hartIdOffset, ...  Harts in a core are assigned consecutive
    /// hart-ids (values of MHARTID CSRs) starting with the start if
    /// od the core.
    System(unsigned coreCount, unsigned hartsPerCore, unsigned hartIdOffset,
           size_t memSize, size_t pageSize);

    ~System();

    /// Return count of cores in this system.
    unsigned coreCount() const
    { return cores_.size(); }

    /// Return the number of harts per core.
    unsigned hartsPerCore() const
    { return hartsPerCore_; }

    /// Return count of harts (coreCount * hartsPerCore) in this
    /// system.
    unsigned hartCount() const
    { return hartCount_; }

    /// Return pointer to the ith hart in the system or null if i is
    /// out of bounds. A hart index is valid if it is less than the
    /// value returned by the hartCount method.
    std::shared_ptr<HartClass> ithHart(unsigned i) const
    {
      if (i >= sysHarts_.size())
	return std::shared_ptr<HartClass>();
      return sysHarts_.at(i);
    }

    /// Return pointer to this system hart having the given value as
    /// its hart-id (value of MHARTID CSR) or null if no such hart.
    std::shared_ptr<HartClass> findHartByHartId(URV hartId) const
    {
      const auto& iter = hartIdToIndex_.find(hartId);
      if (iter != hartIdToIndex_.end())
        return ithHart(iter->second);
      return std::shared_ptr<HartClass>();
    }

    /// Return pointer to the ith core in the system or null if i is
    /// out of bounds.
    std::shared_ptr<CoreClass> ithCore(unsigned i)
    {
      if (i >= cores_.size())
	return std::shared_ptr<CoreClass>();
      return cores_.at(i);
    }

    /// Return pointer to memory.
    std::shared_ptr<Memory> memory() const
    { return memory_; }

    /// With a true flag, when loading ELF files, error out if ELF
    /// file refers to unmapped memory. With a false flag, ignore
    /// unmapped memory in the ELF file.
    void checkUnmappedElf(bool flag);

    /// Enable compressed address tracing (last occurrence of each
    /// address is reported) snapping addresses to a multiple of the
    /// line size. For example, if addreses 15, 65, 67, 129, 68
    /// are generated by the program then the line addresses will be
    /// 0, 1, 1, 2, 1 and the compresses addresse reported will be
    /// 0, 2, 1.
    void enableDataLineTrace(const std::string& path)
    {
      memory_->enableDataLineTrace(path);
      for (auto hart : sysHarts_)  hart->enableDataLineTrace(true);
    }

    /// Similar to enableDataLineTrace but for instructions.
    void enableInstructionLineTrace(const std::string& path)
    {
      memory_->enableInstructionLineTrace(path);
      for (auto hart : sysHarts_)  hart->enableInstructionLineTrace(true);
    }

    /// Define read memory callback. This (along with
    /// defineWriteMemoryCallback) allows the caller to bypass the
    /// memory model with their own.
    void defineReadMemoryCallback(
         std::function<bool(uint64_t, unsigned, uint64_t&)> callback )
    {
      memory_->defineReadMemoryCallback(std::move(callback));
    }

    /// Define write memory callback. This (along with
    /// defineReadMemoryCallback) allows the caller to bypass the
    /// memory model with their own.
    void defineWriteMemoryCallback(
         std::function<bool(uint64_t, unsigned, uint64_t)> callback )
    {
      memory_->defineWriteMemoryCallback(std::move(callback));
    }

    /// Break a hart-index-in-system into a core-index and a
    /// hart-index in core. Return true if successful and false if
    /// igven hart-index-in-system is out of bounds.
    bool unpackSystemHartIx(unsigned hartIxInSys, unsigned& coreIx,
                            unsigned& hartIxInCore)
    {
      if (hartIxInSys >= sysHarts_.size())
        return false;
      coreIx = hartIxInSys / hartsPerCore_;
      hartIxInCore = hartIxInSys % hartsPerCore_;
      return true;
    }

    /// Print the ELF symbols on the given stream. Output format:
    /// <name> <value>
    void printElfSymbols(std::ostream& out) const
    { memory_->printElfSymbols(out); }

    /// Locate the given ELF symbol (symbols are collected for every
    /// loaded ELF file) returning true if symbol is found and false
    /// otherwise. Set value to the corresponding value if symbol is
    /// found.
    bool findElfSymbol(const std::string& symbol, ElfSymbol& value) const
    { return memory_->findElfSymbol(symbol, value); }

    /// Load the given ELF files into memory. Return true on succes
    /// and false on failure. If not raw, and a hart PC is non-zero
    /// then set that PC to the first non-zero ELF entry point found.
    /// Similary set the global pointer regiser to the value of the
    /// ELF __global_pointer$ symbol.
    bool loadElfFiles(const std::vector<std::string>& files, bool raw, bool verbose);

    /// Load the given hex files and set memory locations accordingly.
    /// Return true on success. Return false if file does not exists,
    /// cannot be opened or contains malformed data.
    /// File format: A line either contains @address where address
    /// is a hexadecimal memory address or one or more space separated
    /// tokens each consisting of two hexadecimal digits.
    bool loadHexFiles(const std::vector<std::string>& files, bool verbose);

    /// Load the binary files and set memory locations accordingly.
    /// Return true on success. Return false if file does not exist,
    /// or cannot be opened. A file is loaded at the given default
    /// offset unless the filename has the form <path>:<value>
    /// where value is an integer in which case the effective file
    /// name will be <path> and the load addres will be <value>.
    bool loadBinaryFiles(const std::vector<std::string>& files,
			 uint64_t defOffset, bool verbose);

    /// Load the lz4 compressed binary files and set memory locations
    /// accordingly.
    /// Return true on success. Return false if file does not exist,
    /// or cannot be opened. A file is loaded at the given default
    /// offset unless the filename has the form <path>:<value>
    /// where value is an integer in which case the effective file
    /// name will be <path> and the load addres will be <value>.
    bool loadLz4Files(const std::vector<std::string>& files,
		      uint64_t defOffset, bool verbose);

    /// Save snapshot (registers, memory etc) into given directory
    /// which should alread exist. Return true on success.
    bool saveSnapshot(const std::string& dirPath);

    /// Load register and memory state from snapshot previously saved
    /// in the given directory. If restoreTrace is true, also restore
    /// branch/inst/data address traces. Return true on success and
    /// false on failure.
    bool loadSnapshot(const std::string& snapshotDirectory, bool restoreTrace);

    /// Write contents of memory accessed by current run in verilog
    /// hex format to the file at the given path. Return true on
    /// success and false on failure. Currently this will write the
    /// contents of accessed pages.
    bool writeAccessedMemory(const std::string& path) const;

    /// Special target program symbol writing to which stops the
    /// simulated program or performs console io.
    void setTohostSymbol(const std::string& sym)
    { toHostSym_ = sym; }

    /// Special target program symbol writing to which provide
    /// host data (console input) to the simulated hart.
    void setFromHostSymbol(const std::string& sym)
    { fromHostSym_ = sym; }

    /// Special target program symbol writing/reading to/from which
    /// writes/reads to/from the console.
    void setConsoleIoSymbol(const std::string& sym)
    { consoleIoSym_ = sym; }

    /// Device a UART device at given address reserving given size (in bytes) of address
    /// space for it. Return true on success and false if type is not supported (supported
    /// types: uartsf, uart8250).
    bool defineUart(const std::string& type, uint64_t addr, uint64_t size,
		    uint32_t iid, const std::string& channel);

    /// Return the memory page size.
    size_t pageSize() const
    { return memory_->pageSize(); }

    /// Configure incoming message signaled interrupt controller.  The
    /// addresses of the machine files of hart0, hart1, ... will be
    /// mbase, mbase + mstride, mbase + 2*mstride ...  Similartly the
    /// address of the supervisor files will be sbase, sbase + sstride,
    /// sbase + 2*sstride.  If mstride is zero then no machine file is
    /// defined. If sstride is zero then no supervisor file is defined.
    /// Guest files will take one page each and will start one page
    /// after each supervisor file (supervisor stride must be large
    /// enough). Guest files require supervisor files which require
    /// machine files. The ids parameter denotes the max interrupt
    /// id plus 1 and must be a multiple of 64.
    bool configImsic(uint64_t mbase, uint64_t mstride,
		     uint64_t sbase, uint64_t sstride,
		     unsigned guests, const std::vector<unsigned>& ids,
                     const std::vector<unsigned>& thresholdMasks,
                     bool trace);

    /// Configure the Advanced Platform-Level Interrupt Controller (APLIC).
    /// num_sources specifies the number of interrupt sources up to a maximum
    /// of 1023. For each item in the domain_params list, the APLIC model will
    /// instantiate a domain with the given parameters. The domain hierarchy,
    /// among other things, is configured by these parameters.
    bool configAplic(unsigned num_sources, std::span<const TT_APLIC::DomainParams> domain_params);

    /// Enable memory consistency model with given merge buffer size. This is relevant in
    /// server/interactive where RTL monitor or interactive command may initiate out of
    /// order memory transactions. Behavior is undefined if used in
    /// non-server/non-interactive mode or if used after execution has started. The
    /// mergeBuffserSize is the merge buffer line size in bytes. Only the PPO rules with
    /// numbers in the enabledPpos vector are enabled.
    bool enableMcm(unsigned mbSize, bool mbLineCheckAll,
		   const std::vector<unsigned>& enabledPpos);

    /// Similar to preceding method but with all PPO rules enabled/disabled.
    bool enableMcm(unsigned mbSize, bool mbLineCheckAll, bool enablePpos);

    /// Terminate MCM. Experimental. This unlikely to be useful except for executing one
    /// extra instruction at the end of a test to simplify some debugging.
    void endMcm();

    /// Enable the performance mode API.
    bool enablePerfApi(std::vector<FILE*>& traceFiles);

    /// Enable/disable total-store-order: Valid only if mcm is enabled.
    void enableTso(bool);

    /// Configure PCIe host-root-complex and construct associated devices
    /// which use transport.
    bool configPci(uint64_t configBase, uint64_t mmioBase, uint64_t mmioSize, unsigned buses, unsigned slots);

    /// Add PCIe devices specified by the user.
    bool addPciDevices(const std::vector<std::string>& devs);

    /// Return true if memory consistency model is enabled.
    bool isMcmEnabled() const
    { return mcm_ != nullptr; }

    /// Return the merge buffer line size in bytes (see enableMcm).
    unsigned mergeBufferSize() const
    { return mbSize_; }

    /// Initiate an out of order read operation for a load instruction. If a preceding
    /// overlapping store has not yet left the merge/store buffer then forward data from
    /// that store to the read operation; otherwise, get the data from global memory.
    /// Return true on success and false if global memory is not readable (in the case
    /// where we do not forward). Tag is the instruction number of the load
    /// instruction. Addr/size/data are address,size, and data of the read operation.
    /// For vector load elemIx is the element index and fieldIx is the field number
    /// for segmented loads (zero if non-segment).
    bool mcmRead(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
		 unsigned size, uint64_t data, unsigned elemIx, unsigned fieldIx);

    /// Initiate a merge buffer write.  All associated store write
    /// transactions are marked completed. Write instructions where
    /// all writes are complete are marked complete. Return true on
    /// success.  The given physical address must be a multiple of the
    /// merge buffer line size (which is also the cache line
    /// size). The rtlData vector must be of size n or larger where n
    /// is the merge buffer line size. The rtlData bytes will be
    /// placed in memory in consecutive locations starting with
    /// physAddr. If not-empty, the mask vector contains a flag that
    /// is set for each byte of rtlData that is written by the RTL.
    bool mcmMbWrite(Hart<URV>& hart, uint64_t time, uint64_t pysAddr,
		    const std::vector<uint8_t>& rtlData,
		    const std::vector<bool>& mask);

    bool mcmMbInsert(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
                     unsigned size, uint64_t data, unsigned elem, unsigned field);

    /// Initiate a write for a store instruction bypassing the merge
    /// buffer.
    bool mcmBypass(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
                   unsigned size, uint64_t data, unsigned elem, unsigned field);

    /// Initiate a fetch of a line from memory into the isntruction
    /// cache.
    bool mcmIFetch(Hart<URV>& hart, uint64_t time, uint64_t addr);

    /// Initiate an eviction of a line from the instruction cache.
    bool mcmIEvict(Hart<URV>& hart, uint64_t time, uint64_t addr);

    /// Initiate an instruction retire.
    bool mcmRetire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		   const DecodedInst& di, bool trapped);

    /// Enable to skip an address read check for specified range. This is used for the
    /// testbench to resync memory operations.
    bool mcmSkipReadDataCheck(uint64_t addr, unsigned size, bool enable);

    /// Perf model APIs.
    void perfApiCommandLog(FILE* log);

    void perfApiTraceLog(std::vector<FILE*>& files);

    bool perfApiFetch(unsigned hart, uint64_t time, uint64_t tag, uint64_t vpc);

    bool perfApiDecode(unsigned hart, uint64_t time, uint64_t tag);

    bool perfApiExecute(unsigned hart, uint64_t time, uint64_t tag);

    bool perfApiRetire(unsigned hart, uint64_t time, uint64_t tag);

    bool perfApiDrainStore(unsigned hart, uint64_t time, uint64_t tag);

    bool perfApiPredictBranch(unsigned hart, uint64_t time, uint64_t tag, bool taken,
			      uint64_t addr);

    bool perfApiFlush(unsigned hart, uint64_t time, uint64_t tag);

    bool perfApiShouldFlush(unsigned hart, uint64_t time, uint64_t tag, bool& flush,
			    uint64_t& addr);

    std::shared_ptr<TT_PERF::PerfApi> getPerfApi()
    { return perfApi_; }

    /// Produce a signature file used to score tests from the
    /// riscv-arch-tests project.  The file is written to the
    // path specified in the parameter.
    bool produceTestSignatureFile(std::string_view outPath) const;

    bool getSparseMemUsedBlocks(std::vector<std::pair<uint64_t, uint64_t>>& usedBlocks) const;

    /// Run the simulated harts. Return true on sucess or false if
    /// target program ends with a non-zero exit. If stepWindow is
    /// not empty, run the harts in a single simulator thread
    /// round-robin with each hart executing n instructions where n is
    /// a random number in the range [stepWindowLo, stepWindowHi]. If stepWindow is
    /// 0, each hart runs in its own simulator thread independent of
    /// the other harts.
    bool batchRun(std::vector<FILE*>& traceFiles, bool waitAll, uint64_t stepWinLo, uint64_t stepWinHi);

    /// Run producing a snapshot after each snapPeriod instructions. Each
    /// snapshot goes into its own directory names <dir><n> where <dir> is
    /// the string in snapDir and <n> is a sequential integer starting at
    /// 0. Return true on success and false on failure.
    bool snapshotRun(std::vector<FILE*>& traceFiles, const std::vector<uint64_t>& periods);

    /// Set snapshot directory path.
    void setSnapshotDir(const std::string& snapDir)
    { snapDir_ = snapDir; }

  private:

    unsigned hartCount_;
    unsigned hartsPerCore_;
    TT_IMSIC::ImsicMgr imsicMgr_;
    uint64_t time_;

    std::vector< std::shared_ptr<CoreClass> > cores_;
    std::vector< std::shared_ptr<HartClass> > sysHarts_; // All harts in system.
    std::unordered_map<URV, unsigned> hartIdToIndex_;
    std::shared_ptr<Memory> memory_;
    std::shared_ptr<Syscall<URV>> syscall_;
    std::unique_ptr<SparseMem> sparseMem_;
    std::shared_ptr<Mcm<URV>> mcm_;
    std::shared_ptr<TT_PERF::PerfApi> perfApi_;
    unsigned mbSize_ = 64;  // Merge buffer size.
    std::string toHostSym_ = "tohost";   // ELF symbol to use as "tohost" addr.
    std::string fromHostSym_ = "fromhost";
    std::string consoleIoSym_ = "__whisper_console_io";  // ELF symbol to use as console-io addr.
    std::vector<std::shared_ptr<IoDevice>> ioDevs_;
    std::shared_ptr<Pci> pci_;
    std::shared_ptr<TT_APLIC::Aplic> aplic_;

    // Name, size, and address in memory of a binary file.
    typedef std::tuple<std::string, uint64_t, uint64_t> BinaryFile;
    std::vector<BinaryFile> binaryFiles_;

    std::string snapDir_ = "snapshot"; // Directory to save snapshots.
    std::atomic<int> snapIx_ = -1;
  };
}
