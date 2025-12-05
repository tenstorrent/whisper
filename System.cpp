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

#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <set>
#include <cinttypes>
#include "Hart.hpp"
#include "Core.hpp"
#include "SparseMem.hpp"
#include "System.hpp"
#include "iommu/Iommu.hpp"
#include "Mcm.hpp"
#include "PerfApi.hpp"
#include "Uart8250.hpp"
#include "Uartsf.hpp"
#include "pci/virtio/Blk.hpp"
#if REMOTE_FRAME_BUFFER
#include "RemoteFrameBuffer.hpp"
#endif


using namespace WdRiscv;


inline bool
isPowerOf2(uint64_t x)
{
  return x != 0 and (x & (x-1)) == 0;
}


template <typename URV>
System<URV>::System(unsigned coreCount, unsigned hartsPerCore,
                    unsigned hartIdOffset, size_t memSize,
                    size_t pageSize)
  : hartCount_(coreCount * hartsPerCore), hartsPerCore_(hartsPerCore),
    imsicMgr_(pageSize), time_(0),
    syscall_(std::make_unique<Syscall<URV>>(sysHarts_, memSize)),
    sparseMem_(nullptr)
{
  cores_.resize(coreCount);

  memory_ = std::make_unique<Memory>(memSize, pageSize);

  Memory& mem = *memory_;
  mem.setHartCount(hartCount_);

  for (unsigned ix = 0; ix < coreCount; ++ix)
    {
      URV coreHartId = ix * hartIdOffset;
      cores_.at(ix) = std::make_shared<CoreClass>(coreHartId, ix, hartsPerCore, mem, *syscall_, time_);

      // Maintain a vector of all the harts in the system.  Map hart-id to index
      // of hart in system.
      auto core = cores_.at(ix);
      for (unsigned i = 0; i < hartsPerCore; ++i)
        {
          auto hart = core->ithHart(i);
          sysHarts_.push_back(hart);
          URV hartId = coreHartId + i;
          unsigned hartIx = ix*hartsPerCore + i;
          hartIdToIndex_[hartId] = hartIx;
        }
    }

#ifdef MEM_CALLBACKS
  sparseMem_ = std::make_unique<SparseMem>();

  auto readf = [this](uint64_t addr, unsigned size, uint64_t& value) -> bool {
                 return sparseMem_->read(addr, size, value); };

  auto writef = [this](uint64_t addr, unsigned size, uint64_t value) -> bool {
                  return sparseMem_->write(addr, size, value); };

  auto initf = [this](uint64_t addr, const std::span<uint8_t> buffer) -> bool {
                 return sparseMem_->initializePage(addr, buffer); };

  mem.defineReadMemoryCallback(readf);
  mem.defineWriteMemoryCallback(writef);
  mem.defineInitPageCallback(initf);
#endif
}

template <typename URV>
bool
System<URV>::defineUart(const std::string& type, uint64_t addr, uint64_t size,
    uint32_t iid, const std::string& channel_type, unsigned regShift)
{
  std::shared_ptr<IoDevice> dev;


  auto createChannel = [](std::string_view channel_type) -> std::unique_ptr<UartChannel> {
    auto createChannelImpl = [](std::string_view channel_type, auto &createChannelImpl) -> std::unique_ptr<UartChannel> {
      constexpr std::string_view unixPrefix = "unix:";

      auto pos = channel_type.find(';');
      if (pos != std::string::npos)
      {
        std::string_view readWriteChannelType = channel_type.substr(0, pos);
        std::string_view writeOnlyChannelType = channel_type.substr(pos + 1);

        std::unique_ptr<UartChannel> readWriteChannel = createChannelImpl(readWriteChannelType, createChannelImpl);
        std::unique_ptr<UartChannel> writeOnlyChannel = createChannelImpl(writeOnlyChannelType, createChannelImpl);

        return std::make_unique<ForkChannel>(std::move(readWriteChannel), std::move(writeOnlyChannel));
      }
      if (channel_type == "stdio")
        return std::make_unique<FDChannel>(fileno(stdin), fileno(stdout));
      if (channel_type == "pty")
        return std::make_unique<PTYChannel>();
      if (channel_type.find(unixPrefix, 0) == 0)
      {
        std::string filename(channel_type.substr(unixPrefix.length()));
        if (filename.empty()) {
          std::cerr << "Error: System::defineUart: Missing filename for unix socket channel\n";
          return nullptr;
        }

        int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (server_fd < 0) {
          perror("System::defineUart: Failed to create unix socket");
          return nullptr;
        }

        struct sockaddr_un addr{};
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy((char*) addr.sun_path, filename.c_str(), sizeof(addr.sun_path) - 1);

        // Remove existing socket file if present before binding
        unlink(filename.c_str());

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
          perror("System::defineUart: Failed to bind unix socket");
          close(server_fd);
          return nullptr;
        }

        if (listen(server_fd, 1) < 0) {
          perror("System::defineUart: Failed to listen on unix socket");
          close(server_fd);
          return nullptr;
        }

        std::cerr << "Info: System::defineUart: Listening on unix socket: " << filename << "\n";
        std::unique_ptr<SocketChannel> channel;
        try {
          channel = std::make_unique<SocketChannel>(server_fd);
        } catch (const std::runtime_error& e) {
          std::cerr << "Error: System::defineUart: Failed to create SocketChannel: " << e.what() << "\n";
          close(server_fd);
          unlink(filename.c_str());
          return nullptr;
        }

        // SocketChannel constructor called accept(), server fd no longer needed here.
        close(server_fd);
        unlink(filename.c_str());

        return channel;
      }

      std::cerr << "Error: System::defineUart: Invalid channel type: " << channel_type << "\n"
      << "Valid channels: stdio, pty, unix:<server socket path>, or a"
      << "semicolon separated list of those.\n";
      return nullptr;
    };
    return createChannelImpl(channel_type, createChannelImpl);
  };

  if (type == "uartsf")
    dev = std::make_shared<Uartsf>(addr, size);
  else if (type == "uart8250")
  {
    std::unique_ptr<UartChannel> channel = createChannel(channel_type);
    if (!channel)
      return false;
    dev = std::make_shared<Uart8250>(addr, size, aplic_, iid, std::move(channel), false, regShift);
  }
  else
  {
    std::cerr << "Error: System::defineUart: Invalid uart type: " << type << "\n";
    return false;
  }

  memory_->registerIoDevice(dev);
  ioDevs_.push_back(std::move(dev));

  return true;
}

#if REMOTE_FRAME_BUFFER
template <typename URV>
bool
System<URV>::defineFrameBuffer(const std::string& type, uint64_t addr, uint64_t width, uint64_t height, uint64_t bytes_per_pixel, int port)
{
  std::shared_ptr<IoDevice> dev;

  if (type == "rfb")
    dev = std::make_shared<RemoteFrameBuffer>(addr, width, height, bytes_per_pixel, port);
  else
    {
      std::cerr << "System::defineFrameBuffer: Invalid frame_buffer type: " << type << "\n";
      return false;
    }

  memory_->registerIoDevice(dev);
  ioDevs_.push_back(std::move(dev));

  return true;
}
#endif

template <typename URV>
System<URV>::~System()
{
  // Final MCM checks
  if (mcm_)
    for (const auto& hartPtr : sysHarts_)
      mcm_->finalChecks(*hartPtr);

  // Write back binary files were marked for update.
  for (auto bf : binaryFiles_)
    {
      auto path = std::get<0>(bf);
      uint64_t addr = std::get<1>(bf);
      uint64_t size = std::get<2>(bf);
      std::cerr << "Info: Updating " << path << " from addr: 0x" << std::hex << addr
		<< std::dec << " size: " << size << '\n';
      util::file::SharedFile file = util::file::make_shared_file(fopen(path.c_str(), "w"));
      if (not file)
	{
	  std::cerr << "Error: Failed to open " << path << " for update\n";
	  continue;
	}
      for (uint64_t i = 0; i < size; ++i)
	{
	  uint8_t byte = 0;
	  memory_->peek(addr + i, byte, false);
	  fputc(byte, file.get());
	}
    }
}


template <typename URV>
void
System<URV>::checkUnmappedElf(bool flag)
{
  if (memory_)
    memory_->checkUnmappedElf(flag);
}


template <typename URV>
bool
System<URV>::writeAccessedMemory(const std::string& path) const
{
  if (not sparseMem_)
    return false;
  return sparseMem_->writeHexFile(path);
}


template <typename URV>
bool
System<URV>::loadElfFiles(const std::vector<std::string>& files, bool raw, bool verbose)
{
  unsigned registerWidth = sizeof(URV)*8;
  uint64_t end = 0, entry = 0;
  uint64_t gp = 0, tp = 0; // gp: global-pointer, tp: thread-pointer
  unsigned errors = 0;
  ElfSymbol sym;

  for (const auto& file : files)
    {
      if (verbose)
	std::cerr << "Info: Loading ELF file " << file << '\n';
      uint64_t end0 = 0, entry0 = 0;
      if (not memory_->loadElfFile(file, registerWidth, entry0, end0))
	errors++;
      else
	{
	  if (not entry)
	    entry = entry0;

	  if (memory_->findElfSymbol("_end", sym))   // For newlib/linux emulation.
	    end = std::max(end, sym.addr_);
	  else
	    end = std::max(end, end0);

	  if (not gp and memory_->findElfSymbol("__global_pointer$", sym))
	    gp = sym.addr_;

	  if (not tp and memory_->findElfSection(".tdata", sym))
	    tp = sym.addr_;
	}
    }

  for (const auto& hart : sysHarts_)
    {
      if (not toHostSym_.empty() and memory_->findElfSymbol(toHostSym_, sym))
	hart->setToHostAddress(sym.addr_);
      if (not fromHostSym_.empty() and memory_->findElfSymbol(fromHostSym_, sym))
	hart->setFromHostAddress(sym.addr_, true);
      if (not consoleIoSym_.empty() and memory_->findElfSymbol(consoleIoSym_, sym))
	hart->setConsoleIo(URV(sym.addr_));

      if (verbose)
	std::cerr << "Info: Setting program break to 0x" << std::hex << end << std::dec << '\n';
      hart->setTargetProgramBreak(end);

      if (not raw)
	{
	  if (not hart->peekIntReg(RegGp) and gp)
	    {
              if (verbose)
                std::cerr << "Info: Setting register gp to 0x" << std::hex << gp << std::dec << '\n';
	      hart->pokeIntReg(RegGp, URV(gp));
	    }
	  if (not hart->peekIntReg(RegTp) and tp)
	    {
              if (verbose)
                std::cerr << "Info: Setting register tp to 0x" << std::hex << tp << std::dec << '\n';
	      hart->pokeIntReg(RegTp, URV(tp));
	    }
	  if (entry)
	    {
	      if (verbose)
                std::cerr << "Info: Setting PC to 0x" << std::hex << entry << std::dec << '\n';
	      hart->pokePc(URV(entry));
	    }
	}
    }

  return errors == 0;
}


template <typename URV>
bool
System<URV>::loadHexFiles(const std::vector<std::string>& files, bool verbose)
{
  unsigned errors = 0;
  for (const auto& file : files)
    {
      if (verbose)
	std::cerr << "Info: Loading HEX file " << file << '\n';
      if (not memory_->loadHexFile(file))
	errors++;
    }
  return errors == 0;
}


static bool
binaryFileParams(std::string spec, uint64_t defOffset, std::string& filename, uint64_t &offset, bool &update)
{
  using std::cerr;
  // Split filespec around colons. Spec format: <file>,
  // <file>:<offset>, or <file>:<offset>:u
  std::vector<std::string> parts;
  boost::split(parts, spec, boost::is_any_of(":"));

  filename = parts.at(0);
  offset = defOffset;
  update = false;

  if (parts.empty())
    {
      std::cerr << "Error: Empty binary file name\n";
      return false;
    }

  filename = parts.at(0);

  if (parts.size() > 1)
    {
      std::string offsStr = parts.at(1);
      if (offsStr.empty())
	cerr << "Warning: Empty binary file offset: " << spec << '\n';
      else
	{
	  char* tail = nullptr;
	  offset = strtoull(offsStr.c_str(), &tail, 0);
	  if (tail and *tail)
	    {
	      cerr << "Error: Invalid binary file offset: " << spec << '\n';
	      return false;
	    }
	}
    }
  else
    cerr << "Warning: Binary file " << filename << " does not have an address, will use address 0x"
	 << std::hex << offset << std::dec << '\n';

  if (parts.size() > 2)
    {
      if (parts.at(2) != "u")
	{
	  cerr << "Error: Invalid binary file attribute: " << spec << '\n';
	  return false;
	}
      update = true;
    }

  return true;
}


template <typename URV>
bool
System<URV>::loadBinaryFiles(const std::vector<std::string>& fileSpecs,
			     uint64_t defOffset, bool verbose)
{
  using std::cerr;
  unsigned errors = 0;

  for (const auto& spec : fileSpecs)
    {

      std::string filename;
      uint64_t offset = 0;
      bool update = false;

      if (!binaryFileParams(spec, defOffset, filename, offset, update))
        {
	  errors++;
	  continue;
        }

      if (verbose)
	cerr << "Info: Loading binary " << filename << " at address 0x" << std::hex
	     << offset << std::dec << '\n';

      if (not memory_->loadBinaryFile(filename, offset))
	{
	  errors++;
	  continue;
	}

      if (update)
	{
	  uint64_t size = Filesystem::file_size(filename);
	  BinaryFile bf = { filename, offset, size };
	  binaryFiles_.push_back(bf);
	}
    }

  return errors == 0;
}


#if LZ4_COMPRESS
template <typename URV>
bool
System<URV>::loadLz4Files(const std::vector<std::string>& fileSpecs,
			  uint64_t defOffset, bool verbose)
{
  using std::cerr;
  unsigned errors = 0;

  for (const auto& spec : fileSpecs)
    {
      std::string filename;
      uint64_t offset;
      bool update;

      if (!binaryFileParams(spec, defOffset, filename, offset, update))
        {
	  errors++;
	  continue;
        }

      if (update)
	{
	  cerr << "Error: Updating not supported on lz4 files, ignoring " << filename << '\n';
	  errors++;
	  continue;
	}

      if (verbose)
	cerr << "Info: Loading lz4 compressed file " << filename << " at address 0x" << std::hex
	     << offset << std::dec << '\n';

      if (not memory_->loadLz4File(filename, offset))
	{
	  errors++;
	  continue;
	}

      if (update)
	{
	  uint64_t size = Filesystem::file_size(filename);
	  BinaryFile bf = { filename, offset, size };
	  binaryFiles_.push_back(bf);
	}
    }

  return errors == 0;
}
#endif


static
bool
saveUsedMemBlocks(const std::string& filename,
		  std::vector<std::pair<uint64_t, uint64_t>>& blocks)
{
  std::ofstream ofs(filename, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "Error: saveUsedMemBlocks failed - cannot open "
                << filename << " for write\n";
      return false;
    }
  for (auto& it: blocks)
    ofs << it.first << " " << it.second << "\n";
  return true;
}


static
bool
saveTime(const std::string& filename, uint64_t time)
{
  std::ofstream ofs(filename, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "Error: saveTime failed - cannot open "
                << filename << " for write\n";
      return false;
    }
  ofs << std::dec << time << "\n";
  return true;
}


template <typename URV>
bool
System<URV>::saveSnapshot(const std::string& dir)
{
  for (auto& dev : ioDevs_)
    dev->disable();

  Filesystem::path dirPath = dir;
  if (not Filesystem::is_directory(dirPath))
    if (not Filesystem::create_directories(dirPath))
      {
	std::cerr << "Error: Failed to create snapshot directory " << dir << '\n';
	return false;
      }

  uint64_t minSp = ~uint64_t(0);

  for (const auto& hartPtr : sysHarts_)
    {
      std::string name = "registers";
      if (hartCount_ > 1)
	name += std::to_string(hartPtr->sysHartIndex());
      Filesystem::path regPath = dirPath / name;
      if (not hartPtr->saveSnapshotRegs(regPath.string()))
	return false;

      URV sp = 0;
      if (not hartPtr->peekIntReg(IntRegNumber::RegSp, sp))
	assert(0 && "Error: Assertion failed");
      minSp = std::min(minSp, uint64_t(sp));
    }

  auto& hart0 = *ithHart(0);
  auto& syscall = hart0.getSyscall();

  Filesystem::path usedBlocksPath = dirPath / "usedblocks";
  std::vector<std::pair<uint64_t,uint64_t>> usedBlocks;
  if (sparseMem_)
    sparseMem_->getUsedBlocks(usedBlocks);
  else
    syscall.getUsedMemBlocks(minSp, usedBlocks);

  if (not saveUsedMemBlocks(usedBlocksPath.string(), usedBlocks))
    return false;

  Filesystem::path timePath = dirPath / "time";
  if (not saveTime(timePath.string(), time_))
    return false;

  Filesystem::path memPath = dirPath / "memory";

  if (snapCompressionType_ == "lz4")
    {
#if LZ4_COMPRESS
      if (not memory_->saveSnapshot_lz4(memPath.string(), usedBlocks))
        {
          std::cerr << "Error in saving snapshot - lz4\n";
          return false;
        }
#else
      std::cerr << "Error: LZ4 compression is not enabled\n";
      return false;
#endif
    }
  else if (snapCompressionType_ == "gzip") 
    {
      if (not memory_->saveSnapshot_gzip(memPath.string(), usedBlocks))
        {
          std::cerr << "Error in saving snapshot - gzip\n";
          return false;
        }
    }
  else
    {
      std::cerr << "Error: Invalid compression type: " << snapCompressionType_ << "\n";
      return false;
    }

  Filesystem::path mtimecmpPath = dirPath / "mtimecmp";
  {
    std::ofstream ofs(mtimecmpPath.string());
    if (not ofs)
      {
        std::cerr << "Failed to open snapshot file for saving mtimecmp\n";
        return false;
      }
    for (auto& hartPtr : sysHarts_)
      {
        uint64_t timecmp = hartPtr->getAclintAlarm();
        ofs << std::hex << "0x" << timecmp << std::dec << "\n";
      }
  }

  Filesystem::path fdPath = dirPath / "fd";
  if (not syscall.saveFileDescriptors(fdPath.string()))
    return false;

  Filesystem::path mmapPath = dirPath / "mmap";
  if (not syscall.saveMmap(mmapPath.string()))
    return false;

  Filesystem::path dtracePath = dirPath / "data-lines";
  if (not memory_->saveDataAddressTrace(dtracePath))
    return false;

  Filesystem::path itracePath = dirPath / "instr-lines";
  if (not memory_->saveInstructionAddressTrace(itracePath))
    return false;

  Filesystem::path branchPath = dirPath / "branch-trace";
  if (not hart0.saveBranchTrace(branchPath))
    return false;

  Filesystem::path cachePath = dirPath / "cache-trace";
  if (not hart0.saveCacheTrace(cachePath))
    return false;

  Filesystem::path imsicPath = dirPath / "imsic";
  if (not imsicMgr_.saveSnapshot(imsicPath))
    return false;

  if (not saveAplicSnapshot(dirPath))
    return false;

  std::set<std::string_view> ioDevTypes;
  for (const auto& dev : ioDevs_)
    {
      if (ioDevTypes.contains(dev->type()))
        {
          std::cerr << "Error: currently cannot save snapshots for multiple devices of the same type, " <<  dev->type() << '\n';
          return false;
        }
      ioDevTypes.insert(dev->type());
      Filesystem::path devPath = dirPath / dev->type();
      if (not dev->saveSnapshot(devPath))
        return false;
    }

  for (auto& dev : ioDevs_)
    dev->enable();
  return true;
}


static
bool
loadUsedMemBlocks(const std::string& filename,
		  std::vector<std::pair<uint64_t, uint64_t>>& blocks)
{
  blocks.clear();
  std::ifstream ifs(filename);
  if (not ifs)
    {
      std::cerr << "Error: loadUsedMemBlocks failed - cannot open "
                << filename << " for read\n";
      return false;
    }

  std::string line;
  while (std::getline(ifs, line))
    {
      std::istringstream iss(line);
      uint64_t addr = 0, length = 0;
      iss >> addr;
      iss >> length;
      blocks.emplace_back(addr, length);
    }

  return true;
}


static
bool
loadTime(const std::string& filename, uint64_t& time)
{
  std::ifstream ifs(filename);
  if (not ifs)
    {
      std::cerr << "Error: loadTime failed - cannot open "
                << filename << " for read\n";
      return false;
    }

  std::string line;
  std::getline(ifs, line);
  uint64_t val = strtoull(line.c_str(), nullptr, 0);
  time = val;
  return true;
}


template <typename URV>
bool
System<URV>::configImsic(uint64_t mbase, uint64_t mstride,
			 uint64_t sbase, uint64_t sstride,
			 unsigned guests, const std::vector<unsigned>& idsVec,
                         const std::vector<unsigned>& tmVec, // Threshold masks
                         bool maplic, bool saplic, bool trace)
{
  using std::cerr;

  imsicMgr_.createImsics(hartCount_);

  size_t ps = pageSize();

  if ((mbase % ps) != 0)
    {
      cerr << "Error: IMISC mbase (0x" << std::hex << mbase << ") is not"
	   << " a multiple of page size (0x" << ps << ")\n" << std::dec;
      return false;
    }

  if (mstride == 0)
    {
      cerr << "Error: IMSIC mstride must not be zero.\n";
      return false;
    }

  if ((mstride % ps) != 0)
    {
      cerr << "Error: IMISC mstride (0x" << std::hex << mstride << ") is not"
	   << " a multiple of page size (0x" << ps << ")\n" << std::dec;
      return false;
    }

  if (sstride)
    {
      if ((sbase % ps) != 0)
	{
	  cerr << "Error: IMISC sbase (0x" << std::hex << sbase << ") is not"
	       << " a multiple of page size (0x" << ps << ")\n" << std::dec;
	  return false;
	}

      if ((sstride % ps) != 0)
	{
	  cerr << "Error: IMISC sstride (0x" << std::hex << sstride << ") is not"
	       << " a multiple of page size (0x" << ps << ")\n" << std::dec;
	  return false;
	}
    }

  if (guests and sstride < (guests + 1)*ps)
    {
      cerr << "Error: IMISC supervisor stride (0x" << std::hex << sstride << ") is"
	   << " too small for configured guests (" << std::dec << guests << ").\n";
      return false;
    }

  if (mstride and sstride)
    {
      unsigned hc = hartCount();
      uint64_t mend = mbase + hc*mstride, send = sbase + hc*sstride;
      if ((sbase > mbase and sbase < mend) or
	  (send > mbase and send < mend))
	{
	  cerr << "Error: IMSIC machine file address range overlaps that of supervisor.\n";
	  return false;
	}
    }

  if (idsVec.size() != 3)
    {
      cerr << "Error: IMSIC interrupt-ids array size (" << idsVec.size() << ") is "
	   << "invalid -- Expecting 3.\n";
      return false;
    }

  for (auto ids : idsVec)
    {
      if ((ids % 64) != 0)
	{
	  cerr << "Error: IMSIC interrupt id limit (" << ids << ") is not a multiple of 64.\n";
	  return false;
	}

      if (ids > 2048)
	{
	  cerr << "Error: IMSIC interrupt id limit (" << ids << ") is larger than 2048.\n";
	  return false;
	}
    }

  if (idsVec.size() != tmVec.size())
    {
      cerr << "Error: IMSIC interrupt ids count (" << idsVec.size() << ") is different "
	   << " thant the threshold-mask count (" << tmVec.size() << ")\n";
      return false;
    }

  for (size_t i = 0; i < idsVec.size(); ++i)
    {
      if (tmVec.at(i) < idsVec.at(i) - 1)
	{
	  cerr << "Error: Threshold mask (" << tmVec.at(0) << ") cannot be less than the "
	       << "max interrupt id (" << (idsVec.at(i) - 1) << ").\n";
	  return false;
	}
    }

  bool ok = imsicMgr_.configureMachine(mbase, mstride, idsVec.at(0), tmVec.at(0), maplic);
  ok = imsicMgr_.configureSupervisor(sbase, sstride, idsVec.at(1), tmVec.at(1), saplic) and ok;
  ok = imsicMgr_.configureGuests(guests, idsVec.at(2), tmVec.at(2)) and ok;
  if (not ok)
    {
      cerr << "Error: Failed to configure IMSIC.\n";
      return false;
    }

  uint64_t mend = mbase + mstride * hartCount();
  uint64_t send = sbase + sstride * hartCount();

  auto readFunc = [this](uint64_t addr, unsigned size, uint64_t& data) -> bool {
    return this->imsicMgr_.read(addr, size, data);
  };

  auto writeFunc = [this](uint64_t addr, unsigned size, uint64_t data) -> bool {
    return  this->imsicMgr_.write(addr, size, data);
  };

  for (unsigned i = 0; i < hartCount(); ++i)
    {
      auto hart = ithHart(i);
      auto imsic = imsicMgr_.ithImsic(i);
      hart->attachImsic(imsic, mbase, mend, sbase, send, readFunc, writeFunc, trace);
    }

  return true;
}


template <typename URV>
bool
System<URV>::configAplic(unsigned num_sources, std::span<const TT_APLIC::DomainParams> domain_params)
{
  aplic_ = std::make_shared<TT_APLIC::Aplic>(hartCount_, num_sources, domain_params);

  TT_APLIC::DirectDeliveryCallback aplicCallback = [this] (unsigned hartIx, TT_APLIC::Privilege privilege, bool interState) -> bool {
    bool is_machine = privilege == TT_APLIC::Machine;
    std::cerr << "Info: Delivering interrupt hart=" << hartIx << " privilege="
              << (is_machine ? "machine" : "supervisor")
              << " interrupt-state=" << (interState? "on" : "off") << '\n';
    // if an IMSIC is present, then interrupt should only be delivery if its eidelivery is 0x40000000
    auto& hart = *ithHart(hartIx);
    auto imsic = hart.imsic();
    if (imsic)
      {
        unsigned eidelivery = is_machine ? imsic->machineDelivery() : imsic->supervisorDelivery();
        if (eidelivery != 0x40000000)
          {
            std::cerr << "Error: Cannot deliver interrupt; for direct delivery mode, IMSIC's eidelivery must be 0x40000000\n";
            return false;
          }
      }
    auto mip = hart.peekCsr(CsrNumber::MIP);
    mip = hart.overrideWithMvip(mip);
    int xeip = is_machine ? 11 : 9;
    if (interState)
      mip |= 1 << xeip;
    else
      mip &= ~(1 << xeip);
    return hart.pokeCsr(CsrNumber::MIP, mip);
  };
  aplic_->setDirectCallback(aplicCallback);

  TT_APLIC::MsiDeliveryCallback imsicFunc = [this] (uint64_t addr, uint32_t data) -> bool {
    return imsicMgr_.write(addr, 4, data);
  };
  aplic_->setMsiCallback(imsicFunc);

  for (auto& hart : sysHarts_)
    hart->attachAplic(aplic_);

  return true;
}


template <typename URV>
bool
System<URV>::configIommu(uint64_t base_addr, uint64_t size, uint64_t capabilities,
                         unsigned aplic_source)
{
  iommuAplicSource_ = aplic_source;
  uint64_t memSize = this->memory_->size();
  iommu_ = std::make_shared<TT_IOMMU::Iommu>(base_addr, size, memSize, capabilities);

  auto readCb = [this](uint64_t addr, unsigned size, uint64_t& data) -> bool {
    uint8_t data8 = 0;
    uint16_t data16 = 0;
    uint32_t data32 = 0;
    auto& hart0 = sysHarts_[0];
    bool result = false;
    if (hart0->isDeviceAddr(addr))
      {
        hart0->deviceRead(addr, size, data);
        return true;
      }
    switch (size)
       {
          case 1: result = this->memory_->read(addr, data8);  data = data8;  break;
          case 2: result = this->memory_->read(addr, data16); data = data16; break;
          case 4: result = this->memory_->read(addr, data32); data = data32; break;
          case 8: result = this->memory_->read(addr, data); break;
         default: assert(0);
       }
    return result;
  };

  auto writeCb = [this](uint64_t addr, unsigned size, uint64_t data) -> bool {
    uint8_t data8 = data;
    uint16_t data16 = data;
    uint32_t data32 = data;
    auto& hart0 = sysHarts_[0];
    if (hart0->isDeviceAddr(addr))
      {
        switch (size)
          {
            case 1: hart0->deviceWrite(addr, data8); break;
            case 2: hart0->deviceWrite(addr, data16); break;
            case 4: hart0->deviceWrite(addr, data32); break;
            case 8: hart0->deviceWrite(addr, data); break;
            default: assert(0);
          }
        return true;
      }
    switch (size)
      {
        case 1: return this->memory_->write(0, addr, data8);
        case 2: return this->memory_->write(0, addr, data16);
        case 4: return this->memory_->write(0, addr, data32);
        case 8: return this->memory_->write(0, addr, data);
        default: assert(0);
      }
    return false;
  };

  iommu_->setMemReadCb(readCb);
  iommu_->setMemWriteCb(writeCb);

  auto sendInvalReqCb = [](uint32_t devId, uint32_t pid, bool pv, uint64_t address, bool global, TT_IOMMU::InvalidationScope scope, uint8_t itag) {
    printf("Sending invalidation request to device. devId: %u pid: %u pv: %d address: %lu global: %d scope: %d itag: %u\n", devId, pid, pv, address, global, static_cast<int>(scope), itag);
  };
  auto sendPrgrCb = [](uint32_t devId, uint32_t pid, bool pv, uint32_t prgi, uint32_t resp_code, bool dsv, uint32_t dseg) {
    printf("Sending PageRequestGroupResponse to device. devId: %u pid: %u pv: %d prgi: %u resp code: %u dsv: %d dseg: %u\n", devId, pid, pv, prgi, resp_code, dsv, dseg);
  };

  iommu_->setSendInvalReqCb(sendInvalReqCb);
  iommu_->setSendPrgrCb(sendPrgrCb);

  // iommu wsi callback to APLIC (single-wire mode)
  auto wiredInterruptCb = [this](unsigned /*vector*/, bool assertInt) {
    if (!aplic_) {
      // No APLIC configured - WSI not supported in this configuration
      return;
    }
    
    // If aplic_source is 0 (not configured), WSI is not used
    if (iommuAplicSource_ == 0) {
      return;
    }
    
    // Single-wire mode: All IOMMU interrupts use the same APLIC source
    // Source is configurable via whisper.json (iommu.aplic_source)
    if (iommuAplicSource_ > aplic_->numSources()) {
      std::cerr << "Error: IOMMU interrupt source " << iommuAplicSource_
                << " exceeds APLIC source count " << aplic_->numSources() << '\n';
      return;
    }
    
    // Signal APLIC to assert/deassert the interrupt source
    aplic_->setSourceState(iommuAplicSource_, assertInt);
  };
  
  iommu_->setSignalWiredInterruptCb(wiredInterruptCb);

  iommuVirtMem_ = std::make_shared<VirtMem>(0, 4096, 2048);
  iommuVirtMem_->enableNapot(true);
  TT_IOMMU::Capabilities cap = { .value = capabilities };
  iommuVirtMem_->enablePbmt(cap.fields.svpbmt);
  iommuVirtMem_->enableVsPbmt(cap.fields.svpbmt);
  iommuVirtMem_->enableRsw60t59b(cap.fields.svrsw60t59b);

  auto readCallbackDoubleword = [this](uint64_t addr, bool bigEndian, uint64_t& data) -> bool {
    (void) bigEndian;
    return this->memory_->read(addr, data);
  };
  auto readCallbackWord = [this](uint64_t addr, bool bigEndian, uint32_t& data) -> bool {
    (void) bigEndian;
    return this->memory_->read(addr, data);
  };
  std::function<bool(uint64_t,bool,uint64_t)> writeCallbackDoubleword = [this](uint64_t addr, bool bigEndian, uint64_t data) -> bool {
    (void) bigEndian;
    return this->memory_->write(0, addr, data);
  };
  std::function<bool(uint64_t,bool,uint32_t)> writeCallbackWord = [this](uint64_t addr, bool bigEndian, uint32_t data) -> bool {
    (void) bigEndian;
    return this->memory_->write(0, addr, data);
  };
  iommuVirtMem_->setMemReadCallback(readCallbackDoubleword);
  iommuVirtMem_->setMemReadCallback(readCallbackWord);
  iommuVirtMem_->setMemWriteCallback(writeCallbackDoubleword);
  iommuVirtMem_->setMemWriteCallback(writeCallbackWord);

  auto configStage1 = [this](unsigned mode, unsigned asid, uint64_t ppn, bool sum) {
    this->iommuVirtMem_->configStage1(WdRiscv::Tlb::Mode(mode), asid, ppn, sum);
  };

  auto configStage2 = [this](unsigned mode, unsigned vmid, uint64_t ppn) {
    this->iommuVirtMem_->configStage2(WdRiscv::Tlb::Mode(mode), vmid, ppn);
  };

  auto setFaultOnFirstAccess = [this](unsigned stage, bool flag) {
    switch (stage) {
      case 0: this->iommuVirtMem_->setFaultOnFirstAccess(flag); break;
      case 1: this->iommuVirtMem_->setFaultOnFirstAccessStage1(flag); break;
      case 2: this->iommuVirtMem_->setFaultOnFirstAccessStage2(flag); break;
      default: assert(false);
    }
  };

  auto stage1Cb = [this](uint64_t va, unsigned privMode, bool r, bool w, bool x, uint64_t& gpa, unsigned& cause) -> bool {
    cause = int(this->iommuVirtMem_->stage1Translate(va, WdRiscv::PrivilegeMode(privMode), r, w, x, gpa));
    return cause == int(WdRiscv::ExceptionCause::NONE);
  };

  auto stage2Cb = [this](uint64_t gpa, unsigned privMode, bool r, bool w, bool x, uint64_t& pa, unsigned& cause) -> bool {
    cause = int(this->iommuVirtMem_->stage2Translate(gpa, WdRiscv::PrivilegeMode(privMode), r, w, x, false, pa));
    return cause == int(WdRiscv::ExceptionCause::NONE);
  };

  auto stage2TrapInfo = [](uint64_t& gpa, bool& implicit, bool& write) {
    gpa = 0;
    implicit = false;
    write = false;
  };
  iommu_->setStage1ConfigCb(configStage1);
  iommu_->setStage2ConfigCb(configStage2);
  iommu_->setStage1Cb(stage1Cb);
  iommu_->setStage2Cb(stage2Cb);
  iommu_->setStage2TrapInfoCb(stage2TrapInfo);
  iommu_->setSetFaultOnFirstAccess(setFaultOnFirstAccess);

  for (auto& hart : sysHarts_)
    hart->attachIommu(iommu_);

  return true;
}


template <typename URV>
bool
System<URV>::configPci(uint64_t configBase, uint64_t mmioBase, uint64_t mmioSize, unsigned buses, unsigned slots)
{
  if (mmioBase - configBase < (1ULL << 28))
    {
      std::cerr << "Error: PCI config space typically needs 28bits to fully cover entire region" << '\n';
      return false;
    }

  pci_ = std::make_shared<Pci>(configBase, (1ULL << 28), mmioBase, mmioSize, buses, slots);

  auto readf = [this](uint64_t addr, size_t size, uint64_t& data) -> bool {
    bool ok = false;
    if (size == 1)
      {
        uint8_t tmp = 0;
        ok = this->memory_->peek(addr, tmp, false);
        data = tmp;
      }
    if (size == 2)
      {
        uint16_t tmp = 0;
        ok = this->memory_->peek(addr, tmp, false);
        data = tmp;
      }
    if (size == 4)
      {
        uint32_t tmp = 0;
        ok = this->memory_->peek(addr, tmp, false);
        data = tmp;
      }
    if (size == 8)
      return this->memory_->peek(addr, data, false);
    return ok;
  };

  auto writef = [this](uint64_t addr, size_t size, uint64_t data) -> bool {
    bool ok = false;
    if (size == 1)
      ok = this->memory_->poke(addr, uint8_t(data), false);
    if (size == 2)
      ok = this->memory_->poke(addr, uint16_t(data), false);
    if (size == 4)
      ok = this->memory_->poke(addr, uint32_t(data), false);
    if (size == 8)
      ok = this->memory_->poke(addr, data, false);
    return ok;
  };

  auto msif = [this](uint64_t addr, unsigned size, uint64_t data) -> bool {
    return this->imsicMgr_.write(addr, size, data);
  };

  pci_->define_read_mem(readf);
  pci_->define_write_mem(writef);
  pci_->define_msi(msif);

  for (auto& hart : sysHarts_)
    hart->attachPci(pci_);
  return true;
}


template <typename URV>
bool
System<URV>::addPciDevices(const std::vector<std::string>& devs)
{
  // NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks)
  if (not pci_)
    {
      std::cerr << "Error: Please specify a PCI region in the json" << '\n';
      return false;
    }

  for (const auto& devStr : devs)
    {
      std::vector<std::string> tokens;
      boost::split(tokens, devStr, boost::is_any_of(":"), boost::token_compress_on);

      if (tokens.size() < 3)
        {
          std::cerr << "Error: PCI device string should have at least 3 fields" << '\n';
          return false;
        }

      std::string name = tokens.at(0);
      unsigned bus = std::stoi(tokens.at(1));
      unsigned slot = std::stoi(tokens.at(2));

      if (name == "virtio-blk")
        {
          if (not (tokens.size() == 4))
            {
              std::cerr << "Error: virtio-blk requires backing input file" << '\n';
              return false;
            }

          std::shared_ptr<Blk> dev = std::make_shared<Blk>(false);
          if (not dev->open_file(tokens.at(3)))
            return false;

          if (not pci_->register_device(dev, bus, slot))
            return false;
        }
      else
        return false;
    }
  return true;
  // NOLINTEND(clang-analyzer-cplusplus.NewDeleteLeaks)
}


template <typename URV>
bool
System<URV>::enableMcm(unsigned mbLineSize, bool mbLineCheckAll, bool mcmCache,
		       const std::vector<unsigned>& enabledPpos)
{
  if (mbLineSize == 0 or not isPowerOf2(mbLineSize) or mbLineSize > 512)
    {
      std::cerr << "Error: Invalid merge buffer line size: "
                << mbLineSize << '\n';
      return false;
    }

  mcm_ = std::make_shared<Mcm<URV>>(this->hartCount(), pageSize(), mbLineSize);
  mbSize_ = mbLineSize;
  mcm_->setCheckWholeMbLine(mbLineCheckAll);

  mcm_->enablePpo(false);

  // For easier handling of CMOs. I-cache is considered non-coherent.
  // FIXME: Make mcmCache apply to fetchCache as well.
  if (mcmCache)
    {
      dataCache_ = std::make_shared<TT_CACHE::Cache>();
      auto dataMemRead = [this](uint64_t addr, uint64_t& value) {
        return this->memory_->peek(addr, value, false);
      };
      auto memWrite = [this](uint64_t addr, uint64_t value) {
        return this->memory_->poke(addr, value, false);
      };
      dataCache_->addMemReadCallback(dataMemRead);
      dataCache_->addMemWriteCallback(memWrite);
    }

  for (auto ppoIx : enabledPpos)
    if (ppoIx < Mcm<URV>::PpoRule::Limit)
      {
	typedef typename Mcm<URV>::PpoRule Rule;
	Rule rule = Rule(ppoIx);
	mcm_->enablePpo(rule, true);
      }

  for (auto& hart :  sysHarts_)
    {
      auto fetchCache = std::make_shared<TT_CACHE::Cache>();
      auto fetchMemRead = [this](uint64_t addr, uint64_t& value) {
        if (dataCache_)
          return dataCache_->read(addr, value)? true : this->memory_->peek(addr, value, false);
        return this->memory_->peek(addr, value, false);
      };
      fetchCache->addMemReadCallback(fetchMemRead);
      hart->setMcm(mcm_, fetchCache, dataCache_);
    }

  return true;
}


template <typename URV>
bool
System<URV>::enableMcm(unsigned mbLineSize, bool mbLineCheckAll, bool mcmCache, bool enablePpos)
{
  if (mbLineSize != 0)
    if (not isPowerOf2(mbLineSize) or mbLineSize > 512)
      {
	std::cerr << "Error: Invalid merge buffer line size: "
		  << mbLineSize << '\n';
	return false;
      }

  mcm_ = std::make_shared<Mcm<URV>>(this->hartCount(), pageSize(), mbLineSize);
  mbSize_ = mbLineSize;
  mcm_->setCheckWholeMbLine(mbLineCheckAll);

  // For easier handling of CMOs.
  if (mcmCache)
    {
      dataCache_ = std::make_shared<TT_CACHE::Cache>();
      auto dataMemRead = [this](uint64_t addr, uint64_t& value) {
        return this->memory_->peek(addr, value, false);
      };
      auto memWrite = [this](uint64_t addr, uint64_t value) {
        return this->memory_->poke(addr, value, false);
      };
      dataCache_->addMemReadCallback(dataMemRead);
      dataCache_->addMemWriteCallback(memWrite);
    }

  typedef typename Mcm<URV>::PpoRule Rule;

  for (unsigned ix = 0; ix < Rule::Io; ++ix)   // Temporary: Disable IO rule.
    {
      Rule rule = Rule(ix);
      mcm_->enablePpo(rule, enablePpos);
    }

  for (auto& hart :  sysHarts_)
  {
    auto fetchCache = std::make_shared<TT_CACHE::Cache>();
    auto fetchMemRead = [this](uint64_t addr, uint64_t& value) {
      if (dataCache_)
        return dataCache_->read(addr, value)? true : this->memory_->peek(addr, value, false);
      return this->memory_->peek(addr, value, false);
    };
    fetchCache->addMemReadCallback(fetchMemRead);
    hart->setMcm(mcm_, fetchCache, dataCache_);
  }

  return true;
}


template <typename URV>
void
System<URV>::endMcm()
{
  if (mcm_)
    {
      const auto& path = memory_->dataLineTracePath();
      if (not path.empty())
        {
          bool skipClean = true;
          bool includeValues = true;
          memory_->saveDataAddressTrace(path, skipClean, includeValues);

          std::string emptyPath;
          memory_->enableDataLineTrace(emptyPath);  // Disable
        }
    }

  for (auto& hart :  sysHarts_)
    hart->setMcm(nullptr, nullptr, nullptr);
  mcm_ = nullptr;
}


template <typename URV>
bool
System<URV>::enablePerfApi(std::vector<FILE*>& traceFiles)
{
  if constexpr (sizeof(URV) == 4)
    {
      std::cerr << "Error: Performance model API is not supported for RV32\n";
      return false;
    }
  else
    {
      perfApi_ = std::make_shared<TT_PERF::PerfApi>(*this);
      for (auto& hart : sysHarts_)
	hart->setPerfApi(perfApi_);
      perfApi_->enableTraceLog(traceFiles);
    }

  return true;
}


template <typename URV>
void
System<URV>::enableTso(bool flag)
{
  if (mcm_)
    mcm_->enableTso(flag);
}


template <typename URV>
bool
System<URV>::mcmRead(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
		     unsigned size, uint64_t data, unsigned elemIx, unsigned field, bool cache)
{
  if (not mcm_)
    return false;
  return mcm_->readOp(hart, time, tag, addr, size, data, elemIx, field, cache);
}


template <typename URV>
bool
System<URV>::mcmMbWrite(Hart<URV>& hart, uint64_t time, uint64_t addr,
			const std::vector<uint8_t>& data,
			const std::vector<bool>& mask, bool skipCheck)
{
  if (not mcm_)
    return false;
  bool ok = dataCache_? hart.template mcmCacheInsert<McmMem::Data>(addr) : true;
  return ok and mcm_->mergeBufferWrite(hart, time, addr, data, mask, skipCheck);
}


template <typename URV>
bool
System<URV>::mcmMbInsert(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
                         unsigned size, uint64_t data, unsigned elem, unsigned field)
{
  if (not mcm_)
    return false;
  return mcm_->mergeBufferInsert(hart, time, tag, addr, size, data, elem, field);
}


template <typename URV>
bool
System<URV>::mcmBypass(Hart<URV>& hart, uint64_t time, uint64_t tag, uint64_t addr,
                       unsigned size, uint64_t data, unsigned elem, unsigned field, bool cache)
{
  if (not mcm_)
    return false;
  bool ok = (dataCache_ and cache)? hart.template mcmCacheInsert<McmMem::Data>(addr) : true;
  return ok and mcm_->bypassOp(hart, time, tag, addr, size, data, elem, field, cache);
}


template <typename URV>
bool
System<URV>::mcmIFetch(Hart<URV>& hart, uint64_t /*time*/, uint64_t addr)
{
  if (not mcm_)
    return false;
  return hart.template mcmCacheInsert<McmMem::Fetch>(addr);
}


template <typename URV>
bool
System<URV>::mcmIEvict(Hart<URV>& hart, uint64_t /*time*/, uint64_t addr)
{
  if (not mcm_)
    return false;
  return hart.template mcmCacheEvict<McmMem::Fetch>(addr);
}


template <typename URV>
bool
System<URV>::mcmDFetch(Hart<URV>& hart, uint64_t /*time*/, uint64_t addr)
{
  if (not mcm_)
    return false;
  return hart.template mcmCacheInsert<McmMem::Data>(addr);
}


template <typename URV>
bool
System<URV>::mcmDEvict(Hart<URV>& hart, uint64_t /*time*/, uint64_t addr)
{
  if (not mcm_ or not dataCache_)
    return false;
  return hart.template mcmCacheEvict<McmMem::Data>(addr);
}


template <typename URV>
bool
System<URV>::mcmDWriteback(Hart<URV>& hart, uint64_t /*time*/, uint64_t addr, const std::vector<uint8_t>& rtlData)
{
  if (not mcm_ or not dataCache_)
    return false;
  return hart.template mcmCacheWriteback<McmMem::Data>(addr, rtlData);
}


template <typename URV>
bool
System<URV>::mcmRetire(Hart<URV>& hart, uint64_t time, uint64_t tag,
		       const DecodedInst& di, bool trapped)
{
  if (not mcm_)
    return false;
  return mcm_->retire(hart, time, tag, di, trapped);
}


template <typename URV>
bool
System<URV>::mcmSkipReadDataCheck(uint64_t addr, unsigned size, bool enable)
{
  if (not mcm_)
    return false;
  mcm_->skipReadDataCheck(addr, size, enable);
  return true;
}


template <typename URV>
void
System<URV>::perfApiCommandLog(FILE* log)
{
  if (not perfApi_)
    return;
  perfApi_->enableCommandLog(log);
}


template <typename URV>
void
System<URV>::perfApiTraceLog(std::vector<FILE*>& files)
{
  if (not perfApi_)
    return;
  perfApi_->enableTraceLog(files);
}


template <typename URV>
bool
System<URV>::perfApiFetch(unsigned hart, uint64_t time, uint64_t tag, uint64_t vpc)
{
  if (not perfApi_)
    return false;

  bool trap{}; ExceptionCause cause{}; uint64_t trapPc{};
  return perfApi_->fetch(hart, time, tag, vpc, trap, cause, trapPc);
}


template <typename URV>
bool
System<URV>::perfApiDecode(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->decode(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiExecute(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->execute(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiRetire(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->retire(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiDrainStore(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->drainStore(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiPredictBranch(unsigned hart, uint64_t /*time*/, uint64_t tag,
				  bool taken, uint64_t target)
{
  if (not perfApi_)
    return false;
  return perfApi_->predictBranch(hart, tag, taken, target);
}


template <typename URV>
bool
System<URV>::perfApiFlush(unsigned hart, uint64_t time, uint64_t tag)
{
  if (not perfApi_)
    return false;
  return perfApi_->flush(hart, time, tag);
}


template <typename URV>
bool
System<URV>::perfApiShouldFlush(unsigned hart, uint64_t time, uint64_t tag, bool& flush,
				uint64_t& addr)
{
  flush = false;
  if (not perfApi_)
    return false;
  return perfApi_->shouldFlush(hart, time, tag, flush, addr);
}


template <typename URV>
bool
System<URV>::produceTestSignatureFile(std::string_view outPath) const
{
  // Find the begin_signature and end_signature section addresses
  // from the elf file
  WdRiscv::ElfSymbol beginSignature, endSignature;
  for (auto&& [symbolName, pSymbol] : { std::pair{ "begin_signature", &beginSignature },
                                        std::pair{ "end_signature",   &endSignature } })
    {
      if (not findElfSymbol(symbolName, *pSymbol))
        {
          std::cerr << "Error: Failed to find symbol " << symbolName << " in memory.\n";
          return false;
        }
    }

  if (beginSignature.addr_ > endSignature.addr_)
    {
      std::cerr << "Error: Ending address for signature file is before starting address.\n";
      return false;
    }

  // Get all of the data between the two sections and store it in a vector to ensure
  // it can all be read correctly.
  std::vector<uint32_t> data;
  data.reserve((endSignature.addr_ - beginSignature.addr_) / 4);
  for (std::size_t addr = beginSignature.addr_; addr < endSignature.addr_; addr += 4)
    {
      uint32_t value{};
      if (not memory_->peek(addr, value, true))
        {
          std::cerr << "Error: Unable to read data at address 0x" << std::hex << addr << ".\n";
          return false;
        }

      data.push_back(value);
    }

  // If all data has been read correctly, write it as 32-bit hex values with one
  // per line.
  std::ofstream outFile(outPath.data());
  outFile << std::hex << std::setfill('0');
  for (uint32_t value : data)
    {
      outFile << std::setw(8) << value << '\n';
    }

  return true;
}


template <typename URV>
bool
System<URV>::getSparseMemUsedBlocks(std::vector<std::pair<uint64_t, uint64_t>>& usedBlocks) const
{
  if (sparseMem_)
    {
      sparseMem_->getUsedBlocks(usedBlocks);
      return true;
    }
  return false;
}


extern void forceUserStop(int);


template <typename URV>
bool
System<URV>::batchRun(std::vector<util::file::SharedFile>& traceFiles, bool waitAll, uint64_t stepWinLo, uint64_t stepWinHi, bool earlyRoiTerminate)
{
  auto forceSnapshot = [this]() -> void {
      uint64_t tag = ++snapIx_;
      std::string pathStr = snapDir_ + std::to_string(tag);
      Filesystem::path path = pathStr;
      if (not Filesystem::is_directory(path) and
	  not Filesystem::create_directories(path))
        {
          std::cerr << "Error: Failed to create snapshot directory " << pathStr << '\n';
          std::cerr << "Error: Continuing...\n";
        }

      if (not saveSnapshot(pathStr))
        {
          std::cerr << "Error: Failed to save a snapshot\n";
          std::cerr << "Error: Continuining...\n";
        }
  };

  if (hartCount() == 0)
    return true;

  while (true)
    {
      struct ExitCondition {
        bool snap = false;
        bool stop = false;
        bool roi = false;

        ExitCondition() = default;

        ExitCondition(CoreException::Type type) {
          if (type == CoreException::Type::Snapshot or
              type == CoreException::Type::SnapshotAndStop)
            snap = true;
          roi = (type == CoreException::Type::RoiEntry);
          stop = (type != CoreException::Type::Snapshot) and not roi;
        };

        ExitCondition& operator|=(const ExitCondition& other) {
          snap = snap or other.snap,
          stop = stop or other.stop,
          roi = roi or other.roi;
          return *this;
        };

      } cond;

      std::atomic<bool> result = true;

      if (hartCount() == 1)
        {
          auto& hart = *ithHart(0);
          try
            {
              result = hart.run(traceFiles.at(0).get());
#if FAST_SLOPPY
              hart.reportOpenedFiles(std::cout);
#endif
              cond = ExitCondition(CoreException::Type::Exit);
            }
          catch (const CoreException& ce)
            {
              cond = ExitCondition(ce.type());
            }
        }
      else if (not stepWinLo and not stepWinHi)
        {
          // Run each hart in its own thread.
          std::vector<std::thread> threadVec;
          std::atomic<unsigned> finished = 0;  // Count of finished threads.

          auto threadFunc = [&result, &finished, &cond] (Hart<URV>* hart, FILE* traceFile) {
                              try
                                {
                                  bool r = hart->run(traceFile);
                                  result = result and r;
                                  cond |= ExitCondition(CoreException::Type::Exit);
                                }
                              catch (const CoreException& ce)
                                {
                                  cond = ExitCondition(ce.type());
                                }
                              finished++;
                            };

          for (unsigned i = 0; i < hartCount(); ++i)
            {
              Hart<URV>* hart = ithHart(i).get();
              threadVec.emplace_back(std::thread(threadFunc, hart, traceFiles.at(i).get()));
            }

          if (not waitAll)
            {
              // First thread to finish terminates run.
              while (finished == 0)
                sleep(1);
              forceUserStop(0);
            }

          for (auto& t : threadVec)
            {
              if (cond.snap or cond.roi)
                forceUserStop(0);
              t.join();
            }
        }
      else
        {
          // Run all harts in one thread round-robin.
          const uint64_t stepWindow = stepWinHi - stepWinLo + 1;
          unsigned finished = 0;
          std::vector<bool> stopped(sysHarts_.size(), false);

          for (const auto& hptr : sysHarts_)
            finished += hptr->hasTargetProgramFinished();

          while ((waitAll and finished != hartCount()) or
                 (not waitAll and finished == 0))
            {
              for (const auto& hptr : sysHarts_)
                {
                  unsigned ix = hptr->sysHartIndex();
                  if (stopped.at(ix))
                    continue;

                  // step N times
                  unsigned steps = (rand() % stepWindow) + stepWinLo;
                  try
                    {
                      bool stop{};
                      result = hptr->runSteps(steps, stop, traceFiles.at(ix).get()) and result;
                      stopped.at(ix) = stop;
                      if (stop)
                        cond |= ExitCondition(CoreException::Type::Exit);
                    }
                  catch (const CoreException& ce)
                    {
                      cond = ExitCondition(ce.type());
                      stopped.at(ix) = cond.stop or (cond.roi and earlyRoiTerminate);
                    }
                  if (stopped.at(ix))
                    finished++;
                }

              if (cond.snap or cond.roi)
                break;
            }
        }

      if (cond.snap or (cond.roi and earlyRoiTerminate))
        forceSnapshot();
      if (cond.stop or
          (cond.roi and earlyRoiTerminate))
        return result;
    }
}


/// Run producing a snapshot after each snapPeriod instructions. Each
/// snapshot goes into its own directory names <dir><n> where <dir> is
/// the string in snapDir and <n> is a sequential integer starting at
/// 0. Return true on success and false on failure.
template <typename URV>
bool
System<URV>::snapshotRun(std::vector<util::file::SharedFile>& traceFiles, const std::vector<uint64_t>& periods, bool aperiodic)
{
  if (hartCount() == 0)
    return true;

  if (periods.size() > 1 and not aperiodic)
    assert(false);

  Hart<URV>& hart0 = *ithHart(0);

  // Turns on roi-offset snapshots.
  bool hasRoi = hart0.hasRoiTraceEnabled();
  std::string origSnapDir = snapDir_;
  int roiIx = -1;

  // For the first ROI, we run until the end -- then apply a user specified limit (if specified)
  uint64_t globalLimit = hasRoi? ~uint64_t(0) : hart0.getInstructionCountLimit();
  uint64_t userLimit = hart0.getInstructionCountLimit();

  while (true)
    {
      uint64_t offset = 0;
      bool done = false; ++roiIx;
      // Fast-forward to ROI.
      if (hasRoi)
        {
          snapIx_ = -1;
          snapDir_ = origSnapDir + "-roi" + std::to_string(roiIx) + "-";

          for (auto& hartPtr : sysHarts_)
            hartPtr->setInstructionCountLimit(globalLimit);

          batchRun(traceFiles, true /*waitAll*/, 0 /*stepWindowLo*/, 0 /*stepWindowHi*/, true /*earlyRoiTerminate*/);

          offset = hart0.getInstructionCount();
          for (auto& hartPtr : sysHarts_)
            done = done or hartPtr->hasTargetProgramFinished() or (offset >= globalLimit);
          if (userLimit != ~uint64_t(0))
            globalLimit = offset + userLimit;
        }

      if (done)
        break;

      for (size_t ix = 0; ix < periods.size();)
        {
          uint64_t nextLimit = globalLimit;
          if (not periods.empty())
            {
              if (not aperiodic)
                {
                  nextLimit = hart0.getInstructionCount() + periods.at(0);
                  // early exit if we exited the ROI for periodic snaps
                  if (hasRoi)
                    {
                      bool inRoi = false;
                      for (auto& hartPtr : sysHarts_)
                        inRoi = inRoi or hartPtr->traceOn();
                      if (not inRoi)
                        break;
                    }
                }
              else
                nextLimit = offset + periods.at(ix);
            }
          nextLimit = std::min(nextLimit, globalLimit);

          uint64_t tag = 0;
          if (aperiodic)
            {
              tag = periods.at(ix);
              ++ix;
            }
          else
            tag = ++snapIx_;
          std::string pathStr = snapDir_ + std::to_string(tag);
          Filesystem::path path = pathStr;
          if (not Filesystem::is_directory(path) and
              not Filesystem::create_directories(path))
            {
              std::cerr << "Error: Failed to create snapshot directory " << pathStr << '\n';
              return false;
            }

          for (auto& hartPtr : sysHarts_)
            hartPtr->setInstructionCountLimit(nextLimit);

          batchRun(traceFiles, true /*waitAll*/, 0 /*stepWinLo*/, 0 /*stepWinHi*/);

          for (auto& hartPtr : sysHarts_)
            if (hartPtr->hasTargetProgramFinished() or nextLimit >= globalLimit)
              {
                done = true;
                Filesystem::remove_all(path);
                break;
              }
          if (done)
            break;

          if (not saveSnapshot(pathStr))
            {
              std::cerr << "Error: Failed to save a snapshot\n";
              return false;
            }
        }

      if (done)
        break;

      // Finish the run if not using ROI.
      if (not hasRoi)
        {
          for (auto& hartPtr : sysHarts_)
            hartPtr->setInstructionCountLimit(userLimit);

          batchRun(traceFiles, true /*waitAll*/, 0 /*stepWinLo*/, 0 /*stepWinHi*/);
          break;
        }
    }

  // Incremental branch traces are in snapshot directories. Turn off
  // branch tracing to prevent top-level branch tracing file from
  // being generated since the data will be for the last snapshot and
  // not for the whole run. Same is done for instruction and data line
  // tracing.
  for (const auto& hartPtr : sysHarts_)
    {
      hartPtr->traceBranches(std::string(), 0);
      std::string emptyPath;
      memory_->enableDataLineTrace(emptyPath);
      memory_->enableInstructionLineTrace(emptyPath);
    }

  return true;
}


template <typename URV>
bool
System<URV>::loadSnapshot(const std::string& snapDir, bool restoreTrace)
{
  using std::cerr;

  if (not Filesystem::is_directory(snapDir))
    {
      cerr << "Error: Path is not a snapshot directory: " << snapDir << '\n';
      return false;
    }

  if (hartCount_ == 0)
    {
      cerr << "Error: System::loadSnapshot: System with no harts\n";
      return false;
    }

  Filesystem::path dirPath = snapDir;

  // Restore the register values.
  for (auto& hartPtr : sysHarts_)
    {
      unsigned ix = hartPtr->sysHartIndex();

      std::string name = "registers" + std::to_string(ix);

      Filesystem::path regPath = dirPath / name;
      bool missing = not Filesystem::is_regular_file(regPath);
      if (missing and ix == 0 and hartCount_ == 1)
	{
	  // Support legacy snapshots where hart index was not appended to filename.
	  regPath = dirPath / "registers";
	  missing = not Filesystem::is_regular_file(regPath);
	}
      if (missing)
	{
	  cerr << "Error: Snapshot file does not exists: " << regPath << '\n';
	  return false;
	}

      if (not hartPtr->loadSnapshotRegs(regPath.string()))
	return false;
    }


  Filesystem::path usedBlocksPath = dirPath / "usedblocks";
  std::vector<std::pair<uint64_t,uint64_t>> usedBlocks;
  if (not loadUsedMemBlocks(usedBlocksPath.string(), usedBlocks))
    return false;

  auto& hart0 = *ithHart(0);

  Filesystem::path timePath = dirPath / "time";
  if (not loadTime(timePath.string(), time_))
    {
      std::cerr << "Error: Using instruction count for time\n";
      time_ = hart0.getInstructionCount();  // Legacy snapshots.
    }

  auto& syscall = hart0.getSyscall();
  Filesystem::path mmapPath = dirPath / "mmap";
  if (not syscall.loadMmap(mmapPath.string()))
    return false;

  if (restoreTrace)
    {
      Filesystem::path dtracePath = dirPath / "data-lines";
      if (not memory_->loadDataAddressTrace(dtracePath))
        return false;

      Filesystem::path itracePath = dirPath / "instr-lines";
      if (not memory_->loadInstructionAddressTrace(itracePath))
        return false;

      Filesystem::path branchPath = dirPath / "branch-trace";
      if (not hart0.loadBranchTrace(branchPath))
        return false;

      Filesystem::path cachePath = dirPath / "cache-trace";
      if (not hart0.loadCacheTrace(cachePath))
        return false;
    }

  Filesystem::path memPath = dirPath / "memory";
  if (snapDecompressionType_  == "lz4")
    {
#if LZ4_COMPRESS
      if (not memory_->loadSnapshot_lz4(memPath.string(), usedBlocks))
        return false;
#else
      std::cerr << "Error: LZ4 compression is not enabled\n";
      return false;
#endif
    }
  else if (snapDecompressionType_ == "gzip")
    {
      if (not memory_->loadSnapshot_gzip(memPath.string(), usedBlocks))
        return false;
    }
  else
    {
      std::cerr << "Error: Invalid decompression type: " << snapDecompressionType_ << '\n';
      return false;
    }

  // Rearm CLINT time compare.
  Filesystem::path mtimecmpPath = dirPath / "mtimecmp";
  {
    std::ifstream ifs(mtimecmpPath);
    if (not ifs.good())
      {
        for (auto& hartPtr : sysHarts_)
          {
            uint64_t mtimeCmpBase = 0;
            if (hartPtr->hasAclintTimeCompare(mtimeCmpBase))
              {
                uint64_t timeCmpAddr = mtimeCmpBase + hartPtr->sysHartIndex() * 8;
                uint64_t timeCmp = 0;
                memory_->peek(timeCmpAddr, timeCmp, false);
                hartPtr->setAclintAlarm(timeCmp);
              }
          }
      }
    else
      {
        std::string line;
        unsigned i = 0;
        while (std::getline(ifs, line) and i < sysHarts_.size())
          {
            uint64_t timeCmp = std::strtoull(line.c_str(), nullptr, 16);
            sysHarts_.at(i++)->setAclintAlarm(timeCmp);
          }
      }
  }

  Filesystem::path fdPath = dirPath / "fd";
  if (not syscall.loadFileDescriptors(fdPath.string()))
    return false;

  Filesystem::path imsicPath = dirPath / "imsic";
  if (not imsicMgr_.loadSnapshot(imsicPath))
    return false;

  if (not loadAplicSnapshot(dirPath))
    return false;

  std::set<std::string_view> ioDevTypes;
  for (auto& dev : ioDevs_)
    {
      if (ioDevTypes.contains(dev->type()))
        {
          std::cerr << "Error: currently cannot load snapshots for multiple devices of the same type, " <<  dev->type() << '\n';
          return false;
        }
      ioDevTypes.insert(dev->type());
      Filesystem::path devPath = dirPath / dev->type();
      if (not dev->loadSnapshot(devPath))
        return false;
    }

  return true;
}


template <typename URV>
bool
System<URV>::saveAplicSnapshot(const Filesystem::path& snapDir) const
{
  if (not aplic_)
    return true;

  auto filepath = snapDir / "aplic-source-states";
  std::ofstream ofs(filepath);
  if (not ofs)
    {
      std::cerr << "Error: failed to open snapshot file for writing: " << filepath << "\n";
      return false;
    }
  unsigned nsources = aplic_->numSources();
  for (unsigned i = 1; i <= nsources; i++)
    {
      bool state = aplic_->getSourceState(i);
      if (state)
        ofs << i << " " << state << "\n";
    }

  auto domainsPath = snapDir / "aplic-domains";
  if (not Filesystem::is_directory(domainsPath) and
      not Filesystem::create_directories(domainsPath))
    {
      std::cerr << "Error: failed to create subdirectory for snapshots of APLIC domains: " << domainsPath << "\n";
      return false;
    }
  return saveAplicDomainSnapshot(domainsPath, aplic_->root(), nsources);
}


template <typename URV>
bool
System<URV>::saveAplicDomainSnapshot(const Filesystem::path& snapDir,
                                     const std::shared_ptr<TT_APLIC::Domain>& domain,
                                     unsigned nsources) const
{
  auto filepath = snapDir / domain->name();
  std::ofstream ofs(filepath);
  if (not ofs)
    {
      std::cerr << "Error: failed to open snapshot file for writing: " << filepath << "\n";
      return false;
    }
  ofs << std::hex;

  uint32_t domaincfg = domain->peekDomaincfg();
  if (domaincfg)
    ofs << "domaincfg 0x" << domaincfg << "\n";

  for (unsigned i = 1; i <= nsources; i++)
    {
      uint32_t sourcecfg = domain->peekSourcecfg(i);
      if (sourcecfg != 0)
        ofs << "sourcecfg " << std::to_string(i) << " 0x" << sourcecfg << "\n";
    }

  for (unsigned i = 1; i <= nsources; i++)
    {
      uint32_t target = domain->peekTarget(i);
      if (target != 0)
        ofs << "target " << std::to_string(i) << " 0x" << target << "\n";
    }

  for (unsigned i = 0; i < nsources/32; i++)
    {
      uint32_t setip = domain->peekSetip(i);
      for (unsigned j = 0; j < 32; j++)
        {
          if ((setip >> j) & 1)
            ofs << "setipnum 0x" << i*32 + j << "\n";
        }
    }

  for (unsigned i = 0; i < nsources/32; i++)
    {
      uint32_t setie = domain->peekSetie(i);
      for (unsigned j = 0; j < 32; j++)
        {
          if ((setie >> j) & 1)
            ofs << "setienum 0x" << i*32 + j << "\n";
        }
    }

  uint32_t genmsi = domain->peekGenmsi();
  if (genmsi)
    ofs << "genmsi 0x" << genmsi << "\n";

  if (domain->parent() == nullptr)
    {
      uint32_t mmsiaddrcfg  = domain->peekMmsiaddrcfg();
      uint32_t mmsiaddrcfgh = domain->peekMmsiaddrcfgh();
      uint32_t smsiaddrcfg  = domain->peekSmsiaddrcfg();
      uint32_t smsiaddrcfgh = domain->peekSmsiaddrcfgh();
      if (mmsiaddrcfg)
        ofs << "mmsiaddrcfg 0x"  << mmsiaddrcfg << "\n";
      if (mmsiaddrcfgh)
        ofs << "mmsiaddrcfgh 0x" << mmsiaddrcfgh << "\n";
      if (smsiaddrcfg)
        ofs << "smsiaddrcfg 0x"  << smsiaddrcfg << "\n";
      if (smsiaddrcfgh)
        ofs << "smsiaddrcfgh 0x" << smsiaddrcfgh << "\n";
    }

  for (auto hartIndex : domain->hartIndices())
    {
      bool xeipBit = domain->peekXeip(hartIndex);
      if (xeipBit)
        ofs << "xeip " << std::dec << hartIndex << std::hex << " " << xeipBit << "\n";
    }

  for (auto hartIndex : domain->hartIndices())
    {
      uint32_t idelivery = domain->peekIdelivery(hartIndex);
      uint32_t iforce = domain->peekIforce(hartIndex);
      uint32_t ithreshold = domain->peekIthreshold(hartIndex);
      uint32_t topi = domain->peekTopi(hartIndex);
      if (idelivery)
        ofs << "idelivery " << std::dec << hartIndex << std::hex << " " << idelivery << "\n";
      if (iforce)
        ofs << "iforce " << std::dec << hartIndex << std::hex << " " << iforce << "\n";
      if (ithreshold)
        ofs << "ithreshold " << std::dec << hartIndex << std::hex << " 0x" << ithreshold << "\n";
      if (topi)
        ofs << "topi " << std::dec << hartIndex << std::hex << " 0x" << topi << "\n";
    }

  // NOLINTNEXTLINE(readability-use-anyofallof)
  for (auto& child : *domain)
    {
      if (not saveAplicDomainSnapshot(snapDir, child, nsources))
        return false;
    }
  return true;
}


static std::string
stripComment(const std::string& line)
{
    size_t pos = line.find('#');
    if (pos != std::string::npos)
        return line.substr(0, pos);
    return line;
}


template <typename URV>
bool
System<URV>::loadAplicSnapshot(const Filesystem::path& snapDir)
{
  if (not aplic_)
    return true;
  auto filepath = snapDir / "aplic-source-states";
  std::ifstream ifs(filepath);
  if (not ifs)
    std::cerr << "Warning: failed to open snapshot file " << filepath << "\n";

  unsigned nsources = aplic_->numSources();
  std::string line;
  int lineno = 0;
  while (std::getline(ifs, line))
    {
      lineno++;
      std::string data = stripComment(line);
      boost::algorithm::trim(data);
      if (data.empty())
        continue;
      std::istringstream iss(data);
      int state = 0;
      unsigned source_id = 0;
      if (not (iss >> source_id >> state))
        {
          std::cerr << "Error: failed to parse APLIC snapshot file " << filepath << " line " << lineno << ": \n" << line << '\n';
          return false;
        }
      std::string dummy;
      if (iss >> dummy)
        {
          std::cerr << "Error: failed to parse APLIC snapshot file " << filepath << " line " << lineno << ": "
                    << "unexpected tokens\n";
          return false;
        }
      if (source_id < 1 or source_id > nsources)
        {
          std::cerr << "Error: failed to parse APLIC snapshot file " << filepath << " line " << lineno << ": "
                    << source_id << " is not a valid source id\n";
          return false;
        }
      if (state != 0 and state != 1)
        {
          std::cerr << "Error: failed to parse APLIC snapshot file " << filepath << " line " << lineno << ": "
                    << state << " is not a valid source state\n";
          return false;
        }
      aplic_->setSourceState(source_id, state);
    }

  return loadAplicDomainSnapshot(snapDir / "aplic-domains", aplic_->root(), nsources);
}


enum class AplicRegister {
  DOMAINCFG,
  SOURCECFG,
  TARGET,
  SETIPNUM,
  SETIENUM,
  GENMSI,
  MMSIADDRCFG,
  MMSIADDRCFGH,
  SMSIADDRCFG,
  SMSIADDRCFGH,
  IDELIVERY,
  IFORCE,
  ITHRESHOLD,
  TOPI,
  XEIP,
};

static bool
parseAplicRegisterName(const std::string& name, AplicRegister& reg, bool& hasSourceId, bool& hasHartIndex)
{
  hasSourceId = false;
  hasHartIndex = false;

  using AR = AplicRegister;
  if      (name == "domaincfg")     { reg = AR::DOMAINCFG;                          }
  else if (name == "sourcecfg")     { reg = AR::SOURCECFG;    hasSourceId = true;   }
  else if (name == "target")        { reg = AR::TARGET;       hasSourceId = true;   }
  else if (name == "setipnum")      { reg = AR::SETIPNUM;                           }
  else if (name == "setienum")      { reg = AR::SETIENUM;                           }
  else if (name == "genmsi")        { reg = AR::GENMSI;                             }
  else if (name == "mmsiaddrcfg")   { reg = AR::MMSIADDRCFG;                        }
  else if (name == "mmsiaddrcfgh")  { reg = AR::MMSIADDRCFGH;                       }
  else if (name == "smsiaddrcfg")   { reg = AR::SMSIADDRCFG;                        }
  else if (name == "smsiaddrcfgh")  { reg = AR::SMSIADDRCFGH;                       }
  else if (name == "idelivery")     { reg = AR::IDELIVERY;    hasHartIndex = true;  }
  else if (name == "iforce")        { reg = AR::IFORCE;       hasHartIndex = true;  }
  else if (name == "ithreshold")    { reg = AR::ITHRESHOLD;   hasHartIndex = true;  }
  else if (name == "topi")          { reg = AR::TOPI;         hasHartIndex = true;  }
  else if (name == "xeip")          { reg = AR::XEIP;         hasHartIndex = true;  }
  else return false;

  return true;
}


template <typename URV>
bool
System<URV>::loadAplicDomainSnapshot(const Filesystem::path& snapDir,
                                     const std::shared_ptr<TT_APLIC::Domain>& domain,
                                     unsigned nsources)
{
  auto filepath = snapDir / domain->name();
  std::ifstream ifs(filepath);
  if (not ifs)
    std::cerr << "Warning: failed to open snapshot file " << filepath << "\n";

  std::string line;
  int lineno = 0;
  while (std::getline(ifs, line))
    {
      lineno++;
      std::string data = stripComment(line);
      boost::algorithm::trim(data);
      if (data.empty())
        continue;
      std::istringstream iss(data);
      iss >> std::hex;
      std::string name;
      uint32_t value = 0;
      if (not (iss >> name))
        {
          std::cerr << "Error: failed to parse domain snapshot file " << filepath << " line " << lineno << ": \n" << line << '\n';
          return false;
        }
      AplicRegister reg{};
      bool hasSourceId = false;
      bool hasHartIndex = false;
      if (not parseAplicRegisterName(name, reg, hasSourceId, hasHartIndex))
        {
          std::cerr << "Error: failed to parse domain snapshot file " << filepath << " line " << lineno << ": '"
                    << name << "' is not a valid APLIC register name\n";
          return false;
        }
      unsigned sourceId = 0;
      if (hasSourceId)
        {
          iss >> std::dec;
          if (not (iss >> sourceId))
            {
              std::cerr << "Error: failed to parse domain snapshot file " << filepath << " line " << lineno << ": \n" << line << '\n';
              return false;
            }
          iss >> std::hex;
          if (sourceId < 1 && sourceId > nsources)
            {
              std::cerr << "Error: invalid source id in domain snapshot file " << filepath << " line " << lineno << ": \n" << line << '\n';
              return false;
            }
        }
      unsigned hartIndex = 0;
      if (hasHartIndex)
        {
          iss >> std::dec;
          if (not (iss >> hartIndex))
            {
              std::cerr << "Error: failed to parse domain snapshot file " << filepath << " line " << lineno << ": \n" << line << '\n';
              return false;
            }
          iss >> std::hex;
        }
      if (not (iss >> value))
        {
          std::cerr << "Error: failed to parse domain snapshot file " << filepath << " line " << lineno << ": \n" << line << '\n';
          return false;
        }
      std::string dummy;
      if (iss >> dummy)
        {
          std::cerr << "Error: failed to parse domain snapshot file " << filepath << " line " << lineno << ": "
                    << "unexpected tokens\n";
          return false;
        }

      using AR = AplicRegister;
      switch (reg)
        {
          case AR::DOMAINCFG:     domain->pokeDomaincfg(value); break;
          case AR::SOURCECFG:     domain->pokeSourcecfg(sourceId, value); break;
          case AR::TARGET:        domain->pokeTarget(sourceId, value); break;
          case AR::SETIPNUM:      domain->pokeSetipnum(value); break;
          case AR::SETIENUM:      domain->pokeSetienum(value); break;
          case AR::GENMSI:        domain->pokeGenmsi(value); break;
          case AR::MMSIADDRCFG:   domain->pokeMmsiaddrcfg(value); break;
          case AR::MMSIADDRCFGH:  domain->pokeMmsiaddrcfgh(value); break;
          case AR::SMSIADDRCFG:   domain->pokeSmsiaddrcfg(value); break;
          case AR::SMSIADDRCFGH:  domain->pokeSmsiaddrcfgh(value); break;
          case AR::IDELIVERY:     domain->pokeIdelivery(hartIndex, value); break;
          case AR::IFORCE:        domain->pokeIforce(hartIndex, value); break;
          case AR::ITHRESHOLD:    domain->pokeIthreshold(hartIndex, value); break;
          case AR::TOPI:          domain->pokeTopi(hartIndex, value); break;
          case AR::XEIP:          domain->pokeXeip(hartIndex, value); break;
          default: assert(false);
        }
    }

  // NOLINTNEXTLINE(readability-use-anyofallof)
  for (auto& child : *domain)
    {
      if (not loadAplicDomainSnapshot(snapDir, child, nsources))
        return false;
    }
  return true;
}


template class WdRiscv::System<uint32_t>;
template class WdRiscv::System<uint64_t>;
