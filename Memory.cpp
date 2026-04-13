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
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <elfio/elfio.hpp>
#include <zlib.h>
#if LZ4_COMPRESS
#include <lz4frame.h>
#endif
#include "Memory.hpp"
#include "wideint.hpp"
#include <iomanip>

using namespace WdRiscv;


inline bool
isPowerOf2(uint64_t x)
{
  return x != 0 and (x & (x-1)) == 0;
}


Memory::Memory(uint64_t size, uint64_t pageSize)
  : size_(size), data_(nullptr), pageSize_(pageSize), reservations_(1),
    lastWriteData_(1), pmaMgr_(size)
{
  assert(size >= pageSize);
  assert(pageSize >= 64);

  assert(isPowerOf2(pageSize));

  pageShift_ = static_cast<unsigned>(std::log2(pageSize_));

#ifndef MEM_CALLBACKS

  errno = 0;
  void* mem = mmap(nullptr, size_, PROT_READ | PROT_WRITE,
		   MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  if (errno != 0)
    {
      std::cerr << "Error: Failed to map " << size_ << " bytes using mmap.\n";
      throw std::runtime_error("Out of memory");
    }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  data_ = reinterpret_cast<uint8_t*>(mem);

#endif

}


Memory::~Memory()
{
  if (data_)
    {
      munmap(data_, size_);
      data_ = nullptr;
    }

  if (not dataLineFile_.empty())
    saveDataAddressTrace(dataLineFile_);

  if (not instrLineFile_.empty())
    saveInstructionAddressTrace(instrLineFile_);
}


bool
Memory::loadHexFile(const std::string& fileName)
{
  std::ifstream input(fileName);

  if (not input.good())
    {
      std::cerr << "Error: Failed to open hex-file '" << fileName << "' for input\n";
      return false;
    }

  size_t addr = 0, errors = 0, unmappedCount = 0;
  size_t oob = 0; // Out of bounds addresses

  std::string line;

  for (unsigned lineNum = 0; std::getline(input, line); ++lineNum)
    {
      boost::algorithm::trim(line);
      if (line.empty())
	continue;

      if (line[0] == '@')
	{
	  if (line.size() == 1)
	    {
	      std::cerr << "Error: File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid hexadecimal address: " << line << '\n';
	      errors++;
	      continue;
	    }
	  char* end = nullptr;
	  addr = std::strtoull(&line.at(1), &end, 16);
	  if (end and *end and not isspace(*end))
	    {
	      std::cerr << "Error: File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid hexadecimal address: " << line << '\n';
	      errors++;
	    }
	  continue;
	}

      std::istringstream iss(line);
      uint32_t value = 0;
      while (iss)
	{
	  iss >> std::hex >> value;
	  if (iss.fail())
	    {
	      std::cerr << "Error: File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid data: " << line << '\n';
	      errors++;
	      break;
	    }
	  if (value > 0xff)
	    {
	      std::cerr << "Error: File " << fileName << ", Line " << lineNum << ": "
			<< "Invalid value: " << std::hex << value << '\n'
			<< std::dec;
	      errors++;
	    }
	  if (addr < size_)
	    {
	      if (not errors)
		{
                  if (not initializeByte(addr, value & 0xff))
                    {
                      if (unmappedCount == 0)
                        std::cerr << "Error: Failed to copy HEX file byte at address 0x"
                                  << std::hex << addr << std::dec
                                  << ": corresponding location is not mapped\n";
                      unmappedCount++;
                      if (checkUnmappedElf_)
                        return false;
                    }
                  addr++;
		}
	    }
	  else
	    {
              if (not oob)
                std::cerr << "Error: File " << fileName << ", Line " << lineNum << ": "
                          << " Address out of bounds: "
                          << std::hex << addr << '\n' << std::dec;
	      oob++;
	    }
	  if (iss.eof())
	    break;
	}

      if (iss.bad())
	{
	  std::cerr << "Error: File " << fileName << ", Line " << lineNum << ": "
		    << "Failed to parse data line: " << line << '\n';
	  errors++;
	}
    }

  if (oob > 1)
    std::cerr << "Error: File " << fileName << ":  File contained "
              << oob << " out of bounds addresses.\n";

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  return errors == 0;
}


#if 0
bool
Memory::loadBinaryFile(const std::string& fileName, uint64_t addr)
{
  std::ifstream input(fileName, std::ios::binary);

  if (not input.good())
    {
      std::cerr << "Error: Failed to open binary file '" << fileName << "' for input\n";
      return false;
    }

  // unmapped and out of bounds addresses
  size_t unmappedCount = 0, oob = 0, num = 0;

  char b = 0;
  while (input.get(b))
    {
      if (addr < size_)
        {
          if (not initializeByte(addr, b))
            {
              if (unmappedCount == 0)
                std::cerr << "Error: Failed to copy binary file byte at address 0x"
                          << std::hex << addr << std::dec
                          << ": corresponding location is not mapped\n";
              unmappedCount++;
              if (checkUnmappedElf_)
                return false;
            }
          addr++;
        }
      else
        {
          if (not oob)
            std::cerr << "Error: File " << fileName << ", Byte " << num << ": "
                      << " Address out of bounds: "
                      << std::hex << addr << '\n' << std::dec;
          oob++;
        }
      num++;
    };


  if (oob > 1)
    std::cerr << "Error: File " << fileName << ":  File contained "
              << oob << " out of bounds addresses.\n";

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  return true;
}
#else
bool
Memory::loadBinaryFile(const std::string& fileName, uint64_t addr)
{
  int fd = open(fileName.c_str(), O_RDONLY);
  if (fd < 0)
    {
      std::cerr << "Error: Failed to open binary file '" << fileName << "' for input\n";
      return false;
    }

  struct stat st = {};
  if (fstat(fd, &st) < 0)
    {
      std::cerr << "Error: Failed to stat binary file '" << fileName << "'\n";
      close(fd);
      return false;
    }
  auto fileSize = static_cast<uint64_t>(st.st_size);

  // Only map as many bytes as fit within the simulated memory.
  uint64_t usable = (addr < size_) ? std::min(fileSize, size_ - addr) : 0;
  uint64_t oob = fileSize - usable;

  if (oob)
    {
      std::cerr << "Error: File " << fileName << ", Byte " << usable << ": "
                << " Address out of bounds: "
                << std::hex << (addr + usable) << '\n' << std::dec;
      if (oob > 1)
        std::cerr << "Error: File " << fileName << ":  File contained "
                  << oob << " out of bounds addresses.\n";
    }

  if (usable == 0)
    {
      close(fd);
      return true;
    }

  auto* mapped = static_cast<uint8_t*>(mmap(nullptr, usable, PROT_READ, MAP_PRIVATE, fd, 0));
  close(fd);
  if (mapped == static_cast<uint8_t*>(MAP_FAILED))
    {
      std::cerr << "Error: Failed to mmap binary file '" << fileName << "'\n";
      return false;
    }

  std::span<uint8_t> mappedSpan(mapped, usable);
  size_t unmappedCount = 0;

  for (size_t n = 0; n < usable; ++n)
    {
      // Optimization: write a full page at once for page-aligned regular memory.
      size_t remaining = usable - n;
      if (isPageAligned(addr) and remaining >= pageSize_ and addr + pageSize_ - 1 < size_ and
          not pmaMgr_.overlapsMemMappedRegs(addr, addr + pageSize_ - 1))
        {
          auto page = mappedSpan.subspan(n, pageSize_);
          bool allZero = page[0] == 0 && memcmp(page.data(), &page[1], pageSize_ - 1) == 0;
          if (not allZero)
            if (not initializePage(addr, page))
              assert(0 && "Error: initializePage failed");
          addr += pageSize_;
          n += pageSize_ - 1;  // loop will add 1 more
          continue;
        }

      if (not initializeByte(addr, mappedSpan[n]))
        {
          if (unmappedCount == 0)
            std::cerr << "Error: Failed to copy binary file byte at address 0x"
                      << std::hex << addr << std::dec
                      << ": corresponding location is not mapped\n";
          unmappedCount++;
          if (checkUnmappedElf_)
            {
              munmap(mappedSpan.data(), usable);
              return false;
            }
        }
      addr++;
    }

  munmap(mappedSpan.data(), usable);

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  return true;
}
#endif


void
Memory::loadFile(const std::string& filename, std::vector<uint8_t>& data)
{

  std::ifstream f(filename, std::ios::binary);
  if (f.fail())
    throw std::runtime_error("Failed to load LZ4 file");

  f.seekg(0, std::ios::end);
  std::streampos length = f.tellg();
  f.seekg(0, std::ios::beg);

  data.clear();
  data.resize(length);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  f.read(reinterpret_cast<char*>(data.data()), length);
}

#define BLOCK_SIZE (4*1024*1024)

#if LZ4_COMPRESS
bool
Memory::loadLz4File(const std::string& fileName, uint64_t addr)
{
  using std::cerr;

  LZ4F_dctx *dctx = nullptr;
  LZ4F_errorCode_t ret = LZ4F_createDecompressionContext(&dctx, LZ4F_VERSION);
  if (LZ4F_isError(ret))
    throw std::runtime_error("Couldn't initialize LZ4 context");

  std::vector<uint8_t> src;
  loadFile(fileName, src);
  auto src_size = src.size();
  size_t src_offset = 0;

  size_t dst_size = BLOCK_SIZE;
  std::vector<uint8_t> dst(dst_size);

  size_t unmappedCount = 0, num = 0;  // Unmapped addresses, byte in file.

  while (src_size)
    {
      size_t src_bytes_read = src_size;
      size_t dst_bytes_written = dst_size;

      size_t ret = LZ4F_decompress(dctx, dst.data(), &dst_bytes_written,
                                   &src.at(src_offset), &src_bytes_read, NULL);
      if (LZ4F_isError(ret))
        throw std::runtime_error("LZ4F_decompress failed");

      for (size_t n = 0; n < dst_bytes_written; ++n, ++addr, ++num)
	{
          size_t remaining = dst_bytes_written - n;
          if (isPageAligned(addr) and remaining >= pageSize_ and addr < size_ and
              addr + pageSize_ - 1 < size_)
            {
              // Optimization: If page is regular memory, write it in one shot.

              Pma pma;
              if (not pmaMgr_.overlapsMemMappedRegs(addr, addr + pageSize_ - 1))
                {
                  uint8_t* data = &dst.at(n);
                  bool allZero = *data == 0 && memcmp(data, data + 1, pageSize_ - 1) == 0;
                  if (not allZero)
                    if (not initializePage(addr, std::span(data, pageSize_)))
                      assert(0 && "Error: Assertion failed");
                  addr += pageSize_ - 1;
                  n += pageSize_ - 1;
                  num += pageSize_ - 1;
                  continue;
                }
            }

	  if (addr < size_)
	    {
	      // Speed things up by not initalizing zero bytes
              uint8_t b = dst.at(n);
	      if (b and not initializeByte(addr, b))
		{
		  if (unmappedCount == 0)
		    cerr << "Error: File " << fileName << ", Byte " << num << ": "
                         << " Address is not mapped: "
                         << std::hex << addr << std::dec << '\n';
		  unmappedCount++;
		  if (checkUnmappedElf_)
		    return false;
		}
	    }
	  else
	    {
              cerr << "Error: File " << fileName << ", Byte " << num << ": "
                   << " Address out of bounds: "
                   << std::hex << addr << std::dec << '\n';
              break;
	    }
	}

      src_offset = src_offset + src_bytes_read;
      src_size = src_size - src_bytes_read;
    }

  return true;
}
#endif

bool
Memory::loadElfSegment(ELFIO::elfio& reader, int segIx, uint64_t& end)
{
  const ELFIO::segment* seg = reader.segments[segIx];
  ELFIO::Elf64_Addr paddr = seg->get_physical_address();
  ELFIO::Elf_Xword segSize = seg->get_file_size(); // Size in file.
  end = 0;
  if (seg->get_type() != PT_LOAD)
    return true;

  if (paddr + seg->get_memory_size() > size_)
    {
      std::cerr << "Error: End of ELF segment " << segIx << " (0x"
                << std::hex << (paddr+segSize)
                << ") is beyond end of simulated memory (0x"
                << size_ << ")\n" << std::dec;
      if (checkUnmappedElf_)
        return false;
    }

  size_t unmappedCount = 0;

  // Load segment directly.
  std::span segData(seg->get_data(), segSize);
  for (size_t i = 0; i < segData.size(); ++i)
    {
      if (not initializeByte(paddr + i, segData[i]))
        {
          if (unmappedCount == 0)
            std::cerr << "Error: Failed to copy ELF byte at address 0x"
                      << std::hex << (paddr + i) << std::dec
                      << ": corresponding location is not mapped\n";
          unmappedCount++;
          if (checkUnmappedElf_)
            return false;
        }
    }

  end = paddr + uint64_t(seg->get_memory_size());
  return true;
}


/// Extract an unsigned little-endian length encoded 128-bit value from given
/// stream.  Return true on success and afalse on failure.
/// See: https://en.wikipedia.org/wiki/LEB128
static
bool
extractUleb128(std::istream& in, Uint128& value)
{
  value = 0;
  char ch = 0;
  unsigned shift = 0;
  unsigned count = 0;

  while (in.read(&ch, 1) and count < 19)
    {
      auto byte = std::bit_cast<uint8_t>(ch);
      uint8_t msb = byte >> 7;  // Most sig bit
      byte = (byte << 1) >> 1;  // Clear most sig bit
      value = value | (Uint128(byte) << shift);
      shift += 8;
      count++;
      if (not msb)
        return true;
    }

  return false;
}


bool
Memory::collectElfRiscvTags(const std::string& fileName,
                            std::vector<std::string>& tags)
{
  // NOLINTNEXTLINE(clang-analyzer-optin.cplusplus.VirtualCall)
  ELFIO::elfio reader;

  if (not reader.load(fileName))
    {
      std::cerr << "Error: Failed to load ELF file " << fileName << '\n';
      return false;
    }

  for (const auto* sec : reader.sections)
    {
      if (sec->get_type() != 0x70000003)
        continue;

      const char* secData = sec->get_data();
      size_t size = sec->get_size();
      if (not secData or not size)
        continue;

      // 1st char is format verion. Currently supported version is 'A'.
      std::string dataString(secData, size);
      std::istringstream iss(dataString);
      char version = 0;
      iss.read(&version, 1);
      if (not iss or version != 'A')
        {
          std::cerr << "Error: Unknown ELF RISCV section format: '" << version << "'\n";
          return false;
        }

      // Next is a 4-byte section length.
      uint32_t secLen = 0;
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
      iss.read((char*) &secLen, sizeof(secLen));

      // Next is a null terminated string containing vendor name.
      std::string vendorName;
      std::getline(iss, vendorName, '\0');

      // Next is tag: file (1), section(2) or symbol(3).
      char tag = 0;
      iss.read((char*) &tag, sizeof(tag));
      if (not iss or tag != 1)
        {
          std::cerr << "Error: Unexpected ELF RISCV section tag: " << unsigned(tag) << "(expecting 1)\n";
          return false;
        }

      // Next is a 4-byte attributes size including tag and size.
      // https://embarc.org/man-pages/as/RISC_002dV_002dATTRIBUTE.html#RISC_002dV_002dATTRIBUTE
      uint32_t attribsSize = 0;
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
      iss.read((char*) &attribsSize, sizeof(attribsSize));
      if (not iss)
        {
          std::cerr << "Error: Corrupted ELF RISCV file attributes subsection\n";
          return false;
        }

      if (attribsSize == 0)
        continue;

      if (attribsSize <= sizeof(tag) + sizeof(attribsSize))
        {
          std::cerr << "Error: Corrupted ELF RISCV file attributes subsection: Invalid size\n";
          return true;
        }

      attribsSize -= (sizeof(tag) + sizeof(attribsSize));

      auto attribsStart = iss.tellg();

      while (iss and (iss.tellg() - attribsStart < attribsSize))
        {
          // Next is a unsigned lengh-encoded binary 128 tag.
          Uint128 tag = 0;
          if (not extractUleb128(iss, tag))
            {
              std::cerr << "Error: Empty/corrupted ELF RISCV file attributes subsection: Invalid tag\n";
              return false;
            }

          // If tag is even, value is another uleb128. If odd, value
          // is a null-terminated string.
          if ((tag & 1) == 0)
            {
              Uint128 value = 0;
              if (not extractUleb128(iss, value))
                {
                  std::cerr << "Error: Empty/corrupted ELF RISCV file attributes subsection: Invalid tag value\n";
                  return false;
                }
            }
          else
            {
              std::string value;
              std::getline(iss, value, '\0');
              if (not iss)
                {
                  std::cerr << "Error: Corrupted ELF RISCV file attributes subsection: Missing architeture tag string\n";
                  return false;
                }
              if (tag == 5)
                tags.push_back(value);
              return true;
            }
        }
    }

  return true;
}


void
Memory::collectElfSymbols(ELFIO::elfio& reader)
{
  for (const auto& sec : reader.sections)
    {
      if (sec->get_type() != SHT_SYMTAB)
	continue;

      const ELFIO::symbol_section_accessor symAccesor(reader, sec);
      ELFIO::Elf64_Addr address = 0;
      ELFIO::Elf_Xword size = 0;
      unsigned char bind = 0, type = 0, other = 0;
      ELFIO::Elf_Half index = 0;

      // Finding symbol by name does not work. Walk all the symbols.
      ELFIO::Elf_Xword symCount = symAccesor.get_symbols_num();
      for (ELFIO::Elf_Xword symIx = 0; symIx < symCount; ++symIx)
	{
	  std::string name;
	  if (symAccesor.get_symbol(symIx, name, address, size, bind, type,
				    index, other))
	    {
	      if (name.empty())
		continue;

    if (type == STT_NOTYPE || type == STT_FUNC || type == STT_OBJECT) {
      symbols_[name] = ElfSymbol(address, size);
      addrToSymName_[address] = name;
    }
	    }
	}
    }
}


void
Memory::collectElfSections(ELFIO::elfio& reader)
{
  auto secCount = reader.sections.size();

  for (int secIx = 0; secIx < secCount; ++secIx)
    {
      auto* sec = reader.sections[secIx];
      sections_[sec->get_name()] = ElfSymbol(sec->get_address(), sec->get_size());
    }
}



bool
Memory::loadElfFile(const std::string& fileName, unsigned regWidth,
		    uint64_t& entryPoint, uint64_t& end)
{
  entryPoint = 0;
  end = 0;

  ELFIO::elfio reader;

  if (regWidth != 32 and regWidth != 64)
    {
      std::cerr << "Error: Memory::loadElfFile called with a unsupported "
		<< "register width: " << regWidth << '\n';
      return false;
    }

  if (not reader.load(fileName))
    {
      std::cerr << "Error: Failed to load ELF file " << fileName << '\n';
      return false;
    }

  bool is32 = reader.get_class() == ELFCLASS32;
  bool is64 = reader.get_class() == ELFCLASS64;
  if (not (is32 or is64))
    {
      std::cerr << "Error: ELF file is neither 32 nor 64-bit. Only 32/64-bit ELFs are currently supported\n";
      return false;
    }

  if (regWidth == 32 and not is32)
    {
      if (is64)
	std::cerr << "Error: Loading a 64-bit ELF file in 32-bit mode.\n";
      else
	std::cerr << "Error: Loading non-32-bit ELF file in 32-bit mode.\n";
      return false;
    }

  if (regWidth == 64 and not is64)
    {
      std::cerr << "Error: Loading non-64-bit ELF file in 64-bit mode.\n";
      return false;
    }

  if (reader.get_machine() != EM_RISCV)
    {
      std::cerr << "Error: non-riscv ELF file\n";
      return false;
    }

  // Copy loadable ELF segments into memory.
  uint64_t maxEnd = 0;  // Largest end address of a segment.
  unsigned errors = 0;

  for (int segIx = 0; segIx < reader.segments.size(); ++segIx)
    {
      uint64_t end = 0;
      if (loadElfSegment(reader, segIx, end))
        maxEnd = std::max(end, maxEnd);
      else
        errors++;
    }

  if (maxEnd == 0)
    {
      std::cerr << "Error: No loadable segment in ELF file\n";
      errors++;
    }

  // In case writing ELF data modified last-written-data associated
  // with each hart.
  for (unsigned hartId = 0; hartId < reservations_.size(); ++hartId)
    clearLastWriteInfo(hartId);

  // Collect symbols.
  collectElfSymbols(reader);

  // Collect address/size of sections.
  collectElfSections(reader);

  // Get the program entry point.
  if (not errors)
    {
      entryPoint = reader.get_entry();
      end = maxEnd;
    }

  return errors == 0;
}


bool
Memory::findElfSymbol(const std::string& name, ElfSymbol& symbol) const
{
  auto symbol_it = symbols_.find(name);
  if (symbol_it == symbols_.end())
    return false;

  symbol = symbol_it->second;
  return true;
}


bool
Memory::findElfSection(const std::string& name, ElfSymbol& symbol) const
{
  auto section_it = sections_.find(name);
  if (section_it == sections_.end())
    return false;

  symbol = section_it->second;
  return true;
}


bool
Memory::findElfFunction(uint64_t addr, std::string& name, ElfSymbol& value) const
{
  for (const auto& kv : symbols_)
    {
      const auto& sym = kv.second;
      size_t start = sym.addr_, end = sym.addr_ + sym.size_;
      if (addr >= start and addr < end)
	{
	  name = kv.first;
	  value = sym;
	  return true;
	}
    }

  return false;
}


void
Memory::printElfSymbols(std::ostream& out) const
{
  out << std::hex;
  for (const auto& kv : symbols_)
    out << kv.first << ' ' << "0x" << kv.second.addr_ << '\n';
  out << std::dec;
}


bool
Memory::getElfFileAddressBounds(const std::string& fileName, uint64_t& minAddr,
				uint64_t& maxAddr)
{
  ELFIO::elfio reader;

  if (not reader.load(fileName))
    {
      std::cerr << "Error: Failed to load ELF file " << fileName << '\n';
      return false;
    }

  // Get min max bounds of the segments.
  size_t minBound = ~ size_t(0);
  size_t maxBound = 0;
  unsigned validSegs = 0;
  for (const ELFIO::segment* seg : reader.segments)
    {
      if (seg->get_type() != PT_LOAD)
	continue;

      ELFIO::Elf64_Addr vaddr = seg->get_virtual_address();
      ELFIO::Elf_Xword size = seg->get_file_size(); // Size in file.

      minBound = std::min(minBound, size_t(vaddr));
      maxBound = std::max(maxBound, size_t(vaddr + size));
      validSegs++;
    }

  if (validSegs == 0)
    {
      std::cerr << "Error: No loadable segment in ELF file\n";
      return false;
    }

  minAddr = minBound;
  maxAddr = maxBound;
  return true;
}


bool
Memory::checkElfFile(const std::string& path, bool& is32bit,
		     bool& is64bit, bool& isRiscv)
{
  ELFIO::elfio reader;

  if (not reader.load(path))
    return false;

  is32bit = reader.get_class() == ELFCLASS32;
  is64bit = reader.get_class() == ELFCLASS64;
  isRiscv = reader.get_machine() == EM_RISCV;

  return true;
}


bool
Memory::isSymbolInElfFile(const std::string& path, const std::string& target)
{
  ELFIO::elfio reader;

  if (not reader.load(path))
    return false;

  auto secCount = reader.sections.size();
  for (int secIx = 0; secIx < secCount; ++secIx)
    {
      auto* sec = reader.sections[secIx];
      if (sec->get_type() != SHT_SYMTAB)
	continue;

      const ELFIO::symbol_section_accessor symAccesor(reader, sec);
      ELFIO::Elf64_Addr address = 0;
      ELFIO::Elf_Xword size = 0;
      unsigned char bind = 0, type = 0, other = 0;
      ELFIO::Elf_Half index = 0;

      // Finding symbol by name does not work. Walk all the symbols.
      ELFIO::Elf_Xword symCount = symAccesor.get_symbols_num();
      for (ELFIO::Elf_Xword symIx = 0; symIx < symCount; ++symIx)
	{
	  std::string name;
	  if (symAccesor.get_symbol(symIx, name, address, size, bind, type,
				    index, other))
	    {
	      if (name.empty())
		continue;
	      if (type == STT_NOTYPE or type == STT_FUNC or type == STT_OBJECT)
		if (name == target)
		  return true;
	    }
	}
    }
  return false;
}


bool
Memory::saveSnapshot_gzip(const std::string& filename,
                     const std::vector<std::pair<uint64_t,uint64_t>>& usedBlocks) const
{

  constexpr size_t maxChunk = size_t(1) << 28;

  // Open binary file for write (compressed) and check success.
  std::cerr << "Info: saveSnapshot starts..\n";
  gzFile gzout = gzopen(filename.c_str(), "wb1");
  if (not gzout)
    {
      std::cerr << "Error: Memory::saveSnapshot failed - cannot open " << filename
                << " for write\n";
      return false;
    }

  std::vector<uint32_t> temp;  // To collect sparse memory data.

  // write the simulated memory into the file and check success
  uint64_t prevAddr = 0;
  (void)prevAddr;
  bool success = true;
  for (const auto& blk: usedBlocks)
    {
#ifndef MEM_CALLBACKS
      if (blk.first >= size_)
	{
	  std::cerr << "Error: Memory::saveSnapshot: Block address (0x" << std::hex << blk.first
		    << ") out of bounds (0x" << size_ << ")\n" << std::dec;
	  success = false;
	  break;
	}
#endif

      size_t remainingSize = blk.second;
#ifndef MEM_CALLBACKS
      if (remainingSize > size_ or size_ - remainingSize < blk.first)
	{
	  std::cerr << "Error: Memory::saveSnapshot: Block at (0x" << std::hex << blk.first
		    << std::dec << ") extends beyond memory bound\n";
	  success = false;
	  break;
	}
#endif

      assert(prevAddr <= blk.first);
      prevAddr = blk.first + blk.second;

#ifdef MEM_CALLBACKS
      temp.resize(remainingSize);
      assert((blk.first & 3) == 0);
      assert((remainingSize & 3) == 0);
      size_t wordCount = remainingSize / 4;
      uint64_t addr = blk.first;
      for (size_t i = 0; i < wordCount; ++i, addr += 4)
	{
	  uint32_t x = 0;
	  peek(addr, x, false);
	  temp.at(i) = x;
	}
      uint8_t* buffer = reinterpret_cast<uint8_t*>(temp.data());
#else
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      uint8_t* buffer = data_ + blk.first;
#endif
      std::cerr << "*";
      while (remainingSize)  // write in chunk due to limitation of gzwrite
        {
          std::cerr << "-";
          fflush(stdout);
          size_t currentChunk = std::min(remainingSize, maxChunk);
          int resp = gzwrite(gzout, buffer, currentChunk);
          success = resp > 0 and size_t(resp) == currentChunk;
          if (not success)
            break;
          remainingSize -= currentChunk;

          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          buffer += currentChunk;
        }
      if (not success)
        break;
    }

  if (not success)
    std::cerr << "Error: Memory::saveSnapshot failed - write into " << filename
              << " failed with errno " << strerror(errno) << "\n";
  gzclose(gzout);
  std::cerr << "Info: \nsaveSnapshot finished\n";
  return success;
}

#if LZ4_COMPRESS
bool compress_lz4(FILE* out, const uint8_t* buffer, size_t mem_block_size) {
  // Check for the file pointer 
  if (not out) 
   {
    std::cerr << "Error: Memory::lz4_compress failed - cannot open file for write\n";
    return false;
  }
 
  // Set up the frame info for LZ4 compression
  LZ4F_frameInfo_t frame_info = {
    .blockSizeID = LZ4F_max4MB,                    // Use the largest block size (4MB)
    .blockMode = LZ4F_blockIndependent,            // Blocks are compressed independently (better for parallelism)
    .contentChecksumFlag = LZ4F_contentChecksumEnabled, // Enable content checksum for end-to-end integrity
    .frameType = LZ4F_frame,                       // Standard LZ4 frame
    .contentSize = mem_block_size,                // Total size of uncompressed data (optional but helps decompressor)
    .dictID = 0,                                   // No external dictionary used
    .blockChecksumFlag = LZ4F_blockChecksumEnabled // Enable per-block checksums (detect block-level corruption)
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

  // Set up LZ4 compression preferences
  LZ4F_preferences_t frame_preferences = {
    .frameInfo = frame_info,       // Attach the frame info defined above
    .compressionLevel = 0,         // Fastest compression (0 = default)
    .autoFlush = 1,                // Automatically flush at the end of each block
    .favorDecSpeed = 0             // Favor compression ratio (not decompression speed)
  };

#pragma GCC diagnostic pop

  // Get the max size of the destination buffer 
  size_t dst_size = LZ4F_compressFrameBound(mem_block_size, &frame_preferences);
  // Make the dst buffer, unique pointer 
  std::vector<uint8_t> dst(dst_size);
  // std::unique_ptr<uint8_t[]> dst(new uint8_t[dst_size]);
  // Compress the data in the buffer [src] into the dst buffer 
  size_t compressed_size = LZ4F_compressFrame(dst.data(), dst_size, buffer, mem_block_size, &frame_preferences);

  // Check if there was an error in compression
  if (LZ4F_isError(compressed_size)) 
   {
    std::cerr << "Error: Memory::saveSnapshot failed - LZ4 Compression Error: " << LZ4F_getErrorName(compressed_size) << "\n";
    return false;
  }

  // Check if 0 was returned, i.e nothing was written into the dst buffer 
  if (compressed_size == 0) 
   {
    std::cerr << "Error: Memory::saveSnapshot failed - Non-zero value returned \n"
          << "dst size: " << dst_size << ", "
          << "mem_block_size: " << mem_block_size << ", "
          << "compressed_size: " << compressed_size << "\n";
    return false;
  }

  // Write the compressed data into the file 
  fwrite(dst.data(), 1, compressed_size, out);
  return true;
}

bool
Memory::saveSnapshot_lz4(const std::string& filename,
                     const std::vector<std::pair<uint64_t,uint64_t>>& usedBlocks) const
{

  // Open binary file for write (compressed) and check success.
  std::cerr << "Info: saveSnapshot_lz4 starts..\n";
  // Open the file to append the data 
  FILE *out = fopen(filename.c_str(), "wb");
  // Check if the file was opened successfully 
  if (not out)
    {
      std::cerr << "Error: Memory::saveSnapshot failed - cannot open " << filename
                << " for write\n";
      return false;
    }

  std::vector<uint32_t> temp;  // To collect sparse memory data.

  // write the simulated memory into the file and check success
  uint64_t prevAddr = 0;
  (void)prevAddr;
  bool success = true;
  for (const auto& blk: usedBlocks)
    {
#ifndef MEM_CALLBACKS
      if (blk.first >= size_)
	{
	  std::cerr << "Error: Memory::saveSnapshot: Block address (0x" << std::hex << blk.first
		    << ") out of bounds (0x" << size_ << ")\n" << std::dec;
	  success = false;
	  break;
	}
#endif

      size_t remainingSize = blk.second;
#ifndef MEM_CALLBACKS
      if (remainingSize > size_ or size_ - remainingSize < blk.first)
	{
	  std::cerr << "Error: Memory::saveSnapshot: Block at (0x" << std::hex << blk.first
		    << std::dec << ") extends beyond memory bound\n";
	  success = false;
	  break;
	}
#endif

      assert(prevAddr <= blk.first);
      prevAddr = blk.first + blk.second;

#ifdef MEM_CALLBACKS
      temp.resize(remainingSize);
      assert((blk.first & 3) == 0);
      assert((remainingSize & 3) == 0);
      size_t wordCount = remainingSize / 4;
      uint64_t addr = blk.first;
      for (size_t i = 0; i < wordCount; ++i, addr += 4)
	{
	  uint32_t x = 0;
	  peek(addr, x, false);
	  temp.at(i) = x;
	}
      uint8_t* buffer = reinterpret_cast<uint8_t*>(temp.data());
#else
      uint8_t* buffer = data_ + blk.first;
#endif
      std::cerr << "*";

      // Compress the data in the block 
      // Buffer Contains the sim memory 
      // blk.second has the size 
      bool success = compress_lz4(out, buffer, blk.second);

      if (not success)
        break;
    }
  // Close the file 
  fclose(out);

  if (not success)
    std::cerr << "Error: Memory::saveSnapshot failed - write into " << filename; 
  
  std::cerr << "Info: \nsaveSnapshot finished\n";
  return success;
}
#endif


bool
Memory::loadSnapshot_gzip(const std::string & filename,
                     const std::vector<std::pair<uint64_t,uint64_t>>& usedBlocks)
{
  constexpr size_t maxChunk = size_t(1) << 28;  // This must match saveSnapshot
  std::cerr << "Info: loadSnapshot starts..\n";

  // open binary file for read (decompress) and check success
  gzFile gzin = gzopen(filename.c_str(), "rb");
  if (not gzin or gzeof(gzin))
    {
      std::cerr << "Error: Memory::loadSnapshot failed - cannot open "
                << filename << " for read\n";
      return false;
    }

  std::vector<uint32_t> temp;

  // read (decompress) file into simulated memory and check success
  bool success = true;
  uint64_t prevAddr = 0;
  (void)prevAddr;
  size_t remainingSize = 0;
  for (const auto& blk: usedBlocks)
    {
#ifndef MEM_CALLBACKS
      if (blk.first >= size_)
	{
	  std::cerr << "Error: Memory::loadSnapshot: Block address (0x" << std::hex << blk.first
		    << ") out of bounds (0x" << size_ << ")\n" << std::dec;
	  success = false;
	  break;
	}
#endif
      remainingSize = blk.second;
#ifndef MEM_CALLBACKS
      if (remainingSize > size_ or size_ - remainingSize < blk.first)
	{
	  std::cerr << "Error: Memory::loadSnapshot: Block at (0x" << std::hex << blk.first
		    << ") extends beyond memory bound\n" << std::dec;
	  success = false;
	  break;
	}
#endif

      assert((blk.first & 3) == 0);
      assert((remainingSize & 3) == 0);
      assert(prevAddr <= blk.first);
      temp.resize(maxChunk);
      prevAddr = blk.first + blk.second;
      uint64_t addr = blk.first;

      std::cerr << "*";
      while (remainingSize) // read in chunk due to gzread limitation
        {
          std::cerr << "-";
          fflush(stdout);
          size_t currentChunk = std::min(remainingSize, maxChunk);
          int resp = gzread(gzin, temp.data(), currentChunk);
	  int words = resp / 4;
	  for (int i = 0; i < words; ++i, addr += 4)
	    {
	      // Avoid poking zero pages to maintain sparsity.
	      uint32_t prev = 0;
	      peek(addr, prev, false);
	      uint32_t curr = temp.at(i);
	      if (curr != prev)
		poke(addr, curr);
	    }
          if (resp == 0)
            {
              success = gzeof(gzin);
              break;
            }
          remainingSize -= resp;
        }
      if (not success)
        break;
    }
  std::cerr << "\n";

  std::cerr << '\n';

  if (not success)
    std::cerr << "Error: Memory::loadSnapshot failed - read from " << filename
              << " failed: " << gzerror(gzin, nullptr) << "\n";
  else if (remainingSize > 0)
    std::cerr << "Warning: Memory::loadSnapshot: Snapshot data size smaller than memory size\n";

  gzclose(gzin);
  std::cerr << "Info: loadSnapshot finished\n";
  return success;
}

#if LZ4_COMPRESS
// src_buffer - buffer with the contents of the compressed lz4 file 
// dst_buffer - buffer with the decompressed data 
// mem_block_size - size of the decompressed data 

bool decompress_frame_lz4(uint8_t** src_buffer, std::vector<uint32_t>& dst_buffer, size_t mem_block_size) {
  // Get the frame info for lz4 decompression
  // Get a decompresstion context 
  
  LZ4F_dctx* dctx; 
  // Error checking 
  size_t error = LZ4F_createDecompressionContext(&dctx, LZ4F_VERSION); 

  // Check if there was an error in creating a decompression context 
  if (LZ4F_isError(error)) 
   {
    std::cerr << "Error: Memory::decompress_lz4 failed - LZ4 Unable to create decompression context: " << LZ4F_getErrorName(error) << "\n";
    return false;
  }

  struct DctxGuard {
    LZ4F_dctx* dctx;
    DctxGuard(LZ4F_dctx* dctx) : dctx(dctx) {}
    ~DctxGuard() { LZ4F_freeDecompressionContext(dctx); }
  };

  // RAII for decompression context 
  DctxGuard dctxGuard(dctx);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

  // Get decomrpession preferences 
  LZ4F_decompressOptions_t decompress_options = {
    .stableDst = 1,
  };

#pragma GCC diagnostic pop

  size_t frame_size = LZ4F_HEADER_SIZE_MAX; // Setting the frame size variable to the max lz4 frame size 
  LZ4F_frameInfo_t frame_info;

  // Read the frame 
  error = LZ4F_getFrameInfo(dctx, &frame_info, *src_buffer, &frame_size);

  // Check if there was an error : 
  if (LZ4F_isError(error)) 
    {
      std::cerr << "Error: Memory::decompress_lz4 failed - LZ4 Frame Error: " << LZ4F_getErrorName(error) << "\n";
      return false;
    }

  // frame_size will now contain the size of the frame 
  // Increment the src buffer by the size of the frame consumed 
  *src_buffer += frame_size;

  // frame_info will now contain the size of the de-compressed data in the frame. 
  // Check the size of the content in the frame and the size of the mem block 
  if (frame_info.contentSize != mem_block_size) 
    {
      std::cerr << "Error: Memory::decompress_frame_lz4 failed - Content size mismatch: " << frame_info.contentSize << " != " << mem_block_size << " (frame_size: " << frame_size << ")\n"; 
      return false;
    }
  
  // Get the decompressed data 
  // Content size + lz4_max_header_size + 1 CRC byte 
  size_t dst_size = frame_info.contentSize + LZ4F_HEADER_SIZE_MAX + 1;
  
  // Resize the dst_buffer to the correct size, dst_size is in bytes. 
  dst_buffer.resize(dst_size / 4);

  // Decompress "src_size" bytes from the src_buffer 
  // When a frame is fully decoded, @return will be 0 (no more data expected).
  // When provided with more bytes than necessary to decode a frame,
  // dst_size >>> size of the compressed frame in the src_buffer. 

  size_t src_size = dst_size;    
  // Decompress the frame into dst_buffer 
  error = LZ4F_decompress(dctx, dst_buffer.data(), &dst_size, *src_buffer, &src_size, &decompress_options); 

  // Check for error 
  if (LZ4F_isError(error))
    {
      std::cerr << "Error: decompress_file failed - LZ4F_decompressFrame failed: " << LZ4F_getErrorName(error) << "\n";
      return false; 
    }

  // Make sure the error is 0 
  if (error != 0)
    {
      std::cerr << "Decompress_file Non-zero value returned :  " << error <<  "\n";; 
      return false; 
    }

  dst_buffer.resize(dst_size / 4);


  // Check if the size of the decomrpessed data is equal to the contentSize 
  if (dst_buffer.size() * 4 != mem_block_size) 
    {
      std::cerr << "Error: Memory::decompress_frame_lz4 failed - Decompressed data size mismatch: \n"
                << "dst_buffer.size(): " << dst_buffer.size() << "\n"
                << "dst_buffer bytes: " << dst_buffer.size() * 4 << "\n\n"
                << "mem block bytes: " << mem_block_size << "\n\n"
                << "Content size from frame: " << frame_info.contentSize << "\n\n"
                << "dst size: " << dst_size << "\n"
                << "src size: " << src_size << "\n";
      return false; 
    }

  // Increment the src_buffer by the size of the data read
  *src_buffer += src_size; 

  // Return true 
  return true; 
}

bool
Memory::loadSnapshot_lz4(const std::string & filename,
                     const std::vector<std::pair<uint64_t,uint64_t>>& usedBlocks)
{
  std::cerr << "Info: loadSnapshot starts_lz4..\n";

  // Open the file with the compressed data 
  FILE *in = fopen(filename.c_str(), "rb");
  // Check if the file was opened successfully 

  if (not in)
    {
      std::cerr << "Error: Memory::loadSnapshot_lz4 failed - cannot open " << filename << " for read\n";
      return false;
    }

  // Read the entire file into the src_buffer 
  // Get the size of the file 
  fseek(in, 0, SEEK_END);
  int src_size = ftell(in);
  fseek(in, 0, SEEK_SET);

  // Allocate memory for the source buffer 
  uint8_t* src_buffer = (uint8_t*)malloc(src_size);
  uint8_t* src_buffer_end = src_buffer + src_size;

  // Read the file into the source buffer 
  if (not fread(src_buffer, 1, src_size, in))
    {
      std::cerr << "Error: Memory::loadSnapshot_lz4 failed - cannot read " << filename << "\n";
      return false;
    }

  std::vector<uint32_t> temp;

  // read (decompress) file into simulated memory and check success
  bool success = true;
  uint64_t prevAddr = 0;
  (void)prevAddr;
  int block_count = -1;
  for (const auto& blk: usedBlocks)
    {
      block_count++;
#ifndef MEM_CALLBACKS
      if (blk.first >= size_)
	{
	  std::cerr << "Error: Memory::loadSnapshot: Block address (0x" << std::hex << blk.first
		    << ") out of bounds (0x" << size_ << ")\n" << std::dec;
	  success = false;
	  break;
	}
#endif
      size_t remainingSize = blk.second;
#ifndef MEM_CALLBACKS
      if (remainingSize > size_ or size_ - remainingSize < blk.first)
	{
	  std::cerr << "Error: Memory::loadSnapshot: Block at (0x" << std::hex << blk.first
		    << ") extends beyond memory bound\n" << std::dec;
	  success = false;
	  break;
	}
#endif

      assert((blk.first & 3) == 0);
      assert(prevAddr <= blk.first);
      if ((remainingSize & 3) != 0)
        assert(0);

      prevAddr = blk.first + blk.second;
      uint64_t addr = blk.first;
      std::cerr << "*";

      temp.clear();

      // Decompress the frame 
      success = decompress_frame_lz4(&src_buffer, temp, blk.second);
      // Check if the decompression was successful 
      if(not success)
        {
          std::cerr << "Decompressing BLOCK: " << block_count << " Block size: " << blk.second << " Block Address : " << blk.first << "\n";
          std::cerr << "Error: Memory::loadSnapshot_lz4 failed - decompression failed\n";
          break;
        }

      //Check if the src_buffer has reached beyond the end of the file 
      if(src_buffer > src_buffer_end) 
       {
        std::cerr << "Error: Memory::loadSnapshot_lz4 failed - src_buffer has reached beyond the end of the file\n";
        success = false; 
        break; 
      }

      int words = temp.size(); // temp is of type uint32_t, size = # words. 
      for (int i = 0; i < words; ++i, addr += 4) 
       { 
        uint32_t prev = 0; 
        peek(addr, prev, false); 
        uint32_t curr = temp.at(i); 
        if(curr != prev) 
         {
          poke(addr, curr);
        } 
      }
    }

  if (not success)
    std::cerr << "Error: Memory::loadSnapshot failed - read from " << filename << " failed\n";

  std::cerr << "Error: \nloadSnapshot finished\n";
  return success;
}
#endif

bool
Memory::saveAddressTrace(std::string_view tag, const LineMap& lineMap,
			 const std::string& path, bool skipClean,
                         bool includeValues) const
{
  std::ofstream out(path, std::ios::trunc);
  if (not out)
    {
      std::cerr << "Error: Memory::saveAddressTrace: Failed to open " << path << " for write\n";
      return false;
    }

  std::cerr << "Info: Trace map size for " << tag << ": " << lineMap.size() << '\n';

  std::vector<uint64_t> addrVec;
  addrVec.reserve(lineMap.size());

  for (const auto& kv : lineMap)
    addrVec.push_back(kv.first);

  std::sort(addrVec.begin(), addrVec.end(),
	    [&lineMap](uint64_t a, uint64_t b) {
	      return lineMap.at(a).order < lineMap.at(b).order;
	    });

  out << std::hex;

  unsigned lineSize = 1 << lineShift_;

  for (auto vaddr : addrVec)
    {
      const auto& entry = lineMap.at(vaddr);
      if (skipClean and entry.clean)
        continue;

      uint64_t paddr = entry.paddr;
      out << vaddr << ':' << paddr;

      if (includeValues)
        {
          out << ':';
          uint64_t lineAddr = paddr << lineShift_;
          for (unsigned i = 0; i < lineSize; ++i)
            {
              uint8_t byte = 0;
              uint64_t byteAddr = lineAddr + lineSize - 1 - i;
              peek(byteAddr, byte, false);
              out << unsigned(byte >> 4) << unsigned(byte & 0xf);
            }
        }
      out << '\n';
    }

  out << std::dec;

  return true;
}


bool
Memory::loadAddressTrace(LineMap& lineMap, uint64_t& refCount, const std::string& path)
{
  std::ifstream ifs(path);

  if (not ifs.good())
    {
      std::cerr << "Error: Failed to open lines file " << path << "' for input.\n";
      return false;
    }

  std::string line;
  while (std::getline(ifs, line))
    {
      std::vector<std::string> tokens;
      boost::split(tokens, line, boost::is_any_of(":"));

      if (tokens.size() < 2)
        {
          std::cerr << "Error: Failed to load addresses from line.\n";
          return false;
        }

      uint64_t vaddr = strtoull(tokens.at(0).c_str(), nullptr, 16);
      uint64_t paddr = strtoull(tokens.at(1).c_str(), nullptr, 16);

      lineMap[vaddr] = LineEntry{paddr, refCount++};
    }
  return true;
}


bool
Memory::saveDataAddressTrace(const std::string& path, bool skipClean,
                             bool includeValues) const
{
  if (not dataLineTrace_)
    return true;
  return saveAddressTrace("data", dataLineMap_, path, skipClean, includeValues);
}


bool
Memory::saveInstructionAddressTrace(const std::string& path) const
{
  if (not instrLineTrace_)
    return true;
  return saveAddressTrace("instruction", instrLineMap_, path);
}


bool
Memory::loadDataAddressTrace(const std::string& path)
{
  if (not dataLineTrace_)
    return true;
  return loadAddressTrace(dataLineMap_, memRefCount_, path);
}


bool
Memory::loadInstructionAddressTrace(const std::string& path)
{
  if (not dataLineTrace_)
    return true;
  return loadAddressTrace(instrLineMap_, memRefCount_, path);
}


bool
Memory::initializeByte(uint64_t addr, uint8_t value)
{
  if (addr >= size_)
    return false;

  if (pmaMgr_.isMemMappedReg(addr))
    {
      if (not pmaMgr_.pokeRegisterByte(addr, value))
        return false;
    }

  // We initialize both the memory-mapped-register and the external
  // memory to match/simplify the test-bench.
  if (writeCallback_)
    {
      writeCallback_(addr, 1, value);
    }
  else
    {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      data_[addr] = value;
    }
  return true;
}


bool
Memory::initializePage(uint64_t addr, const std::span<uint8_t> buffer)
{
  if (not isPageAligned(addr))
    return false;

  if (addr >= size_ or addr + pageSize_ - 1 >= size_)
    return false;

  // Caller is responsible for checking that the page is all in regular memory.

  assert(buffer.size() >= pageSize_);

#ifndef MEM_CALLBACKS

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  memcpy(data_ + addr, buffer.data(), pageSize_);
  return true;

#else
  
  if (initPageCallback_)
    return initPageCallback_(addr, buffer);

  const uint8_t* ba = buffer.data();
  for (unsigned i = 0; i < pageSize_; i += 8, addr += 8, ba += 8)
    {
      uint64_t value = *(reinterpret_cast<const uint64_t*>(ba));
      writeCallback_(addr, 8, value);
    }

  return true;

#endif
}


void
Memory::resetMemoryMappedRegisters()
{
  pmaMgr_.resetMemMapped();
}


bool Memory::findSymbolByAddress(uint64_t address, std::string &symbol) const {
  auto it = addrToSymName_.find(address);
  if (it != addrToSymName_.end()) {
    symbol = it->second;
    return true;
  }
  return false;
}
