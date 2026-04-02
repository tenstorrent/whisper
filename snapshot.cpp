#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <unistd.h>
#include <fcntl.h>
#include "Filesystem.hpp"
#include "Hart.hpp"


using namespace WdRiscv;


template <typename URV>
bool
Hart<URV>::saveSnapshotRegs(const std::string & filename)
{
  // open file for write, check success
  std::ofstream ofs(filename, std::ios::trunc);
  if (not ofs)
    {
      std::cerr << "Error: Hart::saveSnapshotRegs failed - cannot open " << filename << " for write\n";
      return false;
    }

  // Write Privilege Mode, Virtual mode, Program Order, Program Break,
  // and Program Counter.
  ofs << "pm " << unsigned(privilegeMode()) << '\n';
  ofs << "vm " << unsigned(virtMode()) << '\n';
  ofs << "po " << getInstructionCount() << '\n';
  ofs << "pr " << getRetiredInstructionCount() << '\n';
  ofs << "pb 0x" << std::hex << syscall_.targetProgramBreak() << std::dec << '\n';
  ofs << "pc 0x" << std::hex << peekPc() << std::dec << '\n';
  ofs << "elp " << unsigned(getElp()) << '\n';

  // write integer registers
  for (unsigned i = 1; i < 32; i++)
    {
      URV val = peekIntReg(i);
      if (val)
	ofs << "x " << std::dec << i << " 0x" << std::hex << val << std::dec << "\n";
    }

  // write floating point registers
  if (isRvf())
    for (unsigned i = 0; i < 32; i++)
      {
	uint64_t val = 0;
	peekFpReg(i, val);
	if (val)
	  ofs << "f " << std::dec << i << " 0x" << std::hex << val << std::dec << "\n";
      }

  // Write control & status registers. Write MISA first.
  using CN = CsrNumber;
  ofs << std::hex;
  auto csr = csRegs_.findCsr(CN::MISA);
  if (csr)
    ofs << "c 0x" << unsigned(CN::MISA) << " 0x" << csr->read() << "\n";

  for (auto i = unsigned(CN::PMACFG15); i >= unsigned(CN::PMACFG0); i--)
    {
      auto csr = csRegs_.findCsr(CN(i));
      if (not csr or not csr->isImplemented())
	continue;
      if (csr->read() == 0 and csr->getResetValue() == 0)
	continue;
      ofs << "c 0x" << i << " 0x" << csr->read() << "\n";
    }

  for (auto i = unsigned(CN::MIN_CSR_); i <= unsigned(CN::MAX_CSR_); i++)
    {
      if ( (i == unsigned(CN::MISA)) or
	   (i >= unsigned(CN::PMACFG0) and i <= unsigned(CN::PMACFG15)))
	continue;
      auto csr = csRegs_.findCsr(CN(i));
      if (not csr or not csr->isImplemented())
	continue;
      if (csr->read() == 0 and csr->getResetValue() == 0)
	continue;
      ofs << "c 0x" << i << " 0x" << csr->read() << "\n";
    }
  ofs << std::dec;

  // write vector registers.
  if (isRvv())
    {
      std::vector<uint8_t> vecBytes;
      for (unsigned i = 0; i < vecRegCount(); ++i)
	{
	  peekVecReg(i, vecBytes);
	  ofs << "v " << std::dec << i << " 0x";
	  ofs << std::hex;
	  for (auto byte : vecBytes)
	    ofs << std::setw(2) << std::setfill('0') << unsigned(byte);
	  ofs << std::dec;
	  ofs << '\n';
	}
    }

  ofs.close();
  return true;
}


/// Read an integer value fromt the given stream. Return true on
/// success and false on failure. Use strtoull to do the string to
/// integer conversion so that numbers starting with 0 or 0x are
/// interpreted as octal or hexadecimal.
static
bool
loadSnapshotValue(std::istream& stream, uint64_t& val)
{
  std::string str;
  if (not (stream >> str))
    return false;
  char* extra = nullptr;
  val = strtoull(str.c_str(), &extra, 0);
  return not extra or not *extra;
}


/// Read from the given stream a register number and a register value.
/// Return true on success and false on failure. Use strtoull to do
/// the string to integer conversions so that numbers starting with 0
/// or 0x are interpreted as octal or hexadecimal.
static
bool
loadRegNumAndValue(std::istream& stream, unsigned& num, uint64_t& val)
{
  std::string str;
  if (not (stream >> str))
    return false;
  char* extra = nullptr;
  num = strtoull(str.c_str(), &extra, 0);
  if (extra and *extra)
    return false;
  return loadSnapshotValue(stream, val);
}


/// Read from the given stream a vector register number and a register value.
/// Return true on success and false on failure. Register value must have
/// with a 0x prefix and must be a hexadecimal number. The most sig digits
/// of the register value are placed in the first entry of vecBytes.
static
bool
loadVecRegNumAndValue(std::istream& stream, unsigned& num,
		      std::vector<uint8_t>& vecBytes)
{
  vecBytes.clear();

  std::string str;
  if (not (stream >> str))
    return false;

  char* extra = nullptr;
  num = strtoull(str.c_str(), &extra, 0);
  if (extra and *extra)
    return false;

  std::string vvs;  // vector value string
  if (not (stream >> vvs))
    return false;

  if (not (vvs.starts_with("0x") or vvs.starts_with("0X")))
    return false;

  vvs = vvs.substr(2);  // Skip leading 0x

  for (auto c : vvs)
    if (not std::isxdigit(c))
      return false;

  if ((vvs.size() & 1) != 0)
    vvs.insert(vvs.begin(), '0');  // Make hex digit count even by prepeindg a '0'.

  vecBytes.resize(vvs.size() / 2);

  for (size_t i = 0; i < vvs.size(); ++i)
    {
      using uchar = unsigned char;

      unsigned hd = uchar(vvs[i]);  // hex digit
      unsigned nibble = 0;
      if      (hd >= '0' and hd <= '9') nibble = hd - '0';
      else if (hd >= 'a' and hd <= 'f') nibble = 10 + hd - 'a';
      else if (hd >= 'A' and hd <= 'F') nibble = 10 + hd - 'A';
      if ((i & 1) == 0)
	vecBytes[i/2] |= (nibble << 4);
      else
	vecBytes[i/2] |= nibble;
    }

  return true;
}


template <typename URV>
bool
Hart<URV>::loadSnapshotRegs(const std::string & filename)
{
  // open file for read, check success
  std::ifstream ifs(filename);
  if (not ifs)
    {
      std::cerr << "Error: Hart::loadSnapshotRegs failed - cannot open " << filename << " for read\n";
      return false;
    }
  // read line by line and set registers
  std::string line;
  unsigned lineNum = 0, num = 0, errors = 0;
  uint64_t val = 0;
  auto privMode = PrivilegeMode::Machine;
  bool virtMode = false;

  URV mstatus = 0, mstatush = 0;
  bool foundMstatus = false, foundMstatush = false;

  URV mipVal = 0;
  bool foundMip = false;

  std::vector<std::pair<CsrNumber, uint64_t>> deferredVecCsrs;

  using std::cerr;

  while(std::getline(ifs, line))
    {
      lineNum++;
      std::istringstream iss(line);
      std::string type;
      // parse first part (register class)
      if (not (iss >> type))
	{
	  errors++;
	  cerr << "Error: Register snapshot loader: Line " << lineNum
	       << ": Failed to read line tag\n";
	  break;
	}

      if (type == "pc")     // PC
        {
          if (not loadSnapshotValue(iss, val))
            errors++;
          else
	    pokePc(val);
        }
      else if (type == "pm")  // Prrivilege mode
	{
	  if (loadSnapshotValue(iss, val))
            {
              if (val == 0)
                privMode = PrivilegeMode::User;
              else if (val == 1)
                privMode = PrivilegeMode::Supervisor;
              else if (val == 3)
                privMode = PrivilegeMode::Machine;
              else
                errors++;
            }
	  else
	    errors++;
	}
      else if (type == "vm")  // Virtual mode
	{
	  if (loadSnapshotValue(iss, val))
	    virtMode = val;
	  else
	    errors++;
	}
      else if (type == "po")  // Program order
        {
          if (loadSnapshotValue(iss, val))
            setInstructionCount(val);
          else
            errors++;
        }
      else if (type == "pr")  // Program retired
        {
          if (loadSnapshotValue(iss, val))
            setRetiredInstructionCount(val);
          else
            errors++;
        }
      else if (type == "pb")  // Program break
        {
          if (loadSnapshotValue(iss, val))
            setTargetProgramBreak(val);
          else
            errors++;
        }
      else if (type == "elp") // ELP
        {
          if (loadSnapshotValue(iss, val))
            setElp(val);
          else
            errors++;
        }
      else if (type == "c")   // CSR
        {
          if (not loadRegNumAndValue(iss, num, val))
	    {
	      errors++;
	      continue;
	    }
          auto csrNum = CsrNumber(num);
	  auto csr = csRegs_.findCsr(csrNum);
          // Poke to propagate updates to cached values
	  if (csr and csr->isImplemented())
            {
              if ((csrNum >= CsrNumber::MIREG and csrNum <= CsrNumber::MIREG6) or
                  (csrNum >= CsrNumber::SIREG and csrNum <= CsrNumber::SIREG6) or
                  (csrNum >= CsrNumber::VSIREG and csrNum <= CsrNumber::VSIREG6))
                continue;
              if (csrNum == CsrNumber::MSTATUS)
                {
                  foundMstatus = true;
                  mstatus = val;
                  continue;
                }
              if (csrNum == CsrNumber::MSTATUSH)
                {
                  foundMstatush = true;
                  mstatush = val;
                  continue;
                }
              if (csrNum == CsrNumber::MIP)
                {
                  foundMip = true;
                  mipVal = val;
                  continue;
                }
              if (csrNum == CsrNumber::VSTART or csrNum == CsrNumber::VXSAT or
                  csrNum == CsrNumber::VXRM or csrNum == CsrNumber::VCSR or
                  csrNum == CsrNumber::VL or csrNum == CsrNumber::VTYPE or
                  csrNum == CsrNumber::VLENB)
                {
                  deferredVecCsrs.emplace_back(csrNum, val);
                  continue;
                }
              pokeCsr(csrNum, val);
            }
	  else
	    cerr << "Error: Register snapshot loader: Line " << lineNum
		 << ": No such CSR: " << line << '\n';
        }
      else if (type == "x")   // Integer register
        {
          if (not loadRegNumAndValue(iss, num, val))
	    {
	      errors++;
	      continue;
	    }
          if (not pokeIntReg(num, val))
	    cerr << "Error: Register snapshot loader: Line " << lineNum
		 << ": No such register: " << line << '\n';
        }
      else if (type == "f")   // FP register
        {
          if (isRvf() or isRvd())
            {
              if (not loadRegNumAndValue(iss, num, val))
		{
		  errors++;
		  continue;
		}
              if (not pokeFpReg(num, val))
		cerr << "Error: Register snapshot loader: Line " << lineNum
		     << ": No such FP register: " << line << '\n';
            }
        }
      else if (type == "v") // Vector registers
	{
	  if (isRvv())
	    {
	      std::vector<uint8_t> vecBytes;
	      if (not loadVecRegNumAndValue(iss, num, vecBytes))
		{
		  errors++;
		  continue;
		}
	      if (not pokeVecReg(num, vecBytes))
		cerr << "Error: Register snapshot loader: Line " << lineNum
		     << ": No such vector register: " << line << '\n';
	    }
	}
      else
	{
	  cerr << "Error: Register snapshot loader: Line " << lineNum
	       << ": Ignoring unexpected line: " << line << '\n';
	}
    }

  setPrivilegeMode(privMode);
  virtMode_ = virtMode;

  // We poke MSTATUS before vector CSRs but after all other CSRs to prevent corruption.
  if (foundMstatus)
    pokeCsr(CsrNumber::MSTATUS, mstatus);
  if constexpr (sizeof(URV) == 4)
    if (foundMstatush)
      pokeCsr(CsrNumber::MSTATUSH, mstatush);

  // Vector CSRs are deferred because pokeCsr silently ignores them when MSTATUS.VS is
  // Off.  The snapshot may legitimately have VS=Off (e.g. after a context switch) while
  // still carrying valid vtype/vl values.  Temporarily set VS to Initial so that the
  // pokes go through, then restore the original MSTATUS value.
  if (not deferredVecCsrs.empty())
    {
      bool tempEnabled = not isVecEnabled();
      if (tempEnabled)
        pokeCsr(CsrNumber::MSTATUS, mstatus | (URV(1) << 9));  // VS = Initial

      for (auto& [csrNum, csrVal] : deferredVecCsrs)
        pokeCsr(csrNum, URV(csrVal));

      if (tempEnabled)
        pokeCsr(CsrNumber::MSTATUS, mstatus);
    }

  // Restore MIP to its snapshot value (or clear it). CSR poke side effects during loading
  // (e.g. MHPMEVENT OF bit transition sets MIP.LCOF, timer comparisons set MIP.MTIP) can
  // leave MIP in a state that doesn't match the snapshot. Poking MIP here overrides those
  // spurious bits. Timer/interrupt bits will be correctly recomputed on the first singleStep.
  pokeCsr(CsrNumber::MIP, foundMip ? mipVal : 0);

  if (errors)
    {
      cerr << "Error: Hart::loadSnapshotRegs failed to parse " << filename << ":"
	   << std::dec << lineNum << "\n";
      cerr << "Error: \t" << line << "\n";
    }
  ifs.close();

  updateCachedMstatus();
  updateAddressTranslation();
  updateTranslationPbmt();

  return errors == 0;
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
