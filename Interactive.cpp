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
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include "Interactive.hpp"
#include "linenoise.hpp"

using namespace WdRiscv;


/// Return format string suitable for printing an integer of type URV
/// in hexadecimal form.
template <typename URV>
static
const char*
getHexForm()
{
  if (sizeof(URV) == 4)
    return "0x%08x";
  if (sizeof(URV) == 8)
    return "0x%016x";
  if (sizeof(URV) == 16)
    return "0x%032x";
  return "0x%x";
}


/// Convert the command line string numberStr to a number using
/// strotull and a base of zero (prefixes 0 and 0x are
/// honored). Return true on success and false on failure (string does
/// not represent a number). TYPE is an integer type (e.g
/// uint32_t). Option is the command line option associated with the
/// string and is used for diagnostic messages.
template <typename TYPE>
static
bool
parseCmdLineNumber(const std::string& option,
		   const std::string& numberStr,
		   TYPE& number)
{
  bool good = not numberStr.empty();

  if (good)
    {
      char* end = nullptr;
      uint64_t value = strtoull(numberStr.c_str(), &end, 0);
      number = static_cast<TYPE>(value);
      if (number != value)
	{
	  std::cerr << "parseCmdLineNumber: Number too large: " << numberStr
		    << '\n';
	  return false;
	}
      if (end and *end)
	good = false;  // Part of the string are non parseable.
    }

  if (not good)
    std::cerr << "Invalid command line " << option << " value: " << numberStr
	      << '\n';
  return good;
}


static
bool
parseCmdLineVecData(const std::string& option,
		    const std::string& valStr,
		    std::vector<uint8_t>& val)
{
  val.clear();

  if (not (boost::starts_with(valStr, "0x") or boost::starts_with(valStr, "0X")))
    {
      std::cerr << "Value of vector " << option << " must begin with 0x: "
		<< valStr << '\n';
      return false;
    }

  std::string trimmed = valStr.substr(2); // Remove leading 0x
  if (trimmed.empty())
    {
      std::cerr << "Empty value for vector " << option << ": "
		<< valStr << '\n';
      return false;
    }

  if ((trimmed.size() & 1) != 0)
    {
      std::cerr << "Value for vector " << option << " must have an even"
		<< " number of hex digits: " << valStr << '\n';
      return false;
    }

  for (size_t i = 0; i < trimmed.size(); i += 2)
    {
      std::string byteStr = trimmed.substr(i, 2);
      char* end = nullptr;
      unsigned value = strtoul(byteStr.c_str(), &end, 16);
      if (end and *end)
	{
	  std::cerr << "Invalid hex digit(s) in vector " << option << ": "
		    << byteStr << '\n';
	  return false;
	}
      val.push_back(value);
    }

  return true;
}


template <typename URV>
Interactive<URV>::Interactive(System<URV>& system)
  : system_(system)
{
}


template <typename URV>
bool
Interactive<URV>::untilCommand(Hart<URV>& hart, const std::string& line,
			       const std::vector<std::string>& tokens,
			       FILE* traceFile)
{
  if (tokens.size() != 2)
    {
      std::cerr << "Invalid until command: " << line << '\n';
      std::cerr << "Expecting: until address\n";
      return false;
    }

  size_t addr = 0;
  if (not parseCmdLineNumber("address", tokens.at(1), addr))
    return false;

  if (addr >= hart.memorySize())
    std::cerr << "Warning: Address outside memory range.\n";

  if (hart.inDebugMode())
    {
      hart.exitDebugMode();   // Resume from halt.
      if (hart.hasDcsrStep())
	{
	  hart.singleStep(traceFile);
	  hart.enterDebugMode(hart.peekPc());
	  return true;
	}
    }

  return hart.untilAddress(addr, traceFile);
}


template <typename URV>
bool
Interactive<URV>::runCommand(Hart<URV>& hart, const std::string& /*line*/,
			     const std::vector<std::string>& /*tokens*/,
			     FILE* traceFile)
{
  if (hart.inDebugMode())
    {
      hart.exitDebugMode();
      if (hart.hasDcsrStep())
	{
	  hart.singleStep(traceFile);
	  hart.enterDebugMode(hart.peekPc());
	  return true;
	}
    }
  return hart.run(traceFile);
}


template <typename URV>
bool
Interactive<URV>::stepCommand(Hart<URV>& hart, const std::string& /*line*/,
			      const std::vector<std::string>& tokens,
			      FILE* traceFile)
{
  uint64_t count = 1;

  if (tokens.size() > 1)
    {
      if (not parseCmdLineNumber("instruction-count", tokens.at(1), count))
	return false;
      if (count == 0)
	return true;
    }

  uint64_t tag = 0;
  bool hasTag = false;
  if (tokens.size() > 2)
    {
      if (not parseCmdLineNumber("instruction-tag", tokens.at(2), tag))
	return false;
      hasTag = true;
    }

  bool wasInDebug = hart.inDebugMode();
  if (wasInDebug)
    hart.exitDebugMode();

  for (uint64_t i = 0; i < count; ++i)
    {
      if (hasTag)
	{
	  system_.mcmSetCurrentInstruction(hart, tag);
	  DecodedInst di;
	  hart.setInstructionCount(tag-1);
	  hart.singleStep(di, traceFile);
	  if (not di.isValid())
	    assert(hart.lastInstructionTrapped());
	  system_.mcmRetire(hart, this->time_, tag++, di);
	}
      else
	hart.singleStep(traceFile);
    }

  if (wasInDebug)
    hart.enterDebugMode(hart.peekPc());

  return true;
}


template <typename URV>
void
Interactive<URV>::peekAllFpRegs(Hart<URV>& hart, std::ostream& out)
{
  for (unsigned i = 0; i < hart.fpRegCount(); ++i)
    {
      uint64_t val = 0;
      if (hart.peekFpReg(i, val))
	{
	  out << "f" << i << ": "
              << (boost::format("0x%016x") % val) << '\n';
	}
    }
}


template <typename URV>
void
Interactive<URV>::peekAllVecRegs(Hart<URV>& hart, std::ostream& out)
{
  for (unsigned i = 0; i < hart.vecRegCount(); ++i)
    {
      std::vector<uint8_t> val;
      if (hart.peekVecReg(i, val))
	{
	  out << "v" << i << ": 0x";
	  for (unsigned byte : val)
	    out << (boost::format("%02x") % byte);
	  out << '\n';
	}
    }
}


template <typename URV>
void
Interactive<URV>::peekAllIntRegs(Hart<URV>& hart, std::ostream& out)
{
  bool abiNames = hart.abiNames();
  auto hexForm = getHexForm<URV>(); // Format string for printing a hex val

  for (unsigned i = 0; i < hart.intRegCount(); ++i)
    {
      std::string name;
      URV val = 0;
      if (hart.peekIntReg(i, val, name))
	{
	  std::string tag = name;
	  if (abiNames)
	    tag += "(" + std::to_string(i) + ")";
	  tag += ":";

          out << (boost::format("%-9s") % tag)
              << (boost::format(hexForm) % val) << '\n';
	}
    }
}


template <typename URV>
void
Interactive<URV>::peekAllCsrs(Hart<URV>& hart, std::ostream& out)
{
  auto hexForm = getHexForm<URV>(); // Format string for printing a hex val

  out << (boost::format("%-23s") % "csr");
  if (sizeof(URV) == 4)
    out << (boost::format("%-10s %-10s %-10s %-10s\n") % "value" %
            "reset" % "mask" % "pokemask");
  else
    out << (boost::format("%-18s %-18s %-18s %-10s\n") % "value" %
            "reset" % "mask" % "pokemask");

  for (size_t i = 0; i <= size_t(CsrNumber::MAX_CSR_); ++i)
    {
      CsrNumber csr = CsrNumber(i);
      std::string name;
      URV val = 0;
      if (hart.peekCsr(csr, val, name))
	{
	  std::ostringstream oss;
	  oss << name << "(0x" << std::hex << i << "):"  << std::dec;

	  out << (boost::format("%-23s") % oss.str())
              << (boost::format(hexForm) % val);

	  URV reset = 0, writeMask = 0, pokeMask = 0;
	  if (hart.peekCsr(csr, val, reset, writeMask, pokeMask))
	    {
	      out << ' ' << (boost::format(hexForm) % reset);
	      out << ' ' << (boost::format(hexForm) % writeMask);
	      out << ' ' << (boost::format(hexForm) % pokeMask);
	    }
	  out << '\n';
	}
    }

  out << '\n';

  PrivilegeMode pm = hart.privilegeMode();
  out << "Privilege mode: ";
  switch(pm)
    {
    case PrivilegeMode::User:       out << "user\n";       break;
    case PrivilegeMode::Supervisor: out << "supervisor\n"; break;
    case PrivilegeMode::Reserved:   out << "reserved\n";   break;
    case PrivilegeMode::Machine:    out << "machine\n";    break;
    }

  out << '\n';

  out << "pmpaddr  type mode locked low                high\n";

  uint64_t low = 0, high = 0;
  Pmp::Type type = Pmp::Type::Off;
  Pmp::Mode mode = Pmp::Mode::None;
  bool locked = false;

  for (unsigned ix = 0; ix < 16; ++ix)
    {
      if (not hart.unpackMemoryProtection(ix, type, mode, locked, low, high))
        continue;

      std::string typeStr = Pmp::toString(type);
      std::string modeStr = Pmp::toString(mode);
      const char* lockStr = locked? "y" : "n";
      out << 
        (boost::format("%7d %5s %4s %6s 0x%016x 0x%016x") % ix % typeStr %
         modeStr % lockStr % low % high) << '\n';
    }
}


template <typename URV>
void
Interactive<URV>::peekAllTriggers(Hart<URV>& hart, std::ostream& out)
{
  auto hexForm = getHexForm<URV>(); // Format string for printing a hex val

  out << (boost::format("%-12s") % "trigger");
  if (sizeof(URV) == 4)
    out << (boost::format("%-10s %-10s %-10s %-10s %-10s %-10s %-10s %-10s %-10s\n") %
            "value1" % "value2" % "value3" %
            "mask1" % "mask2" % "mask3" %
            "poke-mask1" % "poke-mask2"  % "poke-mask3");
  else
    out << (boost::format("%-18s %-18s %-18s %-18s %-18s %-18s %-18s %-18s %-18s\n") %
            "value1" % "value2" % "value3" %
            "mask1" % "mask2" % "mask3" %
            "poke-mask1" % "poke-mask2"  % "poke-mask3");


  // value/reset/write-mask/poke-mask
  URV tselVal = 0, tselReset, tselWm = 0, tselPm = 0;

  if (hart.peekCsr(CsrNumber::TSELECT, tselVal, tselReset, tselWm, tselPm))
    {
      URV maxTrigger = tselWm;
      for (URV trigger = 0; trigger <= maxTrigger; ++trigger)
	{
	  uint64_t v1(0), v2(0), v3(0), wm1(0), wm2(0), wm3(0);
	  uint64_t pm1(0), pm2(0), pm3(0);

	  if (hart.peekTrigger(trigger, v1, v2, v3, wm1, wm2, wm3,
			       pm1, pm2, pm3))
	    {
	      std::string name = "trigger" + std::to_string(trigger) + ":";
	      out << (boost::format("%-11s") % name);
	      out << ' ' << (boost::format(hexForm) % v1);
	      out << ' ' << (boost::format(hexForm) % v2);
	      out << ' ' << (boost::format(hexForm) % v3);
	      out << ' ' << (boost::format(hexForm) % wm1);
	      out << ' ' << (boost::format(hexForm) % wm2);
	      out << ' ' << (boost::format(hexForm) % wm3);
	      out << ' ' << (boost::format(hexForm) % pm1);
	      out << ' ' << (boost::format(hexForm) % pm2);
	      out << ' ' << (boost::format(hexForm) % pm3);
	      out << '\n';
	    }
	  else
	    break;
	}
    }
}


template <typename URV>
static
bool
peekMemory(Hart<URV>& hart, uint64_t addr0, uint64_t addr1, std::ostream& out)
{
  auto hexForm = getHexForm<URV>(); // Format string for printing a hex val

  uint32_t word = 0;
  bool usePma = false;

  for (uint64_t addr = addr0; addr <= addr1; addr += 4)
    {
      if (not hart.peekMemory(addr, word, usePma))
        {
          std::cerr << "Peek memory address out of bounds: 0x"
                    << std::hex << addr << std::dec << '\n';
          return false;
        }
      out << (boost::format(hexForm) % addr) << ": ";
      out << (boost::format("0x%08x") % word) << std::endl;
    }

  return true;
}


template <typename URV>
bool
Interactive<URV>::peekCommand(Hart<URV>& hart, const std::string& line,
			      const std::vector<std::string>& tokens,
                              std::ostream& out)
{
  if (tokens.size() < 2)
    {
      std::cerr << "Invalid peek command: " << line << '\n';
      std::cerr << "Expecting: peek <item> <addr>  or  peek pc  or  peek all\n";
      std::cerr << "  Item is one of r, f, c, v, t , pc, m, or s for integer, floating point,\n";
      std::cerr << "  CSR, vector, trigger register, program counter, memory location, or special respectively\n";

      std::cerr << "  example:  peek r x3\n";
      std::cerr << "  example:  peek f f4\n";
      std::cerr << "  example:  peek c mtval\n";
      std::cerr << "  example:  peek c mtval v\n";
      std::cerr << "  example:  peek v v2\n";
      std::cerr << "  example:  peek m 0x4096\n";
      std::cerr << "  example:  peek t 0\n";
      std::cerr << "  example:  peek pc\n";
      std::cerr << "  example:  peek s pm\n";
      return false;
    }

  auto hexForm = getHexForm<URV>(); // Format string for printing a hex val
  URV val = 0;

  const std::string& resource = tokens.at(1);

  if (resource == "all")
    {
      out << "pc: " << (boost::format(hexForm) % hart.peekPc()) << '\n';
      out << "\n";

      peekAllIntRegs(hart, out);
      out << "\n";

      peekAllFpRegs(hart, out);
      out << "\n";

      peekAllVecRegs(hart, out);
      out << "\n";

      peekAllCsrs(hart, out);
      out << "\n";

      peekAllTriggers(hart, out);
      return true;

    }

  if (resource == "pc")
    {
      URV pc = hart.peekPc();
      out << (boost::format(hexForm) % pc) << std::endl;
      return true;
    }

  if (tokens.size() < 3)
    {
      std::cerr << "Invalid peek command: " << line << '\n';
      std::cerr << "Expecting: peek <resource> <address>\n";
      return false;
    }

  const std::string& addrStr = tokens.at(2);

  if (resource == "m")
    {
      uint64_t addr0 = 0;
      if (not parseCmdLineNumber("memory-address", addrStr, addr0))
	return false;

      uint64_t addr1 = addr0;
      if (tokens.size() >= 4)
	if (not parseCmdLineNumber("memory-address", tokens.at(3), addr1))
	  return false;

      if (tokens.size() >= 5)
        {
          std::ofstream out(tokens.at(4));
          if (not out)
            {
              std::cerr << "Failed to open " << tokens.at(4) << " for write of peek command output\n";
              return false;
            }
          return peekMemory(hart, addr0, addr1, out);
        }

      return peekMemory(hart, addr0, addr1, out);
    }

  if (resource == "r")
    {
      if (addrStr == "all")
	{
	  peekAllIntRegs(hart, out);
	  return true;
	}

      unsigned intReg = 0;
      if (not hart.findIntReg(addrStr, intReg))
	{
	  std::cerr << "No such integer register: " << addrStr << '\n';
	  return false;
	}
      if (hart.peekIntReg(intReg, val))
	{
	  out << (boost::format(hexForm) % val) << std::endl;
	  return true;
	}
      std::cerr << "Failed to read integer register: " << addrStr << '\n';
      return false;
    }

  if (resource == "f")
    {
      if (not hart.isRvf())
	{
	  std::cerr << "Floating point extension is no enabled\n";
	  return false;
	}

      if (addrStr == "all")
	{
	  peekAllFpRegs(hart, out);
	  return true;
	}

      unsigned fpReg = 0;
      if (not hart.findFpReg(addrStr, fpReg))
	{
	  std::cerr << "No such integer register: " << addrStr << '\n';
	  return false;
	}
      uint64_t fpVal = 0;
      if (hart.peekFpReg(fpReg, fpVal))
	{
	  out << (boost::format("0x%016x") % fpVal) << std::endl;
	  return true;
	}
      std::cerr << "Failed to read fp register: " << addrStr << '\n';
      return false;
    }

  if (resource == "c")
    {
      if (addrStr == "all")
	{
	  peekAllCsrs(hart, out);
	  return true;
	}

      auto csr = hart.findCsr(addrStr);
      if (not csr)
	{
	  std::cerr << "No such CSR: " << addrStr << '\n';
	  return false;
	}
      if (hart.peekCsr(csr->getNumber(), val))
	{
          std::string verbose;
          if (tokens.size() > 3)
            verbose = tokens.at(3);
          if (verbose == "v")
            {
              auto fields = csr->fields();
              if (fields.size())
                {
                  for (auto& field : fields)
                    {
                      URV mask = ((1 << field.width) - 1);
                      std::string form = "0x%0" + std::to_string((field.width / 4) + 1) + "x";
                      out << (boost::format("%-8s: " + form)
                              % field.field % (val & mask)) << std::endl;
                      val >>= field.width;
                    }
                }
            }
          else
            out << (boost::format(hexForm) % val) << std::endl;
	  return true;
	}
      std::cerr << "Failed to read CSR: " << addrStr << '\n';
      return false;
    }

  if (resource == "v")
    {
      if (not hart.isRvv())
	{
	  std::cerr << "Vector extension is no enabled\n";
	  return false;
	}

      if (addrStr == "all")
	{
	  peekAllVecRegs(hart, out);
	  return true;
	}

      unsigned vecReg = 0;
      if (not hart.findVecReg(addrStr, vecReg))
	{
	  std::cerr << "No such vector register: " << addrStr << '\n';
	  return false;
	}
      std::vector<uint8_t> data;
      if (hart.peekVecReg(vecReg, data))
	{
	  // Print data in reverse order (most significant byte first).
	  out << "0x";
	  for (unsigned byte : data)
	    out << (boost::format("%02x") % byte);
	  out << '\n';
	  return true;
	}
      std::cerr << "Failed to read vector register: " << addrStr << '\n';
      return false;
    }

  if (resource == "t")
    {
      if (addrStr == "all")
	{
	  peekAllTriggers(hart, out);
	  return true;
	}

      URV trigger = 0;
      if (not parseCmdLineNumber("trigger-number", addrStr, trigger))
	return false;
      uint64_t v1(0), v2(0), v3(0);
      if (hart.peekTrigger(trigger, v1, v2, v3))
	{
	  out << (boost::format(hexForm) % v1) << ' '
              << (boost::format(hexForm) % v2) << ' '
              << (boost::format(hexForm) % v3) << std::endl;
	  return true;
	}
      std::cerr << "Trigger number out of bounds: " << addrStr << '\n';
      return false;
    }

  if (resource == "s")
    {
      bool ok = true;
      if (addrStr == "pm")
	out << unsigned(hart.privilegeMode()) << std::endl;
      else if (addrStr == "ppm")
	out << unsigned(hart.lastPrivMode()) << std::endl;
      else if (addrStr == "iff")
	out << (boost::format("0x%x") % hart.lastFpFlags()) << std::endl;
      else if (addrStr == "trap")
	out << (hart.lastInstructionTrapped() ? "1" : "0") << std::endl;
      else
	ok = false;

      if (ok)
	return true;

      std::cerr << "Invalid special resource: " << addrStr << '\n';
      return false;
    }

  std::cerr << "No such resource: " << resource
	    << " -- expecting r, f, v, c, t, m , s, or pc\n";
  return false;
}


template <typename URV>
bool
Interactive<URV>::pokeCommand(Hart<URV>& hart, const std::string& line,
			      const std::vector<std::string>& tokens)
{
  if (tokens.size() < 3)
    {
      std::cerr << "Invalid poke command: " << line << '\n';
      std::cerr << "  Expecting: poke pc <value>\n";
      std::cerr << "    or       poke <resource> <address> <value>\n";
      std::cerr << "    or       poke t <number> <value1> <value2> <value3>\n";
      std::cerr << "  where <resource> is one of r, f, c, t, pc or m\n";
      return false;
    }

  const std::string& resource = tokens.at(1);

  uint64_t value = 0;

  if (resource == "pc")
    {
      if (not parseCmdLineNumber("pc", tokens.at(2), value))
	return false;
      hart.pokePc(value);
      return true;
    }

  size_t count = tokens.size();
  if ((resource == "t" and count != 6) or (resource != "t" and count != 4))
    {
      std::cerr << "Invalid poke command: " << line << '\n';
      std::cerr << "  Expecting: poke <resource> <address> <value>\n";
      std::cerr << "    or       poke t <number> <value1> <value2> <value3>\n";
      std::cerr << "  where <resource> is one of r, f, c, t, pc, or m\n";
      return false;
    }

  const std::string& addrStr = tokens.at(2);
  const std::string& valueStr = tokens.at(3);

  std::vector<uint8_t> vecVal;

  if (resource == "v")
    {
      if (not parseCmdLineVecData("poke", valueStr, vecVal))
	return false;
    }
  else
    {
      if (not parseCmdLineNumber("poke", valueStr, value))
	return false;
    }

  if (resource == "r")
    {
      unsigned intReg = 0;
      if (hart.findIntReg(addrStr, intReg))
	{
	  if (hart.pokeIntReg(intReg, value))
	    return true;
	  std::cerr << "Failed to write integer register " << addrStr << '\n';
	  return false;
	}

      std::cerr << "No such integer register " << addrStr << '\n';
      return false;
    }

  if (resource == "f")
    {
      unsigned fpReg = 0;
      if (hart.findFpReg(addrStr, fpReg))
	{
	  if (hart.pokeFpReg(fpReg, value))
	    return true;
	  std::cerr << "Failed to write FP register " << addrStr << '\n';
	  return false;
	}

      std::cerr << "No such FP register " << addrStr << '\n';
      return false;
    }

  if (resource == "v")
    {
      unsigned vecReg = 0;
      if (hart.findVecReg(addrStr, vecReg))
	{
	  if (hart.pokeVecReg(vecReg, vecVal))
	    return true;
	  std::cerr << "Failed to write vec register " << addrStr << '\n';
	  return false;
	}

      std::cerr << "No such vector register " << addrStr << '\n';
      return false;
    }

  if (resource == "c")
    {
      auto csr = hart.findCsr(addrStr);
      if (csr)
	{
	  if (hart.externalPokeCsr(csr->getNumber(), value))
	    return true;
	  std::cerr << "Failed to write CSR " << addrStr << '\n';
	  return false;
	}

      std::cerr << "No such CSR " << addrStr << '\n';
      return false;
    }

  if (resource == "t")
    {
      URV trigger = 0, v1 = 0, v2 = 0, v3 = 0;
      if (not parseCmdLineNumber("trigger", addrStr, trigger))
	return false;
      if (not parseCmdLineNumber("value1", tokens.at(3), v1))
	return false;
      if (not parseCmdLineNumber("value2", tokens.at(4), v2))
	return false;
      if (not parseCmdLineNumber("value3", tokens.at(5), v3))
	return false;
      if (hart.pokeTrigger(trigger, v1, v2, v3))
	return true;
      std::cerr << "Trigger out of bounds: " << addrStr << '\n';
      return false;
    }

  if (resource == "m")
    {
      size_t addr = 0;
      if (not parseCmdLineNumber("address", addrStr, addr))
	return false;
      bool usePma = false; // Ignore physicla memory attributes.
      uint32_t word = value; // Memory peek/poke in words.
      if (hart.pokeMemory(addr, word, usePma))
	return true;
      std::cerr << "Address out of bounds: " << addrStr << '\n';
      return false;
    }

  std::cerr << "No such resource: " << resource <<
    " -- expecting r, c, m or pc\n";
  return false;
}


template <typename URV>
bool
Interactive<URV>::disassCommand(Hart<URV>& hart, const std::string& line,
				const std::vector<std::string>& tokens)
{
  if (tokens.size() >= 2 and tokens.at(1) == "opcode")
    {
      for (size_t i = 2; i < tokens.size(); ++i)
	{
	  uint32_t code = 0;
	  if (not parseCmdLineNumber("opcode", tokens[i], code))
	    return false;
	  std::string str;
	  hart.disassembleInst(code, str);
	  std::cout << "  " << tokens[i] << ":  " << str << '\n';
	}
      return true;
    }

  auto hexForm = getHexForm<URV>(); // Format string for printing a hex val

  if (tokens.size() == 3 and (tokens.at(1) == "func" or
			      tokens.at(1) == "function"))
    {
      std::string item = tokens.at(2);
      std::string name;  // item (if a symbol) or function name containing item
      ElfSymbol symbol;
      if (system_.findElfSymbol(item, symbol))
	name = item;
      else
	{
	  // See if address falls in a function, then disassemble function.
	  URV addr = 0;
	  if (not parseCmdLineNumber("address", item, addr))
	    return false;

	  // Find function containing address.
	  hart.findElfFunction(addr, name, symbol);
	}

      if (name.empty())
	{
	  std::cerr << "Not a function or an address withing a function: " << item
		    << '\n';
	  return false;
	}

      std::cout << "disassemble function " << name << ":\n";

      size_t start = symbol.addr_, end = symbol.addr_ + symbol.size_;
      for (size_t addr = start; addr < end; )
	{
	  uint32_t inst = 0;
          bool usePma = false;
	  if (not hart.peekMemory(addr, inst, usePma))
	    {
	      std::cerr << "Address out of bounds: 0x" << std::hex << addr
			<< '\n' << std::dec;
	      return false;
	    }

	  unsigned instSize = instructionSize(inst);
	  if (instSize == 2)
	    inst = (inst << 16) >> 16; // Clear top 16 bits.

	  std::string str;
	  hart.disassembleInst(inst, str);
	  std::cout << "  " << (boost::format(hexForm) % addr) << ' '
		    << (boost::format(hexForm) % inst) << ' ' << str << '\n';

	  addr += instSize;
	}
      return true;
    }

  if (tokens.size() != 3)
    {
      std::cerr << "Invalid disass command: " << line << '\n';
      std::cerr << "Expecting: disass opcode <number> ...\n";
      std::cerr << "       or: disass function <name>\n";
      std::cerr << "       or: disass function <addr>\n";
      std::cerr << "       or: disass <addr1> <addr2>\n";
      return false;
    }

  URV addr1, addr2;
  if (not parseCmdLineNumber("address", tokens[1], addr1))
    return false;

  if (not parseCmdLineNumber("address", tokens[2], addr2))
    return false;

  for (URV addr = addr1; addr <= addr2; )
    {
      uint32_t inst = 0;
      bool usePma = false;
      if (not hart.peekMemory(addr, inst, usePma))
	{
	  std::cerr << "Address out of bounds: 0x" << std::hex << addr
		    << '\n' << std::dec;
	  return false;
	}

      unsigned instSize = instructionSize(inst);
      if (instSize == 2)
	inst = (inst << 16) >> 16; // Clear top 16 bits.

      std::string str;
      hart.disassembleInst(inst, str);
      std::cout << (boost::format(hexForm) % addr) << ' '
		<< (boost::format(hexForm) % inst) << ' '
		<< str << '\n';

      addr += instSize;
    }

  return true;
}


template <typename URV>
bool
Interactive<URV>::elfCommand(Hart<URV>& hart, const std::string& line,
			     const std::vector<std::string>& tokens)
{
  if (tokens.size() != 2)
    {
      std::cerr << "Invalid elf command: " << line << '\n';
      std::cerr << "Expecting: elf <file-name>\n";
      return false;
    }

  std::string filePath = tokens.at(1);

  uint64_t entryPoint = 0;

  std::vector<std::string> files = { filePath };
  if (not system_.loadElfFiles(files, false /*raw*/, false /*verbose*/))
    return false;

  hart.pokePc(URV(entryPoint));

  return true;
}


template <typename URV>
bool
Interactive<URV>::hexCommand(Hart<URV>& , const std::string& line,
			     const std::vector<std::string>& tokens)
{
  if (tokens.size() != 2)
    {
      std::cerr << "Invalid hex command: " << line << '\n';
      std::cerr << "Expecting: hex <file-name>\n";
      return false;
    }

  std::vector<std::string> fileNames = { tokens.at(1) };
  return system_.loadHexFiles(fileNames, false /*verbse*/);
}


template <typename URV>
bool
Interactive<URV>::resetCommand(Hart<URV>& hart, const std::string& /*line*/,
			       const std::vector<std::string>& tokens)
{
  if (tokens.size() == 1)
    {
      hart.reset(resetMemoryMappedRegs_);
      return true;
    }

  if (tokens.size() == 2)
    {
      URV resetPc = 0;
      if (not parseCmdLineNumber("reset-pc", tokens[1], resetPc))
	return false;

      hart.defineResetPc(resetPc);
      hart.reset(resetMemoryMappedRegs_);
      return true;
    }

  std::cerr << "Invalid reset command (extra arguments)\n";
  return false;
}


template <typename URV>
bool
Interactive<URV>::replayFileCommand(const std::string& line,
				    const std::vector<std::string>& tokens,
				    std::ifstream& stream)
{
  if (tokens.size() != 2)
    {
      std::cerr << "Invalid replay_file command: " << line << '\n';
      std::cerr << "Expecting: replay_file <file-name>\n";
      return false;
    }

  std::string fileName = tokens.at(1);

  stream.close();
  stream.open(fileName.c_str());
  if (not stream.good())
    {
      std::cerr << "Failed to open replay-file '" << fileName << "'\n";
      return false;
    }

  return true;
}


template <typename URV>
bool
Interactive<URV>::dumpMemoryCommand(const std::string& line,
                                    const std::vector<std::string>& tokens)
{
  if (tokens.size() != 2)
    {
      std::cerr << "Invalid dump_memory command: " << line << '\n';
      std::cerr << "  Expecting: dump_memory path\n";
      return false;
    }

  return system_.writeAccessedMemory(tokens[1]);
}


/// Remove from the token vector tokens of the form key=value
/// and put them in the given map (which maps a key to a value).
static
void
extractKeywords(std::vector<std::string>& tokens,
		std::unordered_map<std::string,std::string>& strMap)
{
  size_t newSize = 0;
  for (size_t i = 0; i < tokens.size(); ++i)
    {
      const auto& token = tokens.at(i);
      auto pos = token.find('=');
      if (pos != std::string::npos)
	{
	  auto key = token.substr(0, pos);
	  auto value = token.substr(pos + 1);
	  strMap[key] = value;
	}
      else
	{
	  if (newSize < i)
	    tokens.at(newSize) = token;
	  newSize++;
	}
    }
  tokens.resize(newSize);
}


/// Interactive "help" command.
static
void
printInteractiveHelp()
{
  using std::cout;
  cout << "The arguments hart=<id> and.or time=<tine> may be used with any command\n";
  cout << "to select a hart and specify event time (relevant to memory model)\n";
  cout << "They presist until explicitly changed.\n\n";
  cout << "help [<command>]\n";
  cout << "  Print help for given command or for all commands if no command given.\n\n";
  cout << "run\n";
  cout << "  Run till interrupted.\n\n";
  cout << "until <address>\n";
  cout << "  Run until address or interrupted.\n\n";
  cout << "step [<n>]\n";
  cout << "  Execute n instructions (1 if n is missing).\n\n";
  cout << "peek <res> <addr>\n";
  cout << "  Print value of resource res (one of r, f, c, v, m) and address addr.\n";
  cout << "  For memory (m) up to 2 addresses may be provided to define a range\n";
  cout << "  of memory locations to be printed; also, an optional filename after\n";
  cout << "  the two addresses writes the command output to that file.\n";
  cout << "  examples: peek r x1   peek c mtval   peek m 0x4096\n";
  cout << "            peek m 0x10 0x40 out\n\n";
  cout << "peek pc\n";
  cout << "  Print value of the program counter.\n\n";
  cout << "peek all\n";
  cout << "  Print value of all non-memory resources\n\n";
  cout << "poke res addr value\n";
  cout << "  Set value of resource res (one of r, c or m) and address addr\n";
  cout << "  Examples: poke r x1 0xff  poke c 0x4096 0xabcd\n\n";
  cout << "disass opcode <code> <code> ...\n";
  cout << "  Disassemble opcodes. Example: disass opcode 0x3b 0x8082\n\n";
  cout << "disass function <name>\n";
  cout << "  Disassemble function with given name. Example: disas func main\n\n";
  cout << "disass <addr1> <addr2>>\n";
  cout << "  Disassemble memory locations between addr1 and addr2.\n\n";
  cout << "elf file\n";
  cout << "  Load elf file into simulated memory.\n\n";
  cout << "hex file\n";
  cout << "  Load hex file into simulated memory.\n\n";
  cout << "replay_file file\n";
  cout << "  Open command file for replay.\n\n";
  cout << "replay n\n";
  cout << "  Execute the next n commands in the replay file or all the\n";
  cout << "  remaining commands if n is missing.\n\n";
  cout << "replay step n\n";
  cout << "  Execute consecutive commands from the replay file until n\n";
  cout << "  step commands are executed or the file is exhausted\n\n";
  cout << "reset [<reset_pc>]\n";
  cout << "  Reset hart.  If reset_pc is given, then change the reset program\n";
  cout << "  counter to the given reset_pc before resetting the hart.\n\n";
  cout << "symbols\n";
  cout << "  List all the symbols in the loaded ELF file(s).\n\n";
  cout << "pagetable\n";
  cout << "  Print the entries of the address tanslation table.\n\n";
  cout << "exception inst [<offset>]\n";
  cout << "  Take an instruction access fault on the subsequent step command. Given\n";
  cout << "  offset (defaults to zero) is added to the instruction PC to form the address\n";
  cout << "  responsible for the fault (that address is placed in the mtval CSR).\n\n";
  cout << "exception data [<offset>]\n";
  cout << "  Take a data access fault on the subsequent load/store instruction executed\n";
  cout << "  by a step command. The offset value is currently not used.\n\n";
  cout << "mread tag addr size data i|e\n";
  cout << "  Perform a memory model (out of order) read for load/amo instruction with\n";
  cout << "  given tag. Data is the RTL data to be compared with whisper data\n";
  cout << "  when instruction is later retired. The whisper data is obtained\n";
  cout << "  forwarding from preceding instructions if 'i' is present; otherwise,\n";
  cout << "  it is obtained from memory.\n\n";
  cout << "mbwrite addr data\n";
  cout << "  Perform a memory model merge-buffer-write for given address. Given\n";
  cout << "  data (hexadecimal string) is from a different model (RTL) and is compared\n";
  cout << "  to whisper data. Addr should be a multiple of cache-line size. If hex\n";
  cout << "  string is smaller than twice the cache-line size, it will be padded with\n";
  cout << "  zeros on the most signficant side.\n\n";
  cout << "quit\n";
  cout << "  Terminate the simulator\n\n";
}


template <typename URV>
void
Interactive<URV>::helpCommand(const std::vector<std::string>& tokens)
{
  using std::cout;

  if (tokens.size() <= 1)
    {
      printInteractiveHelp();
      return;
    }

  auto& tag = tokens.at(1);
  if (tag == "help")
    {
      cout << "help [<command>]\n"
	   << "  Print information about interactive commands. If a command\n"
	   << "  argument is given, print info about that command.\n";
      return;
    }

  if (tag == "run")
    {
      cout << "run\n"
	   << "  Run the target program until it exits (in newlib emulation mode),\n"
	   << "  it writes into the \"tohost\" location, or the user interrupts\n"
	   << "  it by pressing control-c on the keyboard.\n";
      return;
    }

  if (tag == "until")
    {
      cout << "until <address>\n"
	   << "  Same as run but the target program will also stop when the\n"
	   << "  instruction at the given address is reached (but before it is\n"
	   << "  executed).\n";
      return;
    }

  if (tag == "step")
    {
      cout << "step [<n>]\n"
	   << "  Execute a single instruction. If an integer argument <n> is\n"
	   << "  given, then execute up to n instructions or until a stop\n"
	   << "  condition (see run command) is encountered\n";
      return;
    }

  if (tag == "peek")
    {
      cout << "peek <res> <addr>\n"
	   << "peek m <addr> [<addr>] [<file>]\n"
	   << "peek pc\n"
	   << "peek s  pm | ppm | iff | trap\n"
	   << "  Show the contents of the item at the given address within the given\n"
	   << "  resource. Possible resources are r, f, c, v, m, or s for integer, FP,\n"
	   << "  CSR, vector register, memory, or special respectively. Addr stands for a\n"
	   << "  register number, register name, or memory address. If resource is\n"
	   << "  memory (m), then an additional address may be provided to define a\n"
	   << "  range of memory locations to be display and an optional filename\n"
	   << "  after 2nd address may be provided to write memory contents to a file.\n"
	   << "  Vector register values are printed just like intger register (most\n"
	   << "  significant byte first). If resource is special (s) then following\n"
	   << "  special items may be queried: pm, ppm, iff, and trap which stand for\n"
	   << "  privilege-mode, previous-privilege-mode, incremental-fp-flags, and\n"
	   << "  whether or not the last executed instruction took a trap.\n"
	   <<"   Examples:\n"
	   << "    peek pc\n"
	   << "    peek r t0\n"
	   << "    peek r x12\n"
	   << "    peek c mtval\n"
	   << "    peek v v2\n"
	   << "    peek s pm\n"
	   << "    peek m 0x80000000\n"
	   << "    peek m 0x80000000 0x80000010\n"
      	   << "    peek m 0x80000000 0x80000010 out\n";
      return;
    }

  if (tag == "poke")
    {
      cout << "poke <res> <addr> <value>\n"
	   << "poke pc <value>\n"
	   << "  Set the entry with the given address wihinin the given resource to\n"
	   << "  the given value. Possible resources are r, f, c, v, or m for integer,\n"
	   << "  FP, CSR, vector register or for memory respectively. Addr stands for\n"
	   << "  a register number, register name or memory address. Vector Register\n"
	   << "  poke values are expected in most significant byte first order.\n"
	   << "  Values of FP registers are expected in decimal or hexadcecimal notation\n"
	   << "  and they denote the bit patterns to be placed in those registers.\n"
	   << "  The memory poke unit is 1 word (4 byes).  Examples:\n"
	   << "    poke r t0 0\n"
	   << "    poke r x12 0x44\n"
	   << "    poke c mtval 0xff\n"
	   << "    poke m 0x80000000 0xabdcffff\n";
      return;
    }

  if (tag == "disas" or tag == "disass")
    {
      cout << "disass opcode <op0> <op1> ...\n"
	   << "disass func <address>\n"
	   << "disass <addr1> <addr2>\n"
	   << "  The first form will disassemble the given opcodes.\n"
	   << "  The second form will disassemble the instructions of the\n"
	   << "  function containing the given address.\n"
	   << "  The third form will disassemble the memory contents between\n"
	   << "  addresses addr1 and addr2 inclusive.\n";
      return;
    }

  if (tag == "elf")
    {
      cout << "elf <file> ...\n"
	   << "  Load into memory the contents of the given ELF file.\n"
	   << "  Set the program counter to the value of the ELF file entry point.\n"
	   << "  If the file contains the symbol \"tohost\" then subsequent writes\n"
	   << "  to the corresponding address will stop the simulation.\n";
      return;
    }

  if (tag == "replay_file")
    {
      cout << "replay_file <file> ...\n"
	   << "  Define the input replay file to serve as input for the replay\n"
	   << "  command. The user would typically load the commands of a session\n"
	   << "  and replays them in a subsequent session.\n";
      return;
    }

  if (tag == "replay")
    {
      cout << "replay [step] [<n>]\n"
	   << "  Without any arguments, replay all remaining commands in the\n"
	   << "  replay file (defined by the replay_file command).\n"
	   << "  With the keyword step, key-in on step commands in the replay\n"
	   << "  file. With an integer number n, replay n commands (or n step\n"
	   << "  commands if step keyword is present).\n";
      return;
    }

  if (tag == "reset")
    {
      cout << "reset [<reset_pc>]\n"
	   << "  Reset simulated processor. If reset_pc is given, then change\n"
	   << "  reset program counter to the given reset_pc before resetting\n"
	   << "  the processor.\n";
      return;
    }

  if (tag == "quit")
    {
      cout << "quit\n"
	   << "  Terminate the simulator.\n";
      return;
    }

  std::cerr << "No such command: " << tag	<< '\n';
}


template <typename URV>
bool
Interactive<URV>::processKeywords(const StringMap& strMap)
{
  unsigned errors = 0;
  for (const auto& kv : strMap)
    {
      const auto& key = kv.first;
      const auto& valueStr = kv.second;
      
      if (key == "hart" or key == "h")
	{
	  uint64_t val = 0;
	  if (not parseCmdLineNumber(key, valueStr, val))
	    errors++;
	  hartId_ = val;
	}
      else if (key == "time" or key == "t")
	{
	  uint64_t val = 0;
	  if (not parseCmdLineNumber(key, valueStr, val))
	    errors++;
	  time_ = val;
	}
      else
	{
	  if (key.empty())
	    std::cerr << "Empty key -- ignored\n";
	  else
	    std::cerr << "Unknown key: " << key << "  -- ignored\n";
	}
    }

  return errors == 0;
}
  

/// Command line interpreter: Execute a command line.
template <typename URV>
bool
Interactive<URV>::executeLine(const std::string& inLine, FILE* traceFile,
			      FILE* commandLog,
			      std::ifstream& replayStream, bool& done)
{
  // Remove comments (anything starting with #).
  std::string line = inLine;
  auto sharpIx = line.find_first_of('#');
  if (sharpIx != std::string::npos)
    line = line.substr(0, sharpIx);

  // Remove leading/trailing white space
  boost::algorithm::trim_if(line, boost::is_any_of(" \t"));

  if (line.empty())
    return true;

  // Break line into tokens.
  std::vector<std::string> tokens;
  boost::split(tokens, line, boost::is_any_of(" \t"),
	       boost::token_compress_on);
  if (tokens.empty())
    return true;

  StringMap strMap;
  extractKeywords(tokens, strMap);

  bool ok = processKeywords(strMap);

  if (tokens.empty())
    {
      if (ok and commandLog and not strMap.empty())
	fprintf(commandLog, "%s\n", line.c_str());
      return ok;
    }

  // If there is a quit command ececute it regardless of errors.
  const std::string& command = tokens.front();
  if (command == "q" or command == "quit")
    {
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      done = true;
      return true;
    }

  if (not ok)
    return false;

  auto hartPtr = system_.findHartByHartId(hartId_);
  if (not hartPtr)
    {
      std::cerr << "Hart id out of bounds: " << hartId_ << '\n';
      return false;
    }

  Hart<URV>& hart = *hartPtr;

  // After the first step/run/until command, a reset command will reset
  // the memory mapped registers.
  if (command == "step" or command == "run" or command == "until")
    resetMemoryMappedRegs_ = true;

  if (command == "run")
    {
      bool success = runCommand(hart, line, tokens, traceFile);
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return success;
    }

  if (command == "u" or command == "until")
    {
      bool success = untilCommand(hart, line, tokens, traceFile);
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return success;
    }

  if (command == "s" or command == "step")
    {
      if (not stepCommand(hart, line, tokens, traceFile))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "peek")
    {
      if (not peekCommand(hart, line, tokens, std::cout))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
       return true;
    }

  if (command == "poke")
    {
      if (not pokeCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "d" or command == "disas" or command == "disass")
    {
      if (not disassCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "elf")
    {
      if (not elfCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "hex")
    {
      if (not hexCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "reset")
    {
      if (not resetCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "enter_debug")
    {
      hart.enterDebugMode(hart.peekPc());
      if (commandLog)
	{
	  fprintf(commandLog, "%s", line.c_str());
	  if (tokens.size() == 1)
	    fprintf(commandLog, " false");
	  fprintf(commandLog, "\n");
	}
      return true;
    }

  if (command == "exit_debug")
    {
      hart.exitDebugMode();
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "cancel_div")
    {
      if (not hart.cancelLastDiv())
        std::cerr << "Warning: Unexpected cancel_div\n";
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "cancel_lr")
    {
      hart.cancelLr();
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "replay_file")
    {
      if (not replayFileCommand(line, tokens, replayStream))
	return false;
      return true;
    }

  if (command == "replay")
    {
      if (not replayStream.is_open())
	{
	  std::cerr << "No replay file defined. Use the replay_file to define one\n";
	  return false;
	}
      bool replayDone = false;
      if (not replayCommand(line, tokens, traceFile, commandLog,
			    replayStream, replayDone))
	return false;
      return true;
    }

  if (command == "symbols")
    {
      system_.printElfSymbols(std::cout);
      return true;
    }

  if (command == "pagetable")
    {
      hart.printPageTable(std::cout);
      return true;
    }

  if (command == "dump_memory")
    {
      if (not dumpMemoryCommand(line, tokens))
        return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "mread" or command == "memory_model_read")
    {
      if (not mReadCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "mbwrite" or command == "merge_buffer_write")
    {
      if (not mbWriteCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "mbinsert" or command == "merge_buffer_insert")
    {
      if (not mbInsertCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "translate")
    {
      if (not translateCommand(hart, line, tokens))
	return false;
      if (commandLog)
	fprintf(commandLog, "%s\n", line.c_str());
      return true;
    }

  if (command == "h" or command == "?" or command == "help")
    {
      helpCommand(tokens);
      return true;
    }

  std::cerr << "No such command: " << line << '\n';
  return false;
}


/// Interactive "replay" command.
template <typename URV>
bool
Interactive<URV>::replayCommand(const std::string& line,
				const std::vector<std::string>& tokens,
				FILE* traceFile, FILE* commandLog,
				std::ifstream& replayStream, bool& done)
{
  std::string replayLine;
  uint64_t maxCount = ~uint64_t(0);  // Unlimited

  if (tokens.size() <= 2)    // Either replay or replay n.
    {
      if (tokens.size() == 2)
	if (not parseCmdLineNumber("command-count", tokens.at(1), maxCount))
	  return false;

      uint64_t count = 0;
      while (count < maxCount  and  not done  and
	     std::getline(replayStream, replayLine))
	{
	  if (not executeLine(replayLine, traceFile,
			      commandLog, replayStream, done))
	    return false;
	  count++;
	}
      return true;
    }

  if (tokens.size() == 3)
    {
      if (tokens.at(1) != "step")
	{
	  std::cerr << "Invalid command: " << line << '\n';
	  std::cerr << "Expecting: replay <step> <count>\n";
	  return false;
	}

      if (not parseCmdLineNumber("step-count", tokens.at(2), maxCount))
	return false;
      
      uint64_t count = 0;
      while (count < maxCount  and  not done   and
	     std::getline(replayStream, replayLine))
	{
	  if (not executeLine(replayLine, traceFile,
			      commandLog, replayStream, done))
	    return false;

	  std::vector<std::string> tokens;
	  boost::split(tokens, replayLine, boost::is_any_of(" \t"),
		       boost::token_compress_on);
	  if (tokens.size() > 0 and tokens.at(0) == "step")
	    count++;
	  else if (tokens.size() > 1 and tokens.at(1) == "step")
	    count++;
	}

      return true;
    }

  std::cerr << "Invalid command: " << line << '\n';
  std::cerr << "Expecting: replay, replay <count>, or replay step <count>\n";
  return false;    
}


template <typename URV>
bool
Interactive<URV>::mReadCommand(Hart<URV>& hart, const std::string& line,
			       const std::vector<std::string>& tokens)
{
  // Format: [hart=<number>] [time=<number>] mread <instruction-tag> <physical-address> <size> <rtl-data> <i>|<é>
  if (tokens.size() != 6)
    {
      std::cerr << "Invalid mread command: " << line << '\n';
      std::cerr << "  Expecting: mread <tag> <addr> <size> <data> <i>|<e>\n";
      return false;
    }

  uint64_t tag = 0;
  if (not parseCmdLineNumber("instruction-tag", tokens.at(1), tag))
    return false;

  uint64_t addr = 0;
  if (not parseCmdLineNumber("address", tokens.at(2), addr))
    return false;

  uint64_t size = 0;
  if (not parseCmdLineNumber("size", tokens.at(3), size))
    return false;
  if (size > 8 or size == 0)
    {
      std::cerr << "Invalid size: << " << size << " -- Expecting 1 to 8\n";
      return false;
    }

  uint64_t data = 0;
  if (not parseCmdLineNumber("data", tokens.at(4), data))
    return false;

  const auto& ie = tokens.at(5);
  if (ie.empty() or (ie.at(0) != 'i' and ie.at(0) != 'e'))
    {
      std::cerr << "Invalid internal/external token: '" << ie << "' -- expecting  'i' or 'e'\n";
      return false;
    }
  bool internal = ie.at(0) == 'i';

  return system_.mcmRead(hart, this->time_, tag, addr, size, data, internal);
}


template <typename URV>
bool
Interactive<URV>::mbWriteCommand(Hart<URV>& hart, const std::string& line,
				 const std::vector<std::string>& tokens)
{
  // Format: mbwrite <physical-address> <rtl-data> [<mask>]
  // Data is up to 64 bytes (each byte is 2 hex digits) with least significant
  // byte (rightmost two hex digits) corresponding to smallest address.
  if (tokens.size() != 3 and tokens.size() != 4)
    {
      std::cerr << "Invalid mbwrite command: " << line << '\n';
      std::cerr << "  Expecting: mbwrite <addr> <data>\n";
      return false;
    }

  uint64_t addr = 0;
  if (not parseCmdLineNumber("address", tokens.at(1), addr))
    return false;

  std::vector<uint8_t> data;
  std::string hexDigits = tokens.at(2);

  if (not (boost::starts_with(hexDigits, "0x") or
	   boost::starts_with(hexDigits, "0X")))
    {
      std::cerr << "Error: mbwrite data must begin with 0x: " << hexDigits << '\n';
      return false;
    }
  hexDigits = hexDigits.substr(2);

  if ((hexDigits.size() & 1) == 1)
    {
      std::cerr << "Error: mbwrite hex digit count must be even\n";
      return false;
    }

  for (size_t i = 0; i < hexDigits.size(); i += 2)
    {
      std::string byteStr = hexDigits.substr(i, 2);
      char* end = nullptr;
      unsigned value = strtoul(byteStr.c_str(), &end, 16);
      if (end and *end)
	{
	  std::cerr << "Error: Invalid hex digit(s) in mbwrite data: "
		    << byteStr << '\n';
	  return false;
	}
      data.push_back(value);
    }

  unsigned lineSize = system_.mergeBufferSize();
  std::reverse(data.begin(), data.end()); // Least sig byte now first
  if (data.size() > lineSize)
    {
      std::cerr << "Mbwrite data too long -- truncating\n";
      data.resize(lineSize);
    }
  else if (data.size() < lineSize)
    {
      std::cerr << "Mbwrite data too short -- padding\n";
      data.resize(lineSize);
    }
		     
  std::vector<bool> mask;
  if (tokens.size() == 4)
    {
      hexDigits = tokens.at(3);
      if (not (boost::starts_with(hexDigits, "0x")) or
	  not (boost::starts_with(hexDigits, "0x")))
	{
	  std::cerr << "Error mbwrtie mask myst begin with 0x: " << hexDigits << '\n';
	  return false;
	}
      hexDigits = hexDigits.substr(2);
      if ((hexDigits.size() & 1) == 1)
	{
	  std::cerr << "Error: mbwrite hex digit count must be even\n";
	  return false;
	}
      for (size_t i = 0; i < hexDigits.size(); i += 2)
	{
	  std::string byteStr = hexDigits.substr(i, 2);
	  char* end = nullptr;
	  unsigned value = strtoul(byteStr.c_str(), &end, 16);
	  if (end and *end)
	    {
	      std::cerr << "Error: Invalid hex digit(s) in mbwrite data: "
			<< byteStr << '\n';
	      return false;
	    }
	  mask.push_back(value);
	}
    }

  return system_.mcmMbWrite(hart, this->time_, addr, data, mask);
}


template <typename URV>
bool
Interactive<URV>::mbInsertCommand(Hart<URV>& hart, const std::string& line,
				  const std::vector<std::string>& tokens)
{
  // Format: mbinsert <instr-tag> <physical-address> <size> <rtl-data>
  if (tokens.size() != 5)
    {
      std::cerr << "Invalid minsert command: " << line << '\n';
      std::cerr << "  Expecting: mbinsert <tag> <addr> size> <data>\n";
      return false;
    }

  uint64_t tag = 0;
  if (not parseCmdLineNumber("instruction-tag", tokens.at(1), tag))
    return false;

  uint64_t addr = 0;
  if (not parseCmdLineNumber("address", tokens.at(2), addr))
    return false;

  uint64_t size = 0;
  if (not parseCmdLineNumber("size", tokens.at(3), size))
    return false;
  if (size > 8 or size == 0)
    {
      std::cerr << "Invalid size: << " << size << " -- Expecting 1 to 8\n";
      return false;
    }

  uint64_t data = 0;
  if (not parseCmdLineNumber("data", tokens.at(4), data))
    return false;

  return system_.mcmMbInsert(hart, this->time_, tag, addr, size, data);
}


template <typename URV>
bool
Interactive<URV>::translateCommand(Hart<URV>& hart, const std::string& line,
				   const std::vector<std::string>& tokens)
{
  // translate va [r|w|x [s|u]]. Mode defaults to r, privilege to u.
  if (tokens.size() < 2)
    {
      std::cerr << "Invalid translate command: " << line << '\n';
      std::cerr << "Expecting: translate <vaddr> [r|w|x [s|u]]\n";
    }

  uint64_t va = 0;
  if (not parseCmdLineNumber("virtual-address", tokens.at(1), va))
    return false;

  bool read = false, write = false, exec = false;
  if (tokens.size() > 2)
    {
      if      (tokens.at(2) == "r") read  = true;
      else if (tokens.at(2) == "w") write = true;
      else if (tokens.at(2) == "x") exec  = true;
      else
	{
	  std::cerr << "Invalid protection mode: " << tokens.at(2) << " -- expecting r, w, or x\n";
	  return false;
	}
    }
  else
    read = true;

  PrivilegeMode pm = PrivilegeMode::User;
  if (tokens.size() > 3)
    {
      if      (tokens.at(3) == "u") pm = PrivilegeMode::User;
      else if (tokens.at(3) == "s") pm = PrivilegeMode::Supervisor;
      else
	{
	  std::cerr << "Invalid privilege mode: " << tokens.at(3) << " -- expecting s, or u\n";
	  return false;
	}
    }

  uint64_t pa = 0;
  auto ec = hart.transAddrNoUpdate(va, pm, read, write, exec, pa);
  if (ec == ExceptionCause::NONE)
    {
      std::cout << "0x" << std::hex << pa << std::dec << '\n';
      return true;
    }

  std::cerr << "Translation failed -- exception code: " << unsigned(ec) << '\n';
  return false;
}


template <typename URV>
bool
Interactive<URV>::interact(FILE* traceFile, FILE* commandLog)
{
  linenoise::SetHistoryMaxLen(1024);

  uint64_t errors = 0;
  hartId_ = 0;
  std::string replayFile;
  std::ifstream replayStream;

  const char* prompt = isatty(0) ? "whisper> " : "";

  auto hartPtr = system_.ithHart(0);
  if (hartPtr)
    {
      URV value = 0;
      if (hartPtr->peekCsr(CsrNumber::MHARTID, value))
        hartId_ = value;
    }

  bool done = false;
  while (not done)
    {
      errno = 0;
      std::string line = linenoise::Readline(prompt);

      if (line.empty())
	{
	  if (std::cin.eof())
	    return true;
	  continue;
	}

      linenoise::AddHistory(line.c_str());

      if (not executeLine(line, traceFile, commandLog,
			  replayStream, done))
	errors++;
    }

  return errors == 0;
}


template class WdRiscv::Interactive<uint32_t>;
template class WdRiscv::Interactive<uint64_t>;
