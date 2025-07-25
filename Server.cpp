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

#include <atomic>
#include <iostream>
#include <span>
#include <sstream>
#include <fstream>
#include <map>
#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <sys/socket.h>
#include "DecodedInst.hpp"
#include "WhisperMessage.h"
#include "Hart.hpp"
#include "Server.hpp"
#include "System.hpp"
#include "Interactive.hpp"


using namespace WdRiscv;


static bool
receiveMessage(int soc, WhisperMessage& msg)
{
  std::array<char, sizeof(msg)> buffer;
  char* p = buffer.data();

  size_t remain = buffer.size();

  while (remain > 0)
    {
      ssize_t l = recv(soc, p, remain, 0);
      if (l < 0)
	{
	  if (errno == EINTR)
	    continue;
	  std::cerr << "Error: Failed to receive socket message\n";
	  return false;
	}
      if (l == 0)
	{
	  msg.type = Quit;
	  return true;
	}
      remain -= l;
      p += l;
    }

  msg = WhisperMessage::deserializeFrom(buffer);

  return true;
}


static bool
sendMessage(int soc, const WhisperMessage& msg)
{
  std::array<char, sizeof(msg)> buffer;

  msg.serializeTo(buffer);

  // Send command.
  ssize_t remain = buffer.size();
  char* p = buffer.data();
  while (remain > 0)
    {
      ssize_t l = send(soc, p, remain , MSG_NOSIGNAL);
      if (l < 0)
	{
	  if (errno == EINTR)
	    continue;
	  std::cerr << "Error: Failed to send socket command\n";
	  return false;
	}
      remain -= l;
      p += l;
    }

  return true;
}


static bool
receiveMessage(std::span<char> shm, WhisperMessage& msg)
{
  // reserve first byte for locking
  std::atomic_char* guard = (std::atomic_char*) shm.data();
  while (std::atomic_load(guard) != 's');
  // Byte alignment for WhisperMessage - get next address after guard aligned on 4-byte boundary.
  std::span<char> buffer = shm.subspan(sizeof(uint32_t) - (reinterpret_cast<uintptr_t>(shm.data()) % sizeof(uint32_t)));
  msg = WhisperMessage::deserializeFrom(buffer);
  return true;
}


static bool
sendMessage(std::span<char> shm, const WhisperMessage& msg)
{
  // reserve first byte for locking
  std::atomic_char* guard = (std::atomic_char*) shm.data();
  while (std::atomic_load(guard) != 's'); // redundant
  // Byte alignment for WhisperMessage - get next address after guard aligned on 4-byte boundary.
  std::span<char> buffer = shm.subspan(sizeof(uint32_t) - (reinterpret_cast<uintptr_t>(shm.data()) % sizeof(uint32_t)));
  msg.serializeTo(buffer);
  std::atomic_store(guard, 'c');
  return true;
}


template <typename URV>
Server<URV>::Server(System<URV>& system)
  : system_(system)
{
  // In server mode the test-bench will issue a cancel-lr explcitly for wrs instructions.
  for (unsigned i = 0; i < system.hartCount(); ++i)
    {
      auto& hart = *(system_.ithHart(i));
      hart.setWrsCancelsLr(false);
    }
}


template <typename URV>
bool
Server<URV>::pokeCommand(const WhisperMessage& req, WhisperMessage& reply, Hart<URV>& hart)
{
  reply = req;

  switch (req.resource)
    {
    case 'r':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	URV val = static_cast<URV>(req.value);
	if (reg == req.address)
	  if (hart.pokeIntReg(reg, val))
	    return true;
      }
      break;

    case 'f':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	uint64_t val = req.value;
	if (reg == req.address)
	  if (hart.pokeFpReg(reg, val))
	    return true;
      }
      break;

    case 'c':
      {
	URV val = static_cast<URV>(req.value);
        bool virtMode = WhisperFlags{req.flags}.bits.virt;
	if (hart.externalPokeCsr(CsrNumber(req.address), val, virtMode))
	  return true;
      }
      break;

    case 'v':
      {
        unsigned reg = static_cast<unsigned>(req.address);
        // vector reg poke uses the buffer instead
        if (reg == req.address and req.size <= req.buffer.size())
          {
            std::vector<uint8_t> vecVal;
            for (uint32_t i = 0; i < req.size; ++i)
              vecVal.push_back(req.buffer[i]);
            std::reverse(vecVal.begin(), vecVal.end());
            if (hart.pokeVecReg(reg, vecVal))
              return true;
          }
      }
      break;

    case 'm':
      {
        bool usePma = false; // Ignore phsical memory attributes.
        bool cache = WhisperFlags{req.flags}.bits.cache;
        bool skipMem = WhisperFlags{req.flags}.bits.skipMem;

	if (req.size == 0)
	  {
	    // Default size is 4 bytes.
	    if (hart.pokeMemory(req.address, uint32_t(req.value), usePma, false, not cache, skipMem))
	      return true;
	  }
	else
	  {
	    switch (req.size)
	      {
	      case 1:
		if (hart.pokeMemory(req.address, uint8_t(req.value), usePma, false, not cache, skipMem))
		  return true;
		break;
	      case 2:
		if (hart.pokeMemory(req.address, uint16_t(req.value), usePma, false, not cache, skipMem))
		  return true;
		break;
	      case 4:
		if (hart.pokeMemory(req.address, uint32_t(req.value), usePma, false, not cache, skipMem))
		  return true;
		break;
	      case 8:
		if (hart.pokeMemory(req.address, uint64_t(req.value), usePma, false, not cache, skipMem))
		  return true;
		break;
	      default:
		break;
	      }
	  }
      }
      break;

    case 'p':
      {
	URV val = static_cast<URV>(req.value);
	hart.pokePc(val);
	return true;
      }
      break;

    case 's':
      {
        bool ok = true;
        URV val = static_cast<URV>(req.value);
        if (req.address == WhisperSpecialResource::DeferredInterrupts)
          hart.setDeferredInterrupts(val);
        else if (req.address == WhisperSpecialResource::Seipin)
	  hart.setSeiPin(val);
	else
          ok = false;
        if (ok)
          return true;
        break;
      }
    }

  reply.type = Invalid;
  return false;
}


template <typename URV>
bool
Server<URV>::peekCommand(const WhisperMessage& req, WhisperMessage& reply, Hart<URV>& hart)
{
  reply = req;

  URV value;

  switch (req.resource)
    {
    case 'r':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	if (reg == req.address)
	  if (hart.peekIntReg(reg, value))
	    {
	      reply.value = value;
	      return true;
	    }
      }
      break;
    case 'f':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	uint64_t fpVal = 0;
	if (reg == req.address)
	  if (hart.peekFpReg(reg, fpVal))
	    {
	      reply.value = fpVal;
	      return true;
	    }
      }
      break;
    case 'c':
      {
	URV reset = 0, mask = 0, pokeMask = 0, readMask = 0;
        bool virtMode = WhisperFlags{req.flags}.bits.virt;
	if (hart.peekCsr(CsrNumber(req.address), value, reset, mask, pokeMask, readMask, virtMode))
	  {
	    reply.value = value;
	    reply.address = mask;
	    reply.time = pokeMask;
            reply.instrTag = readMask;
	    return true;
	  }
      }
      break;
    case 'v':
      {
        unsigned reg = static_cast<unsigned>(req.address);
        if (reg == req.address)
          {
            std::vector<uint8_t> vecVal;
            if (hart.peekVecReg(reg, vecVal) and reply.buffer.size() >= vecVal.size())
              {
                std::reverse(vecVal.begin(), vecVal.end());
                for (unsigned i = 0; i < vecVal.size(); ++i)
                  reply.buffer[i] = vecVal.at(i);
                return true;
              }
          }
      }
      break;
    case 'm':
      if (hart.peekMemory(req.address, value, false /*usePma*/))
	{
	  reply.value = value;
	  return true;
	}
      break;
    case 'p':
      reply.value = hart.peekPc();
      return true;
    case 's':
      {
	switch(req.address)
	  {
	  case WhisperSpecialResource::PrivMode:
	    reply.value = unsigned(hart.privilegeMode());
	    return true;
	  case WhisperSpecialResource::PrevPrivMode:
	    reply.value = unsigned(hart.lastPrivMode());
	    return true;
	  case WhisperSpecialResource::FpFlags:
	    reply.value = hart.lastFpFlags();
	    return true;
      case WhisperSpecialResource::IncrementalVec:
        {
          std::vector<uint8_t> fpFlags; std::vector<uint8_t> vxsat;
          std::vector<VecRegs::Step> steps;
          hart.lastIncVec(fpFlags, vxsat, steps);
          assert((fpFlags.size() != 0 and vxsat.size() == 0) or
                 (fpFlags.size() == 0 and vxsat.size() != 0));
          for (unsigned i = 0; i < fpFlags.size(); ++i)
            reply.buffer[i] = fpFlags.at(i);
          for (unsigned i = 0; i < vxsat.size(); ++i)
            reply.buffer[i] = vxsat.at(i);
          return true;
        }
	  case WhisperSpecialResource::Trap:
	    reply.value = hart.lastInstructionTrapped()? 1 : 0;
	    return true;
	  case WhisperSpecialResource::DeferredInterrupts:
	    reply.value = hart.deferredInterrupts();
	    return true;
	  case WhisperSpecialResource::Seipin:
	    reply.value = hart.getSeiPin();
	    return true;
      case WhisperSpecialResource::EffMemAttr:
        // Special resource so we don't have to re-translate.
        {
        uint64_t va = 0, pa = 0;
        if (hart.lastLdStAddress(va, pa))
          {
            auto pma = hart.getPma(pa);
            auto& virtMem = hart.virtMem();
            auto effpbmt = virtMem.lastEffectivePbmt();
            pma = hart.overridePmaWithPbmt(pma, effpbmt);
            reply.value = pma.attributesToInt();
            return true;
          }
        else
          break;
        }
      case WhisperSpecialResource::LastLdStAddress:
        {
        uint64_t va = 0, pa = 0;
        if (hart.lastLdStAddress(va, pa))
            reply.value = pa;
        return true;
        }
	  default:
	    break;
	  }
	break;
      }
    case 'i':
      {
        uint32_t inst;
        if (hart.readInst(req.address, inst))
          {
            reply.value = inst;
            return true;
          }
      }
    }

  reply.type = Invalid;
  return true;
}


template <typename URV>
void
Server<URV>::disassembleAnnotateInst(Hart<URV>& hart,
                                     const DecodedInst& di, bool interrupted,
				     bool hasPreTrigger, bool hasPostTrigger,
				     std::string& text)
{
  hart.disassembleInst(di.inst(), text);
  if (di.isBranch())
    {
      if (hart.lastPc() + di.instSize() != hart.peekPc())
       text += " (T)";
      else
       text += " (NT)";
    }

  if (not interrupted)
    {
      uint64_t va = 0, pa = 0;
      if (hart.lastLdStAddress(va, pa))
	{
	  std::ostringstream oss;
	  oss << " [0x" << std::hex << va << "]" << std::dec;
	  text += oss.str();
	}
    }

  if (interrupted)
    text += " (interrupted)";
  else if (hasPreTrigger)
    text += " (pre-trigger)";
  else if (hasPostTrigger)
    text += " (post-trigger)";
}


template <typename URV>
void
Server<URV>::processStepChanges(Hart<URV>& hart,
				uint32_t inst,
				std::vector<WhisperMessage>& pendingChanges,
				bool interrupted, bool hasPre, bool hasPost,
				WhisperMessage& reply)
{
  // Get executed instruction.
  URV pc = hart.lastPc();

  // Add pc and instruction to reply.
  reply.type = ChangeCount;
  reply.address = pc;
  reply.resource = inst;

  // Add disassembly of instruction to reply.
  DecodedInst di;
  hart.decode(0 /*addr: fake*/, 0 /*physAddr: fake*/, inst, di);
  std::string text;
  if (disassemble_)
    disassembleAnnotateInst(hart, di, interrupted, hasPre, hasPost, text);

  strncpy(reply.buffer.data(), text.c_str(), reply.buffer.size() - 1);
  reply.buffer.back() = 0;

  // Order of changes: rfcvm (int reg, fp reg, csr, vec reg, memory, csr)

  // Collect integer register change caused by execution of instruction.
  pendingChanges.clear();
  uint64_t lastVal = 0;
  int regIx = hart.lastIntReg(lastVal);
  if (regIx > 0)
    {
      URV value = 0;
      if (hart.peekIntReg(regIx, value))
	{
	  WhisperMessage msg;
	  msg.type = Change;
	  msg.resource = 'r';
	  msg.address = regIx;
	  msg.value = value;
	  msg.size = sizeof(msg.value);
	  msg.time = lastVal;  // Re-purpose otherwise unused time field.
	  pendingChanges.push_back(msg);
	}
    }

  // Collect floating point register change.
  int fpRegIx = hart.lastFpReg(lastVal);
  if (fpRegIx >= 0)
    {
      uint64_t val = 0;
      if (hart.peekFpReg(fpRegIx, val))
	{
	  WhisperMessage msg;
	  msg.type = Change;
	  msg.resource = 'f';
	  msg.address = fpRegIx;
	  msg.value = val;
	  msg.size = sizeof(msg.value);
	  msg.time = lastVal;  // Re-purpose otherwise unused time field.
	  pendingChanges.push_back(msg);
	}
    }

  // Collect vector register change.
  unsigned groupSize = 0;
  int vecReg = hart.lastVecReg(di, groupSize);
  if (vecReg >= 0)
    {
      for (unsigned ix = 0; ix < groupSize; ++ix, ++vecReg)
	{
	  std::vector<uint8_t> vecData;
	  if (not hart.peekVecReg(vecReg, vecData))
	    assert(0 && "Error: Failed to peek vec register");

	  // Reverse bytes since peekVecReg returns most significant
	  // byte first.
	  std::reverse(vecData.begin(), vecData.end());

	  // Send a change message for each vector element starting
	  // with element zero and assuming a vector of double words
	  // (uint64_t). Last element will be padded with zeros if
	  // vector size in bytes is not a multiple of 8.
	  unsigned byteCount = vecData.size();
	  for (unsigned byteIx = 0; byteIx < byteCount; )
	    {
	      WhisperMessage msg;
	      msg.type = Change;
	      msg.resource = 'v';
	      msg.address = vecReg;

	      unsigned size = sizeof(msg.value);
	      unsigned remain = byteCount - byteIx;
	      size = std::min(size, remain);
	      msg.size = size;

	      for (unsigned i = 0; i < size; ++i)
		msg.value |= uint64_t(vecData.at(byteIx++)) << (8*i);
	      pendingChanges.push_back(msg);
	    }
	}
    }

  // Collect CSR and trigger changes.
  std::vector<CsrNumber> csrs;
  std::vector<unsigned> triggers;
  hart.lastCsr(csrs, triggers);

  // Map to keep CSRs in order and to drop duplicate entries.
  std::map<URV,URV> csrMap;

  // Collect changed CSRs and their values.
  for (CsrNumber csr : csrs)
    {
      URV value;
      // We always record the real csr number for VS/S mappings
      if (hart.peekCsr(csr, value, false))
	csrMap[URV(csr)] = value;
    }

  for (const auto& [key, val] : csrMap)
    {
      WhisperMessage msg(0, Change, 'c', key, val);
      msg.size = sizeof(msg.value);
      pendingChanges.push_back(msg);
    }

  // Collect memory change.
  uint64_t memAddr = 0, memVal = 0;
  unsigned size = hart.lastStore(memAddr, memVal);
  if (size)
    {
      WhisperMessage msg(0, Change, 'm', memAddr, memVal, size);
      pendingChanges.push_back(msg);
    }
  else
    {
      auto& info = hart.getLastVectorMemory();
      unsigned elemSize = info.elemSize_;
      if (not info.empty() and not info.isLoad_)
	for (auto& einfo : info.elems_)
	  {
	    WhisperMessage msg(0, Change, 'm', einfo.va_, einfo.data_, elemSize);
	    pendingChanges.push_back(msg);
	  }
    }

  // Add count of changes to reply.
  reply.value = pendingChanges.size();

  // The changes will be retrieved one at a time from the back of the
  // pendigChanges vector: Put the vector in reverse order. Changes
  // are retrieved using a Change request (see interactUsingSocket).
  std::reverse(pendingChanges.begin(), pendingChanges.end());
}


template <typename URV>
bool
Server<URV>::checkHartId(const WhisperMessage& req, WhisperMessage& reply)
{
  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Hart ID (" << std::dec << hartId
                << ") out of bounds\n";
      reply.type = Invalid;
      return false;
    }
  return true;
}


template <typename URV>
bool
Server<URV>::checkHart(const WhisperMessage& req, const std::string& /*command*/,
                       WhisperMessage& reply)
{
  return checkHartId(req, reply);
}


// Server mode step command.
template <typename URV>
bool
Server<URV>::stepCommand(const WhisperMessage& req, 
			 std::vector<WhisperMessage>& pendingChanges,
			 WhisperMessage& reply,
                         Hart<URV>& hart,
			 FILE* traceFile)
{
  reply = req;

  unsigned pm = unsigned(hart.privilegeMode());

  // Execute instruction. Determine if an interrupt was taken or if a
  // trigger got tripped.

  bool prevDebug = hart.inDebugMode();

  bool reenterDebug = false;  // True if we should re-enter debug after step.

  if (not hart.hasDebugParkLoop())
    {
      reenterDebug = prevDebug;
      if (prevDebug)
	hart.exitDebugMode();
    }

  uint32_t inst = 0;
  hart.readInst(hart.pc(), inst);  // In case instruction is interrupted.

  DecodedInst di;
  bool ok = true;
  // Memory consistency model support. No-op if mcm is off.
  if (system_.isMcmEnabled())
    {
      hart.setInstructionCount(req.instrTag - 1);
      hart.singleStep(di, traceFile);
      if (not di.isValid())
	assert(hart.lastInstructionCancelled());
      ok = system_.mcmRetire(hart, req.time, req.instrTag, di, hart.lastInstructionCancelled());
    }
  else
    hart.singleStep(di, traceFile);

  unsigned interrupted = hart.lastInstructionInterrupted() ? 1 : 0;
  if (not interrupted)
    inst = di.inst();

  unsigned preCount = 0, postCount = 0;
  hart.countTrippedTriggers(preCount, postCount);

  bool hasPre = preCount > 0;
  bool hasPost = postCount > 0;

  processStepChanges(hart, inst, pendingChanges, interrupted, hasPre,
		     hasPost, reply);

  // Send privilege mode (2 bits), incremental floating point flags (4 bits),
  // and trap info (1 bit), stop indicator (1 bit), interrupt (1 bit),
  // and virtual mode (1 bit).
  WhisperFlags flags;
  flags.bits.privMode = pm;
  flags.bits.fpFlags = hart.lastFpFlags();
  flags.bits.trap = hart.lastInstructionTrapped();
  flags.bits.stop = hart.hasTargetProgramFinished();
  flags.bits.interrupt = interrupted;
  flags.bits.virt = hart.lastVirtMode();
  flags.bits.debug = prevDebug;
  flags.bits.load = di.isLoad() or di.isAmo() or di.isVectorLoad();
  flags.bits.cancelled = hart.lastInstructionCancelled();
  reply.flags = flags.value;

  if (reenterDebug)
    hart.enterDebugMode(hart.peekPc());
  return ok;
}


// Server mode step command.
template <typename URV>
bool
Server<URV>::translateCommand(const WhisperMessage& req, 
			      WhisperMessage& reply)
{
  // FIXME: this needs to be updated for 2-stage translation
  reply = req;

  // Hart id must be valid. Hart must be started.
  if (not checkHart(req, "translate", reply))
    return false;

  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    return false;
  auto& hart = *hartPtr;

  uint64_t va = req.address;
  bool r = req.flags & 1, w = (req.flags & 2) != 0, x = (req.flags & 4) != 0;
  PrivilegeMode pm = (req.flags & 8) ? PrivilegeMode::Supervisor : PrivilegeMode::User;
  bool twoStage = req.flags & 16;

  uint64_t pa = 0;
  auto ec = hart.transAddrNoUpdate(va, pm, twoStage, r, w, x, pa);
  if (ec != ExceptionCause::NONE)
    {
      reply.type = Invalid;
      return false;
    }

  reply.address = pa;
  return true;
}


template <typename URV>
bool
Server<URV>::mcmReadCommand(const WhisperMessage& req, WhisperMessage& reply,
			    Hart<URV>& hart, FILE* cmdLog)
{
  bool ok = true;
  uint32_t hartId = req.hart;
  unsigned elem = uint16_t(req.resource >> 16);  // Vector element ix.
  unsigned field = uint16_t(req.resource);       // Vector element field (for segment load).

  if (req.size <= 8)
    {
      ok = system_.mcmRead(hart, req.time, req.instrTag, req.address, req.size,
			   req.value, elem, field);
      if (cmdLog)
          fprintf(cmdLog, "hart=%" PRIu32 " time=%" PRIu64 " mread %" PRIu64 " 0x%" PRIx64 " %" PRIu32 " 0x%" PRIx64 " %d %d\n",
                  hartId, req.time, req.instrTag, req.address, req.size, req.value,
		  elem, field);
    }
  else
    {
      if (req.size > req.buffer.size())
	{
	  std::cerr << "Error: Server command: McmRead data size too large: "
		    << req.size << '\n';
	  ok = false;
	}
      else
	{
	  // For speed, use double-word insert when possible, else word, else byte.
	  uint64_t size = req.size, time = req.time, tag = req.instrTag, addr = req.address;
	  if ((size & 0x7) == 0 and (addr & 0x7) == 0)
	    {
	      const uint64_t* data = reinterpret_cast<const uint64_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; i += 8, ++data)
		ok = system_.mcmRead(hart, time, tag, addr + i, 8, *data, elem, field);
	    }
	  else if ((size & 0x3) == 0 and (addr & 0x3) == 0)
	    {
	      const uint32_t* data = reinterpret_cast<const uint32_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; i += 4, ++data)
		ok = system_.mcmRead(hart, time, tag, addr + i, 4, *data, elem, field);
	    }
	  else
	    {
	      const uint8_t* data = reinterpret_cast<const uint8_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; ++i, ++data)
		ok = system_.mcmRead(hart, time, tag, addr + i, 1, *data, elem, field);
	    }

	  if (cmdLog)
	    {
	      fprintf(cmdLog, "hart=%" PRIu32 " time=%" PRIu64 " mread %" PRIu64 " 0x%" PRIx64 " %" PRIu32 " 0x",
		      hartId, req.time, req.instrTag, req.address, req.size);
	      const uint8_t* data = reinterpret_cast<const uint8_t*>(req.buffer.data());
	      for (unsigned i = req.size; i > 0; --i)
		{
		  unsigned val = data[i-1];
		  fprintf(cmdLog, "%02x", val);
		}
	      fprintf(cmdLog, " %d %d\n", elem, field);
	    }
	}
    }

  if (not ok)
    reply.type = Invalid;

  return ok;
}


template <typename URV>
bool
Server<URV>::mcmInsertCommand(const WhisperMessage& req, WhisperMessage& reply,
			      Hart<URV>& hart, FILE* cmdLog)
{
  bool ok = true;
  uint32_t hartId = req.hart;
  unsigned elem = uint16_t(req.resource >> 16);  // Vector element ix.
  unsigned field = uint16_t(req.resource);       // Vector element field (for segment store).

  if (req.size <= 8)
    {
      ok = system_.mcmMbInsert(hart, req.time, req.instrTag, req.address, req.size, req.value, elem, field);

      if (cmdLog)
	fprintf(cmdLog, "hart=%" PRIu32 " time=%" PRIu64 " mbinsert %" PRIu64 " 0x%" PRIx64 " %" PRIu32 " 0x%" PRIx64 " %d %d\n",
		hartId, req.time, req.instrTag, req.address, req.size, req.value, elem, field);
    }
  else
    {
      if (req.size > req.buffer.size())
	{
	  std::cerr << "Error: Server command: McmInsert data size too large: "
		    << req.size << '\n';
	  ok = false;
	}
      else
	{
	  // For speed, use double-word insert when possible, else word, else byte.
	  uint64_t size = req.size, time = req.time, tag = req.instrTag, addr = req.address;
	  if ((size & 0x7) == 0 and (addr & 0x7) == 0)
	    {
	      const uint64_t* data = reinterpret_cast<const uint64_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; i += 8, ++data)
		ok = system_.mcmMbInsert(hart, time, tag, addr + i, 8, *data, elem, field);
	    }
	  else if ((size & 0x3) == 0 and (addr & 0x3) == 0)
	    {
	      const uint32_t* data = reinterpret_cast<const uint32_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; i += 4, ++data)
		ok = system_.mcmMbInsert(hart, time, tag, addr + i, 4, *data, elem, field);
	    }
	  else
	    {
	      const uint8_t* data = reinterpret_cast<const uint8_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; ++i, ++data)
		ok = system_.mcmMbInsert(hart, time, tag, addr + i, 1, *data, elem, field);
	    }

	  if (cmdLog)
	    {
	      fprintf(cmdLog, "hart=%" PRIu32 " time=%" PRIu64 " mbinsert %" PRIu64 " 0x%" PRIx64 " %" PRIu32 " 0x",
		      hartId, req.time, req.instrTag, req.address, req.size);
	      const uint8_t* data = reinterpret_cast<const uint8_t*>(req.buffer.data());
	      for (unsigned i = req.size; i > 0; --i)
		{
		  unsigned val = data[i-1];
		  fprintf(cmdLog, "%02x", val);
		}
	      fprintf(cmdLog, " %d %d\n", elem, field);
	    }
	}
    }

  if (not ok)
    reply.type = Invalid;

  return ok;
}


template <typename URV>
bool
Server<URV>::mcmBypassCommand(const WhisperMessage& req, WhisperMessage& reply,
			      Hart<URV>& hart, FILE* cmdLog)
{
  bool ok = true;
  uint32_t hartId = req.hart;
  unsigned elem = uint16_t(req.resource >> 16);  // Vector element ix.
  unsigned field = uint16_t(req.resource);       // Vector element field (for segment store).

  if (req.size <= 8)
    {
      ok = system_.mcmBypass(hart, req.time, req.instrTag, req.address, req.size, req.value, elem, field, WhisperFlags{req.flags}.bits.cache);

      if (cmdLog)
	fprintf(cmdLog, "hart=%" PRIu32 " time=%" PRIu64 " mbbypass %" PRIu64 " 0x%" PRIx64 " %" PRIu32 " 0x%" PRIx64 " %d %d %d\n",
		hartId, req.time, req.instrTag, req.address, req.size, req.value, elem, field, WhisperFlags{req.flags}.bits.cache);
    }
  else
    {
      if (req.size > req.buffer.size())
	{
	  std::cerr << "Error: Server command: McmBypass data size too large: "
		    << req.size << '\n';
	  ok = false;
	}
      else
	{
	  // For speed, use double-word insert when possible, else word, else byte.
	  uint64_t size = req.size, time = req.time, tag = req.instrTag, addr = req.address;
          bool cache = WhisperFlags{req.flags}.bits.cache;
	  if ((size & 0x7) == 0 and (addr & 0x7) == 0)
	    {
	      const uint64_t* data = reinterpret_cast<const uint64_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; i += 8, ++data)
		ok = system_.mcmBypass(hart, time, tag, addr + i, 8, *data, elem, field, cache);
	    }
	  else if ((size & 0x3) == 0 and (addr & 0x3) == 0)
	    {
	      const uint32_t* data = reinterpret_cast<const uint32_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; i += 4, ++data)
		ok = system_.mcmBypass(hart, time, tag, addr + i, 4, *data, elem, field, cache);
	    }
	  else
	    {
	      const uint8_t* data = reinterpret_cast<const uint8_t*>(req.buffer.data());
	      for (unsigned i = 0; i < size and ok; ++i, ++data)
		ok = system_.mcmBypass(hart, time, tag, addr + i, 1, *data, elem, field, cache);
	    }

	  if (cmdLog)
	    {
	      fprintf(cmdLog, "hart=%" PRIu32 " time=%" PRIu64 " mbbypass %" PRIu64 " 0x%" PRIx64 " %" PRIu32 " 0x",
		      hartId, req.time, req.instrTag, req.address, req.size);
	      const uint8_t* data = reinterpret_cast<const uint8_t*>(req.buffer.data());
	      for (unsigned i = req.size; i > 0; --i)
		{
		  unsigned val = data[i-1];
		  fprintf(cmdLog, "%02x", val);
		}
	      fprintf(cmdLog, " %d %d %d\n", elem, field, WhisperFlags{req.flags}.bits.cache);
	    }
	}
    }

  if (not ok)
    reply.type = Invalid;

  return ok;
}


/// Dump all registers contents in tracefile.
template <typename URV>
static void
serverPrintFinalRegisterState(std::shared_ptr<Hart<URV>> hartPtr)
{
  std::ofstream out("issfinal.log");
  if (not out)
    return;
  Interactive<URV>::peekAllIntRegs(*hartPtr, out);
  out << "\n";
  Interactive<URV>::peekAllFpRegs(*hartPtr, out);
  out << "\n";
  Interactive<URV>::peekAllTriggers(*hartPtr, out);
  out << "\n";
  Interactive<URV>::peekAllCsrs(*hartPtr, out);
}


static
const char*
specialResourceToStr(uint64_t v)
{
  WhisperSpecialResource sr = WhisperSpecialResource(v);
  switch (sr)
    {
    case WhisperSpecialResource::PrivMode:            return "pm";
    case WhisperSpecialResource::PrevPrivMode:        return "ppm";
    case WhisperSpecialResource::FpFlags:             return "iff";
    case WhisperSpecialResource::IncrementalVec:      return "iv";
    case WhisperSpecialResource::Trap:                return "trap";
    case WhisperSpecialResource::DeferredInterrupts:  return "defi";
    case WhisperSpecialResource::Seipin:              return "seipin";
    case WhisperSpecialResource::EffMemAttr:          return "effma";
    case WhisperSpecialResource::LastLdStAddress:     return "lastldst";
    }
  return "?";
}


template <typename URV>
void
doPageTableWalk(const Hart<URV>& hart, WhisperMessage& reply)
{
  bool isInstr = reply.flags & 1;
  bool isAddr = reply.flags & 2;
  unsigned index = reply.address;

  std::vector<uint64_t> items;
  if (isAddr)
    {
      std::vector<VirtMem::WalkEntry> addrs;
      hart.getPageTableWalkAddresses(isInstr, index, addrs);
      for (auto& addr : addrs)
        if (addr.type_ == VirtMem::WalkEntry::Type::PA)
          items.push_back(std::move(addr.addr_));
    }
  else
    hart.getPageTableWalkEntries(isInstr, index, items);

  reply.size = items.size();
  if (not items.empty())
    {
      auto itemsBytes = std::as_bytes(std::span(items));
      auto replyBytes = std::as_writable_bytes(std::span(reply.buffer));

      assert(itemsBytes.size() <= replyBytes.size());

      std::copy(itemsBytes.begin(), itemsBytes.end(), replyBytes.begin());
    }
}


// Server mode loop: Receive command and send reply till a quit
// command is received. Return true on successful termination (quit
// received). Return false otherwise.
template <typename URV>
bool
Server<URV>::interact(int soc, FILE* traceFile, FILE* commandLog)
{
  while (true)
    {
      WhisperMessage msg, reply;
      if (not receiveMessage(soc, msg))
	return false;

      if (not checkHartId(msg, reply))
        return false;

      if (interact(msg, reply, traceFile, commandLog))
        return true;

      if (not sendMessage(soc, reply))
	return false;
    }

  return false;
}


template <typename URV>
bool
Server<URV>::interact(std::span<char> shm, FILE* traceFile, FILE* commandLog)
{
  while (true)
    {
      WhisperMessage msg, reply;
      if (not receiveMessage(shm, msg))
	return false;

      if (not checkHartId(msg, reply))
        return false;

      if (interact(msg, reply, traceFile, commandLog))
        return true;

      if (not sendMessage(shm, reply))
	return false;
    }

  return false;
}


template <typename URV>
bool
Server<URV>::interact(const WhisperMessage& msg, WhisperMessage& reply, FILE* traceFile, FILE* commandLog)
{
  // Initial resets do not reset memory mapped registers.
  bool resetMemoryMappedReg = false;
  reply = msg;

  std::string timeStamp = std::to_string(msg.time);

  uint32_t hartId = msg.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  assert(hartPtr);
  auto& hart = *hartPtr;

  if (msg.type == Step or msg.type == Until)
    resetMemoryMappedReg = true;

  switch (msg.type)
    {
      case Quit:
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " quit\n", hartId);
        serverPrintFinalRegisterState(hartPtr);
        return true;

      case Poke:
        if (not pokeCommand(msg, reply, hart))
          reply.type = Invalid;
        if (commandLog)
          {
            if (msg.resource == 'p')
              fprintf(commandLog, "hart=%" PRIu32 " poke pc 0x%" PRIxMAX " # ts=%s tag=%s\n",
		      hartId, uintmax_t(msg.value), timeStamp.c_str(), msg.tag.data());
            else if (msg.resource == 's')
              fprintf(commandLog, "hart=%" PRIu32 " poke s %s 0x%" PRIxMAX " # ts=%s tag=%s\n",
		      hartId, specialResourceToStr(msg.address), uintmax_t(msg.value),
		      timeStamp.c_str(), msg.tag.data());
            else if (msg.resource == 'c')
              fprintf(commandLog, "hart=%" PRIu32 " poke c 0x%" PRIxMAX " 0x%" PRIxMAX " 0x%" PRIu8 " # ts=%s tag=%s\n",
		      hartId, uintmax_t(msg.address), uintmax_t(msg.value), WhisperFlags{msg.flags}.bits.virt,
		      timeStamp.c_str(), msg.tag.data());
            else if (msg.resource == 'v')
              {
                fprintf(commandLog, "hart=%" PRIu32 " poke v 0x%" PRIxMAX " 0x",
			hartId, uintmax_t(msg.address));
                for (uint32_t i = 0; i < msg.size; ++i)
                  fprintf(commandLog, "%02x", uint8_t(msg.buffer[msg.size - 1 - i]));
                fprintf(commandLog, " # ts=%s tag=%s\n", timeStamp.c_str(), msg.tag.data());
              }
            else
	      {
		fprintf(commandLog, "hart=%" PRIu32 " poke %c 0x%" PRIxMAX " 0x%" PRIxMAX,
			hartId, msg.resource, uintmax_t(msg.address),
			uintmax_t(msg.value));
		if (msg.resource == 'm' and msg.size != 0)
		  fprintf(commandLog, " %d 0x%" PRIu8 " 0x%" PRIu8, int(msg.size),
                          WhisperFlags{msg.flags}.bits.cache,
                          WhisperFlags{msg.flags}.bits.skipMem);
                fprintf(commandLog, " # ts=%s tag=%s", timeStamp.c_str(), msg.tag.data());
		fprintf(commandLog, "\n");
	      }
          }
        break;

      case Peek:
        peekCommand(msg, reply, hart);
        if (commandLog)
          {
            if (msg.resource == 'p')
              fprintf(commandLog, "hart=%" PRIu32 " peek pc # ts=%s tag=%s\n",
                      hartId, timeStamp.c_str(), msg.tag.data());
            else if (msg.resource == 's')
              fprintf(commandLog, "hart=%" PRIu32 " peek s %s # ts=%s tag=%s\n",
                      hartId, specialResourceToStr(msg.address),
                      timeStamp.c_str(), msg.tag.data());
            else if (msg.resource == 'c')
              fprintf(commandLog, "hart=%" PRIu32 " peek c 0x%" PRIxMAX " 0x%" PRIu8 " # ts=%s tag=%s\n",
                      hartId, uintmax_t(msg.address), WhisperFlags{msg.flags}.bits.virt,
                      timeStamp.c_str(), msg.tag.data());
            else
              fprintf(commandLog, "hart=%" PRIu32 " peek %c 0x%" PRIxMAX " # ts=%s tag=%s\n",
                      hartId, msg.resource, uintmax_t(msg.address),
                      timeStamp.c_str(), msg.tag.data());
          }
        break;

      case Step:
        if (not stepCommand(msg, pendingChanges_, reply, hart, traceFile))
          reply.type = Invalid;
        if (commandLog)
          {
            if (system_.isMcmEnabled())
              fprintf(commandLog, "hart=%" PRIu32 " time=%s step 1 %" PRIuMAX "\n",
                      hartId, timeStamp.c_str(), uintmax_t(msg.instrTag));
            else
              fprintf(commandLog, "hart=%" PRIu32 " step #%" PRIuMAX " # ts=%s\n",
                      hartId, uintmax_t(hart.getInstructionCount()),
                      timeStamp.c_str());
          }
        break;

      case ChangeCount:
        reply.type = ChangeCount;
        reply.value = pendingChanges_.size();
        reply.address = hart.lastPc();
        {
          uint32_t inst = 0;
          hart.readInst(hart.lastPc(), inst);
          reply.resource = inst;
          std::string text;
          hart.disassembleInst(inst, text);
          uint32_t op0 = 0, op1 = 0, op2 = 0, op3 = 0;
          const InstEntry& entry = hart.decode(inst, op0, op1, op2, op3);
          if (entry.isBranch())
            {
              if (hart.lastPc() + instructionSize(inst) != hart.peekPc())
                text += " (T)";
              else
                text += " (NT)";
            }
          strncpy(reply.buffer.data(), text.c_str(), reply.buffer.size() - 1);
          reply.buffer.back() = 0;
        }
        break;

      case Change:
        if (pendingChanges_.empty())
          reply.type = Invalid;
        else
          {
            reply = pendingChanges_.back();
            pendingChanges_.pop_back();
          }
        break;

      case Reset:
        {
          URV addr = static_cast<URV>(msg.address);
          if (addr != msg.address)
            std::cerr << "Error: Address too large (" << std::hex
                      << msg.address << ") in reset command.\n" << std::dec;
          pendingChanges_.clear();
          if (msg.value != 0)
            hart.defineResetPc(addr);
          hart.reset(resetMemoryMappedReg);
          if (commandLog)
            {
              if (msg.value != 0)
                fprintf(commandLog, "hart=%" PRIu32 " reset 0x%" PRIxMAX " # ts=%s\n", hartId,
                        uintmax_t(addr), timeStamp.c_str());
              else
                fprintf(commandLog, "hart=%" PRIu32 " reset # ts=%s\n", hartId,
                        timeStamp.c_str());
            }
        }
        break;

      case Nmi:
	{
	  if (checkHart(msg, "nmi", reply))
	    hart.setPendingNmi(NmiCause(msg.value));
	  if (commandLog)
            fprintf(commandLog, "hart=%" PRIu32 " nmi 0x%x # ts=%s\n", hartId,
		    uint32_t(msg.value), timeStamp.c_str());
	  break;
	}

      case ClearNmi:
        {
	  if (checkHart(msg, "nmi", reply))
	    hart.clearPendingNmi();
	  if (commandLog)
            fprintf(commandLog, "hart=%" PRIu32 " clear_nmi # ts=%s\n", hartId,
		    timeStamp.c_str());
	  break;
        }

      case EnterDebug:
        {
          if (checkHart(msg, "enter_debug", reply))
            hart.enterDebugMode(hart.peekPc());
          if (commandLog)
            fprintf(commandLog, "hart=%" PRIu32 " enter_debug # ts=%s\n", hartId,
                    timeStamp.c_str());
        }
        break;

      case ExitDebug:
        if (checkHart(msg, "exit_debug", reply))
          hart.exitDebugMode();
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " exit_debug # ts=%s\n", hartId,
                  timeStamp.c_str());
        break;

      case CancelDiv:
        if (checkHart(msg, "cancel_div", reply))
          if (not hart.cancelLastDiv())
            reply.type = Invalid;
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " cancel_div # ts=%s\n", hartId,
                  timeStamp.c_str());
        break;

      case CancelLr:
        if (checkHart(msg, "cancel_lr", reply))
          hart.cancelLr(CancelLrCause::SERVER);
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " cancel_lr # ts=%s\n", hartId,
                  timeStamp.c_str());
        break;

      case DumpMemory:
        if (not system_.writeAccessedMemory(msg.buffer.data()))
          reply.type = Invalid;
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " dump_memory %s # ts=%s\n",
                  hartId, msg.buffer.data(), timeStamp.c_str());
        break;

      case McmRead:
	mcmReadCommand(msg, reply, hart, commandLog);
        break;

      case McmInsert:
	mcmInsertCommand(msg, reply, hart, commandLog);
	break;

      case McmEnd:
	system_.endMcm();
	if (commandLog)
	  fprintf(commandLog, "hart=%" PRIu32 " time=%" PRIu64 " end_mcm\n", hartId, msg.time);
	break;

      case McmWrite:
        if (msg.size > msg.buffer.size())
          {
            std::cerr << "Error: Server command: McmWrite data size too large: " << msg.size << '\n';
            reply.type = Invalid;
          }
        else
          {
            std::vector<bool> mask;
            mask.resize(msg.size);

            bool hasMask = msg.flags & 1;
            assert(hasMask);

            bool skipCheck = msg.flags & 2;

            std::vector<uint8_t> data(msg.size);
            for (size_t i = 0; i < msg.size; ++i)
              {
                data.at(i) = msg.buffer.at(i);
                if (hasMask)
                  mask.at(i) = msg.tag.at(i/8) & (1 << (i%8));
              }

            if (commandLog)
              {
                fprintf(commandLog, "hart=%" PRIu32 " time=%" PRIu64 " mbwrite 0x%" PRIx64 " 0x",
                        hartId, msg.time, msg.address);
                for (unsigned i = data.size(); i > 0; --i)
                  fprintf(commandLog, "%02x", data.at(i-1));
                // Print mask with least sig digit on the right
                fprintf(commandLog, " 0x");
                unsigned n = msg.size / 8;
                for (unsigned i = 0; i < n; ++i)
                  fprintf(commandLog, "%02x", unsigned(msg.tag.at(n-1-i)) & 0xff);
                if (skipCheck)
                  fprintf(commandLog, skipCheck? " 1" : " 0");
                fprintf(commandLog, "\n");
              }

            if (not system_.mcmMbWrite(hart, msg.time, msg.address, data, mask, skipCheck))
              reply.type = Invalid;
          }
        break;

      case McmBypass:
	mcmBypassCommand(msg, reply, hart, commandLog);
        break;

      case McmIFetch:
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " time=%" PRIu64 " mifetch 0x%" PRIx64 "\n",
                  hartId, msg.time, msg.address);
        if (not system_.mcmIFetch(hart, msg.time, msg.address))
          reply.type = Invalid;
	break;

      case McmIEvict:
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " time=%" PRIu64 " mievict 0x%" PRIx64 "\n",
                  hartId, msg.time, msg.address);
        if (not system_.mcmIEvict(hart, msg.time, msg.address))
          reply.type = Invalid;
	break;

      case McmDFetch:
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " time=%" PRIu64 " mdfetch 0x%" PRIx64 "\n",
                  hartId, msg.time, msg.address);
        if (not system_.mcmDFetch(hart, msg.time, msg.address))
          reply.type = Invalid;
	break;

      case McmDEvict:
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " time=%" PRIu64 " mdevict 0x%" PRIx64 "\n",
                  hartId, msg.time, msg.address);
        if (not system_.mcmDEvict(hart, msg.time, msg.address))
          reply.type = Invalid;
	break;

      case McmDWriteback:
          {
            std::vector<uint8_t> data(msg.size);
            for (size_t i = 0; i < msg.size; ++i)
              data.at(i) = msg.buffer.at(i);

            if (commandLog)
              {
                fprintf(commandLog, "hart=%" PRIu32 " time=%" PRIu64 " mdwriteback 0x%" PRIx64,
                        hartId, msg.time, msg.address);
                if (data.size() > 0)
                  fprintf(commandLog, " 0x");
                for (unsigned i = data.size(); i > 0; --i)
                  fprintf(commandLog, "%02x", data.at(i-1));
                fprintf(commandLog, "\n");
              }
            if (not system_.mcmDWriteback(hart, msg.time, msg.address, data))
              reply.type = Invalid;
          }
	break;

      case McmSkipReadChk:
        if (commandLog)
          fprintf(commandLog, "hart=%" PRIu32 " mskipreadchk 0x%" PRIx64 " 0x%" PRIx32 " %" PRIu64 "\n",
                  hartId, msg.address, msg.size, msg.value);
        if (not system_.mcmSkipReadDataCheck(msg.address, msg.size, msg.value))
          reply.type = Invalid;
        break;

      case PageTableWalk:
        doPageTableWalk(hart, reply);
        break;

      case Translate:
        translateCommand(msg, reply);
        if (commandLog)
          {
            auto flags = msg.flags;
            const char* rwx = "r";
            if (flags & 1) rwx = "r";
            else if (flags & 2) rwx = "w";
            else if (flags & 4) rwx = "x";
            const char* su = (flags & 8) ? "s" : "u";
            fprintf(commandLog, "hart=%" PRIu32 " translate 0x%" PRIxMAX " %s %s\n", hartId,
                    uintmax_t(msg.address), rwx, su);
          }
        break;

      case CheckInterrupt:
        {
	  // We want to check for interrupts regardless of deferral.
	  URV deferred = hart.deferredInterrupts();
	  hart.setDeferredInterrupts(0);

          InterruptCause cause = InterruptCause{0};
          PrivilegeMode nextMode; bool nextVirt;
          reply.flags = hart.isInterruptPossible(cause, nextMode, nextVirt);
          reply.value = static_cast<uint64_t>(cause);

	  hart.setDeferredInterrupts(deferred);

          if (commandLog)
            fprintf(commandLog, "hart=%" PRIu32 " check_interrupt\n", hartId);
        }
        break;

      case PmpEntry:
        {
          auto pmp = hart.getPmp(msg.address);

          reply.flags = pmp.isRead(PrivilegeMode::Machine);
          reply.flags |= (pmp.isWrite(PrivilegeMode::Machine) << 1);
          reply.flags |= (pmp.isExec(PrivilegeMode::Machine) << 2);
          if (commandLog)
            fprintf(commandLog, "hart=%" PRIu32 " pmp 0x%" PRIx64 "\n",
                    hartId, msg.address);
          break;
        }

      case PmaEntry:
        {
          auto pma = hart.getPma(msg.address);

          reply.flags = uint32_t(pma.isRead());
          reply.flags |= (uint32_t(pma.isWrite()) << 1);
          reply.flags |= (uint32_t(pma.isExec()) << 2);
          reply.flags |= (uint32_t(pma.isIdempotent()) << 3);
          reply.flags |= (uint32_t(pma.isAmo()) << 4);
          reply.flags |= (uint32_t(pma.isRsrv()) << 5);
          reply.flags |= (uint32_t(pma.isIo()) << 6);
          reply.flags |= (uint32_t(pma.isCacheable()) << 7);
          reply.flags |= (uint32_t(pma.isMisalignedOk()) << 8);
          if (commandLog)
            fprintf(commandLog, "hart=%" PRIu32 " pma 0x%" PRIx64 "\n",
                    hartId, msg.address);
          break;
        }

      case InjectException:
        {
          // This won't work correctly for segmented vector loads with partial segment
          // completion.
          hart.injectException(WhisperFlags(msg.flags).bits.load, msg.address, msg.resource,
                               msg.value);
          if (commandLog)
            fprintf(commandLog, "hart=%" PRIu32 " inject_exception 0x%" PRIxMAX " 0x%" PRIxMAX " 0x%" PRIxMAX " 0x%" PRIxMAX "\n", hartId,
              uintmax_t(WhisperFlags(msg.flags).bits.load), uintmax_t(msg.address), uintmax_t(msg.resource), msg.value);
          break;
        }

      default:
        std::cerr << "Error: Unknown command\n";
        reply.type = Invalid;
    }

  return false;
}


template class WdRiscv::Server<uint32_t>;
template class WdRiscv::Server<uint64_t>;
