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

#include <vector>
#include <string>
#include <unordered_map>
#include "InstId.hpp"
#include "Isa.hpp"


namespace WdRiscv
{

  enum class OperandType { IntReg, FpReg, CsReg, VecReg, Imm, None };
  enum class OperandMode { Read, Write, ReadWrite, None };
  enum class RvFormat { R, R4, I, S, B, U, J, None };

  /// Return true if given instruction is a 4-byte instruction.
  inline bool
  isFullSizeInst(uint32_t inst)
  { return (inst & 3) == 3; }


  /// Return true if given instruction is a compressed instruction.
  inline bool
  isCompressedInst(uint32_t inst)
  { return (inst & 3) != 3; }


  /// Return the size of the given instruction (2 or 4) based on its
  /// opcode.
  inline unsigned
  instructionSize(uint32_t inst)
  { return (inst & 3) == 3 ? 4 : 2; }


  ///
  /// Generic information about an instruction including, opcode,
  /// type (integer, floatin-pointg, etc...), operand count, operand type,
  /// and operand direction (source versus destination). This is used to
  /// represent an entry in the instruction table defining the RISCV
  /// instruction set arcitecture.
  ///
  /// An instruction may have up to 4 operands: op0, op1, op2, op3:
  /// - For instructions of the form "inst rd, rs1, rs2", rd, rs1 and
  ///   rs2 correspond to op0, op1 and op2 respectively.
  /// - For instructions of the form "inst rd, rs1, immediate", rd,
  ///   rs1 and immediate correspond to op0, op1 and op2 respectively.
  /// - For load instructions (e.g. load rd, offset(rs1)), rd, rs1 and
  ///   offset correspond to op0, op1, and op2 respectively.
  /// - For store instructions (e.g. store rs2, offset(rs1)), rs2, rs1
  ///   and offset correspond to op0, op1, and op2 respectively.
  ///
  class InstEntry
  {
  public:

    friend class InstTable;

    // Constructor.
    InstEntry(std::string name = "", InstId id = InstId::illegal,
	      uint32_t code = 0, uint32_t mask = ~0,
	      RvExtension ext = RvExtension::I,
	      RvFormat fmt = RvFormat::None,
	      OperandType op0Type = OperandType::None,
	      OperandMode op0Mode = OperandMode::None,
	      uint32_t op0Mask = 0,
	      OperandType op1Type = OperandType::None,
	      OperandMode op1Mode = OperandMode::None,
	      uint32_t op1Mask = 0,
	      OperandType op2Type = OperandType::None,
	      OperandMode op2Mode = OperandMode::None,
	      uint32_t op2Mask = 0,
	      OperandType op3Type = OperandType::None,
	      OperandMode op3Mode = OperandMode::None,
	      uint32_t op3Mask = 0);


    /// Return the name of the instruction.
    std::string_view name() const { return name_; }

    /// Return the id of the instruction (an integer between 0 and n
    /// where n is the number of defined instructions). Note that it is
    /// possible for two instructions with the same code to have
    /// different ids. This is because RISCV has instruction alias:
    /// same code corresponds to different instruction depending on the
    /// feature set and mode of the processor.
    InstId instId() const
    { return id_; }

    /// Return the instruction bits with all the operand specifiers set
    /// to zero.
    uint32_t code() const
    { return code_; }

    /// Return the mask corresponding to the code bis: Returned value
    /// has a 1 for each non-operand-specifier bit.
    uint32_t codeMask() const
    { return codeMask_; }

    /// Return valid operand count
    unsigned operandCount() const
    { return opCount_; }

    // Return the type of the ith operand or None if no such operand.
    // First operand corresponds to an index of zero.
    OperandType ithOperandType(unsigned i) const
    {
      if (i == 0) return op0Type_;
      if (i == 1) return op1Type_;
      if (i == 2) return op2Type_;
      if (i == 3) return op3Type_;
      return OperandType::None;
    }

    // Return the mode of the ith operand of None if no such operand.
    // First operand corresponds to an index of zero.
    OperandMode ithOperandMode(unsigned i) const
    {
      if (i == 0) return op0Mode_;
      if (i == 1) return op1Mode_;
      if (i == 2) return op2Mode_;
      if (i == 3) return op3Mode_;
      return OperandMode::None;
    }

    /// Return true if the ith operand is a write operand.
    bool isIthOperandWrite(unsigned i) const
    {
      OperandMode mode = ithOperandMode(i);
      return mode == OperandMode::Write or mode == OperandMode::ReadWrite;
    }

    /// Return true if the ith operand is a read operand.
    bool isIthOperandRead(unsigned i) const
    {
      OperandMode mode = ithOperandMode(i);
      return mode == OperandMode::Read or mode == OperandMode::ReadWrite;
    }

    /// Return the mask corresponding to the bits of the specifier of the
    /// ith operand. Return 0 if no such operand.
    uint32_t ithOperandMask(unsigned i) const
    {
      if (i == 0) return op0Mask_;
      if (i == 1) return op1Mask_;
      if (i == 2) return op2Mask_;
      if (i == 3) return op3Mask_;
      return 0;
    }

    /// Return true if ith operand is an integer register and is a source.
    bool isIthOperandIntRegSource(unsigned i) const
    {
      if (ithOperandType(i) != OperandType::IntReg)
	return false;
      return ithOperandMode(i) == OperandMode::Read;
    }

    /// Return true if ith operand is an integer register and is a destination.
    bool isIthOperandIntRegDest(unsigned i) const
    {
      if (ithOperandType(i) != OperandType::IntReg)
	return false;
      return ithOperandMode(i) == OperandMode::Write;
    }

    /// Return true if ith operand is a floating point register and is
    /// a source.
    bool isIthOperandFpRegSource(unsigned i) const
    {
      if (ithOperandType(i) != OperandType::FpReg)
	return false;
      return ithOperandMode(i) == OperandMode::Read;
    }

    /// Return the extension containing this instruction. Compressed
    /// insructions (e.g. c.fld) return the secondary extension (RvExtension::D
    /// for c.fld).
    RvExtension extension() const
    { return ext_; }

    /// Return the RISCV instruction format.
    RvFormat format() const
    { return fmt_; }

    /// Return true if this is a load instruction (lb, lh, flw, lr ...)
    bool isLoad() const
    { return isLoad_; }

    /// Return true if this is a load instruction (lb, lh, flw, lr ...)
    bool isLoad(bool& isUnsigned) const
    {
      if (not isLoad_)
	return false;
      if (isHypervisor())
	isUnsigned = (code_ >> 20) & 1;
      else
	{
	  unsigned f3 = (code_ >> 12) & 7;
	  isUnsigned = (f3 & 4) == 4;
	}
      return true;
    }

    /// Return true if this is a store instruction (sb, sh, fsw, sc ...)
    bool isStore() const
    { return isStore_; }

    /// Return true if this instruction is viewed as a load by the
    /// performance counters. By default LR is not a perf-load
    /// instuctions. Also by default FP loads are not perf-loads.
    bool isPerfLoad() const
    { return isPerfLoad_; }

    /// Return true if this instruction is viewed as a store by the
    /// performance counters. By default SC is not a perf-store
    /// instuctions. Also by default FP stores are not perf-stores.
    bool isPerfStore() const
    { return isPerfStore_; }

    /// Return true if this is a branch instruction (beq, jal, ...)
    bool isBranch() const
    { return isBranch_; }

    /// Return true if this is a multiply instruction (mul, mulh, ...)
    bool isMultiply() const
    { return ext_ == RvExtension::M and not isDivide(); }

    /// Return true if this is a divide instruction (div, rem, ...)
    bool isDivide() const
    { return isDiv_; }

    /// Return true if a floating point instruction (fadd.s, fadd.d ...)
    bool isFp() const
    { return ext_ == RvExtension::F or ext_ == RvExtension::D or
	ext_ == RvExtension::Zfh or ext_ == RvExtension::Zfbfmin or
	ext_ == RvExtension::Zfa; }

    /// Return true if this is a CSR instruction.
    bool isCsr() const
    { return id_ >= InstId::csrrw and id_ <= InstId::csrrci; }

    /// Return true if this is an atomic instruction.
    bool isAtomic() const
    { return ext_ == RvExtension::A or ext_ == RvExtension::Zacas; }

    /// Return true if this is a hypervisor instruction.
    bool isHypervisor() const
    { return ext_ == RvExtension::H; }

    /// Return true if this a compressed instruction.
    bool isCompressed() const
    { return isCompressedInst(code_); }

    /// Return true if this a load-reserve.
    bool isLr() const
    { return id_ == InstId::lr_w or id_ == InstId::lr_d; }

    /// Return true if this a store-conditional.
    bool isSc() const
    { return id_ == InstId::sc_w or id_ == InstId::sc_d; }

    /// Return true if this is an amo instruction (lr/sc are atomic but not amo).
    bool isAmo() const
    { return isAtomic() and not isLr() and not isSc(); }

    /// Return true if this is an vector instruction.
    bool isVector() const
    { return isVector_; }

    /// Return true if this is a CMO instruction
    bool isCmo() const
    { return ext_ == RvExtension::Zicbom or
             ext_ == RvExtension::Zicboz or
             ext_ == RvExtension::Zicbop; }

    /// Return true if source operands have unsigned integer values.
    bool isUnsigned() const
    { return isUns_; }

    /// Return true if this is a branch instruction where the target
    /// address is in a register.
    bool isBranchToRegister() const
    { return isRegBranch_; }

    /// Return true if this is a conditional branch instruction.
    bool isConditionalBranch() const
    { return isCond_; }

    /// Return true if this is a bit manipulation instruction.
    bool isBitManipulation() const
    { return isBitManip_; }

    /// Return the data size in bytes of a load instruction. Return
    /// zero for a non-load instruction.
    unsigned loadSize() const
    { return ldSize_; }

    /// Return the data size in bytes of a store instruction. Return zero for a non-store
    /// instruction.
    unsigned storeSize() const
    { return stSize_; }

    /// Return the data size in bytes of an AMO instruction (excluding ld/sc). Return zero
    /// for a non-store instruction.
    unsigned amoSize() const
    {
      if (not isAmo())
	return 0;
      return ((code_ >> 12) & 7) == 2 ? 4 : 8;
    }

    /// Return the size with which the immediate bits are to be (left)shifted
    unsigned immediateShiftSize() const 
    { return immedShiftSize_; }

    /// Return true if instruction has an explicit rouning mode field.
    bool hasRoundingMode() const
    { return hasRm_; }

    /// Return true if instruction writes FFLAGS CSR.
    bool modifiesFflags() const
    { return modifiesFflags_; }

    /// Returns true if compressed instruction is an rv32 variant
    bool isCompressedRv32() const
    { return isCompressedRv32_; }

    /// Returns true if compressed instruction is an rv64 variant
    bool isCompressedRv64() const
    { return isCompressedRv64_; }

  protected:

    /// Mark instruction as having a rounding mode field.
    void setHasRoundingMode(bool flag)
    { hasRm_ = flag; }

    /// Mark instruction as modifying FFLAGS.
    void setModifiesFflags(bool flag)
    { modifiesFflags_ = flag; }

    /// Mark instruction as having unsigned source operands.
    void setIsUnsigned(bool flag)
    { isUns_ = flag; }

    /// Mark instruction as a divide/remainder instruction.
    void setIsDivide(bool flag)
    { isDiv_ = flag; }

    /// Set the size of load instructions.
    void setLoadSize(unsigned size)
    { ldSize_ = size; isLoad_ = true; isPerfLoad_ = true; }

    /// Set the size of store instructions.
    void setStoreSize(unsigned size)
    { stSize_ = size; isStore_ = true; isPerfStore_ = true; }

    /// Set the shift size of immediate val
    void setImmedShiftSize(unsigned size)
    { immedShiftSize_ = size; }

    /// Mark as a conditional branch instruction.
    void setConditionalBranch(bool flag)
    { isBranch_ = flag; isCond_ = flag; }

    /// Mark as a branch to register instruction.
    void setBranchToRegister(bool flag)
    { isBranch_ = flag; isRegBranch_ = flag; }

    /// Mark as a branch insruction.
    void setBranch(bool flag)
    { isBranch_ = flag; }

    /// Mask as a compressed rv32 instruction
    void setCompressedRv32(bool flag)
    { isCompressedRv32_ = flag; }

    /// Mark as a compressed rv64 instruction
    void setCompressedRv64(bool flag)
    { isCompressedRv64_ = flag; }

    /// Mark an instruction as a vector instruction.
    void setVector(bool flag)
    { isVector_ = flag; }

  private:

    std::string name_;
    InstId id_;
    uint32_t code_;      // Code with all operand bits set to zero.
    uint32_t codeMask_;  // Bit corresponding to code bits are 1. Bits

    RvExtension ext_ = RvExtension::I;
    RvFormat fmt_ = RvFormat::None;

    uint32_t op0Mask_;
    uint32_t op1Mask_;
    uint32_t op2Mask_;
    uint32_t op3Mask_;

    OperandType op0Type_;
    OperandType op1Type_;
    OperandType op2Type_;
    OperandType op3Type_;

    OperandMode op0Mode_;
    OperandMode op1Mode_;
    OperandMode op2Mode_;
    OperandMode op3Mode_;

    unsigned opCount_;
    unsigned ldSize_ = 0;      // Load size: Zero for non-load.
    unsigned stSize_ = 0;      // Store size: Zero for non-store.
    unsigned immedShiftSize_ = 0; // Shift size of immediate operand (eg. lui/auipc are automatically shifted left by 12) 
    bool isUns_ = false;       // True if source operands are unsigned.
    bool isBranch_ = false;    // True if a branch instruction.
    bool isCond_ = false;      // True if conditional branch.
    bool isRegBranch_ = false; // True if branch to register.
    bool isBitManip_ = false;  // True if bit manipulation instruction.
    bool isLoad_ = false;
    bool isStore_ = false;
    bool isPerfLoad_ = false;  // True if perf counters view instr as load.
    bool isPerfStore_ = false; // True if perf counters view instr as store.
    bool hasRm_ = false;       // True if instr has an explicit rounding mode .
    bool modifiesFflags_ = false; // True if instr modifed FFLAGS.
    bool isDiv_ = false;       // True for integer divide or remainder instr.
    bool isCompressedRv32_ = false; // True if compressed rv32 instruction variant.
    bool isCompressedRv64_ = false; // True if compressed rv64 instruction variant.
    bool isVector_ = false;         // True if V extension or other vector sub-extension.
  };


  // Instruction table: Map an instruction id or an instruction name to
  // the opcode/operand information corresponding to that instruction.
  class InstTable
  {
  public:
    InstTable();

    // Return the info corresponding to the given id or the info of the
    // illegal instruction if no such id.
    const InstEntry& getEntry(InstId) const;

    // Return the info corresponding to the given name or the info of
    // the illegal instruction if no such instruction.
    const InstEntry& getEntry(std::string_view name) const;

    // Return true if given id is present in the table.
    bool hasInfo(InstId) const;

    // Return true if given instance name is present in the table.
    bool hasInfo(std::string_view name) const;

    /// Mark lr as a load instruction and sc as a store for the
    /// purpose of performance counters if flag is true; otherwise,
    /// lr and sc are not counted as load/store.
    void perfCountAtomicLoadStore(bool flag);

    /// Mark floating point load/store instructions as load/store for
    /// the purpose of performance counters if flag is true;
    /// otherwise, floating point load/store are not counted.  If flag
    /// is true, flw will count as both a load instruction and as an
    /// fp instruction.
    void perfCountFpLoadStore(bool flag);

    /// Return the instruction vector table.
    const std::vector<InstEntry>& getInstVec()
    { return instVec_; }

  private:

    // Helper to the constructor.
    void setupInstVec();

    std::vector<InstEntry> instVec_;
    std::unordered_map<std::string_view, InstId> instMap_;
  };
}
