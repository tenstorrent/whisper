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

#include <cassert>
#include "InstEntry.hpp"

using namespace WdRiscv;

InstEntry::InstEntry(std::string name, InstId id,
		     uint32_t code, uint32_t mask,
		     RvExtension ext, RvFormat fmt,
		     OperandType op0Type, OperandMode op0Mode, uint32_t op0Mask,
		     OperandType op1Type, OperandMode op1Mode, uint32_t op1Mask,
		     OperandType op2Type, OperandMode op2Mode, uint32_t op2Mask,
		     OperandType op3Type, OperandMode op3Mode, uint32_t op3Mask)
  : name_(std::move(name)), id_(id), code_(code), codeMask_(mask), ext_(ext), fmt_(fmt),
    op0Mask_(op0Mask), op1Mask_(op1Mask), op2Mask_(op2Mask), op3Mask_(op3Mask),
    op0Type_(op0Type), op1Type_(op1Type), op2Type_(op2Type), op3Type_(op3Type),
    op0Mode_(op0Mode), op1Mode_(op1Mode), op2Mode_(op2Mode), op3Mode_(op3Mode),
    opCount_(0)
{
  unsigned count = 0;

  if (op0Type != OperandType::None) count++;
  if (op1Type != OperandType::None) count++;
  if (op2Type != OperandType::None) count++;
  if (op3Type != OperandType::None) count++;
  opCount_ = count;
  isBitManip_ = ext >= RvExtension::Zba and ext <= RvExtension::Zbs;
}


InstTable::InstTable()
{
  setupInstVec();

  // Sanity check. Mark vector instructions.
  for (unsigned i = 0; InstId(i) <= InstId::maxId; ++i)
    {
      auto& entry = instVec_.at(i);
      assert(entry.instId() == InstId(i));

      auto ext = entry.extension();
      switch(ext)
	{
	case RvExtension::V:
	case RvExtension::Zvfh:
	case RvExtension::Zvfhmin:
	case RvExtension::Zvbb:
	case RvExtension::Zvbc:
	case RvExtension::Zvkg:
	case RvExtension::Zvkned:
	case RvExtension::Zvknha:
	case RvExtension::Zvknhb:
	case RvExtension::Zvksed:
	case RvExtension::Zvksh:
	case RvExtension::Zvfbfmin:
	case RvExtension::Zvfbfwma:
	case RvExtension::Zvqdot:
	  entry.setVector(true);
	  break;
	default:
	  break;
	}
    }

  for (const auto& instInfo : instVec_)
    instMap_[instInfo.name()] = instInfo.instId();

  // Mark instructions with unsigned source opreands.
  instVec_.at(size_t(InstId::bltu))     .setIsUnsigned(true);
  instVec_.at(size_t(InstId::bgeu))     .setIsUnsigned(true);
  instVec_.at(size_t(InstId::sltiu))    .setIsUnsigned(true);
  instVec_.at(size_t(InstId::sltu))     .setIsUnsigned(true);
  instVec_.at(size_t(InstId::mulhsu))   .setIsUnsigned(true);
  instVec_.at(size_t(InstId::mulhu))    .setIsUnsigned(true);
  instVec_.at(size_t(InstId::divu))     .setIsUnsigned(true);
  instVec_.at(size_t(InstId::remu))     .setIsUnsigned(true);

  // Set data size of load instructions.
  instVec_.at(size_t(InstId::lb))      .setLoadSize(1);
  instVec_.at(size_t(InstId::lh))      .setLoadSize(2);
  instVec_.at(size_t(InstId::lw))      .setLoadSize(4);
  instVec_.at(size_t(InstId::lbu))     .setLoadSize(1);
  instVec_.at(size_t(InstId::lhu))     .setLoadSize(2);
  instVec_.at(size_t(InstId::lwu))     .setLoadSize(4);
  instVec_.at(size_t(InstId::ld))      .setLoadSize(8);
  instVec_.at(size_t(InstId::lr_w))    .setLoadSize(4);
  instVec_.at(size_t(InstId::lr_d))    .setLoadSize(8);
  instVec_.at(size_t(InstId::flh))     .setLoadSize(2);
  instVec_.at(size_t(InstId::flw))     .setLoadSize(4);
  instVec_.at(size_t(InstId::fld))     .setLoadSize(8);
  instVec_.at(size_t(InstId::c_lbu))   .setLoadSize(1);
  instVec_.at(size_t(InstId::c_lhu))   .setLoadSize(2);
  instVec_.at(size_t(InstId::c_lh))    .setLoadSize(2);
  instVec_.at(size_t(InstId::c_fld))   .setLoadSize(8);
  instVec_.at(size_t(InstId::c_lq))    .setLoadSize(16);
  instVec_.at(size_t(InstId::c_lw))    .setLoadSize(4);
  instVec_.at(size_t(InstId::c_flw))   .setLoadSize(4);
  instVec_.at(size_t(InstId::c_ld))    .setLoadSize(8);
  instVec_.at(size_t(InstId::c_fldsp)) .setLoadSize(8);
  instVec_.at(size_t(InstId::c_lwsp))  .setLoadSize(4);
  instVec_.at(size_t(InstId::c_flwsp)) .setLoadSize(4);
  instVec_.at(size_t(InstId::c_ldsp))  .setLoadSize(8);
  instVec_.at(size_t(InstId::hlv_b))   .setLoadSize(1);
  instVec_.at(size_t(InstId::hlv_bu))  .setLoadSize(1);
  instVec_.at(size_t(InstId::hlv_h))   .setLoadSize(2);
  instVec_.at(size_t(InstId::hlv_hu))  .setLoadSize(2);
  instVec_.at(size_t(InstId::hlv_w))   .setLoadSize(4);
  instVec_.at(size_t(InstId::hlv_wu))  .setLoadSize(4);
  instVec_.at(size_t(InstId::hlvx_hu)) .setLoadSize(2);
  instVec_.at(size_t(InstId::hlvx_wu)) .setLoadSize(4);
  instVec_.at(size_t(InstId::hlv_d))   .setLoadSize(8);

  // Set data size of store instructions.
  instVec_.at(size_t(InstId::sb))      .setStoreSize(1);
  instVec_.at(size_t(InstId::sh))      .setStoreSize(2);
  instVec_.at(size_t(InstId::sw))      .setStoreSize(4);
  instVec_.at(size_t(InstId::sd))      .setStoreSize(8);
  instVec_.at(size_t(InstId::sc_w))    .setStoreSize(4);
  instVec_.at(size_t(InstId::sc_d))    .setStoreSize(8);
  instVec_.at(size_t(InstId::fsh))     .setStoreSize(2);
  instVec_.at(size_t(InstId::fsw))     .setStoreSize(4);
  instVec_.at(size_t(InstId::fsd))     .setStoreSize(8);
  instVec_.at(size_t(InstId::c_fsd))   .setStoreSize(8);
  instVec_.at(size_t(InstId::c_sb))    .setStoreSize(1);
  instVec_.at(size_t(InstId::c_sh))    .setStoreSize(2);
  instVec_.at(size_t(InstId::c_sw))    .setStoreSize(4);
  instVec_.at(size_t(InstId::c_fsw))   .setStoreSize(4);
  instVec_.at(size_t(InstId::c_sd))    .setStoreSize(8);
  instVec_.at(size_t(InstId::c_fsdsp)) .setStoreSize(8);
  instVec_.at(size_t(InstId::c_sq))    .setStoreSize(16);
  instVec_.at(size_t(InstId::c_swsp))  .setStoreSize(4);
  instVec_.at(size_t(InstId::c_fswsp)) .setStoreSize(4);
  instVec_.at(size_t(InstId::c_sdsp))  .setStoreSize(8);
  instVec_.at(size_t(InstId::hsv_b))   .setStoreSize(1);
  instVec_.at(size_t(InstId::hsv_h))   .setStoreSize(2);
  instVec_.at(size_t(InstId::hsv_w))   .setStoreSize(4);
  instVec_.at(size_t(InstId::hsv_d))   .setStoreSize(8);

  // Mark conditional branch instructions.
  instVec_.at(size_t(InstId::beq))    .setConditionalBranch(true);
  instVec_.at(size_t(InstId::bne))    .setConditionalBranch(true);
  instVec_.at(size_t(InstId::blt))    .setConditionalBranch(true);
  instVec_.at(size_t(InstId::bge))    .setConditionalBranch(true);
  instVec_.at(size_t(InstId::bltu))   .setConditionalBranch(true);
  instVec_.at(size_t(InstId::bgeu))   .setConditionalBranch(true);
  instVec_.at(size_t(InstId::c_beqz)) .setConditionalBranch(true);
  instVec_.at(size_t(InstId::c_bnez)) .setConditionalBranch(true);

  // Mark branch to register instructions.
  instVec_.at(size_t(InstId::jalr))   .setBranchToRegister(true);
  instVec_.at(size_t(InstId::c_jr))   .setBranchToRegister(true);
  instVec_.at(size_t(InstId::c_jalr)) .setBranchToRegister(true);

  // Mark other branch instructions.
  instVec_.at(size_t(InstId::jal))    .setBranch(true);
  instVec_.at(size_t(InstId::c_j))    .setBranch(true);

  // Mark divide instructions.
  instVec_.at(size_t(InstId::div))    .setIsDivide(true);
  instVec_.at(size_t(InstId::divu))   .setIsDivide(true);
  instVec_.at(size_t(InstId::rem))    .setIsDivide(true);
  instVec_.at(size_t(InstId::remu))   .setIsDivide(true);
  instVec_.at(size_t(InstId::divw))   .setIsDivide(true);
  instVec_.at(size_t(InstId::divuw))  .setIsDivide(true);
  instVec_.at(size_t(InstId::remw))   .setIsDivide(true);
  instVec_.at(size_t(InstId::remuw))  .setIsDivide(true);

  // Mark floating point instructions with rounding mode field.
  instVec_.at(size_t(InstId::fmadd_s))   .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fmsub_s))   .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fnmsub_s))  .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fnmadd_s))  .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fadd_s))    .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fsub_s))    .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fmul_s))    .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fdiv_s))    .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fsqrt_s))   .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_w_s))  .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_wu_s)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_s_w))  .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_s_wu)) .setHasRoundingMode(true);

  instVec_.at(size_t(InstId::fcvt_l_s))  .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_lu_s)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_s_l))  .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_s_lu)) .setHasRoundingMode(true);

  instVec_.at(size_t(InstId::fmadd_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fmsub_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fnmsub_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fnmadd_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fadd_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fsub_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fmul_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fdiv_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fsqrt_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_s_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_d_s)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_w_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_wu_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_d_w)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_d_wu)) .setHasRoundingMode(true);

  instVec_.at(size_t(InstId::fcvt_l_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_lu_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_d_l)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_d_lu)) .setHasRoundingMode(true);

  instVec_.at(size_t(InstId::fmadd_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fmsub_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fnmsub_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fnmadd_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fadd_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fsub_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fmul_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fdiv_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fsqrt_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_s_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_d_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_h_s)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_h_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_w_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_wu_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_h_w)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_h_wu)) .setHasRoundingMode(true);

  // rv64 + zfh
  instVec_.at(size_t(InstId::fcvt_l_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_lu_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_h_l)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_h_lu)) .setHasRoundingMode(true);

  // rv64 + zfa
  instVec_.at(size_t(InstId::fround_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fround_s)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fround_d)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::froundnx_h)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::froundnx_s)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::froundnx_d)) .setHasRoundingMode(true);

  // rv64 + zfbfmin
  instVec_.at(size_t(InstId::fcvt_bf16_s)) .setHasRoundingMode(true);
  instVec_.at(size_t(InstId::fcvt_s_bf16)) .setHasRoundingMode(true);

  // Mark compressed instructions which are rv32 variants
  instVec_.at(size_t(InstId::c_flw)) .setCompressedRv32(true);
  instVec_.at(size_t(InstId::c_fsw)) .setCompressedRv32(true);
  instVec_.at(size_t(InstId::c_jal)) .setCompressedRv32(true);
  instVec_.at(size_t(InstId::c_flwsp)) .setCompressedRv32(true);
  instVec_.at(size_t(InstId::c_fswsp)) .setCompressedRv32(true);

  // Mark compressed instructions which are rv64 variants
  instVec_.at(size_t(InstId::c_ld)) .setCompressedRv64(true);
  instVec_.at(size_t(InstId::c_sd)) .setCompressedRv64(true);
  instVec_.at(size_t(InstId::c_addiw)) .setCompressedRv64(true);
  instVec_.at(size_t(InstId::c_subw)) .setCompressedRv64(true);
  instVec_.at(size_t(InstId::c_addw)) .setCompressedRv64(true);
  instVec_.at(size_t(InstId::c_ldsp)) .setCompressedRv64(true);
  instVec_.at(size_t(InstId::c_sdsp)) .setCompressedRv64(true);

  // Mark instruction which have their immediate shifted by n bits
  instVec_.at(size_t(InstId::lui))   .setImmedShiftSize(12);
  instVec_.at(size_t(InstId::auipc)) .setImmedShiftSize(12);
  instVec_.at(size_t(InstId::c_lui)) .setImmedShiftSize(12);
  instVec_.at(size_t(InstId::c_addi16sp)) .setImmedShiftSize(4);
  instVec_.at(size_t(InstId::c_addi4spn)) .setImmedShiftSize(2);
  instVec_.at(size_t(InstId::c_beqz)) .setImmedShiftSize(1);
  instVec_.at(size_t(InstId::c_bnez)) .setImmedShiftSize(1);
  instVec_.at(size_t(InstId::beq)) .setImmedShiftSize(1);
  instVec_.at(size_t(InstId::blt)) .setImmedShiftSize(1);
  instVec_.at(size_t(InstId::bge)) .setImmedShiftSize(1);
  instVec_.at(size_t(InstId::bgeu)).setImmedShiftSize(1);
  instVec_.at(size_t(InstId::jal)) .setImmedShiftSize(1);
  instVec_.at(size_t(InstId::c_j)) .setImmedShiftSize(1);



  // Mark floating point instruction that modify FFLAGS.
  for (unsigned i = unsigned(InstId::flw); i <= unsigned(InstId::fcvt_h_lu); ++i)
    instVec_.at(i).setModifiesFflags(true);

  for (auto id : { InstId::flw, InstId::fsw, InstId::fsgnj_s, InstId::fsgnjn_s,
		  InstId::fsgnjx_s, InstId::fmv_x_w, InstId::fclass_s,
		  InstId::fmv_w_x, InstId::fcvt_l_s, InstId::fcvt_lu_s,
		  InstId::fcvt_s_l, InstId::fcvt_s_lu } )
    instVec_.at(unsigned(id)).setModifiesFflags(false);

  for (auto id : { InstId::fld, InstId::fsd, InstId::fsgnj_d, InstId::fsgnjn_d,
		  InstId::fsgnjx_d, InstId::fmv_x_d, InstId::fclass_d,
		  InstId::fmv_d_x, InstId::fcvt_d_w, InstId::fcvt_d_wu,
		  InstId::fcvt_l_d, InstId::fcvt_lu_d,
		  InstId::fcvt_d_l, InstId::fcvt_d_lu } )
    instVec_.at(unsigned(id)).setModifiesFflags(false);

  for (auto id : { InstId::flh, InstId::fsh, InstId::fsgnj_h, InstId::fsgnjn_h,
		  InstId::fsgnjx_h, InstId::fmv_x_h, InstId::fclass_h,
		  InstId::fmv_h_x, InstId::fcvt_h_w, InstId::fcvt_d_wu,
		  InstId::fcvt_l_h, InstId::fcvt_lu_h,
		  InstId::fcvt_d_h } )
    instVec_.at(unsigned(id)).setModifiesFflags(false);

  // Mark Zfa instructions that modify FFLAGS
  for (unsigned i = unsigned(InstId::fcvtmod_w_d); i <= unsigned(InstId::froundnx_d); ++i)
    instVec_.at(i).setModifiesFflags(true);

  for (auto id : { InstId::fli_h, InstId::fli_s, InstId::fli_d, InstId::fleq_h, InstId::fleq_s,
		   InstId::fleq_d, InstId::fltq_h, InstId::fltq_s, InstId::fltq_d,
		   InstId::fmvh_x_d, InstId::fmvp_d_x } )
    instVec_.at(unsigned(id)).setModifiesFflags(false);

  // For backward compatibility, lr and sc are not counted as load/store
  // by the performance counters.
  perfCountAtomicLoadStore(false);

  // For backward compatibility, floating point load store (flw/fsw,
  // fld/fsd ...)  instructions are not counted as load/store by the
  // performance counters.
  perfCountFpLoadStore(false);
}


const InstEntry&
InstTable::getEntry(InstId id) const
{
  if (size_t(id) >= instVec_.size())
    return instVec_.front();
  return instVec_.at(size_t(id));
}


const InstEntry&
InstTable::getEntry(std::string_view name) const
{
  const auto iter = instMap_.find(name);
  if (iter == instMap_.end())
    return instVec_.front();
  auto id = iter->second;
  return getEntry(id);
}


void
InstTable::perfCountAtomicLoadStore(bool flag)
{
  instVec_.at(size_t(InstId::lr_w)).isPerfLoad_  = flag;
  instVec_.at(size_t(InstId::lr_d)).isPerfLoad_  = flag;
  instVec_.at(size_t(InstId::sc_w)).isPerfStore_ = flag;
  instVec_.at(size_t(InstId::sc_d)).isPerfStore_ = flag;
}


void
InstTable::perfCountFpLoadStore(bool flag)
{
  instVec_.at(size_t(InstId::flh))     .isPerfLoad_ = flag;
  instVec_.at(size_t(InstId::flw))     .isPerfLoad_ = flag;
  instVec_.at(size_t(InstId::fld))     .isPerfLoad_ = flag;
  instVec_.at(size_t(InstId::c_fld))   .isPerfLoad_ = flag;
  instVec_.at(size_t(InstId::c_flw))   .isPerfLoad_ = flag;
  instVec_.at(size_t(InstId::c_fldsp)) .isPerfLoad_ = flag;
  instVec_.at(size_t(InstId::c_flwsp)) .isPerfLoad_ = flag;

  instVec_.at(size_t(InstId::fsh))     .isPerfStore_ = flag;
  instVec_.at(size_t(InstId::fsw))     .isPerfStore_ = flag;
  instVec_.at(size_t(InstId::fsd))     .isPerfStore_ = flag;
  instVec_.at(size_t(InstId::c_fsd))   .isPerfStore_ = flag;
  instVec_.at(size_t(InstId::c_fsw))   .isPerfStore_ = flag;
  instVec_.at(size_t(InstId::c_fsdsp)) .isPerfStore_ = flag;
  instVec_.at(size_t(InstId::c_fswsp)) .isPerfStore_ = flag;
}


void
InstTable::setupInstVec()
{
  uint32_t rdMask = 0x1f << 7;
  uint32_t rs1Mask = 0x1f << 15;
  uint32_t rs2Mask = 0x1f << 20;
  uint32_t rs3Mask = 0x1f << 27;
  uint32_t immTop20 = 0xfffff << 12;  // Immidiate: top 20 bits.
  uint32_t immTop12 = 0xfff << 20;   // Immidiate: top 12 bits.
  uint32_t immBeq = 0xfe000f80;
  uint32_t shamtMask = 0x01f00000;

  uint32_t low7Mask = 0x7f;                 // Opcode mask: lowest 7 bits
  uint32_t funct3Low7Mask = 0x707f;         // Funct3 and lowest 7 bits
  uint32_t fmaddMask = 0x0600007f;          // fmadd-like opcode mask.
  uint32_t faddMask = 0xfe00007f;           // fadd-like opcode mask
  uint32_t fsqrtMask = 0xfff0007f;          // fsqrt-like opcode mask
  uint32_t top7Funct3Low7Mask = 0xfe00707f; // Top7, Funct3 and lowest 7 bits

  instVec_ =
    {
      // Base instructions
      { "illegal", InstId::illegal, 0xffffffff, 0xffffffff },

      { "lui", InstId::lui, 0x37, low7Mask,
	RvExtension::I, RvFormat::U,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, immTop20 },

      { "auipc", InstId::auipc, 0x17, low7Mask,
	RvExtension::I, RvFormat::U,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, immTop20 },

      { "jal", InstId::jal, 0x6f, low7Mask,
	RvExtension::I, RvFormat::J,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, immTop20 },

      { "jalr", InstId::jalr, 0x0067, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "beq", InstId::beq, 0x0063, funct3Low7Mask,
	RvExtension::I, RvFormat::B,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "bne", InstId::bne, 0x1063, funct3Low7Mask,
	RvExtension::I, RvFormat::B,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "blt", InstId::blt, 0x4063, funct3Low7Mask,
	RvExtension::I, RvFormat::B,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "bge", InstId::bge, 0x5063, funct3Low7Mask,
	RvExtension::I, RvFormat::B,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "bltu", InstId::bltu, 0x6063, funct3Low7Mask,
	RvExtension::I, RvFormat::B,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "bgeu", InstId::bgeu, 0x7063, funct3Low7Mask,
	RvExtension::I, RvFormat::B,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "lb", InstId::lb, 0x0003, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "lh", InstId::lh, 0x1003, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "lw", InstId::lw, 0x2003, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "lbu", InstId::lbu, 0x4003, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "lhu", InstId::lhu, 0x5003, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      // For store isntructions: Stored register is op0, address reg is
      // op1, offset is op2.
      { "sb", InstId::sb, 0x0023, funct3Low7Mask,
	RvExtension::I, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "sh", InstId::sh, 0x1023, funct3Low7Mask,
	RvExtension::I, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "sw", InstId::sw, 0x2023, funct3Low7Mask,
	RvExtension::I, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "addi", InstId::addi, 0x0013, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "slti", InstId::slti, 0x2013, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "sltiu", InstId::sltiu, 0x3013, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "xori", InstId::xori, 0x4013, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "ori", InstId::ori, 0x6013, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "andi", InstId::andi, 0x7013, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "slli", InstId::slli, 0x1013, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "srli", InstId::srli, 0x5013, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "srai", InstId::srai, 0x40005013, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "add", InstId::add, 0x0033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sub", InstId::sub, 0x40000033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sll", InstId::sll, 0x1033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "slt", InstId::slt, 0x2033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sltu", InstId::sltu, 0x3033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "xor", InstId::xor_, 0x4033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "srl", InstId::srl, 0x5033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sra", InstId::sra, 0x40005033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "or", InstId::or_, 0x6033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "and", InstId::and_, 0x7033, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "fence", InstId::fence, 0x000f, 0xf000707f,
	RvExtension::I, RvFormat::I,
	OperandType::Imm, OperandMode::None, 0x0f000000,
	OperandType::Imm, OperandMode::None, 0x00f00000 },

      { "pause", InstId::pause, 0x100000f, 0xffffffff,
        RvExtension::I, RvFormat::I },

      { "fence.tso", InstId::fence_tso, 0x800000f, 0xf000707f,
	RvExtension::I, RvFormat::I,
	OperandType::Imm, OperandMode::None, 0x0f000000,
	OperandType::Imm, OperandMode::None, 0x00f00000 },

      { "fence.i", InstId::fence_i, 0x100f, 0x0000707f,  // FIXME: Check mask.
	RvExtension::I, RvFormat::I },

      { "ecall", InstId::ecall, 0x00000073, 0xffffffff,
	RvExtension::I, RvFormat::I },

      { "ebreak", InstId::ebreak, 0x00100073, 0xffffffff,
	RvExtension::I, RvFormat::I },

      // CSR
      { "csrrw", InstId::csrrw, 0x1073, funct3Low7Mask,
	RvExtension::Zicsr, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::CsReg, OperandMode::ReadWrite, immTop12 },

      { "csrrs", InstId::csrrs, 0x2073, funct3Low7Mask,
	RvExtension::Zicsr, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::CsReg, OperandMode::ReadWrite, immTop12 },

      { "csrrc", InstId::csrrc, 0x3073, funct3Low7Mask,
	RvExtension::Zicsr, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::CsReg, OperandMode::ReadWrite, immTop12 },

      { "csrrwi", InstId::csrrwi,  0x5073, funct3Low7Mask,
	RvExtension::Zicsr, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, rs1Mask,
	OperandType::CsReg, OperandMode::ReadWrite, immTop12 },

      { "csrrsi", InstId::csrrsi, 0x6073, funct3Low7Mask,
	RvExtension::Zicsr, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, rs1Mask,
	OperandType::CsReg, OperandMode::ReadWrite, immTop12 },

      { "csrrci", InstId::csrrci, 0x7073, funct3Low7Mask,
	RvExtension::Zicsr, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, rs1Mask,
	OperandType::CsReg, OperandMode::ReadWrite, immTop12 },

      // rv64i
      { "lwu", InstId::lwu, 0x06003, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "ld", InstId::ld, 0x3003, funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "sd", InstId::sd, 0x3023, funct3Low7Mask,
	RvExtension::I, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "addiw", InstId::addiw, 0x001b, 0x707f,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "slliw", InstId::slliw, 0x101b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "srliw", InstId::srliw, 0x501b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "sraiw", InstId::sraiw, 0x4000501b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "addw", InstId::addw, 0x003b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "subw", InstId::subw, 0x4000003b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sllw", InstId::sllw, 0x103b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "srlw", InstId::srlw, 0x503b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sraw", InstId::sraw, 0x4000503b, top7Funct3Low7Mask,
	RvExtension::I, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      // Mul/div
      { "mul", InstId::mul, 0x02000033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "mulh", InstId::mulh, 0x02001033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "mulhsu", InstId::mulhsu, 0x02002033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "mulhu", InstId::mulhu, 0x02003033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "div", InstId::div, 0x02004033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "divu", InstId::divu, 0x02005033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "rem", InstId::rem, 0x02006033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "remu", InstId::remu, 0x02007033, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      // 64-bit mul/div
      { "mulw", InstId::mulw, 0x0200003b, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "divw", InstId::divw, 0x0200403b, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "divuw", InstId::divuw, 0x0200503b, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "remw", InstId::remw, 0x0200603b, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "remuw", InstId::remuw, 0x0200703b, top7Funct3Low7Mask,
	RvExtension::M, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      // Atomic
      { "lr.w", InstId::lr_w, 0x1000202f, 0xf9f0707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "sc.w", InstId::sc_w, 0x1800202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoswap.w", InstId::amoswap_w, 0x0800202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoadd.w", InstId::amoadd_w, 0x0000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoxor.w", InstId::amoxor_w, 0x2000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoand.w", InstId::amoand_w, 0x6000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoor.w", InstId::amoor_w, 0x4000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amomin.w", InstId::amomin_w, 0x8000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amomax.w", InstId::amomax_w, 0xa000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amominu.w", InstId::amominu_w, 0xc000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amomaxu.w", InstId::amomaxu_w, 0xe000202f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      // 64-bit atomic
      { "lr.d", InstId::lr_d, 0x1000302f, 0xf9f0707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "sc.d", InstId::sc_d, 0x1800302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoswap.d", InstId::amoswap_d, 0x0800302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoadd.d", InstId::amoadd_d, 0x0000302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoxor.d", InstId::amoxor_d, 0x2000302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoand.d", InstId::amoand_d, 0x6000302f, 0xf800070f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amoor.d", InstId::amoor_d, 0x4000302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amomin.d", InstId::amomin_d, 0x8000302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amomax.d", InstId::amomax_d, 0xa000302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amominu.d", InstId::amominu_d, 0xc000302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amomaxu.d", InstId::amomaxu_d, 0xe000302f, 0xf800707f,
	RvExtension::A, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      // rv32f
      { "flw", InstId::flw, 0x2007, funct3Low7Mask,
	RvExtension::F, RvFormat::I,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      // Stored register is in op0.
      { "fsw", InstId::fsw, 0x2027, funct3Low7Mask,
	RvExtension::F, RvFormat::S,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "fmadd.s", InstId::fmadd_s, 0x43, fmaddMask,
	RvExtension::F, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fmsub.s", InstId::fmsub_s, 0x47, fmaddMask,
	RvExtension::F, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fnmsub.s", InstId::fnmsub_s, 0x4b, fmaddMask,
	RvExtension::F, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fnmadd.s", InstId::fnmadd_s, 0x4f, fmaddMask,
	RvExtension::F, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fadd.s", InstId::fadd_s, 0x0053, faddMask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsub.s", InstId::fsub_s, 0x08000053, faddMask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmul.s", InstId::fmul_s, 0x10000053, faddMask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fdiv.s", InstId::fdiv_s, 0x18000053, faddMask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsqrt.s", InstId::fsqrt_s, 0x58000053, fsqrtMask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fsgnj.s", InstId::fsgnj_s, 0x20000053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsgnjn.s", InstId::fsgnjn_s, 0x20001053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsgnjx.s", InstId::fsgnjx_s, 0x20002053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmin.s", InstId::fmin_s, 0x28000053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmax.s", InstId::fmax_s, 0x28001053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fcvt.w.s", InstId::fcvt_w_s, 0xc0000053, fsqrtMask,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.wu.s", InstId::fcvt_wu_s, 0xc0100053, fsqrtMask,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fmv.x.w", InstId::fmv_x_w, 0xe0000053, 0xfff0707f,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "feq.s", InstId::feq_s, 0xa0002053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "flt.s", InstId::flt_s, 0xa0001053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fle.s", InstId::fle_s, 0xa0000053, top7Funct3Low7Mask,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fclass.s", InstId::fclass_s, 0xe0001053, 0xfff0707f,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.s.w", InstId::fcvt_s_w, 0xd0000053, fsqrtMask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fcvt.s.wu", InstId::fcvt_s_wu, 0xd0100053, fsqrtMask,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fmv.w.x", InstId::fmv_w_x, 0xf0000053, 0xfff0707f,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      // rv64f
      { "fcvt.l.s", InstId::fcvt_l_s, 0xc0200053, 0xfff0007f,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.lu.s", InstId::fcvt_lu_s, 0xc0300053, 0xfff0007f,
	RvExtension::F, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.s.l", InstId::fcvt_s_l, 0xd0200053, 0xfff0007f,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fcvt.s.lu", InstId::fcvt_s_lu, 0xd0300053, 0xfff0007f,
	RvExtension::F, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      // rv32d
      { "fld", InstId::fld, 0x3007, funct3Low7Mask,
	RvExtension::D, RvFormat::I,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "fsd", InstId::fsd, 0x3027, funct3Low7Mask,
	RvExtension::D, RvFormat::S,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "fmadd.d", InstId::fmadd_d, 0x02000043, fmaddMask,
	RvExtension::D, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fmsub.d", InstId::fmsub_d, 0x02000047, fmaddMask,
	RvExtension::D, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fnmsub.d", InstId::fnmsub_d, 0x0200004b, fmaddMask,
	RvExtension::D, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fnmadd.d", InstId::fnmadd_d, 0x0200004f, fmaddMask,
	RvExtension::D, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fadd.d", InstId::fadd_d, 0x02000053, faddMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsub.d", InstId::fsub_d, 0x0a000053, faddMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmul.d", InstId::fmul_d, 0x12000053, faddMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fdiv.d", InstId::fdiv_d, 0x1a000053, faddMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsqrt.d", InstId::fsqrt_d, 0x5a000053, fsqrtMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fsgnj.d", InstId::fsgnj_d, 0x22000053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsgnjn.d", InstId::fsgnjn_d, 0x22001053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsgnjx.d", InstId::fsgnjx_d, 0x22002053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmin.d", InstId::fmin_d, 0x2a000053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmax.d", InstId::fmax_d, 0x2a001053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fcvt.s.d", InstId::fcvt_s_d, 0x40100053, fsqrtMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.d.s", InstId::fcvt_d_s, 0x42000053, fsqrtMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "feq.d", InstId::feq_d, 0xa2002053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "flt.d", InstId::flt_d, 0xa2001053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fle.d", InstId::fle_d, 0xa2000053, top7Funct3Low7Mask,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fclass.d", InstId::fclass_d, 0xe2001053, 0xfff0707f,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.w.d", InstId::fcvt_w_d, 0xc2000053, 0xfff1c07f,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.wu.d", InstId::fcvt_wu_d, 0xc2100053, fsqrtMask,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.d.w", InstId::fcvt_d_w, 0xd2000053, fsqrtMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fcvt.d.wu", InstId::fcvt_d_wu, 0xd2100053, fsqrtMask,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      // rv64f + rv32d
      { "fcvt.l.d", InstId::fcvt_l_d, 0xc2200053, 0xfff0007f,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.lu.d", InstId::fcvt_lu_d, 0xc2300053, 0xfff0007f,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fmv.x.d", InstId::fmv_x_d, 0xe2000053, 0xfff0707f,
	RvExtension::D, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.d.l", InstId::fcvt_d_l, 0xd2200053, 0xfff0007f,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fcvt.d.lu", InstId::fcvt_d_lu, 0xd2300053, 0xfff0007f,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fmv.d.x", InstId::fmv_d_x, 0xf2000053, 0xfff0707f,
	RvExtension::D, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      // zfh  (half precision floating point)
      { "flh", InstId::flh, 0x1007, funct3Low7Mask,
	RvExtension::Zfh, RvFormat::I,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immTop12 },

      { "fsh", InstId::fsh, 0x1027, funct3Low7Mask,
	RvExtension::Zfh, RvFormat::S,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, immBeq },

      { "fmadd.h", InstId::fmadd_h, 0x04000043, fmaddMask,
	RvExtension::Zfh, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fmsub.h", InstId::fmsub_h, 0x04000047, fmaddMask,
	RvExtension::Zfh, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fnmsub.h", InstId::fnmsub_h, 0x0400004b, fmaddMask,
	RvExtension::Zfh, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fnmadd.h", InstId::fnmadd_h, 0x0400004f, fmaddMask,
	RvExtension::Zfh, RvFormat::R4,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
	OperandType::FpReg, OperandMode::Read, rs3Mask },

      { "fadd.h", InstId::fadd_h, 0x04000053, faddMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsub.h", InstId::fsub_h, 0x0c000053, faddMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmul.h", InstId::fmul_h, 0x14000053, faddMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fdiv.h", InstId::fdiv_h, 0x1c000053, faddMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsqrt.h", InstId::fsqrt_h, 0x5c000053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fsgnj.h", InstId::fsgnj_h, 0x24000053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsgnjn.h", InstId::fsgnjn_h, 0x24001053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fsgnjx.h", InstId::fsgnjx_h, 0x24002053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmin.h", InstId::fmin_h, 0x2c000053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmax.h", InstId::fmax_h, 0x2c001053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fcvt.s.h", InstId::fcvt_s_h, 0x40200053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.d.h", InstId::fcvt_d_h, 0x42200053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.h.s", InstId::fcvt_h_s, 0x44000053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.h.d", InstId::fcvt_h_d, 0x44100053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.w.h", InstId::fcvt_w_h, 0xc4000053, 0xfff1c07f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.wu.h", InstId::fcvt_wu_h, 0xc4100053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fmv.x.h", InstId::fmv_x_h, 0xe4000053, 0xfff0707f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "feq.h", InstId::feq_h, 0xa4002053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "flt.h", InstId::flt_h, 0xa4001053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fle.h", InstId::fle_h, 0xa4000053, top7Funct3Low7Mask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fclass.h", InstId::fclass_h, 0xe4001053, 0xfff0707f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.h.w", InstId::fcvt_h_w, 0xd4000053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fcvt.h.wu", InstId::fcvt_h_wu, 0xd4100053, fsqrtMask,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fmv.h.x", InstId::fmv_h_x, 0xf4000053, 0xfff0707f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fcvt.l.h", InstId::fcvt_l_h, 0xc4200053, 0xfff0007f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.lu.h", InstId::fcvt_lu_h, 0xc4300053, 0xfff0007f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.h.l", InstId::fcvt_h_l, 0xd4200053, 0xfff0007f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "fcvt.h.lu", InstId::fcvt_h_lu, 0xd4300053, 0xfff0007f,
	RvExtension::Zfh, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      // Scalar BF16 converts (Zfbfmin)
      { "fcvt.bf16.s", InstId::fcvt_bf16_s, 0x44800053, 0xfff0007f,
        RvExtension::Zfbfmin, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fcvt.s.bf16", InstId::fcvt_s_bf16, 0x40600053, 0xfff0007f,
	RvExtension::Zfbfmin, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      // Privileged
      { "mret", InstId::mret, 0x30200073, 0xffffffff,
	RvExtension::I, RvFormat::I },

      { "sret", InstId::sret, 0x10200073, 0xffffffff,
	RvExtension::I, RvFormat::I },

      { "mnret", InstId::mnret, 0x70200073, 0xffffffff,
	RvExtension::I, RvFormat::I },

      { "wfi", InstId::wfi, 0x10500073, 0xffffffff,
	RvExtension::I, RvFormat::I },

      // Debug.
      { "dret", InstId::dret, 0x7b200073, 0xffffffff,
	RvExtension::I, RvFormat::I },

      // Supervisor
      { "sfence.vma", InstId::sfence_vma, 0x12000073, 0xfe007fff,
        RvExtension::S, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      // Compressed insts. The operand bits are "swizzled" and the
      // operand masks are not used for obtaining operands.
      { "c.addi4spn", InstId::c_addi4spn, 0x0000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1fe0 },

      { "c.fld", InstId::c_fld, 0x2000, 0xe003,
	RvExtension::D, RvFormat::None,
	OperandType::FpReg, OperandMode::Write, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.lq", InstId::c_lq, 0x2000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.lw", InstId::c_lw, 0x4000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.flw", InstId::c_flw, 0x6000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::FpReg, OperandMode::Write, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.ld", InstId::c_ld, 0x6000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.fsd", InstId::c_fsd, 0xa000, 0xe003,
	RvExtension::D, RvFormat::None,
	OperandType::FpReg, OperandMode::Read, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.sq", InstId::c_sq, 0xa000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.sw", InstId::c_sw, 0xc000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.fsw", InstId::c_fsw, 0xe000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::FpReg, OperandMode::Read, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.sd", InstId::c_sd, 0xe000, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0x1c,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x1c60 },

      { "c.addi", InstId::c_addi, 0x0001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0xf80,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.jal", InstId::c_jal, 0x2001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0,
	OperandType::Imm, OperandMode::None, 0x1ffc },

      { "c.li", InstId::c_li, 0x4001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.addi16sp", InstId::c_addi16sp, 0x6101, 0xef83,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0xf80,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.lui", InstId::c_lui, 0x6001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::Imm, OperandMode::None, 0x170c },

      { "c.srli", InstId::c_srli, 0x8001, 0xfc03,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.srli64", InstId::c_srli64, 0x8001, 0xfc83,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0 },

      { "c.srai", InstId::c_srai, 0x8401, 0x9c03,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.srai64", InstId::c_srai64, 0x8401, 0xfc83,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0 },

      { "c.andi", InstId::c_andi, 0x8801, 0xec03,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.sub", InstId::c_sub, 0x8c01, 0xfc63,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x1c },

      { "c.xor", InstId::c_xor, 0x8c21, 0xfc63,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x1c },

      { "c.or", InstId::c_or, 0x8c41, 0xfc63,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x1c },

      { "c.and", InstId::c_and, 0x8c61, 0xfc63,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x1c },

      { "c.subw", InstId::c_subw, 0x9c01, 0xfc63,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x1c },

      { "c.addw", InstId::c_addw, 0x9c21, 0xfc63,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0x1c },

      { "c.j", InstId::c_j, 0xa001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1ffc },

      { "c.beqz", InstId::c_beqz, 0xc001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1c7c },

      { "c.bnez", InstId::c_bnez, 0xe001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0x380,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1c7c },

      { "c.slli", InstId::c_slli, 0x0002, 0xf003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0xf80,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.slli64", InstId::c_slli64, 0x0002, 0xf083,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0xf80,
	OperandType::Imm, OperandMode::None, 0 },

      { "c.fldsp", InstId::c_fldsp, 0x2002, 0xe003,
	RvExtension::D, RvFormat::None,
	OperandType::FpReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.lwsp", InstId::c_lwsp, 0x4002, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.flwsp", InstId::c_flwsp, 0x6002, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::FpReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.ldsp", InstId::c_ldsp, 0x6002, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.jr", InstId::c_jr, 0x8002, 0xf07f,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0 },

      // This expands to add rd, x0, rs2
      { "c.mv", InstId::c_mv, 0x8002, 0xf003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::IntReg, OperandMode::Read, 0x7c },

      { "c.ebreak", InstId::c_ebreak, 0x9002, 0xffff,
	RvExtension::I, RvFormat::None },

      { "c.jalr", InstId::c_jalr, 0x9002, 0xf07f,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0,
	OperandType::IntReg, OperandMode::Read, 0xf80,
	OperandType::Imm, OperandMode::None, 0 },

      { "c.add", InstId::c_add, 0x9002, 0xf003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0x07c },

      { "c.fsdsp", InstId::c_fsdsp, 0xa002, 0xe003,
	RvExtension::D, RvFormat::None,
	OperandType::FpReg, OperandMode::Read, 0x7c,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1f80 },

      { "c.swsp", InstId::c_swsp, 0xc002, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0x7c,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1f80 },

      { "c.fswsp", InstId::c_fswsp, 0xe002, 0xe003,
	RvExtension::F, RvFormat::None,
	OperandType::FpReg, OperandMode::Read, 0x7c,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1f80 },

      { "c.addiw", InstId::c_addiw, 0x2001, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0xf80,
	OperandType::IntReg, OperandMode::Read, 0xf80,
	OperandType::Imm, OperandMode::None, 0x107c },

      { "c.sdsp", InstId::c_sdsp, 0xe002, 0xe003,
	RvExtension::I, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0x7c,
	OperandType::IntReg, OperandMode::Read, 0,
	OperandType::Imm, OperandMode::None, 0x1f80 },

      { "andn", InstId::andn, 0x40007033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "clz", InstId::clz, 0x60001013, 0xfff0707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "clzw", InstId::clzw, 0x6000101b, 0xfff0707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "cpop", InstId::cpop, 0x60201013, 0xfff0707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "cpopw", InstId::cpopw, 0x6020101b, 0xfff0707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "ctz", InstId::ctz, 0x60101013, 0xfff0707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "ctzw", InstId::ctzw, 0x6010101b, 0xfff0707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "max", InstId::max, 0x0a006033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "maxu", InstId::maxu, 0x0a007033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "min", InstId::min, 0x0a004033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "minu", InstId::minu, 0x0a005033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "orc.b", InstId::orc_b, 0x28005013, 0xf800707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "orn", InstId::orn, 0x40006033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "rol", InstId::rol, 0x60001033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "rolw", InstId::rolw, 0x6000103b, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "ror", InstId::ror, 0x60005033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "rori", InstId::rori, 0x60005013, 0xf800707f,
	RvExtension::Zbb, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "roriw", InstId::roriw, 0x6000501b, 0xfe00707f,
	RvExtension::Zbb, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "rorw", InstId::rorw, 0x6000503b, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sext.b", InstId::sext_b, 0x60401013, 0xfff0707f,
        RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "sext.h", InstId::sext_h, 0x60501013, 0xfff0707f,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "xnor", InstId::xnor, 0x40004033, top7Funct3Low7Mask,
	RvExtension::Zbb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "pack", InstId::pack, 0x08004033, top7Funct3Low7Mask,
	RvExtension::Zbkb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "packh", InstId::packh, 0x08007033, top7Funct3Low7Mask,
        RvExtension::Zbkb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "packw", InstId::packw, 0x0800403b, top7Funct3Low7Mask,
        RvExtension::Zbkb, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "brev8", InstId::brev8, 0x68705013, 0xfff0707f,
	RvExtension::Zbkb, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "rev8_32", InstId::rev8_32, 0x69805013, 0xfff0707f,
	RvExtension::Zbb, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "rev8", InstId::rev8_64, 0x6b805013, 0xfff0707f,
	RvExtension::Zbb, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "zip", InstId::zip, 0x08f01013, 0xfff0707f,
	RvExtension::Zbkb, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "unzip", InstId::unzip, 0x08f05013, 0xfff0707f,
	RvExtension::Zbkb, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "xperm.n", InstId::xperm_n, 0x28002033, top7Funct3Low7Mask,
        RvExtension::Zbkx, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "xperm.b", InstId::xperm_b, 0x28004033, top7Funct3Low7Mask,
        RvExtension::Zbkx, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "bset", InstId::bset, 0x28001033, top7Funct3Low7Mask,
	RvExtension::Zbs, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "bclr", InstId::bclr, 0x48001033, top7Funct3Low7Mask,
	RvExtension::Zbs, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "binv", InstId::binv, 0x68001033, top7Funct3Low7Mask,
	RvExtension::Zbs, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "bext", InstId::bext, 0x08006033, top7Funct3Low7Mask,
	RvExtension::Zbs, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "bseti", InstId::bseti, 0x28001013, 0xf800707f,
	RvExtension::Zbs, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "bclri", InstId::bclri, 0x48001013, 0xf800707f,
	RvExtension::Zbs, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "binvi", InstId::binvi, 0x68001013, 0xf800707f,
	RvExtension::Zbs, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "bexti", InstId::bexti, 0x48005013, 0xf800707f,
	RvExtension::Zbs, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "clmul", InstId::clmul, 0x0a001033, top7Funct3Low7Mask,
	RvExtension::Zbc, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "clmulh", InstId::clmulh, 0x0a003033, top7Funct3Low7Mask,
	RvExtension::Zbc, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "clmulr", InstId::clmulr, 0x0a002033, top7Funct3Low7Mask,
	RvExtension::Zbc, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sh1add", InstId::sh1add, 0x20002033, top7Funct3Low7Mask,
	RvExtension::Zba, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sh2add", InstId::sh2add, 0x20004033, top7Funct3Low7Mask,
	RvExtension::Zba, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sh3add", InstId::sh3add, 0x20006033, top7Funct3Low7Mask,
	RvExtension::Zba, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sh1add.uw", InstId::sh1add_uw, 0x2000203B, top7Funct3Low7Mask,
	RvExtension::Zba, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sh2add.uw", InstId::sh2add_uw, 0x2000403B, top7Funct3Low7Mask,
	RvExtension::Zba, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "sh3add.uw", InstId::sh3add_uw, 0x2000603B, top7Funct3Low7Mask,
	RvExtension::Zba, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "add.uw", InstId::add_uw, 0x800003b, top7Funct3Low7Mask,
        RvExtension::Zba, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "slli.uw", InstId::slli_uw, 0x0800101b, 0xf800707f,
	RvExtension::Zba, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, shamtMask },

      { "vsetvli", InstId::vsetvli,
        0b000000'0'00000'00000'111'00000'1010111, // Opcode
        0b100000'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, 0x7ff00000,
      },

      { "vsetivli", InstId::vsetivli,
        0b110000'0'00000'00000'111'00000'1010111, // Opcode
        0b110000'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::Imm, OperandMode::None, 0x000f8000,
        OperandType::Imm, OperandMode::None, 0x7ff00000,
      },

      { "vsetvl", InstId::vsetvl,
        0b100000'0'00000'00000'111'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vadd.vv", InstId::vadd_vv,
        0b000000'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vadd.vx", InstId::vadd_vx,
        0b000000'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vadd.vi", InstId::vadd_vi,
        0b000000'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vsub.vv", InstId::vsub_vv,
        0b000010'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsub.vx", InstId::vsub_vx,
        0b000010'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vrsub.vx", InstId::vrsub_vx,
        0b000011'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vrsub.vi", InstId::vrsub_vi,
        0b000011'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vwaddu.vv", InstId::vwaddu_vv,
	0b110000'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwaddu.vx", InstId::vwaddu_vx,
        0b110000'0'00000'00000'110'00000'0010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwsubu.vv", InstId::vwsubu_vv,
        0b110010'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwsubu.vx", InstId::vwsubu_vx,
        0b110010'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwadd.vv", InstId::vwadd_vv ,
        0b110001'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwadd.vx", InstId::vwadd_vx ,
        0b110001'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwsub.vv", InstId::vwsub_vv ,
        0b110011'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwsub.vx", InstId::vwsub_vx ,
        0b110011'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwaddu.wv", InstId::vwaddu_wv,
        0b110100'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwaddu.wx", InstId::vwaddu_wx,
        0b110100'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwsubu.wv", InstId::vwsubu_wv,
        0b110110'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwsubu.wx", InstId::vwsubu_wx,
        0b110110'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwadd.wv", InstId::vwadd_wv ,
        0b110101'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwadd.wx", InstId::vwadd_wx ,
        0b110101'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwsub.wv", InstId::vwsub_wv ,
        0b110111'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwsub.wx", InstId::vwsub_wx ,
        0b110111'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111,
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmseq.vv", InstId::vmseq_vv,
        0b011000'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmseq.vx", InstId::vmseq_vx,
        0b011000'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmseq.vi", InstId::vmseq_vi,
        0b011000'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmsne.vv", InstId::vmsne_vv,
        0b011001'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsne.vx", InstId::vmsne_vx,
        0b011001'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmsne.vi", InstId::vmsne_vi,
        0b011001'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmsltu.vv", InstId::vmsltu_vv,
        0b011010'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsltu.vx", InstId::vmsltu_vx,
        0b011010'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmslt.vv", InstId::vmslt_vv,
        0b011011'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmslt.vx", InstId::vmslt_vx,
        0b011011'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmsleu.vv", InstId::vmsleu_vv,
        0b011100'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsleu.vx", InstId::vmsleu_vx,
        0b011100'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmsleu.vi", InstId::vmsleu_vi,
        0b011100'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmsle.vv", InstId::vmsle_vv,
        0b011101'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsle.vx", InstId::vmsle_vx,
        0b011101'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmsle.vi", InstId::vmsle_vi,
        0b011101'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmsgtu.vx", InstId::vmsgtu_vx,
        0b011110'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmsgtu.vi", InstId::vmsgtu_vi,
        0b011110'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmsgt.vx", InstId::vmsgt_vx,
        0b011111'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmsgt.vi", InstId::vmsgt_vi,
        0b011111'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vminu.vv", InstId::vminu_vv,
        0b000100'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vminu.vx", InstId::vminu_vx,
        0b000100'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmin.vv", InstId::vmin_vv,
        0b000101'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmin.vx", InstId::vmin_vx,
        0b000101'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmaxu.vv", InstId::vmaxu_vv,
        0b000110'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmaxu.vx", InstId::vmaxu_vx,
        0b000110'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmax.vv", InstId::vmax_vv,
        0b000111'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmax.vx", InstId::vmax_vx,
        0b000111'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vand.vv", InstId::vand_vv,
        0b001001'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vand.vx", InstId::vand_vx,
        0b001001'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vand.vi", InstId::vand_vi,
        0b001001'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vor.vv", InstId::vor_vv,
        0b001010'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vor.vx", InstId::vor_vx,
        0b001010'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vor.vi", InstId::vor_vi,
        0b001010'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vxor.vv", InstId::vxor_vv,
        0b001011'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vxor.vx", InstId::vxor_vx,
        0b001011'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vxor.vi", InstId::vxor_vi,
        0b001011'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vsll.vv", InstId::vsll_vv,
        0b100101'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsll.vx", InstId::vsll_vx,
        0b100101'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsll.vi", InstId::vsll_vi,
        0b100101'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vsrl.vv", InstId::vsrl_vv,
        0b101000'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsrl.vx", InstId::vsrl_vx,
        0b101000'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsrl.vi", InstId::vsrl_vi,
        0b101000'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vsra.vv", InstId::vsra_vv,
        0b101001'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsra.vx", InstId::vsra_vx,
        0b101001'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsra.vi", InstId::vsra_vi,
        0b101001'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vnsrl.wv", InstId::vnsrl_wv,
        0b101100'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnsrl.wx", InstId::vnsrl_wx,
        0b101100'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vnsrl.wi", InstId::vnsrl_wi,
        0b101100'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vnsra.wv", InstId::vnsra_wv,
        0b101101'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnsra.wx", InstId::vnsra_wx,
        0b101101'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vnsra.wi", InstId::vnsra_wi,
        0b101101'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vrgather.vv", InstId::vrgather_vv,
        0b001100'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vrgather.vx", InstId::vrgather_vx,
        0b001100'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vrgather.vi", InstId::vrgather_vi,
        0b001100'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vrgatherei16.vv", InstId::vrgatherei16_vv,
        0b001110'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vcompress.vm", InstId::vcompress_vm,
        0b010111'0'00000'00011'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredsum.vs", InstId::vredsum_vs,
        0b000000'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredand.vs", InstId::vredand_vs,
        0b000001'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredor.vs", InstId::vredor_vs,
        0b000010'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredxor.vs", InstId::vredxor_vs,
        0b000011'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredminu.vs", InstId::vredminu_vs,
        0b000100'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredmin.vs", InstId::vredmin_vs,
        0b000101'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredmaxu.vs", InstId::vredmaxu_vs,
        0b000110'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vredmax.vs", InstId::vredmax_vs,
        0b000111'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwredsumu.vs", InstId::vwredsumu_vs,
        0b110000'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwredsum.vs", InstId::vwredsum_vs,
        0b110001'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmand.mm", InstId::vmand_mm,
        0b011001'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmnand.mm", InstId::vmnand_mm,
        0b011101'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmandn.mm", InstId::vmandn_mm,
        0b011000'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmxor.mm", InstId::vmxor_mm,
        0b011011'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmor.mm", InstId::vmor_mm,
        0b011010'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmnor.mm", InstId::vmnor_mm,
        0b011110'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmorn.mm", InstId::vmorn_mm,
        0b011100'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmxnor.mm", InstId::vmxnor_mm,
        0b011111'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vcpop.m", InstId::vcpop_m,
        0b010000'0'00000'10000'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfirst.m", InstId::vfirst_m,
        0b010000'0'00000'10001'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsbf.m", InstId::vmsbf_m,
        0b010100'0'00000'00001'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsif.m", InstId::vmsif_m,
        0b010100'0'00000'00011'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsof.m", InstId::vmsof_m,
        0b010100'0'00000'00010'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "viota.m", InstId::viota_m,
        0b010100'0'00000'10000'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vid.v", InstId::vid_v,
        0b010100'0'00000'10001'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
      },

      { "vslideup.vx", InstId::vslideup_vx,
        0b001110'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vslideup.vi", InstId::vslideup_vi,
        0b001110'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vslide1up.vx", InstId::vslide1up_vx,
        0b001110'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vslidedown.vx", InstId::vslidedown_vx,
        0b001111'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vslidedown.vi", InstId::vslidedown_vi,
        0b001111'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vslide1down.vx", InstId::vslide1down_vx,
        0b001111'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vfslide1up.vf", InstId::vfslide1up_vf,
        0b001110'1'00000'00000'101'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfslide1down.vf", InstId::vfslide1down_vf,
        0b001111'0'00000'00000'101'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vmul.vv", InstId::vmul_vv,
        0b100101'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmul.vx", InstId::vmul_vx,
        0b100101'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmulh.vv", InstId::vmulh_vv,
        0b100111'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmulh.vx", InstId::vmulh_vx,
        0b100111'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmulhu.vv", InstId::vmulhu_vv,
        0b100100'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmulhu.vx", InstId::vmulhu_vx,
        0b100100'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmulhsu.vv", InstId::vmulhsu_vv,
        0b100110'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmulhsu.vx", InstId::vmulhsu_vx,
        0b100110'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmadd.vv", InstId::vmadd_vv,
        0b101001'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmadd.vx", InstId::vmadd_vx,
        0b101001'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnmsub.vv", InstId::vnmsub_vv,
        0b101011'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnmsub.vx", InstId::vnmsub_vx,
        0b101011'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmacc.vv", InstId::vmacc_vv,
        0b101101'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmacc.vx", InstId::vmacc_vx,
        0b101101'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnmsac.vv", InstId::vnmsac_vv,
        0b101111'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnmsac.vx", InstId::vnmsac_vx,
        0b101111'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmulu.vv", InstId::vwmulu_vv,
        0b111000'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmulu.vx", InstId::vwmulu_vx,
        0b111000'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwmul.vv", InstId::vwmul_vv,
        0b111011'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmul.vx", InstId::vwmul_vx,
        0b111011'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwmulsu.vv", InstId::vwmulsu_vv,
        0b111010'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmulsu.vx", InstId::vwmulsu_vx,
        0b111010'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vwmaccu.vv", InstId::vwmaccu_vv,
        0b111100'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmaccu.vx", InstId::vwmaccu_vx,
        0b111100'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmacc.vv", InstId::vwmacc_vv,
        0b111101'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmacc.vx", InstId::vwmacc_vx,
        0b111101'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmaccsu.vv", InstId::vwmaccsu_vv,
        0b111111'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmaccsu.vx", InstId::vwmaccsu_vx,
        0b111111'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vwmaccus.vx", InstId::vwmaccus_vx,
        0b111110'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vdivu.vv", InstId::vdivu_vv,
        0b100000'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vdivu.vx", InstId::vdivu_vx,
        0b100000'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vdiv.vv", InstId::vdiv_vv,
        0b100001'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vdiv.vx", InstId::vdiv_vx,
        0b100001'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vremu.vv", InstId::vremu_vv,
        0b100010'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vremu.vx", InstId::vremu_vx,
        0b100010'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vrem.vv", InstId::vrem_vv,
        0b100011'0'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vrem.vx", InstId::vrem_vx,
        0b100011'0'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsext.vf2", InstId::vsext_vf2,
        0b010010'0'00000'00111'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsext.vf4", InstId::vsext_vf4,
        0b010010'0'00000'00101'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsext.vf8", InstId::vsext_vf8,
        0b010010'0'00000'00011'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vzext.vf2", InstId::vzext_vf2,
        0b010010'0'00000'00110'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vzext.vf4", InstId::vzext_vf4,
        0b010010'0'00000'00100'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vzext.vf8", InstId::vzext_vf8,
        0b010010'0'00000'00010'010'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vadc.vvm", InstId::vadc_vvm,
        0b010000'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vadc.vxm", InstId::vadc_vxm,
        0b010000'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vadc.vim", InstId::vadc_vim,
        0b010000'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vsbc.vvm", InstId::vsbc_vvm,
        0b010010'0'00000'00011'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsbc.vxm", InstId::vsbc_vxm,
        0b010010'0'00000'00011'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmadc.vvm", InstId::vmadc_vvm,
        0b010001'0'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmadc.vxm", InstId::vmadc_vxm,
        0b010001'0'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmadc.vim", InstId::vmadc_vim,
        0b010001'0'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmsbc.vvm", InstId::vmsbc_vvm,
        0b010011'0'00000'00011'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmsbc.vxm", InstId::vmsbc_vxm,
        0b010011'0'00000'00011'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmerge.vvm", InstId::vmerge_vvm,
        0b010111'0'00010'00011'100'00001'1010111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmerge.vxm", InstId::vmerge_vxm,
        0b010111'0'00000'00000'100'00000'1010111, // Opcode
        0b010111'1'00010'00011'000'00001'1010111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vmerge.vim", InstId::vmerge_vim,
        0b010111'0'00000'00011'011'00000'1010111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmv.x.s", InstId::vmv_x_s,
        0b010000'0'00000'00000'010'00001'1010111, // Opcode
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vmv.s.x", InstId::vmv_s_x,
        0b010000'0'00000'00000'110'00001'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vfmv.f.s", InstId::vfmv_f_s,
	0b010000'1'00000'00000'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::FpReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vfmv.s.f", InstId::vfmv_s_f,
	0b010000'1'00000'00000'101'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
      },

      { "vmv.v.v", InstId::vmv_v_v,
        0b010111'1'00000'00000'000'00001'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vmv.v.x", InstId::vmv_v_x,
        0b010111'1'00000'00000'100'00000'1010111, // Opcode
        0b010111'1'00000'11111'000'00001'1010111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vmv.v.i", InstId::vmv_v_i,
        0b010111'1'00000'00000'011'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vmv1r.v", InstId::vmv1r_v,
        0b100111'1'00000'00000'011'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vmv2r.v", InstId::vmv2r_v,
        0b100111'1'00000'00001'011'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vmv4r.v", InstId::vmv4r_v,
        0b100111'1'00000'00011'011'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vmv8r.v", InstId::vmv8r_v,
        0b100111'1'00000'00111'011'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vsaddu.vv", InstId::vsaddu_vv,
        0b100000'0'00000'00011'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsaddu.vx", InstId::vsaddu_vx,
        0b100000'0'00000'00011'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsaddu.vi", InstId::vsaddu_vi,
        0b100000'0'00000'00011'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vsadd.vv", InstId::vsadd_vv,
        0b100001'0'00000'00011'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsadd.vx", InstId::vsadd_vx,
        0b100001'0'00000'00011'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsadd.vi", InstId::vsadd_vi,
        0b100001'0'00000'00011'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vssubu.vv", InstId::vssubu_vv,
        0b100010'0'00000'00011'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vssubu.vx", InstId::vssubu_vx,
        0b100010'0'00000'00011'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vssub.vv", InstId::vssub_vv,
        0b100011'0'00000'00011'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vssub.vx", InstId::vssub_vx,
        0b100011'0'00000'00011'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vaaddu.vv", InstId::vaaddu_vv,
        0b001000'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaaddu.vx", InstId::vaaddu_vx,
        0b001000'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vaadd.vv", InstId::vaadd_vv,
        0b001001'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaadd.vx", InstId::vaadd_vx,
        0b001001'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vasubu.vv", InstId::vasubu_vv,
        0b001010'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vasubu.vx", InstId::vasubu_vx,
        0b001001'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vasub.vv", InstId::vasub_vv,
        0b001010'1'00000'00000'010'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vasub.vx", InstId::vasub_vx,
        0b001010'1'00000'00000'110'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsmul.vv", InstId::vsmul_vv,
        0b100111'1'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsmul.vx", InstId::vsmul_vx,
        0b100111'1'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vssrl.vv", InstId::vssrl_vv,
        0b101010'1'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vssrl.vx", InstId::vssrl_vx,
        0b101010'1'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vssrl.vi", InstId::vssrl_vi,
        0b101010'1'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vssra.vv", InstId::vssra_vv,
        0b101011'1'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vssra.vx", InstId::vssra_vx,
        0b101011'1'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vssra.vi", InstId::vssra_vi,
        0b101011'1'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vnclipu.wv", InstId::vnclipu_wv,
        0b101110'1'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnclipu.wx", InstId::vnclipu_wx,
        0b101110'1'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vnclipu.wi", InstId::vnclipu_wi,
        0b101110'1'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vnclip.wv", InstId::vnclip_wv,
        0b101111'1'00000'00000'000'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vnclip.wx", InstId::vnclip_wx,
        0b101111'1'00000'00000'100'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vnclip.wi", InstId::vnclip_wi,
        0b101111'1'00000'00000'011'00000'1010111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::Imm, OperandMode::None, rs2Mask,
      },

      { "vle8.v", InstId::vle8_v,
        0b000000'0'00000'00000'000'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle16.v", InstId::vle16_v,
        0b000000'0'00000'00000'101'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle32.v", InstId::vle32_v,
        0b000000'0'00000'00000'110'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle64.v", InstId::vle64_v,
        0b000000'0'00000'00000'111'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle128.v", InstId::vle128_v,
        0b000100'0'00000'00000'000'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle256.v", InstId::vle256_v,
        0b000100'0'00000'00000'101'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle512.v", InstId::vle512_v,
        0b000100'0'00000'00000'110'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle1024.v", InstId::vle1024_v,
        0b000100'0'00000'00000'111'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse8.v", InstId::vse8_v,
        0b000000'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse16.v", InstId::vse16_v,
        0b000000'0'00000'00000'101'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse32.v", InstId::vse32_v,
        0b000000'0'00000'00000'110'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse64.v", InstId::vse64_v,
        0b000000'0'00000'00000'111'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse128.v", InstId::vse128_v,
        0b000100'0'00000'00000'000'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse256.v", InstId::vse256_v,
        0b000100'0'00000'00000'101'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse512.v", InstId::vse512_v,
        0b000100'0'00000'00000'110'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vse1024.v", InstId::vse1024_v,
        0b000100'0'00000'00000'111'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlm.v", InstId::vlm_v,
        0b000000'0'00000'00000'000'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vsm.v", InstId::vsm_v,
        0b000100'0'00000'00000'111'00000'0100111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre8.v", InstId::vlre8_v,
        0b001000'0'01000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre16.v", InstId::vlre16_v,
        0b001000'0'01000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre32.v", InstId::vlre32_v,
        0b001000'0'01000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre64.v", InstId::vlre64_v,
        0b001000'0'01000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre128.v", InstId::vlre128_v,
        0b001100'0'01000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre256.v", InstId::vlre256_v,
        0b001100'0'01000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre512.v", InstId::vlre512_v,
        0b001100'0'01000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlre1024.v", InstId::vlre1024_v,
        0b001100'0'01000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vs1r.v", InstId::vs1r_v,
        0b000000'1'01000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vs2r.v", InstId::vs2r_v,
        0b000000'0'01000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vs4r.v", InstId::vs4r_v,
        0b000000'0'01000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vs8r.v", InstId::vs8r_v,
        0b000000'0'01000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle8ff.v", InstId::vle8ff_v,
        0b000000'0'10000'00000'000'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle16ff.v", InstId::vle16ff_v,
        0b000000'0'10000'00000'101'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle32ff.v", InstId::vle32ff_v,
        0b000000'0'10000'00000'110'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle64ff.v", InstId::vle64ff_v,
        0b000000'0'10000'00000'111'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle128ff.v", InstId::vle128ff_v,
        0b000100'0'10000'00000'000'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle256ff.v", InstId::vle256ff_v,
        0b000100'0'10000'00000'101'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle512ff.v", InstId::vle512ff_v,
        0b000100'0'10000'00000'110'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vle1024ff.v", InstId::vle1024ff_v,
        0b000100'0'10000'00000'111'00000'0000111,
        0b111111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlse8.v", InstId::vlse8_v,
        0b000010'0'00000'00000'000'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vlse16.v", InstId::vlse16_v,
        0b000010'0'00000'00000'101'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vlse32.v", InstId::vlse32_v,
        0b000010'0'00000'00000'110'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vlse64.v", InstId::vlse64_v,
        0b000010'0'00000'00000'111'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vlse128.v", InstId::vlse128_v,
        0b000110'0'00000'00000'000'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vlse256.v", InstId::vlse256_v,
        0b000110'0'00000'00000'101'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vlse512.v", InstId::vlse512_v,
        0b000110'0'00000'00000'110'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vlse1024.v", InstId::vlse1024_v,
        0b000110'0'00000'00000'111'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse8.v", InstId::vsse8_v,
        0b000010'0'00000'00000'000'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse16.v", InstId::vsse16_v,
        0b000010'0'00000'00000'101'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse32.v", InstId::vsse32_v,
        0b000010'0'00000'00000'110'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse64.v", InstId::vsse64_v,
        0b000010'0'00000'00000'111'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse128.v", InstId::vsse128_v,
        0b000110'0'00000'00000'000'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse256.v", InstId::vsse256_v,
        0b000110'0'00000'00000'101'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse512.v", InstId::vsse512_v,
        0b000110'0'00000'00000'110'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vsse1024.v", InstId::vsse1024_v,
        0b000110'0'00000'00000'111'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vloxei8.v", InstId::vloxei8_v,
        0b000011'0'00000'00000'000'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxei16.v", InstId::vloxei16_v,
        0b000011'0'00000'00000'101'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxei32.v", InstId::vloxei32_v,
        0b000011'0'00000'00000'110'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxei64.v", InstId::vloxei64_v,
        0b000011'0'00000'00000'111'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxei8.v", InstId::vluxei8_v,
        0b000001'0'00000'00000'000'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxei16.v", InstId::vluxei16_v,
        0b000001'0'00000'00000'101'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxei32.v", InstId::vluxei32_v,
        0b000001'0'00000'00000'110'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxei64.v", InstId::vluxei64_v,
        0b000001'0'00000'00000'111'00000'0000111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxei8.v", InstId::vsoxei8_v,
        0b000011'0'00000'00000'000'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxei16.v", InstId::vsoxei16_v,
        0b000011'0'00000'00000'101'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxei32.v", InstId::vsoxei32_v,
        0b000011'0'00000'00000'110'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxei64.v", InstId::vsoxei64_v,
        0b000011'0'00000'00000'111'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxei8.v", InstId::vsuxei8_v,
        0b000001'0'00000'00000'000'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxei16.v", InstId::vsuxei16_v,
        0b000001'0'00000'00000'101'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxei32.v", InstId::vsuxei32_v,
        0b000001'0'00000'00000'110'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxei64.v", InstId::vsuxei64_v,
        0b000001'0'00000'00000'111'00000'0100111,
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vlsege8.v", InstId::vlsege8_v,
        0b001000'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege16.v", InstId::vlsege16_v,
        0b001000'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege32.v", InstId::vlsege32_v,
        0b001000'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege64.v", InstId::vlsege64_v,
        0b001000'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege128.v", InstId::vlsege128_v,
        0b001100'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege256.v", InstId::vlsege256_v,
        0b001100'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege512.v", InstId::vlsege512_v,
        0b001100'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege1024.v", InstId::vlsege1024_v,
        0b001100'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege8.v", InstId::vssege8_v,
        0b001000'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege16.v", InstId::vssege16_v,
        0b001000'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege32.v", InstId::vssege32_v,
        0b001000'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege64.v", InstId::vssege64_v,
        0b001000'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege128.v", InstId::vssege128_v,
        0b001100'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege256.v", InstId::vssege256_v,
        0b001100'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege512.v", InstId::vssege512_v,
        0b001100'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vssege1024.v", InstId::vssege1024_v,
        0b001100'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlssege8.v", InstId::vlssege8_v,
        0b001010'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vlssege16.v", InstId::vlssege16_v,
        0b001010'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vlssege32.v", InstId::vlssege32_v,
        0b001010'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vlssege64.v", InstId::vlssege64_v,
        0b001010'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vlssege128.v", InstId::vlssege128_v,
        0b001110'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vlssege256.v", InstId::vlssege256_v,
        0b001110'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vlssege512.v", InstId::vlssege512_v,
        0b001110'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vlssege1024.v", InstId::vlssege1024_v,
        0b001110'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege8.v", InstId::vsssege8_v,
        0b001010'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege16.v", InstId::vsssege16_v,
        0b001010'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege32.v", InstId::vsssege32_v,
        0b001010'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege64.v", InstId::vsssege64_v,
        0b001010'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege128.v", InstId::vsssege128_v,
        0b001110'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege256.v", InstId::vsssege256_v,
        0b001110'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege512.v", InstId::vsssege512_v,
        0b001110'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vsssege1024.v", InstId::vsssege1024_v,
        0b001110'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "vluxsegei8.v", InstId::vluxsegei8_v,
        0b001001'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxsegei16.v", InstId::vluxsegei16_v,
        0b001001'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxsegei32.v", InstId::vluxsegei32_v,
        0b001001'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxsegei64.v", InstId::vluxsegei64_v,
        0b001001'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxsegei128.v", InstId::vluxsegei128_v,
        0b001101'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxsegei256.v", InstId::vluxsegei256_v,
        0b001101'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxsegei512.v", InstId::vluxsegei512_v,
        0b001101'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vluxsegei1024.v", InstId::vluxsegei1024_v,
        0b001101'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei8.v", InstId::vsuxsegei8_v,
        0b001001'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei16.v", InstId::vsuxsegei16_v,
        0b001001'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei32.v", InstId::vsuxsegei32_v,
        0b001001'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei64.v", InstId::vsuxsegei64_v,
        0b001001'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei128.v", InstId::vsuxsegei128_v,
        0b001101'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei256.v", InstId::vsuxsegei256_v,
        0b001101'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei512.v", InstId::vsuxsegei512_v,
        0b001101'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsuxsegei1024.v", InstId::vsuxsegei1024_v,
        0b001101'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei8.v", InstId::vloxsegei8_v,
        0b001011'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei16.v", InstId::vloxsegei16_v,
        0b001011'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei32.v", InstId::vloxsegei32_v,
        0b001011'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei64.v", InstId::vloxsegei64_v,
        0b001011'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei128.v", InstId::vloxsegei128_v,
        0b001111'0'00000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei256.v", InstId::vloxsegei256_v,
        0b001111'0'00000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei512.v", InstId::vloxsegei512_v,
        0b001111'0'00000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vloxsegei1024.v", InstId::vloxsegei1024_v,
        0b001111'0'00000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei8.v", InstId::vsoxsegei8_v,
        0b001011'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei16.v", InstId::vsoxsegei16_v,
        0b001011'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei32.v", InstId::vsoxsegei32_v,
        0b001011'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei64.v", InstId::vsoxsegei64_v,
        0b001011'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei128.v", InstId::vsoxsegei128_v,
        0b001111'0'00000'00000'000'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei256.v", InstId::vsoxsegei256_v,
        0b001111'0'00000'00000'101'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei512.v", InstId::vsoxsegei512_v,
        0b001111'0'00000'00000'110'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsoxsegei1024.v", InstId::vsoxsegei1024_v,
        0b001111'0'00000'00000'111'00000'0100111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::S,
        OperandType::VecReg, OperandMode::Read, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vlsege8ff.v", InstId::vlsege8ff_v,
	0b001000'1'10000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege16ff.v", InstId::vlsege16ff_v,
        0b001000'1'10000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege32ff.v", InstId::vlsege32ff_v,
        0b001000'1'10000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege64ff.v", InstId::vlsege64ff_v,
        0b001000'1'10000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege128ff.v", InstId::vlsege128ff_v,
        0b001100'1'10000'00000'000'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege256ff.v", InstId::vlsege256ff_v,
        0b001100'1'10000'00000'101'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege512ff.v", InstId::vlsege512ff_v,
        0b001100'1'10000'00000'110'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vlsege1024ff.v", InstId::vlsege1024ff_v,
        0b001100'1'10000'00000'111'00000'0000111,
        0b000111'0'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::I,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vfadd.vv", InstId::vfadd_vv,
        0b000000'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfadd.vf", InstId::vfadd_vf,
        0b000000'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfsub.vv", InstId::vfsub_vv,
        0b000010'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfsub.vf", InstId::vfsub_vf,
        0b000010'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfrsub.vf", InstId::vfrsub_vf,
        0b100111'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfwadd.vv", InstId::vfwadd_vv,
        0b110000'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwadd.vf", InstId::vfwadd_vf,
        0b110000'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfwsub.vv", InstId::vfwsub_vv,
        0b110010'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwsub.vf", InstId::vfwsub_vf,
        0b110010'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfwadd.wv", InstId::vfwadd_wv,
        0b110100'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwadd.wf", InstId::vfwadd_wf,
        0b110100'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfwsub.wv", InstId::vfwsub_wv,
        0b110101'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwsub.wf", InstId::vfwsub_wf,
        0b110101'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfmul.vv", InstId::vfmul_vv,
        0b100100'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmul.vf", InstId::vfmul_vf,
        0b100100'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfdiv.vv", InstId::vfdiv_vv,
        0b100000'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfdiv.vf", InstId::vfdiv_vf,
        0b100000'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfrdiv.vf", InstId::vfrdiv_vf,
        0b100001'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfwmul.vv", InstId::vfwmul_vv,
        0b111000'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwmul.vf", InstId::vfwmul_vf,
        0b111000'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfmadd.vv", InstId::vfmadd_vv,
        0b101000'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmadd.vf", InstId::vfmadd_vf,
        0b101000'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmadd.vv", InstId::vfnmadd_vv,
        0b101001'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmadd.vf", InstId::vfnmadd_vf,
        0b101001'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmsub.vv", InstId::vfmsub_vv,
        0b101010'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmsub.vf", InstId::vfmsub_vf,
        0b101010'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmsub.vv", InstId::vfnmsub_vv,
        0b101011'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmsub.vf", InstId::vfnmsub_vf,
        0b101011'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmacc.vv", InstId::vfmacc_vv,
        0b101100'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmacc.vf", InstId::vfmacc_vf,
        0b101100'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmacc.vv", InstId::vfnmacc_vv,
        0b101101'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmacc.vf", InstId::vfnmacc_vf,
        0b101101'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmsac.vv", InstId::vfmsac_vv,
        0b101110'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmsac.vf", InstId::vfmsac_vf,
        0b101110'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmsac.vv", InstId::vfnmsac_vv,
        0b101111'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfnmsac.vf", InstId::vfnmsac_vf,
        0b101111'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwmacc.vv", InstId::vfwmacc_vv,
        0b111100'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwmacc.vf", InstId::vfwmacc_vf,
        0b111100'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwnmacc.vv", InstId::vfwnmacc_vv,
        0b111101'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwnmacc.vf", InstId::vfwnmacc_vf,
        0b111101'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwmsac.vv", InstId::vfwmsac_vv,
        0b111110'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwmsac.vf", InstId::vfwmsac_vf,
        0b111110'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwnmsac.vv", InstId::vfwnmsac_vv,
        0b111111'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwnmsac.vf", InstId::vfwnmsac_vf,
        0b111111'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfsqrt.v", InstId::vfsqrt_v,
        0b010011'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vfmerge.vfm", InstId::vfmerge_vfm,
        0b010011'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfmv.v.f", InstId::vfmv_v_f,
        0b010011'1'00000'00000'001'00000'1010111, // Opcode
        0b111111'1'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
      },

      { "vmfeq.vv", InstId::vmfeq_vv,
	0b011000'1'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmfeq.vf", InstId::vmfeq_vf,
	0b011000'1'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vmfne.vv", InstId::vmfne_vv,
	0b011100'1'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmfne.vf", InstId::vmfne_vf,
	0b011100'1'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vmflt.vv", InstId::vmflt_vv,
	0b011011'1'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmflt.vf", InstId::vmflt_vf,
	0b011011'1'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vmfle.vv", InstId::vmfle_vv,
	0b011001'1'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vmfle.vf", InstId::vmfle_vf,
	0b011001'1'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vmfgt.vf", InstId::vmfgt_vf,
	0b011101'1'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vmfge.vf", InstId::vmfge_vf,
	0b011111'1'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfclass.v", InstId::vfclass_v,
	0b010011'1'00000'10000'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfcvt.xu.f.v", InstId::vfcvt_xu_f_v,
	0b010010'1'00000'00000'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfcvt.x.f.v", InstId::vfcvt_x_f_v,
	0b010010'1'00000'00001'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfcvt.rtz.xu.f.v", InstId::vfcvt_rtz_xu_f_v,
	0b010010'1'00000'00110'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfcvt.rtz.x.f.v", InstId::vfcvt_rtz_x_f_v,
	0b010010'1'00000'00111'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfcvt.f.xu.v", InstId::vfcvt_f_xu_v,
	0b010010'1'00000'00010'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfcvt.f.x.v", InstId::vfcvt_f_x_v,
	0b010010'1'00000'00011'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvt.xu.f.v", InstId::vfwcvt_xu_f_v,
	0b010010'1'00000'01000'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvt.x.f.v", InstId::vfwcvt_x_f_v,
	0b010010'1'00000'01001'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvt.rtz.xu.f.v", InstId::vfwcvt_rtz_xu_f_v,
	0b010010'1'00000'01110'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvt.rtz.x.f.v", InstId::vfwcvt_rtz_x_f_v,
	0b010010'1'00000'01111'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvt.f.xu.v", InstId::vfwcvt_f_xu_v,
	0b010010'1'00000'01010'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvt.f.x.v", InstId::vfwcvt_f_x_v,
	0b010010'1'00000'01011'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvt.f.f.v", InstId::vfwcvt_f_f_v,
	0b010010'1'00000'01100'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.xu.f.w", InstId::vfncvt_xu_f_w,
	0b010010'1'00000'10000'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.x.f.w", InstId::vfncvt_x_f_w,
	0b010010'1'00000'10001'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.rtz.xu.f.w", InstId::vfncvt_rtz_xu_f_w,
	0b010010'1'00000'10110'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.rtz.x.f.w", InstId::vfncvt_rtz_x_f_w,
	0b010010'1'00000'10111'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.f.xu.w", InstId::vfncvt_f_xu_w,
	0b010010'1'00000'10010'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.f.x.w", InstId::vfncvt_f_x_w,
	0b010010'1'00000'10011'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.f.f.w", InstId::vfncvt_f_f_w,
	0b010010'1'00000'10100'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfncvt.rod.f.f.w", InstId::vfncvt_rod_f_f_w,
	0b010010'1'00000'10101'001'00000'1010111,
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfredusum.vs", InstId::vfredusum_vs,
        0b000001'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfredosum.vs", InstId::vfredosum_vs,
        0b000011'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfredmin.vs", InstId::vfredmin_vs,
        0b000101'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfredmax.vs", InstId::vfredmax_vs,
        0b000111'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwredusum.vs", InstId::vfwredusum_vs,
        0b110001'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwredosum.vs", InstId::vfwredosum_vs,
        0b110011'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfrsqrt7.v", InstId::vfrsqrt7_v,
        0b010011'0'00000'00100'001'00000'1010111, // Opcode
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vfrec7.v", InstId::vfrec7_v,
        0b010011'0'00000'00101'001'00000'1010111, // Opcode
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vfmin.vv", InstId::vfmin_vv,
        0b000100'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmin.vf", InstId::vfmin_vf,
        0b000100'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfmax.vv", InstId::vfmax_vv,
        0b000110'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfmax.vf", InstId::vfmax_vf,
        0b000110'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfsgnj.vv", InstId::vfsgnj_vv,
        0b001000'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfsgnj.vf", InstId::vfsgnj_vf,
        0b001000'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfsgnjn.vv", InstId::vfsgnjn_vv,
        0b001001'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfsgnjn.vf", InstId::vfsgnjn_vf,
        0b001001'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      { "vfsgnjx.vv", InstId::vfsgnjx_vv,
        0b001010'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfsgnjx.vf", InstId::vfsgnjx_vf,
        0b001010'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::V, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::FpReg, OperandMode::Read, rs2Mask,
      },

      // Vector crypto (Zvbb)
      { "vandn.vv", InstId::vandn_vv,
	0b000001'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vandn.vx", InstId::vandn_vx,
	0b000001'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vbrev.v", InstId::vbrev_v,
	0b010010'0'00000'01010'010'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vbrev8.v", InstId::vbrev8_v,
	0b010010'0'00000'01000'010'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vrev8.v", InstId::vrev8_v,
	0b010010'0'00000'01001'010'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vclz.v", InstId::vclz_v,
	0b010010'0'00000'01100'010'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vctz.v", InstId::vctz_v,
	0b010010'0'00000'01101'010'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vcpop.v", InstId::vcpop_v,
	0b010010'0'00000'01110'010'00000'1010111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vrol.vv", InstId::vrol_vv,
	0b010101'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vrol.vx", InstId::vrol_vx,
	0b010101'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vror.vv", InstId::vror_vv,
	0b010100'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vror.vx", InstId::vror_vx,
	0b010100'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vror.vi", InstId::vror_vi,
	0b010100'0'00000'00000'011'00000'1010111, // Opcode
        0b111110'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::Imm, OperandMode::None, rs1Mask,
      },

      { "vwsll.vv", InstId::vwsll_vv,
	0b110101'0'00000'00000'000'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vwsll.vx", InstId::vwsll_vx,
	0b110101'0'00000'00000'100'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vwsll.vi", InstId::vwsll_vi,
	0b110101'0'00000'00000'011'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbb, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::Imm, OperandMode::None, rs1Mask,
      },

      // Vector bfloat conversions (Zvfbfmin)
      { "vfncvtbf16.f.f.w", InstId::vfncvtbf16_f_f_w,
        0b010010'0'00000'11101'001'00000'1010111, // Opcode
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvfbfmin, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      { "vfwcvtbf16.f.f.v", InstId::vfwcvtbf16_f_f_v,
        0b010010'0'00000'01101'001'00000'1010111, // Opcode
        0b111111'0'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvfbfmin, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask
      },

      // Vector BF16 widening mul-add (Zvfbfwma)
      { "vfwmaccbf16.vv", InstId::vfwmaccbf16_vv,
        0b111011'0'00000'00000'001'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvfbfwma, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vfwmaccbf16.vf", InstId::vfwmaccbf16_vf,
        0b111011'0'00000'00000'101'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvfbfwma, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::FpReg, OperandMode::Read, rs1Mask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      // Vector crypto (Zvbc)
      { "vclmul.vv", InstId::vclmul_vv,
	0b001100'0'00000'00000'010'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbc, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vclmul.vx", InstId::vclmul_vx,
	0b001100'0'00000'00000'110'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbc, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "vclmulh.vv", InstId::vclmulh_vv,
	0b001101'0'00000'00000'010'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbc, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vclmulh.vx", InstId::vclmulh_vx,
	0b001101'0'00000'00000'110'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvbc, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      // Vector crypto (zvkg)
      { "vghsh.vv", InstId::vghsh_vv,
	0b101100'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkg, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vgmul.vv", InstId::vgmul_vv,
	0b101000'1'00000'10001'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkg, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      // Vector crypto (zvkned)
      { "vaesdf.vv", InstId::vaesdf_vv,
	0b101000'1'00000'00001'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaesdf.vs", InstId::vaesdf_vs,
	0b101001'1'00000'00001'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaesef.vv", InstId::vaesef_vv,
	0b101000'1'00000'00011'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaesef.vs", InstId::vaesef_vs,
	0b101001'1'00000'00011'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaesem.vv", InstId::vaesem_vv,
	0b101000'1'00000'00010'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaesem.vs", InstId::vaesem_vs,
	0b101001'1'00000'00010'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaesdm.vv", InstId::vaesdm_vv,
	0b101000'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaesdm.vs", InstId::vaesdm_vs,
	0b101001'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vaeskf1.vi", InstId::vaeskf1_vi,
	0b100010'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, rs1Mask,
      },

      { "vaeskf2.vi", InstId::vaeskf2_vi,
	0b101010'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, rs1Mask,
      },

      { "vaesz.vs", InstId::vaesz_vs,
	0b101001'1'00000'00111'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvkned, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      // Vector crypto (Zvknha/b)
      { "vsha2ms.vv", InstId::vsha2ms_vv,
	0b101000'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvknha, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vsha2ch.vv", InstId::vsha2ch_vv,
	0b101110'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvknha, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vsha2cl.vv", InstId::vsha2cl_vv,
	0b101111'1'00000'00000'000'00000'1110111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvknha, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      // Vector crypto (Zvksed)
      { "vsm4k.vi", InstId::vsm4k_vi,
	0b100001'1'00000'00000'010'00000'1110111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvksed, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, rs1Mask,
      },

      { "vsm4r.vv", InstId::vsm4r_vv,
	0b101000'1'00000'10000'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvksed, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vsm4r.vs", InstId::vsm4r_vs,
	0b101001'1'00000'10000'010'00000'1110111, // Opcode
        0b111111'1'00000'11111'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvksed, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      // Vector crypto (Zvksh)
      { "vsm3me.vv", InstId::vsm3me_vv,
	0b100000'0'00000'00000'010'00000'1010111, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvksh, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
      },

      { "vsm3c.vi", InstId::vsm3c_vi,
	0b101011'1'00000'00000'010'00000'1010111, // Opcode
        0b111111'1'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvksh, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs2Mask,
        OperandType::Imm, OperandMode::None, rs1Mask,
      },


      // Crypto

      { "aes32dsi", InstId::aes32dsi,
        0b0010101'00000'00000'000'00000'0110011, // Opcode
        0b0011111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknd, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, 0xc0000000,
      },

      { "aes32dsmi", InstId::aes32dsmi,
        0b0010111'00000'00000'000'00000'0110011, // Opcode
        0b0011111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknd, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, 0xc0000000,
      },

      { "aes32esi", InstId::aes32esi,
        0b0010001'00000'00000'000'00000'0110011, // Opcode
        0b0011111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zkne, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, 0xc0000000,
      },

      { "aes32esmi", InstId::aes32esmi,
        0b0010011'00000'00000'000'00000'0110011, // Opcode
        0b0011111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zkne, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, 0xc0000000,
      },

      { "aes64ds", InstId::aes64ds,
        0b0011101'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknd, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "aes64dsm", InstId::aes64dsm,
        0b0011111'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknd, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "aes64es", InstId::aes64es,
        0b0011001'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zkne, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "aes64esm", InstId::aes64esm,
        0b0011011'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zkne, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "aes64im", InstId::aes64im,
        0b0011000'00000'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknd, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "aes64ks1i", InstId::aes64ks1i,
        0b0011000'10000'00000'001'00000'0010011, // Opcode
        0b1111111'10000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zkne, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, 0x00f00000,
      },

      { "aes64ks2", InstId::aes64ks2,
        0b0111111'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zkne, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "sha256sig0", InstId::sha256sig0,
        0b0001000'00010'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sha256sig1", InstId::sha256sig1,
        0b0001000'00011'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sha256sum0", InstId::sha256sum0,
        0b0001000'00000'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sha256sum1", InstId::sha256sum1,
        0b0001000'00001'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sha512sig0h", InstId::sha512sig0h,
        0b0101110'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "sha512sig0l", InstId::sha512sig0l,
        0b0101010'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "sha512sig1h", InstId::sha512sig1h,
        0b0101111'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "sha512sig1l", InstId::sha512sig1l,
        0b0101011'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "sha512sum0r", InstId::sha512sum0r,
        0b0101000'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "sha512sum1r", InstId::sha512sum1r,
        0b0101001'00000'00000'000'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "sha512sig0", InstId::sha512sig0,
        0b0001000'00110'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sha512sig1", InstId::sha512sig1,
        0b0001000'00111'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sha512sum0", InstId::sha512sum0,
        0b0001000'00100'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sha512sum1", InstId::sha512sum1,
        0b0001000'00101'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zknh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sm3p0", InstId::sm3p0,
        0b0001000'01000'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zksh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sm3p1", InstId::sm3p1,
        0b0001000'01001'00000'001'00000'0010011, // Opcode
        0b1111111'11111'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zksh, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
      },

      { "sm4ed", InstId::sm4ed,
        0b0011000'00000'00000'000'00000'0110011, // Opcode
        0b0011111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zksed, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, 0xc0000000,
      },

      { "sm4ks", InstId::sm4ks,
        0b0011010'00000'00000'000'00000'0110011, // Opcode
        0b0011111'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zksed, RvFormat::R,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::Imm, OperandMode::None, 0xc0000000,
      },

      // Vector dot product.
      { "vqdot.vv", InstId::vqdot_vv,
        0b101100'0'00000'00000'010'00000'1011011, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvqdot, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vqdot.vx", InstId::vqdot_vx,
        0b101100'0'00000'00000'110'00000'1011011, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvqdot, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vqdotu.vv", InstId::vqdotu_vv,
        0b101000'0'00000'00000'010'00000'1011011, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvqdot, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vqdotu.vx", InstId::vqdotu_vx,
        0b101000'0'00000'00000'110'00000'1011011, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvqdot, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vqdotsu.vv", InstId::vqdotsu_vv,
        0b101010'0'00000'00000'010'00000'1011011, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvqdot, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
	OperandType::VecReg, OperandMode::Read, rs2Mask,
      },

      { "vqdotsu.vx", InstId::vqdotsu_vx,
        0b101010'0'00000'00000'110'00000'1011011, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvqdot, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      { "vqdotus.vx", InstId::vqdotus_vx,
        0b101110'0'00000'00000'110'00000'1011011, // Opcode
        0b111111'0'00000'00000'111'00000'1111111, // Mask of opcode bits
        RvExtension::Zvqdot, RvFormat::R,
        OperandType::VecReg, OperandMode::Write, rdMask,
        OperandType::VecReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask,
      },

      // TLB invalidate (svinval)

      { "sinval.vma", InstId::sinval_vma, 0x16000073, 0xfe007fff,
        RvExtension::S, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "sfence.w.inval", InstId::sfence_w_inval, 0x18000073, 0xffffffff,
        RvExtension::S, RvFormat::I
      },

      { "sfence.inval.ir", InstId::sfence_inval_ir, 0x18100073, 0xffffffff,
        RvExtension::S, RvFormat::I
      },

      // Cache block management (zicbom)
      { "cbo.clean", InstId::cbo_clean, 0x0010200f, 0xfff07fff,
	RvExtension::Zicbom, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "cbo.flush", InstId::cbo_flush, 0x0020200f, 0xfff07fff,
	RvExtension::Zicbom, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "cbo.inval", InstId::cbo_inval, 0x0000200f, 0xfff07fff,
	RvExtension::Zicbom, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      // Cache block zero (zicboz)
      { "cbo.zero", InstId::cbo_zero, 0x0040200f, 0xfff07fff,
	RvExtension::Zicboz, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      // Cache block prefetch.
      { "prefetch.i", InstId::prefetch_i, 0x00006013, 0x01f07fff,
	RvExtension::Zicbop, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, 0xfe000000
      },

      { "prefetch.r", InstId::prefetch_r, 0x00106013, 0x01f07fff,
	RvExtension::Zicbop, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, 0xfe000000
      },

      { "prefetch.w", InstId::prefetch_w, 0x00306013, 0x01f07fff,
	RvExtension::Zicbop, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::Imm, OperandMode::None, 0xfe000000
      },

      // Zawrs
      { "wrs.nto", InstId::wrs_nto, 0x00d00073, 0xffffffff,
	RvExtension::Zawrs, RvFormat::I
      },

      { "wrs.sto", InstId::wrs_sto, 0x01d00073, 0xffffffff,
	RvExtension::Zawrs, RvFormat::I
      },

      // Hypervisor
      { "hfence_vvma", InstId::hfence_vvma, 0x22000073, 0xfe007fff,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "hfence_gvma", InstId::hfence_gvma, 0x62000073, 0xfe007fff,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "hlv.b", InstId::hlv_b, 0x60004073, 0xfff0707f,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlv.bu", InstId::hlv_bu, 0x60104073, 0xfff0707f,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlv.h", InstId::hlv_h, 0x62004073, 0xfff0707f,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlv.hu", InstId::hlv_hu, 0x62104073, 0xfff0707f,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlv.w", InstId::hlv_w, 0x68004073, 0xfff0707f,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlvx_hu", InstId::hlvx_hu, 0x64304073, 0xfff0707f,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlvx_wu", InstId::hlvx_wu, 0x68304073, 0xfff0707f,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hsv.b", InstId::hsv_b, 0x62004073, 0xfe007fff,
	RvExtension::H, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hsv.h", InstId::hsv_h, 0x66004073, 0xfe007fff,
	RvExtension::H, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hsv.w", InstId::hsv_w, 0x6a004073, 0xfe007fff,
	RvExtension::H, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlv.wu", InstId::hlv_wu, 0x68104073, 0xfff0707f,
	RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hlv.d", InstId::hlv_d, 0x6c004073, 0xfff0707f,
	RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hsv.d", InstId::hsv_d, 0x6e004073, 0xfe007fff,
	RvExtension::H, RvFormat::S,
	OperandType::IntReg, OperandMode::Read, rs2Mask,
	OperandType::IntReg, OperandMode::Read, rs1Mask
      },

      { "hinval.vvma", InstId::hinval_vvma, 0x26000073, 0xfe007fff,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      { "hinval.gvma", InstId::hinval_gvma, 0x66000073, 0xfe007fff,
        RvExtension::H, RvFormat::I,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask
      },

      // Zicond
      { "czero.eqz", InstId::czero_eqz,
	0b0000111'00000'00000'101'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
	RvExtension::Zicond, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "czero.nez", InstId::czero_nez,
	0b0000111'00000'00000'111'00000'0110011, // Opcode
        0b1111111'00000'00000'111'00000'1111111, // Mask of opcode bits
	RvExtension::Zicond, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      // Zcb
      { "c.lbu", InstId::c_lbu,
	0b100000'000'00'000'00,
	0b111111'000'00'000'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b0000011100,
	OperandType::IntReg, OperandMode::Read,  0b1110000000,
	OperandType::Imm, OperandMode::None,     0b0001100000 },

      { "c.lhu", InstId::c_lhu,
	0b100001'000'00'000'00,
	0b111111'000'10'000'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b0000011100,
	OperandType::IntReg, OperandMode::Read,  0b1110000000,
	OperandType::Imm, OperandMode::None, 	 0b0000100000 },

      { "c.lh", InstId::c_lh,
	0b100001'000'10'000'00,
	0b111111'000'10'000'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b0000011100,
	OperandType::IntReg, OperandMode::Read,  0b1110000000,
	OperandType::Imm, OperandMode::None, 	 0b0000100000 },

      { "c.sb", InstId::c_sb,
	0b100010'000'00'000'00,
	0b111111'000'00'000'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0b0000011100,
	OperandType::IntReg, OperandMode::Read, 0b1110000000,
	OperandType::Imm, OperandMode::None,    0b0001100000 },

      { "c.sh", InstId::c_sh,
	0b100011'000'00'000'00,
	0b111111'000'10'000'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Read, 0b0000011100,
	OperandType::IntReg, OperandMode::Read, 0b1110000000,
	OperandType::Imm, OperandMode::None,    0b0001100000 },

      { "c.zext.b", InstId::c_zext_b,
	0b100111'000'11'000'01,
	0b111111'000'11'111'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b1110000000,
	OperandType::IntReg, OperandMode::Read, 0b1110000000 },

      { "c.sext.b", InstId::c_sext_b,
	0b100111'000'11'001'01,
	0b111111'000'11'111'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b1110000000,
	OperandType::IntReg, OperandMode::Read, 0b1110000000 },

      { "c.zext.h", InstId::c_zext_h,
	0b100111'000'11'010'01,
	0b111111'000'11'111'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b1110000000,
	OperandType::IntReg, OperandMode::Read, 0b1110000000 },

      { "c.sext.h", InstId::c_sext_h,
	0b100111'000'11'011'01,
	0b111111'000'11'111'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b1110000000,
	OperandType::IntReg, OperandMode::Read, 0b1110000000 },

      { "c.zext.w", InstId::c_zext_w,
	0b100111'000'11'100'01,
	0b111111'000'11'111'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b1110000000,
	OperandType::IntReg, OperandMode::Read, 0b1110000000 },

      { "c.not", InstId::c_not,
	0b100111'000'11'101'01,
	0b111111'000'11'111'11,
	RvExtension::Zcb, RvFormat::None,
	OperandType::IntReg, OperandMode::Write, 0b1110000000,
	OperandType::IntReg, OperandMode::Read, 0b1110000000 },

      { "c.mul", InstId::c_mul,
	0b100111'000'10'000'01,
	0b111111'000'11'000'11,
	RvExtension::Zcb, RvFormat::None,
        OperandType::IntReg, OperandMode::Write, 0b1110000000,
	OperandType::IntReg, OperandMode::Read, 0b1110000000,
	OperandType::IntReg, OperandMode::Read,  0b0000011100 },

      // Zfa
      { "fcvtmod.w.d", InstId::fcvtmod_w_d,
	0b1100001'01000'00000'001'00000'1010011,
	0b1100001'11111'00000'111'00000'1010011,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fli.h", InstId::fli_h,
	0b1111010'00001'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, rs1Mask },

      { "fli.s", InstId::fli_s,
	0b1111000'00001'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, rs1Mask },

      { "fli.d", InstId::fli_d,
	0b1111001'00001'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::Imm, OperandMode::None, rs1Mask },

      { "fleq.h", InstId::fleq_h,
	0b1010010'00000'00000'100'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fleq.s", InstId::fleq_s,
	0b1010000'00000'00000'100'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fleq.d", InstId::fleq_d,
	0b1010001'00000'00000'100'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fltq.h", InstId::fltq_h,
	0b1010010'00000'00000'101'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fltq.s", InstId::fltq_s,
	0b1010000'00000'00000'101'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fltq.d", InstId::fltq_d,
	0b1010001'00000'00000'101'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmaxm.h", InstId::fmaxm_h,
	0b0010110'00000'00000'011'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmaxm.s", InstId::fmaxm_s,
	0b0010100'00000'00000'011'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmaxm.d", InstId::fmaxm_d,
	0b0010101'00000'00000'011'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fminm.h", InstId::fminm_h,
	0b0010110'00000'00000'010'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fminm.s", InstId::fminm_s,
	0b0010100'00000'00000'010'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fminm.d", InstId::fminm_d,
	0b0010101'00000'00000'010'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask,
	OperandType::FpReg, OperandMode::Read, rs2Mask },

      { "fmvh.x.d", InstId::fmvh_x_d,
	0b1110001'00001'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fmvp.d.x", InstId::fmvp_d_x,
	0b1011001'00000'00000'000'00000'1010011,
	0b1111111'00000'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "fround.h", InstId::fround_h,
	0b0100010'00100'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fround.s", InstId::fround_s,
	0b0100000'00100'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "fround.d", InstId::fround_d,
	0b0100001'00100'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "froundnx.h", InstId::froundnx_h,
	0b0100010'00101'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "froundnx.s", InstId::froundnx_s,
	0b0100000'00101'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "froundnx.d", InstId::froundnx_d,
	0b0100001'00101'00000'000'00000'1010011,
	0b1111111'11111'00000'111'00000'1111111,
	RvExtension::Zfa, RvFormat::R,
	OperandType::FpReg, OperandMode::Write, rdMask,
	OperandType::FpReg, OperandMode::Read, rs1Mask },

      { "amocas.w", InstId::amocas_w, 0x2800202f, 0xf800707f,
	RvExtension::Zacas, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amocas.d", InstId::amocas_d, 0x2800302f, 0xf800707f,
	RvExtension::Zacas, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "amocas.q", InstId::amocas_q, 0x2800402f, 0xf800707f,
	RvExtension::Zacas, RvFormat::R,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask,
	OperandType::IntReg, OperandMode::Read, rs2Mask },

      { "mop.r", InstId::mop_r, 0x81c04073, 0xfff0707f,
	RvExtension::Zimop, RvFormat::I,
	OperandType::IntReg, OperandMode::Write, rdMask,
	OperandType::IntReg, OperandMode::Read, rs1Mask },

      { "mop.rr", InstId::mop_rr, 0x82004073, 0xfe00707f,
        RvExtension::Zimop, RvFormat::I,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask,
        OperandType::IntReg, OperandMode::Read, rs2Mask, },

      { "c.mop", InstId::c_mop, 0x6081, 0x6781,
        RvExtension::Zcmop, RvFormat::I,
        OperandType::IntReg, OperandMode::Write, rdMask,
        OperandType::IntReg, OperandMode::Read, rs1Mask },
      };
}
