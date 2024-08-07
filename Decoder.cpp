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

#include "DecodedInst.hpp"
#include "IntRegs.hpp"
#include "Decoder.hpp"
#include "instforms.hpp"


using namespace WdRiscv;


Decoder::Decoder() = default;


Decoder::~Decoder() = default;


void
Decoder::decode(uint64_t addr, uint64_t physAddr, uint32_t inst, DecodedInst& di) const
{
  // For vector load/store ops, op3 captures the number of fields
  // (non-zero for segmented, and whole-register ld/st).

  uint32_t op0 = 0, op1 = 0, op2 = 0, op3 = 0;

  const InstEntry& entry = decode(inst, op0, op1, op2, op3);

  di.reset(addr, physAddr, inst, &entry, op0, op1, op2, op3);

  // Set the mask bit for vector instructions.  Set acquire/release bits
  // for atomic instructions.
  if (di.instEntry())
    {
      if (di.instEntry()->isVector())
	{
	  bool masked = ((inst >> 25) & 1) == 0;  // Bit 25 of instruction
	  di.setMasked(masked);
	  di.setVecFieldCount(op3);
	}
    }
}


const InstEntry&
Decoder::decodeFp(uint32_t inst, uint32_t& op0, uint32_t& op1, uint32_t& op2) const
{
  RFormInst rform(inst);

  op0 = rform.bits.rd, op1 = rform.bits.rs1, op2 = rform.bits.rs2;

  unsigned f7 = rform.bits.funct7, f3 = rform.bits.funct3;
  unsigned top5 = f7 >> 2;

  if ((f7 & 3) == 1)
    {
      if (top5 == 0)            return instTable_.getEntry(InstId::fadd_d);
      if (top5 == 1)            return instTable_.getEntry(InstId::fsub_d);
      if (top5 == 2)            return instTable_.getEntry(InstId::fmul_d);
      if (top5 == 3)            return instTable_.getEntry(InstId::fdiv_d);
      if (top5 == 4)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fsgnj_d);
	  if (f3 == 1)          return instTable_.getEntry(InstId::fsgnjn_d);
	  if (f3 == 2)          return instTable_.getEntry(InstId::fsgnjx_d);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 5)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fmin_d);
	  if (f3 == 2)          return instTable_.getEntry(InstId::fminm_d);
	  if (f3 == 1)          return instTable_.getEntry(InstId::fmax_d);
	  if (f3 == 3)          return instTable_.getEntry(InstId::fmaxm_d);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 8)
        {
          if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_d_s);
          if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_d_h);
          if (op2 == 4)         return instTable_.getEntry(InstId::fround_d);
          if (op2 == 5)         return instTable_.getEntry(InstId::froundnx_d);
          return instTable_.getEntry(InstId::illegal);
        }
      if (top5==0xb and op2==0) return instTable_.getEntry(InstId::fsqrt_d);
      if (top5 == 0x14)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fle_d);
	  if (f3 == 4)          return instTable_.getEntry(InstId::fleq_d);
	  if (f3 == 1)          return instTable_.getEntry(InstId::flt_d);
	  if (f3 == 5)          return instTable_.getEntry(InstId::fltq_d);
	  if (f3 == 2)          return instTable_.getEntry(InstId::feq_d);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x18)
	{
	  if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_w_d);
	  if (op2 ==8 && f3==1) return instTable_.getEntry(InstId::fcvtmod_w_d);
	  if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_wu_d);
	  if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_l_d);
	  if (op2 == 3)         return instTable_.getEntry(InstId::fcvt_lu_d);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1a)
	{
	  if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_d_w);
	  if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_d_wu);
	  if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_d_l);
	  if (op2 == 3)         return instTable_.getEntry(InstId::fcvt_d_lu);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1c)
	{
	  if (op2==0 and f3==0) return instTable_.getEntry(InstId::fmv_x_d);
	  if (op2==0 and f3==1) return instTable_.getEntry(InstId::fclass_d);
	  if (op2==1 and f3==0) return instTable_.getEntry(InstId::fmvh_x_d);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1e)
	{
	  if (op2==0 and f3==0) return instTable_.getEntry(InstId::fmv_d_x);
	  if (op2==1 and f3==0) return instTable_.getEntry(InstId::fli_d);
	}
      if (top5 == 0x26)
	if (f3==0) return instTable_.getEntry(InstId::fmvp_d_x);

      return instTable_.getEntry(InstId::illegal);
    }

  if ((f7 & 3) == 0)
    {
      if (top5 == 0)            return instTable_.getEntry(InstId::fadd_s);
      if (top5 == 1)            return instTable_.getEntry(InstId::fsub_s);
      if (top5 == 2)            return instTable_.getEntry(InstId::fmul_s);
      if (top5 == 3)            return instTable_.getEntry(InstId::fdiv_s);
      if (top5 == 4)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fsgnj_s);
	  if (f3 == 1)          return instTable_.getEntry(InstId::fsgnjn_s);
	  if (f3 == 2)          return instTable_.getEntry(InstId::fsgnjx_s);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 5)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fmin_s);
	  if (f3 == 2)          return instTable_.getEntry(InstId::fminm_s);
	  if (f3 == 1)          return instTable_.getEntry(InstId::fmax_s);
	  if (f3 == 3)          return instTable_.getEntry(InstId::fmaxm_s);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 8)
        {
          if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_s_d);
          if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_s_h);
          if (op2 == 4)         return instTable_.getEntry(InstId::fround_s);
          if (op2 == 5)         return instTable_.getEntry(InstId::froundnx_s);
          if (op2 == 6)         return instTable_.getEntry(InstId::fcvt_s_bf16);
          return instTable_.getEntry(InstId::illegal);
        }
      if (top5==0xb and op2==0) return instTable_.getEntry(InstId::fsqrt_s);
      if (top5 == 0x14)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fle_s);
	  if (f3 == 1)          return instTable_.getEntry(InstId::flt_s);
	  if (f3 == 2)          return instTable_.getEntry(InstId::feq_s);
	  if (f3 == 4)          return instTable_.getEntry(InstId::fleq_s);
	  if (f3 == 5)          return instTable_.getEntry(InstId::fltq_s);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x18)
	{
	  if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_w_s);
	  if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_wu_s);
	  if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_l_s);
	  if (op2 == 3)         return instTable_.getEntry(InstId::fcvt_lu_s);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1a)
	{
	  if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_s_w);
	  if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_s_wu);
	  if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_s_l);
	  if (op2 == 3)         return instTable_.getEntry(InstId::fcvt_s_lu);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1c)
	{
	  if (op2==0 and f3==0) return instTable_.getEntry(InstId::fmv_x_w);
	  if (op2==0 and f3==1) return instTable_.getEntry(InstId::fclass_s);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1e)
	{
	  if (op2==0 and f3==0) return instTable_.getEntry(InstId::fmv_w_x);
	  if (op2==1 and f3==0) return instTable_.getEntry(InstId::fli_s);
	}

      return instTable_.getEntry(InstId::illegal);
    }

  if ((f7 & 3) == 2)
    {
      if (top5 == 0)            return instTable_.getEntry(InstId::fadd_h);
      if (top5 == 1)            return instTable_.getEntry(InstId::fsub_h);
      if (top5 == 2)            return instTable_.getEntry(InstId::fmul_h);
      if (top5 == 3)            return instTable_.getEntry(InstId::fdiv_h);
      if (top5 == 4)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fsgnj_h);
	  if (f3 == 1)          return instTable_.getEntry(InstId::fsgnjn_h);
	  if (f3 == 2)          return instTable_.getEntry(InstId::fsgnjx_h);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 5)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fmin_h);
	  if (f3 == 2)          return instTable_.getEntry(InstId::fminm_h);
	  if (f3 == 1)          return instTable_.getEntry(InstId::fmax_h);
	  if (f3 == 3)          return instTable_.getEntry(InstId::fmaxm_h);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 8)
        {
          if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_h_s);
          if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_h_d);
          if (op2 == 4)         return instTable_.getEntry(InstId::fround_h);
          if (op2 == 5)         return instTable_.getEntry(InstId::froundnx_h);
          if (op2 == 8)         return instTable_.getEntry(InstId::fcvt_bf16_s);
          return instTable_.getEntry(InstId::illegal);
        }
      if (top5==0xb and op2==0) return instTable_.getEntry(InstId::fsqrt_h);
      if (top5 == 0x14)
	{
	  if (f3 == 0)          return instTable_.getEntry(InstId::fle_h);
	  if (f3 == 4)          return instTable_.getEntry(InstId::fleq_h);
	  if (f3 == 1)          return instTable_.getEntry(InstId::flt_h);
	  if (f3 == 5)          return instTable_.getEntry(InstId::fltq_h);
	  if (f3 == 2)          return instTable_.getEntry(InstId::feq_h);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x18)
	{
	  if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_w_h);
	  if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_wu_h);
	  if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_l_h);
	  if (op2 == 3)         return instTable_.getEntry(InstId::fcvt_lu_h);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1a)
	{
	  if (op2 == 0)         return instTable_.getEntry(InstId::fcvt_h_w);
	  if (op2 == 1)         return instTable_.getEntry(InstId::fcvt_h_wu);
	  if (op2 == 2)         return instTable_.getEntry(InstId::fcvt_h_l);
	  if (op2 == 3)         return instTable_.getEntry(InstId::fcvt_h_lu);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1c)
	{
	  if (op2==0 and f3==0) return instTable_.getEntry(InstId::fmv_x_h);
	  if (op2==0 and f3==1) return instTable_.getEntry(InstId::fclass_h);
	  return instTable_.getEntry(InstId::illegal);
	}
      if (top5 == 0x1e)
	{
	  if (op2==0 and f3==0) return instTable_.getEntry(InstId::fmv_h_x);
	  if (op2==1 and f3==0) return instTable_.getEntry(InstId::fli_h);
	}

      return instTable_.getEntry(InstId::illegal);
    }

  return instTable_.getEntry(InstId::illegal);
}


inline bool
isMaskedVec(uint32_t inst)
{
  return ((inst >> 25) & 1) == 0;
}


// Least sig 7 bits already determined to be: 1010111
const InstEntry&
Decoder::decodeVec(uint32_t inst, uint32_t& op0, uint32_t& op1, uint32_t& op2,
		   uint32_t& op3) const
{
  RFormInst rform(inst);
  unsigned f3 = rform.bits.funct3, f6 = rform.top6();
  unsigned vm = (inst >> 25) & 1;
  bool masked = vm == 0;
  const InstEntry& illegal = instTable_.getEntry(InstId::illegal);

  op3 = 0;

  if (f3 == 0)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2; // operand order reversed
      op2 = rform.bits.rs1;

      switch (f6)
        {
        case 0:    return instTable_.getEntry(InstId::vadd_vv);
        case 1:    return instTable_.getEntry(InstId::vandn_vv);
        case 2:    return instTable_.getEntry(InstId::vsub_vv);
        case 4:    return instTable_.getEntry(InstId::vminu_vv);
        case 5:    return instTable_.getEntry(InstId::vmin_vv);
        case 6:    return instTable_.getEntry(InstId::vmaxu_vv);
        case 7:    return instTable_.getEntry(InstId::vmax_vv);
        case 9:    return instTable_.getEntry(InstId::vand_vv);
        case 0xa:  return instTable_.getEntry(InstId::vor_vv);
        case 0xb:  return instTable_.getEntry(InstId::vxor_vv);
        case 0xc:  return instTable_.getEntry(InstId::vrgather_vv);
        case 0xe:  return instTable_.getEntry(InstId::vrgatherei16_vv);
        case 0x10: return instTable_.getEntry(InstId::vadc_vvm);
        case 0x11: return instTable_.getEntry(InstId::vmadc_vvm);
        case 0x12: return instTable_.getEntry(InstId::vsbc_vvm);
        case 0x13: return instTable_.getEntry(InstId::vmsbc_vvm);
	case 0x14: return instTable_.getEntry(InstId::vror_vv);
	case 0x15: return instTable_.getEntry(InstId::vrol_vv);
        case 0x17:
          if (vm == 0) return instTable_.getEntry(InstId::vmerge_vvm);
          if (vm == 1)
            {
              std::swap(op1, op2);  // Per spec !
              if (op2 == 0) return instTable_.getEntry(InstId::vmv_v_v);
            }
          break;
        case 0x18: return instTable_.getEntry(InstId::vmseq_vv);
        case 0x19: return instTable_.getEntry(InstId::vmsne_vv);
        case 0x1a: return instTable_.getEntry(InstId::vmsltu_vv);
        case 0x1b: return instTable_.getEntry(InstId::vmslt_vv);
        case 0x1c: return instTable_.getEntry(InstId::vmsleu_vv);
        case 0x1d: return instTable_.getEntry(InstId::vmsle_vv);
        case 0x20: return instTable_.getEntry(InstId::vsaddu_vv);
        case 0x21: return instTable_.getEntry(InstId::vsadd_vv);
        case 0x22: return instTable_.getEntry(InstId::vssubu_vv);
        case 0x23: return instTable_.getEntry(InstId::vssub_vv);
        case 0x25: return instTable_.getEntry(InstId::vsll_vv);
        case 0x27: return instTable_.getEntry(InstId::vsmul_vv);
        case 0x28: return instTable_.getEntry(InstId::vsrl_vv);
        case 0x29: return instTable_.getEntry(InstId::vsra_vv);
        case 0x2a: return instTable_.getEntry(InstId::vssrl_vv);
        case 0x2b: return instTable_.getEntry(InstId::vssra_vv);
        case 0x2c: return instTable_.getEntry(InstId::vnsrl_wv);
        case 0x2d: return instTable_.getEntry(InstId::vnsra_wv);
        case 0x2e: return instTable_.getEntry(InstId::vnclipu_wv);
        case 0x2f: return instTable_.getEntry(InstId::vnclip_wv);
        case 0x30: return instTable_.getEntry(InstId::vwredsumu_vs);
        case 0x31: return instTable_.getEntry(InstId::vwredsum_vs);
	case 0x35: return instTable_.getEntry(InstId::vwsll_vv);
        }
      return instTable_.getEntry(InstId::illegal);  
    }

  if (f3 == 1)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2;
      op2 = rform.bits.rs1;

      switch (f6)
	{
	case 0:    return instTable_.getEntry(InstId::vfadd_vv);
	case 1:    return instTable_.getEntry(InstId::vfredusum_vs);
	case 2:    return instTable_.getEntry(InstId::vfsub_vv);
	case 3:    return instTable_.getEntry(InstId::vfredosum_vs);
	case 4:    return instTable_.getEntry(InstId::vfmin_vv);
	case 5:    return instTable_.getEntry(InstId::vfredmin_vs);
	case 6:    return instTable_.getEntry(InstId::vfmax_vv);
	case 7:    return instTable_.getEntry(InstId::vfredmax_vs);
	case 8:    return instTable_.getEntry(InstId::vfsgnj_vv);
	case 9:    return instTable_.getEntry(InstId::vfsgnjn_vv);
	case 0xa:  return instTable_.getEntry(InstId::vfsgnjx_vv);
        case 0x10:
	  if (op2 == 0)  return instTable_.getEntry(InstId::vfmv_f_s);
	  return instTable_.getEntry(InstId::illegal);
	case 0x12:
          switch (op2)
          {
            case 0:    return instTable_.getEntry(InstId::vfcvt_xu_f_v);
            case 1:    return instTable_.getEntry(InstId::vfcvt_x_f_v);
            case 2:    return instTable_.getEntry(InstId::vfcvt_f_xu_v);
            case 3:    return instTable_.getEntry(InstId::vfcvt_f_x_v);
            case 6:    return instTable_.getEntry(InstId::vfcvt_rtz_xu_f_v);
            case 7:    return instTable_.getEntry(InstId::vfcvt_rtz_x_f_v);
            case 8:    return instTable_.getEntry(InstId::vfwcvt_xu_f_v);
            case 9:    return instTable_.getEntry(InstId::vfwcvt_x_f_v);
            case 0xa:  return instTable_.getEntry(InstId::vfwcvt_f_xu_v);
            case 0xb:  return instTable_.getEntry(InstId::vfwcvt_f_x_v);
            case 0xc:  return instTable_.getEntry(InstId::vfwcvt_f_f_v);
            case 0xd:  return instTable_.getEntry(InstId::vfwcvtbf16_f_f_v);
            case 0xe:  return instTable_.getEntry(InstId::vfwcvt_rtz_xu_f_v);
            case 0xf:  return instTable_.getEntry(InstId::vfwcvt_rtz_x_f_v);
            case 0x10: return instTable_.getEntry(InstId::vfncvt_xu_f_w);
            case 0x11: return instTable_.getEntry(InstId::vfncvt_x_f_w);
            case 0x12: return instTable_.getEntry(InstId::vfncvt_f_xu_w);
            case 0x13: return instTable_.getEntry(InstId::vfncvt_f_x_w);
            case 0x14: return instTable_.getEntry(InstId::vfncvt_f_f_w);
            case 0x15: return instTable_.getEntry(InstId::vfncvt_rod_f_f_w);
            case 0x16: return instTable_.getEntry(InstId::vfncvt_rtz_xu_f_w);
            case 0x17: return instTable_.getEntry(InstId::vfncvt_rtz_x_f_w);
            case 0x1d: return instTable_.getEntry(InstId::vfncvtbf16_f_f_w);
          }
          break;
	case 0x13:
	  if (op2 == 0)    return instTable_.getEntry(InstId::vfsqrt_v);
	  if (op2 == 4)    return instTable_.getEntry(InstId::vfrsqrt7_v);
	  if (op2 == 5)    return instTable_.getEntry(InstId::vfrec7_v);
	  if (op2 == 0x10) return instTable_.getEntry(InstId::vfclass_v);
	  break;
	case 0x18: return instTable_.getEntry(InstId::vmfeq_vv);
	case 0x19: return instTable_.getEntry(InstId::vmfle_vv);
	case 0x1b: return instTable_.getEntry(InstId::vmflt_vv);
	case 0x1c: return instTable_.getEntry(InstId::vmfne_vv);
	case 0x20: return instTable_.getEntry(InstId::vfdiv_vv);
	case 0x24: return instTable_.getEntry(InstId::vfmul_vv);
	case 0x28:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmadd_vv);
	case 0x29:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmadd_vv);
	case 0x2a:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmsub_vv);
	case 0x2b:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmsub_vv);
	case 0x2c:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmacc_vv);
	case 0x2d:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmacc_vv);
	case 0x2e:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmsac_vv);
	case 0x2f:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmsac_vv);
	case 0x30: return instTable_.getEntry(InstId::vfwadd_vv);
	case 0x31: return instTable_.getEntry(InstId::vfwredusum_vs);
	case 0x32: return instTable_.getEntry(InstId::vfwsub_vv);
	case 0x33: return instTable_.getEntry(InstId::vfwredosum_vs);
	case 0x34: return instTable_.getEntry(InstId::vfwadd_wv);
	case 0x36: return instTable_.getEntry(InstId::vfwsub_wv);
	case 0x38: return instTable_.getEntry(InstId::vfwmul_vv);
        case 0x3b:
          std::swap(op1, op2);  // per spec
          return instTable_.getEntry(InstId::vfwmaccbf16_vv);
	case 0x3c:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwmacc_vv);
	case 0x3d:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwnmacc_vv);
	case 0x3e:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwmsac_vv);
	case 0x3f:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwnmsac_vv);
	}
      return instTable_.getEntry(InstId::illegal);
    }

  if (f3 == 2)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2; // operand order reversed
      op2 = rform.bits.rs1;

      switch(f6)
        {
        case 0:    return instTable_.getEntry(InstId::vredsum_vs);
        case 1:    return instTable_.getEntry(InstId::vredand_vs);
        case 2:    return instTable_.getEntry(InstId::vredor_vs);
        case 3:    return instTable_.getEntry(InstId::vredxor_vs);
        case 4:    return instTable_.getEntry(InstId::vredminu_vs);
        case 5:    return instTable_.getEntry(InstId::vredmin_vs);
        case 6:    return instTable_.getEntry(InstId::vredmaxu_vs);
        case 7:    return instTable_.getEntry(InstId::vredmax_vs);
        case 8:    return instTable_.getEntry(InstId::vaaddu_vv);
        case 9:    return instTable_.getEntry(InstId::vaadd_vv);
        case 0xa:  return instTable_.getEntry(InstId::vasubu_vv);
        case 0xb:  return instTable_.getEntry(InstId::vasub_vv);
	case 0xc:  return instTable_.getEntry(InstId::vclmul_vv);
	case 0xd:  return instTable_.getEntry(InstId::vclmulh_vv);
        case 0x10:
          if (op2 == 0)    return instTable_.getEntry(InstId::vmv_x_s);
          if (op2 == 0x10) return instTable_.getEntry(InstId::vcpop_m);
          if (op2 == 0x11) return instTable_.getEntry(InstId::vfirst_m);
          return instTable_.getEntry(InstId::illegal);
        case 0x12:
          if (op2 == 2)  return instTable_.getEntry(InstId::vzext_vf8);
          if (op2 == 4)  return instTable_.getEntry(InstId::vzext_vf4);
          if (op2 == 6)  return instTable_.getEntry(InstId::vzext_vf2);
          if (op2 == 3)  return instTable_.getEntry(InstId::vsext_vf8);
          if (op2 == 5)  return instTable_.getEntry(InstId::vsext_vf4);
          if (op2 == 7)  return instTable_.getEntry(InstId::vsext_vf2);
	  if (op2 == 8)  return instTable_.getEntry(InstId::vbrev8_v);
	  if (op2 == 9)  return instTable_.getEntry(InstId::vrev8_v);
	  if (op2 == 10)  return instTable_.getEntry(InstId::vbrev_v);
	  if (op2 == 12)  return instTable_.getEntry(InstId::vclz_v);
	  if (op2 == 13)  return instTable_.getEntry(InstId::vctz_v);
	  if (op2 == 14)  return instTable_.getEntry(InstId::vcpop_v);
          return instTable_.getEntry(InstId::illegal);
        case 0x14:
          if (op2 == 1)    return instTable_.getEntry(InstId::vmsbf_m);
          if (op2 == 2)    return instTable_.getEntry(InstId::vmsof_m);
          if (op2 == 3)    return instTable_.getEntry(InstId::vmsif_m);
          if (op2 == 0x10) return instTable_.getEntry(InstId::viota_m);
          if (op2 == 0x11) return instTable_.getEntry(InstId::vid_v);
          return instTable_.getEntry(InstId::illegal);
        case 0x17: return instTable_.getEntry(InstId::vcompress_vm);
        case 0x19: return masked? illegal : instTable_.getEntry(InstId::vmand_mm);
        case 0x1d: return masked? illegal : instTable_.getEntry(InstId::vmnand_mm);
        case 0x18: return masked? illegal : instTable_.getEntry(InstId::vmandn_mm);
        case 0x1b: return masked? illegal : instTable_.getEntry(InstId::vmxor_mm);
        case 0x1a: return masked? illegal : instTable_.getEntry(InstId::vmor_mm);
        case 0x1e: return masked? illegal : instTable_.getEntry(InstId::vmnor_mm);
        case 0x1c: return masked? illegal : instTable_.getEntry(InstId::vmorn_mm);
        case 0x1f: return masked? illegal : instTable_.getEntry(InstId::vmxnor_mm);
        case 0x20: return instTable_.getEntry(InstId::vdivu_vv);
        case 0x21: return instTable_.getEntry(InstId::vdiv_vv);
        case 0x22: return instTable_.getEntry(InstId::vremu_vv);
        case 0x23: return instTable_.getEntry(InstId::vrem_vv);
        case 0x24: return instTable_.getEntry(InstId::vmulhu_vv);
        case 0x25: return instTable_.getEntry(InstId::vmul_vv);
        case 0x26: return instTable_.getEntry(InstId::vmulhsu_vv);
        case 0x27: return instTable_.getEntry(InstId::vmulh_vv);
        case 0x29:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vmadd_vv);
        case 0x2b:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vnmsub_vv);
	case 0x2d:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vmacc_vv);
	case 0x2f:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vnmsac_vv);
        case 0x30: return instTable_.getEntry(InstId::vwaddu_vv);
        case 0x31: return instTable_.getEntry(InstId::vwadd_vv);
        case 0x32: return instTable_.getEntry(InstId::vwsubu_vv);
        case 0x33: return instTable_.getEntry(InstId::vwsub_vv);
        case 0x34: return instTable_.getEntry(InstId::vwaddu_wv);
        case 0x35: return instTable_.getEntry(InstId::vwadd_wv);
        case 0x36: return instTable_.getEntry(InstId::vwsubu_wv);
        case 0x37: return instTable_.getEntry(InstId::vwsub_wv);
        case 0x38: return instTable_.getEntry(InstId::vwmulu_vv);
        case 0x3a: return instTable_.getEntry(InstId::vwmulsu_vv);
        case 0x3b: return instTable_.getEntry(InstId::vwmul_vv);
        case 0x3c:
          std::swap(op1, op2);  // Spec is baffling.
          return instTable_.getEntry(InstId::vwmaccu_vv);
        case 0x3d:
          std::swap(op1, op2);  // Spec is baffling.
          return instTable_.getEntry(InstId::vwmacc_vv);
        case 0x3f:
          std::swap(op1, op2);  // Spec is baffling.
          return instTable_.getEntry(InstId::vwmaccsu_vv);
        }
      return instTable_.getEntry(InstId::illegal);  
    }

  if (f3 == 3)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2; // operand order reversed
      uint32_t uimm = rform.bits.rs1;             // Unsigned immediate.
      int32_t imm = (int32_t(uimm) << 27) >> 27;  // Sign extended immediate.
      op2 = imm;

      switch (f6)
        {
        case 0:    return instTable_.getEntry(InstId::vadd_vi);
        case 3:    return instTable_.getEntry(InstId::vrsub_vi);
        case 9:    return instTable_.getEntry(InstId::vand_vi);
        case 0xa:  return instTable_.getEntry(InstId::vor_vi);
        case 0xb:  return instTable_.getEntry(InstId::vxor_vi);
        case 0xc:  op2 = uimm; return instTable_.getEntry(InstId::vrgather_vi);
        case 0xe:  op2 = uimm; return instTable_.getEntry(InstId::vslideup_vi);
        case 0xf:  op2 = uimm; return instTable_.getEntry(InstId::vslidedown_vi);
        case 0x10: return instTable_.getEntry(InstId::vadc_vim);
        case 0x11: return instTable_.getEntry(InstId::vmadc_vim);
	case 0x14: op2 = uimm; return instTable_.getEntry(InstId::vror_vi); // Bit 26 is zero.
	case 0x15: op2 = uimm | 0x20; return instTable_.getEntry(InstId::vror_vi); // Bit 26 is 1.
	case 0x17:
          if (vm == 0) return instTable_.getEntry(InstId::vmerge_vim);
          if (vm == 1)
            {
              op1 = (int32_t(rform.bits.rs1) << 27) >> 27;
              op2 = rform.bits.rs2;
              if (op2 == 0) return instTable_.getEntry(InstId::vmv_v_i);
            }
          break;
        case 0x18: return instTable_.getEntry(InstId::vmseq_vi);
        case 0x19: return instTable_.getEntry(InstId::vmsne_vi);
        case 0x1c: return instTable_.getEntry(InstId::vmsleu_vi);
        case 0x1d: return instTable_.getEntry(InstId::vmsle_vi);
        case 0x1e: return instTable_.getEntry(InstId::vmsgtu_vi);
        case 0x1f: return instTable_.getEntry(InstId::vmsgt_vi);
        case 0x20: return instTable_.getEntry(InstId::vsaddu_vi);
        case 0x21: return instTable_.getEntry(InstId::vsadd_vi);
        case 0x25: op2 = uimm; return instTable_.getEntry(InstId::vsll_vi);
        case 0x27:
          if (imm == 0) return masked? illegal : instTable_.getEntry(InstId::vmv1r_v);
          if (imm == 1) return masked? illegal : instTable_.getEntry(InstId::vmv2r_v);
          if (imm == 3) return masked? illegal : instTable_.getEntry(InstId::vmv4r_v);
          if (imm == 7) return masked? illegal : instTable_.getEntry(InstId::vmv8r_v);
          break;
        case 0x28: op2 = uimm; return instTable_.getEntry(InstId::vsrl_vi);
        case 0x29: op2 = uimm; return instTable_.getEntry(InstId::vsra_vi);
        case 0x2a: op2 = uimm; return instTable_.getEntry(InstId::vssrl_vi);
        case 0x2b: op2 = uimm; return instTable_.getEntry(InstId::vssra_vi);
	case 0x2c: op2 = uimm; return instTable_.getEntry(InstId::vnsrl_wi);
	case 0x2d: op2 = uimm; return instTable_.getEntry(InstId::vnsra_wi);
        case 0x2e: op2 = uimm; return instTable_.getEntry(InstId::vnclipu_wi);
        case 0x2f: op2 = uimm; return instTable_.getEntry(InstId::vnclip_wi);
	case 0x35: op2 = uimm; return instTable_.getEntry(InstId::vwsll_vi);
        }
      return instTable_.getEntry(InstId::illegal);  
    }

  if (f3 == 4)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2; // operand order reversed
      op2 = rform.bits.rs1;

      switch (f6)
        {
        case 0:    return instTable_.getEntry(InstId::vadd_vx);
        case 1:    return instTable_.getEntry(InstId::vandn_vx);
        case 2:    return instTable_.getEntry(InstId::vsub_vx);
        case 3:    return instTable_.getEntry(InstId::vrsub_vx);
        case 4:    return instTable_.getEntry(InstId::vminu_vx);
        case 5:    return instTable_.getEntry(InstId::vmin_vx);
        case 6:    return instTable_.getEntry(InstId::vmaxu_vx);
        case 7:    return instTable_.getEntry(InstId::vmax_vx);
        case 9:    return instTable_.getEntry(InstId::vand_vx);
        case 0xa:  return instTable_.getEntry(InstId::vor_vx);
        case 0xb:  return instTable_.getEntry(InstId::vxor_vx);
        case 0xc:  return instTable_.getEntry(InstId::vrgather_vx);
        case 0xe:  return instTable_.getEntry(InstId::vslideup_vx);
        case 0xf:  return instTable_.getEntry(InstId::vslidedown_vx);
        case 0x10: return instTable_.getEntry(InstId::vadc_vxm);
        case 0x11: return instTable_.getEntry(InstId::vmadc_vxm);
        case 0x12: return instTable_.getEntry(InstId::vsbc_vxm);
        case 0x13: return instTable_.getEntry(InstId::vmsbc_vxm);
	case 0x14: return instTable_.getEntry(InstId::vror_vx);
	case 0x15: return instTable_.getEntry(InstId::vrol_vx);
        case 0x17:
          if (vm == 0) return instTable_.getEntry(InstId::vmerge_vxm);
          if (vm == 1)
            {
              std::swap(op1, op2);  // Per spec!
              if (op2 == 0) return instTable_.getEntry(InstId::vmv_v_x);
            }
          break;
        case 0x18: return instTable_.getEntry(InstId::vmseq_vx);
        case 0x19: return instTable_.getEntry(InstId::vmsne_vx);
        case 0x1a: return instTable_.getEntry(InstId::vmsltu_vx);
        case 0x1b: return instTable_.getEntry(InstId::vmslt_vx);
        case 0x1c: return instTable_.getEntry(InstId::vmsleu_vx);
        case 0x1d: return instTable_.getEntry(InstId::vmsle_vx);
        case 0x1e: return instTable_.getEntry(InstId::vmsgtu_vx);
        case 0x1f: return instTable_.getEntry(InstId::vmsgt_vx);
        case 0x20: return instTable_.getEntry(InstId::vsaddu_vx);
        case 0x21: return instTable_.getEntry(InstId::vsadd_vx);
        case 0x22: return instTable_.getEntry(InstId::vssubu_vx);
        case 0x23: return instTable_.getEntry(InstId::vssub_vx);
        case 0x25: return instTable_.getEntry(InstId::vsll_vx);
        case 0x27: return instTable_.getEntry(InstId::vsmul_vx);
        case 0x28: return instTable_.getEntry(InstId::vsrl_vx);
        case 0x29: return instTable_.getEntry(InstId::vsra_vx);
        case 0x2a: return instTable_.getEntry(InstId::vssrl_vx);
        case 0x2b: return instTable_.getEntry(InstId::vssra_vx);
        case 0x2c: return instTable_.getEntry(InstId::vnsrl_wx);
        case 0x2d: return instTable_.getEntry(InstId::vnsra_wx);
        case 0x2e: return instTable_.getEntry(InstId::vnclipu_wx);
        case 0x2f: return instTable_.getEntry(InstId::vnclip_wx);
	case 0x35: return instTable_.getEntry(InstId::vwsll_vx);
        }
      return instTable_.getEntry(InstId::illegal);  
    }

  if (f3 == 6)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2; // operand order reversed
      op2 = rform.bits.rs1;

      switch (f6)
        {
        case 8:    return instTable_.getEntry(InstId::vaaddu_vx);
        case 9:    return instTable_.getEntry(InstId::vaadd_vx);
        case 0xa:  return instTable_.getEntry(InstId::vasubu_vx);
        case 0xb:  return instTable_.getEntry(InstId::vasub_vx);
        case 0xc:  return instTable_.getEntry(InstId::vclmul_vx);
        case 0xd:  return instTable_.getEntry(InstId::vclmulh_vx);
        case 0xe:   return instTable_.getEntry(InstId::vslide1up_vx);
        case 0xf:   return instTable_.getEntry(InstId::vslide1down_vx);
        case 0x10:
	  std::swap(op1, op2); // per spec !
	  if (op2 == 0) return instTable_.getEntry(InstId::vmv_s_x);
	  return instTable_.getEntry(InstId::illegal);
        case 0x20:  return instTable_.getEntry(InstId::vdivu_vx);
        case 0x21:  return instTable_.getEntry(InstId::vdiv_vx);
        case 0x22:  return instTable_.getEntry(InstId::vremu_vx);
        case 0x23:  return instTable_.getEntry(InstId::vrem_vx);
        case 0x24:  return instTable_.getEntry(InstId::vmulhu_vx);
        case 0x25:  return instTable_.getEntry(InstId::vmul_vx);
        case 0x26:  return instTable_.getEntry(InstId::vmulhsu_vx);
        case 0x27:  return instTable_.getEntry(InstId::vmulh_vx);
	case 0x29:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vmadd_vx);
	case 0x2b:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vnmsub_vx);
	case 0x2d:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vmacc_vx);
	case 0x2f:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vnmsac_vx);
        case 0x30:  return instTable_.getEntry(InstId::vwaddu_vx);
        case 0x31:  return instTable_.getEntry(InstId::vwadd_vx);
        case 0x32:  return instTable_.getEntry(InstId::vwsubu_vx);
        case 0x33:  return instTable_.getEntry(InstId::vwsub_vx);
        case 0x34:  return instTable_.getEntry(InstId::vwaddu_wx);
        case 0x35:  return instTable_.getEntry(InstId::vwadd_wx);
        case 0x36:  return instTable_.getEntry(InstId::vwsubu_wx);
        case 0x37:  return instTable_.getEntry(InstId::vwsub_wx);
        case 0x38:  return instTable_.getEntry(InstId::vwmulu_vx);
        case 0x3a:  return instTable_.getEntry(InstId::vwmulsu_vx);
        case 0x3b:  return instTable_.getEntry(InstId::vwmul_vx);
        case 0x3c:
          std::swap(op1, op2);  // Spec is baffling.
          return instTable_.getEntry(InstId::vwmaccu_vx);
        case 0x3d:
          std::swap(op1, op2);  // Spec is baffling.
          return instTable_.getEntry(InstId::vwmacc_vx);
        case 0x3e:
          std::swap(op1, op2);  // Spec is baffling.
          return instTable_.getEntry(InstId::vwmaccus_vx);
        case 0x3f:
          std::swap(op1, op2);  // Spec is baffling.
          return instTable_.getEntry(InstId::vwmaccsu_vx);
        }
      return instTable_.getEntry(InstId::illegal);  
    }

  if (f3 == 5)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2;
      op2 = rform.bits.rs1;

      switch (f6)
	{
	case 0:    return instTable_.getEntry(InstId::vfadd_vf);
	case 2:    return instTable_.getEntry(InstId::vfsub_vf);
	case 4:    return instTable_.getEntry(InstId::vfmin_vf);
	case 6:    return instTable_.getEntry(InstId::vfmax_vf);
	case 8:    return instTable_.getEntry(InstId::vfsgnj_vf);
	case 9:    return instTable_.getEntry(InstId::vfsgnjn_vf);
	case 0xa:  return instTable_.getEntry(InstId::vfsgnjx_vf);
	case 0xe:  return instTable_.getEntry(InstId::vfslide1up_vf);
	case 0xf:  return instTable_.getEntry(InstId::vfslide1down_vf);
	case 0x10:
	  std::swap(op1, op2);
	  if (op2 == 0) return instTable_.getEntry(InstId::vfmv_s_f);
	  return instTable_.getEntry(InstId::illegal);
        case 0x17:
          if (vm == 0) return instTable_.getEntry(InstId::vfmerge_vfm);
          if (vm == 1)
            {
              op1 = (uint32_t(rform.bits.rs1) << 27) >> 27;
              op2 = rform.bits.rs2;
              if (op2 == 0) return instTable_.getEntry(InstId::vfmv_v_f);
            }
          break;
	case 0x18: return instTable_.getEntry(InstId::vmfeq_vf);
	case 0x19: return instTable_.getEntry(InstId::vmfle_vf);
	case 0x1b: return instTable_.getEntry(InstId::vmflt_vf);
	case 0x1c: return instTable_.getEntry(InstId::vmfne_vf);
	case 0x1d: return instTable_.getEntry(InstId::vmfgt_vf);
	case 0x1f: return instTable_.getEntry(InstId::vmfge_vf);
	case 0x20: return instTable_.getEntry(InstId::vfdiv_vf);
	case 0x21: return instTable_.getEntry(InstId::vfrdiv_vf);
	case 0x24: return instTable_.getEntry(InstId::vfmul_vf);
	case 0x27: return instTable_.getEntry(InstId::vfrsub_vf);
	case 0x28:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmadd_vf);
	case 0x29:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmadd_vf);
	case 0x2a:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmsub_vf);
	case 0x2b:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmsub_vf);
	case 0x2c:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmacc_vf);
	case 0x2d:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmacc_vf);
	case 0x2e:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfmsac_vf);
	case 0x2f:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfnmsac_vf);
	case 0x30: return instTable_.getEntry(InstId::vfwadd_vf);
	case 0x32: return instTable_.getEntry(InstId::vfwsub_vf);
	case 0x34: return instTable_.getEntry(InstId::vfwadd_wf);
	case 0x36: return instTable_.getEntry(InstId::vfwsub_wf);
	case 0x38: return instTable_.getEntry(InstId::vfwmul_vf);
        case 0x3b:
          std::swap(op1, op2);  // per spec
          return instTable_.getEntry(InstId::vfwmaccbf16_vf);
	case 0x3c:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwmacc_vf);
	case 0x3d:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwnmacc_vf);
	case 0x3e:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwmsac_vf);
	case 0x3f:
          std::swap(op1, op2);  // per spec
	  return instTable_.getEntry(InstId::vfwnmsac_vf);
	}
      return instTable_.getEntry(InstId::illegal);
    }

  if (f3 == 7)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs1;
      op2 = rform.bits.rs2;

      if ((f6 >> 5) == 0)
        {
          op2 = ((rform.bits.funct7 & 0x3f) << 5 | op2);
          return instTable_.getEntry(InstId::vsetvli);
        }
      if ((f6 >> 4) == 3)
	{
          op2 = ((rform.bits.funct7 & 0x1f) << 5 | op2);
	  return instTable_.getEntry(InstId::vsetivli);
	}
      if (rform.bits.funct7 == 0x40)  return instTable_.getEntry(InstId::vsetvl);
    }

  return instTable_.getEntry(InstId::illegal);  
}


const InstEntry&
Decoder::decodeVecLoad(uint32_t f3, uint32_t imm12, uint32_t& fieldCount) const
{
  unsigned lumop = imm12 & 0x1f;       // Bits 0 to 4 of imm12
  unsigned mop = (imm12 >> 6) & 3;     // Bits 6 & 7 of imm12
  unsigned mew = (imm12 >> 8) & 1;     // Bit 8 of imm12
  unsigned nf = (imm12 >> 9) & 7;      // Bit 9, 10, and 11 of imm12

  if (mop == 0)
    {      // Unit stride
      if (lumop == 0)
        {
	  if (nf == 0)
	    {
	      if (mew == 0)
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vle8_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vle16_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vle32_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vle64_v);
		}
	      else
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vle128_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vle256_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vle512_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vle1024_v);
		}
	    }
	  else
	    {
	      fieldCount = 1 + nf;   // number of fields in segment.
	      if (mew == 0)
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vlsege8_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vlsege16_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vlsege32_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vlsege64_v);
		}
	      else
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vlsege128_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vlsege256_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vlsege512_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vlsege1024_v);
		}
	    }
        }
      else if (lumop == 0x8)
        {   // load whole registers
	  if (nf != 0 and nf != 1 and nf != 3 and nf != 7)
	    return instTable_.getEntry(InstId::illegal);
          fieldCount = nf + 1;

          if (mew == 0)
            {
              if (f3 == 0) return instTable_.getEntry(InstId::vlre8_v);
              if (f3 == 5) return instTable_.getEntry(InstId::vlre16_v);
              if (f3 == 6) return instTable_.getEntry(InstId::vlre32_v);
              if (f3 == 7) return instTable_.getEntry(InstId::vlre64_v);
            }
          else
            {
              if (f3 == 0) return instTable_.getEntry(InstId::vlre128_v);
              if (f3 == 5) return instTable_.getEntry(InstId::vlre256_v);
              if (f3 == 6) return instTable_.getEntry(InstId::vlre512_v);
              if (f3 == 7) return instTable_.getEntry(InstId::vlre1024_v);
            }
        }
      else if (lumop == 0xb)
	{
	  if (nf == 0 and mew == 0 and f3 == 0)
	    return instTable_.getEntry(InstId::vlm_v);
	}
      else if (lumop == 0x10)
        {
	  if (nf == 0)
	    {
	      // fault only on first
	      if (mew == 0)
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vle8ff_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vle16ff_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vle32ff_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vle64ff_v);
		}
	      else
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vle128ff_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vle256ff_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vle512ff_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vle1024ff_v);
		}
	    }
	  else
	    {
	      fieldCount = nf + 1;
	      if (mew == 0)
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vlsege8ff_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vlsege16ff_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vlsege32ff_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vlsege64ff_v);
		}
	      else
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vlsege128ff_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vlsege256ff_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vlsege512ff_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vlsege1024ff_v);
		}
	    }
        }
    }

  if (mop == 1)
    {      // indexed unordered
      if (nf == 0)
	{
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vluxei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vluxei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vluxei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vluxei64_v);
	    }
	}
      else
	{
	  fieldCount = 1 + nf;  // number of fields in sgement
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vluxsegei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vluxsegei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vluxsegei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vluxsegei64_v);
	    }
	}
    }

  if (mop == 2)
    {      // Strided
      if (nf == 0)
	{
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vlse8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vlse16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vlse32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vlse64_v);
	    }
	  else
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vlse128_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vlse256_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vlse512_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vlse1024_v);
	    }
	}
      else
	{
	  fieldCount = 1 + nf;   // number of fields in segment.
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vlssege8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vlssege16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vlssege32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vlssege64_v);
	    }
	  else
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vlssege128_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vlssege256_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vlssege512_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vlssege1024_v);
	    }
	}
    }

  if (mop == 3)
    {      // Indexed
      if (nf == 0)
	{
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vloxei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vloxei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vloxei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vloxei64_v);
	    }
	}
      else
	{
	  fieldCount = 1 + nf;
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vloxsegei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vloxsegei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vloxsegei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vloxsegei64_v);
	    }
	}
    }

  return instTable_.getEntry(InstId::illegal);
}


const InstEntry&
Decoder::decodeVecStore(uint32_t f3, uint32_t imm12, uint32_t& fieldCount) const
{
  unsigned lumop = imm12 & 0x1f;       // Bits 0 to 4 of imm12
  // unsigned vm = (imm12 >> 5) & 1;      // Bit 5 of imm12
  unsigned mop = (imm12 >> 6) & 3;     // Bits 6 & 7 of imm12
  unsigned mew = (imm12 >> 8) & 1;     // Bit 8 of imm12
  unsigned nf = (imm12 >> 9) & 7;      // Bit 9, 10, and 11 of imm12

  if (mop == 0)
    {      // Unit stride
      if (lumop == 0)
        {
	  if (nf == 0)
	    {
	      if (mew == 0)
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vse8_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vse16_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vse32_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vse64_v);
		}
	      else
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vse128_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vse256_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vse512_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vse1024_v);
		}
	    }
	  else
	    {
	      fieldCount = 1 + nf;   // Number of fields in segment
	      if (mew == 0)
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vssege8_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vssege16_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vssege32_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vssege64_v);
		}
	      else
		{
		  if (f3 == 0) return instTable_.getEntry(InstId::vssege128_v);
		  if (f3 == 5) return instTable_.getEntry(InstId::vssege256_v);
		  if (f3 == 6) return instTable_.getEntry(InstId::vssege512_v);
		  if (f3 == 7) return instTable_.getEntry(InstId::vssege1024_v);
		}
	    }
	}
      else if (lumop == 8)
        {   // store whole register
          if (mew == 0 and f3 == 0)
            {
              fieldCount = 1 + nf;
              if (nf == 0) return instTable_.getEntry(InstId::vs1r_v);
              if (nf == 1) return instTable_.getEntry(InstId::vs2r_v);
              if (nf == 3) return instTable_.getEntry(InstId::vs4r_v);
              if (nf == 7) return instTable_.getEntry(InstId::vs8r_v);
	      return instTable_.getEntry(InstId::illegal);
            }
        }
      else if (lumop == 0xb)
	{
	  if (nf == 0 and mew == 0 and f3 == 0)
	    return instTable_.getEntry(InstId::vsm_v);
	}
    }

  if (mop == 1)
    {      // indexed unordered
      if (nf == 0)
	{
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsuxei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsuxei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsuxei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsuxei64_v);
	    }
	}
      else
	{
	  fieldCount = 1 + nf; // Number of fields in sgemtent
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsuxsegei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsuxsegei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsuxsegei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsuxsegei64_v);
	    }
	}
    }

  if (mop == 2)
    {      // Strided
      if (nf == 0)
	{
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsse8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsse16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsse32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsse64_v);
	    }
	  else
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsse128_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsse256_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsse512_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsse1024_v);
	    }
	}
      else
	{
	  fieldCount = 1 + nf;   // Number of fields in segment.
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsssege8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsssege16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsssege32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsssege64_v);
	    }
	  else
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsssege128_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsssege256_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsssege512_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsssege1024_v);
	    }
	}
    }

  if (mop == 3)
    {      // Indexed
      if (nf == 0)
	{
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsoxei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsoxei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsoxei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsoxei64_v);
	    }
	}
      else
	{
	  fieldCount = 1 + nf;  // number of fields in segment
	  if (mew == 0)
	    {
	      if (f3 == 0) return instTable_.getEntry(InstId::vsoxsegei8_v);
	      if (f3 == 5) return instTable_.getEntry(InstId::vsoxsegei16_v);
	      if (f3 == 6) return instTable_.getEntry(InstId::vsoxsegei32_v);
	      if (f3 == 7) return instTable_.getEntry(InstId::vsoxsegei64_v);
	    }
	}
    }

  return instTable_.getEntry(InstId::illegal);
}


const InstEntry&
Decoder::decodeVecCrypto(uint32_t inst, uint32_t& op0, uint32_t& op1, uint32_t& op2) const
{
  RFormInst rform(inst);
  unsigned f3 = rform.bits.funct3, f6 = rform.top6();
  unsigned vm = (inst >> 25) & 1;
  bool masked = vm == 0;
  const InstEntry& illegal = instTable_.getEntry(InstId::illegal);

  if (f3 == 2)
    {
      op0 = rform.bits.rd;
      op1 = rform.bits.rs2; // operand order reversed
      op2 = rform.bits.rs1;

      switch(f6)
	{
        case 0b100000:
          if (not masked)
            return instTable_.getEntry(InstId::vsm3me_vv);
          break;

        case 0b100001:
          if (not masked)
            return instTable_.getEntry(InstId::vsm4k_vi);
          break;

        case 0b100010:
          if (not masked)
            return instTable_.getEntry(InstId::vaeskf1_vi);
          break;

        case 0b101000:
          if (op2 == 0 and not masked)
            return instTable_.getEntry(InstId::vaesdm_vv);
          if (op2 == 1 and not masked)
            return instTable_.getEntry(InstId::vaesdf_vv);
          if (op2 == 2 and not masked)
            return instTable_.getEntry(InstId::vaesem_vv);
          if (op2 == 3 and not masked)
            return instTable_.getEntry(InstId::vaesef_vv);
          if (op2 == 0x10 and not masked)
            return instTable_.getEntry(InstId::vsm4r_vv);
          if (op2 == 0x11 and not masked)
            return instTable_.getEntry(InstId::vgmul_vv);
          break;

        case 0b101001:
          if (op2 == 0 and not masked)
            return instTable_.getEntry(InstId::vaesdm_vs);
          if (op2 == 1 and not masked)
            return instTable_.getEntry(InstId::vaesdf_vs);
          if (op2 == 2 and not masked)
            return instTable_.getEntry(InstId::vaesem_vs);
          if (op2 == 3 and not masked)
            return instTable_.getEntry(InstId::vaesef_vs);
          if (op2 == 7 and not masked)
            return instTable_.getEntry(InstId::vaesz_vs);
          if (op2 == 0x10 and not masked)
            return instTable_.getEntry(InstId::vsm4r_vs);
          break;

        case 0b101010:
          if (not masked)
            return instTable_.getEntry(InstId::vaeskf2_vi);
          break;

        case 0b101011:
          if (not masked)
            return instTable_.getEntry(InstId::vsm3c_vi);
          break;

        case 0b101100:
          if (not masked)
            return instTable_.getEntry(InstId::vghsh_vv);
          break;

        case 0b101101:
          if (not masked)
            return instTable_.getEntry(InstId::vsha2ms_vv);
          break;

        case 0b101110:
          if (not masked)
            return instTable_.getEntry(InstId::vsha2ch_vv);
          break;

        case 0b101111:
          if (not masked)
            return instTable_.getEntry(InstId::vsha2cl_vv);
          break;

        default:
	  return illegal;
	}
    }

  return illegal;
}


const InstEntry&
Decoder::decode16(uint16_t inst, uint32_t& op0, uint32_t& op1, uint32_t& op2) const
{
  uint16_t quadrant = inst & 0x3;
  uint16_t funct3 =  uint16_t(inst >> 13);    // Bits 15 14 and 13

  op0 = 0; op1 = 0; op2 = 0;

  if (quadrant == 0)
    {
      if (funct3 == 0)    // illegal, c.addi4spn
	{
	  if (inst == 0)
	    return instTable_.getEntry(InstId::illegal);
	  CiwFormInst ciwf(inst);
	  unsigned immed = ciwf.immed();
	  if (immed == 0)
	    return instTable_.getEntry(InstId::illegal);
	  op0 = 8 + ciwf.bits.rdp; op1 = RegSp; op2 = immed;
	  return instTable_.getEntry(InstId::c_addi4spn);
	}

      if (funct3 == 1) // c.fld c.lq
	{
	  ClFormInst clf(inst);
	  op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p; op2 = clf.ldImmed();
	  return instTable_.getEntry(InstId::c_fld);
	}

      if (funct3 == 2) // c.lw
	{
	  ClFormInst clf(inst);
	  op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p; op2 = clf.lwImmed();
	  return instTable_.getEntry(InstId::c_lw);
	}

      if (funct3 == 3) // c.flw, c.ld
	{
	  ClFormInst clf(inst);
	  if (isRv64())
	    {
	      op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p; op2 = clf.ldImmed();
	      return instTable_.getEntry(InstId::c_ld);
	    }

	  // c.flw
	  op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p;
	  op2 = clf.lwImmed();
	  return instTable_.getEntry(InstId::c_flw);
	}

      if (funct3 == 4)  // Zcb instructions
	{
	  ClbFormInst cl(inst);
	  unsigned f6 = cl.bits.funct6;
	  if (f6 == 0x20)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed();
	      return instTable_.getEntry(InstId::c_lbu);
	    }
	  if (f6 == 0x21)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed() & 2;
	      if (cl.funct1() == 0)
		return instTable_.getEntry(InstId::c_lhu);
	      return instTable_.getEntry(InstId::c_lh);
	    }
	  if (f6 == 0x22)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed();
	      return instTable_.getEntry(InstId::c_sb);
	    }
	  if (f6 == 0x23)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed() & 2;
	      if (cl.funct1() == 0)
		return instTable_.getEntry(InstId::c_sh);
	    }
	}

      if (funct3 == 5)  // c.fsd
	{
	  CsFormInst cs(inst);  // Double check this
	  op1=8+cs.bits.rs1p; op0=8+cs.bits.rs2p; op2 = cs.sdImmed();
	  return instTable_.getEntry(InstId::c_fsd);
	}

      if (funct3 == 6)  // c.sw
	{
	  CsFormInst cs(inst);
	  op1 = 8+cs.bits.rs1p; op0 = 8+cs.bits.rs2p; op2 = cs.swImmed();
	  return instTable_.getEntry(InstId::c_sw);
	}

      if (funct3 == 7) // c.fsw, c.sd
	{
	  CsFormInst cs(inst);  // Double check this
	  if (not isRv64())
	    {
	      op1=8+cs.bits.rs1p; op0=8+cs.bits.rs2p; op2 = cs.swImmed();
	      return instTable_.getEntry(InstId::c_fsw);
	    }
	  op1=8+cs.bits.rs1p; op0=8+cs.bits.rs2p; op2 = cs.sdImmed();
	  return instTable_.getEntry(InstId::c_sd);
	}

      // funct3 is 1 (c.fld c.lq), or 4 (reserved), or 5 (c.fsd c.sq)
      return instTable_.getEntry(InstId::illegal);
    }

  if (quadrant == 1)
    {
      if (funct3 == 0)  // c.nop, c.addi
	{
	  CiFormInst cif(inst);
	  op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = cif.addiImmed();
	  return instTable_.getEntry(InstId::c_addi);
	}
	  
      if (funct3 == 1)  // c.jal,  in rv64 and rv128 this is c.addiw
	{
          if (isRv64())
            {
              CiFormInst cif(inst);
              op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = cif.addiImmed();
              if (op0 == 0)
                return instTable_.getEntry(InstId::illegal);
              return instTable_.getEntry(InstId::c_addiw);
            }

          CjFormInst cjf(inst);
          op0 = RegRa; op1 = cjf.immed(); op2 = 0;
          return instTable_.getEntry(InstId::c_jal);
	}

      if (funct3 == 2)  // c.li
	{
	  CiFormInst cif(inst);
	  op0 = cif.bits.rd; op1 = RegX0; op2 = cif.addiImmed();
	  return instTable_.getEntry(InstId::c_li);
	}

      if (funct3 == 3)  // c.addi16sp, c.lui
	{
	  CiFormInst cif(inst);
	  int immed16 = cif.addi16spImmed();
	  if (immed16 == 0)
	    { // could be c.mop
	      if (cif.bits.rd <= 15 and (cif.bits.rd & 1))  // Only odd rd less than or equal 15 is valid.
		{
		  op0 = cif.bits.rd ; op1 = cif.addiImmed(); op2 = 0;
		  return instTable_.getEntry(InstId::c_mop);
		} 
	      else
		return instTable_.getEntry(InstId::illegal);
	    }
	  if (cif.bits.rd == RegSp)  // c.addi16sp
	    {
	      op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = immed16;
	      return instTable_.getEntry(InstId::c_addi16sp);
	    }
	  op0 = cif.bits.rd; op1 = cif.luiImmed(); op2 = 0;
	  return instTable_.getEntry(InstId::c_lui);
	}

      // c.srli c.srli64 c.srai c.srai64 c.andi c.sub c.xor c.and
      // c.subw c.addw
      if (funct3 == 4)
	{
	  CaiFormInst caf(inst);  // compressed and immediate form
	  int immed = caf.andiImmed();
	  unsigned rd = 8 + caf.bits.rdp;
	  unsigned f2 = caf.bits.funct2;
	  if (f2 == 0) // srli64, srli
	    {
	      if (caf.bits.ic5 != 0 and not isRv64())
		return instTable_.getEntry(InstId::illegal);
	      op0 = rd; op1 = rd; op2 = caf.shiftImmed();
	      return instTable_.getEntry(InstId::c_srli);
	    }
	  if (f2 == 1)  // srai64, srai
	    {
	      if (caf.bits.ic5 != 0 and not isRv64())
		return instTable_.getEntry(InstId::illegal);
	      op0 = rd; op1 = rd; op2 = caf.shiftImmed();
	      return instTable_.getEntry(InstId::c_srai);
	    }
	  if (f2 == 2)  // c.andi
	    {
	      op0 = rd; op1 = rd; op2 = immed;
	      return instTable_.getEntry(InstId::c_andi);
	    }

	  // f2 == 3: c.sub c.xor c.or c.subw c.addw
	  unsigned rs2p = (immed & 0x7); // Lowest 3 bits of immed
	  unsigned rs2 = 8 + rs2p;
	  unsigned imm34 = (immed >> 3) & 3; // Bits 3 and 4 of immed
	  op0 = rd; op1 = rd; op2 = rs2;
	  if ((immed & 0x20) == 0)  // Bit 5 of immed
	    {
	      if (imm34 == 0) return instTable_.getEntry(InstId::c_sub);
	      if (imm34 == 1) return instTable_.getEntry(InstId::c_xor);
	      if (imm34 == 2) return instTable_.getEntry(InstId::c_or);
	      return instTable_.getEntry(InstId::c_and);
	    }
	  // Bit 5 of immed is 1
	  if (imm34 == 3)  // Zcb instructions
	    {
	      unsigned imm012 = immed & 7;
	      op0 = op1 = rd;
	      if (imm012 == 0)
		{
		  op2 = 0xff;
		  return instTable_.getEntry(InstId::c_zext_b);
		}
	      if (imm012 == 1)
		return instTable_.getEntry(InstId::c_sext_b);
              if (imm012 == 2)
                return instTable_.getEntry(InstId::c_zext_h);
              if (imm012 == 3)
                return instTable_.getEntry(InstId::c_sext_h);
              if (imm012 == 4)
                {
                  op2 = 0;
                  return instTable_.getEntry(InstId::c_zext_w);
                }
              if (imm012 == 5)
                {
                  op2 = -1;
                  return instTable_.getEntry(InstId::c_not);
                }
	    }
	  if (imm34 == 2) return instTable_.getEntry(InstId::c_mul);
	  if (not isRv64())
	    return instTable_.getEntry(InstId::illegal);
	  if (imm34 == 0) return instTable_.getEntry(InstId::c_subw);
	  if (imm34 == 1) return instTable_.getEntry(InstId::c_addw);
	  return instTable_.getEntry(InstId::illegal);
	}

      if (funct3 == 5)  // c.j
	{
	  CjFormInst cjf(inst);
	  op0 = RegX0; op1 = cjf.immed(); op2 = 0;
	  return instTable_.getEntry(InstId::c_j);
	}

      if (funct3 == 6) // c.beqz
	{
	  CbFormInst cbf(inst);
	  op0=8+cbf.bits.rs1p; op1=RegX0; op2=cbf.immed();
	  return instTable_.getEntry(InstId::c_beqz);
	}

      // funct3 == 7: c.bnez
      CbFormInst cbf(inst);
      op0 = 8+cbf.bits.rs1p; op1=RegX0; op2=cbf.immed();
      return instTable_.getEntry(InstId::c_bnez);
    }

  if (quadrant == 2)
    {
      if (funct3 == 0)  // c.slli, c.slli64
	{
	  CiFormInst cif(inst);
	  unsigned immed = unsigned(cif.slliImmed());
	  if (cif.bits.ic5 != 0 and not isRv64())
	    return instTable_.getEntry(InstId::illegal);
	  op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = immed;
	  return instTable_.getEntry(InstId::c_slli);
	}

      if (funct3 == 1)  // c.fldsp c.lqsp
	{
	  CiFormInst cif(inst);
	  op0 = cif.bits.rd; op1 = RegSp, op2 = cif.ldspImmed();
	  return instTable_.getEntry(InstId::c_fldsp);
	}

      if (funct3 == 2) // c.lwsp
	{
	  CiFormInst cif(inst);
	  unsigned rd = cif.bits.rd;
	  if (rd == 0)
	    return instTable_.getEntry(InstId::illegal);
	  op0 = rd; op1 = RegSp; op2 = cif.lwspImmed();
	  return instTable_.getEntry(InstId::c_lwsp);
	}

      if (funct3 == 3)  // c.ldsp  c.flwsp
	{
	  CiFormInst cif(inst);
	  unsigned rd = cif.bits.rd;
	  if (isRv64())
	    {
	      op0 = rd; op1 = RegSp; op2 = cif.ldspImmed();
	      if (rd == 0)
		return instTable_.getEntry(InstId::illegal);
	      return instTable_.getEntry(InstId::c_ldsp);
	    }
	  op0 = rd; op1 = RegSp; op2 = cif.lwspImmed();
	  return instTable_.getEntry(InstId::c_flwsp);
	}

      if (funct3 == 4) // c.jr c.mv c.ebreak c.jalr c.add
	{
	  CiFormInst cif(inst);
	  unsigned immed = cif.addiImmed();
	  unsigned rd = cif.bits.rd;
	  unsigned rs2 = immed & 0x1f;
	  if ((immed & 0x20) == 0)  // c.jr or c.mv
	    {
	      if (rs2 == RegX0)
		{
		  if (rd == RegX0)
		    return instTable_.getEntry(InstId::illegal);
		  op0 = RegX0; op1 = rd; op2 = 0;
		  return instTable_.getEntry(InstId::c_jr);
		}
	      op0 = rd; op1 = RegX0; op2 = rs2;
	      return instTable_.getEntry(InstId::c_mv);
	    }

          // c.ebreak, c.jalr or c.add
          if (rs2 == RegX0)
            {
              if (rd == RegX0)
                return instTable_.getEntry(InstId::c_ebreak);
              op0 = RegRa; op1 = rd; op2 = 0;
              return instTable_.getEntry(InstId::c_jalr);
            }
          op0 = rd; op1 = rd; op2 = rs2;
          return instTable_.getEntry(InstId::c_add);
        }

      if (funct3 == 5)  // c.fsdsp c.sqsp
	{
	  CswspFormInst csw(inst);
	  op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.sdImmed();
	  return instTable_.getEntry(InstId::c_fsdsp);
	}

      if (funct3 == 6) // c.swsp
	{
	  CswspFormInst csw(inst);
	  op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.swImmed();
	  return instTable_.getEntry(InstId::c_swsp);
	}

      if (funct3 == 7)  // c.sdsp  c.fswsp
	{
	  if (isRv64())  // c.sdsp
	    {
	      CswspFormInst csw(inst);
	      op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.sdImmed();
	      return instTable_.getEntry(InstId::c_sdsp);
	    }
	  CswspFormInst csw(inst);
	  op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.swImmed();
	  return instTable_.getEntry(InstId::c_fswsp);
	}

      return instTable_.getEntry(InstId::illegal);
    }

  return instTable_.getEntry(InstId::illegal); // quadrant 3
}


uint32_t
Decoder::expandCompressedInst(uint16_t inst) const
{
  uint16_t quadrant = inst & 0x3;
  uint16_t funct3 =  uint16_t(inst >> 13);    // Bits 15 14 and 13

  uint32_t op0 = 0, op1 = 0, op2 = 0;

  uint32_t expanded = 0;  // Illegal

  if (quadrant == 0)
    {
      if (funct3 == 0)    // illegal, c.addi4spn
	{
	  if (inst == 0)
            return expanded;  // Illegal
	  CiwFormInst ciwf(inst);
	  unsigned immed = ciwf.immed();
	  if (immed == 0)
	    return expanded; // Illegal
	  op0 = 8 + ciwf.bits.rdp; op1 = RegSp; op2 = immed;
          encodeAddi(op0, op1, op2, expanded);
          return expanded;
	}

      if (funct3 == 1) // c.fld c.lq
	{
	  ClFormInst clf(inst);
	  op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p; op2 = clf.ldImmed();
          encodeFld(op0, op1, op2, expanded);
          return expanded;
	}

      if (funct3 == 2) // c.lw
	{
	  ClFormInst clf(inst);
	  op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p; op2 = clf.lwImmed();
          encodeLw(op0, op1, op2, expanded);
	  return expanded;
	}

      if (funct3 == 3) // c.flw, c.ld
	{
	  ClFormInst clf(inst);
	  if (isRv64())
	    {
	      op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p; op2 = clf.ldImmed();
              encodeLd(op0, op1, op2, expanded);
	      return expanded;
	    }

	  // c.flw
	  op0 = 8+clf.bits.rdp; op1 = 8+clf.bits.rs1p;
	  op2 = clf.lwImmed();
	  encodeFlw(op0, op1, op2, expanded);
	  return expanded;
	}

      if (funct3 == 4)  // Zcb instructions
	{
	  ClbFormInst cl(inst);
	  unsigned f6 = cl.bits.funct6;
	  if (f6 == 0x20)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed();
	      encodeLbu(op0, op1, op2, expanded);
	    }
	  else if (f6 == 0x21)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed() & 2;
	      if (cl.funct1() == 0)
		encodeLhu(op0, op1, op2, expanded);
	      else
		encodeLh(op0, op1, op2, expanded);
	    }
	  else if (f6 == 0x22)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed();
	      encodeSb(op1, op0, op2, expanded);
	    }
	  else if (f6 == 0x23)
	    {
	      op1 = 8 + cl.bits.rs1p; op0 = 8 + cl.bits.rdp; op2 = cl.immed() & 2;
	      if (cl.funct1() == 0)
		encodeSh(op1, op0, op2, expanded);
	    }
	  return expanded;
	}

      if (funct3 == 5)  // c.fsd
	{
	  CsFormInst cs(inst);  // Double check this
	  op1=8+cs.bits.rs1p; op0=8+cs.bits.rs2p; op2 = cs.sdImmed();
	  encodeFsd(op1, op0, op2, expanded);
	  return expanded;
	}

      if (funct3 == 6)  // c.sw
	{
	  CsFormInst cs(inst);
	  op1 = 8+cs.bits.rs1p; op0 = 8+cs.bits.rs2p; op2 = cs.swImmed();
          encodeSw(op1, op0, op2, expanded);
          return expanded;
	}

      if (funct3 == 7) // c.fsw, c.sd
	{
	  CsFormInst cs(inst);  // Double check this
	  if (not isRv64())
	    {
	      op1=8+cs.bits.rs1p; op0=8+cs.bits.rs2p; op2 = cs.swImmed();
	      encodeFsw(op0, op1, op2, expanded);
	      return expanded;
	    }
	  op1=8+cs.bits.rs1p; op0=8+cs.bits.rs2p; op2 = cs.sdImmed();
          encodeSd(op1, op0, op2, expanded);
          return expanded;
	}

      // funct3 is 1 (c.fld c.lq), or 4 (reserved), or 5 (c.fsd c.sq)
      return expanded; // Illegal
    }

  if (quadrant == 1)
    {
      if (funct3 == 0)  // c.nop, c.addi
	{
	  CiFormInst cif(inst);
	  op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = cif.addiImmed();
          encodeAddi(op0, op1, op2, expanded);
          return expanded;
	}
	  
      if (funct3 == 1)  // c.jal,  in rv64 and rv128 this is c.addiw
	{
	  if (isRv64())
	    {
	      CiFormInst cif(inst);
	      op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = cif.addiImmed();
	      if (op0 == 0)
		return expanded; // Illegal
              encodeAddiw(op0, op1, op2, expanded);
              return expanded;
	    }

          CjFormInst cjf(inst);
          op0 = RegRa; op1 = cjf.immed(); op2 = 0;
          encodeJal(op0, op1, op2, expanded);
          return expanded;
        }

      if (funct3 == 2)  // c.li
	{
	  CiFormInst cif(inst);
	  op0 = cif.bits.rd; op1 = RegX0; op2 = cif.addiImmed();
	  encodeAddi(op0, op1, op2, expanded);
          return expanded;
	}

      if (funct3 == 3)  // c.addi16sp, c.lui
	{
	  CiFormInst cif(inst);
	  int immed16 = cif.addi16spImmed();
	  if (immed16 == 0)
	    { // could be c.mop
	      if (cif.bits.rd <= 15 and (cif.bits.rd & 1))
		{
		  op0 = cif.bits.rd ; op1 = 0; op2 = 0;
		  encodeLui(op0, op1, op2, expanded);
		  return expanded;
		} 
	      return expanded;
	    }

	  if (cif.bits.rd == RegSp)  // c.addi16sp
	    {
	      op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = immed16;
	      encodeAddi(op0, op1, op2, expanded);
              return expanded;
	    }
	  op0 = cif.bits.rd; op1 = cif.luiImmed(); op2 = 0;
	  encodeLui(op0, op1, op2, expanded);
          return expanded;
	}

      // c.srli c.srli64 c.srai c.srai64 c.andi c.sub c.xor c.and
      // c.subw c.addw
      if (funct3 == 4)
	{
	  CaiFormInst caf(inst);  // compressed and immediate form
	  int immed = caf.andiImmed();
	  unsigned rd = 8 + caf.bits.rdp;
	  unsigned f2 = caf.bits.funct2;
	  if (f2 == 0) // srli64, srli
	    {
	      if (caf.bits.ic5 != 0 and not isRv64())
		return expanded; // Illegal
	      op0 = rd; op1 = rd; op2 = caf.shiftImmed();
	      encodeSrli(op0, op1, op2, isRv64(), expanded);
              return expanded;
	    }
	  if (f2 == 1)  // srai64, srai
	    {
	      if (caf.bits.ic5 != 0 and not isRv64())
		return expanded; // Illegal
	      op0 = rd; op1 = rd; op2 = caf.shiftImmed();
	      encodeSrai(op0, op1, op2, isRv64(), expanded);
              return expanded;
	    }
	  if (f2 == 2)  // c.andi
	    {
	      op0 = rd; op1 = rd; op2 = immed;
	      encodeAndi(op0, op1, op2, expanded);
              return expanded;
	    }

	  // f2 == 3: c.sub c.xor c.or c.subw c.addw
	  unsigned rs2p = (immed & 0x7); // Lowest 3 bits of immed
	  unsigned rs2 = 8 + rs2p;
	  unsigned imm34 = (immed >> 3) & 3; // Bits 3 and 4 of immed
	  op0 = rd; op1 = rd; op2 = rs2;
	  if ((immed & 0x20) == 0)  // Bit 5 of immed
	    {
              if      (imm34 == 0)   encodeSub(op0, op1, op2, expanded);
              else if (imm34 == 1)   encodeXor(op0, op1, op2, expanded);
              else if (imm34 == 2)   encodeOr(op0, op1, op2, expanded);
              else if (imm34 == 3)   encodeAnd(op0, op1, op2, expanded);
              return expanded;
	    }
	  // Bit 5 of immed is 1
	  if (imm34 == 3)  // Zcb instructions
	    {
	      unsigned imm012 = immed & 7;
	      op0 = op1 = rd;
	      if (imm012 == 0)
		encodeAndi(op0, op0, 0xff, expanded);  // c.zext.b
	      if (imm012 == 1)
		encodeSext_b(op0, op1, expanded);
	      else if (imm012 == 2)
		encodeZext_h(op0, op1, isRv64(), expanded);
	      else if (imm012 == 3)
		encodeSext_h(op0, op1, expanded);
	      else if (imm012 == 4)
		encodeAdd_uw(op0, op1, 0, expanded);
	      else if (imm012 == 5)
		encodeXori(op0, op1, -1, expanded);  // c.not
	      return expanded;
	    }
	  if (imm34 == 2)
	    {
	      encodeMul(op0, op1, op2, expanded);
	      return expanded;
	    }
	  if (not isRv64())
	    return expanded; // Illegal
	  if      (imm34 == 0)     encodeSubw(op0, op1, op2, expanded);
	  else if (imm34 == 1)     encodeAddw(op0, op1, op2, expanded);
	  return expanded; // Illegal
	}

      if (funct3 == 5)  // c.j
	{
	  CjFormInst cjf(inst);
	  op0 = RegX0; op1 = cjf.immed(); op2 = 0;
	  encodeJal(op0, op1, op2, expanded);
          return expanded;
	}

      if (funct3 == 6) // c.beqz
	{
	  CbFormInst cbf(inst);
	  op0=8+cbf.bits.rs1p; op1=RegX0; op2=cbf.immed();
	  encodeBeq(op0, op1, op2, expanded);
          return expanded;
	}

      // funct3 == 7: c.bnez
      CbFormInst cbf(inst);
      op0 = 8+cbf.bits.rs1p; op1=RegX0; op2=cbf.immed();
      encodeBne(op0, op1, op2, expanded);
      return expanded;
    }

  if (quadrant == 2)
    {
      if (funct3 == 0)  // c.slli, c.slli64
	{
	  CiFormInst cif(inst);
	  unsigned immed = unsigned(cif.slliImmed());
	  if (cif.bits.ic5 != 0 and not isRv64())
	    return expanded; // Illegal
	  op0 = cif.bits.rd; op1 = cif.bits.rd; op2 = immed;
	  encodeSlli(op0, op1, op2, isRv64(), expanded);
          return expanded;
	}

      if (funct3 == 1)  // c.fldsp c.lqsp
	{
	  CiFormInst cif(inst);
	  op0 = cif.bits.rd; op1 = RegSp, op2 = cif.ldspImmed();
	  encodeFld(op0, op1, op2, expanded);
	  return expanded;
	}

      if (funct3 == 2) // c.lwsp
	{
	  CiFormInst cif(inst);
	  unsigned rd = cif.bits.rd;
	  if (rd == 0)
	    return expanded; // Illegal
	  op0 = rd; op1 = RegSp; op2 = cif.lwspImmed();
	  encodeLw(op0, op1, op2, expanded);
          return expanded;
	}

      if (funct3 == 3)  // c.ldsp  c.flwsp
	{
	  CiFormInst cif(inst);
	  unsigned rd = cif.bits.rd;
	  if (isRv64())
	    {
	      op0 = rd; op1 = RegSp; op2 = cif.ldspImmed();
	      if (rd == 0)
		return expanded; // Illegal (rd == 0 reserved).
	      encodeLd(op0, op1, op2, expanded);
              return expanded;
	    }
	  op0 = rd; op1 = RegSp; op2 = cif.lwspImmed();
	  encodeFlw(op0, op1, op2, expanded);
	  return expanded;
	}

      if (funct3 == 4) // c.jr c.mv c.ebreak c.jalr c.add
	{
	  CiFormInst cif(inst);
	  unsigned immed = cif.addiImmed();
	  unsigned rd = cif.bits.rd;
	  unsigned rs2 = immed & 0x1f;
	  if ((immed & 0x20) == 0)  // c.jr or c.mv
	    {
	      if (rs2 == RegX0)
		{
		  if (rd == RegX0)
		    return expanded; // Illegal
		  op0 = RegX0; op1 = rd; op2 = 0;
		  encodeJalr(op0, op1, op2, expanded);
                  return expanded;
		}
	      op0 = rd; op1 = RegX0; op2 = rs2;
	      encodeAdd(op0, op1, op2, expanded);
              return expanded;
	    }

	  // c.ebreak, c.jalr or c.add
          if (rs2 == RegX0)
            {
              if (rd == RegX0)
                {
                  encodeEbreak(op0, op1, op2, expanded);
                  return expanded;
                }
              op0 = RegRa; op1 = rd; op2 = 0;
              encodeJalr(op0, op1, op2, expanded);
              return expanded;
            }
          op0 = rd; op1 = rd; op2 = rs2;
          encodeAdd(op0, op1, op2, expanded);
          return expanded;
        }

      if (funct3 == 5)  // c.fsdsp c.sqsp
	{
	  CswspFormInst csw(inst);
	  op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.sdImmed();
	  encodeFsd(op1, op0, op2, expanded);
	  return expanded;
	}

      if (funct3 == 6) // c.swsp
	{
	  CswspFormInst csw(inst);
	  op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.swImmed();
	  encodeSw(op1, op0, op2, expanded);
          return expanded;
	}

      if (funct3 == 7)  // c.sdsp  c.fswsp
	{
	  if (isRv64())  // c.sdsp
	    {
	      CswspFormInst csw(inst);
	      op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.sdImmed();
	      encodeSd(op1, op0, op2, expanded);
              return expanded;
	    }
	  CswspFormInst csw(inst);
	  op1 = RegSp; op0 = csw.bits.rs2; op2 = csw.swImmed();
	  encodeFsw(op1, op0, op2, expanded);
	  return expanded;
	}

      return expanded; // Illegal
    }

  return expanded;  // Illegal
}


const InstEntry&
Decoder::decode(uint32_t inst, uint32_t& op0, uint32_t& op1, uint32_t& op2,
		uint32_t& op3) const
{
  if (isCompressedInst(inst))
    {
      return decode16(uint16_t(inst), op0, op1, op2);
    }

  op0 = 0; op1 = 0; op2 = 0; op3 = 0;

  bool quad3 = (inst & 0x3) == 0x3;
  if (quad3)
    {
      unsigned opcode = (inst & 0x7f) >> 2;  // Upper 5 bits of opcode.

      switch (opcode)
        {
        case 0b00000:  //   I-form
          {
            IFormInst iform(inst);
            op0 = iform.fields.rd;
            op1 = iform.fields.rs1;
            op2 = iform.immed();
            switch (iform.fields.funct3)
              {
              case 0:  return instTable_.getEntry(InstId::lb);
              case 1:  return instTable_.getEntry(InstId::lh);
              case 2:  return instTable_.getEntry(InstId::lw);
              case 3:  return instTable_.getEntry(InstId::ld);
              case 4:  return instTable_.getEntry(InstId::lbu);
              case 5:  return instTable_.getEntry(InstId::lhu);
              case 6:  return instTable_.getEntry(InstId::lwu);
              default: return instTable_.getEntry(InstId::illegal);
              }
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b00001:
          {
            IFormInst iform(inst);
            op0 = iform.fields.rd;
            op1 = iform.fields.rs1;
            uint32_t f3 = iform.fields.funct3;
            if (f3 == 1 or f3 == 2 or f3 == 3)
              op2 = iform.immed();  // flh, flw, or fld
            else
              op2 = iform.rs2();  // vector load

            if (f3 == 0)  return decodeVecLoad(f3, iform.uimmed(), op3);
            if (f3 == 1)  return instTable_.getEntry(InstId::flh);
            if (f3 == 2)  return instTable_.getEntry(InstId::flw);
            if (f3 == 3)  return instTable_.getEntry(InstId::fld);
            if (f3 == 5)  return decodeVecLoad(f3, iform.uimmed(), op3);
            if (f3 == 6)  return decodeVecLoad(f3, iform.uimmed(), op3);
            if (f3 == 7)  return decodeVecLoad(f3, iform.uimmed(), op3);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b00010:       //   I-form
          {
            return instTable_.getEntry(InstId::illegal);
          }

        case 0b00111:
          return instTable_.getEntry(InstId::illegal);

        case 0b01001:
          {
            // For store instructions: op0 is the stored register.
            SFormInst sform(inst);
            op0 = sform.bits.rs2;
            op1 = sform.bits.rs1;
            op2 = sform.immed();
            unsigned f3 = sform.bits.funct3;
            if (f3 != 1 and f3 != 2 and f3 != 3)
              {     // vector instructions.
                op0 = sform.vbits.rd;
                op1 = sform.vbits.rs1;
                op2 = sform.rs2();
              }

            if (f3 == 0)  return decodeVecStore(f3, sform.vbits.imm12, op3);
            if (f3 == 1)  return instTable_.getEntry(InstId::fsh);
            if (f3 == 2)  return instTable_.getEntry(InstId::fsw);
            if (f3 == 3)  return instTable_.getEntry(InstId::fsd);
            if (f3 == 5)  return decodeVecStore(f3, sform.vbits.imm12, op3);
            if (f3 == 6)  return decodeVecStore(f3, sform.vbits.imm12, op3);
            if (f3 == 7)  return decodeVecStore(f3, sform.vbits.imm12, op3);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b01010:      //  S-form
        case 0b01111:
          return instTable_.getEntry(InstId::illegal);

        case 0b10000:
          {
            RFormInst rform(inst);
            op0 = rform.bits.rd, op1 = rform.bits.rs1, op2 = rform.bits.rs2;
            unsigned funct7 = rform.bits.funct7;
            op3 = funct7 >> 2;
            if ((funct7 & 3) == 0)
              return instTable_.getEntry(InstId::fmadd_s);
            if ((funct7 & 3) == 1)
              return instTable_.getEntry(InstId::fmadd_d);
            if ((funct7 & 3) == 2)
              return instTable_.getEntry(InstId::fmadd_h);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b10001:
          {
            RFormInst rform(inst);
            op0 = rform.bits.rd, op1 = rform.bits.rs1, op2 = rform.bits.rs2;
            unsigned funct7 = rform.bits.funct7;
            op3 = funct7 >> 2;
            if ((funct7 & 3) == 0)
              return instTable_.getEntry(InstId::fmsub_s);
            if ((funct7 & 3) == 1)
              return instTable_.getEntry(InstId::fmsub_d);
            if ((funct7 & 3) == 2)
              return instTable_.getEntry(InstId::fmsub_h);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b10010:
          {
            RFormInst rform(inst);
            op0 = rform.bits.rd, op1 = rform.bits.rs1, op2 = rform.bits.rs2;
            unsigned funct7 = rform.bits.funct7;
            op3 = funct7 >> 2;
            if ((funct7 & 3) == 0)
              return instTable_.getEntry(InstId::fnmsub_s);
            if ((funct7 & 3) == 1)
              return instTable_.getEntry(InstId::fnmsub_d);
            if ((funct7 & 3) == 2)
              return instTable_.getEntry(InstId::fnmsub_h);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b10011:
          {
            RFormInst rform(inst);
            op0 = rform.bits.rd, op1 = rform.bits.rs1, op2 = rform.bits.rs2;
            unsigned funct7 = rform.bits.funct7;
            op3 = funct7 >> 2;
            if ((funct7 & 3) == 0)
              return instTable_.getEntry(InstId::fnmadd_s);
            if ((funct7 & 3) == 1)
              return instTable_.getEntry(InstId::fnmadd_d);
            if ((funct7 & 3) == 2)
              return instTable_.getEntry(InstId::fnmadd_h);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b10100:
          return decodeFp(inst, op0, op1, op2);

        case 0b10101:
          return decodeVec(inst, op0, op1, op2, op3);

        case 0b10110:  //  R-form custom vector opcode.
	  {
	    RFormInst rform(inst);
            op0 = rform.bits.rd;
            op1 = rform.bits.rs2;  // Operand order reversed
            op2 = rform.bits.rs1;
	    unsigned f3 = rform.bits.funct3, f6 = rform.top6();
	    const InstEntry& illegal = instTable_.getEntry(InstId::illegal);

	    if (f3 == 2)
	      {
		if (f6 == 0b101100) return instTable_.getEntry(InstId::vqdot_vv);
		if (f6 == 0b101000) return instTable_.getEntry(InstId::vqdotu_vv);
		if (f6 == 0b101010) return instTable_.getEntry(InstId::vqdotsu_vv);
	      }
	    else if (f3 == 6)
	      {
		if (f6 == 0b101100) return instTable_.getEntry(InstId::vqdot_vx);
		if (f6 == 0b101000) return instTable_.getEntry(InstId::vqdotu_vx);
		if (f6 == 0b101010) return instTable_.getEntry(InstId::vqdotsu_vx);
		if (f6 == 0b101110) return instTable_.getEntry(InstId::vqdotus_vx);
	      }

	    return illegal;
	  }

        case 0b10111:
        case 0b11010:
          return instTable_.getEntry(InstId::illegal);

        case 0b11101:
	  return decodeVecCrypto(inst, op0, op1, op2);

        case 0b11110:
        case 0b11111:
          return instTable_.getEntry(InstId::illegal);

        case 0b00011: //  I-form
          {
            IFormInst iform(inst);
            unsigned funct3 = iform.fields.funct3;
            unsigned imm = iform.uimmed(), rd = iform.fields.rd;

            if (funct3 == 0)
              {
		if (iform.top4() == 0)
                  {
                    unsigned pred = iform.pred();
                    unsigned succ = iform.succ();
                    unsigned rd = iform.fields.rd;
                    unsigned rs1 = iform.fields.rs1;
                    if (pred == 1 and succ == 0 and rd == 0 and rs1 == 0)
                      return instTable_.getEntry(InstId::pause);
                    return instTable_.getEntry(InstId::fence);
                  }
		if (iform.top4() == 8)
		  return instTable_.getEntry(InstId::fence_tso);
		// Spec says that reserved sfence.fm field values should be treated as zero.
		return instTable_.getEntry(InstId::fence);
              }
            if (funct3 == 1)
              {
		return instTable_.getEntry(InstId::fence_i);
              }
            if (funct3 == 2)
              {
		op0 = iform.fields.rs1;
                if (imm == 0 and rd == 0)
                  return instTable_.getEntry(InstId::cbo_inval);
                if (imm == 1 and rd == 0)
                  return instTable_.getEntry(InstId::cbo_clean);
                if (imm == 2 and rd == 0)
                  return instTable_.getEntry(InstId::cbo_flush);
                if (imm == 4 and rd == 0)
                  return instTable_.getEntry(InstId::cbo_zero);
              }
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b00100:  //  I-form
          {
            IFormInst iform(inst);
            op0 = iform.fields.rd;
            op1 = iform.fields.rs1;
            op2 = iform.immed();
            unsigned funct3 = iform.fields.funct3;

            if (funct3 == 0)  return instTable_.getEntry(InstId::addi);
            if (funct3 == 1)
              {
		if (iform.uimmed() == 0x08f)  // Top 12 bits of opcode
		  return instTable_.getEntry(InstId::zip);
                if (op2 == 0x100)
                  return instTable_.getEntry(InstId::sha256sum0);
                if (op2 == 0x101)
                  return instTable_.getEntry(InstId::sha256sum1);
                if (op2 == 0x102)
                  return instTable_.getEntry(InstId::sha256sig0);
                if (op2 == 0x103)
                  return instTable_.getEntry(InstId::sha256sig1);
                if (op2 == 0x104)
                  return instTable_.getEntry(InstId::sha512sum0);
                if (op2 == 0x105)
                  return instTable_.getEntry(InstId::sha512sum1);
                if (op2 == 0x106)
                  return instTable_.getEntry(InstId::sha512sig0);
                if (op2 == 0x107)
                  return instTable_.getEntry(InstId::sha512sig1);
                if (op2 == 0x108)
                  return instTable_.getEntry(InstId::sm3p0);
                if (op2 == 0x109)
                  return instTable_.getEntry(InstId::sm3p1);

                unsigned top5 = iform.uimmed() >> 7;
                unsigned amt = iform.uimmed() & 0x7f;
                if (top5 == 0)
                  {
                    op2 = amt;
                    return instTable_.getEntry(InstId::slli);
                  }
                else if (top5 == 5)
                  {
                    op2 = amt;
                    return instTable_.getEntry(InstId::bseti);
                  }
                else if (top5 == 9)
                  {
                    op2 = amt;
                    return instTable_.getEntry(InstId::bclri);
                  }
                else if (top5 == 0x0c)
                  {
                    if (amt == 0)    return instTable_.getEntry(InstId::clz);
                    if (amt == 1)    return instTable_.getEntry(InstId::ctz);
                    if (amt == 2)    return instTable_.getEntry(InstId::cpop);
                    if (amt == 0x04) return instTable_.getEntry(InstId::sext_b);
                    if (amt == 0x05) return instTable_.getEntry(InstId::sext_h);
                  }
                else if (top5 == 0x0d)
                  {
                    op2 = amt;
                    if (funct3 == 1)
                      return instTable_.getEntry(InstId::binvi);
                  }
                else if (op2 == 0x300)
                  {
                    return instTable_.getEntry(InstId::aes64im);
                  }
                else if ((op2 >> 4) == 0x31) // op2 == 0x0x31?
                  {
                    op2 = op2 & 0xf;
                    return instTable_.getEntry(InstId::aes64ks1i);
                  }
              }
            else if (funct3 == 2)  return instTable_.getEntry(InstId::slti);
            else if (funct3 == 3)  return instTable_.getEntry(InstId::sltiu);
            else if (funct3 == 4)  return instTable_.getEntry(InstId::xori);
            else if (funct3 == 5)
              {
                unsigned imm   = iform.uimmed();  // 12-bit immediate
                unsigned top5  = imm >> 7;
                unsigned shamt = imm & 0x7f;      // Shift amount (low 7 bits of imm)

                op2 = shamt;
                if (top5 == 0)
                  return instTable_.getEntry(InstId::srli);
                if (top5 == 5)
                  {
                    if (shamt == 0x7)
                      return instTable_.getEntry(InstId::orc_b);
                    return instTable_.getEntry(InstId::illegal);
                  }
                if (top5 == 0x8)  return instTable_.getEntry(InstId::srai);
                if (top5 == 0x9)  return instTable_.getEntry(InstId::bexti);
                if (top5 == 0xc)  return instTable_.getEntry(InstId::rori);
                if (imm == 0x687) return instTable_.getEntry(InstId::brev8);
		if (imm == 0x08f) return instTable_.getEntry(InstId::unzip);

		bool i64 = isRv64(), i32 = not isRv64();
		if (i64 and imm == 0x6b8)  return instTable_.getEntry(InstId::rev8_64);
		if (i32 and imm == 0x698)  return instTable_.getEntry(InstId::rev8_32);
              }
            else if (funct3 == 6)
	      {
		if (op0 != 0) return instTable_.getEntry(InstId::ori);

#if 0
		unsigned rs2 = iform.rs2();
		if (rs2 == 0 or rs2 == 1 or rs2 == 3)
		  {
		    op0 = iform.fields.rs1;
		    op1 = iform.immed() >> 5;
		    if (iform.rs2() == 0) return instTable_.getEntry(InstId::prefetch_i);
		    if (iform.rs2() == 1) return instTable_.getEntry(InstId::prefetch_r);
		    if (iform.rs2() == 3) return instTable_.getEntry(InstId::prefetch_w);
		  }
#endif
		return instTable_.getEntry(InstId::ori);
	      }
            else if (funct3 == 7)  return instTable_.getEntry(InstId::andi);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b00101:  //   U-form
          {
            UFormInst uform(inst);
            op0 = uform.bits.rd;
            op1 = uform.immed();
            return instTable_.getEntry(InstId::auipc);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b00110:  //  I-form
          {
            IFormInst iform(inst);
            op0 = iform.fields.rd;
            op1 = iform.fields.rs1;
            op2 = iform.immed();
            unsigned funct3 = iform.fields.funct3;
            if (funct3 == 0)
              return instTable_.getEntry(InstId::addiw);
            if (funct3 == 1)
              {
                if (iform.top7() == 0)
                  {
                    op2 = iform.fields2.shamt;
                    return instTable_.getEntry(InstId::slliw);
                  }
                if (iform.top6() == 2)
                  {
                    op2 = op2 & 0x7f;
                    return instTable_.getEntry(InstId::slli_uw);
                  }
                if (iform.top5() == 0x0c)
                  {
                    unsigned amt = iform.uimmed() & 0x7f;
                    if (amt == 0)
                      return instTable_.getEntry(InstId::clzw);
                    if (amt == 1)
                      return instTable_.getEntry(InstId::ctzw);
                    if (amt == 2)
                      return instTable_.getEntry(InstId::cpopw);
                  }
              }
            else if (funct3 == 5)
              {
                op2 = iform.fields2.shamt;
                if (iform.top7() == 0)    return instTable_.getEntry(InstId::srliw);
                if (iform.top7() == 0x20) return instTable_.getEntry(InstId::sraiw);
                if (iform.top7() == 0x30) return instTable_.getEntry(InstId::roriw);
              }
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b01000:  //  S-form
          {
            // For the store instructions, the stored register is op0, the
            // base-address register is op1 and the offset is op2.
            SFormInst sform(inst);
            op0 = sform.bits.rs2;
            op1 = sform.bits.rs1;
            op2 = sform.immed();
            uint32_t funct3 = sform.bits.funct3;

            if (funct3 == 0) return instTable_.getEntry(InstId::sb);
            if (funct3 == 1) return instTable_.getEntry(InstId::sh);
            if (funct3 == 2) return instTable_.getEntry(InstId::sw);
            if (funct3 == 3 and isRv64()) return instTable_.getEntry(InstId::sd);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b01011:  //  R-form atomics
          {
            RFormInst rf(inst);
            uint32_t top5 = rf.top5(), f3 = rf.bits.funct3;
            op0 = rf.bits.rd; op1 = rf.bits.rs1; op2 = rf.bits.rs2;

            if (f3 == 2)
              {
                if (top5 == 0)    return instTable_.getEntry(InstId::amoadd_w);
                if (top5 == 1)    return instTable_.getEntry(InstId::amoswap_w);
                if (top5 == 2 and op2==0)    return instTable_.getEntry(InstId::lr_w);
                if (top5 == 3)    return instTable_.getEntry(InstId::sc_w);
                if (top5 == 4)    return instTable_.getEntry(InstId::amoxor_w);
                if (top5 == 5)    return instTable_.getEntry(InstId::amocas_w);
                if (top5 == 8)    return instTable_.getEntry(InstId::amoor_w);
                if (top5 == 0x0c) return instTable_.getEntry(InstId::amoand_w);
                if (top5 == 0x10) return instTable_.getEntry(InstId::amomin_w);
                if (top5 == 0x14) return instTable_.getEntry(InstId::amomax_w);
                if (top5 == 0x18) return instTable_.getEntry(InstId::amominu_w);
                if (top5 == 0x1c) return instTable_.getEntry(InstId::amomaxu_w);
              }
            else if (f3 == 3)
              {
                if (top5 == 0)    return instTable_.getEntry(InstId::amoadd_d);
                if (top5 == 1)    return instTable_.getEntry(InstId::amoswap_d);
                if (top5 == 2 and op2 == 0)    return instTable_.getEntry(InstId::lr_d);
                if (top5 == 3)    return instTable_.getEntry(InstId::sc_d);
                if (top5 == 4)    return instTable_.getEntry(InstId::amoxor_d);
                if (top5 == 5)    return instTable_.getEntry(InstId::amocas_d);
                if (top5 == 8)    return instTable_.getEntry(InstId::amoor_d);
                if (top5 == 0xc)  return instTable_.getEntry(InstId::amoand_d);
                if (top5 == 0x10) return instTable_.getEntry(InstId::amomin_d);
                if (top5 == 0x14) return instTable_.getEntry(InstId::amomax_d);
                if (top5 == 0x18) return instTable_.getEntry(InstId::amominu_d);
                if (top5 == 0x1c) return instTable_.getEntry(InstId::amomaxu_d);
              }
	    else if (f3 == 4)
	      {
		if (top5 == 5)    return instTable_.getEntry(InstId::amocas_q);
	      }
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b01100:  //  R-form
          {
            RFormInst rform(inst);
            op0 = rform.bits.rd;
            op1 = rform.bits.rs1;
            op2 = rform.bits.rs2;
            unsigned funct7 = rform.bits.funct7, funct3 = rform.bits.funct3;
            if (funct7 == 0)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::add);
                if (funct3 == 1) return instTable_.getEntry(InstId::sll);
                if (funct3 == 2) return instTable_.getEntry(InstId::slt);
                if (funct3 == 3) return instTable_.getEntry(InstId::sltu);
                if (funct3 == 4) return instTable_.getEntry(InstId::xor_);
                if (funct3 == 5) return instTable_.getEntry(InstId::srl);
                if (funct3 == 6) return instTable_.getEntry(InstId::or_);
                if (funct3 == 7) return instTable_.getEntry(InstId::and_);
              }
            else if (funct7 == 1)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::mul);
                if (funct3 == 1) return instTable_.getEntry(InstId::mulh);
                if (funct3 == 2) return instTable_.getEntry(InstId::mulhsu);
                if (funct3 == 3) return instTable_.getEntry(InstId::mulhu);
                if (funct3 == 4) return instTable_.getEntry(InstId::div);
                if (funct3 == 5) return instTable_.getEntry(InstId::divu);
                if (funct3 == 6) return instTable_.getEntry(InstId::rem);
                if (funct3 == 7) return instTable_.getEntry(InstId::remu);
              }
            else if (funct7 == 4)
              {
                if (funct3 == 4) return instTable_.getEntry(InstId::pack);
                if (funct3 == 7) return instTable_.getEntry(InstId::packh);
              }
            else if (funct7 == 5)
              {
                if (funct3 == 1) return instTable_.getEntry(InstId::clmul);
                if (funct3 == 2) return instTable_.getEntry(InstId::clmulr);
                if (funct3 == 3) return instTable_.getEntry(InstId::clmulh);
                if (funct3 == 4) return instTable_.getEntry(InstId::min);
                if (funct3 == 6) return instTable_.getEntry(InstId::max);
                if (funct3 == 5) return instTable_.getEntry(InstId::minu);
                if (funct3 == 7) return instTable_.getEntry(InstId::maxu);
              }
            else if (funct7 == 7)
	      {
		if (funct3 == 5) return instTable_.getEntry(InstId::czero_eqz);
		if (funct3 == 7) return instTable_.getEntry(InstId::czero_nez);
	      }
            else if (funct7 == 0x10)
              {
                if (funct3 == 2) return instTable_.getEntry(InstId::sh1add);
                if (funct3 == 4) return instTable_.getEntry(InstId::sh2add);
                if (funct3 == 6) return instTable_.getEntry(InstId::sh3add);
              }
            else if (funct7 == 0x14)
              {
                if (funct3 == 1) return instTable_.getEntry(InstId::bset);
                if (funct3 == 2) return instTable_.getEntry(InstId::xperm_n);
                if (funct3 == 4) return instTable_.getEntry(InstId::xperm_b);
              }
            else if (funct7 == 0x19)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::aes64es);
              }
            else if (funct7 == 0x1b)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::aes64esm);
              }
            else if (funct7 == 0x1d)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::aes64ds);
              }
            else if (funct7 == 0x1f)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::aes64dsm);
              }
            else if (funct7 == 0x20)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::sub);
                if (funct3 == 4) return instTable_.getEntry(InstId::xnor);
                if (funct3 == 5) return instTable_.getEntry(InstId::sra);
                if (funct3 == 6) return instTable_.getEntry(InstId::orn);
                if (funct3 == 7) return instTable_.getEntry(InstId::andn);
              }
            else if (funct7 == 0x24)
              {
                if (funct3 == 1) return instTable_.getEntry(InstId::bclr);
                if (funct3 == 5) return instTable_.getEntry(InstId::bext);
              }
            else if (funct7 == 0x28)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::sha512sum0r);
              }
            else if (funct7 == 0x29)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::sha512sum1r);
              }
            else if (funct7 == 0x2a)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::sha512sig0l);
              }
            else if (funct7 == 0x2b)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::sha512sig1l);
              }
            else if (funct7 == 0x2e)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::sha512sig0h);
              }
            else if (funct7 == 0x2f)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::sha512sig1h);
              }
            else if (funct7 == 0x30)
              {
                if (funct3 == 1) return instTable_.getEntry(InstId::rol);
                if (funct3 == 5) return instTable_.getEntry(InstId::ror);
              }
            else if (funct7 == 0x34)
              {
                if (funct3 == 1) return instTable_.getEntry(InstId::binv);
              }
            else if (funct7 == 0x3f)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::aes64ks2);
              }
            else if ((funct7 & 0x1f) == 0x11)
              {
                if (funct3 == 0)
                  {
                    op3 = inst >> 30;  // Upper 2 bits.
                    return instTable_.getEntry(InstId::aes32esi);
                  }
              }
            else if ((funct7 & 0x1f) == 0x15)
              {
                if (funct3 == 0)
                  {
                    op3 = inst >> 30;  // Upper 2 bits.
                    return instTable_.getEntry(InstId::aes32dsi);
                  }
              }
            else if ((funct7 & 0x1f) == 0x17)
              {
                if (funct3 == 0)
                  {
                    op3 = inst >> 30;  // Upper 2 bits.
                    return instTable_.getEntry(InstId::aes32dsmi);
                  }
              }
            else if ((funct7 & 0x1f) == 0x13)
              {
                if (funct3 == 0)
                  {
                    op3 = inst >> 30;  // Upper 2 bits.
                    return instTable_.getEntry(InstId::aes32esmi);
                  }
              }
            else if ((funct7 & 0x1f) == 0x18)
              {
                if (funct3 == 0)
                  {
                    op3 = inst >> 30;  // Upper 2 bits.
                    return instTable_.getEntry(InstId::sm4ed);
                  }
              }
            else if ((funct7 & 0x1f) == 0x1a)
              {
                if (funct3 == 0)
                  {
                    op3 = inst >> 30;  // Upper 2 bits.
                    return instTable_.getEntry(InstId::sm4ks);
                  }
              }
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b01101:  //  U-form
          {
            UFormInst uform(inst);
            op0 = uform.bits.rd;
            op1 = uform.immed();
            return instTable_.getEntry(InstId::lui);
          }

        case 0b01110: //  R-Form
          {
            const RFormInst rform(inst);
            op0 = rform.bits.rd;
            op1 = rform.bits.rs1;
            op2 = rform.bits.rs2;
            unsigned funct7 = rform.bits.funct7, funct3 = rform.bits.funct3;
            if (funct7 == 0)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::addw);
                if (funct3 == 1) return instTable_.getEntry(InstId::sllw);
                if (funct3 == 5) return instTable_.getEntry(InstId::srlw);
              }
            else if (funct7 == 1)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::mulw);
                if (funct3 == 4) return instTable_.getEntry(InstId::divw);
                if (funct3 == 5) return instTable_.getEntry(InstId::divuw);
                if (funct3 == 6) return instTable_.getEntry(InstId::remw);
                if (funct3 == 7) return instTable_.getEntry(InstId::remuw);
              }
            else if (funct7 == 4)
              {
                if (funct3 == 0) return instTable_.getEntry(InstId::add_uw);
                if (funct3 == 4) return instTable_.getEntry(InstId::packw);
              }
            else if (funct7 == 0x10)
              {
                if (funct3 == 2) return instTable_.getEntry(InstId::sh1add_uw);
                if (funct3 == 4) return instTable_.getEntry(InstId::sh2add_uw);
                if (funct3 == 6) return instTable_.getEntry(InstId::sh3add_uw);
              }
            else if (funct7 == 0x20)
              {
                if (funct3 == 0)  return instTable_.getEntry(InstId::subw);
                if (funct3 == 5)  return instTable_.getEntry(InstId::sraw);
              }
            else if (funct7 == 0x30)
              {
                if (funct3 == 1) return instTable_.getEntry(InstId::rolw);
                if (funct3 == 5) return instTable_.getEntry(InstId::rorw);
              }
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b11000: //  B-form
          {
            BFormInst bform(inst);
            op0 = bform.bits.rs1;
            op1 = bform.bits.rs2;
            op2 = bform.immed();
            uint32_t funct3 = bform.bits.funct3;
            if (funct3 == 0)  return instTable_.getEntry(InstId::beq);
            if (funct3 == 1)  return instTable_.getEntry(InstId::bne);
            if (funct3 == 4)  return instTable_.getEntry(InstId::blt);
            if (funct3 == 5)  return instTable_.getEntry(InstId::bge);
            if (funct3 == 6)  return instTable_.getEntry(InstId::bltu);
            if (funct3 == 7)  return instTable_.getEntry(InstId::bgeu);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b11001:  //  I-form
          {
            IFormInst iform(inst);
            op0 = iform.fields.rd;
            op1 = iform.fields.rs1;
            op2 = iform.immed();
            if (iform.fields.funct3 == 0)
              return instTable_.getEntry(InstId::jalr);
          }
          return instTable_.getEntry(InstId::illegal);

        case 0b11011:  //  J-form
          {
            JFormInst jform(inst);
            op0 = jform.bits.rd;
            op1 = jform.immed();
            return instTable_.getEntry(InstId::jal);
          }

        case 0b11100:  //  I-form
          {
            IFormInst iform(inst);
            op0 = iform.fields.rd;
            op1 = iform.fields.rs1;
            op2 = iform.uimmed(); // csr
            unsigned f3 = iform.fields.funct3;
            switch (f3)
              {
              case 0:
                {
                  uint32_t funct7 = op2 >> 5;
                  if (funct7 == 0) // ecall ebreak
                    {
                      if (op1 != 0 or op0 != 0)
                        return instTable_.getEntry(InstId::illegal);
                      if (op2 == 0)
                        return instTable_.getEntry(InstId::ecall);
                      if (op2 == 1)
                        return instTable_.getEntry(InstId::ebreak);
                      if (op2 == 0x0d and op0 == 0 and op1 == 0)
                        return instTable_.getEntry(InstId::wrs_nto);
                      if (op2 == 0x1d and op0 == 0 and op1 == 0)
                        return instTable_.getEntry(InstId::wrs_sto);
                    }
                  else if (funct7 == 9)
                    {
                      if (op0 != 0)
                        return instTable_.getEntry(InstId::illegal);
                      // sfence.vma
                      op0 = iform.rs1();
                      op1 = iform.rs2();
                      return instTable_.getEntry(InstId::sfence_vma);
                    }
                  else if (funct7 == 0xb and op0 == 0)
                    {
                      op0 = iform.rs1();
                      op1 = iform.rs2();
                      return instTable_.getEntry(InstId::sinval_vma);
                    }
                  else if (funct7 == 0xc)
                    {
                      op2 = iform.rs2();
                      if (op0 == 0 and op1 == 0 and op2 == 0 and f3 == 0)
                        return instTable_.getEntry(InstId::sfence_w_inval);
                      if (op0 == 0 and op1 == 0 and op2 == 1 and f3 == 0)
                        return instTable_.getEntry(InstId::sfence_inval_ir);
                      return instTable_.getEntry(InstId::illegal);
                    }
                  else if (funct7 == 0x11 and op0 == 0)
                    {
                      op0 = iform.rs1();
                      op1 = iform.rs2();
                      return instTable_.getEntry(InstId::hfence_vvma);
                    }
                  else if (funct7 == 0x13 and op0 == 0)
                    {
                      op0 = iform.rs1();
                      op1 = iform.rs2();
                      return instTable_.getEntry(InstId::hinval_vvma);
                    }
                  else if (funct7 == 0x31 and op0 == 0)
                    {
                      op0 = iform.rs1();
                      op1 = iform.rs2();
                      return instTable_.getEntry(InstId::hfence_gvma);
                    }
                  else if (funct7 == 0x33 and op0 == 0)
                    {
                      op0 = iform.rs1();
                      op1 = iform.rs2();
                      return instTable_.getEntry(InstId::hinval_gvma);
                    }
                  else if (op2 == 0x102 and op0 == 0 and op1 == 0)
                    return instTable_.getEntry(InstId::sret);
                  else if (op2 == 0x302 and op0 == 0 and op1 == 0)
                    return instTable_.getEntry(InstId::mret);
                  else if (op2 == 0x702 and op0 == 0 and op1 == 0)
                    return instTable_.getEntry(InstId::mnret);
                  else if (op2 == 0x105 and op0 == 0 and op1 == 0)
                    return instTable_.getEntry(InstId::wfi);
                  else if (op2 == 0x7b2 and op0 == 0 and op1 == 0)
                    return instTable_.getEntry(InstId::dret);
                }
                break;
              case 1:  return instTable_.getEntry(InstId::csrrw);
              case 2:  return instTable_.getEntry(InstId::csrrs);
              case 3:  return instTable_.getEntry(InstId::csrrc);
              case 5:  return instTable_.getEntry(InstId::csrrwi);
              case 6:  return instTable_.getEntry(InstId::csrrsi);
              case 7:  return instTable_.getEntry(InstId::csrrci);
              case 4:
                {
                  unsigned top12 = op2;
                  unsigned top7 = top12 >> 5;
                  RFormInst rform(inst);
                  op2 = rform.bits.rs2;
                  // mop.rr, although are I format seem to have rs2 as well.
                  if (top7 == 0x41) return instTable_.getEntry(InstId::mop_rr);
                  if (top7 == 0x43) return instTable_.getEntry(InstId::mop_rr);
                  if (top7 == 0x45) return instTable_.getEntry(InstId::mop_rr);
                  if (top7 == 0x47) return instTable_.getEntry(InstId::mop_rr);
                  if (top7 == 0x61) return instTable_.getEntry(InstId::mop_rr);
                  if (top7 == 0x63) return instTable_.getEntry(InstId::mop_rr);
                  if (top7 == 0x65) return instTable_.getEntry(InstId::mop_rr);
                  if (top7 == 0x67) return instTable_.getEntry(InstId::mop_rr);


                  op2 = 0; // No offset for these instructions.
                  if (top12 == 0x81C) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x81d) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x81e) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x81f) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x85c) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x85d) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x85e) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x85f) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x89c) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x89d) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x89e) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x89f) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x8dc) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x8dd) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x8de) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0x8df) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc1c) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc1d) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc1e) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc1f) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc5c) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc5d) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc5e) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc5f) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc9c) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc9d) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc9e) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xc9f) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xcdc) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xcdd) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xcde) return instTable_.getEntry(InstId::mop_r);
                  if (top12 == 0xcdf) return instTable_.getEntry(InstId::mop_r);

                  if (top12 == 0x600) return instTable_.getEntry(InstId::hlv_b);
                  if (top12 == 0x601) return instTable_.getEntry(InstId::hlv_bu);
                  if (top12 == 0x640) return instTable_.getEntry(InstId::hlv_h);
                  if (top12 == 0x641) return instTable_.getEntry(InstId::hlv_hu);
                  if (top12 == 0x680) return instTable_.getEntry(InstId::hlv_w);
                  if (top12 == 0x643) return instTable_.getEntry(InstId::hlvx_hu);
                  if (top12 == 0x683) return instTable_.getEntry(InstId::hlvx_wu);
                  if (top12 == 0x681) return instTable_.getEntry(InstId::hlv_wu);
                  if (top12 == 0x6c0) return instTable_.getEntry(InstId::hlv_d);

                  unsigned rd = iform.fields.rd;
                  op0 = top12 & 0x1f;  // rs2 field
                  if (top7 == 0x31 and rd == 0) return instTable_.getEntry(InstId::hsv_b);
                  if (top7 == 0x33 and rd == 0) return instTable_.getEntry(InstId::hsv_h);
                  if (top7 == 0x35 and rd == 0) return instTable_.getEntry(InstId::hsv_w);
                  if (top7 == 0x37 and rd == 0) return instTable_.getEntry(InstId::hsv_d);
                }
                break;
              default: return instTable_.getEntry(InstId::illegal);
              }
            return instTable_.getEntry(InstId::illegal);
          }

        default:
          assert(0 and "Shouldn't be able to get here");
          __builtin_unreachable();
        }
    }
  else
    return instTable_.getEntry(InstId::illegal);
}
