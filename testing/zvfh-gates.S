// Compile with -DXLEN=32 or 64 and arch_test_target/link.ld.
// Run with Zfhmin, Zvfh, Zfa, VLEN=128, and ELEN=64. Also run with
// scalar Zfh enabled, compiling with -DFULL_ZFH for that control.
#include "../arch_test_target/model_test.h"

.option norvc
.option norelax
.section .text.init
.globl _start, rvtest_entry_point
_start:
rvtest_entry_point:
  la t0, trap
  csrw mtvec, t0
  li s0, 0
  li a0, 1

.macro expect reg, value
  addi a0, a0, 1
  li t1, \value
  bne \reg, t1, fail
.endm
.macro illegal instruction:vararg
  addi a0, a0, 1
  li s0, 1
  \instruction
  bnez s0, fail
.endm

  // FLI.H is scalar: FS must be on, but VS may be off.
  li t0, 0x6000
  csrw mstatus, t0
  csrw fcsr, zero
  .word 0xf4180053  // fli.h f0, 1.0
  fmv.x.h t0, f0
  expect t0, 0x3c00
  csrr t0, fcsr
  expect t0, 0
  csrr t0, mstatus
  andi t0, t0, 0x600
  expect t0, 0

  li t0, 0x6600
  csrw mstatus, t0
  csrw vstart, zero
  vsetivli t0, 1, e16, m1, tu, mu
  li t0, 0x3c00
  fmv.h.x f8, t0
  vfmv.v.f v8, f8
  vmv.x.s t0, v8
  expect t0, 0x3c00
  vmfeq.vv v16, v8, v8
  vcpop.m t0, v16
  expect t0, 1

  // SEW=8 integer/half conversions also require Zvfh, not scalar Zfh.
  vsetivli t0, 1, e8, m1, tu, mu
  vmv.v.i v16, 1
  vfwcvt.f.xu.v v8, v16
  vsetivli t0, 1, e16, m1, tu, mu
  vmv.x.s t0, v8
  expect t0, 0x3c00
  vsetivli t0, 1, e8, m1, tu, mu
  vfncvt.x.f.w v16, v8
  vmv.x.s t0, v16
  expect t0, 1
  csrr t0, fcsr
  expect t0, 0

#ifndef FULL_ZFH
  illegal .word 0x5c0474d3  // fsqrt.h f9, f8
#else
  fsqrt.h f9, f8
  fmv.x.h t0, f9
  expect t0, 0x3c00
#endif

  // FLI.H must also ignore an illegal VTYPE.
  li t0, -1
  vsetvl t0, zero, t0
  .word 0xf4180053
  fmv.x.h t0, f0
  expect t0, 0x3c00
  vsetivli t0, 1, e16, m1, tu, mu

  // Vector operations still require VS and FS; FLI.H still requires FS.
  li t0, 0x6000
  csrw mstatus, t0
  illegal vfmv.v.f v8, f8
  li t0, 0x600
  csrw mstatus, t0
  illegal vfmv.v.f v8, f8
  illegal .word 0xf4180053

  RVMODEL_HALT

.balign 4
trap:
  beqz s0, fail
  csrr t0, mcause
  li t1, 2
  bne t0, t1, fail
  li s0, 0
  csrr t0, mepc
  addi t0, t0, 4
  csrw mepc, t0
  mret
fail:
  slli t0, a0, 1
  ori t0, t0, 1
  la t1, tohost
  S_TOHOST t0, 0(t1)
1: j 1b

.section .data
  RVMODEL_DATA_SECTION
