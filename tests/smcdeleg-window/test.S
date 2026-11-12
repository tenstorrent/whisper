// Check accesses to counter state through Smcdeleg: from M-mode, with menvcfg.CDE set,
// sireg* at siselect 0x40 + i reach counter i (sireg: mcycle, minstret or mhpmcounter<i>;
// sireg2: mcyclecfg, minstretcfg or mhpmevent<i>; in RV32, sireg4 and sireg5 reach their
// high halves). Each check:
//
// 1. A read through sireg does not trap, with or without an IMSIC (see the Makefile).
// 2. A write to minstret or mcycle through sireg takes effect after the writing
//    instruction (machine.adoc norm:mcycle_minstret_wr), as a write to the counter's own
//    number does: the next instruction reads the value written.
// 3. A write to mhpmevent3 through sireg2 selects the event mhpmcounter3 counts, and
//    keeps MINH, which is read-only zero through sireg2 (sireg5 in RV32).
// 4. A write through sireg (sireg4 in RV32) to a counter mcounteren does not delegate
//    raises an illegal-instruction exception, and one to minstret leaves minstret as any
//    other trapping instruction does.
//
// Ends by writing tohost: 1 for pass, (check << 1) | 1 for a failure.

#define MENVCFG     0x30a
#define MENVCFGH    0x31a
#define MCOUNTEREN  0x306
#define SISELECT    0x150
#define SIREG       0x151
#define SIREG2      0x152
#define SIREG4      0x155
#define SIREG5      0x156
#define MCYCLE      0xb00
#define MINSTRET    0xb02
#define MHPMCOUNTER3 0xb03
#define MHPMEVENT3  0x323
#define MHPMEVENT3H 0x723

#define INST_COMMITTED 4  // Instructions retired, see counters.json.

#if __riscv_xlen == 64
#define STORE      sd
#define CDE_REG    MENVCFG
#define CDE        (1 << 60)
#define HI_EVENT   MHPMEVENT3     // Holds MINH (bit 62) and SINH (bit 61).
#define HI_IREG    SIREG2
#define MINH_BIT   62
#define SINH_BIT   61
#define WIN_COUNTER SIREG         // Reaches minstret at 0x42.
#else
#define STORE      sw
#define CDE_REG    MENVCFGH
#define CDE        (1 << 28)
#define HI_EVENT   MHPMEVENT3H    // Holds MINH (bit 30) and SINH (bit 29).
#define HI_IREG    SIREG5
#define MINH_BIT   30
#define SINH_BIT   29
#define WIN_COUNTER SIREG4        // Reaches minstreth at 0x42.
#endif

// Fail with check number n unless a == b.
#define CHECK_EQ(n, a, b)  li s0, n; bne a, b, fail

    .text
    .globl _start
_start:
    li      s0, 1                 // Check number, for a failure.
    la      t0, trap
    csrw    mtvec, t0
    li      s1, 0                 // Set while a trap is expected.
    li      t0, CDE
    csrs    CDE_REG, t0
    li      t0, 0xd               // Delegate CY, IR and HPM3.
    csrw    MCOUNTEREN, t0

    // 1. Read minstret through sireg.
    li      t0, 0x42
    csrw    SISELECT, t0
    csrr    a0, SIREG

    // 2. Write minstret, then mcycle, through sireg.
    li      t1, 0x1000
    csrw    SIREG, t1
    csrr    a0, MINSTRET
    CHECK_EQ(2, a0, t1)
    li      t0, 0x40
    csrw    SISELECT, t0
    csrw    SIREG, t1
    csrr    a0, MCYCLE
    CHECK_EQ(2, a0, t1)

    // 3. Count instructions in mhpmcounter3 by writing mhpmevent3 through sireg2. Then set
    // MINH directly, read it as zero through the window, and set SINH through the window.
    li      t0, 0x43
    csrw    SISELECT, t0
    li      t1, INST_COMMITTED
    csrw    SIREG2, t1
    csrw    MHPMCOUNTER3, zero
    nop
    nop
    nop
    csrr    a0, MHPMCOUNTER3
    li      t1, 3
    CHECK_EQ(3, a0, t1)
    li      t1, 1 << MINH_BIT
    csrs    HI_EVENT, t1
    csrr    a0, HI_IREG
    srli    a0, a0, SINH_BIT      // SINH, MINH and OF clear.
    CHECK_EQ(3, a0, zero)
    li      t1, 1 << SINH_BIT
    csrs    HI_IREG, t1
    csrr    a0, HI_EVENT
    srli    a0, a0, SINH_BIT
    li      t1, 3                 // SINH and MINH set, OF clear.
    CHECK_EQ(3, a0, t1)

    // 4. Write through sireg (sireg4) with only CY and HPM3 delegated: at 0x44, a counter
    // that is not minstret, then at 0x42, minstret. Both trap, and minstret advances the
    // same between the reads around each.
    li      t0, 0x9
    csrw    MCOUNTEREN, t0
    li      t0, 0x44
    csrw    SISELECT, t0
    li      s1, 1
    csrr    a1, MINSTRET
    csrw    WIN_COUNTER, zero
    csrr    a2, MINSTRET
    CHECK_EQ(4, s1, zero)
    sub     s2, a2, a1
    li      t0, 0x42
    csrw    SISELECT, t0
    li      s1, 1
    csrr    a1, MINSTRET
    csrw    WIN_COUNTER, zero
    csrr    a2, MINSTRET
    CHECK_EQ(4, s1, zero)
    sub     a0, a2, a1
    CHECK_EQ(4, a0, s2)

    li      t0, 1
    j       done

    // An expected illegal-instruction exception skips the instruction and clears s1.
    // Any other trap fails.
    .align 2
trap:
    beqz    s1, fail
    csrr    t0, mcause
    li      t1, 2
    bne     t0, t1, fail
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
    li      s1, 0
    mret

fail:
    slli    t0, s0, 1
    ori     t0, t0, 1
done:
    la      t1, tohost
    STORE   t0, 0(t1)
1:  j       1b

    .section .tohost, "aw"
    .align 3
    .globl tohost
tohost: .dword 0
