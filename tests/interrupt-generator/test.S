// Check the simple interrupt generator. SIG_BASE and GEILEN (4) must match sig.json.
//
// 1. The version register reads 0x00010001 and the platform register reads 0. With H
//    the guest register reads 0; without H loading or storing it is an access fault.
// 2. Setting MSIP, MEIP, SEIP or SSIP through the platform register makes the bit read
//    as 1 in mip, and clearing it makes it read as 0. Clearing mip.SSIP by a CSR write
//    leaves SSIP pending while the generator drives it.
// 3. With mie.MEIE and mstatus.MIE set, raising MEIP takes a machine external interrupt.
// 4. Byte, halfword, misaligned and doubleword accesses, unused offsets and stores that
//    set reserved bits are access faults.
// 5. With H, setting and clearing guest interrupt 2 and 4 sets and clears the hgeip bit,
//    and with hgeie set, mip.SGEIP follows it. Guest numbers outside 1..GEILEN and
//    reserved bits are access faults.
//
// Ends by writing tohost: 1 for pass, (check << 1) | 1 for a failure.

#define SIG_BASE     0x10000000
#define SIG_VERSION  0
#define SIG_PLATFORM 4
#define SIG_GUEST    8
#define HGEIE        0x607
#define HGEIP        0xe12
#define SET          (1 << 31)
#define MSIP         (1 << 3)
#define MEIP         (1 << 11)
#define SEIP         (1 << 9)
#define SSIP         (1 << 1)
#define SGEIP        (1 << 12)
#define LOAD_FAULT   5
#define STORE_FAULT  7

#if __riscv_xlen == 64
#define STORE      sd
#else
#define STORE      sw
#endif

// Fail with check number n unless a == b.
#define CHECK_EQ(n, a, b)  li s0, n; bne a, b, fail

// Fail with check number n unless the instruction takes the given exception. The trap
// handler clears s1.
#define FAULT(n, cause, ...)  li s0, n; li s1, cause; __VA_ARGS__; bnez s1, fail

// Fail with check number n unless hgeip reads as the given value.
#define CHECK_HGEIP(n, value)  csrr t0, HGEIP; li t1, value; CHECK_EQ(n, t0, t1)

// Store the given value to the given generator register.
#define SIG_WRITE(reg, value)  li t0, value; sw t0, reg(s2)

// Fail with check number n unless the mip bit reads as the given level (0 or 1).
#define CHECK_MIP(n, bit, level)  li t1, bit; csrr t2, mip; \
                                  and t2, t2, t1; li t1, (level) * (bit); CHECK_EQ(n, t2, t1)

// Raise the given platform interrupts, check they read as pending in mip, lower them and
// check they no longer do: checks n and n + 1.
#define SET_CLEAR(n, bits)  SIG_WRITE(SIG_PLATFORM, SET | (bits)); CHECK_MIP(n, bits, 1); \
                            SIG_WRITE(SIG_PLATFORM, bits); CHECK_MIP(n + 1, bits, 0)

    .text
    .globl _start
_start:
    la      t0, trap
    csrw    mtvec, t0
    // Clear mstatus.MDT: with Smdbltrp it resets to 1, making a trap from M-mode a
    // double trap and holding mstatus.MIE at 0.
#if __riscv_xlen == 64
    li      t0, 1 << 42
    csrc    mstatus, t0
#else
    li      t0, 1 << 10
    csrc    mstatush, t0
#endif
    li      s1, 0                 // Expected exception cause, 0 when none.
    li      s3, 0                 // Count of machine external interrupts taken.
    li      s2, SIG_BASE
    csrr    s4, misa
    andi    s4, s4, 1 << 7        // Non-zero with H.

    // 1. Register reads.
    lw      t0, SIG_VERSION(s2)
    li      t1, 0x00010001
    CHECK_EQ(1, t0, t1)
    lw      t0, SIG_PLATFORM(s2)
    CHECK_EQ(2, t0, zero)
    bnez    s4, 1f
    FAULT(3, LOAD_FAULT, lw t0, SIG_GUEST(s2))
    FAULT(5, STORE_FAULT, sw zero, SIG_GUEST(s2))
    j       2f
1:  lw      t0, SIG_GUEST(s2)
    CHECK_EQ(4, t0, zero)
2:

    // 2. Platform interrupts in mip.
    SET_CLEAR(10, MSIP)
    SET_CLEAR(12, MEIP)
    SET_CLEAR(14, SEIP)
    SET_CLEAR(16, MSIP | SSIP)
    SIG_WRITE(SIG_PLATFORM, SET | SSIP)
    csrc    mip, SSIP
    CHECK_MIP(18, SSIP, 1)
    SIG_WRITE(SIG_PLATFORM, SSIP)
    CHECK_MIP(19, SSIP, 0)

    // 3. A machine external interrupt is taken; the handler lowers MEIP.
    li      t0, MEIP
    csrw    mie, t0
    csrsi   mstatus, 8
    SIG_WRITE(SIG_PLATFORM, SET | MEIP)
    nop
    csrci   mstatus, 8
    li      t0, 1
    CHECK_EQ(30, s3, t0)
    CHECK_MIP(31, MEIP, 0)
    csrw    mie, zero

    // 4. Access faults.
    FAULT(40, STORE_FAULT, sb zero, SIG_VERSION(s2))
    FAULT(41, STORE_FAULT, sh zero, SIG_VERSION(s2))
    FAULT(42, LOAD_FAULT, lb t0, SIG_VERSION(s2))
    FAULT(43, LOAD_FAULT, lhu t0, SIG_PLATFORM(s2))
#if __riscv_xlen == 64
    FAULT(44, LOAD_FAULT, ld t0, SIG_VERSION(s2))
    FAULT(45, STORE_FAULT, sd zero, SIG_VERSION(s2))
#endif
    FAULT(46, LOAD_FAULT, lw t0, SIG_PLATFORM + 2(s2))
    FAULT(47, STORE_FAULT, sw zero, SIG_PLATFORM + 2(s2))
    FAULT(48, LOAD_FAULT, lw t0, 0xc(s2))
    FAULT(49, STORE_FAULT, sw zero, 0x1c(s2))
    li      s5, SET | (1 << 0)    // Reserved bit.
    FAULT(50, STORE_FAULT, sw s5, SIG_PLATFORM(s2))
    li      s5, SET | (1 << 7)    // Reserved bit (MTIP is not driven).
    FAULT(51, STORE_FAULT, sw s5, SIG_PLATFORM(s2))
    CHECK_MIP(52, MEIP | SEIP | MSIP | SSIP, 0)
    beqz    s4, pass

    // 5. Guest external interrupts.
    SIG_WRITE(SIG_GUEST, SET | 2)
    CHECK_HGEIP(60, 1 << 2)
    CHECK_MIP(61, SGEIP, 0)
    li      t0, 1 << 2
    csrw    HGEIE, t0
    CHECK_MIP(62, SGEIP, 1)
    SIG_WRITE(SIG_GUEST, SET | 4)
    CHECK_HGEIP(63, (1 << 2) | (1 << 4))
    SIG_WRITE(SIG_GUEST, 2)
    CHECK_HGEIP(64, 1 << 4)
    CHECK_MIP(65, SGEIP, 0)
    SIG_WRITE(SIG_GUEST, 4)
    CHECK_HGEIP(66, 0)
    li      s5, SET               // Guest 0.
    FAULT(67, STORE_FAULT, sw s5, SIG_GUEST(s2))
    li      s5, SET | 5           // Guest 5 > GEILEN.
    FAULT(68, STORE_FAULT, sw s5, SIG_GUEST(s2))
    li      s5, SET | (1 << 6) | 1  // Reserved bit.
    FAULT(69, STORE_FAULT, sw s5, SIG_GUEST(s2))
    CHECK_HGEIP(70, 0)

pass:
    li      t0, 1
    j       done

    // An interrupt must be a machine external one: count it and lower MEIP. An exception
    // must be the expected one: skip the instruction and clear s1.
    .align 2
trap:
    csrr    t0, mcause
    bgez    t0, 1f
    slli    t0, t0, 1
    li      t1, 11 << 1
    bne     t0, t1, fail
    addi    s3, s3, 1
    li      t0, MEIP
    sw      t0, SIG_PLATFORM(s2)
    mret
1:  beqz    s1, fail
    bne     t0, s1, fail
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
#if __riscv_xlen == 32
    sw      zero, 4(t1)           // Sail's HTIF acts on the write of the upper half.
#endif
1:  j       1b

    .section .tohost, "aw"
    .align 3
    .globl tohost
tohost: .dword 0
