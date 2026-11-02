// Check that the time CSR is consistent between harts.
//
// The harts pass a token around a ring. The holder reads time, checks it
// isn't earlier than the value the previous holder read, records its own
// value and passes the token on. Each read happens after the previous one
// (it's ordered by the token hand-off), so with a single system timer the
// values can never go backwards.
//
// If each hart had its own time counter, the counters would drift apart as
// the harts run at different speeds. Once any two differ by more than a
// hand-off takes, some hand-off in the ring must go from a higher counter to
// a lower one, and the check fails.
//
// Ends by writing tohost: 1 for pass, (hartid + 1) << 1 | 1 for a failure,
// after printing the failing hand-off on the HTIF console.

#ifndef NHARTS
#define NHARTS 4
#endif

#ifndef ROUNDS
#define ROUNDS 10000
#endif

.globl _start
_start:
    csrr    s0, mhartid
    li      t0, NHARTS
    bgeu    s0, t0, park        // Not part of the ring

    la      s1, turn
    la      s2, last_time
    li      s3, ROUNDS
    li      s4, NHARTS

wait:
    ld      t0, 0(s1)
    bgeu    t0, s3, pass
    remu    t1, t0, s4
    bne     t1, s0, wait

    fence   ir, ir              // Read time after seeing our turn
    rdtime  t2
    ld      t3, 0(s2)
    bltu    t2, t3, fail
    sd      t2, 0(s2)
    addi    t0, t0, 1
    fence   rw, w               // Publish last_time before the token
    sd      t0, 0(s1)
    j       wait

pass:
    bnez    s0, park            // Only hart 0 reports
    la      a0, msg_pass
    call    puts
    li      t0, 1
    sd      t0, tohost, t1
    j       park

fail:
    // t0 = round, t2 = our time, t3 = previous holder's time
    mv      s5, t0
    mv      s6, t2
    mv      s7, t3
    la      a0, msg_fail
    call    puts
    mv      a0, s0
    call    putdec
    la      a0, msg_round
    call    puts
    mv      a0, s5
    call    putdec
    la      a0, msg_time
    call    puts
    mv      a0, s6
    call    putdec
    la      a0, msg_prev
    call    puts
    mv      a0, s7
    call    putdec
    la      a0, msg_nl
    call    puts
    addi    t0, s0, 1
    slli    t0, t0, 1
    ori     t0, t0, 1
    sd      t0, tohost, t1

park:
    j       park

// Print the NUL terminated string at a0.
puts:
    li      t1, 0x0101000000000000      // HTIF device 1 (console), command 1 (putchar)
1:  lbu     t0, 0(a0)
    beqz    t0, 2f
    or      t0, t0, t1
    sd      t0, tohost, t2
    addi    a0, a0, 1
    j       1b
2:  ret

// Print a0 in decimal.
putdec:
    la      t3, numbuf + 31
    sb      zero, 0(t3)
    li      t4, 10
1:  remu    t5, a0, t4
    addi    t5, t5, '0'
    addi    t3, t3, -1
    sb      t5, 0(t3)
    divu    a0, a0, t4
    bnez    a0, 1b
    mv      a0, t3
    j       puts

.section .rodata
msg_pass:  .string "PASS: time never went backwards between harts\n"
msg_fail:  .string "FAIL: time went backwards when hart "
msg_round: .string " took the token in round "
msg_time:  .string ": read time "
msg_prev:  .string ", previous hart read "
msg_nl:    .string "\n"

.data
.align 3
turn:      .dword 0
last_time: .dword 0
numbuf:    .space 32

.section .tohost, "aw"
.align 3
.globl tohost
.globl fromhost
tohost:    .dword 0
fromhost:  .dword 0
