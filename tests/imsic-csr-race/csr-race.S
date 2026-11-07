// Check that a CSR instruction accessing mtopei or mireg is atomic with
// respect to an MSI from another hart (run whisper with 2 harts, which
// runs each hart in its own thread).
//
// Hart 0 sends MSIs to hart 1's machine interrupt file. Hart 1 claims
// them with csrrw mtopei. An id is resent only after it has been claimed,
// so with no race every send is claimed exactly once.
//
// If the read and write of csrrw mtopei are not atomic, an MSI for a
// lower id landing in between is claimed by the write but not returned
// by the read: that id is lost and the returned one is claimed twice.
//
// With IREG_RMW, hart 1 also does a csrrc of eip0 (clearing bit 0, which
// is read-only zero) before each claim. If its read and write are not
// atomic, it writes back a stale eip0 and wipes out an MSI that landed in
// between.
//
// Lost ids are never resent, so once all are lost the sender would spin
// forever: it gives up after SPIN_LIMIT passes without a send. Hart 1
// writes 1 (pass) or 3 (fail) to tohost.

#define MISELECT   0x350
#define MIREG      0x351
#define MTOPEI     0x35c
#define EIDELIVERY 0x70
#define EIP0       0x80
#define EIE0       0xc0

#define IMSIC1     0x28001000      // Hart 1 machine file, see csr-race.json.
#define IDS        64
#define NSENDS     20000
#define SPIN_LIMIT 4000000

        .section .tohost, "aw"
        .globl tohost
tohost: .dword 0

        .data
        .align 3
ready:   .dword 0
done:    .dword 0
result:  .dword 0
outstanding: .zero IDS*8        // Non-zero while an id is sent but unclaimed.

        .text
        .globl _start
_start:
        csrr    a0, mhartid
        bnez    a0, receiver

sender:
        la      t0, ready
1:      ld      t1, 0(t0)
        beqz    t1, 1b
        li      s0, NSENDS       // Remaining sends.
        la      s1, outstanding
        li      s2, IMSIC1
        li      s4, SPIN_LIMIT   // Passes left without a send.
2:      li      s3, 1            // id
3:      slli    t2, s3, 3
        add     t2, t2, s1
        li      t3, 1
        amoswap.d t4, t3, (t2)   // Mark the id outstanding.
        bnez    t4, 4f           // Already outstanding.
        sw      s3, 0(s2)        // MSI.
        li      s4, SPIN_LIMIT
        addi    s0, s0, -1
        beqz    s0, 5f
4:      addi    s4, s4, -1
        beqz    s4, 5f           // Give up: ids have been lost.
        addi    s3, s3, 1
        li      t5, IDS
        blt     s3, t5, 3b
        j       2b
5:      la      t0, done
        li      t1, 1
        sd      t1, 0(t0)
        la      t0, result       // Wait for the receiver's verdict.
6:      ld      t1, 0(t0)
        beqz    t1, 6b
        la      t0, tohost
        sd      t1, 0(t0)
7:      j       7b

receiver:
        li      t0, EIDELIVERY
        csrw    MISELECT, t0
        li      t0, 1
        csrw    MIREG, t0
        li      t0, EIE0         // Enable all ids.
        csrw    MISELECT, t0
        li      t0, -1
        csrw    MIREG, t0
        li      t0, EIP0         // mireg is eip0 from here on.
        csrw    MISELECT, t0
        li      t6, 1
        la      s1, outstanding
        li      s4, 0            // Claims.
        li      s5, 0            // Claims of an id that was not outstanding.
        li      s6, 0            // Idle polls after done.
        la      t0, ready
        li      t1, 1
        sd      t1, 0(t0)
1:
#ifdef IREG_RMW
        csrrc   zero, MIREG, t6  // Read-modify-write of eip0.
#endif
        csrrw   t0, MTOPEI, zero // Claim.
        srli    t0, t0, 16
        beqz    t0, 3f
        addi    s4, s4, 1
        slli    t2, t0, 3
        add     t2, t2, s1
        amoswap.d t3, zero, (t2)
        bnez    t3, 1b
        addi    s5, s5, 1
        j       1b
3:      la      t1, done
        ld      t1, 0(t1)
        beqz    t1, 1b
        addi    s6, s6, 1
        li      t1, 10000
        blt     s6, t1, 1b
        // Pass if every send was claimed exactly once.
        li      t1, NSENDS
        li      a1, 3
        bne     s4, t1, 4f
        bnez    s5, 4f
        li      a1, 1
4:      la      t0, result
        sd      a1, 0(t0)
        la      t0, tohost
        sd      a1, 0(t0)
5:      j       5b
