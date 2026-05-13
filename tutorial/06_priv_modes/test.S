.globl _start
_start:

    li t0, 0x1f
    csrw pmpcfg0, t0
    li t0, -1
    csrw pmpaddr0, t0

    la t0, 4f
    csrw mtvec, t0

    la t0, 1f
    csrw mepc, t0

    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0

    mret

1: // in S-mode
    la t0, 3f
    csrw stvec, t0

    la t0, 2f
    csrw sepc, t0

    li t0, 0x100
    csrc sstatus, t0

    sret

2: // in U-mode
    ecall

.align 3
3: // in S-mode
    ecall

.align 3
4: // in M-mode
    li t0, 1<<8
    csrs medeleg, t0

    la t0, 5f
    csrw mtvec, t0

    la t0, 2b
    csrw mepc, t0

    li t0, 0x1800
    csrc mstatus, t0

    mret

5: // in M-mode

pass:
    li t0, 1
    sd t0, tohost, t1
1:  j 1b

.section .tohost
.globl tohost
.globl fromhost
tohost: .dword 0
fromhost: .dword 0
