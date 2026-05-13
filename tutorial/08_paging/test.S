.globl _start
_start:

    li t0, 0x1f
    csrw pmpcfg0, t0
    li t0, -1
    csrw pmpaddr0, t0

    la t0, 1f
    csrw mtvec, t0

    la t0, 2f
    csrw mepc, t0

    li t0, 0x1800
    csrc mstatus, t0
    li t0, 0x800
    csrs mstatus, t0

    li t0, 0x8000000000080002 // Sv39
    csrw satp, t0

    mret

.align 3
1:  li t0, 1
    sd t0, tohost, t1
1:  j 1b

// 0x80001000
.align 12
2:  ecall

.data

.align 12
table_l0: // 0x80002000
.dword 0
.dword 0
.dword 0x0000000020000c01

.align 12
table_l1: // 0x80003000
.dword 0x0000000020001001

.align 12
table_l2: // 0x80004000
.dword 0                  // .text
.dword 0x000000002000044b // .text (S-mode) dAguXwRV
.dword 0                  // .data

.section .tohost
.globl tohost
.globl fromhost
tohost: .dword 0
fromhost: .dword 0
