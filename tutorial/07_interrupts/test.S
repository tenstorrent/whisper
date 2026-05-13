#define mtime    0x0200bff8
#define mtimecmp 0x02004000

.globl _start
_start:

    csrsi mstatus, 8 // MIE=1
    li t0, 1<<7
    csrs mie, t0 // MTIE=1

    la t0, 2f
    csrw mtvec, t0

    la t0, mtimecmp
    li t1, 100
    sd t1, (t0)

1:  j 1b // wait for timer interrupt

// TODO: add examples for IMSIC and APLIC

.align 3
2:
    li t0, 1
    sd t0, tohost, t1
1:  j 1b

.section .tohost
.globl tohost
.globl fromhost
tohost: .dword 0
fromhost: .dword 0
