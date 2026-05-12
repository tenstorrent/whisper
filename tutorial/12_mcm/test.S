.globl _start
_start:
    la a0, x
    csrr t0, mhartid
    bnez t0, 1f
    lw t0, (a0)
    lw t0, (a0)
    j pass

1:  li t0, 1
    sw t0, (a0)

pass:
    li t0, 1
    sd t0, tohost, t1
1:  j 1b

.data
x: .word 0

.section .tohost
.globl tohost
.globl fromhost
tohost: .dword 0
fromhost: .dword 0
