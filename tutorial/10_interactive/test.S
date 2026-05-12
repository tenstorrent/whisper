.globl _start
_start:
    li t0, 1
    sd t0, tohost, t1
1:  j 1b

.section .tohost
.globl tohost
.globl fromhost
tohost: .dword 0
fromhost: .dword 0
