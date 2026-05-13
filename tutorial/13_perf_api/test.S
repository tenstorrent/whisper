.text
.align 2
.globl _start
_start:

    ld a0, x
    ld a1, y
    jal gcd
    addi a0, a0, 0
    j pass

gcd:
1:  beq a0, a1, 3f
    bgt a0, a1, 2f
    sub a1, a1, a0
    j 1b
2:  sub a0, a0, a1
    j 1b
3:  ret

pass:
    li t0, 1
    sd t0, tohost, t1
1:  j 1b

.data
#x:  .dword 420196140727489673
#y:  .dword 679891637638612258
x:  .dword 28
y:  .dword 20

.section .tohost
.globl tohost
.globl fromhost
tohost: .dword 0
fromhost: .dword 0
