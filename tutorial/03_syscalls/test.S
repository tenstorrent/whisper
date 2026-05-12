.globl _start
_start:

    la a0, msg
    jal print_syscall

pass:
    li a0, 0
    li a7, 93
    ecall
1:  j 1b

print_syscall:
    mv a1, a0
    li a0, 1
    li a2, 13
    li a7, 64
    ecall
    ret

.data
msg: .string "hello, world\n"
