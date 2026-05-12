.globl _start
_start:

    la a0, msg
    jal print_htif

    la a0, msg
    jal print_whisper

pass:
    li t0, 1
    sd t0, tohost, t1
1:  j 1b

print_htif:
    li t0, 0x101000000000000
    la t1, tohost
1:  lb t2, (a0)
    beqz t2, 1f
    or t2, t0, t2
    sd t2, (t1)
    addi a0, a0, 1
    j 1b
1:  ret

print_whisper:
    la t1, __whisper_console_io
1:  lb t0, (a0)
    beqz t0, 1f
    sb t0, (t1)
    addi a0, a0, 1
    j 1b
1:  ret

.data
msg: .string "hello, world\n"
__whisper_console_io: .byte 0

.section .tohost
.globl tohost
.globl fromhost
tohost: .dword 0
fromhost: .dword 0
