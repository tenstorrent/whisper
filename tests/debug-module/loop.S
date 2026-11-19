// Idle program for the debug module test: every hart counts in s2 forever.
// The test drives the harts through the debug module and stops whisper itself.

.globl _start
_start:
    li      s2, 0
loop:
    addi    s2, s2, 1
    j       loop

.section .tohost, "aw"
.globl tohost
tohost: .dword 0
