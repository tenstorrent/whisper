.globl _start
_start:
  li      a0, 4         # Write0 syscall code
  la      a1, msg       # Write0 parameter: address of string
  slli    x0, x0, 0x1f  # Semihosting sequence start
  ebreak                # Semihosting sequence middle
  srai    x0, x0, 0x7   # Semihosting sequence end

  li      a0, 24        # Exit syscall code
  li      a1, 0         # Exit parameter: 0
  slli    x0, x0, 0x1f
  ebreak
  srai    x0, x0, 0x7

  unimp   # Should not reach this

.data
msg: .string "hello, world\n"
