# Semihosting

Semihosting is very similar to syscall emulation in that both call out to the
host OS, but whereas syscall emulation intercepts ecall instructions,
semihosting intercepts ebreak instructions surrounded by special hint ops. So
ecall instructions work exactly like normal as defined by the spec.

The RISC-V semihosting specification can be found
[here](https://github.com/riscv-non-isa/riscv-semihosting).
