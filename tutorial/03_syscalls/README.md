# Syscall Emulation

This tutorial shows an example of using syscall emulation. When the `--newlib`
or `--linux` options are used with Whisper, it will intercept the `ecall`
instructions and invoke the appropriate syscall for the host OS.

If your target program is compiled with the C library statically linked with
it, Whisper can usually automatically infer the appropriate setting to use, but
in this case, we're compiling out program without linking the C library at all
so we need to manually specify the `--newlib` option.
