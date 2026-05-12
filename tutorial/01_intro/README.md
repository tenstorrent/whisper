# Intro

This is basically the simplest possible test that can be run on Whisper. The
test stores the value 1 to a special symbol named `tohost` which ends the
simulation with exit code 0.

The symbols `tohost` and `fromhost` are defined by something call the
Host-Target Interface (HTIF), which is [documented
here](https://github.com/riscv-software-src/riscv-isa-sim/issues/364#issuecomment-607657754).

Note that depending on your toolchain, you may need to modify the build command
in the Makefile for the test to compile.
