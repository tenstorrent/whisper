# Privilege Modes

This example shows how a program would move between privilege levels. The test
starts in machine-mode and uses the `mret` instruction to switch to supervisor.
It first has to set the `mstatus.MPP` field to 1 so that the destination mode
is supervisor. It also sets the `mtvec` CSR to specify where the hart should go
to when a trap occurs.

In turn, the supervisor-mode code also sets `sstatus.SPP` to 0 before executing
`sret` to return to user-mode. It also sets `stvec` to the supervisor-mode trap
handler.

However, notice that when the `ecall` instruction is executed from user-mode,
it doesn't switch to supervisor-mode, but rather machine-mode. That is because
"ecall from U-mode" was never delegated to supervisor-mode.

This is done by setting the appropriate bit in the `medeleg` CSR. The machine
mode code then does another `mret` to return to user mode so it can reexecute
the `ecall` and this time trap to supervisor-mode.
