# Multi-hart time consistency

Checks that every hart sees the same time CSR, ie. that time comes from a
single system timer rather than a per-hart counter.

The harts pass a token around a ring. The token holder reads time, checks
it isn't earlier than the value the previous holder read, records its own
value and passes the token on. Each read is ordered after the previous one
by the hand-off, so with a shared timer the values can never go backwards.
If the harts had separate counters they'd drift apart as the harts run at
different speeds, and some hand-off would go from a higher counter to a
lower one.

The test needs no OS. It prints PASS or FAIL on the HTIF console, and
ends by writing tohost, so whisper exits with 0 for a pass.

```sh
make run HARTS=4
```

`HARTS` (default 4) sets the number of harts, `ROUNDS` (default 10000)
the number of token hand-offs. `WHISPER` defaults to the whisper built in
this tree. Any RISC-V gcc can be used as the `TOOLCHAIN` (default
`riscv64-unknown-linux-gnu-`), the test doesn't use a libc.

This catches the bug fixed by "Hart: Share the time counter between harts
again", where each hart got a private copy of the time counter. With that
bug the test fails within the first few hand-offs, eg:

```
FAIL: time went backwards when hart 2 took the token in round 2: read time 4, previous hart read 36
```
