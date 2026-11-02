# Tests

Each subdirectory is a test, with a Makefile whose `check` target builds
and runs it, failing if the test fails. Run them all with:

```sh
make -C tests check
```

That runs every test, even after a failure, then lists the ones that
failed. CI runs it too (`tests-job` in `.gitlab-ci.yml`).

Tests that run under whisper use the whisper built in this tree, and build
with a RISC-V gcc (default `riscv64-unknown-linux-gnu-`, any bare-metal
toolchain works as they don't use a libc). Override them with, eg:

```sh
make -C tests check WHISPER=$PWD/build-$(uname -s)/whisper TOOLCHAIN=riscv64-unknown-elf-
```

`WHISPER` must be an absolute path, as each test runs in its own
directory.

To add a test, create a subdirectory with a Makefile providing `check` and
`clean` targets.

| Test | Checks |
|------|--------|
| `imsic-claim-race` | A host program racing IMSIC MSI writes against claims, on the IMSIC model directly. `make tsan` builds it with ThreadSanitizer. |
| `imsic-csr-race` | That `mtopei`/`mireg` CSR accesses are atomic with respect to MSIs from another hart. |
