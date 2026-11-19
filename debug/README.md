# Debug Module and JTAG

Whisper implements a RISC-V Debug Module (Debug Spec 1.0) reached through a JTAG Debug
Transport Module, served on a TCP socket with the remote_bitbang protocol, as in Spike.
OpenOCD connects to whisper's socket, and gdb connects to OpenOCD.

The debug module is enabled by the `--rbb-port` command line option or by a
`debug_module` block in the configuration file. On start, whisper prints
```
    Listening for remote bitbang connection on port N.
```
on the standard output. Each hart of the system is a hart of the debug module:
`dmcontrol.hartsel` is the hart index (0 to n-1).

## Contents

[Debugging a program with OpenOCD and gdb](#Quickstart)

[How whisper runs with a debug module](#Running)

[Options that cannot be used with the debug module](#Conflicts)

[Configuration](#Configuration)

[Limitations](#Limitations)

[Running the riscv-tests debug suite](#RiscvTests)

<a name="Quickstart"/>

## Debugging a program with OpenOCD and gdb

This needs an OpenOCD with RISC-V support, such as
[riscv-openocd](https://github.com/riscv-collab/riscv-openocd) (see
[Running the riscv-tests debug suite](#RiscvTests) for building it), and a RISC-V gdb.

1. A configuration file for whisper, `dm.json` (every key of the `debug_module` block has
   a default, see [Configuration](#Configuration)):
   ```
   {
     "isa": "rv64imafdc_zicsr_zifencei",
     "debug_module": {}
   }
   ```

2. Start whisper with the program to debug:
   ```
   $ whisper --configfile dm.json --rbb-port 9824 prog.elf
   Listening for remote bitbang connection on port 9824.
   ```
   The program runs until OpenOCD halts it. `--rbb-port 0` lets the operating system
   choose a free port; read it from the line whisper prints.

3. An OpenOCD configuration file, `whisper.cfg`. The IR length of whisper's TAP is 5, and
   `-expected-id` is the `dtm.idcode` configuration value:
   ```
   adapter driver remote_bitbang
   remote_bitbang host localhost
   remote_bitbang port 9824
   jtag newtap riscv cpu -irlen 5 -expected-id 0xdeadbeef
   target create riscv.cpu riscv -chain-position riscv.cpu
   init
   halt
   ```
   With `"authentication": true`, authenticate between `init` and `halt`:
   ```
   set challenge [riscv authdata_read]
   riscv authdata_write [expr {$challenge + 1}]
   ```

4. Start OpenOCD. It halts the hart and waits for gdb on port 3333:
   ```
   $ openocd -f whisper.cfg
   ```

5. Start gdb and connect to OpenOCD:
   ```
   $ riscv64-unknown-elf-gdb prog.elf
   (gdb) target extended-remote :3333
   (gdb) break main
   (gdb) continue
   (gdb) info registers pc
   (gdb) stepi
   (gdb) p/x $dcsr
   (gdb) monitor reset halt
   (gdb) monitor shutdown
   ```
   `monitor` passes a command to OpenOCD. `monitor shutdown` stops OpenOCD. Whisper keeps
   running and accepts a new connection; it ends when the program ends, or when it is
   killed.

<a name="Running"/>

## How whisper runs with a debug module

With a debug module, the harts run in slices of instructions (the `slice` key, default
1000) and the debugger is serviced between slices, so a halt request takes effect within
one slice. A halted hart executes only the instructions of abstract commands and the
program buffer. When every hart is halted, whisper waits for the debugger. The run ends
when the target programs finish, as without a debug module, even if a debugger is
connected.

Abstract commands are carried out directly on the hart. Access Register reads and writes
GPRs and FPRs, and executes a CSR instruction in debug mode for CSRs. Access Memory
executes a load or store in debug mode, with M-mode permissions; virtual addresses
(aamvirtual) are translated as M-mode with MPRV set. The Program Buffer is executed by
the hart in debug mode, from a 128-byte window of memory at the `base` address:

| Offset | Content |
|--------|---------|
| 0x00 - 0x43 | Program buffer (progbufsize words) followed by the implicit ebreak |
| 0x44 | Park address: a halted hart's pc, where ebreak returns in debug mode |
| 0x48 | Exception address: an exception in debug mode jumps here (cmderr = exception) |
| 0x4c | Instruction slot used by abstract commands |
| 0x50 - 0x7f | data0 to data11 (a memory-mapped device: hartinfo.dataaddr) |

The window must be memory that the harts can read, write and execute (no PMA region may
deny that). The debug module sets the debug park loop and debug trap address of every
hart to the park and exception addresses (see `debug_park_loop` and `debug_trap_address`
in the [README](../README.md#Configuring)). The data registers are visible to Program Buffer
loads and stores with x0-relative addresses when the window lies within 2 KiB of address
0 (hartinfo.dataaccess = 1); elsewhere hartinfo reads 0.

In debug mode, a hart takes no traps, so exceptions raised there (for example, while gdb
reads CSRs that do not exist) do not count toward whisper's stop after 16 consecutive
illegal instructions.

A hart reset by the debug module (`dmcontrol.ndmreset` or `dmcontrol.hartreset`) restarts
at its reset vector (`reset_vec` or `--startpc`), not at the ELF entry point.

<a name="Conflicts"/>

## Options that cannot be used with the debug module

The debug module is serviced only by a batch run. Whisper stops with an error if a
debug module is configured together with one of these options:

* `--interactive`
* `--gdb` and `--gdb-tcp-port` (whisper's own gdb stub: use OpenOCD and gdb instead)
* `--server`
* `--snapshotperiod`
* `--roi`
* `--deterministic` (the harts already take turns in a fixed order)

<a name="Configuration"/>

## Configuration

The `debug_module` block of the configuration file configures the debug module. Every
key is optional. The defaults are those of Spike, so that `--rbb-port` alone gives a debug
module that behaves like Spike's. Unknown keys are errors.
```
    "debug_module": {
      "enable": true, "slice": 1000, "base": "0x400",
      "datacount": 2, "progbufsize": 2, "impebreak": true, "abstractauto": true,
      "access_register": { "gpr": true, "fpr": true, "csr": true, "aarsize": [32, 64],
                           "postincrement": false, "postexec": true },
      "quick_access": false,
      "access_memory": { "enable": true, "virtual": true, "sizes": [8, 16, 32, 64],
                         "postincrement": true },
      "abstract_busy_cycles": 0,
      "sba": { "enable": false, "asize": 64, "sizes": [8, 16, 32, 64], "busy_cycles": 20 },
      "hasel": true, "haltgroups": 1, "resumegroups": 0, "external_triggers": 0,
      "authentication": false, "resethaltreq": true, "hartreset": true,
      "relaxedpriv": "fixed", "stickyunavail": false,
      "confstrptr": null, "nextdm": 0, "spike_test_hooks": false,
      "dtm": { "idcode": "0xdeadbeef", "abits": 7, "idle": 0, "dmi_busy_cycles": 0 }
    }
```

* enable: define the debug module. With `"enable": false`, there is a debug module only
  if `--rbb-port` is given (it then uses the other keys of the block). Without
  `--rbb-port`, the port is chosen by the OS.
* slice: number of instructions a hart executes between two services of the debugger.
* base: address of the memory window (a multiple of 4).
* datacount (1 to 12), progbufsize (0 to 16), impebreak: abstractcs.datacount,
  abstractcs.progbufsize and dmstatus.impebreak. impebreak must be true when progbufsize
  is 1. Without impebreak, the program buffer must end with ebreak.
* abstractauto: implement the abstractauto register.
* access_register: register groups the Access Register command supports (gpr: 0x1000 -
  0x101f, fpr: 0x1020 - 0x103f, csr: 0x0000 - 0x0fff), supported aarsize values in bits
  (32, 64), and the aarpostincrement and postexec variants. A register the hart does not
  have (a nonexistent CSR, an FPR without F) gives cmderr 3 (exception); an unsupported
  group or size gives cmderr 2 (not supported).
* quick_access: implement the Quick Access command.
* access_memory: implement the Access Memory command, and its aamvirtual, aamsize (in
  bits) and aampostincrement variants. Its arguments are XLEN wide, so RV64 needs
  datacount 4.
* abstract_busy_cycles: number of JTAG Run-Test/Idle cycles an abstract command keeps
  abstractcs.busy set (to exercise a debugger's busy handling).
* sba: System Bus Access: sbasize (up to 64), supported sbaccess sizes in bits (8 to 64),
  and the number of Run-Test/Idle cycles an access keeps sbbusy set. System bus accesses
  read and write memory and memory-mapped devices of the memory system, without address
  translation or PMA/PMP checks (an address outside memory gives sberror 2).
* hasel: implement the hart array mask (hasel, hawindowsel, hawindow).
* haltgroups, resumegroups: highest halt and resume group number (0: groups are not
  implemented). external_triggers: number of DM external triggers that dmcs2 can assign to
  groups (whisper has no trigger sources).
* authentication: require authentication. The debugger reads a challenge from authdata
  and writes the challenge plus 1 back (the scheme Spike's --dm-auth and the riscv-tests
  OpenOCD configurations use). A wrong value gives a new challenge.
* resethaltreq: implement setresethaltreq/clrresethaltreq (halt on reset).
* hartreset: implement dmcontrol.hartreset.
* relaxedpriv: "fixed" (abstractcs.relaxedpriv reads 0) or "writable" (the bit can be
  written; whisper performs the same permission checks either way).
* stickyunavail: make the per-hart unavailable status sticky until ackunavail. Harts are
  unavailable while held in reset and after their program finished.
* confstrptr: address reported in confstrptr0-3 (dmstatus.confstrptrvalid = 1); null for
  none.
* nextdm: value of nextdm.
* spike_test_hooks: Spike's features for debugger tests: the custom DMI register 0x1f
  holds one availability bit per hart (harts 0-31), and Access Register of an odd regno
  from 0xc000 accesses a custom register: after writing V to register N, register M reads
  V - N + M.
* dtm: JTAG DTM: IDCODE value, dtmcs.abits (7 to 30), dtmcs.idle, and the number of
  Run-Test/Idle cycles a DMI access takes (a scan started earlier reports busy).

<a name="Limitations"/>

## Limitations

* dcsr.stoptime is not modelled: the time CSR advances only while harts run, whatever its
  value.
* A `dret` in the program buffer leaves debug mode, and the command reports cmderr 3
  (exception). The Debug Spec leaves `dret` in the program buffer unspecified.
* Access Memory with aamvirtual translates without updating the A and D bits of page
  table entries, and then checks PMP as M-mode rather than as the privilege mode in
  dcsr.prv.
* System Bus Access reaches memory and the memory-mapped devices of the memory system,
  but not the per-hart CLINT and IMSIC registers, and it does no PMA check.
* The remote_bitbang SRST commands (`s`, `u`) have no effect. TRST (`t`, `u`) resets the
  TAP.

<a name="RiscvTests"/>

## Running the riscv-tests debug suite

The [riscv-tests](https://github.com/riscv-software-src/riscv-tests) `debug` directory
holds gdbserver.py, a suite of about 80 tests that drive a target through OpenOCD and gdb.
[`tests/debug-module/riscv-tests`](../tests/debug-module/riscv-tests) holds whisper targets
for it: `whisper32.py` (one RV32 hart) and `whisper64-2.py` (two RV64 harts), the
counterparts of the suite's spike32 and spike64-2 targets, and `whisperlib.py`, which
starts whisper with a configuration that matches Spike's memory map and debug module
options.

1. Get riscv-tests (tested at commit e06a435) and build riscv-openocd (tested at commit
   d4c5d26):
   ```
   $ git clone https://github.com/riscv-software-src/riscv-tests
   $ git clone https://github.com/riscv-collab/riscv-openocd
   $ cd riscv-openocd && ./bootstrap && ./configure --disable-libjaylink && make
   ```

2. From the whisper directory, copy the whisper targets into riscv-tests:
   ```
   $ cp tests/debug-module/riscv-tests/*.py riscv-tests/debug/targets/RISC-V/
   ```

3. Run the suite from `riscv-tests/debug`, once per target. `--sim_cmd` is the whisper
   binary, `--server_cmd` the OpenOCD binary; the gcc and gdb are RISC-V cross tools.
   A test name after the target runs only that test.
   ```
   $ cd riscv-tests/debug
   $ ./gdbserver.py --isolate --sim_cmd /path/to/whisper \
       --server_cmd /path/to/riscv-openocd/src/openocd \
       --gcc riscv64-unknown-elf-gcc --gdb riscv64-unknown-elf-gdb \
       targets/RISC-V/whisper32.py
   ```
   Each target takes a few minutes. `--logs DIR` keeps the per-test logs of whisper,
   OpenOCD and gdb.

No test should fail. HwbpManual reports "not applicable" on whisper, though it passes on
Spike: it expects mcontrol maskmax to be 0, and whisper's mcontrol triggers report
maskmax 31.
