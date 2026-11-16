#!/usr/bin/env python3
# Copyright 2026 David_Harris@hmc.edu.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test whisper's debug module through its JTAG DTM and remote bitbang socket.

Usage: dmtest.py WHISPER ELF64 ELF32

Three cases start whisper with --rbb-port 0 and two harts: the default
debug_module options (RV64), most optional features turned off (RV32), and most
turned on (RV64). Each connects as a remote bitbang client (the protocol OpenOCD
uses) and checks the Debug Module registers and abstract commands.
"""

import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import time

# DMI registers
DATA0, DMCONTROL, DMSTATUS, HARTINFO, HALTSUM1 = 0x04, 0x10, 0x11, 0x12, 0x13
HAWINDOWSEL, HAWINDOW, ABSTRACTCS, COMMAND, ABSTRACTAUTO = 0x14, 0x15, 0x16, 0x17, 0x18
CONFSTRPTR0, NEXTDM, PROGBUF0, AUTHDATA, DMCS2 = 0x19, 0x1D, 0x20, 0x30, 0x32
SBCS, SBADDRESS0, SBADDRESS1, SBDATA0, SBDATA1, HALTSUM0 = 0x38, 0x39, 0x3A, 0x3C, 0x3D, 0x40

# dmcontrol bits
HALTREQ, RESUMEREQ, HARTRESET, ACKHAVERESET = 1 << 31, 1 << 30, 1 << 29, 1 << 28
HASEL, SETRESETHALTREQ, CLRRESETHALTREQ, NDMRESET, DMACTIVE = 1 << 26, 1 << 3, 1 << 2, 1 << 1, 1

# dmstatus bits
NDMRESETPENDING, IMPEBREAK, ALLHAVERESET, ANYHAVERESET = 1 << 24, 1 << 22, 1 << 19, 1 << 18
ALLRESUMEACK, ANYNONEXISTENT, ALLUNAVAIL, ANYUNAVAIL = 1 << 17, 1 << 14, 1 << 13, 1 << 12
ALLRUNNING, ANYRUNNING, ALLHALTED, ANYHALTED = 1 << 11, 1 << 10, 1 << 9, 1 << 8
AUTHENTICATED, HASRESETHALTREQ, CONFSTRPTRVALID = 1 << 7, 1 << 5, 1 << 4

DCSR, DPC, MISA, DSCRATCH0 = 0x7B0, 0x7B1, 0x301, 0x7B2
GPR = 0x1000
CMDERR_NONE, CMDERR_BUSY, CMDERR_NOTSUP, CMDERR_EXCEPTION, CMDERR_HALTRESUME = 0, 1, 2, 3, 4

EBREAK = 0x00100073
NOP = 0x00000013

failures = []


def check(cond, msg):
    if not cond:
        failures.append(msg)
        print("  FAIL: " + msg, flush=True)


def expect(actual, wanted, msg):
    check(actual == wanted, f"{msg}: got {actual:#x}, expected {wanted:#x}")


def addi(rd, rs1, imm):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (rd << 7) | 0x13


def lw(rd, rs1, imm):
    return ((imm & 0xFFF) << 20) | (rs1 << 15) | (2 << 12) | (rd << 7) | 0x03


def sw(rs2, rs1, imm):
    return (((imm >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (2 << 12) | ((imm & 0x1F) << 7) | 0x23


def xlen_size(xlen):
    """The aarsize encoding of an XLEN-wide access."""
    return 3 if xlen == 64 else 2


def access_register(regno, size=None, write=False, transfer=True, postexec=False, xlen=64):
    size = size or xlen_size(xlen)
    return (size << 20) | (postexec << 18) | (transfer << 17) | (write << 16) | regno


class Whisper:
    """A whisper process with a debug module, and a JTAG client connected to it."""

    def __init__(self, whisper, elf, xlen, dm, harts=1, extra=None):
        self.xlen = xlen
        self.tmp = tempfile.TemporaryDirectory()
        config = {
            "xlen": xlen,
            "isa": f"rv{xlen}imafdcsu_zicsr_zifencei",
            "reset_vec": "0x80000000",
            "csr": {
                "dcsr": {"mask": "0xbe17", "poke_mask": "0xbfff"},
                "dscratch0": {"exists": "true"},
                "dscratch1": {"exists": "true"},
            },
            "debug_module": dm,
        }
        config.update(extra or {})
        path = os.path.join(self.tmp.name, "whisper.json")
        with open(path, "w") as f:
            json.dump(config, f)
        self.log = open(os.path.join(self.tmp.name, "whisper.log"), "w+")
        cmd = [whisper, "--configfile", path, "--harts", str(harts), "--rbb-port", "0", elf]
        self.proc = subprocess.Popen(cmd, shell=False, stdout=self.log, stderr=subprocess.STDOUT)
        port = None
        for _ in range(200):
            self.log.seek(0)
            m = re.search(r"Listening for remote bitbang connection on port (\d+)\.", self.log.read())
            if m:
                port = int(m.group(1))
                break
            if self.proc.poll() is not None:
                break
            time.sleep(0.05)
        if port is None:
            self.log.seek(0)
            raise RuntimeError("whisper did not open its remote bitbang port:\n" + self.log.read())
        self.sock = socket.create_connection(("127.0.0.1", port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.ir = None
        self.reset_tap()
        dtmcs = self.dtmcs()
        self.abits = (dtmcs >> 4) & 0x3F
        self.idle = (dtmcs >> 12) & 7

    def close(self):
        try:
            self.sock.sendall(b"Q")
            self.sock.close()
        except OSError:
            pass
        self.proc.kill()
        self.proc.wait()
        self.log.close()
        self.tmp.cleanup()

    # JTAG
    def clock(self, bits):
        out, n = bytearray(), 0
        for tms, tdi, sample in bits:
            out += b"%d" % ((tms << 1) | tdi)
            if sample:
                out += b"R"
                n += 1
            out += b"%d" % (4 | (tms << 1) | tdi)
        self.sock.sendall(out)
        resp = bytearray()
        while len(resp) < n:
            chunk = self.sock.recv(n - len(resp))
            if not chunk:
                raise RuntimeError("connection closed")
            resp += chunk
        return [b - ord("0") for b in resp]

    def reset_tap(self):
        self.clock([(1, 0, False)] * 6 + [(0, 0, False)])
        self.ir = 1

    def run_idle(self, cycles):
        if cycles:
            self.clock([(0, 0, False)] * cycles)

    def scan(self, head, value, length):
        bits = [(tms, 0, False) for tms in head]
        for i in range(length):
            bits.append((int(i == length - 1), (value >> i) & 1, True))
        bits += [(1, 0, False), (0, 0, False)]
        tdo = self.clock(bits)
        return sum(b << i for i, b in enumerate(tdo))

    def scan_ir(self, value):
        self.ir = value
        return self.scan([1, 1, 0, 0], value, 5)

    def scan_dr(self, value, length):
        return self.scan([1, 0, 0], value, length)

    def select(self, ir):
        if self.ir != ir:
            self.scan_ir(ir)

    def idcode(self):
        self.select(0x01)
        return self.scan_dr(0, 32)

    def dtmcs(self, value=0):
        self.select(0x10)
        return self.scan_dr(value, 32)

    def dmi_scan(self, op, addr, data):
        self.select(0x11)
        r = self.scan_dr((addr << 34) | ((data & 0xFFFFFFFF) << 2) | op, self.abits + 34)
        self.run_idle(self.idle)
        return r & 3, (r >> 2) & 0xFFFFFFFF

    def dmi(self, op, addr, data=0):
        for _ in range(100):
            self.dmi_scan(op, addr, data)
            status, result = self.dmi_scan(0, 0, 0)
            if status == 0:
                return result
            self.dtmcs(1 << 16)  # dmireset
            self.idle += 1
        raise RuntimeError(f"DMI access to {addr:#x} failed")

    def read(self, addr):
        return self.dmi(1, addr)

    def write(self, addr, data):
        self.dmi(2, addr, data)

    # Debug module helpers
    def activate(self):
        self.write(DMCONTROL, 0)
        self.write(DMCONTROL, DMACTIVE)

    def halt(self, sel=0):
        self.write(DMCONTROL, DMACTIVE | HALTREQ | (sel << 16))
        self.write(DMCONTROL, DMACTIVE | (sel << 16))

    def resume(self, sel=0):
        self.write(DMCONTROL, DMACTIVE | RESUMEREQ | (sel << 16))

    def cmderr(self):
        return (self.read(ABSTRACTCS) >> 8) & 7

    def clear_cmderr(self):
        self.write(ABSTRACTCS, 7 << 8)

    def command(self, cmd):
        self.write(COMMAND, cmd)
        for _ in range(100):
            if not (self.read(ABSTRACTCS) >> 12) & 1:
                break
            self.run_idle(10)
        err = self.cmderr()
        if err:
            self.clear_cmderr()
        return err

    def read_reg(self, regno, size=None):
        err = self.command(access_register(regno, size, xlen=self.xlen))
        value = self.read(DATA0)
        if (size or xlen_size(self.xlen)) == 3:
            value |= self.read(DATA0 + 1) << 32
        return err, value

    def write_reg(self, regno, value, size=None):
        size = size or xlen_size(self.xlen)
        self.write(DATA0, value & 0xFFFFFFFF)
        if size == 3:
            self.write(DATA0 + 1, value >> 32)
        return self.command(access_register(regno, size, write=True, xlen=self.xlen))


def defaults(w):
    """Spike-like defaults on a 2-hart RV64 system."""
    expect(w.idcode(), 0xDEADBEEF, "idcode")
    dtmcs = w.dtmcs()
    expect(dtmcs & 0xF, 1, "dtmcs.version")
    expect(w.abits, 7, "dtmcs.abits")
    w.select(0x1F)
    expect(w.scan_dr(1, 1), 0, "BYPASS captures 0")
    w.select(0x0A)
    expect(w.scan_dr(1, 1), 0, "unimplemented IR selects BYPASS")

    expect(w.read(DMSTATUS), 0, "dmstatus reads 0 while dmactive is 0")
    w.activate()
    expect(w.read(DMCONTROL) & 1, 1, "dmactive")
    status = w.read(DMSTATUS)
    expect(status & 0xF, 3, "dmstatus.version")
    check(status & AUTHENTICATED, "authenticated")
    check(status & IMPEBREAK, "impebreak")
    check(status & HASRESETHALTREQ, "hasresethaltreq")
    check(status & ALLRUNNING, "allrunning at start")
    check(status & ALLHAVERESET, "allhavereset at start")
    expect(w.read(ABSTRACTCS) & 0x1F00000F, (2 << 24) | 2, "progbufsize and datacount")

    # hartsel: 2 harts need 1 bit.
    w.write(DMCONTROL, DMACTIVE | (0x3FF << 16) | (0x3FF << 6))
    expect((w.read(DMCONTROL) >> 16) & 0x3FF, 1, "hartsel width")
    expect(w.read(DMSTATUS) & ANYNONEXISTENT, 0, "hart 1 exists")
    w.write(DMCONTROL, DMACTIVE)

    # Halt hart 0.
    w.write(DMCONTROL, DMACTIVE | ACKHAVERESET)
    check(not w.read(DMSTATUS) & ANYHAVERESET, "ackhavereset")
    w.write(DMCONTROL, DMACTIVE | HALTREQ)
    check(w.read(DMSTATUS) & ALLHALTED, "allhalted after haltreq")
    expect(w.read(DMCONTROL) >> 31, 0, "haltreq reads 0")
    w.write(DMCONTROL, DMACTIVE)
    expect(w.read(HALTSUM0), 1, "haltsum0")
    err, dcsr = w.read_reg(DCSR)
    expect(err, 0, "read dcsr")
    expect((dcsr >> 6) & 7, 3, "dcsr.cause = haltreq")
    expect(dcsr >> 28, 4, "dcsr.debugver")
    err, dpc = w.read_reg(DPC)
    check(0x80000000 <= dpc < 0x80000010, f"dpc {dpc:#x} in the loop")

    # GPRs: s2 (x18) counts.
    err, s2 = w.read_reg(GPR + 18)
    expect(err, 0, "read s2")
    check(s2 > 0, "s2 counted")
    expect(w.write_reg(GPR + 18, 0x1234), 0, "write s2")
    expect(w.read_reg(GPR + 18)[1], 0x1234, "s2 written")
    expect(w.write_reg(GPR, 5), 0, "write x0")
    expect(w.read_reg(GPR)[1], 0, "x0 stays 0")
    expect(w.read_reg(GPR + 18, 2)[1], 0x1234, "32-bit read of a 64-bit GPR")
    expect(w.command(access_register(GPR + 18, 4)), CMDERR_NOTSUP, "aarsize 128 not supported")
    expect(w.command(access_register(GPR + 18, 4, transfer=False)), 0, "aarsize ignored without transfer")
    expect(w.command(access_register(GPR + 18) | (1 << 19)), CMDERR_NOTSUP, "aarpostincrement off")
    expect(w.command(access_register(GPR + 18) | (1 << 23)), CMDERR_NOTSUP, "bit 23 must be 0")
    expect(w.command(access_register(0x1040)), CMDERR_NOTSUP, "regno 0x1040")
    expect(w.command(access_register(0xC001)), CMDERR_NOTSUP, "custom register without test hooks")

    # FPRs and CSRs.
    expect(w.write_reg(0x1020 + 3, 0x400921FB54442D18), 0, "write f3")
    expect(w.read_reg(0x1020 + 3)[1], 0x400921FB54442D18, "read f3")
    err, misa = w.read_reg(MISA)
    expect(err, 0, "read misa")
    expect(misa >> 62, 2, "misa.mxl")
    expect(w.command(access_register(0x8F0)), CMDERR_EXCEPTION, "nonexistent CSR")
    expect(w.write_reg(DSCRATCH0, 0x55AA), 0, "write dscratch0")
    expect(w.read_reg(DSCRATCH0)[1], 0x55AA, "read dscratch0")
    expect(w.read_reg(0x003)[0], 0, "fcsr readable")

    # Abstract commands need a halted hart.
    w.write(DMCONTROL, DMACTIVE | (1 << 16))
    expect(w.command(access_register(GPR + 18)), CMDERR_HALTRESUME, "hart 1 running")
    w.write(DMCONTROL, DMACTIVE)

    # cmderr blocks later commands until cleared.
    w.write(COMMAND, access_register(0x8F0))
    w.write(COMMAND, access_register(GPR + 18))
    expect(w.cmderr(), CMDERR_EXCEPTION, "cmderr sticky")
    w.clear_cmderr()

    # Access Memory needs 4 data registers on RV64.
    expect(w.command((2 << 24) | (2 << 20)), CMDERR_NOTSUP, "access memory with datacount 2")

    # Program buffer: hartinfo, postexec, data access, exception.
    hartinfo = w.read(HARTINFO)
    check((hartinfo >> 16) & 1, "hartinfo.dataaccess")
    expect((hartinfo >> 12) & 0xF, 2, "hartinfo.datasize")
    expect(hartinfo & 0xFFF, 0x450, "hartinfo.dataaddr")
    expect((hartinfo >> 20) & 0xF, 2, "hartinfo.nscratch")
    w.write(PROGBUF0, addi(6, 0, 0x123))
    w.write(PROGBUF0 + 1, NOP)
    expect(w.command(access_register(GPR, transfer=False, postexec=True)), 0, "postexec")
    expect(w.read_reg(GPR + 6)[1], 0x123, "progbuf wrote t1")
    w.write(DATA0, 0xCAFE)
    w.write(PROGBUF0, lw(7, 0, 0x450))
    w.write(PROGBUF0 + 1, sw(7, 0, 0x454))
    expect(w.command(access_register(GPR, transfer=False, postexec=True)), 0, "progbuf data access")
    expect(w.read(DATA0 + 1), 0xCAFE, "progbuf store to data1")
    expect(w.read_reg(GPR + 7)[1], 0xCAFE, "progbuf load of data0")
    w.write(PROGBUF0, 0)
    expect(w.command(access_register(GPR, transfer=False, postexec=True)), CMDERR_EXCEPTION, "progbuf exception")
    w.write(PROGBUF0, EBREAK)
    expect(w.command(access_register(GPR, transfer=False, postexec=True)), 0, "explicit ebreak")
    check(w.read(DMSTATUS) & ALLHALTED, "still halted after progbuf")

    # Autoexec: reading data0 repeats the command.
    w.write(PROGBUF0, addi(18, 18, 1))
    w.write(PROGBUF0 + 1, NOP)
    w.write_reg(GPR + 18, 0)
    w.write(COMMAND, access_register(GPR + 18, transfer=False, postexec=True))
    w.write(ABSTRACTAUTO, 0xFFFFFFFF)
    expect(w.read(ABSTRACTAUTO), (0x3 << 16) | 0x3, "abstractauto absent bits are 0")
    w.write(ABSTRACTAUTO, 1)
    w.read(DATA0)
    w.read(DATA0)
    w.write(ABSTRACTAUTO, 0)
    expect(w.read_reg(GPR + 18)[1], 3, "command write and 2 autoexec reads ran the program buffer")

    # Single step.
    pc0 = w.read_reg(DPC)[1]
    w.write_reg(DCSR, dcsr | 4)
    w.resume()
    status = w.read(DMSTATUS)
    check(status & ALLHALTED and status & ALLRESUMEACK, "halted again after a step")
    expect((w.read_reg(DCSR)[1] >> 6) & 7, 4, "dcsr.cause = step")
    check(w.read_reg(DPC)[1] != pc0, "step moved the pc")
    w.write_reg(DCSR, dcsr & ~4)

    # Resume ignored when haltreq is set; resume.
    w.write(DMCONTROL, DMACTIVE | HALTREQ | RESUMEREQ)
    check(w.read(DMSTATUS) & ALLHALTED, "resumereq ignored with haltreq")
    w.resume()
    status = w.read(DMSTATUS)
    check(status & ALLRUNNING and status & ALLRESUMEACK, "running after resume")

    # Hart array: halt both harts with one write.
    w.write(HAWINDOWSEL, 0)
    w.write(HAWINDOW, 3)
    expect(w.read(HAWINDOW), 3, "hawindow")
    w.write(DMCONTROL, DMACTIVE | HASEL | HALTREQ)
    check(w.read(DMSTATUS) & ALLHALTED, "hasel halts both harts")
    expect(w.read(HALTSUM0), 3, "haltsum0 both")
    expect(w.read(HALTSUM1), 1, "haltsum1")
    w.write(DMCONTROL, DMACTIVE | HASEL | RESUMEREQ)
    check(w.read(DMSTATUS) & ALLRUNNING, "hasel resumes both harts")

    # Halt groups: halting hart 0 halts hart 1.
    w.write(DMCONTROL, DMACTIVE | HASEL)
    w.write(DMCS2, (1 << 2) | 2)
    expect((w.read(DMCS2) >> 2) & 0x1F, 1, "halt group 1")
    w.write(DMCS2, (2 << 2) | 2)
    expect((w.read(DMCS2) >> 2) & 0x1F, 1, "halt group 2 not supported")
    w.write(DMCONTROL, DMACTIVE)
    w.write(DMCONTROL, DMACTIVE | HALTREQ)
    w.write(DMCONTROL, DMACTIVE | (1 << 16))
    check(w.read(DMSTATUS) & ALLHALTED, "group halted hart 1")
    expect((w.read_reg(DCSR)[1] >> 6) & 7, 6, "dcsr.cause = group")
    w.write(DMCONTROL, DMACTIVE | HASEL | RESUMEREQ)
    w.write(DMCONTROL, DMACTIVE | HASEL)
    w.write(DMCS2, (0 << 2) | 2)

    # ndmreset with resethaltreq.
    w.write(DMCONTROL, DMACTIVE | SETRESETHALTREQ)
    w.write(DMCONTROL, DMACTIVE | NDMRESET)
    check(w.read(DMSTATUS) & NDMRESETPENDING, "ndmresetpending")
    check(w.read(DMSTATUS) & ALLUNAVAIL, "harts unavailable in reset")
    w.write(DMCONTROL, DMACTIVE)
    status = w.read(DMSTATUS)
    check(not status & NDMRESETPENDING, "ndmresetpending clears")
    check(status & ALLHAVERESET, "havereset after ndmreset")
    check(status & ALLHALTED, "resethaltreq halted hart 0")
    expect((w.read_reg(DCSR)[1] >> 6) & 7, 5, "dcsr.cause = resethaltreq")
    expect(w.read_reg(DPC)[1], 0x80000000, "dpc = reset vector")
    w.write(DMCONTROL, DMACTIVE | (1 << 16))
    check(w.read(DMSTATUS) & ALLRUNNING, "hart 1 runs after ndmreset")
    w.write(DMCONTROL, DMACTIVE | CLRRESETHALTREQ | ACKHAVERESET | HASEL)

    # hartreset with haltreq set: hart 0 halts out of reset.
    w.write(DMCONTROL, DMACTIVE | HALTREQ | HARTRESET)
    check(w.read(DMCONTROL) & HARTRESET, "hartreset reads back")
    w.write(DMCONTROL, DMACTIVE | HALTREQ)
    status = w.read(DMSTATUS)
    check(status & ALLHAVERESET and status & ALLHALTED, "halted out of hartreset")
    expect((w.read_reg(DCSR)[1] >> 6) & 7, 3, "dcsr.cause = haltreq after reset")
    w.write(DMCONTROL, DMACTIVE)

    # dmactive=0 resets the DM but not the harts.
    w.write(DATA0, 0x77)
    w.write(PROGBUF0, 0x99)
    w.write(COMMAND, access_register(0x8F0))
    w.write(DMCONTROL, 0)
    expect(w.read(DMCONTROL), 0, "dmactive cleared")
    w.activate()
    expect(w.read(DATA0), 0, "data0 reset")
    expect(w.read(PROGBUF0), 0, "progbuf0 reset")
    expect(w.cmderr(), 0, "cmderr reset")
    check(w.read(DMSTATUS) & ALLHALTED, "hart stays halted across a DM reset")

    # Absent registers read 0.
    for addr in (SBCS, SBADDRESS0, SBDATA0, AUTHDATA, CONFSTRPTR0, NEXTDM, 0x1F, 0x7F):
        expect(w.read(addr), 0, f"register {addr:#x} reads 0")


def options(whisper, elf64, elf32):
    """Optional features turned off or on."""
    print("case: features off", flush=True)
    dm = {
        "progbufsize": 0,
        "datacount": 1,
        "abstractauto": False,
        "hasel": False,
        "haltgroups": 0,
        "resethaltreq": False,
        "hartreset": False,
        "access_register": {"csr": False, "fpr": False},
        "access_memory": {"enable": False},
    }
    w = Whisper(whisper, elf32, 32, dm, harts=2)
    try:
        w.activate()
        status = w.read(DMSTATUS)
        check(not status & HASRESETHALTREQ, "hasresethaltreq off")
        check(not status & IMPEBREAK, "impebreak reads 0 without a program buffer")
        expect(w.read(ABSTRACTCS) & 0x1F00000F, 1, "progbufsize 0, datacount 1")
        w.write(DMCONTROL, DMACTIVE | HASEL | HARTRESET)
        expect(w.read(DMCONTROL) & (HASEL | HARTRESET), 0, "hasel and hartreset off")
        w.write(DMCONTROL, DMACTIVE)
        w.write(ABSTRACTAUTO, 1)
        expect(w.read(ABSTRACTAUTO), 0, "abstractauto off")
        w.halt()
        expect(w.read_reg(GPR + 18)[0], 0, "GPR access")
        expect(w.read_reg(GPR + 18, 3)[0], CMDERR_NOTSUP, "64-bit access to an RV32 GPR")
        expect(w.command(access_register(MISA, xlen=32)), CMDERR_NOTSUP, "CSR access off")
        expect(w.command(access_register(0x1020, xlen=32)), CMDERR_NOTSUP, "FPR access off")
        expect(w.command(access_register(GPR, transfer=False, postexec=True, xlen=32)), CMDERR_NOTSUP,
               "postexec without a program buffer")
        expect(w.command((2 << 24) | (2 << 20)), CMDERR_NOTSUP, "access memory off")
        w.write(DMCS2, (1 << 2) | 2)
        expect(w.read(DMCS2), 0, "halt groups off")
    finally:
        w.close()

    print("case: features on", flush=True)
    dm = {
        "datacount": 4,
        "progbufsize": 4,
        "quick_access": True,
        "authentication": True,
        "stickyunavail": True,
        "relaxedpriv": "writable",
        "confstrptr": "0x12345678",
        "nextdm": "0x200",
        "abstract_busy_cycles": 50,
        "resumegroups": 1,
        "access_register": {"postincrement": True},
        "sba": {"enable": True, "busy_cycles": 50},
        "spike_test_hooks": True,
        "dtm": {"idcode": "0x10e31913", "abits": 9, "idle": 2, "dmi_busy_cycles": 2},
    }
    w = Whisper(whisper, elf64, 64, dm, harts=2)
    try:
        expect(w.idcode(), 0x10E31913, "configured idcode")
        expect(w.abits, 9, "configured abits")
        expect(w.idle, 2, "configured idle")

        # A DMI access scanned too early reports busy.
        w.activate()
        w.select(0x11)
        w.scan_dr((DMSTATUS << 34) | 1, w.abits + 34)
        r = w.scan_dr(0, w.abits + 34)
        expect(r & 3, 3, "DMI busy")
        expect((w.dtmcs() >> 10) & 3, 3, "dtmcs.dmistat sticky busy")
        w.dtmcs(1 << 16)
        expect((w.dtmcs() >> 10) & 3, 0, "dmireset")

        # Authentication: write the challenge plus 1.
        status = w.read(DMSTATUS)
        check(not status & AUTHENTICATED, "not authenticated")
        expect(w.read(DMSTATUS) & 0xF, 3, "version readable before authentication")
        w.write(DATA0, 5)
        challenge = w.read(AUTHDATA)
        w.write(AUTHDATA, challenge + 2)
        check(not w.read(DMSTATUS) & AUTHENTICATED, "wrong response")
        challenge = w.read(AUTHDATA)
        w.write(AUTHDATA, challenge + 1)
        check(w.read(DMSTATUS) & AUTHENTICATED, "authenticated")
        expect(w.read(DATA0), 0, "writes ignored before authentication")

        status = w.read(DMSTATUS)
        check(status & CONFSTRPTRVALID, "confstrptrvalid")
        expect(w.read(CONFSTRPTR0), 0x12345678, "confstrptr0")
        expect(w.read(NEXTDM), 0x200, "nextdm")
        w.write(ABSTRACTCS, 1 << 11)
        check(w.read(ABSTRACTCS) & (1 << 11), "relaxedpriv writable")

        # Busy abstract commands.
        w.halt()
        w.write(COMMAND, access_register(GPR + 18))
        check(w.read(ABSTRACTCS) & (1 << 12), "abstractcs.busy")
        w.write(DATA0, 1)
        w.run_idle(60)
        check(not w.read(ABSTRACTCS) & (1 << 12), "busy clears after run-test/idle cycles")
        expect(w.cmderr(), CMDERR_BUSY, "data0 write while busy")
        w.clear_cmderr()

        # Postincrement.
        w.write_reg(GPR + 5, 55)
        w.write_reg(GPR + 6, 66)
        expect(w.command(access_register(GPR + 5) | (1 << 19)), 0, "postincrement")
        expect(w.read(COMMAND), 0, "command reads 0")
        w.write(ABSTRACTAUTO, 1)
        expect(w.read(DATA0), 55, "x5")
        w.run_idle(60)
        expect(w.read(DATA0), 66, "x6 by postincrement and autoexec")
        w.run_idle(60)
        w.write(ABSTRACTAUTO, 0)
        w.run_idle(60)

        # Access Memory (4 data registers).
        def access_memory(addr, size, write=False, data=0, virtual=False, postinc=False):
            w.write(DATA0 + 2, addr & 0xFFFFFFFF)
            w.write(DATA0 + 3, addr >> 32)
            if write:
                w.write(DATA0, data & 0xFFFFFFFF)
                w.write(DATA0 + 1, data >> 32)
            cmd = (2 << 24) | (virtual << 23) | (size << 20) | (postinc << 19) | (write << 16)
            err = w.command(cmd)
            return err, w.read(DATA0) | (w.read(DATA0 + 1) << 32)

        expect(access_memory(0x80001000, 3, True, 0x1122334455667788)[0], 0, "memory write")
        expect(access_memory(0x80001000, 3)[1], 0x1122334455667788, "64-bit memory read")
        expect(access_memory(0x80001002, 1)[1], 0x5566, "16-bit memory read")
        expect(access_memory(0x80001000, 2, virtual=True)[1], 0x55667788, "virtual read")
        access_memory(0x80001000, 0, postinc=True)
        expect(w.read(DATA0 + 2), 0x80001001, "aampostincrement")
        expect(access_memory(0x80000000, 2)[1], 0x00000913, "read code (li s2, 0)")
        expect(access_memory(0x80000000, 4)[0], CMDERR_NOTSUP, "128-bit memory access")
        expect(access_memory(1 << 40, 2)[0], CMDERR_EXCEPTION, "memory access fault")

        # An ebreak with dcsr.ebreakm halts the hart at the ebreak (hart 1, which
        # runs the same code, stays halted meanwhile).
        w.halt(1)
        w.write(DMCONTROL, DMACTIVE | ACKHAVERESET)
        old_inst = access_memory(0x80000004, 2)[1] & 0xFFFFFFFF
        expect(access_memory(0x80000004, 2, True, EBREAK)[0], 0, "write ebreak")
        dcsr = w.read_reg(DCSR)[1]
        w.write_reg(DCSR, dcsr | (1 << 15))
        minstret = w.read_reg(0xB02)[1]
        w.resume()
        status = w.read(DMSTATUS)
        check(status & ALLHALTED, "halted by ebreak")
        expect((w.read_reg(DCSR)[1] >> 6) & 7, 1, "dcsr.cause = ebreak")
        expect(w.read_reg(DPC)[1], 0x80000004, "dpc = ebreak")
        check(w.read_reg(0xB02)[1] - minstret < 8, "minstret stops at the ebreak")
        access_memory(0x80000004, 2, True, old_inst)
        w.write_reg(DCSR, dcsr)
        w.resume(1)
        w.write(DMCONTROL, DMACTIVE)

        # System bus access.
        sbcs = w.read(SBCS)
        expect(sbcs >> 29, 1, "sbversion")
        expect((sbcs >> 5) & 0x7F, 64, "sbasize")
        expect(sbcs & 0x1F, 0xF, "sbaccess sizes")
        w.write(SBCS, (3 << 17) | (1 << 16))
        w.write(SBADDRESS1, 0)
        w.write(SBADDRESS0, 0x80001000)
        w.write(SBDATA1, 0xAABBCCDD)
        w.write(SBDATA0, 0x01020304)
        check(w.read(SBCS) & (1 << 21), "sbbusy")
        w.write(SBDATA0, 0)
        check(w.read(SBCS) & (1 << 22), "sbbusyerror")
        w.run_idle(60)
        w.write(SBCS, (1 << 22) | (3 << 17) | (1 << 16) | (1 << 20))
        expect(w.read(SBADDRESS0), 0x80001008, "sbautoincrement")
        w.write(SBADDRESS0, 0x80001000)
        w.run_idle(60)
        expect(w.read(SBDATA0) | (w.read(SBDATA1) << 32), 0xAABBCCDD01020304, "sbreadonaddr read")
        w.write(SBCS, (2 << 17))
        w.write(SBADDRESS0, 0x80001001)
        w.write(SBDATA0, 0)
        expect((w.read(SBCS) >> 12) & 7, 3, "sberror misaligned")

        # Quick access: the hart must be running.
        w.write(PROGBUF0, addi(19, 0, 0x77))
        w.write(PROGBUF0 + 1, EBREAK)
        expect(w.command(1 << 24), CMDERR_HALTRESUME, "quick access on a halted hart")
        w.resume()
        w.write(COMMAND, 1 << 24)
        w.run_idle(60)
        expect(w.cmderr(), 0, "quick access")
        check(w.read(DMSTATUS) & ALLRUNNING, "running after quick access")
        w.halt()
        expect(w.read_reg(GPR + 19)[1], 0x77, "quick access ran the program buffer")

        # Resume groups.
        w.write(DMCONTROL, DMACTIVE | (1 << 16) | HALTREQ)
        w.write(DMCONTROL, DMACTIVE | HASEL)
        w.write(HAWINDOW, 3)
        w.write(DMCS2, (1 << 11) | (1 << 2) | 2)
        expect(w.read(DMCS2), (1 << 11) | (1 << 2), "resume group 1")
        w.write(HAWINDOW, 0)
        w.write(DMCONTROL, DMACTIVE | RESUMEREQ)
        w.write(DMCONTROL, DMACTIVE | (1 << 16))
        check(w.read(DMSTATUS) & ALLRUNNING, "resume group resumed hart 1")
        w.write(DMCONTROL, DMACTIVE)

        # Spike test hooks: availability register and custom registers.
        w.write(0x1F, 2)
        status = w.read(DMSTATUS)
        check(status & ALLUNAVAIL, "hart 0 unavailable")
        w.write(0x1F, 3)
        check(w.read(DMSTATUS) & ALLUNAVAIL, "unavail sticky")
        w.write(DMCONTROL, DMACTIVE | (1 << 27))
        check(not w.read(DMSTATUS) & ANYUNAVAIL, "ackunavail")
        w.halt()
        w.write(DATA0, 100)
        expect(w.command(access_register(0xC003, 2, write=True)), 0, "write custom register")
        expect(w.read_reg(0xC005, 2)[1], 102, "read custom register")
    finally:
        w.close()


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    whisper, elf64, elf32 = sys.argv[1:4]
    print("case: defaults", flush=True)
    w = Whisper(whisper, elf64, 64, {}, harts=2)
    try:
        defaults(w)
    finally:
        w.close()
    options(whisper, elf64, elf32)

    if failures:
        print(f"{len(failures)} failures")
        return 1
    print("All debug module checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
