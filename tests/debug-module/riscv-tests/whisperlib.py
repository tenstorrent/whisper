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

"""Launch Whisper with its Debug Module for the debug tests, like testlib.Spike.

The Whisper configuration mirrors the Spike targets: Spike's memory map (RAM at
the hart's ram address, CLINT at 0x2000000, everything else inaccessible so that
bad_address faults), 4 triggers, and the Debug Module options that
testlib.Spike passes to Spike (authentication, no abstract FPR access, ...).
"""

import json
import os
import re
import shlex
import shutil
import subprocess
import tempfile
import time

import testlib


class Whisper:
    # pylint: disable=too-many-instance-attributes,too-many-locals
    def __init__(self, target, isa, dmi_rti=0, haltgroups=1):
        self.process = None
        self.harts = target.harts or [target]
        hart = self.harts[0]
        xlen = hart.xlen

        self.infinite_loop = target.compile(hart,
                "programs/checksum.c", "programs/tiny-malloc.c",
                "programs/infinite_loop.S", "-DDEFINE_MALLOC", "-DDEFINE_FREE")

        trigger = {
            "reset": [0, 0, 0, "0x100807c"],  # tinfo: mcontrol, icount, itrigger, etrigger, mcontrol6
            "mask": ["0xfdffffff" if xlen == 32 else "0xfdffffffffffffff",
                     "0xffffffff" if xlen == 32 else "0xffffffffffffffff",
                     "0xffffffff" if xlen == 32 else "0xffffffffffffffff", 0],
        }
        trigger["poke_mask"] = trigger["mask"]
        ram_end = hart.ram + hart.ram_size
        memsize = 1 << max(32, (ram_end - 1).bit_length())
        dm_base = 0x400
        config = {
            "isa": isa,
            "reset_vec": hex(hart.reset_vectors[0]),
            "clint": "0x2000000",
            "memmap": {
                "size": hex(memsize),
                "page_size": 4096,
                "pma": [
                    {"low": hex(dm_base), "high": hex(dm_base + 0x7f),
                     "attribs": ["read", "write", "exec"]},
                    {"low": "0x2000000", "high": "0x200ffff",
                     "attribs": ["read", "write", "io"]},
                    {"low": hex(hart.ram), "high": hex(ram_end - 1),
                     "attribs": ["read", "write", "exec", "amo", "rsrv",
                                 "idempotent", "misal_ok", "cacheable"]},
                    {"low": "0", "high": hex(memsize - 1), "attribs": []},
                ],
            },
            "csr": {
                # dcsr: ebreakm/s/u, stepie, stopcount, stoptime, mprven, step, prv writable.
                "dcsr": {"mask": "0xbe17", "poke_mask": "0xbfff"},
                "dscratch0": {"exists": "true"},
                "dscratch1": {"exists": "true"},
            },
            "trigger_types": ["none", "disabled", "mcontrol", "mcontrol6", "icount", "itrigger", "etrigger"],
            "trigger_actions": ["raisebreak", "enterdebug"],
            "trigger_use_tcontrol": False,
            "triggers": [trigger] * 4,
            "debug_module": {
                "base": hex(dm_base),
                "authentication": True,
                "haltgroups": haltgroups,
                "access_register": {"fpr": False},
                "spike_test_hooks": True,
                "dtm": {"idcode": "0x10e31913", "dmi_busy_cycles": dmi_rti},
            },
        }
        if "v" in isa.split("_")[0][4:]:
            config["vector"] = {"bytes_per_vec": 16, "max_bytes_per_elem": 8}

        # pylint: disable-next=consider-using-with
        self.config_file = tempfile.NamedTemporaryFile(prefix="whisper-", suffix=".json",
                mode="w")
        json.dump(config, self.config_file, indent=2)
        self.config_file.flush()

        cmd = shlex.split(target.sim_cmd) if target.sim_cmd else ["whisper"]
        sim = shutil.which(cmd[0]) if cmd else None
        if not sim:
            raise testlib.TestLibError(f"whisper executable not found: {target.sim_cmd}")
        cmd[0] = os.path.realpath(sim)
        cmd += ["--configfile", self.config_file.name, "--harts", str(len(self.harts)),
                "--rbb-port", "0", self.infinite_loop]

        os.environ['WORK_AREA'] = f'0x{hart.ram:x}'
        os.environ['REMOTE_BITBANG_HOST'] = 'localhost'

        # pylint: disable-next=consider-using-with
        self.logfile = tempfile.NamedTemporaryFile(prefix="whisper-", suffix=".log")
        logname = self.logfile.name
        self.lognames = [logname]
        self.logfile.write(("+ " + " ".join(cmd) + "\n").encode())
        self.logfile.flush()
        # pylint: disable-next=consider-using-with
        self.process = subprocess.Popen(cmd, shell=False, stdin=subprocess.PIPE,
                stdout=self.logfile, stderr=self.logfile)

        self.port = None
        for _ in range(100):
            with open(logname, encoding='utf-8') as fd:
                m = re.search(r"Listening for remote bitbang connection on "
                        r"port (\d+).", fd.read())
            if m:
                self.port = int(m.group(1))
                os.environ['REMOTE_BITBANG_PORT'] = m.group(1)
                break
            time.sleep(0.11)
        if not self.port:
            testlib.print_log(logname)
            raise testlib.TestLibError("Didn't get whisper message about bitbang "
                    "connection")

    def __del__(self):
        if self.process:
            try:
                self.process.kill()
                self.process.wait()
            except OSError:
                pass

    def wait(self, *args, **kwargs):
        return self.process.wait(*args, **kwargs)
