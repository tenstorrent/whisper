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

"""Whisper counterpart of spike32: RV32IMAFDCV, one hart, Whisper Debug Module."""
import targets
import whisperlib

class whisper32_hart(targets.Hart):
    xlen = 32
    ram = 0x10100000
    ram_size = 0x10000000
    bad_address = ram - 8
    instruction_hardware_breakpoint_count = 4
    reset_vectors = [0x10110000]
    link_script_path = "spike32.lds"

class whisper32(targets.Target):
    harts = [whisper32_hart(misa=0x4034112d)]
    openocd_config_path = "spike-1.cfg"
    timeout_sec = 180
    implements_custom_test = True
    support_memory_sampling = False # Needs SBA
    freertos_binary = "bin/RTOSDemo32.axf"
    support_unavailable_control = True

    def create(self):
        # Same Debug Module options as spike32: dmi_rti=4, no halt groups.
        return whisperlib.Whisper(self, isa="rv32imafdcvsu_zicsr_zifencei_zicntr_sdtrig",
                dmi_rti=4, haltgroups=0)
