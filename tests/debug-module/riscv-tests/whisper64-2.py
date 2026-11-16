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

"""Whisper counterpart of spike64-2: two RV64G harts, Whisper Debug Module."""
import targets
import whisperlib

class whisper64_hart(targets.Hart):
    xlen = 64
    ram = 0x1212340000
    ram_size = 0x10000000
    bad_address = ram - 8
    instruction_hardware_breakpoint_count = 4
    reset_vectors = [0x1212340000]
    link_script_path = "spike64.lds"

class whisper64_2(targets.Target):
    harts = [whisper64_hart(misa=0x8000000000141129),
            whisper64_hart(misa=0x8000000000141129)]
    openocd_config_path = "spike-2.cfg"
    timeout_sec = 180
    implements_custom_test = True
    support_memory_sampling = False # Needs SBA
    support_unavailable_control = True

    def create(self):
        return whisperlib.Whisper(self, isa="rv64imafdsu_zicsr_zifencei_zicntr_sdtrig")
