from ralph.test_entry import TestEntry
from ralph.config_utils import check_valid
from ralph.generators.simple import generate_simple_exec
from ralph.validators import (create_cpi_validator, create_ipc_validator)

import re

midcore_failing_tests = []
def dependency_tests(override_dict={}):
    default_config = dict(
        ADD_latency=1,
        ADD_throughput=6,
        MUL_latency=3,
        MUL_throughput=2,
        DIV_latency=6,
        DIV_throughput=1,
        FADD_latency=1,
        FADD_throughput=2,
        FDIV_latency=6,
        FDIV_throughput=1,
        FMUL_latency=6,
        FMUL_throughput=1

    )

    default_config.update(override_dict)

    valid_inst = [
        'ADD_V0', 'DIV_V0', 'MUL_V0', 'FADD.S_V0', 'FDIV.S_V0', 'FMUL.S_V0',
        'FADD.D_V0', 'FDIV.D_V0', 'FMUL.D_V0']

    tests = []

    def n_hop_tests(instruction="ADD_V0", desc="{dep_distance}-hop test"):
        tests = []

        for dep_distance in [1, 6]:
            test_name = f"{instruction}_{dep_distance}_hop"
            if dep_distance == 1:
                # We generate a test with microprobe
                key = f"{re.findall(r'[^ ._]*', instruction)[0]}_latency"
                validator = create_cpi_validator(default_config[key], 0.1)
            else:
                # We generate a test with microprobe
                key = f"{re.findall(r'[^ ._]*', instruction)[0]}_throughput"
                validator = create_ipc_validator(default_config[key], 0.1)

            test_path = generate_simple_exec(
                name=f"{test_name}", instructions=instruction, dep_distance=dep_distance)

            test = TestEntry.microprobe_test(name=f"{instruction}_{'throughput' if dep_distance == 6 else 'latency'}",
                                             path_to_benchmark=test_path,
                                             whisper_cycles=500000,
                                             description=f"measures {key}",
                                             validator=validator)

            tests.append(test)
        return tests

    for inst in valid_inst:
        tests += n_hop_tests(instruction=inst,
                             desc=f"{{dep_distance}}-hop test with {inst} ")
    return tests


all_tests = dict(
    n_hop_tests=dependency_tests())


if __name__ == "__main__":
    check_valid(all_tests, "trace")


"""
Test for max parallelism across different functional units



Test window size (many dependants -> some independant inst)

stream of loads/stores -> should be seeing saturating bw

load to use latency



"""
