"""RISC-V 64-bit cross-compilation toolchain definitions."""

load("@bazel_tools//tools/cpp:unix_cc_toolchain_config.bzl", "cc_toolchain_config")

def riscv64_toolchains():
    """Register RISC-V 64-bit cross-compilation toolchains (GCC and Clang)."""

    # RISC-V 64-bit Linux platform definition
    native.platform(
        name = "riscv64_linux",
        constraint_values = [
            "@platforms//os:linux",
            "@platforms//cpu:riscv64",
        ],
        visibility = ["//visibility:public"],
    )

    # Shared filegroup for toolchain files
    native.filegroup(
        name = "riscv64_all_files",
        srcs = [],
    )

    # GCC toolchain
    cc_toolchain_config(
        name = "riscv64_linux_gcc_toolchain_config",
        cpu = "riscv64",
        compiler = "gcc",
        toolchain_identifier = "riscv64-linux-gnu-gcc",
        host_system_name = "x86_64-linux-gnu",
        target_system_name = "riscv64-linux-gnu",
        target_libc = "glibc",
        abi_version = "unknown",
        abi_libc_version = "unknown",
        tool_paths = {
            "gcc": "riscv64-unknown-linux-gnu-gcc",
            "g++": "riscv64-unknown-linux-gnu-g++",
            "cpp": "riscv64-unknown-linux-gnu-cpp",
            "ar": "riscv64-unknown-linux-gnu-ar",
            "ld": "riscv64-unknown-linux-gnu-ld",
            "nm": "riscv64-unknown-linux-gnu-nm",
            "objdump": "riscv64-unknown-linux-gnu-objdump",
            "objcopy": "riscv64-unknown-linux-gnu-objcopy",
            "strip": "riscv64-unknown-linux-gnu-strip",
            "gcov": "riscv64-unknown-linux-gnu-gcov",
            "llvm-cov": "riscv64-unknown-linux-gnu-gcov",
            "dwp": "/usr/bin/false",
        },
        compile_flags = [
            "-fstack-protector",
            "-Wall",
            "-Wunused-but-set-parameter",
            "-Wno-free-nonheap-object",
            "-fno-omit-frame-pointer",
        ],
        cxx_flags = ["-std=c++20"],
        link_flags = [
            "-lstdc++",
            "-lm",
        ],
        cxx_builtin_include_directories = [
            "/opt/riscv64-linux-gnu/lib/gcc/riscv64-unknown-linux-gnu/16.0.0/include",
            "/opt/riscv64-linux-gnu/lib/gcc/riscv64-unknown-linux-gnu/16.0.0/include-fixed",
            "/opt/riscv64-linux-gnu/riscv64-unknown-linux-gnu/include/c++/16.0.0",
            "/opt/riscv64-linux-gnu/riscv64-unknown-linux-gnu/include/c++/16.0.0/riscv64-unknown-linux-gnu",
            "/opt/riscv64-linux-gnu/sysroot/usr/include",
        ],
    )

    native.cc_toolchain(
        name = "riscv64_linux_gcc_cc_toolchain",
        all_files = ":riscv64_all_files",
        compiler_files = ":riscv64_all_files",
        dwp_files = ":riscv64_all_files",
        linker_files = ":riscv64_all_files",
        objcopy_files = ":riscv64_all_files",
        strip_files = ":riscv64_all_files",
        supports_param_files = 1,
        toolchain_config = ":riscv64_linux_gcc_toolchain_config",
        toolchain_identifier = "riscv64-linux-gnu-gcc",
    )

    native.toolchain(
        name = "riscv64_linux_gcc_toolchain",
        exec_compatible_with = [
            "@platforms//os:linux",
            "@platforms//cpu:x86_64",
        ],
        target_compatible_with = [
            "@platforms//os:linux",
            "@platforms//cpu:riscv64",
        ],
        toolchain = ":riscv64_linux_gcc_cc_toolchain",
        toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
    )

    # Clang/LLVM toolchain
    cc_toolchain_config(
        name = "riscv64_linux_clang_toolchain_config",
        cpu = "riscv64",
        compiler = "clang",
        toolchain_identifier = "riscv64-linux-gnu-clang",
        host_system_name = "x86_64-linux-gnu",
        target_system_name = "riscv64-linux-gnu",
        target_libc = "glibc",
        abi_version = "unknown",
        abi_libc_version = "unknown",
        tool_paths = {
            "gcc": "/opt/riscv64-linux-gnu/bin/clang",
            "g++": "/opt/riscv64-linux-gnu/bin/clang++",
            "cpp": "/opt/riscv64-linux-gnu/bin/clang",
            "ar": "/opt/riscv64-linux-gnu/bin/llvm-ar",
            "ld": "/opt/riscv64-linux-gnu/bin/ld.lld",
            "nm": "/opt/riscv64-linux-gnu/bin/llvm-nm",
            "objdump": "/opt/riscv64-linux-gnu/bin/llvm-objdump",
            "objcopy": "/opt/riscv64-linux-gnu/bin/llvm-objcopy",
            "strip": "/opt/riscv64-linux-gnu/bin/llvm-strip",
            "gcov": "/opt/riscv64-linux-gnu/bin/llvm-cov",
            "llvm-cov": "/opt/riscv64-linux-gnu/bin/llvm-cov",
            "dwp": "/opt/riscv64-linux-gnu/bin/llvm-dwp",
        },
        compile_flags = [
            "--target=riscv64-unknown-linux-gnu",
            "-fstack-protector",
            "-Wall",
            "-fno-omit-frame-pointer",
        ],
        cxx_flags = ["-std=c++20"],
        link_flags = [
            "--target=riscv64-unknown-linux-gnu",
            "-fuse-ld=lld",
            "-lstdc++",
            "-lm",
        ],
        cxx_builtin_include_directories = [
            "/opt/riscv64-linux-gnu/lib/gcc/riscv64-unknown-linux-gnu/16.0.0/include",
            "/opt/riscv64-linux-gnu/lib/gcc/riscv64-unknown-linux-gnu/16.0.0/include-fixed",
            "/opt/riscv64-linux-gnu/riscv64-unknown-linux-gnu/include/c++/16.0.0",
            "/opt/riscv64-linux-gnu/riscv64-unknown-linux-gnu/include/c++/16.0.0/riscv64-unknown-linux-gnu",
            "/opt/riscv64-linux-gnu/sysroot/usr/include",
            "/opt/riscv64-linux-gnu/lib/clang/22/include",
        ],
    )

    native.cc_toolchain(
        name = "riscv64_linux_clang_cc_toolchain",
        all_files = ":riscv64_all_files",
        compiler_files = ":riscv64_all_files",
        dwp_files = ":riscv64_all_files",
        linker_files = ":riscv64_all_files",
        objcopy_files = ":riscv64_all_files",
        strip_files = ":riscv64_all_files",
        supports_param_files = 1,
        toolchain_config = ":riscv64_linux_clang_toolchain_config",
        toolchain_identifier = "riscv64-linux-gnu-clang",
    )

    native.toolchain(
        name = "riscv64_linux_clang_toolchain",
        exec_compatible_with = [
            "@platforms//os:linux",
            "@platforms//cpu:x86_64",
        ],
        target_compatible_with = [
            "@platforms//os:linux",
            "@platforms//cpu:riscv64",
        ],
        toolchain = ":riscv64_linux_clang_cc_toolchain",
        toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
    )
