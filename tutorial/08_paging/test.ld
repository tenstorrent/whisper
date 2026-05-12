ENTRY(_start)
OUTPUT_ARCH(riscv)

SECTIONS {
    . = 0x70000000;
    .tohost : { *(.tohost) }
    . = 0x80000000;
    .text : { *(.text) }
    . = ALIGN(4096);
    .data : { *(.data) }
}
