ENTRY(_start)
OUTPUT_ARCH(riscv)

SECTIONS {
    . = 0x80000000;
    .text : { *(.text) }
    .rodata : { *(.rodata*) }
    .data : { *(.data) }
    .tohost : { *(.tohost) }
}
