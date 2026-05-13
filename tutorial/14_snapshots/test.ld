ENTRY(_start)
OUTPUT_ARCH(riscv)

STACK_SIZE = 0x1000;

SECTIONS {
    . = 0x70000000;
    .tohost : { *(.tohost) }
    . = 0x80000000;
    .text : { *(.text) }
    . = ALIGN(4096);
    .data : { *(.data) }
    .bss : { *(.bss) }
    . = ALIGN(4096);
    .stack (NOLOAD): {
        _sstack = .;
        . = . + STACK_SIZE;
        _estack = .;
    }
}
