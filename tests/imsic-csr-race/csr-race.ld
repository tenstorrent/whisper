ENTRY(_start)
SECTIONS {
    . = 0x70000000;
    .tohost : { *(.tohost) }
    . = 0x80000000;
    .text : { *(.text) }
    .data : { *(.data) }
}
