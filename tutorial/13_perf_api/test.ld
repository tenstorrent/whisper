OUTPUT_ARCH("riscv")
ENTRY(_start)

STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x800;

SECTIONS {

    . = 0x70000000;
    .tohost : {
        *(.tohost)
    }

    . = 0x80000000;
    .text : {
        *(.text)
    }

    . = ALIGN(4096);
    .data : {
        PROVIDE( __global_pointer$ = . + 0x800 );
        *(.data)
    }

    .bss (NOLOAD) : {
        . = ALIGN(4);
        _szero = .;
        *(.sbss .bss)
        . = ALIGN(4);
        _ezero = .;
    }

    .stack (NOLOAD) : {
        . = ALIGN(8);
        _sstack = .;
        . = . + STACK_SIZE;
        . = ALIGN(8);
        _estack = .;
    }
}
