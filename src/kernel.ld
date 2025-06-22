/* kernel.ld - Simple linker script for Raspberry Pi kernel */

ENTRY(_start)

SECTIONS
{
    . = 0x80000; /* Kernel load address for RPi */

    .text : {
        KEEP(*(.text.boot))
        *(.text*)
    }

    .rodata : {
        *(.rodata*)
        . = ALIGN(8);
        _init_array_start = .;
        KEEP(*(.init_array))
        _init_array_end = .;
        *(.eh_frame*)
    }

    . = ALIGN(4096);
    PROVIDE(_data = .);

    .data : {
        *(.data*)
    }

    .bss (NOLOAD) : {
        . = ALIGN(16);
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        _bss_end = .;
    }

    . = ALIGN(4096);
    _data_end = .;

    /DISCARD/ : { *(.debug*) *(.comment) *(.gnu*) *(.note*) }
}