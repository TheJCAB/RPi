/* kernel.ld - Simple linker script for Raspberry Pi kernel */

ENTRY(_start)

SECTIONS
{
    . = 0x80000; /* Kernel load address for RPi */
    _start = .;
    .text : {
        KEEP(*(.text.boot))
        *(.text*)
    }

    .rodata : {
        *(.rodata*)
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
}