/* kernel.ld - Simple linker script for Raspberry Pi kernel */

ENTRY(_start)

SECTIONS
{
    . = 0x8000; /* Kernel load address for RPi */

    .text : {
        *(.text*)
    }

    .rodata : {
        *(.rodata*)
    }

    .data : {
        *(.data*)
    }

    .bss : {
        *(.bss*)
        *(COMMON)
    }
}