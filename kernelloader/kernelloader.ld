/* kernel.ld - Simple linker script for Raspberry Pi kernel */

ENTRY(_start)

SECTIONS
{
    . = 0x80000; /* Kernel load address for RPi */

    .text : {
        KEEP(*(.text.boot))
        *(.text*)
    }

    _end = .;

    /DISCARD/ : { *(.debug*) *(.comment) *(.gnu*) *(.note*) }
}