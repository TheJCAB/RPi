.section .text

// This is where our code starts running.
// Note that it must be located/loaded at a specific address. 0x8'0000 on the Raspberry Pi 3/4.
// Upon boot, core zero starts here in EL2.
// The MMU is not configured so all the core sees is physical memory in "device" mode.
// Code runs quite slowly. Carefully, one could say :-).
// And things like atomic instructions and unaligned accesses don't work.
.global _start
_start:
    // Set up the stack buffer.
    // We'll use the 512 KB of space before the kernel image.
    adr x4, _start
    mov sp, x4

    // And dispatch straight into C++.
    .extern KernelMain
    b KernelMain
