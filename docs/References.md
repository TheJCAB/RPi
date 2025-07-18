
## Documentation

The Pi 1-3 SoC peripherals: https://datasheets.raspberrypi.com/bcm2835/bcm2835-peripherals.pdf

The Pi 3B SoC: https://www.raspberrypi.com/documentation/computers/processors.html#bcm2837
- Quad-A7 (in-ARM) peripherals in the SoC: https://datasheets.raspberrypi.com/bcm2836/bcm2836-peripherals.pdf
- Cortex A53 processor overview (not everything included in the SoC, especially the GiC): https://developer.arm.com/documentation/ddi0500/latest/
- VideoCore IV GPU: https://docs.broadcom.com/doc/12358545

The Pi 4 SoC: https://www.raspberrypi.com/documentation/computers/processors.html#bcm2711

ARMv8-A system registers and instructions: https://developer.arm.com/documentation/ddi0595/2021-06?lang=en

config.txt documentation: https://www.raspberrypi.com/documentation/computers/config_txt.html#what-is-config-txt

Firmware wiki has some good information on the GPU mailbox tag interface: https://github.com/raspberrypi/firmware/wiki

ABI (calling conventions): https://github.com/ARM-software/abi-aa/blob/main/aapcs64/aapcs64.rst



## Code references

A lot of OS. GPL3, but I'm only looking for insight: https://github.com/rsta2/uspi/tree/master/lib

CC0 code for RPi 4: https://github.com/babbleberry/rpi4-osdev

MIT, looks like, but unclear. It's the barest bare metal and includes V3D: https://github.com/kumaashi/RaspberryPI


## Interesting stuff

Microkernel written in Rust: https://www.redox-os.org/
Another (semi-open - although they claim Apache, but it looks complicated) OS: https://www.riscosopen.org/content/about
