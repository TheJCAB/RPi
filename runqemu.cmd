qemu-system-aarch64.exe -machine virt -cpu cortex-a72 -smp cpus=4 -m 2G -kernel "%~dp0build\clang\bin\kernelvirt.elf" -d guest_errors -serial mon:stdio -device pci-testdev -device qemu-xhci -device usb-hub,port=1 -device usb-kbd,port=1.1 -device usb-mouse,port=2 -nographic %* 
@rem ,dumpdtb=c:\temp\virt.dtb
@rem 