qemu-system-aarch64.exe -machine virt -cpu cortex-a72 -smp cpus=4 -m 2G -kernel "%~dp0build\clang\bin\kernelvirt.img" -d guest_errors -serial mon:stdio %* 
@rem ,dumpdtb=c:\temp\virt.dtb
