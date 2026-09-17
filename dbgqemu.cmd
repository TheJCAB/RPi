start cmd /c "%~dp0runqemu.cmd" -S -s %*
start windbgx -y "%~dp0build\clang\bin" -c ".symopt+ 0x40 ; .reload kernelvirt.elf=40100000 ; bp kernelvirt!BootLib::Cpu::Halt" -remote gdb:server=localhost,port=1234
