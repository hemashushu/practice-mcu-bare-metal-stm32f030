#!/bin/bash
./build.sh
openocd -f interface/cmsis-dap.cfg  -f target/stm32f0x.cfg -s "/usr/share/openocd/scripts" -c "program main.elf verify reset exit"