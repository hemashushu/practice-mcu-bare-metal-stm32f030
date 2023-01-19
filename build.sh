#!/bin/bash

rm main.elf

# FPU options
#
# CFLAGS += -mfloat-abi=soft # No FP
# CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16 # Soft FP
# CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 # Hard FP
#
# start files and standard libraries
#
# LDFLAGS += -nostartfiles # dont use standard start files
# LDFLAGS += -nodefaultlibs # dont use standard libraries
# LDFLAGS += -nostdlib # dont use startup or default libs
#
# Optimization
#
# the GCC optimization (e.g. arguments `-O1`, `-O2`, `-O3`, `-Os` etc.) may
# cause this demo program do not work.
#
# all options:
# https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html

arm-none-eabi-gcc \
    -mcpu=cortex-m0 \
    -mthumb \
    -Wall \
    -g \
    --specs=nosys.specs \
    -nostartfiles \
    -Wl,-T,linker.ld \
    -Wl,-gc-sections \
    -o main.elf \
    startup.c main.c peripheral.c utils.c

arm-none-eabi-size main.elf