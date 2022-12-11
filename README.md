# MCU STM32F030C8T6 Bare-metal in C

[EN](README.md) | [中文](README.zh-Hans.md)

A bare metal (register level) STM32F030C8T6/STM32F030 MCU program written in pure C without any IDE, SDK, HAL or library, the only tool required is the GCC compiler.Implement basic input and output, timer, serial communication, etc. by directly reading and writing registers of hardware.

## Routines

- test_set_clock: PLL and SYSCLK
- test_blink: GPIO output
- test_button: GPIO input
- test_systick: SysTick
- test_uart: UART communication
- test_button_interrupt: GPIO interrupt
- test_timer: general timer
- test_timer_interrupt: general timer with interrupt
- test_eeprom: I2C peripheral

## Wires

- PC13: builtin LED -
- PB9: external LED +
- PB0: button 0 (the another button pin connect to GND)
- PB1: button 1 (the another button pin connect to GND)
- PA9: USART1_TX -> CP2012 RX
- PA10: USART1_RX -> CP2012 TX
- PB6: SCL -> EEPROM SCL
- PB7: SDA -> EEPROM SDA

## Compile

Install `arm-none-eabi-gcc` on your platform first, then run:

`$ ./build.sh`

## Flash

Connect the MCU to your computer using the `CMSIS_DAP Link` and then run:

`$ ./flash.sh`

## Debug

`$ ./server-gdb-server.sh`

then open another terminal, and run:

`$ ./start-gdb-client-with-svd.sh`

## Reference Documents

Hardware description and register structure and value description documentations:

- STM32F030c8 Datasheet - production data
  https://www.st.com/resource/en/datasheet/stm32f030c8.pdf

- RM0360 Reference manual
  https://www.st.com/resource/en/reference_manual/rm0360-stm32f030x4x6x8xc-and-stm32f070x6xb-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

- PM0215 Programming manual
  https://www.st.com/resource/en/programming_manual/pm0215-stm32f0xxx-cortexm0-programming-manual-stmicroelectronics.pdf

Thanks:

- Getting Started with Bare Metal ESP32 Programming - Vivonomicon's blog
  https://vivonomicon.com/category/stm32_baremetal_examples/
- https://github.com/cpq/bare-metal-programming-guide
- https://github.com/fcayci/stm32f4-bare-metal
- https://github.com/getoffmyhack/STM32F103-Bare-Metal
- https://github.com/ataradov/mcu-starter-projects

## Rust language version

This project is also implemented in Rust by the author, for those interested please check out [MCU STM32F103C8T6 Bare-metal in Rust](https://github.com/hemashushu/practice-mcu-bare-metal-rust)
