#include <stdint.h>

extern void NMI_Handler();
extern void HardFault_Handler();
extern void SysTick_Handler();
extern void EXTI0_1_IRQHandler();
extern void DMA1_Channel1_IRQHandler();
extern void TIM3_IRQHandler();

extern void main();

// the true startup code
// keyword `naked` indicates this function no function prologue.
__attribute__((naked, noreturn)) void Reset_Handler()
{
    // set initial stack pointer
    //
    // note::
    // since the stack point is already included in the vector_table (in the first entry),
    // this statement is not necessary.
    //
    // asm("ldr sp, = _estack");

    // memset .bss to zero, and copy .data section to RAM region
    extern long _sbss, _ebss, _sdata, _edata, _sidata;

    // initialize `BSS`
    // set all bytes within `.bss` to `0`
    for (long *mem_addr = &_sbss; mem_addr < &_ebss; mem_addr++)
    {
        *mem_addr = 0;
    }

    // initialize `Data`
    // copy the content of `.data` from `flash` to `RAM`
    for (long *mem_addr = &_sdata, *flash_addr = &_sidata; mem_addr < &_edata;)
    {
        *mem_addr++ = *flash_addr++;
    }

    // call user's main function
    main();

    // infinite loop in the case if main() returns
    for (;;)
    {
        (void)0;
    }
}

// PM0215 Programming manual
// 2.3.4 Vector table
// 47 exception handlers (includes 16 standard handlers)
//
// vector table entry list and item name from
// `STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f030x8.s`
__attribute__((section(".vector_table.exceptions"))) void (*Exceptions[46])() = {
    // &_estack,        // idx: 0 the initial stack pointer
    Reset_Handler,            // idx: 1 the address of the entry function
    NMI_Handler,              // idx: 2
    HardFault_Handler,        // idx: 3
    0,                        // idx: 4
    0,                        // idx: 5
    0,                        // idx: 6
    0,                        // idx: 7
    0,                        // idx: 8
    0,                        // idx: 9
    0,                        // idx: 10
    0,                        // idx: 11 SVC_Handler
    0,                        // idx: 12
    0,                        // idx: 13
    0,                        // idx: 14 PendSV_Handler
    SysTick_Handler,          // idx: 15 Systick
    0,                        // idx: 16
    0,                        // idx: 17
    0,                        // idx: 18
    0,                        // idx: 19
    0,                        // idx: 20
    EXTI0_1_IRQHandler,       // idx: 21
    0,                        // idx: 22
    0,                        // idx: 23
    0,                        // idx: 24
    DMA1_Channel1_IRQHandler, // idx: 25
    0,                        // idx: 26
    0,                        // idx: 27
    0,                        // idx: 28
    0,                        // idx: 29
    0,                        // idx: 30
    0,                        // idx: 31
    TIM3_IRQHandler,          // idx: 32
};

__attribute__((naked, noreturn)) void Default_Handler()
{
    for (;;)
    {
        (void)0;
    }
}