/* the entry point */
ENTRY(Reset_Handler)

/* memory layout of STM32F030C8T6 */
MEMORY
{
  FLASH ( rx )      : ORIGIN = 0x08000000, LENGTH = 64K
  RAM ( rxw )       : ORIGIN = 0x20000000, LENGTH = 8K
}

/*
 * vector table entry list and item name from
 * `STM32Cube/Repository/STM32Cube_FW_F0_V1.11.3/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f030x8.s`
 *
 * the keyword `PROVIDE` means:
 * set LHS = RHS only when the left symbol does not exists,
 * i.e. the RHS is the default value.
 *
 * it can also define handlers in `startup.c`, and remove all `PROVIDE(*_Handler = Default_Handler)` in linker.ld, e.g.
 *
 * ```c
 * void NMI_Handler() __attribute__((weak, alias("Default_Handler")));
 * void HardFault_Handler() __attribute__((weak, alias("Default_Handler")));
 * ```
 */

/* ------ index 0-15 ------ */
/* idx 0: _estack */
/* idx 1: Reset_Handler */
PROVIDE(NMI_Handler = Default_Handler);
PROVIDE(HardFault_Handler = Default_Handler);
/* idx 4,5,6,7,8,9,10: 0 */
PROVIDE(SVC_Handler = Default_Handler);
/* idx 12,13: 0 */
PROVIDE(PendSV_Handler = Default_Handler);
PROVIDE(SysTick_Handler = Default_Handler);

/* ------ index 16-31 ------ */
PROVIDE(WWDG_IRQHandler = Default_Handler); /* Window Watchdog */
/* idx 17: 0 */
PROVIDE(RTC_IRQHandler = Default_Handler);
PROVIDE(FLASH_IRQHandler = Default_Handler);
PROVIDE(RCC_IRQHandler = Default_Handler);
PROVIDE(EXTI0_1_IRQHandler = Default_Handler);
PROVIDE(EXTI2_3_IRQHandler = Default_Handler);
PROVIDE(EXTI4_15_IRQHandler = Default_Handler);
/* idx 24: 0 */
PROVIDE(DMA1_Channel1_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel2_3_IRQHandler = Default_Handler);
PROVIDE(DMA1_Channel4_5_IRQHandler = Default_Handler);
PROVIDE(ADC1_IRQHandler = Default_Handler);
PROVIDE(TIM1_BRK_UP_TRG_COM_IRQHandler = Default_Handler); /* TIM1 Break, Update, Trigger and Commutation */
PROVIDE(TIM1_CC_IRQHandler = Default_Handler); /* TIM1 Capture Compare */
/* idx 31: 0 */

/* ------ index 32-47 ------ */
PROVIDE(TIM3_IRQHandler = Default_Handler);
PROVIDE(TIM6_IRQHandler = Default_Handler);
/* idx 34: 0 */
PROVIDE(TIM14_IRQHandler = Default_Handler);
PROVIDE(TIM15_IRQHandler = Default_Handler);
PROVIDE(TIM16_IRQHandler = Default_Handler);
PROVIDE(TIM17_IRQHandler = Default_Handler);
PROVIDE(I2C1_IRQHandler = Default_Handler);
PROVIDE(I2C2_IRQHandler = Default_Handler);
PROVIDE(SPI1_IRQHandler = Default_Handler);
PROVIDE(SPI2_IRQHandler = Default_Handler);
PROVIDE(USART1_IRQHandler = Default_Handler);
PROVIDE(USART2_IRQHandler = Default_Handler);
/* idx 45,46,47: 0 */

/*
 * use the `EXTERN` statement to preserve symbols that NOT directly or
 * indirectly referenced by the entry point.
 */
EXTERN(Reset_Handler);
EXTERN(Exceptions);

/* the stack point is the end address of RAM, i.e. `0x20000000 + 0x5000` */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{

  /* The vector table goes at the start of flash. */
  .vector_table :
  {
    /* first entry: the initial Stack Pointer (SP) value */
    LONG(_estack);

    /* the next 15 entries are exception vectors */
    /* keyword `KEEP` means that it is kept even if no other item refers to it. */
    KEEP(*(.vector_table.exceptions));
  } >FLASH

  /* The 'text' section contains the main program code. */
  .text : ALIGN(4)
  {
    *(.text .text.*);
  } > FLASH

  /* The 'rodata' section contains read-only data,
   * constants, strings, information that won't change. */
  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*);
  } > FLASH

  /* The 'data' section is space set aside in RAM for
   * things like variables, which can change.
   *
   * set the Load Memory Address (LMA) of the `.data` section by
   * `AT(...)` at header line or `AT > FLASH` at footer line
   */
  .data : ALIGN(4) /* AT(ADDR(.rodata) + SIZEOF(.rodata)) */
  {
    _sdata = .;
    *(.data .data.*);
    _edata = .;
  } >RAM AT >FLASH

  /**
   * get the LMA of the section `.data`
   * - 3.1 Basic Linker Script Concepts
   *   https://sourceware.org/binutils/docs/ld/Basic-Script-Concepts.html
   * - 3.6.8.2 Output Section LMA
   *   https://sourceware.org/binutils/docs/ld/Output-Section-LMA.html
   * - LOADADDR(section)
   *   https://sourceware.org/binutils/docs/ld/Builtin-Functions.html#index-LOADADDR_0028section_0029
   */
  /* LMA of .data */
  _sidata = LOADADDR(.data);

  /* The 'bss' section is similar to the 'data' section,
   * but its space is initialized to all 0s at the
   * start of the program. */
  .bss : ALIGN(4)
  {
    _sbss = .;
    *(.bss .bss.*);
    _ebss = .;
  } >RAM

  . = ALIGN(4);
  _heap_start = .;

  /* Discarded sections */
  /DISCARD/ :
  {
    *(.ARM.exidx .ARM.exidx.* .ARM.extab.*);
  }
}