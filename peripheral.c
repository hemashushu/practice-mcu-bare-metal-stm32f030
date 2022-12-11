#include <stdint.h>
#include <stdbool.h>
#include "cmsis/stm32f030x8.h"

#include "common.h"
#include "pin.h"
#include "utils.h"

extern uint32_t SYSCLK; // SYSCLK
extern uint32_t HCLK;   // HCLK, for AHB, core, memory and DMA
extern uint32_t PCLK;   // PCLK, for APB peripherals, TIM, ADC, USART1

extern uint64_t systicks;

// for DMA memory test
extern char mem_dest[16];

void systick_init(uint32_t ticks)
{
    // ticks:
    // how many ticks to trigger the `SysTick` interrupt,

    if ((ticks - 1) > 0xffffff)
    {
        // Systick timer is 24 bit
        return;
    }

    // PM0215 4.4 SysTick timer (STK)
    // STK_CSR
    // - Bit 2 CLKSOURCE: Selects the timer clock source.
    //   - 0: External reference clock, i.e. the AHB clock (HCLK) divided by 8
    //   - 1: Processor clock, i.e. HCLK
    // - Bit 1 TICKINT: SysTick exception request enable
    // - Bit 0 ENABLE: Counter enable
    SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk |
                      SysTick_CTRL_TICKINT_Msk |
                      SysTick_CTRL_ENABLE_Msk);
    SysTick->LOAD = ticks - 1; // the ending number
    SysTick->VAL = 0;          // the start number

    // 6.3.22 RCC register map
    // 0x44 RCC_APB2ENR
    // bit 14: SYSCFGEN enable

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable SYSCFGEN
}

void systick_init_with_millisecond()
{
    // The SysTick clock source is the HCLK
    uint32_t ticks = HCLK / 1000;
    systick_init(ticks);
}

void SysTick_Handler()
{
    systicks++;
}

uint64_t get_expired_tick(uint64_t current_tick, uint64_t period_ticks)
{
    return current_tick + period_ticks;
}

bool is_tick_expired(uint64_t current_tick, uint64_t expired_tick)
{
    return current_tick > expired_tick;
}

void systick_delay(uint64_t period_ticks)
{
    uint64_t expired_tick = get_expired_tick(systicks, period_ticks);
    while (!is_tick_expired(systicks, expired_tick))
    {
        //
    }
}

void uart_write_byte(USART_TypeDef *USARTx, char c)
{
    // rm0360 23.7.10 Transmit data register (USART_TDR)
    while (!(USARTx->ISR & USART_ISR_TXE))
    {
        //
    }
    USARTx->TDR = c;
}

bool uart_read_ready(USART_TypeDef *USARTx)
{
    // If RXNE bit is set, data is ready
    return (USARTx->ISR & USART_ISR_RXNE) != 0;
}

char uart_read_byte(USART_TypeDef *USARTx)
{
    // rm0360 23.7.9 Receive data register (USART_RDR)
    return (USARTx->RDR & 0xff);
}

/**
 * @brief write a string to UART
 *
 * it also can implment a function as
 * `int _write(int handle, char* str, int size)`
 * it's one "system call" name for "standard library", so
 * the function `printf`, `putchar` will become available.
 * note it requires appending `LFLAGS += -lgcc` to compiler.
 *
 * @param USARTx
 * @param str
 * @param size
 */
void uart_write_str(USART_TypeDef *USARTx, char *str)
{
    int count = strlen(str);
    while (count--)
    {
        char c = *str++;
        uart_write_byte(USARTx, c);
    }
}

void uart_write_int(USART_TypeDef *USARTx, int i)
{
    char s[21];
    itoa(i, s, 21);
    uart_write_str(USARTx, s);
}

/**
 * @brief for the `button interrupt test`
 *
 */
void EXTI0_1_IRQHandler()
{
    uint8_t led_pin_number = get_pin_number(EXTERNAL_LED_PIN);
    uint8_t button_0_pin_number = get_pin_number(BUTTON_0_PIN);

    // RM0360 11.3.6 Pending register (EXTI_PR)
    // Bits 17:0 PIFx: Pending bit on line x (x = 17 to 0)
    // 0: no trigger request occurred
    // 1: selected trigger request occurred
    // This bit is set when the selected edge event arrives on the external interrupt line. This bit is
    // cleared by writing a 1 to the bit.
    if (EXTI->PR & (1 << button_0_pin_number))
    {
        // clear the EXTI status flag.
        EXTI->PR |= (1 << button_0_pin_number);

        // read the button value
        uint32_t button_value = GPIOB->IDR;
        if (button_value & (1 << button_0_pin_number)) // means button "release"
        {
            GPIOB->ODR &= ~(1 << led_pin_number); // set `0` to turn off external LED
        }
        else
        {
            GPIOB->ODR |= (1 << led_pin_number); // set `1` to turn on external LED
        }
    }
}

void timer_delay(TIM_TypeDef *TIMx, uint32_t prescaler, uint32_t ticks)
{
    // both `prescaler` and `delay_ticks` MUST less than 0xFFFF

    // disable the timer first
    TIMx->CR1 &= ~(TIM_CR1_CEN);
    // set prescaler and auto-reload registers.
    TIMx->PSC = (prescaler); // from 0 to SYSCLK
    TIMx->ARR = (0xFFFF);    // from 0 to 0xFFFF

    // apply settings to timer update event.
    // rm0360 14.4.6 TIM3 event generation register (TIM3_EGR)
    // UG: Update generation
    // This bit can be set by software, it is automatically cleared by hardware.
    // 0: No action
    // 1: Re-initialize the counter and generates an update of the registers.
    TIMx->EGR |= (TIM_EGR_UG);

    // enable the timer
    TIMx->CR1 |= (TIM_CR1_CEN);
    while (TIMx->CNT < ticks)
    {
    };

    // timer finish, disable it again and reset CNT.
    TIMx->CR1 &= ~(TIM_CR1_CEN);
    TIMx->CNT = (0);
}

/**
 * @brief for `test timer interrupt`
 *
 * @param TIMx
 * @param ms
 */
void start_timer(TIM_TypeDef *TIMx, uint16_t ms)
{
    // disable timer first
    TIMx->CR1 &= ~(TIM_CR1_CEN);

    // optional: reset the peripheral.
    // if (TIMx == TIM3)
    // {
    //     RCC->APB1RSTR |= (RCC_APB1RSTR_TIM3RST);
    //     RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST);
    // } ...

    // set prescaler and auto-reload registers.
    TIMx->PSC = SYSCLK / 1000; // from 0 to SYSCLK
    TIMx->ARR = ms;            // from 0 to 0xFFFF

    // send an update event to reset the timer and apply settings.
    TIMx->EGR |= TIM_EGR_UG;

    // enable the hardware interrupt.
    TIMx->DIER |= TIM_DIER_UIE;

    // enable the timer.
    TIMx->CR1 |= TIM_CR1_CEN;
}

void stop_timer(TIM_TypeDef *TIMx)
{
    // disable the timer
    TIMx->CR1 &= ~(TIM_CR1_CEN);

    // clear the 'pending update interrupt' flag.
    TIMx->SR &= ~(TIM_SR_UIF);
}

void TIM3_IRQHandler()
{
    static bool led_on = false;
    uint8_t led_pin_number = get_pin_number(EXTERNAL_LED_PIN);

    // handle a timer 'update' interrupt event

    // RM0360 14.4.5 TIM3 status register (TIM3_SR)
    // Bit 0 UIF: Update interrupt flag
    // This bit is set by hardware on an update event. It is cleared by software.
    // 0: No update occurred.
    // 1: Update interrupt pending.
    // This bit is set by hardware when the registers are updated:
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~(TIM_SR_UIF);

        // toggle the LED
        led_on = !led_on;
        if (led_on)
        {
            GPIOB->ODR |= (1 << led_pin_number); // set `1` to turn on external LED
        }
        else
        {
            GPIOB->ODR &= ~(1 << led_pin_number); // set `0` to turn off external LED
        }
    }
}

void i2c_start()
{
    // send 'Start' condition, and wait for acknowledge.
    I2C1->CR2 |= (I2C_CR2_START);
    while ((I2C1->CR2 & I2C_CR2_START))
    {
    }
}

void i2c_stop()
{
    // send 'Stop' condition, and wait for acknowledge.
    I2C1->CR2 |= (I2C_CR2_STOP);
    while ((I2C1->CR2 & I2C_CR2_STOP))
    {
    }

    // reset the ICR ('Interrupt Clear Register') event flag.
    I2C1->ICR |= (I2C_ICR_STOPCF);
    while ((I2C1->ICR & I2C_ICR_STOPCF))
    {
    }
}

void i2c_write_byte(uint8_t data)
{
    I2C1->TXDR = (I2C1->TXDR & 0xFFFFFF00) | data;

    // Wait for one of these ISR bits:
    // 'TXIS' ("ready for next byte")
    // 'TC'   ("transfer complete")
    while (!(I2C1->ISR & (I2C_ISR_TXIS | I2C_ISR_TC)))
    {
    }
}

uint8_t i2c_read_byte()
{
    // wait for a byte of data to be available, then read it.
    while (!(I2C1->ISR & I2C_ISR_RXNE))
    {
    }
    return (I2C1->RXDR & 0xFF);
}

void DMA1_Channel1_IRQHandler()
{
    // RM0360 10.4.1 DMA interrupt status register (DMA_ISR)
    // check if channel 1 "TCIFx" (Channel x transfer complete flag)
    if (DMA1->ISR & (1 << DMA_ISR_TCIF1))
    {
        // RM0360 10.4.2 DMA interrupt flag clear register (DMA_IFCR)
        // clear channel 1 transfer complete interrupt
        DMA1->IFCR |= (1 << DMA_IFCR_CTCIF1);

        uart_write_str(USART1, "DMA1 transfer complete interrupt occured\r\n");

        // print memory destination content
        uart_write_str(USART1, "memory dest NEW data: ");
        for (int i = 0; i < 16; i++)
        {
            uart_write_int(USART1, mem_dest[i]);
            uart_write_str(USART1, ",");
        }
        uart_write_str(USART1, "\r\n");
    }
}