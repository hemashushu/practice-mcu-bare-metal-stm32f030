#include <stdint.h>
#include <stdbool.h>
#include "cmsis/stm32f030x8.h"

#include "common.h"
#include "pin.h"
#include "peripheral.h"

void test_set_clock();
void test_blink();
void test_button();
void test_systick();
void test_uart();
void test_button_interrupt();
void test_timer();
void test_timer_interrupt();
void test_eeprom();
void test_dma();
void test_dma_interrupt();

uint32_t SYSCLK = 8000000; // SYSCLK
uint32_t HCLK = 8000000;   // HCLK, for AHB, core, memory and DMA
uint32_t PCLK = 8000000;   // PCLK, for APB peripherals, TIM, ADC, USART1

volatile uint64_t systicks = 0; // the current/total ticks

// for DMA memory test
#define MEM_TEST_LEN 16
char mem_src[MEM_TEST_LEN];
char mem_dest[MEM_TEST_LEN];

int main()
{
    test_set_clock();
    // test_blink();            // Test A
    // test_button();           // Test B
    // test_systick();          // Test C
    // test_uart();             // Test D
    test_button_interrupt(); // Test E
    // test_timer();            // Test F
    // test_timer_interrupt();  // Test G
    // test_eeprom();           // Test H
    // test_dma();              // Test I
    // test_dma_interrupt();    // Test J

    // ! Only one of the test (A,B,...,J) can be selected at a time.
    // ! `test_set_clock` can be combined with other test.

    while (1)
    {
        //
    }
}

void test_blink()
{
    // enable the GPIOx peripheral (clock)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // initialize the builtin LED pin
    uint8_t led_pin_number = get_pin_number(BUILTIN_LED_PIN);
    GPIOC->MODER &= ~(0x3 << (led_pin_number * 2));   // clear bits
    GPIOC->MODER |= (0x1 << (led_pin_number * 2));    // set bits to 0x01, `output` mode
    GPIOC->OSPEEDR &= ~(0x3 << (led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOC->OTYPER &= ~(1 << led_pin_number);          // clear bits, set output type to `push-pull`

    while (1)
    {
        // clear the bit of `output data register`, i.e. set the
        // bit to `0`
        //
        // because the builtin LED- pin connect to MCU, and LED+ connect to VCC,
        // so set output pin to HIGH level voltage whill turn off the LED,
        // and set to LOW to turn on.
        GPIOC->ODR &= ~(1 << led_pin_number); // set `0` to turn on builtin LED

        for (int i = 0; i < 200000; i++)
        {
        }

        GPIOC->ODR |= (1 << led_pin_number); // set `1` to turn off builtin LED

        for (int i = 0; i < 200000; i++)
        {
        }

    }
}

void test_button()
{
    // enable the GPIOx peripheral (clock)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // initialize the external LED pin
    uint8_t led_pin_number = get_pin_number(EXTERNAL_LED_PIN);
    GPIOB->MODER &= ~(0x3 << (led_pin_number * 2));   // clear bits
    GPIOB->MODER |= (0x1 << (led_pin_number * 2));    // set to `output` mode
    GPIOB->OSPEEDR &= ~(0x3 << (led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOB->OTYPER &= ~(0x1 << led_pin_number);        // clear bits, set output type to `push-pull`

    // initialize button 0
    uint8_t button_0_pin_number = get_pin_number(BUTTON_0_PIN);
    GPIOB->MODER &= ~(0x3 << (button_0_pin_number * 2)); // clear bits, set to `input` mode
    GPIOB->PUPDR &= ~(0x3 << (button_0_pin_number * 2)); // clear bits, set to `No pull-up, pull-down`
    GPIOB->PUPDR |= (0x1 << (button_0_pin_number * 2));  // set to `pull-up`

    while (1)
    {
        // one pin of the button connects to MCU, another pin connects to GND,
        // and the BUTTON_0_PIN was set to `pull-up` mode.
        // so when button "release" the value is `1`, and "press" is `0`
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

void test_set_clock()
{
    // clear some bits of the Flash 'Access Control Register'
    FLASH->ACR &= ~(FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);

    // set 1 wait-state and enable the prefetch buffer.
    FLASH->ACR |= FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE;

    // turn on HSE, a crystal oscillator with 8MHz,
    // and wait for it to be ready
    RCC->CR |= RCC_CR_HSEON;
    while ((!(RCC->CR & RCC_CR_HSERDY)))
    {
    };

    // set HSE DIV2, result to 4MHz
    RCC->CFGR2 &= ~(RCC_CFGR2_PREDIV);
    RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV2;

    // set PLL mul x 12 and PLL src to HSE
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
    RCC->CFGR |= (RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLSRC_HSE_PREDIV);

    // turn on PLL and wait for it to be ready
    RCC->CR |= (RCC_CR_PLLON);
    while (!(RCC->CR & RCC_CR_PLLRDY))
    {
    };

    // select PLL as the system clock source
    RCC->CFGR &= ~(RCC_CFGR_SW);
    RCC->CFGR |= (RCC_CFGR_SW_PLL);
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL))
    {
    };

    // update global variables
    SYSCLK = 48000000;
    HCLK = SYSCLK;
    PCLK = SYSCLK;
}

void setup_builtin_led_for_blink()
{
    // enable the GPIOx peripheral (clock)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // set builtin LED to output mode
    // initialize the builtin LED pin
    uint8_t led_pin_number = get_pin_number(BUILTIN_LED_PIN);
    GPIOC->MODER &= ~(0x3 << (led_pin_number * 2));   // clear bits
    GPIOC->MODER |= (0x1 << (led_pin_number * 2));    // set bits to 0x01, `output` mode
    GPIOC->OSPEEDR &= ~(0x3 << (led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOC->OTYPER &= ~(1 << led_pin_number);          // clear bits, set output type to `push-pull`
}

void test_systick()
{
    // set tick every 1 ms
    systick_init_with_millisecond();

    setup_builtin_led_for_blink();
    uint8_t led_pin_number = get_pin_number(BUILTIN_LED_PIN);

    while (1)
    {
        systick_delay(500);
        GPIOC->ODR &= ~(1 << led_pin_number); // set `0` to turn on builtin LED

        systick_delay(500);
        GPIOC->ODR |= (1 << led_pin_number); // set `1` to turn off builtin LED
    }
}

void test_uart()
{
    // set tick every 1 ms
    systick_init_with_millisecond();

    // enable the GPIOx peripheral (clock)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // set builtin LED to output mode
    // initialize the builtin LED pin
    uint8_t builtin_led_pin_number = get_pin_number(BUILTIN_LED_PIN);
    GPIOC->MODER &= ~(0x3 << (builtin_led_pin_number * 2));   // clear bits
    GPIOC->MODER |= (0x1 << (builtin_led_pin_number * 2));    // set bits to 0x01, `output` mode
    GPIOC->OSPEEDR &= ~(0x3 << (builtin_led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOC->OTYPER &= ~(1 << builtin_led_pin_number);          // clear bits, set output type to `push-pull`

    // set external LED to output mode
    // initialize the external LED pin
    uint8_t external_led_pin_number = get_pin_number(EXTERNAL_LED_PIN);
    GPIOB->MODER &= ~(0x3 << (external_led_pin_number * 2));   // clear bits
    GPIOB->MODER |= (0x1 << (external_led_pin_number * 2));    // set to `output` mode
    GPIOB->OSPEEDR &= ~(0x3 << (external_led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOB->OTYPER &= ~(0x1 << external_led_pin_number);        // clear bits, set output type to `push-pull`

    // STM32F030 datasheet Table 11. STM32F030x4/6/8/C pin definitions (continued)
    // Pinouts
    // - PA9: USART1_TX
    // - PA10: USART1_RX
    //
    // note:
    // The connection wires between the MCU and USB-TTL dongle (CP2102, FT232, CH340 etc.)
    // need to have the TX and RX crossed, e.g.
    //
    // MCU                CP210x/FT232/CH340 Dongle
    // -------            -----------
    // PA9  TX  <-------> RX  (or TX)
    // PA10 RX  <-------> TX  (or RX)
    //     GND  <-------> GND
    //     3.3  <-------> 3.3V

    // to check out the UART output, run the following command in host:
    // `$ picocom -b 115200 /dev/ttyUSB1`
    // the USB-UART device path may be `/dev/ttyUSB0` or other else.
    // press `Ctrl+a, Ctrl+x` to exit `picocom`.

    // enable the GPIOx peripheral (clock)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // STM32F030 datasheet
    // Table 12. Alternate functions selected through GPIOA_AFR registers for port A
    // init USART1, use AF1
    uint8_t tx_pin_number = get_pin_number(TX_PIN);
    uint8_t rx_pin_number = get_pin_number(RX_PIN);
    GPIOA->AFR[tx_pin_number / 8] &= ~(0xF << ((tx_pin_number % 8) * 4));
    GPIOA->AFR[tx_pin_number / 8] |= (0x1 << ((tx_pin_number % 8) * 4));
    GPIOA->AFR[rx_pin_number / 8] &= ~(0xF << ((rx_pin_number % 8) * 4));
    GPIOA->AFR[rx_pin_number / 8] |= (0x1 << ((rx_pin_number % 8) * 4));

    // TX (PA9) pin is configured as output mode, push-pull
    GPIOA->MODER &= ~(0x3 << (tx_pin_number * 2));   // clear bits
    GPIOA->MODER |= (0x2 << (tx_pin_number * 2));    // set bits to 0b10, `Alternate function mode`
    GPIOA->OSPEEDR &= ~(0x3 << (tx_pin_number * 2)); // clear bits, set speed to `low`
    GPIOA->OTYPER &= ~(1 << tx_pin_number);          // clear bits, set output type to `push-pull`

    // RX (PA10) pin is configured as input mode and floating.
    GPIOA->MODER &= ~(0x3 << (rx_pin_number * 2)); // clear bits, set to `input` mode
    GPIOA->MODER |= (0x2 << (rx_pin_number * 2));  // set bits to 0b10, `Alternate function mode`
    GPIOA->PUPDR &= ~(0x3 << (rx_pin_number * 2)); // clear bits, set to `No pull-up, pull-down`

    // enable USART1 clock
    // RM0360 7.4.7 APB peripheral clock enable register 2 (RCC_APB2ENR)
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // set UART baud rate to 115200.
    uint32_t baudrate = 115200;

    // RM0360 7.4.13 Clock configuration register 3 (RCC_CFGR3)
    // Bits 1:0 USART1SW[1:0]: USART1 clock source selection
    // This bit is set and cleared by software to select the USART1 clock source.
    // - 00: PCLK selected as USART1 clock source (default)
    // - 01: System clock (SYSCLK) selected as USART1 clock
    // - 10: LSE clock selected as USART1 clock
    // - 11: HSI clock selected as USART1 clock
    uint32_t clock = PCLK;

    uint32_t baud = (clock + baudrate / 2) / baudrate;
    USART1->BRR = baud;
    USART1->CR1 = USART_CR1_TE | // Transmitter enable
                  USART_CR1_RE | // Receiver enable
                  USART_CR1_UE;  // USART enable

    while (1)
    {
        // turn on builtin LED
        GPIOC->ODR &= ~(1 << builtin_led_pin_number); // set `0` to turn on builtin LED
        uart_write_str(USART1, "builtin LED on\r\n");
        systick_delay(100);

        // turn on external LED
        GPIOB->ODR |= (1 << external_led_pin_number); // set `1` to turn on external LED
        uart_write_str(USART1, "external LED on\r\n");
        systick_delay(100);

        GPIOC->ODR |= (1 << builtin_led_pin_number);   // set `1` to turn off builtin LED
        GPIOB->ODR &= ~(1 << external_led_pin_number); // set `0` to turn off external LED
        uart_write_str(USART1, "LED off\r\n");
        systick_delay(500);
    }
}

void test_button_interrupt()
{
    // set tick every 1 ms
    systick_init_with_millisecond();

    // enable the GPIOx peripheral (clock)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // set builtin LED to output mode
    // initialize the builtin LED pin
    uint8_t builtin_led_pin_number = get_pin_number(BUILTIN_LED_PIN);
    GPIOC->MODER &= ~(0x3 << (builtin_led_pin_number * 2));   // clear bits
    GPIOC->MODER |= (0x1 << (builtin_led_pin_number * 2));    // set bits to 0x01, `output` mode
    GPIOC->OSPEEDR &= ~(0x3 << (builtin_led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOC->OTYPER &= ~(1 << builtin_led_pin_number);          // clear bits, set output type to `push-pull`

    // set external LED to output mode
    // initialize the external LED pin
    uint8_t external_led_pin_number = get_pin_number(EXTERNAL_LED_PIN);
    GPIOB->MODER &= ~(0x3 << (external_led_pin_number * 2));   // clear bits
    GPIOB->MODER |= (0x1 << (external_led_pin_number * 2));    // set to `output` mode
    GPIOB->OSPEEDR &= ~(0x3 << (external_led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOB->OTYPER &= ~(0x1 << external_led_pin_number);        // clear bits, set output type to `push-pull`

    // initialize button 0
    uint8_t button_0_pin_number = get_pin_number(BUTTON_0_PIN);
    GPIOB->MODER &= ~(0x3 << (button_0_pin_number * 2)); // clear bits, set to `input` mode
    GPIOB->PUPDR &= ~(0x3 << (button_0_pin_number * 2)); // clear bits, set to `No pull-up, pull-down`
    GPIOB->PUPDR |= (0x1 << (button_0_pin_number * 2));  // set to `pull-up`

    // setup button interrupt

    // enable the SYSCFG peripheral.
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // set SYSCFG to connect the button EXTI line to GPIOB.
    // equals to:
    // - SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1_Msk);
    // - SYSCFG->EXTICR[0] |=  (SYSCFG_EXTICR1_EXTI1_PB);
    SYSCFG->EXTICR[(button_0_pin_number / 4)] &= ~(0xF << ((button_0_pin_number % 4) * 4));
    SYSCFG->EXTICR[(button_0_pin_number / 4)] |= (0x1 << ((button_0_pin_number % 4) * 4));

    // setup the button's EXTI line as an interrupt.
    EXTI->IMR |= (1 << button_0_pin_number);
    // enable the 'rising edge' trigger (button release).
    EXTI->RTSR |= (1 << button_0_pin_number);
    // enable the 'falling edge' trigger (button press).
    EXTI->FTSR |= (1 << button_0_pin_number);

    // enable the NVIC interrupt for EXTI0 and EXTI1 at minimum priority.
    NVIC_SetPriority(EXTI0_1_IRQn, 0x03);
    NVIC_EnableIRQ(EXTI0_1_IRQn);

    // blink the builtin LED
    while (1)
    {
        systick_delay(200);
        GPIOC->ODR &= ~(1 << builtin_led_pin_number); // set `0` to turn on builtin LED

        systick_delay(200);
        GPIOC->ODR |= (1 << builtin_led_pin_number); // set `1` to turn off builtin LED
    }
}

void test_timer()
{
    setup_builtin_led_for_blink();
    uint8_t led_pin_number = get_pin_number(BUILTIN_LED_PIN);

    // enable the TIM3 clock.
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    uint32_t delay_ms = 200;
    // both `prescaler` and `delay_ticks` MUST less than 0xFFFF
    uint32_t prescaler = SYSCLK / 1000; // 1ms
    uint32_t delay_ticks = delay_ms;

    while (1)
    {
        timer_delay(TIM3, prescaler, delay_ticks);
        GPIOC->ODR &= ~(1 << led_pin_number); // set `0` to turn on builtin LED

        timer_delay(TIM3, prescaler, delay_ticks);
        GPIOC->ODR |= (1 << led_pin_number); // set `1` to turn off builtin LED
    }
}

void test_timer_interrupt()
{
    // set tick every 1 ms
    systick_init_with_millisecond();

    // enable the GPIOx peripheral (clock)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // set builtin LED to output mode
    // initialize the builtin LED pin
    uint8_t builtin_led_pin_number = get_pin_number(BUILTIN_LED_PIN);
    GPIOC->MODER &= ~(0x3 << (builtin_led_pin_number * 2));   // clear bits
    GPIOC->MODER |= (0x1 << (builtin_led_pin_number * 2));    // set bits to 0x01, `output` mode
    GPIOC->OSPEEDR &= ~(0x3 << (builtin_led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOC->OTYPER &= ~(1 << builtin_led_pin_number);          // clear bits, set output type to `push-pull`

    // set external LED to output mode
    // initialize the external LED pin
    uint8_t external_led_pin_number = get_pin_number(EXTERNAL_LED_PIN);
    GPIOB->MODER &= ~(0x3 << (external_led_pin_number * 2));   // clear bits
    GPIOB->MODER |= (0x1 << (external_led_pin_number * 2));    // set to `output` mode
    GPIOB->OSPEEDR &= ~(0x3 << (external_led_pin_number * 2)); // clear bits, set speed to `low`
    GPIOB->OTYPER &= ~(0x1 << external_led_pin_number);        // clear bits, set output type to `push-pull`

    // enable the TIM3 clock.
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // enable the NVIC interrupt for TIM3.
    NVIC_SetPriority(TIM3_IRQn, 0x03);
    NVIC_EnableIRQ(TIM3_IRQn);

    start_timer(TIM3, 500);

    // blink the builtin LED
    while (1)
    {
        systick_delay(200);
        GPIOC->ODR &= ~(1 << builtin_led_pin_number); // set `0` to turn on builtin LED

        systick_delay(200);
        GPIOC->ODR |= (1 << builtin_led_pin_number); // set `1` to turn off builtin LED
    }
}

void setup_uart1_for_printing()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    uint8_t tx_pin_number = get_pin_number(TX_PIN);
    uint8_t rx_pin_number = get_pin_number(RX_PIN);

    GPIOA->AFR[tx_pin_number / 8] &= ~(0xF << ((tx_pin_number % 8) * 4));
    GPIOA->AFR[tx_pin_number / 8] |= (0x1 << ((tx_pin_number % 8) * 4));
    GPIOA->AFR[rx_pin_number / 8] &= ~(0xF << ((rx_pin_number % 8) * 4));
    GPIOA->AFR[rx_pin_number / 8] |= (0x1 << ((rx_pin_number % 8) * 4));

    // TX (PA9) pin is configured as output mode, push-pull
    GPIOA->MODER &= ~(0x3 << (tx_pin_number * 2));   // clear bits
    GPIOA->MODER |= (0x2 << (tx_pin_number * 2));    // set bits to 0b10, `Alternate function mode`
    GPIOA->OSPEEDR &= ~(0x3 << (tx_pin_number * 2)); // clear bits, set speed to `low`
    GPIOA->OTYPER &= ~(1 << tx_pin_number);          // clear bits, set output type to `push-pull`

    // RX (PA10) pin is configured as input mode and floating.
    GPIOA->MODER &= ~(0x3 << (rx_pin_number * 2)); // clear bits, set to `input` mode
    GPIOA->MODER |= (0x2 << (rx_pin_number * 2));  // set bits to 0b10, `Alternate function mode`
    GPIOA->PUPDR &= ~(0x3 << (rx_pin_number * 2)); // clear bits, set to `No pull-up, pull-down`

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // set UART baud rate to 115200.
    uint32_t baudrate = 115200;

    uint32_t clock = PCLK;

    uint32_t baud = (clock + baudrate / 2) / baudrate;
    USART1->BRR = baud;
    USART1->CR1 = USART_CR1_TE | // Transmitter enable
                  USART_CR1_RE | // Receiver enable
                  USART_CR1_UE;  // USART enable
}

void test_eeprom()
{
    setup_uart1_for_printing();

    // enable the GPIOx peripheral
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    uint8_t scl_pin_number = get_pin_number(SCL_PIN);
    uint8_t sda_pin_number = get_pin_number(SDA_PIN);

    // initialize I2C1 pins, use AF1
    GPIOB->AFR[scl_pin_number / 8] &= ~(0xF << ((scl_pin_number % 8) * 4));
    GPIOB->AFR[scl_pin_number / 8] |= (0x1 << ((scl_pin_number % 8) * 4));
    GPIOB->AFR[sda_pin_number / 8] &= ~(0xF << ((sda_pin_number % 8) * 4));
    GPIOB->AFR[sda_pin_number / 8] |= (0x1 << ((sda_pin_number % 8) * 4));

    // setup pins
    GPIOB->MODER &= ~(0x3 << (scl_pin_number * 2));
    GPIOB->MODER |= (0x2 << (scl_pin_number * 2)); // set to alternative function mode
    GPIOB->PUPDR &= ~(0x3 << (scl_pin_number * 2));
    GPIOB->PUPDR |= (0x1 << (scl_pin_number * 2)); // pull-up
    GPIOB->OTYPER |= (0x1 << scl_pin_number);      // open-drain

    GPIOB->MODER &= ~(0x3 << (sda_pin_number * 2));
    GPIOB->MODER |= (0x2 << (sda_pin_number * 2)); // set to alternative function mode
    GPIOB->PUPDR &= ~(0x3 << (sda_pin_number * 2));
    GPIOB->PUPDR |= (0x1 << (sda_pin_number * 2)); // pull-up
    GPIOB->OTYPER |= (0x1 << sda_pin_number);      // open-drain

    // enable I2C1 peripheral clock
    RCC->CFGR3 &= ~(RCC_CFGR3_I2C1SW);
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW_SYSCLK;

    // enable the I2C1 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // initialize the I2C1 peripheral.

    // disable the peripheral.
    I2C1->CR1 &= ~(I2C_CR1_PE);

    // configure I2C timing.
    I2C1->TIMINGR &= (I2C_TIMINGR_PRESC |
                      I2C_TIMINGR_SCLDEL |
                      I2C_TIMINGR_SDADEL |
                      I2C_TIMINGR_SCLL |
                      I2C_TIMINGR_SCLH);

    // rm0360 22.7.5 Timing register (I2C_TIMINGR)
    // The STM32CubeMX tool calculates and provides the I2C_TIMINGR content in the I2C
    // Configuration window.
    // I2C1->TIMINGR |= 0x2000090E; // for default HSI 8MHz
    I2C1->TIMINGR |= (0x20303E5D); // for HSE and PLL 48MHz

    // enable the peripheral
    I2C1->CR1 |= I2C_CR1_PE;

    // ============= read the first 4 bytes of EEPROM data.

    unsigned char data[4] = {
        0x00, // this should be `100`
        0x00, // this should be `0`
        0x00, // this should be an increasing number
        0x00  // this is an uninitialized number
    };

    // set the EEPROM's I2C address.
    I2C1->CR2 &= ~(I2C_CR2_SADD);
    I2C1->CR2 |= (EEPROM_ADDR << I2C_CR2_SADD_Pos);

    // write the EEPROM address: 0x000000.
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos);
    i2c_start();
    i2c_write_byte(0x00);
    i2c_write_byte(0x00);
    i2c_stop();

    // read 4 bytes
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 |= (4 << I2C_CR2_NBYTES_Pos);

    I2C1->CR2 |= (I2C_CR2_RD_WRN); // set 'read' I2C direction.
    i2c_start();
    for (uint8_t idx = 0; idx < 4; idx++)
    {
        data[idx] = i2c_read_byte();
    }
    i2c_stop();

    // print data content
    uart_write_str(USART1, "EEPROM data (at 0x000000): ");
    for (uint8_t idx = 0; idx < 4; idx++)
    {
        uart_write_int(USART1, data[idx]);
        uart_write_str(USART1, ",");
    }
    uart_write_str(USART1, "\r\n");

    I2C1->CR2 &= ~(I2C_CR2_RD_WRN); // restore to 'write' I2C direction

    // ============= write the 3 bytes to EEPROM.

    // Set the EEPROM's I2C address.
    I2C1->CR2 &= ~(I2C_CR2_SADD);
    I2C1->CR2 |= (EEPROM_ADDR << I2C_CR2_SADD_Pos);

    // write 5 bytes total, includes 2 bytes for address
    I2C1->CR2 &= ~(I2C_CR2_NBYTES);
    I2C1->CR2 |= (17 << I2C_CR2_NBYTES_Pos);
    // write to address 0.
    i2c_start();
    i2c_write_byte(0x00);
    i2c_write_byte(0x00);
    // write data
    i2c_write_byte(100);         // idx 0
    i2c_write_byte(0);           // idx 1
    i2c_write_byte(data[2] + 1); // idx 2
    i2c_stop();

    uart_write_str(USART1, "EEPROM data updated.\r\n");
    uart_write_str(USART1, "Press [Reset] button on the MCU to checkout the updated value.\r\n");
}

void test_dma()
{
    setup_uart1_for_printing();

    // initialize `mem_src` with `0..15`
    for (int i = 0; i < MEM_TEST_LEN; i++)
    {
        mem_src[i] = i;
    }

    // print memory source content
    uart_write_str(USART1, "memory source addr: ");
    uart_write_int(USART1, mem_src);
    uart_write_str(USART1, ", data: ");
    for (int i = 0; i < MEM_TEST_LEN; i++)
    {
        uart_write_int(USART1, mem_src[i]);
        uart_write_str(USART1, ",");
    }
    uart_write_str(USART1, "\r\n");

    // print memory destination content
    uart_write_str(USART1, "memory dest addr: ");
    uart_write_int(USART1, mem_dest);
    uart_write_str(USART1, ", data: ");
    for (int i = 0; i < MEM_TEST_LEN; i++)
    {
        uart_write_int(USART1, mem_dest[i]);
        uart_write_str(USART1, ",");
    }
    uart_write_str(USART1, "\r\n");

    // setup DMA ===============

    // enable DMA clock
    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    // RM0360 10.4.3 DMA channel x configuration register (DMA_CCRx)

    // disable DMA1 first
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    DMA1_Channel1->CCR |= DMA_CCR_MEM2MEM; // memory-to-memory
    DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;   // no circular/loop

    DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel1->CCR |= (0 << DMA_CCR_MSIZE_Pos); // memory size: 8 bits

    DMA1_Channel1->CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel1->CCR |= (0 << DMA_CCR_PSIZE_Pos); // peripheral size: 8 bits

    DMA1_Channel1->CCR |= DMA_CCR_MINC; // enable memory increment mode
    DMA1_Channel1->CCR |= DMA_CCR_PINC; // enable peripheral increment mode

    // RM0360 10.4.3 DMA channel x configuration register (DMA_CCRx)
    //
    // Bit 4 DIR: Data transfer direction
    // This bit is set and cleared by software.
    // 0: Read from peripheral
    // 1: Read from memory
    DMA1_Channel1->CCR |= DMA_CCR_DIR; // memory -> peripheral

    DMA1_Channel1->CMAR = mem_src;  // memory address (source address)
    DMA1_Channel1->CPAR = mem_dest; // peripheral address (destination address)

    // RM0360 10.4.4 DMA channel x number of data register
    DMA1_Channel1->CNDTR = MEM_TEST_LEN; // number of data to transfer

    // enable DMA1
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // wait for DMA1 complete
    // CNDTR will decrease until it reaches 0.
    while (DMA1_Channel1->CNDTR)
    {
        //
    }

    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    uart_write_str(USART1, "DMA1 transfer complete.\r\n");

    // print memory destination content
    uart_write_str(USART1, "memory dest NEW data: ");
    for (int i = 0; i < MEM_TEST_LEN; i++)
    {
        uart_write_int(USART1, mem_dest[i]);
        uart_write_str(USART1, ",");
    }
    uart_write_str(USART1, "\r\n");
}

void test_dma_interrupt()
{
    systick_init_with_millisecond();
    setup_builtin_led_for_blink();
    uint8_t led_pin_number = get_pin_number(BUILTIN_LED_PIN);

    setup_uart1_for_printing();

    // initialize `mem_src` with `0..15`
    for (int i = 0; i < MEM_TEST_LEN; i++)
    {
        mem_src[i] = i;
    }

    // print memory source content
    uart_write_str(USART1, "memory source addr: ");
    uart_write_int(USART1, mem_src);
    uart_write_str(USART1, ", data: ");
    for (int i = 0; i < MEM_TEST_LEN; i++)
    {
        uart_write_int(USART1, mem_src[i]);
        uart_write_str(USART1, ",");
    }
    uart_write_str(USART1, "\r\n");

    // print memory destination content
    uart_write_str(USART1, "memory dest addr: ");
    uart_write_int(USART1, mem_dest);
    uart_write_str(USART1, ", data: ");
    for (int i = 0; i < MEM_TEST_LEN; i++)
    {
        uart_write_int(USART1, mem_dest[i]);
        uart_write_str(USART1, ",");
    }
    uart_write_str(USART1, "\r\n");

    // setup DMA ===============

    // enable DMA clock
    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    // RM0360 10.4.3 DMA channel x configuration register (DMA_CCRx)

    // disable DMA1 first
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    DMA1_Channel1->CCR |= DMA_CCR_MEM2MEM; // memory-to-memory
    DMA1_Channel1->CCR &= ~DMA_CCR_CIRC;   // no circular/loop

    DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE;
    DMA1_Channel1->CCR |= (0 << DMA_CCR_MSIZE_Pos); // memory size: 8 bits

    DMA1_Channel1->CCR &= ~DMA_CCR_PSIZE;
    DMA1_Channel1->CCR |= (0 << DMA_CCR_PSIZE_Pos); // peripheral size: 8 bits

    DMA1_Channel1->CCR |= DMA_CCR_MINC; // enable memory increment mode
    DMA1_Channel1->CCR |= DMA_CCR_PINC; // enable peripheral increment mode

    // RM0360 10.4.3 DMA channel x configuration register (DMA_CCRx)
    //
    // Bit 4 DIR: Data transfer direction
    // This bit is set and cleared by software.
    // 0: Read from peripheral
    // 1: Read from memory
    DMA1_Channel1->CCR |= DMA_CCR_DIR; // memory -> peripheral

    DMA1_Channel1->CMAR = mem_src;  // memory address (source address)
    DMA1_Channel1->CPAR = mem_dest; // peripheral address (destination address)

    // RM0360 10.4.4 DMA channel x number of data register
    DMA1_Channel1->CNDTR = MEM_TEST_LEN; // number of data to transfer

    // RM0360 10.4.3 DMA channel x configuration register (DMA_CCRx)
    //
    // Bit 1 TCIE: Transfer complete interrupt enable
    // This bit is set and cleared by software.
    // 0: TC interrupt disabled
    // 1: TC interrupt enabled
    DMA1_Channel1->CCR |= DMA_CCR_TCIE; // enable

    // Bits 13:12 PL[1:0]: Channel priority level
    // These bits are set and cleared by software.
    // 00: Low
    // 01: Medium
    // 10: High
    // 11: Very high
    DMA1_Channel1->CCR &= ~DMA_CCR_PL;
    DMA1_Channel1->CCR |= 1 << DMA_CCR_PL_Pos; // set Channel priority level to 1

    // enable the NVIC interrupt for DMA1_Channel1
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0x03); // optional
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // enable DMA1
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    while (1)
    {
        // turn on builtin LED
        GPIOC->ODR &= ~(1 << led_pin_number); // set `0` to turn on builtin LED
        systick_delay(500);

        GPIOC->ODR |= (1 << led_pin_number); // set `1` to turn off builtin LED
        systick_delay(500);
    }
}
