#ifndef _PERIPHERIAL_HEADER
#define _PERIPHERIAL_HEADER

#include <stdint.h>
#include <stdbool.h>
#include "cmsis/stm32f030x8.h"

void systick_init(uint32_t ticks);
void systick_init_with_millisecond();
uint64_t get_expired_tick(uint64_t current_tick, uint64_t period_ticks);
bool is_tick_expired(uint64_t current_tick, uint64_t expired_tick);
void systick_delay(uint64_t period_ticks);

void uart_write_byte(USART_TypeDef *USARTx, char c);
bool uart_read_ready(USART_TypeDef *USARTx);
char uart_read_byte(USART_TypeDef *USARTx);
void uart_write_str(USART_TypeDef *USARTx, char *str);
void uart_write_int(USART_TypeDef *USARTx, int i);

void timer_delay(TIM_TypeDef *TIMx, uint32_t prescaler, uint32_t ticks);

void start_timer(TIM_TypeDef *TIMx, uint16_t ms);
void stop_timer(TIM_TypeDef *TIMx);

void i2c_start();
void i2c_stop();
void i2c_write_byte(uint8_t data);
uint8_t i2c_read_byte();

#endif