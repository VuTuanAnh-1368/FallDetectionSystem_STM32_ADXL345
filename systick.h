#ifndef SYSTICK_H
#define SYSTICK_H
#include "stm32f10x.h" 

void systick_init(void);
void delay_us(unsigned long t);
uint32_t get_tick();
void delay_ms(unsigned long t);
#endif