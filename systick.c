#include "systick.h"

volatile uint32_t tick_count = 0;

void SysTick_Handler(){
	tick_count ++;
	if(tick_count >= 4000000000){
		tick_count = 0;
	}
}

uint32_t get_tick(){
	return tick_count;
}

void systick_init(void) {
    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = SystemCoreClock / 1000000-1; 
    NVIC_SetPriority(SysTick_IRQn, 15);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);
    __enable_irq();
}

void delay_us(unsigned long t)
{
	uint32_t current = get_tick();
	while((get_tick - current) < t){}
}

void delay_ms(unsigned long t)
{
 uint32_t current = get_tick();
 while((get_tick() - current) < (t * 1000)){}
 for(int i = 0; i < t; i++){
	 delay_us(1000);
 }
}
