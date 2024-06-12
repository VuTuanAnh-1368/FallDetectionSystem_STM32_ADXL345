#include "gpio.h"

void gpio_init(unsigned short PORT, unsigned short PIN, unsigned short CNF, unsigned short MODE){
	if(PORT == PortA){
		RCC->APB2ENR |= (1 << 2); // enable GPIOA
		if(PIN < 8){
			GPIOA->CRL &= ~(uint32_t)((0xF) << (PIN) * 4); // clear 4 bit in config register
			GPIOA->CRL |= (MODE) << (PIN * 4);
			GPIOA->CRL |=  (CNF) << (PIN * 4 + 2);
		}
		else{
			GPIOA->CRH &= ~(uint32_t)((0xF) << ((PIN- 8)) * 4); // clear 4 bit in config register

			GPIOA->CRH |= (MODE) << ((PIN - 8)* 4);
			GPIOA->CRH |=  (CNF) << ((PIN - 8) * 4 + 2);	
		}
	}
	else if(PORT == PortB){
		RCC->APB2ENR |= (1 << 3); // enable GPIOB
		if(PIN < 8 ){
			GPIOB->CRL &= ~(uint32_t)((0xF) << (PIN) * 4); // clear 4 bit in config register
			GPIOB->CRL |= (MODE) << (PIN * 4);
			GPIOB->CRL |=  (CNF) << (PIN * 4 + 2);
		}
		else{
			GPIOB->CRH &= ~(uint32_t)((0xF) << ((PIN-8)) * 4); // clear 4 bit in config register

			GPIOB->CRH |= (MODE) << ((PIN-8)* 4);
			GPIOB->CRH |=  (CNF) << ((PIN-8) * 4 + 2);	
		}
	}else if(PORT == PortC){
		RCC->APB2ENR |= (1 << 4); // enable GPIOC
		if(PIN < 8 ){
			GPIOC->CRL &= ~(uint32_t)((0xF) << (PIN) *4); // clear 4 bit in config register
			GPIOC->CRL |= (MODE) << (PIN * 4);
			GPIOC->CRL |=  (CNF) << (PIN * 4 + 2);
		}
		else{
			GPIOC->CRH &= ~(uint32_t)((0xF) << ((PIN-8)) * 4); // clear 4 bit in config register
			GPIOC->CRH |= (MODE) << ((PIN-8)* 4);
			GPIOC->CRH |=  (CNF) << ((PIN-8) * 4 + 2);	
		}
	}
}

void gpio_write(unsigned short PORT, unsigned short PIN, unsigned short PinState){
    if (PinState == GPIO_PIN_SET) {
        switch (PORT) {
            case PortA:
                GPIOA->BSRR |= (1 << PIN);
                break;
            case PortB:
                GPIOB->BSRR |= (1 << PIN);
                break;
            case PortC:
                GPIOC->BSRR |= (1 << PIN);
                break;
            default:
                break;
        }
    } else {
        switch (PORT) {
            case PortA:
                GPIOA->BSRR |= (1 << (PIN + 16));
                break;
            case PortB:
                GPIOB->BSRR |= (1 << (PIN + 16));
                break;
            case PortC:
                GPIOC->BSRR |= (1 << (PIN + 16));
                break;
            default:
                break;
        }
    }
}

uint8_t gpio_read(unsigned short PORT, unsigned short PIN){
    uint32_t pin_mask = (1 << PIN);  // Convert PIN to Bit
    switch (PORT) {
        case PortA:
            return ((GPIOA->IDR & pin_mask) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        case PortB:
            return ((GPIOB->IDR & pin_mask) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        case PortC:
            return ((GPIOC->IDR & pin_mask) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        default:
            return GPIO_PIN_RESET; 
    }
}