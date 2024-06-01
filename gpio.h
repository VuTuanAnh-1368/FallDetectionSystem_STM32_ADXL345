#ifndef GPIO_H
#define GPIO_H
#include "stm32f10x.h"

#define PortA 1
#define PortB 2
#define PortC 3
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
// mode
#define IN 0
#define OUT10 1
#define OUT2 2
#define OUT50 3
// cnf
#define IN_ANA 0
#define IN_FLOAT 1
#define IN_PUSHPULL 2
#define OUT_GP_PP 0
#define OUT_GP_OD 1
#define OUT_AF_PP 2
#define OUT_AF_OD 3

void gpio_init(unsigned short PORT, unsigned short PIN, unsigned short CNF, unsigned short MODE);
void gpio_write(unsigned short PORT, unsigned short PIN, unsigned short value);
uint8_t gpio_read(unsigned short PORT, unsigned short PIN);

#endif