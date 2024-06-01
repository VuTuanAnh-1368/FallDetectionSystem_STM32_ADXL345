#ifndef I2C_H
#define I2C_H

#include "stm32f10x.h"
#include "systick.h"
#include "gpio.h"

// PB6,10 SCL
// PB7,11 SCA

//speed mode
#define I2C_FM 0x2D
#define I2C_SM 0xB4
#define ACK 0
#define NACK 1
#define I2C_1 0
#define I2C_2 1

typedef enum{Error = 0, Success = !Error} Status;
Status i2c_init(uint8_t i2c, unsigned short speed_mode);
Status i2c_add(uint8_t i2c, char address, char RW);
Status i2c_write(uint8_t i2c, char address, char data[]);
Status i2c_data(uint8_t i2c, char data);
Status i2c_start(uint8_t i2c);
Status i2c_stop(uint8_t i2c);
uint8_t i2c_read(uint8_t i2c, uint8_t ack);

#endif