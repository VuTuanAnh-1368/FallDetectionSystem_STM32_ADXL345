#ifndef I2C_LCD_H
#define I2C_LCD_H
#include "stm32f10x.h"
#include "i2c.h"

#define Slave_Address 0x4E

void add_slave(uint8_t i2c, char RW);
void trans_slave(uint8_t i2c, char data);
void lcd_i2c_cmd(uint8_t i2c, unsigned char data);
void lcd_i2c_data(uint8_t i2c, unsigned char data);
void lcd_i2c_init(uint8_t i2c);
void lcd_i2c_send(uint8_t i2c, char str[]);
void lcd_i2c_msg(uint8_t i2c,  unsigned char line_1_2, unsigned char pos_0_16, char msg[]);

#endif