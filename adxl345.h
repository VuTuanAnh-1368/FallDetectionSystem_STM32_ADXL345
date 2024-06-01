#ifndef ADXL345_H
#define ADXL345_H

#include "stm32f10x.h"
#include "i2c.h"

#define ADXL345_DEVID          0x00
#define ADXL345_POWER_CTL      0x2D
#define ADXL345_DATAX0         0x32
#define ADXL345_DATAX1         0x33
#define ADXL345_DATAY0         0x34
#define ADXL345_DATAY1         0x35
#define ADXL345_DATAZ0         0x36
#define ADXL345_DATAZ1         0x37
#define ADXL345_ADDRESS        0x53 

void ADXL345_Init(void);
void ADXL345_ReadXYZ(int16_t* x, int16_t* y, int16_t* z);

#endif 
