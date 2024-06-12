#include "adxl345.h"
#include "i2c_lcd.h"
#include "systick.h"
#include "i2c.h"
#include "stdio.h"

#define SCALE_FACTOR 256.0  // Scale factor for ±2g range
#define GRAVITY 9.80665  // Gravity constant for converting g to m/s²

float coeff = 0.8;
	

void ADXL345_Init(uint8_t i2c) {
    // Initialize I2C if not already initialized
	  lcd_i2c_cmd(I2C_1, 0x01);
    i2c_init(i2c, I2C_FM);
	
	  delay_ms(500);
    // Debug message to indicate start of initialization
    lcd_i2c_msg(I2C_1, 1, 0, "ADXL345 Init Start");
    delay_ms(500);
    lcd_i2c_cmd(I2C_1, 0x01);

    // Set power control to measure mode
	  delay_ms(500);
    lcd_i2c_msg(I2C_1, 1, 0, "Setting Power Ctrl");
    delay_ms(500);
	  lcd_i2c_cmd(I2C_1, 0x01);
    ADXL345_WriteReg(i2c, ADXL345_POWER_CTL, 0x08);

    // Set data format to full resolution with ±2g range
	  delay_ms(500);
    lcd_i2c_msg(I2C_1, 1, 0, "Setting Data Format");
    delay_ms(1000);
    lcd_i2c_cmd(I2C_1, 0x01);
    ADXL345_WriteReg(i2c, ADXL345_DATA_FORMAT, 0x08);
    delay_ms(500);

    // Set data rate to 100 Hz
    lcd_i2c_msg(I2C_1, 1, 0, "Setting Data Rate");
    delay_ms(1000);
    lcd_i2c_cmd(I2C_1, 0x01);
    ADXL345_WriteReg(i2c, ADXL345_BW_RATE, 0x0A);
    delay_ms(1000);
}

void ADXL345_WriteReg(uint8_t i2c, uint8_t reg, uint8_t value) {
    i2c_start(i2c);
    i2c_add(i2c, ADXL345_ADDRESS << 1, 0); // Write mode
    i2c_data(i2c, reg);
    i2c_data(i2c, value);
    i2c_stop(i2c);
}

void ADXL345_ReadXYZ(uint8_t i2c, float* x, float* y, float* z, float *grav_x, float *grav_y, float *grav_z) {
    uint8_t data[6];
    int16_t raw_x, raw_y, raw_z;

    data[0] = ADXL345_ReadReg(i2c, ADXL345_DATAX0);
    data[1] = ADXL345_ReadReg(i2c, ADXL345_DATAX1);
    data[2] = ADXL345_ReadReg(i2c, ADXL345_DATAY0);
    data[3] = ADXL345_ReadReg(i2c, ADXL345_DATAY1);
    data[4] = ADXL345_ReadReg(i2c, ADXL345_DATAZ0);
    data[5] = ADXL345_ReadReg(i2c, ADXL345_DATAZ1);

    raw_x = ((int16_t)data[1] << 8) | data[0];
    raw_y = ((int16_t)data[3] << 8) | data[2];
    raw_z = ((int16_t)data[5] << 8) | data[4];

    *x = (raw_x / SCALE_FACTOR) * GRAVITY;
    *y = (raw_y / SCALE_FACTOR) * GRAVITY;
    *z = (raw_z / SCALE_FACTOR) * GRAVITY;
	
	  *grav_x = *grav_x * coeff + (1 - coeff)*(*x);
    *grav_y = *grav_y * coeff + (1 -coeff)*(*y);
    *grav_z = (*grav_z) *  coeff + (1 - coeff)*(*z);
		*x = *x - (*grav_x);
		*y = *y - (*grav_y);
		*z = *z - (*grav_z);
}

uint8_t ADXL345_ReadReg(uint8_t i2c, uint8_t reg) {
    uint8_t value;
    
    i2c_start(i2c);
    i2c_add(i2c, ADXL345_ADDRESS << 1, 0); // Write mode
    i2c_data(i2c, reg);
    i2c_start(i2c);
    i2c_add(i2c, ADXL345_ADDRESS << 1, 1); // Read mode
    value = i2c_read(i2c, 0);
    i2c_stop(i2c);
    
    return value;
}
