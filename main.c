#include <stdio.h>
#include "gpio.h"
#include "i2c_lcd.h"
#include "systick.h"
#include "stm32f10x.h"
#include "adxl345.h"

int main(void) {
    systick_init();  // Initialize system tick for delays

    lcd_i2c_init(I2C_1);  // Initialize the LCD on I2C bus 1
    float x, y, z;  // Variables to hold accelerometer data
    char buffer[32];  // Buffer to hold the string for LCD display
	
    // Clear the LCD display
    lcd_i2c_cmd(I2C_1, 0x01);
    delay_ms(50);  // Ensure the display has enough time to clear
  
    // Initialize the ADXL345 accelerometer
    lcd_i2c_msg(I2C_1, 1, 0, "Init ADXL345...");
    ADXL345_Init(I2C_2);
    delay_ms(1000);

    // Debug message to ensure the ADXL345 is initialized
    lcd_i2c_msg(I2C_1, 1, 0, "ADXL345 Initialized");
    delay_ms(1000);

    // Clear the initialization message
    lcd_i2c_cmd(I2C_1, 0x01);
    delay_ms(50);

    while(1) {
        // Read the accelerometer values
        ADXL345_ReadXYZ(I2C_2, &x, &y, &z);

        // Format the data into strings
        snprintf(buffer, sizeof(buffer), "X: %.2f m/s²", x);
        lcd_i2c_msg(I2C_1, 1, 0, buffer);

        snprintf(buffer, sizeof(buffer), "Y: %.2f m/s²", y);
        lcd_i2c_msg(I2C_1, 2, 0, buffer);

        snprintf(buffer, sizeof(buffer), "Z: %.2f m/s²", z);
        lcd_i2c_msg(I2C_1, 2, 10, buffer);

        delay_ms(1000);  // Update the display every second
    }
}
