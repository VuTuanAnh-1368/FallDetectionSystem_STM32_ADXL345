#include <stdio.h>
#include "gpio.h"
#include "i2c_Lcd.h"
#include "systick.h"
#include "stm32f10x.h"
#include "adxl345.h"

int main(void) {
    systick_init();  // Initialize system tick for delays

    lcd_i2c_init(I2C_1);  // Initialize the LCD on I2C bus 1
    int16_t x, y, z;  // Variables to hold accelerometer data
    char buffer[32];  // Buffer to hold the string for LCD display

    lcd_i2c_msg(I2C_1, 1, 0, "Init ADXL345...");
    delay_ms(1000);
    // Clear the LCD display
    lcd_i2c_cmd(I2C_1, 0x01);
    delay_ms(50);  // Ensure the display has enough time to clear

    // Initialize I2C_2 for ADXL345
    i2c_init(I2C_2, 0x28); // Set the correct speed mode (0x28 is 100kHz)
    ADXL345_Init();

    // Verify ADXL345 device ID
    uint8_t dev_id = 0;
    i2c_start(I2C_2);
    i2c_add(I2C_2, ADXL345_ADDRESS << 1, 0); // Write mode
    i2c_data(I2C_2, ADXL345_DEVID);
    i2c_stop(I2C_2);
    i2c_start(I2C_2);
    i2c_add(I2C_2, ADXL345_ADDRESS << 1, 1); // Read mode
    dev_id = i2c_read(I2C_2, 0);
    i2c_stop(I2C_2);

    if (dev_id != 0xE5) {
        lcd_i2c_msg(I2C_1, 0, 0, "ADXL345 Error");
        while (1);
    } else {
        lcd_i2c_msg(I2C_1, 0, 0, "ADXL345 Ready");
    }

    delay_ms(1000);
    lcd_i2c_cmd(I2C_1, 0x01); // Clear the LCD
    delay_ms(50);

    while (1) {
        lcd_i2c_cmd(I2C_1, 0x01);
        delay_ms(50); 

        // Read acceleration data from ADXL345
        ADXL345_ReadXYZ(&x, &y, &z);

        // Format the data to be displayed on the LCD
        snprintf(buffer, sizeof(buffer), "X: %d Y: %d", x, y);
        lcd_i2c_msg(I2C_1, 0, 0, buffer);  // Display x and y on the first line

        snprintf(buffer, sizeof(buffer), "Z: %d", z);
        lcd_i2c_msg(I2C_1, 1, 0, buffer);  // Display z on the second line

        delay_ms(1000);  // Update the display every second
    }
}
