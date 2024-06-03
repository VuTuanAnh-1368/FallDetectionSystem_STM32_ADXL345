#include <stdio.h>
#include "gpio.h"
#include "i2c_lcd.h"
#include "systick.h"
#include "stm32f10x.h"
#include "adxl345.h"


#define NORMAL 0
#define FALL 1
#define ACTIVE 1
#define STOPPED 0

void init_gpio(void) {
    // Enable clock for GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Configure PA0 and PA1 as input (SW1 and SW2)
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;  // Input with pull-up / pull-down
    GPIOA->ODR |= GPIO_ODR_ODR0;    // Pull-up

    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF1_1;  // Input with pull-up / pull-down
    GPIOA->ODR |= GPIO_ODR_ODR1;    // Pull-up

    // Configure PA5 and PA6 as output (Green LED and Red LED)
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOA->CRL |= GPIO_CRL_MODE5_1;  // Output mode, max speed 2 MHz

    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOA->CRL |= GPIO_CRL_MODE6_1;  // Output mode, max speed 2 MHz
}

int main(void) {
    systick_init();  // Initialize system tick for delays

    lcd_i2c_init(I2C_1);  // Initialize the LCD on I2C bus 1
    float x, y, z;  // Variables to hold accelerometer data
    char buffer[32];  // Buffer to hold the string for LCD display
	
    // Clear the LCD display
    lcd_i2c_cmd(I2C_1, 0x01);
    delay_ms(50);  // Ensure the display has enough time to clear
	  ADXL345_Init(I2C_2);
    // Initialize the ADXL345 accelerometer
    lcd_i2c_msg(I2C_1, 1, 0, "Init ADXL345...");
    delay_ms(1000);

    // Clear the initialization message
    lcd_i2c_cmd(I2C_1, 0x01);
    delay_ms(50);
	  float grav_x = 0;
    float grav_y = 0;
    float grav_z = 0;
    while(1) {
			  lcd_i2c_cmd(I2C_1, 0x01);
       delay_ms(50);
        // Read the accelerometer values
        ADXL345_ReadXYZ(I2C_2, &x, &y, &z,&grav_x, &grav_y, &grav_z);

        // Format the data into strings
        snprintf(buffer, sizeof(buffer), "X: %.1f m/s2", x);
        lcd_i2c_msg(I2C_1, 1, 0, buffer);

        snprintf(buffer, sizeof(buffer), "Y: %.1f", y);
        lcd_i2c_msg(I2C_1, 2, 0, buffer);

        snprintf(buffer, sizeof(buffer), "Z: %.1f", z);
        lcd_i2c_msg(I2C_1, 2, 8, buffer);

        delay_ms(500);  // Update the display every second
    }
}
