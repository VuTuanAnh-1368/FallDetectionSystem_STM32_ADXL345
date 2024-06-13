#include <stdio.h>
#include "gpio.h"
#include "i2c_Lcd.h"
#include "systick.h"
#include "stm32f10x.h"
#include "adxl345.h"
#include "math.h"

#define STAY_STILL 1.0
#define POST_FALL_THRESHOLD 1.5 // Low acceleration for final stage (m/s^2)
#define IMPACT_THRESHOLD 15.0   // High acceleration change to indicate falling (m/s^2)
#define FALL_DURATION 1000      // Time to wait before checking for low acceleration (ms)

#define NORMAL 0
#define FALL 1
#define ACTIVE 1
#define STOPPED 0

#define GREEN_LED_PIN 5
#define RED_LED_PIN 6
#define SW1_PIN 0
#define SW2_PIN 1

volatile uint8_t system_state = STOPPED; 
volatile uint8_t fall_status = NORMAL; 
volatile uint8_t fall_stage = 1;
float prev_magnitude = 1.0;
uint32_t fall_start_time = 0;

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void init_peripherals(void);
void detect_fall(float x, float y, float z);
void control_leds(void);

int main(void) {
    systick_init(); 
    lcd_i2c_init(I2C_1); 
    ADXL345_Init(I2C_2);  
    init_peripherals();  

    float x, y, z;  // Variables to hold accelerometer data
    float grav_x = 0, grav_y = 0, grav_z = 0;  // Gravity components
    char buffer[32];  // Buffer to hold the string for LCD display

    // Clear the LCD display
    lcd_i2c_cmd(I2C_1, 0x01);
    delay_ms(50);  // Ensure the display has enough time to clear
    // Debug message to ensure the ADXL345 is initialized
    lcd_i2c_msg(I2C_1, 1, 0, "ADXL345 Initialized");
    delay_ms(500);
	  lcd_i2c_cmd(I2C_1, 0x01);
	  delay_ms(1000);

    while (1) { 
			  lcd_i2c_cmd(I2C_1, 0x01);
        if (system_state == ACTIVE) {
            ADXL345_ReadXYZ(I2C_2, &x, &y, &z, &grav_x, &grav_y, &grav_z);

            detect_fall(x, y, z);

					  lcd_i2c_cmd(I2C_1, 0x01);
            if (fall_status == FALL) {
                snprintf(buffer, sizeof(buffer), "1 - Fall Detected");
            } else {
                snprintf(buffer, sizeof(buffer), "0 - Normal");
            }
            lcd_i2c_msg(I2C_1, 1, 0, buffer);
        } else {
					  lcd_i2c_cmd(I2C_1, 0x01);
					  delay_ms(10);
            snprintf(buffer, sizeof(buffer), "Stopped");
					  lcd_i2c_msg(I2C_1, 1, 0, buffer);
        }
        control_leds();
        delay_ms(100); 
    }
}


void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        system_state = !system_state;  // Toggle system state between ACTIVE and STOPPED
        EXTI->PR = EXTI_PR_PR0;        // Clear interrupt flag
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
			  system_state = ACTIVE;
        fall_status = NORMAL;    // Reset fall detection status to NORMAL
			  fall_stage = 1;
        EXTI->PR = EXTI_PR_PR1;  // Clear interrupt flag
    }
}

void detect_fall(float x, float y, float z) {
    float magnitude = sqrt(x * x + y * y + z * z);

    if (fall_stage == 1 && fabs(magnitude - prev_magnitude) > IMPACT_THRESHOLD) { 
        fall_stage = 2; 
        fall_start_time = get_tick(); 
    } else if (fall_stage == 2 && magnitude < POST_FALL_THRESHOLD && (get_tick() - fall_start_time) >= FALL_DURATION) {
        fall_stage = 3; 
    }

    if (fall_stage == 3 && magnitude < STAY_STILL) {
        fall_status = FALL;
    }
    prev_magnitude = magnitude;
		
}

void init_peripherals(void) {
    // Enable clocks for GPIOA and AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    // Initialize GPIOs for SW1 and SW2
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1);  // Reset bits
    GPIOA->CRL |= GPIO_CRL_CNF0_1 | GPIO_CRL_CNF1_1;  // Input with pull-up / pull-down 
    GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1;      // Pull-up

    // Configure EXTI lines for SW1 and SW2
    AFIO->EXTICR[0] &= ~(AFIO_EXTICR1_EXTI0 | AFIO_EXTICR1_EXTI1);
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA | AFIO_EXTICR1_EXTI1_PA;

    EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1;
    EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1;

    // Enable NVIC for EXTI lines
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 0);

    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetPriority(EXTI1_IRQn, 1);

    // Initialize GPIOs for LEDs
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5 | GPIO_CRL_MODE6 | GPIO_CRL_CNF6);  // Reset bits
    GPIOA->CRL |= GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1 | GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1;  // Output mode, max speed 2 MHz
    GPIOA->CRL &= ~(GPIO_CRL_CNF5_0 | GPIO_CRL_CNF5_1 | GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1);  // General purpose output push-pull
}

void control_leds(void) {
    static uint32_t last_tick_green = 0;
    static uint32_t last_tick_red = 0;
    uint32_t current_tick = get_tick();

    if (system_state == ACTIVE) {
        if (current_tick - last_tick_green >= 500000) {  // Toggle every 1 second (1Hz blink)
            GPIOA->ODR ^= (1 << GREEN_LED_PIN);
            last_tick_green = current_tick;
        }
    } else {
        GPIOA->BRR = (1 << GREEN_LED_PIN);  // Turn off green LED
    }

    if (fall_status == FALL) {
			 // GPIOA->BRR = (1 << GREEN_LED_PIN); // Turn off green LED
        if (current_tick - last_tick_red >= 250000) {  // Toggle every 0.5 second (2Hz blink)
            GPIOA->ODR ^= (1 << RED_LED_PIN);
            last_tick_red = current_tick;
        }
    } else {
        GPIOA->BRR = (1 << RED_LED_PIN);  // Turn off red LED
    }
}
