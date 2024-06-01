#include "i2c.h"

Status i2c_init(uint8_t i2c, unsigned short speed_mode){
		RCC->APB2ENR  |= 1;
		if(i2c == I2C_1){
			RCC->APB1ENR |= (1 << 21);
			gpio_init(PortB, 6, OUT_AF_OD, OUT50);
			gpio_init(PortB, 7, OUT_AF_OD, OUT50);

		// set frequency // 0b1000
			I2C1->CR2 =0x8;
			I2C1->CCR = speed_mode;
			I2C1->CR1 |= 1;
		}
		else if(i2c == I2C_2){
		
			RCC->APB1ENR |= (1 << 22);
			gpio_init(PortB, 10, OUT_AF_OD, OUT50);
			gpio_init(PortB, 11, OUT_AF_OD, OUT50);
			I2C2->CR2 =0x8;
			I2C2->CCR = speed_mode;
			I2C2->CR1 |= 1;
		}
		return Success;

}

Status i2c_start(uint8_t i2c){
		if(i2c == I2C_1){
			I2C1->CR1 |= 0x100;
			while (!(I2C1->SR1 & (1 << 0))){};
		}else if(i2c == I2C_2){
			I2C2->CR1 |= 0x100;
			while (!(I2C2->SR1 & (1 << 0))){};
		}

		return Success;
}

Status i2c_add(uint8_t i2c, char address, char RW){
		volatile int tmp;

		if(i2c == I2C_1){
		I2C1->DR = (address|RW);
		while((I2C1->SR1 & (1 << 1))==0){};
		while((I2C1->SR1 & (1 << 1))){
			tmp = I2C1->SR1;
			tmp = I2C1->SR2;
			if((I2C1->SR1 & (1 << 1))==0)
			{
				break;
			}
		}
		}else if(i2c == I2C_2){
			I2C2->DR = (address|RW);
			while((I2C2->SR1 & (1 << 1))==0){};
			while((I2C2->SR1 & (1 << 1))){
				tmp = I2C2->SR1;
				tmp = I2C2->SR2;
				if((I2C2->SR1 & (1 << 1))==0){
					break;
				}
			}
		}
	return Success;
}

Status i2c_data(uint8_t i2c, char data){
	if(i2c == I2C_1){
		while((I2C1->SR1 & (1 << 7)) == 0){}
		I2C1->DR = data;
		while((I2C1->SR1 & (1 << 7)) == 0){}
	}else if(i2c == I2C_2){
		while((I2C2->SR1 & (1 << 7)) == 0){}
		I2C2->DR = data;
		while((I2C2->SR1 & (1 << 7)) == 0){}
	}
	return Success;
}

Status i2c_stop(uint8_t i2c){
	if(i2c == I2C_1){
		I2C1->CR1 |= (1 << 9);
	}else if(i2c == I2C_2){
		I2C2->CR1 |= (1 << 9);
	}
	return Success;
}

Status i2c_write(uint8_t i2c, char address, char data[]){
	uint32_t i = 0;
	i2c_start(i2c);
	i2c_add(i2c,address,0);
	while(data[i]!='\0')
		{
			i2c_data(i2c, data[i]);
			i++;
		}
	i2c_stop(i2c);
	return Success;
}

uint8_t i2c_read(uint8_t i2c, uint8_t ack) {
    uint8_t data;
    if (i2c == I2C_1) {
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        data = I2C1->DR;
        if (ack) {
            I2C1->CR1 |= I2C_CR1_ACK;
        } else {
            I2C1->CR1 &= ~I2C_CR1_ACK;
        }
    } else if (i2c == I2C_2) {
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        data = I2C2->DR;
        if (ack) {
            I2C2->CR1 |= I2C_CR1_ACK;
        } else {
            I2C2->CR1 &= ~I2C_CR1_ACK;
        }
    }
    return data;
}