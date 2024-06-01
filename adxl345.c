#include "adxl345.h"

void ADXL345_Init(void) {
    // Set Power Control to start measuring
    uint8_t data[] = {ADXL345_POWER_CTL, 0x08};
    i2c_write(I2C_2, ADXL345_ADDRESS << 1, data);
}

void ADXL345_ReadXYZ(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t data[6];

    // Read 6 bytes of data from ADXL345
    i2c_start(I2C_2);
    i2c_add(I2C_2, ADXL345_ADDRESS << 1, 0); // Write address
    i2c_data(I2C_2, ADXL345_DATAX0);
    i2c_stop(I2C_2);

    i2c_start(I2C_2);
    i2c_add(I2C_2, ADXL345_ADDRESS << 1, 1); // Read address
    for (int i = 0; i < 6; i++) {
        data[i] = i2c_read(I2C_2, i < 5);
    }
    i2c_stop(I2C_2);

    // Combine MSB and LSB for x, y, z
    *x = ((int16_t)data[1] << 8) | data[0];
    *y = ((int16_t)data[3] << 8) | data[2];
    *z = ((int16_t)data[5] << 8) | data[4];
}
