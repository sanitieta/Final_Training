//
// Created by xuhao on 2025/10/15.
//

#include "../Inc/IST8310.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "IST8310reg.h"

void IST8310_init() {
    // 重启磁力计
    HAL_GPIO_WritePin(IST8310_GPIOx, IST8310_GPIOp, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(IST8310_GPIOx, IST8310_GPIOp, GPIO_PIN_SET);
    HAL_Delay(50);

    IST8310WriteSingleData(IST8310_CNTL2_ADDR, IST8310_STAT2_NONE_ALL);
    IST8310WriteSingleData(IST8310_AVGCNTL_ADDR, IST8310_AVGCNTL_FOURTH);
    IST8310WriteSingleData(IST8310_CNTL1_ADDR,IST8310_CNTL1_CONTINUE); // 200Hz
}

void IST8310WriteSingleData(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&IST8310_I2C,
                      (IST8310_I2C_ADDR << 1),
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      10);
}

void IST8310WriteData(uint8_t reg, uint8_t* data, uint8_t len) {
    HAL_I2C_Mem_Write(&IST8310_I2C,
                      (IST8310_I2C_ADDR << 1),
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      data,
                      len,
                      10);
}

void IST8310ReadSingleData(uint8_t reg, uint8_t* data) {
    HAL_I2C_Mem_Read(&IST8310_I2C,
                     IST8310_I2C_ADDR << 1,
                     reg,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     1,
                     1000);
}

void IST8310ReadData(uint8_t reg, uint8_t* val, uint8_t len) {
    HAL_I2C_Mem_Read(&IST8310_I2C,
                     IST8310_I2C_ADDR << 1,
                     reg,
                     I2C_MEMADD_SIZE_8BIT,
                     val,
                     len,
                     1000);
}

void IST8310ReadMagData(float* mag_x, float* mag_y, float* mag_z) {
    uint8_t rx_data[6]{ 0 };
    IST8310ReadData(IST8310_DATA_XL_ADDR, rx_data, 6);
    // 量程是0.3uT/LSB
    *mag_x = 0.3f * static_cast<int16_t>(rx_data[1] << 8 | rx_data[0]);
    *mag_y = 0.3f * static_cast<int16_t>(rx_data[3] << 8 | rx_data[2]);
    *mag_z = 0.3f * static_cast<int16_t>(rx_data[5] << 8 | rx_data[4]);
}