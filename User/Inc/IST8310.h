//
// Created by xuhao on 2025/10/15.
//

#ifndef SPI_IST8310_H
#define SPI_IST8310_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define IST8310_I2C_ADDR 0x0E
#define IST8310_I2C hi2c3

#define IST8310_GPIOx GPIOG
#define IST8310_GPIOp GPIO_PIN_6

void IST8310_init();
void IST8310WriteSingleData(uint8_t reg, uint8_t data);
void IST8310WriteData(uint8_t reg, uint8_t* data, uint8_t len);
void IST8310ReadSingleData(uint8_t reg, uint8_t* data);
void IST8310ReadData(uint8_t reg, uint8_t* val, uint8_t len);
void IST8310ReadMagData(float* mag_x, float* mag_y, float* mag_z);
#ifdef __cplusplus
}
#endif
#endif //SPI_IST8310_H