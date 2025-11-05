#ifndef __BMI088_H
#define __BMI088_H
#include "stm32f4xx_hal.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CS1_ACCEL_GPIO_Port     GPIOA
#define CS1_ACCEL_Pin           GPIO_PIN_4
#define CS1_GYRO_GPIO_Port      GPIOB
#define CS1_GYRO_Pin            GPIO_PIN_0

#ifndef PI
#define PI 3.14159265358979323846
#endif

void bmi088_init(void);
void BMI088_ACCEL_NS_L();
void BMI088_ACCEL_NS_H();
void BMI088_GYRO_NS_L();
void BMI088_GYRO_NS_H();
void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t* rx_data, uint8_t length);
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length);
void bmi088_gyro_read_data(float* range, float* wx, float* wy, float* wz);
void bmi088_accel_read_data(float* range, float* x, float* y, float* z);
#ifdef __cplusplus
}
#endif


#endif