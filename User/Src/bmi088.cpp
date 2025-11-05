//
// Created by xuhao on 2025/10/12.
//
#include "bmi088.h"

#include "IMU.h"
#include <cmath>

void BMI088_ACCEL_NS_L() {
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H() {
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L() {
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H() {
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

void bmi088_write_byte(uint8_t tx_data) {
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}

void bmi088_read_byte(uint8_t* rx_data, uint8_t length) {
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

/* acc写入，相当于加上片选的 bmi088_write_reg 函数 */
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg & 0x7F); // 0b 0111 1111
    bmi088_write_byte(data);

    BMI088_ACCEL_NS_H();
}

void bmi088_accel_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length) {
    uint8_t tmp;
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    bmi088_write_byte(reg | 0x80); // 写入地址时，最高位为1，表示读操作
    bmi088_read_byte(&tmp, 1);
    bmi088_read_byte(rx_data, length);

    BMI088_ACCEL_NS_H();
} // 加速度计读取，注意需要忽略第一位数据dummy byte

/* gyro写入 */
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg & 0x7F); // 0b 0111 1111
    bmi088_write_byte(tx_data);

    BMI088_GYRO_NS_H();
}

void bmi088_gyro_read_reg(uint8_t reg, uint8_t* rx_data, uint8_t length) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();

    bmi088_write_byte(reg | 0x80); // 写入地址时，最高位为1，表示读操作 0b 1000 0000
    bmi088_read_byte(rx_data, length);

    BMI088_GYRO_NS_H();
} // 陀螺仪读取

void bmi088_gyro_read_data(float* range, float* wx, float* wy, float* wz) {
    uint8_t raw_range;
    uint8_t rx_gyro_data[6]{ 0 };
    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    switch (raw_range) {
        case 0x00:
            *range = 2000.0f;
            break;
        case 0x01:
            *range = 1000.0f;
            break;
        case 0x02:
            *range = 500.0f;
            break;
        case 0x03:
            *range = 250.0f;
            break;
        case 0x04:
            *range = 125.0f;
            break;
        default:
            *range = 0.0f;
    }
    *range = (*range) * PI / 180.0f;
    // 2. 读取acc0x12寄存器中的6位gyro数据 以弧度输出
    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
    *wx = static_cast<int16_t>(rx_gyro_data[1] << 8 | rx_gyro_data[0]) * (*range) / 32767.0f;
    *wy = static_cast<int16_t>(rx_gyro_data[3] << 8 | rx_gyro_data[2]) * (*range) / 32767.0f;
    *wz = static_cast<int16_t>(rx_gyro_data[5] << 8 | rx_gyro_data[4]) * (*range) / 32767.0f;
}

void bmi088_accel_read_data(float* range, float* x, float* y, float* z) {
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    uint8_t raw_range;
    uint8_t rx_acc_data[6]{ 0 };
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    switch (raw_range) {
        case 0x00:
            *range = 3.0f;
            break;
        case 0x01:
            *range = 6.0f;
            break;
        case 0x02:
            *range = 12.0f;
            break;
        case 0x03:
            *range = 24.0f;
            break;
        default:
            *range = 0.0f;
    }
    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);
    // 3. 用量程系数将原始数据转换为常用单位
    *x = static_cast<int16_t>(rx_acc_data[1] << 8 | rx_acc_data[0]) / 32768.0f * 1000.0f *
        pow(2, raw_range + 1) * 1.5f;
    *y = static_cast<int16_t>(rx_acc_data[3] << 8 | rx_acc_data[2]) / 32768.0f * 1000.0f *
        pow(2, raw_range + 1) * 1.5f;
    *z = static_cast<int16_t>(rx_acc_data[5] << 8 | rx_acc_data[4]) / 32768.0f * 1000.0f *
        pow(2, raw_range + 1) * 1.5f;
}

void bmi088_init(void) {
    // ACC 重置
    bmi088_accel_write_single_reg(0x7E, 0xB6);
    HAL_Delay(50); // 至少50ms

    // GYRO 重置
    bmi088_gyro_write_single_reg(0x14, 0xB6);
    HAL_Delay(30);

    bmi088_accel_read_reg(0x00, nullptr, 1);
    // 打开加速度计电源
    bmi088_accel_write_single_reg(0x7D, 0x04); // ACC_PWR_CTRL: enable accelerometer
    HAL_Delay(5);
    bmi088_accel_write_single_reg(0x7C, 0x00);
    HAL_Delay(5);
    // 检查是否进入正常模式
    uint8_t id;
    bmi088_accel_read_reg(0x02, &id, 1);
    while (id != 0x00) {}
    // 配置测量范围和带宽
    bmi088_gyro_write_single_reg(0x0F, 0x02); // ±500°/s
    bmi088_accel_write_single_reg(0x41, 0x00); // ±3g
    bmi088_accel_write_single_reg(0x40, 0x0A); // ODR = 100Hz, bandwidth = 100Hz
}