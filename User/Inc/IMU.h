//
// Created by xuhao on 2025/10/12.
//

#ifndef SPI_IMU_H
#define SPI_IMU_H
#include "stm32f4xx_hal.h"


class IMU {
public:
    void update(float dt);
    void get_euler_angle(float* roll, float* pitch, float* yaw);

private:
    typedef struct EulerAngle {
        float roll_ = 0, pitch_ = 0, yaw_ = 0;
        float roll_degree_ = 0, pitch_degree_ = 0, yaw_degree_ = 0;
    } EulerAngle;

    typedef struct Gyro {
        float roll_rate_ = 0, pitch_rate_ = 0, yaw_rate_ = 0;
        float last_roll_rate_ = 0, last_pitch_rate_ = 0, last_yaw_rate_ = 0;
        float roll_rate_bias_ = 0, pitch_rate_bias_ = 0, yaw_rate_bias_ = 0;
        float gyro_roll_ = 0, gyro_pitch_ = 0, gyro_yaw_ = 0;
        float gyro_range_ = 0;
        void gyro_calculate(const EulerAngle& euler, float dt);
        void gyro_calibrate(size_t calibration_count,size_t calibration_total);
    } Gyro;

    typedef struct Accel {
        float ax_ = 0, ay_ = 0, az_ = 0;
        float acc_pitch_ = 0, acc_roll_ = 0;
        float acc_range_ = 0;
        void acc_calculate();
    } Accel;

    typedef struct Compass {
        float mx_ = 0, my_ = 0, mz_ = 0; // 单位 uT
        float compass_yaw_ = 0;
        bool if_updated_ = false; // 磁力计更新标志位 200Hz
        void compass_calculate(const EulerAngle& euler);
    } Compass;

    void imu_init();
    void complement_calculate(float comp_acc_alpha_ = 0.02, float comp_compass_alpha_ = 0.01);
    void kalman_calculate();
    Accel accel_;
    Gyro gyro_;
    Compass compass_;
    EulerAngle euler_;
};

#endif //SPI_IMU_H