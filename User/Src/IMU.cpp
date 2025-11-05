//
// Created by xuhao on 2025/10/12.
//
#include "bmi088.h"
#include "IMU.h"
#include "IST8310.h"
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

// @param dt: time interval in seconds
void IMU::update(float dt) {
    static bool if_first = true;
    static size_t calibration_count = 0;
    static const size_t CALIBRATION_TOTAL = 2000; // 2000次约10秒
    static size_t compass_duty = 0;
    // 初始化
    if (if_first) {
        imu_init();
        if_first = false;
    }
    // 校准陀螺仪零偏 2000次约10秒
    if (calibration_count < CALIBRATION_TOTAL) {
        calibration_count++;
        gyro_.gyro_calibrate(calibration_count, CALIBRATION_TOTAL);
        return;
    }
    // 正常运行
    accel_.acc_calculate();
    gyro_.gyro_calculate(this->euler_, dt);
    if (compass_duty % 5 == 0) {
        compass_.compass_calculate(this->euler_); // 磁力计频率低一些 200Hz
        compass_duty = 0;
    } else {
        compass_.if_updated_ = false; // 手动清除标志位1
    }
    compass_duty++;
    kalman_calculate();
}

// 输出接口依旧用角度，方便上层使用
void IMU::get_euler_angle(float* roll, float* pitch, float* yaw) {
    *roll = euler_.roll_degree_;
    *pitch = euler_.pitch_degree_;
    *yaw = euler_.yaw_degree_;
}

// 加速度计
void IMU::Accel::acc_calculate() {
    bmi088_accel_read_data(&acc_range_, &ax_, &ay_, &az_);
    // 输出弧度
    acc_pitch_ = atan2f(-ax_, sqrtf(ay_ * ay_ + az_ * az_));
    acc_roll_ = atan2f(ay_, az_);
}

// 陀螺仪
void IMU::Gyro::gyro_calculate(const EulerAngle& euler, float dt) {
    // 0. 更新last值
    last_roll_rate_ = roll_rate_;
    last_pitch_rate_ = pitch_rate_;
    last_yaw_rate_ = yaw_rate_;

    // 1. 获取陀螺仪数据（单位：rad/s） 减去零偏
    float imu_wx, imu_wy, imu_wz;
    bmi088_gyro_read_data(&gyro_range_, &imu_wx, &imu_wy, &imu_wz);
    imu_wx -= roll_rate_bias_;
    imu_wy -= pitch_rate_bias_;
    imu_wz -= yaw_rate_bias_;

    // 2. 坐标系变换（机体系 → 地面系）
    float roll = euler.roll_;
    float pitch = euler.pitch_;
    float R[3][3] = { 0 };
    R[0][0] = 1.0f;
    R[0][1] = sinf(roll) * tanf(pitch);
    R[0][2] = cosf(roll) * tanf(pitch);
    R[1][0] = 0.0f;
    R[1][1] = cosf(roll);
    R[1][2] = -sinf(roll);
    R[2][0] = 0.0f;
    R[2][1] = sinf(roll) / cosf(pitch);
    R[2][2] = cosf(roll) / cosf(pitch);
    roll_rate_ = R[0][0] * imu_wx + R[0][1] * imu_wy + R[0][2] * imu_wz;
    pitch_rate_ = R[1][0] * imu_wx + R[1][1] * imu_wy + R[1][2] * imu_wz;
    yaw_rate_ = R[2][0] * imu_wx + R[2][1] * imu_wy + R[2][2] * imu_wz;

    // 3. 角度积分（中值）
    gyro_pitch_ = euler.pitch_ + (last_pitch_rate_ + pitch_rate_) / 2.0f * dt;
    gyro_roll_ = euler.roll_ + (last_roll_rate_ + roll_rate_) / 2.0f * dt;
    gyro_yaw_ = euler.yaw_ + (last_yaw_rate_ + yaw_rate_) / 2.0f * dt;

    // 4. 限制yaw在[-π, π]之间
    while (gyro_yaw_ >= PI) {
        gyro_yaw_ -= 2.0f * PI;
    }
    while (gyro_yaw_ < -PI) {
        gyro_yaw_ += 2.0f * PI;
    }
}

void IMU::Gyro::gyro_calibrate(size_t calibration_count, size_t calibration_total) {
    float wx, wy, wz;
    bmi088_gyro_read_data(nullptr, &wx, &wy, &wz); // 读取陀螺仪数据 不用读量程
    roll_rate_bias_ += wx;
    pitch_rate_bias_ += wy;
    yaw_rate_bias_ += wz;

    if (calibration_count == calibration_total) {
        roll_rate_bias_ /= calibration_total;
        pitch_rate_bias_ /= calibration_total;
        yaw_rate_bias_ /= calibration_total;
    }
}

// 磁力计
void IMU::Compass::compass_calculate(const EulerAngle& euler) {
    IST8310ReadMagData(&mx_, &my_, &mz_);

    float roll = euler.roll_;
    float pitch = euler.pitch_;
    float R[2][3] = { 0 };
    R[0][0] = cosf(pitch);
    R[0][1] = sinf(roll) * sinf(pitch);
    R[0][2] = cosf(roll) * sinf(pitch);
    R[1][0] = 0.0f;
    R[1][1] = cosf(roll);
    R[1][2] = -sinf(roll);

    // 坐标转换
    float mx_h = R[0][0] * mx_ + R[0][1] * my_ + R[0][2] * mz_;
    float my_h = R[1][0] * mx_ + R[1][1] * my_ + R[1][2] * mz_;

    compass_yaw_ = atan2f(-my_h, mx_h); // 输出为弧度
    if_updated_ = true;
}

// IMU初始化
void IMU::imu_init() {
    // 空读几次丢掉前几次不稳定的数据
    for (size_t i = 0; i < 10; i++) {
        IST8310ReadMagData(nullptr, nullptr, nullptr);
        bmi088_gyro_read_data(nullptr, nullptr, nullptr, nullptr);
        bmi088_accel_read_data(nullptr, nullptr, nullptr, nullptr);
        for (volatile int j = 0; j < 5000; j++) {}
    }

    // 初始姿态 假设初始时静止且水平
    accel_.acc_calculate();
    euler_.pitch_ = accel_.acc_pitch_;
    euler_.roll_ = accel_.acc_roll_;
    compass_.compass_calculate(euler_);
    euler_.yaw_ = compass_.compass_yaw_;
}

// 互补滤波
void IMU::complement_calculate(float comp_acc_alpha_, float comp_compass_alpha_) {
    if (compass_.if_updated_ == true) {
        // 仅在磁力计更新时使用互补滤波
        euler_.yaw_ = (1.0f - comp_compass_alpha_) * gyro_.gyro_yaw_ + comp_compass_alpha_ * compass_.compass_yaw_;
    } else {
        euler_.yaw_ = gyro_.gyro_yaw_;
    }
    euler_.pitch_ = (1.0f - comp_acc_alpha_) * gyro_.gyro_pitch_ + comp_acc_alpha_ * accel_.acc_pitch_;
    euler_.roll_ = (1.0f - comp_acc_alpha_) * gyro_.gyro_roll_ + comp_acc_alpha_ * accel_.acc_roll_;

    // 限制 pitch，防止 tan(pitch) → ∞
    if (fabsf(cosf(euler_.pitch_)) < 1e-3f) {
        euler_.pitch_ = (euler_.pitch_ > 0.0f) ? (PI / 2.0f - 1e-3f) : (-PI / 2.0f + 1e-3f);
    }

    euler_.roll_degree_ = euler_.roll_ * RAD2DEG;
    euler_.pitch_degree_ = euler_.pitch_ * RAD2DEG;
    euler_.yaw_degree_ = euler_.yaw_ * RAD2DEG;
}

void IMU::kalman_calculate() {
    static float P_roll = 1e-3f, P_pitch = 1e-3f, P_yaw = 1e-3f; // 状态协方差
    const float Q_roll = 1e-5f, Q_pitch = 1e-5f, Q_yaw = 1e-5f; // 过程噪声
    const float R_roll = 0.01f, R_pitch = 0.01f, R_yaw = 0.05f; // 观测噪声
    // 1. 预测
    P_roll += Q_roll;
    P_pitch += Q_pitch;
    P_yaw += Q_yaw;
    // 2. 更新
    // roll
    float K_roll = P_roll / (P_roll + R_roll);
    euler_.roll_ += K_roll * (accel_.acc_roll_ - gyro_.gyro_roll_);
    P_roll *= (1 - K_roll);
    // pitch
    float K_pitch = P_pitch / (P_pitch + R_pitch);
    euler_.pitch_ += K_pitch * (accel_.acc_pitch_ - gyro_.gyro_pitch_);
    P_pitch *= (1 - K_pitch);
    // yaw (磁力计更新时才使用卡尔曼滤波更新)
    if (compass_.if_updated_ == true) {
        float K_yaw = P_yaw / (P_yaw + R_yaw);
        euler_.yaw_ += K_yaw * (compass_.compass_yaw_ - gyro_.gyro_yaw_);
        P_yaw *= (1 - K_yaw);
    } else {
        euler_.yaw_ = gyro_.gyro_yaw_;
    }

    euler_.roll_degree_ = euler_.roll_ * RAD2DEG;
    euler_.pitch_degree_ = euler_.pitch_ * RAD2DEG;
    euler_.yaw_degree_ = euler_.yaw_ * RAD2DEG;
}