//
// Created by xuhao on 2025/10/12.
//
#include "bmi088.h"
#include "IMU.h"
#include "IST8310.h"
#include <cmath>
#include <cmsis_os2.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

// @param dt: time interval in seconds
void IMU::update(float dt) {
    // 正常运行 1kHz
    if (osMutexAcquire(bmi088_data_mutex_, 10) == osOK) {
        accel_.acc_calculate();
        gyro_.gyro_calculate(this->euler_, dt);
        osMutexRelease(bmi088_data_mutex_);
    } else {
        accel_.acc_calculate();
        gyro_.gyro_calculate(this->euler_, dt);
    }
    // 磁力计频率低一些 200Hz
    compass_.compass_duty_++;
    if (compass_.compass_duty_ >= 5) {
        compass_.compass_duty_ = 0;
        if (osMutexAcquire(ist8310_data_mutex_, 10) == osOK) {
            compass_.compass_calculate(this->euler_);
            osMutexRelease(ist8310_data_mutex_);
        } else {
            compass_.compass_calculate(this->euler_);
        }
        compass_.if_updated_ = true; // 手动清除标志位1
    } else {
        compass_.if_updated_ = false; // 手动清除标志位1
    }

    // 卡尔曼滤波
    kalman_calculate();
}

// 输出接口依旧用角度，方便上层使用
void IMU::GetEulerAngle(float* roll, float* pitch, float* yaw) {
    if (osMutexAcquire(euler_mutex_, 10) == osOK) {
        *roll = euler_.roll_ * RAD2DEG;
        *pitch = euler_.pitch_ * RAD2DEG;
        *yaw = euler_.yaw_ * RAD2DEG;
        osMutexRelease(euler_mutex_);
    } else {
        *roll = euler_.roll_ * RAD2DEG;
        *pitch = euler_.pitch_ * RAD2DEG;
        *yaw = euler_.yaw_ * RAD2DEG;
    }
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

// 陀螺仪校准
void IMU::Gyro::gyro_calibrate(size_t calibration_count, size_t calibration_total) {
    float wx, wy, wz;
    bmi088_gyro_read_data(nullptr, &wx, &wy, &wz); // 读取陀螺仪数据 不用读量程
    roll_rate_bias_ += wx;
    pitch_rate_bias_ += wy;
    yaw_rate_bias_ += wz;

    if (calibration_count == calibration_total) {
        roll_rate_bias_ /= static_cast<float>(calibration_total);
        pitch_rate_bias_ /= static_cast<float>(calibration_total);
        yaw_rate_bias_ /= static_cast<float>(calibration_total);
    }
}

// 磁力计
void IMU::Compass::compass_calculate(const EulerAngle& euler) {
    IST8310ReadMagData(&mx_, &my_, &mz_);

    float roll = euler.roll_;
    float pitch = euler.pitch_;

    // 机体系磁场 -> 水平面磁场
    float mx_h = mx_ * cosf(pitch) + mz_ * sinf(pitch);
    float my_h = mx_ * sinf(roll) * sinf(pitch) + my_ * cosf(roll) - mz_ * sinf(roll) * cosf(pitch);

    // 注意：这里负号方向要根据实际旋转方向确定
    compass_yaw_ = atan2f(-my_h, mx_h); // 输出为弧度
}

// IMU任务初始化 这个函数只会被执行一次
void IMU::ImuRtosInit(const osThreadAttr_t* thread_attr) {
    bmi088_data_mutex_ = osMutexNew(nullptr); // 创建BMI088数据互斥锁
    ist8310_data_mutex_ = osMutexNew(nullptr); // 创建IST8310数据互斥锁
    euler_mutex_ = osMutexNew(nullptr); // 创建欧拉角互斥锁
    auto wapper = [](void* arg) {
        auto* self = static_cast<IMU*>(arg);
        self->taskEntry();
    }; // 包装一下taskEntry() 以适应osThreadNew的参数要求
    imu_task_handle_ = osThreadNew(wapper, this, thread_attr);
}

void IMU::ImuHardwareInit() {
    IST8310_init();
    bmi088_init();
    // 空读几次丢掉前几次不稳定的数据
    for (size_t i = 0; i < 10; i++) {
        IST8310ReadMagData(nullptr, nullptr, nullptr);
        bmi088_gyro_read_data(nullptr, nullptr, nullptr, nullptr);
        bmi088_accel_read_data(nullptr, nullptr, nullptr, nullptr);
        osDelay(2);
    }
    // 初始姿态 假设初始时静止
    accel_.acc_calculate();
    euler_.pitch_ = accel_.acc_pitch_;
    euler_.roll_ = accel_.acc_roll_;
    compass_.compass_calculate(euler_);
    euler_.yaw_ = compass_.compass_yaw_;
    // 校准陀螺仪零偏 2000次约10秒
    size_t calibration_count = 0;
    constexpr size_t CALIBRATION_TOTAL = 2000; // 2000次约10秒
    while (calibration_count < CALIBRATION_TOTAL) {
        calibration_count++;
        gyro_.gyro_calibrate(calibration_count, CALIBRATION_TOTAL);
        osDelay(2);
    }
}

void IMU::taskEntry() {
    uint32_t last_tick_ = HAL_GetTick();
    const uint32_t PERIOD_MS = 1; // 1ms周期
    ImuHardwareInit();
    while (true) {
        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick_) / 1000.0f; // 转换为秒
        if (dt <= 0.0f) {
            dt = PERIOD_MS / 1000.0f; // 最小1ms
        }
        last_tick_ = now;
        update(dt);
        osDelay(PERIOD_MS); // 1ms周期
    }
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
}

// 卡尔曼滤波
void IMU::kalman_calculate() {
    EulerAngle euler_copy;
    if (osMutexAcquire(euler_mutex_, 10) == osOK) {
        euler_copy = euler_;
        osMutexRelease(euler_mutex_);
    } else {
        euler_copy = euler_;
    }

    // 1. 预测
    P_roll += Q_roll;
    P_pitch += Q_pitch;
    P_yaw += Q_yaw;
    // 2. 更新
    // roll
    float K_roll = P_roll / (P_roll + R_roll);
    euler_copy.roll_ += K_roll * (accel_.acc_roll_ - gyro_.gyro_roll_);
    P_roll *= (1 - K_roll);
    // pitch
    float K_pitch = P_pitch / (P_pitch + R_pitch);
    euler_copy.pitch_ += K_pitch * (accel_.acc_pitch_ - gyro_.gyro_pitch_);
    P_pitch *= (1 - K_pitch);
    // yaw (磁力计更新时才使用卡尔曼滤波更新)
    if (compass_.if_updated_ == true) {
        float K_yaw = P_yaw / (P_yaw + R_yaw);
        euler_copy.yaw_ += K_yaw * (compass_.compass_yaw_ - gyro_.gyro_yaw_);
        P_yaw *= (1 - K_yaw);
    } else {
        euler_copy.yaw_ = gyro_.gyro_yaw_;
    }
    // yaw 限制在[-π, π]之间
    while (euler_copy.yaw_ >= PI) {
        euler_copy.yaw_ -= 2.0f * PI;
    }
    while (euler_copy.yaw_ < -PI) {
        euler_copy.yaw_ += 2.0f * PI;
    }
    euler_copy.roll_degree_ = euler_copy.roll_ * RAD2DEG;
    euler_copy.pitch_degree_ = euler_copy.pitch_ * RAD2DEG;
    euler_copy.yaw_degree_ = euler_copy.yaw_ * RAD2DEG;
    // 3. 写回
    if (osMutexAcquire(euler_mutex_, 10) == osOK) {
        euler_ = euler_copy;
        osMutexRelease(euler_mutex_);
    } else {
        euler_ = euler_copy;
    }
}