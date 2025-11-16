//
// Created by xuhao on 2025/11/16.
//

#include "../Inc/main_manager.h"
#include "can.h"

#include <cmath>

main_manager::main_manager(const osThreadAttr_t* main_manager_attr

):
    remote_controller(&huart3),
    motor_yaw(1, 1.0f, POSITION_SPEED),
    motor_pitch(4, 1.0f, POSITION_SPEED) {
    auto wrapper = [](void* arg) {
        auto* self = static_cast<main_manager*>(arg);
        self->taskEntry();
    };
    osThreadNew(wrapper, this, main_manager_attr);
}

void main_manager::sysInit(const osThreadAttr_t* imu_task_attr,
                           const osThreadAttr_t* motor_manager_attr,
                           const osThreadAttr_t* can_tx_manager_task_attr,
                           const osThreadAttr_t* remote_control_attr,
                           const CAN_FilterTypeDef* hcan1_filter) {
    // 电机PID参数初始化
    motor_yaw.set_spid(0.007f, 0.0f, 0.0f, 0.0f);
    motor_yaw.set_ppid(1.9f, 0.2f, 25.0f, 0.04f);
    motor_pitch.set_spid(0.003f, 0.0f, 0.0f, 0.0f);
    motor_pitch.set_ppid(8.0f, 0.2f, 15.0f, 0.05f);

    // RemoteController 初始化f
    remote_controller.RC_RTOSInit(remote_control_attr);
    // IMU 初始化
    imu.ImuRtosInit(imu_task_attr);
    // CanTxManager 初始化
    auto& can_tx_manager = CanTxManager::instance();
    can_tx_manager.CanTxRtosInit(can_tx_manager_task_attr);
    // MotorManager 初始化
    auto& motor_manager = MotorManager::instance();
    motor_manager.MotorManagerRTOSInit(motor_manager_attr);
    motor_manager.addMotor(&motor_yaw);
    motor_manager.addMotor(&motor_pitch);
    motor_manager.RTOSInitAllMotor();

    // CAN 初始化
    HAL_CAN_ConfigFilter(&hcan1, hcan1_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void main_manager::taskEntry() {
    auto& motor_manager = MotorManager::instance();
    while (true) {
        /* 获取遥控器数据  */
        if (osSemaphoreAcquire(remote_controller.ifReceived_semaphore_, 0) == osOK) {
            rc_data_ = remote_controller.getData();
            compute_target_pos();
        }
        if (rc_data_.s2 == DOWN) { motor_manager.stopAll(); } else { motor_manager.startAll(); }
        motor_yaw.setPosition(motor_yaw_target, 0, 0);
        motor_pitch.setPosition(motor_pitch_target, 0, 0);
        osDelay(10);
    }
}

void main_manager::compute_target_pos() {
    float ch2 = rc_data_.ch2; // 已经是 -1.0 ~ +1.0 的归一化值
    float ch3 = rc_data_.ch3; // 已经是 -1.0 ~ +1.0 的归一化值
    const float deadzone = 0.05f; // 5% 死区，可调
    const float max_angle_ch2 = 180.0f; // 输出的最大角度，可调
    const float max_angle_ch3 = 25.0f; // 输出的最大角度，可调
    // 死区处理
    if (fabsf(ch2) < deadzone) { motor_yaw_target = 0.0f; }
    if (fabsf(ch3) < deadzone) { motor_pitch_target = 0.0f; }

    // 去死区后的重新归一化
    if (ch2 > 0) {
        ch2 = (ch2 - deadzone) / (1.0f - deadzone);
    } else {
        ch2 = (ch2 + deadzone) / (1.0f - deadzone);
    }
    if (ch3 > 0) {
        ch3 = (ch3 - deadzone) / (1.0f - deadzone);
    } else {
        ch3 = (ch3 + deadzone) / (1.0f - deadzone);
    }
    // 限幅（理论上不需要，但保险）
    if (ch2 > 1.0f) { ch2 = 1.0f; }
    if (ch2 < -1.0f) { ch2 = -1.0f; }

    // 映射到目标角度
    motor_yaw_target = - ch2 * max_angle_ch2; // 电机定义正方向与遥控器相反
    motor_pitch_target = - ch3 * max_angle_ch3 - 30.0f; // 初始位置下调30度 并反向
}