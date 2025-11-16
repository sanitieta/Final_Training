//
// Created by xuhao on 2025/11/16.
//

#include "../Inc/main_manager.h"
#include "can.h"

main_manager::main_manager(const osThreadAttr_t* main_manager_attr

):
    remote_controller(&huart3),
    motor_yaw(1, 1.0f, SPEED),
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
    motor_yaw.set_spid(0.012, 0.0005, 0.05, 0.008);
    motor_yaw.set_ppid(25, 0.08, 1400, 0.055);

    // RemoteController 初始化
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
    while (true) {
        if (osSemaphoreAcquire(ifReceived_semaphore_))
        osDelay(10);
    }
}