//
// Created by xuhao on 2025/11/5.
//

#include "../Inc/myInit.h"

#include "IMU.h"
#include "usart.h"
#include "RemoteController.h"
#include "MotorManager.h"
#include "MotorBase.h"
#include "M3508Motor.h"
#include "M6020Motor.h"

RemoteController remote_controller(&huart3);
IMU imu;
M6020Motor motor_yaw(1, 1.0f, TORQUE);
// M6020Motor motor_pitch(2, 1.0f, POSITION_SPEED);

osThreadAttr_t remote_control_attr = {
    .name = "RemoteController_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityAboveNormal,
};
osThreadAttr_t imu_task_attr = {
    .name = "IMU_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityNormal,
};
osThreadAttr_t motor_manager_attr = {
    .name = "MotorManager_Task",
    .stack_size = 2048 * 4,
    .priority = osPriorityHigh,
};

void myInit(void) {
    // remote_controller.RCInit(&remote_control_attr);
    // imu.ImuRtosInit(&imu_task_attr);

    auto& motor_manager = MotorManager::instance();
    motor_manager.MotorManagerRTOSInit(&motor_manager_attr);
    motor_manager.addMotor(&motor_yaw);
    motor_manager.RTOSInitAllMotor();

    auto& can_tx_manager = CanTxManager::instance();
    can_tx_manager.CanTxRtosInit(nullptr);
}