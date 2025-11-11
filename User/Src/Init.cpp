//
// Created by xuhao on 2025/11/5.
//

#include "../Inc/Init.h"

#include "IMU.h"
#include "usart.h"
#include "RemoteController.h"
#include "MotorManager.h"
#include "MotorBase.h"
#include "M3508Motor.h"
#include "M6020Motor.h"

RemoteController remote_controller(&huart3);
IMU imu;
M3508Motor motor_yaw(1, 3591.0f / 187, POSITION_SPEED);
// M6020Motor motor_pitch(2, 3591.0f / 187, POSITION_SPEED);

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

void MyInit(void) {
    // remote_controller.RCInit(&remote_control_attr);
    imu.ImuInit(&imu_task_attr);
}