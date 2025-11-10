//
// Created by xuhao on 2025/11/5.
//

#include "../Inc/Init.h"

#include "IMU.h"
#include "usart.h"
#include "motor.h"
#include "RemoteController.h"

RemoteController remote_controller(&huart3);
IMU imu;
Motor motor_yaw(1, 3591.0f / 187, POSITION_SPEED);
Motor motor_pitch(2, 3591.0f / 187, POSITION_SPEED);

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
osThreadAttr_t motor_task_attr = {
    .name =
}
void SystemInit(void) {
    remote_controller.RCInit(&remote_control_attr);
    imu.ImuInit(&imu_task_attr);
}