//
// Created by xuhao on 2025/11/5.
//

#include "../Inc/myInit.h"
#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "usart.h"
#include "RemoteController.h"
#include "MotorManager.h"
#include "MotorBase.h"
#include "M3508Motor.h"
#include "M6020Motor.h"
#include "can.h"

// volatile uint8_t tmp0 = 0;
// volatile uint8_t tmp1 = 0;
// volatile uint8_t tmp2 = 0;

RemoteController remote_controller(&huart3);
IMU imu;
M6020Motor motor_yaw(4, 1.0f, POSITION_SPEED);
// M6020Motor motor_pitch(2, 1.0f, POSITION_SPEED);

osThreadAttr_t remote_control_attr = {
    .name = "RemoteController_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityNormal,
};
osThreadAttr_t imu_task_attr = {
    .name = "IMU_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityNormal,
};
osThreadAttr_t motor_manager_attr = {
    .name = "MotorManager_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityAboveNormal,
};
osThreadAttr_t can_tx_manager_task_attr = {
    .name = "CanTxManager_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityAboveNormal,
};
CAN_FilterTypeDef hcan1_filter = {
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterActivation = ENABLE,
};

void myInit(void) {
    // remote_controller.RCInit(&remote_control_attr);
    // imu.ImuRtosInit(&imu_task_attr);
    // CAN 初始化
    HAL_CAN_ConfigFilter(&hcan1, &hcan1_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // MotorManager 初始化
    auto& motor_manager = MotorManager::instance();
    motor_manager.MotorManagerRTOSInit(&motor_manager_attr);
    motor_manager.addMotor(&motor_yaw);
    motor_manager.RTOSInitAllMotor();
    // CanTxManager 初始化
    auto& can_tx_manager = CanTxManager::instance();
    can_tx_manager.CanTxRtosInit(&can_tx_manager_task_attr);
}