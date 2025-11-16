//
// Created by xuhao on 2025/11/5.
//

#include "../Inc/myInit.h"
#include "IMU.h"
#include "usart.h"
#include "RemoteController.h"
#include "MotorManager.h"
#include "MotorBase.h"
#include "M6020Motor.h"
#include "can.h"
#include "../Inc/main_manager.h"

#include <cmsis_os2.h>
#include "stm32f4xx_hal_can.h"

osThreadAttr_t remote_control_attr = {
    .name = "RemoteController_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityNormal,
};
osThreadAttr_t imu_task_attr = {
    .name = "IMU_Task",
    .stack_size = 1024,
    .priority = osPriorityNormal,
};
osThreadAttr_t motor_manager_attr = {
    .name = "MotorManager_Task",
    .stack_size = 1024 * 4,
    .priority = osPriorityAboveNormal,
};
osThreadAttr_t can_tx_manager_task_attr = {
    .name = "CanTxManager_Task",
    .stack_size = 1024,
    .priority = osPriorityHigh,
};
osThreadAttr_t main_manager_attr = {
    .name = "MainManager_Task",
    .stack_size = 1024,
    .priority = osPriorityLow,
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
main_manager project_manager(&main_manager_attr);

void myInit(void) {
    project_manager.sysInit(&imu_task_attr,
                            &motor_manager_attr,
                            &can_tx_manager_task_attr,
                            &remote_control_attr,
                            &hcan1_filter);
}