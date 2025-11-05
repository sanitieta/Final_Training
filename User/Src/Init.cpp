//
// Created by xuhao on 2025/11/5.
//

#include "../Inc/Init.h"
#include "usart.h"
#include "motor.h"
#include "RemoteController.h"

RemoteController remote_controller(&huart3);
Motor motor_yaw(1, 3591.0f / 187, POSITION_SPEED);
Motor motor_pitch(2, 3591.0f / 187, POSITION_SPEED);

void SystemInit(void) {
    remote_controller.init(); // 启动DMA
}
