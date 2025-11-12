//
// Created by xuhao on 2025/11/5.
//
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "can.h"
#include "RemoteController.h"
#include "MotorManager.h"
extern RemoteController remote_controller;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == &hcan1) {
        auto& motor_manager = MotorManager::instance();
        CAN_RxHeaderTypeDef message_header;
        uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &message_header, rx_data);
        if (message_header.StdId >= 0x205 && message_header.StdId <= 0x20B) {
            // 6020电机消息
            auto motor_id = static_cast<uint8_t>(message_header.StdId - 0x204);
            auto *motor = motor_manager.getMotorById(motor_id);
            motor->CanRxCallback(rx_data);
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart3) {
        remote_controller.ITcallback(Size);
    }
}
