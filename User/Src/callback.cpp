//
// Created by xuhao on 2025/11/5.
//
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "can.h"
#include "RemoteController.h"

extern RemoteController remote_controller;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == &hcan1) {}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart3) {
        remote_controller.ITcallback(Size);
    }
}