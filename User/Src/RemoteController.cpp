//
// Created by xuhao on 2025/10/22.
//
#include "stm32f4xx_hal.h"
#include "RemoteController.h"
#include "usart.h"
#include <string.h>

RemoteController::RemoteController(UART_HandleTypeDef* huart):
    huart_(huart) {}

void RemoteController::parseData() {
    RCData data_copy;
    // 解包数据
    data_copy.ch0 = normalize_channel(
        (static_cast<int16_t>(rx_data_[0]) | (static_cast<int16_t>(rx_data_[1]) << 8)) & 0x07FF);
    data_copy.ch1 = normalize_channel(
        ((static_cast<int16_t>(rx_data_[1]) >> 3) | (static_cast<int16_t>(rx_data_[2]) << 5)) & 0x07FF);
    data_copy.ch2 = normalize_channel(
        ((static_cast<int16_t>(rx_data_[2]) >> 6) | (static_cast<int16_t>(rx_data_[3]) << 2) | (static_cast<
            int16_t>(rx_data_[4]) << 10)) & 0x07FF);
    data_copy.ch3 = normalize_channel(
        ((static_cast<int16_t>(rx_data_[4]) >> 1) | (static_cast<int16_t>(rx_data_[5]) << 7)) & 0x07FF);
    data_copy.s1 = static_cast<Switch>(((rx_data_[5] >> 4) & 0x000C) >> 2);
    data_copy.s2 = static_cast<Switch>((rx_data_[5] >> 4) & 0x0003);
    data_copy.mouse_x = static_cast<int16_t>(rx_data_[6]) | (static_cast<int16_t>(rx_data_[7]) << 8);
    data_copy.mouse_y = static_cast<int16_t>(rx_data_[8]) | (static_cast<int16_t>(rx_data_[9]) << 8);
    data_copy.mouse_z = static_cast<int16_t>(rx_data_[10]) | (static_cast<int16_t>(rx_data_[11]) << 8);
    data_copy.mouse_press_l = rx_data_[12];
    data_copy.mouse_press_r = rx_data_[13];
    data_copy.key = static_cast<int16_t>(rx_data_[14]) | (static_cast<int16_t>(rx_data_[15]) << 8);

    osMutexAcquire(data_mutex_,osWaitForever); // 获取数据互斥锁
    data_ = data_copy;
    osMutexRelease(data_mutex_);
}

void RemoteController::taskEntry() {
    // 启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buffer_, RX_DMA_BUFFER_SIZE);
    while (true) {
        // 等待数据准备好信号量
        if (osSemaphoreAcquire(data_ready_semaphore_, osWaitForever) == osOK) {
            parseData(); // 解析数据
        }
        connectState(); // 检查连接状态
        osDelay(10);
    }
}

void RemoteController::RCInit(const osThreadAttr_t* thread_attr) {
    data_ready_semaphore_ = osSemaphoreNew(1, 0, nullptr); // 创建信号量
    data_mutex_ = osMutexNew(nullptr); // 创建互斥锁
    // 包装一下taskEntry() 以适应osThreadNew的参数要求
    auto wrapper = [](void* arg) {
        auto* self = static_cast<RemoteController*>(arg);
        self->taskEntry();
    };
    task_handle_ = osThreadNew(wrapper, this, thread_attr);

}

// 中断回调函数
void RemoteController::ITcallback(uint16_t Size) {
    if (Size == FRAME_SIZE) {
        memcpy(rx_data_, rx_dma_buffer_, FRAME_SIZE);
        last_rx_tick_ = HAL_GetTick(); // 更新最后接收时间戳
        osSemaphoreRelease(data_ready_semaphore_); // 释放信号量，通知数据已准备好
    }
    // 重新开启接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buffer_, RX_DMA_BUFFER_SIZE);
}


bool RemoteController::connectState() {
    if (HAL_GetTick() - last_rx_tick_ < TIMEOUT) {
        isConnected_ = true;
    } else {
        isConnected_ = false;
    }
    return isConnected_;
}

RemoteController::RCData RemoteController::getData() {
    RCData copy;
    osMutexAcquire(data_mutex_,osWaitForever); // 获取数据互斥锁
    copy = data_;
    osMutexRelease(data_mutex_);
    return copy;
}