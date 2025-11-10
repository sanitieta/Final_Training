//
// Created by xuhao on 2025/10/22.
//

#ifndef REMOTE_CONTROL_REMOTECONTROLLER_H
#define REMOTE_CONTROL_REMOTECONTROLLER_H
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

enum Switch { UP = 1, DOWN, MID };

class RemoteController {
private:
    static constexpr size_t RX_DMA_BUFFER_SIZE = 540;
    static constexpr size_t FRAME_SIZE = 18;
    static constexpr uint32_t TIMEOUT = 100; // 超时时间 ms

    uint8_t rx_dma_buffer_[RX_DMA_BUFFER_SIZE]{ 0 };
    uint8_t rx_data_[FRAME_SIZE]{ 0 };
    UART_HandleTypeDef* huart_;
    bool isConnected_ = false; // 连接状态
    uint32_t last_rx_tick_ = 0; // 上次接收数据的tick

    osSemaphoreId_t data_ready_semaphore_ = nullptr;
    osMutexId_t data_mutex_ = nullptr;
    osThreadId_t task_handle_ = nullptr;

    struct RCData {
        float ch0 = 0.0f, ch1 = 0.0f, ch2 = 0.0f, ch3 = 0.0f;
        Switch s1 = UP, s2 = UP;
        int16_t mouse_x = 0, mouse_y = 0, mouse_z = 0;
        uint8_t mouse_press_l = 0, mouse_press_r = 0;
        uint16_t key = 0;
    } data_;

    static float normalize_channel(int16_t value) {
        return (static_cast<float>(value) - 1024.0f) / 660.0f;
    }

    void parseData(); // 解包
    void taskEntry(); // 任务入口函数
public:
    void RCInit(const osThreadAttr_t* thread_attr);
    void ITcallback(uint16_t Size);
    bool connectState();
    RCData getData();
    RemoteController(UART_HandleTypeDef* huart);
};


#endif //REMOTE_CONTROL_REMOTECONTROLLER_H