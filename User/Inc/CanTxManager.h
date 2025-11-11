//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_CANTXMANAGER_H
#define FINAL_CANTXMANAGER_H
#include <stm32f4xx_hal.h>
#include <cmsis_os2.h>
#include <utility>
enum class MotorType {
    M3508,
    M6020,
};


class CanTxManager {
public:
    static CanTxManager& instance();
    void CanTxInit(const osThreadAttr_t* attr);
    void SetMotorCurrent(uint8_t motor_id, int16_t current_cmd, MotorType motor_type);

private:
    void task_entry();
    void sendCanMessages();

    osMutexId_t data_mutex_ = nullptr;
    osThreadId_t CanTxThread = nullptr;

    std::pair<MotorType, int16_t> motor_type_[8]{};

    static constexpr  uint32_t TX_PERIOD_MS = 2;

    CAN_TxHeaderTypeDef header_3508{};
    CAN_TxHeaderTypeDef header_6020{};
};

#endif //FINAL_CANTXMANAGER_H