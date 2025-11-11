//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_CANTXMANAGER_H
#define FINAL_CANTXMANAGER_H
#include <stm32f4xx_hal.h>
#include <cmsis_os2.h>

enum class MotorType {
    M3508,
    M6020,
};


class CanTxManager {
public:
    static CanTxManager& instance();
    void CanTxInit(const osThreadAttr_t* attr);
    void SetMotorCurrent(uint8_t motor_id, float current_cmd, MotorType motor_type);

private:
    void task_entry();
    void sendCanMessages();

    osMutexId_t data_mutex_ = nullptr;
    osThreadId_t CanTxThread = nullptr;

    float motor_currents_3508[8] = { 0 };
    float motor_currents_6020[8] = { 0 };

    static constexpr uint32_t TX_PERIOD_MS = 1;

    CAN_TxHeaderTypeDef header_3508 = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE,
    };
    CAN_TxHeaderTypeDef header_6020 = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE,
    };
};

#endif //FINAL_CANTXMANAGER_H