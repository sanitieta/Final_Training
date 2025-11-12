//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_CANTXMANAGER_H
#define FINAL_CANTXMANAGER_H
#include <stm32f4xx_hal.h>
#include <cmsis_os2.h>
#include "MotorBase.h"

class CanTxManager {
public:
    static CanTxManager& instance();
    void CanTxRtosInit(const osThreadAttr_t* attr);
    void SetMotorCurrent(uint8_t motor_id, float current_cmd, MotorType motor_type);
private:
    void taskEntry();
    void sendCanMessages();

    osMutexId_t data_mutex_ = nullptr;
    osThreadId_t CanTxThread = nullptr;

    float motor_currents_3508[8] = { 0 };
    float motor_currents_6020[8] = { 0 };

    const uint32_t TX_PERIOD_MS = 1;
    const uint32_t M3508_STDID_1_4 = 0x200;
    const uint32_t M3508_STDID_5_8 = 0x1FF;
    const uint32_t M6020_STDID_1_4 = 0X1FE;
    const uint32_t M6020_STDID_5_7 = 0X2FE;

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