//
// Created by xuhao on 2025/11/11.
//

#include "CanTxManager.h"
#include "can.h"
#include "cmsis_os.h"

CanTxManager& CanTxManager::instance() {
    static CanTxManager instance;
    return instance;
}

void CanTxManager::CanTxRtosInit(const osThreadAttr_t* attr) {
    data_mutex_ = osMutexNew(nullptr);
    auto wrapper = [](void* arg) {
        auto* self = static_cast<CanTxManager*>(arg);
        self->taskEntry();
    };
    CanTxThread = osThreadNew(wrapper, this, attr);
}

void CanTxManager::SetMotorCurrent(uint8_t motor_id, int16_t current_cmd, MotorType motor_type) {
    if (motor_type == MotorType::M3508) {
        if (motor_id >= 1 && motor_id <= 8) {
            osMutexAcquire(data_mutex_, osWaitForever);
            motor_currents_3508[motor_id - 1] = current_cmd;
            osMutexRelease(data_mutex_);
        }
    } else if (motor_type == MotorType::M6020) {
        if (motor_id >= 1 && motor_id <= 8) {
            osMutexAcquire(data_mutex_, osWaitForever);
            motor_currents_6020[motor_id - 1] = current_cmd;
            osMutexRelease(data_mutex_);
        }
    }
}

void CanTxManager::taskEntry() {
    while (true) {
        sendCanMessages();
        osDelay(TX_PERIOD_MS);
    }
}

void CanTxManager::sendCanMessages() {
    uint8_t tx_data_1_4_3508[8]{};
    uint8_t tx_data_5_8_3508[8]{};
    uint8_t tx_data_1_4_6020[8]{};
    uint8_t tx_data_5_8_6020[8]{};
    osMutexAcquire(data_mutex_, osWaitForever);
    // 打包1-4号电机数据
    for (size_t i = 1; i <= 4; i++) {
        auto current_3508 = motor_currents_3508[i - 1];
        auto current_6020 = motor_currents_6020[i - 1];
        auto data_pos_high = 2 * i - 2;
        auto data_pos_low = 2 * i - 1;
        // 限幅
        if (current_3508 > 16384) { current_3508 = 16384; }
        if (current_3508 < -16384) { current_3508 = -16384; }
        if (current_6020 > 16384) { current_6020 = 16384; }
        if (current_6020 < -16384) { current_6020 = -16384; }

        tx_data_1_4_3508[data_pos_high] = current_3508 >> 8;
        tx_data_1_4_3508[data_pos_low] = current_3508 & 0xff;
        tx_data_1_4_6020[data_pos_high] = current_6020 >> 8;
        tx_data_1_4_6020[data_pos_low] = current_6020 & 0xff;
    }
    // 打包5-8号电机数据
    for (size_t i = 5; i <= 8; i++) {
        auto current_3508 = motor_currents_3508[i - 1];
        auto current_6020 = motor_currents_6020[i - 1];
        auto data_pos_high = 2 * (i - 4) - 2;
        auto data_pos_low = 2 * (i - 4) - 1;

        if (current_3508 > 16384) { current_3508 = 16384; }
        if (current_3508 < -16384) { current_3508 = -16384; }
        if (current_6020 > 16384) { current_6020 = 16384; }
        if (current_6020 < -16384) { current_6020 = -16384; }

        tx_data_5_8_3508[data_pos_high] = current_3508 >> 8;
        tx_data_5_8_3508[data_pos_low] = current_3508 & 0xff;
        tx_data_5_8_6020[data_pos_high] = current_6020 >> 8;
        tx_data_5_8_6020[data_pos_low] = current_6020 & 0xff;
    }
    osMutexRelease(data_mutex_);
    // 先不发5-8号电机数据
    header_3508.StdId = M3508_STDID_1_4;
    header_6020.StdId = M6020_STDID_1_4;
    HAL_CAN_AddTxMessage(&hcan1, &header_3508, tx_data_1_4_3508, nullptr);
    HAL_CAN_AddTxMessage(&hcan1, &header_6020, tx_data_1_4_6020, nullptr);
    // header_3508.StdId = 0x1FF;
    // header_6020.StdId = 0x2FE;
    // HAL_CAN_AddTxMessage(&hcan1, &header_3508, tx_data_5_8_3508, nullptr);
    // HAL_CAN_AddTxMessage(&hcan1, &header_6020, tx_data_5_8_6020, nullptr);
}