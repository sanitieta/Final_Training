//
// Created by xuhao on 2025/11/11.
//

#include "MotorManager.h"
#include <cmsis_os2.h>
MotorManager& MotorManager::instance() {
    static MotorManager instance;
    return instance;
}

void MotorManager::addMotor(MotorBase* motor) {
    motors_.push_back(motor);
}

MotorBase* MotorManager::getMotorById(uint8_t id) {
    for (auto* motor: motors_) {
        if (motor->getId() == id) {
            return motor;
        }
    }
    return nullptr;
}
void MotorManager::RTOSInitAllMotor() {
    for (auto* motor: motors_) {
        motor->MotorRtosInit();
    }
}

void MotorManager::handleAll() {
    for (auto* motor: motors_) { motor->handle(); }
}

void MotorManager::taskEntry() {
    while (true) {
        handleAll();
        osDelay(1);
    }
}

void MotorManager::MotorManagerRTOSInit(const osThreadAttr_t* attr) {
    auto wrapper = [](void* arg) {
        auto *self = static_cast<MotorManager*>(arg);
        self->taskEntry();
    };
    MotorManagerThread = osThreadNew(wrapper, this, attr);
}