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
    if (motor_count_ < 8) { motors_[motor_count_++] = motor; }
}

MotorBase* MotorManager::getMotorById(uint8_t id) {
    for (uint8_t i = 0; i < motor_count_; i++) {
        if (motors_[i]->getId() == id) {
            return motors_[i];
        }
    }
    return nullptr;
}

void MotorManager::RTOS_InitAllMotor() {
    for (uint8_t i = 0; i < motor_count_; i++) {
        motors_[i]->RTOS_MotorInit();
    }
}

void MotorManager::handleAll() {
    for (uint8_t i = 0; i < motor_count_; i++) {
        motors_[i]->handle();
    }
}

void MotorManager::stopAll() {
    for (uint8_t i = 0; i < motor_count_; i++) {
        motors_[i]->stop();
    }
}

void MotorManager::startAll() {
    for (uint8_t i = 0; i < motor_count_; i++) {
        motors_[i]->start();
    }
}

void MotorManager::taskEntry() {
    while (true) {
        handleAll();
        osDelay(1);
    }
}

void MotorManager::MotorManagerRTOSInit(const osThreadAttr_t* attr) {
    auto wrapper = [](void* arg) {
        auto* self = static_cast<MotorManager*>(arg);
        self->taskEntry();
    };
    MotorManagerThread = osThreadNew(wrapper, this, attr);
}

MotorBase* MotorManager::motors_[8] = { nullptr };