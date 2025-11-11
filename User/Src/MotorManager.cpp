//
// Created by xuhao on 2025/11/11.
//

#include "../Inc/MotorManager.h"

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

void MotorManager::handleAll() {
    for (auto* motor: motors_) { motor->handle(); }
}