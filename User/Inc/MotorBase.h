//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_MOTORBASE_H
#define FINAL_MOTORBASE_H

#include "stm32f4xx_hal.h"

enum ControlMethod { TORQUE, SPEED, POSITION_SPEED, };

enum class MotorType { M3508, M6020, };

class MotorBase {
public:
    virtual ~MotorBase() = default;
    virtual void CanRxCallback(const uint8_t rxdata[8]) = 0;
    virtual void handle() = 0;
    virtual uint8_t getId() const = 0;
    virtual MotorType getType() const = 0;
    virtual void setTorque(float torque) = 0;
    virtual void setSpeed(float target_speed, float ff_intensity) = 0;
    virtual void setPosition(float target_pos, float ff_speed, float ff_intensity) = 0;
    virtual void MotorRtosInit() = 0;
};


#endif //FINAL_MOTORBASE_H