//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_MOTORMANAGER_H
#define FINAL_MOTORMANAGER_H
#include "MotorBase.h"
#include <cmsis_os2.h>

class MotorManager {
public:
    static MotorManager& instance();
    void addMotor(MotorBase* motor);
    MotorBase* getMotorById(uint8_t id);
    void RTOS_InitAllMotor();
    void handleAll(); // 在任务中周期性调用所有motor的handle
    void stopAll(); // 停止所有电机
    void startAll(); // 启动所有电机
    void RTOS_MotorManagerInit(const osThreadAttr_t* attr);

private:
    static MotorBase* motors_[8];
    uint8_t motor_count_ = 0;
    osThreadId_t MotorManagerThread = nullptr;
    void taskEntry(); // 任务入口函数
};

#endif //FINAL_MOTORMANAGER_H