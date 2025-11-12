//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_MOTORMANAGER_H
#define FINAL_MOTORMANAGER_H
#include "MotorBase.h"
#include "stm32_vector_static.h"
#include <cmsis_os2.h>

class MotorManager {
public:
    static MotorManager& instance();
    void addMotor(MotorBase* motor);
    MotorBase* getMotorById(uint8_t id);
    void RTOSInitAllMotor();
    void handleAll(); // 在任务中周期性调用所有motor的handle
    void MotorManagerRTOSInit(const osThreadAttr_t* attr);

private:
    Vector<MotorBase*, 8> motors_;
    osThreadId_t MotorManagerThread = nullptr;
    void taskEntry(); // 任务入口函数
};


#endif //FINAL_MOTORMANAGER_H