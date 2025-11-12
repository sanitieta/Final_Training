//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_MOTORMANAGER_H
#define FINAL_MOTORMANAGER_H
#include <vector>
#include "MotorBase.h"

class MotorManager {
public:
    static MotorManager& instance();

    void addMotor(MotorBase* motor);
    MotorBase* getMotorById(uint8_t id);
    void MotorRtosInitAll();
    void handleAll(); // 在任务中周期性调用所有motor的handle

private:
    std::vector<MotorBase*> motors_;
};


#endif //FINAL_MOTORMANAGER_H