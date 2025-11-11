//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_CANTXMANAGER_H
#define FINAL_CANTXMANAGER_H
#include <stm32f4xx.h>
#include <cmsis_os2.h>
enum class MotorType {
    M3508,
    M6020,
};
struct CanCurrentCMD {
    uint8_t motor_id;
    MotorType motor_type;
    int16_t current_cmd;
};

class CanTxManager {
public:
    void CanTxInit();
    static void enqueue(uint8_t motor_id, int16_t current_cmd);
    static bool dequeue(CanCurrentCMD& cmd);

private:
    static osMessageQueueId_t queue_;
};


#endif //FINAL_CANTXMANAGER_H