//
// Created by xuhao on 2025/11/16.
//

#ifndef FINAL_MAIN_MANAGER_H
#define FINAL_MAIN_MANAGER_H
#include <cmsis_os2.h>
#include <stm32f4xx_hal.h>
#include "IMU.h"
#include "MotorManager.h"
#include "RemoteController.h"
#include "M6020Motor.h"
#include "usart.h"

class main_manager {
public:
    main_manager(const osThreadAttr_t* main_manager_attr);
    RemoteController remote_controller;
    IMU imu;
    M6020Motor motor_yaw;
    M6020Motor motor_pitch;
    void sysInit(const osThreadAttr_t* imu_task_attr,
                 const osThreadAttr_t* motor_manager_attr,
                 const osThreadAttr_t* can_tx_manager_task_attr,
                 const osThreadAttr_t* remote_control_attr,
                 const CAN_FilterTypeDef* hcan1_filter);

private:
    void taskEntry();
    osThreadId_t MainManagerThread = nullptr;
};


#endif //FINAL_MAIN_MANAGER_H