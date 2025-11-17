//
// Created by xuhao on 2025/11/16.
//

#ifndef FINAL_MAIN_MANAGER_H
#define FINAL_MAIN_MANAGER_H
#include <cmsis_os2.h>
#include <stm32f4xx_hal.h>
#include "IMU.h"
#include "RemoteController.h"
#include "M6020Motor.h"

class MainManager {
public:
    explicit MainManager(const osThreadAttr_t* main_manager_attr);
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
    RCData rc_data_;

    float motor_yaw_target = 0.0f, motor_pitch_target = -30.0f; // 根据云台实际要求来
    void compute_target_pos();
};

#endif //FINAL_MAIN_MANAGER_H