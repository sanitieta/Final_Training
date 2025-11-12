//
// Created by xuhao on 2025/11/11.
//

#ifndef FINAL_M6020MOTOR_H
#define FINAL_M6020MOTOR_H

#include "CanTxManager.h"
#include "MotorBase.h"
#include "pid.h"

class M6020Motor: public MotorBase {
public:
    M6020Motor(uint8_t escid, float ratio, ControlMethod control_method);
    ~M6020Motor() override = default;

    void CanRxCallback(const uint8_t rxdata[8]) override;
    void handle() override;
    uint8_t getId() const override { return escid_; }
    MotorType getType() const override { return MotorType::M6020; }
    void setTorque(float torque) override;
    void setSpeed(float target_speed, float ff_torque) override;
    void setPosition(float target_pos, float ff_speed, float ff_torque) override;
    void MotorRtosInit() override;

private:
    void ProcessRxQueue();
    void ParseRxData(const uint8_t rxdata[8]);
    float ComputeOutput();
    void EnqueueCurrentCommand(float current_cmd);
    static float LinearMapping(int in, int in_min, int in_max, float out_min, float out_max);
    float TorqueToCurrent(float torque) const;

private:
    // 电机属性
    const uint8_t escid_; // 电调号（1-8）
    const float ratio_; // 减速比
    const float torque_constant_ = 0.741f; // 力矩常数 Nm/A 已经包括减速比 M6020

    // 电机状态
    float angle_ = 0.0f; // 累计角度（考虑减速比）
    float delta_angle_ = 0.0f; // 角度变化量
    float ecd_angle_ = 0.0f; // 原始电机轴编码器角度（0-360度）
    float last_ecd_angle_ = 0.0f; // 上一次电机轴编码器角度
    float rotate_speed_ = 0.0f; // 转速
    float current_ = 0.0f; // 电流
    float temp_ = 0.0f; // 温度
    bool overheat_flag_ = false; // 过热标志
    bool can_init_flag_ = true; // 初始化标志
    bool stop_flag_ = true; // 停止标志
    // 电机限制参数
    const float MAX_CURRENT = 2.5f; // 最大电流限制 M6020
    const float MAX_SPEED = 500.0f; // 最大转速限制
    const float MAX_TEMP = 85.0f; // 最大温度限制
    const float WARN_TEMP = 70.0f; // 警告温度限制
    // PID
    PID spid_, ppid_;
    float target_angle_ = 0.0f, fdb_angle_ = 0.0f; // 角度控制用
    float target_speed_ = 0.0f, fdb_speed_ = 0.0f, feedforward_speed_ = 0.0f; // 速度控制用
    float output_torque_ = 0.0f, feedforward_torque_ = 0.0f; // 输出力矩 注意：Torque为电机原始力矩，不带减速器
    ControlMethod control_method_;
    ControlMethod prev_method_ = TORQUE;

    // FreeRTOS
    osMessageQueueId_t rx_queue_ = nullptr;
    osMutexId_t data_mutex_ = nullptr;
};

#endif //FINAL_M6020MOTOR_H