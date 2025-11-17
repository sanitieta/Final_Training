//
// Created by xuhao on 2025/11/11.
//
#include <cmsis_os2.h>
#include "M3508Motor.h"
#include <cstring>
#include "CanTxManager.h"
#include "pid.h"

M3508Motor::M3508Motor(uint8_t escid, float ratio, ControlMethod control_method):
    escid_(escid),
    ratio_(ratio),
    control_method_(control_method) {
    // 配置PID参数
    spid_ = PID(0.0f, 0.0f, 0.0f, MAX_SPEED, MAX_CURRENT, 0.0f); // 内环 速度环PID
    ppid_ = PID(0.0f, 0.0f, 0.0f, 100.0f, MAX_SPEED, 0.0f); // 外环 位置环PID
}

void M3508Motor::CanRxCallback(const uint8_t rxdata[8]) {
    uint8_t msg_copy[8];
    memcpy(msg_copy, rxdata, 8);
    osMessageQueuePut(rx_queue_, rxdata, 0, 0);
}

void M3508Motor::handle() {
    /* 1. 解包 */
    ProcessRxQueue();
    /* 2. 计算控制量 */
    float output = ComputeOutput();
    // 发送控制命令
    EnqueueCurrentCommand(TorqueToCurrent(output));
}

void M3508Motor::stop() {
    this->stop_flag_ = true;
    EnqueueCurrentCommand(0.0f);
}

void M3508Motor::start() {
    this->stop_flag_ = false;
}

void M3508Motor::set_spid(float p, float i, float d, float d_filter) {
    spid_.kp_ = p;
    spid_.ki_ = i;
    spid_.kd_ = d;
    spid_.d_filter_k_ = d_filter;
}

void M3508Motor::set_ppid(float p, float i, float d, float d_filter) {
    ppid_.kp_ = p;
    ppid_.ki_ = i;
    ppid_.kd_ = d;
    ppid_.d_filter_k_ = d_filter;
}

void M3508Motor::ProcessRxQueue() {
    uint8_t msg[8]{ 0 };
    while (osMessageQueueGet(rx_queue_, msg, nullptr, 0) == osOK) {
        ParseRxData(msg);
    }
}

void M3508Motor::ParseRxData(const uint8_t rxdata[8]) {
    if (osMutexAcquire(data_mutex_, 0) != osOK) { return; } // 获取数据互斥锁失败，直接返回

    int16_t tmp = (rxdata[0] << 8) | rxdata[1];
    last_ecd_angle_ = ecd_angle_;
    ecd_angle_ = LinearMapping(tmp, 0, 8191, 0.0f, 360.0f);
    if (can_init_flag_) {
        last_ecd_angle_ = ecd_angle_;
        can_init_flag_ = false;
    }
    float delta_ecd_angle = ecd_angle_ - last_ecd_angle_;
    // 解决临界跳变
    while (delta_ecd_angle > 180.0f) {
        delta_ecd_angle -= 360.0f;
    }
    while (delta_ecd_angle < -180.0f) {
        delta_ecd_angle += 360.0f;
    }
    delta_angle_ = delta_ecd_angle / ratio_;
    angle_ += delta_angle_;
    // 解包转速
    tmp = (rxdata[2] << 8) | rxdata[3];
    rotate_speed_ = static_cast<float>(tmp);
    // 解包电流
    tmp = (rxdata[4] << 8) | rxdata[5];
    current_ = LinearMapping(tmp, -16384, 16384, -20.0f, 20.0f);
    // 解包温度，并检查是否过热
    temp_ = static_cast<float>(rxdata[6]);
    // 滞回比较
    if (temp_ >= MAX_TEMP) { overheat_flag_ = true; } else
        if (temp_ <= WARN_TEMP) { overheat_flag_ = false; }

    osMutexRelease(data_mutex_);
}

float M3508Motor::ComputeOutput() {
    if (osMutexAcquire(data_mutex_, 10) != osOK) {
        return 0.0f; // 获取数据互斥锁失败，返回0
    }

    float output = 0.0f;
    if (prev_method_ != control_method_) {
        // 切换控制模式时重置PID状态
        spid_.reset();
        ppid_.reset();
        prev_method_ = control_method_;
    }
    // 根据控制模式计算输出力矩
    if (control_method_ == TORQUE) {
        // 直接控制输出力矩 无需PID
    } else if (control_method_ == SPEED) {
        fdb_speed_ = rotate_speed_;
        // 内环PID 速度误差 -> 输出力矩
        output_torque_ = spid_.calc(target_speed_, fdb_speed_);
    } else if (control_method_ == POSITION_SPEED) {
        fdb_angle_ = angle_;
        fdb_speed_ = rotate_speed_;
        // 外环PID 角度误差 -> 目标速度
        target_speed_ = ppid_.calc(target_angle_, fdb_angle_);
        target_speed_ += feedforward_speed_; // 加上前馈速度
        // 内环PID 速度误差 -> 输出力矩
        output_torque_ = spid_.calc(target_speed_, fdb_speed_);
    }
    // 根据角度更新前馈力矩
    output = output_torque_ + feedforward_torque_;
    osMutexRelease(data_mutex_);
    // 过热保护 & 停止保护
    if (overheat_flag_ || stop_flag_) {
        output = 0.0f;
    }
    return output;
}

void M3508Motor::EnqueueCurrentCommand(float current_cmd) {
    if (current_cmd > MAX_CURRENT) {
        current_cmd = MAX_CURRENT;
    } else if (current_cmd < -MAX_CURRENT) {
        current_cmd = -MAX_CURRENT;
    }
    auto& can_manager = CanTxManager::instance();
    auto intensity = static_cast<int16_t>(current_cmd * 16384.0f / MAX_CURRENT);
    // intensity = -intensity; // 6020电机电流方向与命令相反 给正电流指令逆时针旋转
    can_manager.SetMotorCurrent(escid_, intensity, MotorType::M3508);
}

void M3508Motor::setTorque(float torque) {
    if (osMutexAcquire(data_mutex_, osWaitForever) == osOK) {
        control_method_ = TORQUE;
        this->output_torque_ = torque;
        osMutexRelease(data_mutex_);
    }
}

void M3508Motor::setSpeed(float speed, float ff_torque) {
    if (osMutexAcquire(data_mutex_, osWaitForever) == osOK) {
        control_method_ = SPEED;
        this->target_speed_ = speed;
        this->feedforward_speed_ = ff_torque;
        osMutexRelease(data_mutex_);
    }
}

void M3508Motor::setPosition(float target_pos, float ff_speed, float ff_torque) {
    if (osMutexAcquire(data_mutex_, osWaitForever) == osOK) {
        control_method_ = POSITION_SPEED;
        this->target_angle_ = target_pos;
        this->feedforward_speed_ = ff_speed;
        this->feedforward_torque_ = ff_torque;
        osMutexRelease(data_mutex_);
    }
}

void M3508Motor::RTOS_MotorInit() {
    rx_queue_ = osMessageQueueNew(24, 8, nullptr);
    data_mutex_ = osMutexNew(nullptr);
}

float M3508Motor::LinearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    return static_cast<float>(in - in_min) * (out_max - out_min) / static_cast<float>(in_max - in_min) + out_min;
}

float M3508Motor::TorqueToCurrent(float torque) const {
    return torque / torque_constant_;
}