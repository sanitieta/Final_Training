//
// Created by xuhao on 2025/10/18.
//
#include "stm32f4xx_hal.h"
#include "can.h"
#include "motor.h"
#include <cmath>
uint8_t Motor::tx_data_1_[8] = { 0 };
uint8_t Motor::tx_data_2_[8] = { 0 };

Motor::Motor(uint8_t escid, float ratio, ControlMethod control_method):
    escid_(escid),
    ratio_(ratio),
    control_method_(control_method) {
    // 配置CAN报文头
    if (1 <= escid && escid <= 4) {
        motor_tx_header_.StdId = TX_ID_1_4;
    } else if (5 <= escid && escid <= 8) {
        motor_tx_header_.StdId = TX_ID_5_8;
    }
    motor_tx_header_.DLC = 8;
    motor_tx_header_.IDE = CAN_ID_STD;
    motor_tx_header_.RTR = CAN_RTR_DATA;
    motor_tx_header_.TransmitGlobalTime = DISABLE;
    // 配置PID参数
    spid_ = PID(0.008f, 0.0f, 0.0056f, MAX_SPEED, MAX_CURRENT, 0.05f); // 内环 速度环PID
    ppid_ = PID(200.0f, 0.015f, 220.0f, 100.0f, MAX_SPEED, 0.06f); // 外环 位置环PID
}

void Motor::can_rx_msg_callback(const uint8_t rxdata[8]) {
    // 解包编码器角度
    int16_t tmp = (rxdata[0] << 8) | rxdata[1];
    last_ecd_angle_ = ecd_angle_;
    ecd_angle_ = LinearMapping(tmp, 0, 8191, 0.0f, 360.0f);
    if (can_init_flag_) {
        last_ecd_angle_ = ecd_angle_;
        can_init_flag_ = false;
    }
    float delta_ecd_angle = ecd_angle_ - last_ecd_angle_;
    // 解决临界跳变
    if (delta_ecd_angle > 180.0f) {
        delta_ecd_angle -= 360.0f;
    } else if (delta_ecd_angle < -180.0f) {
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
    if (temp_ >= MAX_TEMP) { // 滞回比较
        overheat_flag_ = true;
    } else if (temp_ <= WARN_TEMP) {
        overheat_flag_ = false;
    }
}

void Motor::SetCurrent(float current) {
    if (current > MAX_CURRENT) {
        current = MAX_CURRENT;
    } else if (current < -MAX_CURRENT) {
        current = -MAX_CURRENT;
    }
    // 映射到电调指令范围 -16384~16384 -20A~20A
    int16_t current_cmd = static_cast<int16_t>(current * 16384.0f / 20.0f);
    int data_pos_high = 0, data_pos_low = 0;
    if (1 <= escid_ && escid_ <= 4) {
        data_pos_high = 2 * escid_ - 2;
        data_pos_low = 2 * escid_ - 1;
        tx_data_1_[data_pos_high] = current_cmd >> 8;
        tx_data_1_[data_pos_low] = current_cmd & 0xff;
        HAL_CAN_AddTxMessage(&hcan1, &motor_tx_header_, tx_data_1_, nullptr);
    } else if (5 <= escid_ && escid_ <= 8) {
        data_pos_high = 2 * (escid_ - 4) - 2;
        data_pos_low = 2 * (escid_ - 4) - 1;
        tx_data_2_[data_pos_high] = current_cmd >> 8;
        tx_data_2_[data_pos_low] = current_cmd & 0xff;
        HAL_CAN_AddTxMessage(&hcan1, &motor_tx_header_, tx_data_2_, nullptr);
    }
}

float Motor::LinearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    return static_cast<float>(in - in_min) * (out_max - out_min) / static_cast<float>(in_max - in_min) + out_min;
}

void Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    control_method_ = POSITION_SPEED;
    this->target_angle_ = target_position;
    this->feedforward_speed_ = feedforward_speed;
    this->feedforward_torque_ = feedforward_intensity;
}

void Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    this->target_speed_ = target_speed;
    this->feedforward_torque_ = feedforward_intensity;
}

void Motor::SetTorque(float torque) {
    control_method_ = TORQUE;
    this->output_torque_ = torque;
}

void Motor::Handle() {
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
    feedforward_torque_ = FeedforwardTorqueCalc(angle_);
    output = output_torque_ + feedforward_torque_;
    // 过热保护 & 停止保护
    if (overheat_flag_ || stop_flag_) {
        output = 0.0f;
    }
    // 发送控制命令
    SetCurrent(TorqueToCurrent(output));
}

float Motor::FeedforwardTorqueCalc(float current_angle) {
    const float m = 500.0f; // 负载质量 g
    const float r = 55.24f; // 负载到旋转轴距离 mm
    const float g = 9.81f; // 重力加速度 m/s²
    float angle_rad = current_angle * PI / 180.0f;
    return m * g * r * std::sin(angle_rad) / 1e6; // N·m
}

float Motor::TorqueToCurrent(float torque) {
    return torque / torque_constant_;
}