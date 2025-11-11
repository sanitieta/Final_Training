//
// Created by xuhao on 2025/10/18.
//

#ifndef PID_MOTOR_H
#define PID_MOTOR_H
#include "stm32f4xx_hal.h"
#include "pid.h"
#ifndef PI
#define PI 3.14159265358979323846f
#endif

enum ControlMethod {
    TORQUE,
    SPEED,
    POSITION_SPEED,
};

class Motor {
public:
    Motor(uint8_t escid, float ratio, ControlMethod control_method);
    void can_rx_msg_callback(const uint8_t rxdata[8]);
    //PID
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetTorque(float torque);
    void Handle();

    bool stop_flag_ = true; // 停止标志
private:
    // 电机属性
    const uint8_t escid_; // 电调号（1-8）
    const float ratio_; // 减速比
    const float torque_constant_ = 0.3f; // 力矩常数 Nm/A 已经包括减速比

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

    // CAN配置
    CAN_TxHeaderTypeDef motor_tx_header_; // CAN发送头
    static constexpr uint16_t TX_ID_1_4 = 0x200; // 1-4号电调发送标识符
    static constexpr uint16_t TX_ID_5_8 = 0x1FF; // 5-8号电调发送标识符

    // 电机限制参数
    float MAX_CURRENT = 15.0f; // 最大电流限制
    float MAX_SPEED = 1500.0f; // 最大转速限制
    float MAX_TEMP = 85.0f; // 最大温度限制
    float WARN_TEMP = 70.0f; // 警告温度限制

    // CAN发送接收数据
    static uint8_t tx_data_1_[8];
    static uint8_t tx_data_2_[8];

    // PID
    PID spid_, ppid_;
    float target_angle_ = 0.0f, fdb_angle_ = 0.0f; // 角度控制用
    float target_speed_ = 0.0f, fdb_speed_ = 0.0f, feedforward_speed_ = 0.0f; // 速度控制用
    float output_torque_ = 0.0f, feedforward_torque_ = 0.0f; // 输出力矩 注意：Torque为电机原始力矩，不带减速器
    ControlMethod control_method_;
    ControlMethod prev_method_ = TORQUE;

private:
    static float LinearMapping(int in, int in_min, int in_max, float out_min, float out_max);
    void SetCurrent(float current);
    float FeedforwardTorqueCalc(float current_angle);
    float TorqueToCurrent(float torque);
};

#endif //PID_MOTOR_H