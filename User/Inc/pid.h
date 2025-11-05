//
// Created by xuhao on 2025/10/18.
//

#ifndef PID_PID_H
#define PID_PID_H
#include "stm32f4xx_hal.h"

class PID {
public:
    PID(void):
        PID(0, 0, 0, 0, 0) {}

    PID(float kp,
        float ki,
        float kd,
        float i_max,
        float out_max,
        float d_filter_k = 1);

    void reset(void);
    float calc(float ref, float fdb);

    float kp_, ki_, kd_, d_filter_k_;
    float i_max_, out_max_;
    float output_;

private:
    float ref_, fdb_;
    float err_, err_sum_, last_err_;
    float pout_, iout_, dout_, last_dout_;
};

#endif //PID_PID_H