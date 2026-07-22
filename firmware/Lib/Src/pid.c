/**
 * @file    pid.c
 * @brief   PID 控制器实现（纯数学，零硬件依赖）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 改动说明（相比原版）：
 *   1. 微分项采用 Derivative-on-Measurement
 */

#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float rate, float integral_max, float output_max)
{
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->dt           = 1.0f / rate;
    pid->integral_max = integral_max;
    pid->output_max   = output_max;
    pid->error        = 0.0f;
    pid->integral     = 0.0f;
    pid->measure_error = 0.0f;
    pid->differential = 0.0f;
    pid->output       = 0.0f;
}

float PID_Calc(PID_t *pid, float error, float measure_error)
{
    pid->error         = error;
    pid->measure_error = measure_error;

    /* ---- 积分项 ---- */
    pid->integral += error;

    /* 积分限幅 */
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    /* ---- 微分项（Derivative-on-Measurement） ---- */
    pid->differential = measure_error;

    /* ---- PID 输出 ---- */
    pid->output = pid->kp * error
                + pid->ki * pid->integral
                + pid->kd * pid->differential;

    /* 输出限幅 */
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}
