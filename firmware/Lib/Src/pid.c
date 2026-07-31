/**
 * @file    pid.c
 * @brief   PID 控制器实现（纯数学，零硬件依赖）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 改动说明（相比原版）：
 *   1. integral += error × dt（时间归一化，频率改变时 ki 不变）
 *   2. 微分项采用 Derivative-on-Measurement
 *   3. Back-calculation anti-windup：输出饱和时反算积分，
 *      使其刚好等于"P+D 补齐后的余量"，不多囤也不少囤。
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

    /* ---- 先算 P 和 D（不依赖积分，用于后续 back-calculation） ---- */
    float p_out = pid->kp * error;
    pid->differential = measure_error;
    float d_out = pid->kd * pid->differential / pid->dt;

    /* ---- 积分项（时间归一化，不受冻结） ---- */
    pid->integral += error * pid->dt;

    /* 积分限幅（安全网） */
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    /* ---- PID 输出 ---- */
    pid->output = p_out + pid->ki * pid->integral + d_out;

    /* ---- 输出限幅 + 积分 back-calculation anti-windup ----
     *
     * 不只钳位输出，还反算积分：output_max = P + ki×integral + D
     * → integral = (output_max - P - D) / ki
     *
     * 效果：
     *   P 大时 integral 自动缩到 0（P 一个人就饱和了，积分不用抢活）
     *   P 减小时 integral 自动涨起来（P 退，积分进，平滑过渡）
     *   Ki 改大改小立即可见差异——因为积分是实时反算的
     *
     * 方向约束：正饱和时积分不为负，负饱和时积分不为正
     */
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
        if (pid->ki > 1e-6f)
        {
            float i_trim = (pid->output_max - p_out - d_out) / pid->ki;
            if (i_trim < 0.0f) i_trim = 0.0f;
            if (pid->integral > i_trim) pid->integral = i_trim;
        }
    }
    else if (pid->output < -pid->output_max)
    {
        pid->output = -pid->output_max;
        if (pid->ki > 1e-6f)
        {
            float i_trim = (-pid->output_max - p_out - d_out) / pid->ki;
            if (i_trim > 0.0f) i_trim = 0.0f;
            if (pid->integral < i_trim) pid->integral = i_trim;
        }
    }

    return pid->output;
}
