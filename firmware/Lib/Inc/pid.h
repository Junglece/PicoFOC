/**
 * @file    pid.h
 * @brief   PID 控制器（纯数学，零硬件依赖）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 特性：
 *   - Derivative-on-Measurement：微分项输入测量值变化量，抑制设定值突变导致的微分冲击
 */

#ifndef __PID_H__
#define __PID_H__

typedef struct
{
    float kp;
    float ki;
    float kd;

    float dt;               /**< 控制周期 (s) = 1 / rate */
    float output_max;       /**< 输出限幅 */
    float integral_max;     /**< 积分限幅 */

    float error;            /**< 当前误差 */
    float integral;         /**< 积分累加值 */
    float measure_error;    /**< 测量值变化量（相邻两次测量值的差值，即 Delta） */
    float differential;     /**< 微分项 */
    float output;           /**< PID 输出 */
} PID_t;

/**
 * @brief  初始化 PID 控制器
 * @param  kp / ki / kd : PID 增益
 * @param  rate         : 控制频率 (Hz)，内部转为 dt = 1 / rate
 * @param  integral_max : 积分限幅
 * @param  output_max   : 输出限幅
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float rate, float integral_max, float output_max);

/**
 * @brief  输入误差，返回 PID 计算结果（已限幅）
 * @param  error         : 设定值 - 测量值
 * @param  measure_error : 测量值的变化量（与上次测量值的差值，即 Delta）
 * @return PID 输出（已限幅）
 */
float PID_Calc(PID_t *pid, float error, float measure_error);

#endif
