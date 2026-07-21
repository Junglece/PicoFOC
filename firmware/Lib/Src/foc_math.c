/**
 * @file    foc_math.c
 * @brief   FOC 纯数学变换实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 本文件对应 FOC_Out() 中的坐标变换部分，拆出以与硬件解耦。
 */

#include "foc_math.h"

/**
 * @brief  SPWM（反 Clarke 变换）→ 三相占空比
 *
 * 公式：Clarke 反变换 + 中线偏移
 *   Ua = Uα + Udc/2
 *   Ub = (√3·Uβ - Uα) / 2 + Udc/2
 *   Uc = (-Uα - √3·Uβ) / 2 + Udc/2
 *
 * 注：此为 SPWM（正弦波调制），直流利用率约 86.6%；
 *     若需更高直流利用率（~100%），应使用 SVPWM 并注入 3 次谐波分量。
 */
void FOC_Math_InvClarke(float u_alpha, float u_beta, float udc,
                         float *duty_a, float *duty_b, float *duty_c)
{
    float half_udc = udc * 0.5f;

    *duty_a = u_alpha + half_udc;
    *duty_b = (1.7320508f * u_beta - u_alpha) * 0.5f + half_udc;
    *duty_c = (-u_alpha - 1.7320508f * u_beta) * 0.5f + half_udc;
}

/**
 * @brief  反 Park 变换：d-q 旋转坐标系 → α-β 静止坐标系
 *
 * 公式：
 *   Uα = Ud·cosθ - Uq·sinθ
 *   Uβ = Uq·cosθ + Ud·sinθ
 */
void FOC_Math_InvPark(float ud, float uq, float theta_rad,
                      float *u_alpha, float *u_beta)
{
    float cos_th = cosf(theta_rad);
    float sin_th = sinf(theta_rad);

    *u_alpha = ud * cos_th - uq * sin_th;
    *u_beta  = uq * cos_th + ud * sin_th;
}
