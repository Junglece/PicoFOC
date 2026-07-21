/**
 * @file    foc_math.h
 * @brief   FOC 纯数学变换（零硬件依赖）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 只做 float 运算，不包含任何 STM32 / HAL 头文件。
 * 可直接在 PC 仿真和芯片间移植，不用改一行代码。
 */

#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__

#include <math.h>

/* ================================================================
 *  SPWM —— 反 Clarke 变换：α-β 电压 → 三相电压
 *
 *  将 α-β 静止坐标系电压变换为三相电压并叠加 Udc/2 偏置。
 *  注：此为 SPWM（正弦波调制），非 SVPWM，两者直流利用率不同。
 * ================================================================ */

/**
 * @brief  SPWM（反 Clarke 变换）生成三相占空比
 * @param  u_alpha / u_beta : α-β 轴电压 (V)
 * @param  udc               : 母线电压 (V)
 * @param  duty_a / b / c    : [out] 三相相电压 (V)，范围 0 ~ udc
 */
void FOC_Math_InvClarke(float u_alpha, float u_beta, float udc,
                         float *duty_a, float *duty_b, float *duty_c);

/* ================================================================
 *  反 Park 变换 —— d-q 旋转坐标系 → α-β 静止坐标系
 * ================================================================ */

/**
 * @brief  反 Park 变换
 *
 * 公式：
 *   Uα = Ud·cosθ - Uq·sinθ
 *   Uβ = Uq·cosθ + Ud·sinθ
 *
 * @param  ud / uq           : d-q 轴电压
 * @param  theta_rad         : 电角度（弧度）
 * @param  u_alpha / u_beta  : [out] α-β 轴电压
 */
void FOC_Math_InvPark(float ud, float uq, float theta_rad,
                      float *u_alpha, float *u_beta);

#endif
