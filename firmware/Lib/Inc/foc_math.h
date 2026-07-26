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
 *  注：此为 SPWM（正弦波调制），直流利用率约 86.6%。
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
 *  SVPWM —— 注入零序分量的简化 SVPWM
 *
 *  原理：先用反 Clarke 算出三相电压（不含直流偏置），
 *  找出三者的最大值和最小值，取平均作为零序分量，
 *  每相减去零序分量后再叠加 Udc/2。
 *
 *  这个零序分量本质上是三次谐波（零序），
 *  把三相波形"压扁"到 ±Udc/2 范围内，使调制深度
 *  从 SPWM 的 1.0 提升到 SVPWM 的 1.1547（≈ 2/√3）。
 *
 *  等价于经典 SVPWM 但在数学上简化为一次 minmax 操作，
 *  不需要扇区判断和切换时间计算。
 *
 *  直流利用率：~100%（对比 SPWM 的 86.6%）
 * ================================================================ */

/**
 * @brief  SVPWM（零序分量注入法）生成三相占空比
 * @param  u_alpha / u_beta : α-β 轴电压 (V)
 * @param  udc               : 母线电压 (V)
 * @param  duty_a / b / c    : [out] 三相相电压 (V)，范围 0 ~ udc
 *
 * 输入限制：|Uα| + |Uβ| ≤ Udc 以内可线性调制。
 *   对 SVPWM，最大不失真相电压峰值为 Udc/√3（≈ 0.577·Udc），
 *   对应调制比 m = 1.1547（SPWM 的 2/√3 倍）。
 *
 *   若输入超过此值，输出会被自然限幅到 [0, udc]，
 *   但实际上 FOC_Output 中的 Uq 经 PID output_max = Umax = Udc 限制，
 *   而 Uα = -Uq·sinθ, Uβ = Uq·cosθ，故 |Uα|,|Uβ| ≤ Udc。
 *   在正弦稳态下 Uq ≤ Udc/√3 才保持线性，需外部保证。
 */
void FOC_Math_InvClarke_SVPWM(float u_alpha, float u_beta, float udc,
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

/* ================================================================
 *  Clarke 变换 —— 三相静止 → 两相静止
 *
 *  等幅值变换（Ialpha = Ia），非等功率变换。
 *  前提: Ia + Ib + Ic = 0（三相无中线系统）
 * ================================================================ */

/**
 * @brief  Clarke 变换: 三相电流 → α-β 轴电流
 *
 * 公式:
 *   Ialpha = Ia
 *   Ibeta  = (Ia + 2×Ib) / √3
 *          = (Ia + 2×Ib) / 1.7320508
 *
 *  ic 参数保留用于 Ia+Ib+Ic=0 校验，不参与计算。
 *
 * @param ia / ib / ic   : 三相电流 (A)
 * @param i_alpha / i_beta : [out] α-β 轴电流 (A)
 */
void FOC_Math_Clarke(float ia, float ib, float ic,
                     float *i_alpha, float *i_beta);

/* ================================================================
 *  Park 变换 —— 两相静止 → 两相旋转
 *
 *  将 α-β 静止坐标系变换到与转子同步旋转的 d-q 坐标系。
 *  角度 θ 为电角度（弧度）。
 * ================================================================ */

/**
 * @brief  Park 变换: α-β 电流 → d-q 轴电流
 *
 * 公式:
 *   Id  = Ialpha×cosθ + Ibeta×sinθ
 *   Iq  = -Ialpha×sinθ + Ibeta×cosθ
 *
 * @param i_alpha / i_beta : α-β 轴电流 (A)
 * @param theta_rad        : 电角度 (弧度)
 * @param id / iq          : [out] d-q 轴电流 (A)
 */
void FOC_Math_Park(float i_alpha, float i_beta, float theta_rad,
                   float *id, float *iq);

#endif
