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
 *     若需更高直流利用率（~100%），使用 FOC_Math_InvClarke_SVPWM。
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
 * @brief  SVPWM（零序分量注入法）→ 三相占空比
 *
 * 原理：
 *   1. 先算三相原始电压（同 SPWM 的第一步，不含直流偏置）
 *   2. 找三者的 Vmax 和 Vmin
 *   3. 零序分量 Vzero = -(Vmax + Vmin) / 2
 *   4. 逐相叠加 Vzero + Udc/2 得到占空比
 *
 * 为什么有效：
 *   注入 Vzero 后，三相电压被"压"到 Udc/2 ± (Vmax-Vmin)/2 范围内，
 *   等效于用了全部 Udc 而不牺牲线性度。
 *   这就是经典 SVPWM 的简化形式（min-max injection）。
 *
 * 对比 SPWM：
 *   SPWM 直接加 Udc/2，最大不失真相电压峰值 = Udc/2
 *   SVPWM 加零序后，最大不失真相电压峰值 = Udc/√3 ≈ 0.577·Udc
 *   提升幅度：1 / 0.866 = 15.47%
 *
 * 以 12V 母线为例：
 *   SPWM 最大相电压 = 6.0V  →  等效线电压幅值 = 10.4V
 *   SVPWM 最大相电压 = 6.93V →  等效线电压幅值 = 12.0V
 */
void FOC_Math_InvClarke_SVPWM(float u_alpha, float u_beta, float udc,
                               float *duty_a, float *duty_b, float *duty_c)
{
    float half_udc = udc * 0.5f;

    /* 第 1 步：三相原始电压（不含直流偏置） */
    float va = u_alpha;
    float vb = (1.7320508f * u_beta - u_alpha) * 0.5f;
    float vc = (-u_alpha - 1.7320508f * u_beta) * 0.5f;

    /* 第 2 步：找最大值、最小值 */
    float vmax = va;
    float vmin = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;

    /* 第 3 步：零序分量 = -(Vmax + Vmin) / 2 */
    float vzero = -(vmax + vmin) * 0.5f;

    /* 第 4 步：每相 + 零序 + 直流偏置 */
    *duty_a = va + vzero + half_udc;
    *duty_b = vb + vzero + half_udc;
    *duty_c = vc + vzero + half_udc;
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

/* ================================================================
 *  Clarke 变换 —— 三相静止 → 两相静止（等幅值）
 *
 *  公式（基于 Ia+Ib+Ic=0 推导）:
 *    Ialpha = Ia
 *    Ibeta  = (Ia + 2×Ib) / √3
 *
 *  等幅值变换: Ialpha = Ia，与反 Clarke 的增益匹配。
 *  如果使用等功率变换需同时修改正反变换，此处不做。
 * ================================================================ */
#define FOC_MATH_SQRT3  (1.7320508075688772f)

void FOC_Math_Clarke(float ia, float ib, float ic,
                     float *i_alpha, float *i_beta)
{
    /* Ialpha = Ia（等幅值变换） */
    *i_alpha = ia;

    /* Ibeta = (Ia + 2×Ib) / √3
     * 根据 Ia+Ib+Ic=0 推导，不需要 Ic 参与运算 */
    *i_beta = (ia + 2.0f * ib) / FOC_MATH_SQRT3;

    /* ic 参数保留，供调用方校验 Ia+Ib+Ic=0 */
    (void)ic;
}

/* ================================================================
 *  Park 变换 —— 两相静止 → 两相旋转
 *
 *  将 α-β 静止坐标系变换到与转子同步旋转的 d-q 坐标系。
 *  角度 θ 为电角度（弧度）。
 *
 *  公式:
 *    Id =  Iα×cosθ + Iβ×sinθ
 *    Iq = -Iα×sinθ + Iβ×cosθ
 * ================================================================ */
void FOC_Math_Park(float i_alpha, float i_beta, float theta_rad,
                   float *id, float *iq)
{
    float c = cosf(theta_rad);
    float s = sinf(theta_rad);

    *id =  i_alpha * c + i_beta * s;
    *iq = -i_alpha * s + i_beta * c;
}
