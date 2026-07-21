/**
 * @file    observer_luenberger.c
 * @brief   龙伯格观测器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 二阶模型（位置 + 速度）：
 *   x1_k+1 = x1_k + Ts × x2_k + l1 × (y_k - x1_k)
 *   x2_k+1 = x2_k + l2 × (y_k - x1_k)
 *
 * 其中：
 *   x1 = 滤波后机械角度 (deg)
 *   x2 = 估计机械速度 (rpm)
 *   y  = 编码器原始读数 (deg)
 *   l1, l2 = 观测器增益
 *   Ts = 1 / rate (s)
 */

#include "observer_luenberger.h"
#include "data_process.h"

/* ================================================================
 *  上下文
 * ================================================================ */
typedef struct {
    float x1_hat;       /**< 估计位置 (deg) */
    float x2_hat;       /**< 估计速度 (rpm) */
    float l1, l2;       /**< 观测器增益 */
    float Ts;           /**< 采样周期 (s) */
    float x2_max;       /**< 速度限幅 (rpm) */
} ObserverLuenbergerCtx_t;

/* ================================================================
 *  接口函数实现
 * ================================================================ */
static void luenberger_update(void *ctx, float raw_angle_deg)
{
    ObserverLuenbergerCtx_t *c = (ObserverLuenbergerCtx_t *)ctx;

    /* 误差 = 实测 - 估计（角度取最短路径） */
    float error = ANGLE_limit(raw_angle_deg - c->x1_hat, 360.0f, -180.0f, 180.0f);

    /* 位置估计：模型预测 + 误差校正
     *
     * 注意：x2_hat 单位为 rpm，需先转为 deg/s（×6）再乘 Ts 得到角度增量：
     *   rpm × (360°/60s) = deg/s,  deg/s × Ts = deg
     */
    c->x1_hat += c->Ts * c->x2_hat * 6.0f + c->l1 * error;
    c->x1_hat  = ANGLE_limit(c->x1_hat, 360.0f, 0.0f, 360.0f);

    /* 速度估计：误差校正 */
    c->x2_hat += c->l2 * error;
    c->x2_hat  = DATA_limit(c->x2_hat, -c->x2_max, c->x2_max);
}

static float luenberger_get_position(void *ctx)
{
    return ((ObserverLuenbergerCtx_t *)ctx)->x1_hat;
}

static float luenberger_get_velocity(void *ctx)
{
    return ((ObserverLuenbergerCtx_t *)ctx)->x2_hat;
}

/* ================================================================
 *  工厂函数
 * ================================================================ */
Observer_t ObserverLuenberger_Create(const ObserverLuenberger_Config *cfg)
{
    static ObserverLuenbergerCtx_t ctx = {0};
    Observer_t obs;
    obs.update       = luenberger_update;
    obs.get_position = luenberger_get_position;
    obs.get_velocity = luenberger_get_velocity;
    obs.ctx          = &ctx;

    ctx.Ts      = 1.0f / cfg->rate_hz;
    ctx.l1      = cfg->l1;
    ctx.l2      = cfg->l2;
    ctx.x2_max  = cfg->speed_max;
    ctx.x1_hat  = 0.0f;
    ctx.x2_hat  = 0.0f;

    return obs;
}

/* ================================================================
 *  运行时调参
 *
 *  谁调我：main.c（或任何想动态改增益的地方）
 *  用途：不改 l1/l2 的情况下不需要调这个——Create 时已经设好了
 * ================================================================ */
void ObserverLuenberger_SetGain(Observer_t *obs, float l1, float l2)
{
    ObserverLuenbergerCtx_t *c = (ObserverLuenbergerCtx_t *)obs->ctx;
    c->l1 = l1;
    c->l2 = l2;
}
