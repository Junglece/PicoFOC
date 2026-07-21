/**
 * @file    observer_diff.c
 * @brief   差分法观测器实现 —— 角度直通，速度差分
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 算法（与旧 FOC_UpdateSensor 完全一致）：
 *   position = raw_angle          （不做滤波，直通）
 *   velocity = angle_delta × rate × 60 / 360
 *
 * 不依赖任何硬件，纯数学运算。
 */

#include "observer_diff.h"
#include "data_process.h"

/* ================================================================
 *  上下文
 * ================================================================ */
typedef struct {
    float angle;        /**< 当前角度 (deg)，直接来自原始读数 */
    float last_angle;   /**< 上一周期角度 (deg) */
    float rate;         /**< 控制频率 (Hz) */
} ObserverDiffCtx_t;

/* ================================================================
 *  接口函数实现
 * ================================================================ */
static void diff_update(void *ctx, float raw_angle_deg)
{
    ObserverDiffCtx_t *c = (ObserverDiffCtx_t *)ctx;
    c->last_angle = c->angle;
    c->angle      = raw_angle_deg;   /* 直通，不做滤波 */
}

static float diff_get_position(void *ctx)
{
    return ((ObserverDiffCtx_t *)ctx)->angle;
}

static float diff_get_velocity(void *ctx)
{
    ObserverDiffCtx_t *c = (ObserverDiffCtx_t *)ctx;
    float delta = ANGLE_limit(c->angle - c->last_angle, 360.0f, -180.0f, 180.0f);
    return delta * c->rate * 60.0f / 360.0f;
}

/* ================================================================
 *  工厂函数
 * ================================================================ */
Observer_t ObserverDiff_Create(float rate_hz)
{
    static ObserverDiffCtx_t ctx = {0};
    Observer_t obs;
    obs.update       = diff_update;
    obs.get_position = diff_get_position;
    obs.get_velocity = diff_get_velocity;
    obs.ctx          = &ctx;

    ctx.rate = rate_hz;
    ctx.angle = 0.0f;
    ctx.last_angle = 0.0f;

    return obs;
}
