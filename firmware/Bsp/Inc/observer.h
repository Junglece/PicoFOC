/**
 * @file    observer.h
 * @brief   角度/速度观测器抽象接口
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 将"从原始角度算出滤波角度和速度"这个算法从 FOC 控制器中解耦出来。
 *
 * 更换算法（差分法 → 龙伯格 → 卡尔曼）时只需在 main.c 换一行工厂函数：
 *   Observer_t obs = ObserverDiff_Create(FOC_RATE_HZ);               // 差分法（默认）
 *   Observer_t obs = ObserverLuenberger_Create(&cfg);                // 龙伯格
 *   Observer_t obs = ObserverKalman_Create(...);                      // 卡尔曼（预留）
 *
 * Foc.c 完全不知道用的是哪种算法——它只跟本接口的 update/get 对话。
 */

#ifndef __OBSERVER_H__
#define __OBSERVER_H__

typedef struct Observer_t Observer_t;

struct Observer_t
{
    /**
     * @brief  每周期喂一次原始角度
     * @param  ctx            实现方的上下文
     * @param  raw_angle_deg  传感器原始机械角度 (0~360°)
     *
     * 内部会更新滤波后的位置和估计速度。
     * 必须在 get_position / get_velocity 之前调用。
     */
    void   (*update)(void *ctx, float raw_angle_deg);

    /**
     * @brief  获取滤波后的机械角度
     * @return 机械角度 (0~360°)
     *
     * 差分法：返回原始角度（不做滤波）
     * 龙伯格：返回模型估计位置（滤除噪声后）
     */
    float  (*get_position)(void *ctx);

    /**
     * @brief  获取估计机械速度
     * @return 机械速度 (rpm)
     *
     * 差分法：角度差分 × 频率 × 60/360（与旧代码一致）
     * 龙伯格：观测器内部状态量（无噪声放大）
     */
    float  (*get_velocity)(void *ctx);

    void   *ctx;   /**< 实现方的上下文结构体指针 */
};

#endif
