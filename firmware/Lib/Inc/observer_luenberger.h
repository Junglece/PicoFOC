/**
 * @file    observer_luenberger.h
 * @brief   龙伯格观测器工厂函数
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 使用位置-速度二阶模型对编码器角度进行滤波，同时估计速度。
 * 对比差分法：速度不会因单次角度噪声跳变而剧烈波动。
 *
 * 增益选取建议（1kHz 控制环）：
 *   中等滤波：l1=0.10f,  l2=2.0f,  speed_max=10000
 *   重滤波：  l1=0.05f,  l2=0.5f,  speed_max=10000
 *   快响应：  l1=0.20f,  l2=5.0f,  speed_max=10000
 *
 * 用法：
 *   ObserverLuenberger_Config cfg = {
 *       .rate_hz   = 1000.0f,
 *       .l1        = 0.14f,
 *       .l2        = 100.0f,
 *       .speed_max = 10000.0f,
 *   };
 *   Observer_t obs = ObserverLuenberger_Create(&cfg);
 *   obs.update(obs.ctx, raw_angle);
 *   float pos = obs.get_position(obs.ctx);
 *   float vel = obs.get_velocity(obs.ctx);
 */

#ifndef __OBSERVER_LUENBERGER_H__
#define __OBSERVER_LUENBERGER_H__

#include "observer.h"

/** 龙伯格观测器配置 —— 一行 #define 对应一个预设档位 */
typedef struct {
    float rate_hz;      /**< 采样频率 (Hz)，必须与被观测系统一致 */
    float l1;           /**< 位置校正增益（决定滤波带宽，典型 0.05~0.3） */
    float l2;           /**< 速度校正增益（决定收敛速度，典型 10~500） */
    float speed_max;    /**< 速度限幅 (rpm)，防止噪声导致估计发散 */
} ObserverLuenberger_Config;

/**
 * @brief 创建龙伯格观测器实例
 * @param cfg  配置参数（拷贝，不持有指针）
 * @return Observer_t 接口结构体
 *
 * 所有参数一次设好，不需要再调 init。
 */
Observer_t ObserverLuenberger_Create(const ObserverLuenberger_Config *cfg);

/**
 * @brief 运行时调整观测器增益（无需重建实例）
 * @param obs  Create 返回的 Observer_t 指针
 * @param l1   新的位置校正增益
 * @param l2   新的速度校正增益
 *
 * 用法：
 *   Observer_t obs = ObserverLuenberger_Create(&cfg);
 *   // ... 跑了一会儿发现响应太慢，直接调：
 *   ObserverLuenberger_SetGain(&obs, 0.2f, 150.0f);
 */
void ObserverLuenberger_SetGain(Observer_t *obs, float l1, float l2);

#endif
