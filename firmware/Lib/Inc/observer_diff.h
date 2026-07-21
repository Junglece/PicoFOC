/**
 * @file    observer_diff.h
 * @brief   差分法观测器工厂函数
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 完全复现原来的速度计算方法（FOC_UpdateSensor 中手动差分），
 * 角度不作滤波。切换到本算法时行为与旧代码一致。
 *
 * 用法：
 *   Observer_t obs = ObserverDiff_Create(1000.0f);    // 1kHz 控制环
 *   obs.update(obs.ctx, raw_angle);
 *   float pos = obs.get_position(obs.ctx);
 *   float vel = obs.get_velocity(obs.ctx);
 */

#ifndef __OBSERVER_DIFF_H__
#define __OBSERVER_DIFF_H__

#include "observer.h"

/**
 * @brief 创建差分法观测器实例
 * @param rate_hz  采样频率 (Hz)，用于速度差分计算
 * @return Observer_t 接口结构体
 */
Observer_t ObserverDiff_Create(float rate_hz);

#endif
