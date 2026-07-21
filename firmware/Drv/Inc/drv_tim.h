/**
 * @file    drv_tim.h
 * @brief   定时器驱动（纯时基，不含业务逻辑）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 改动说明：
 *   - 原 HAL_TIM_PeriodElapsedCallback 里硬编码了 FOC 循环和测试代码，
 *     现已改为回调注册机制。
 *   - 测试变量已移至 motor_ctrl.c。
 *   - 原文件名 bsp_tim.h → drv_tim.h（正名：本文件属于驱动层，非板级支持包）
 */

#ifndef __DRV_TIM_H__
#define __DRV_TIM_H__

#include "stdint.h"

/** 频率分频计数器（供需要多路定时任务的场景使用） */
extern uint16_t FreqDiv_Cnt[3];
extern uint16_t FreqDiv_Base[3];

/** 定时器回调函数类型 */
typedef void (*TIM_Callback_t)(void);

/**
 * @brief  初始化定时器时基中断
 */
void TIM_Init(void);

/**
 * @brief  注册定时器回调（在 TIM1 中断中被调用）
 * @param  cb : 回调函数指针
 */
void TIM_RegisterCallback(TIM_Callback_t cb);

#endif
