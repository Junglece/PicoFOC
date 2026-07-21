/**
 * @file    pwm_3phase.h
 * @brief   三相 PWM 输出实现（基于 STM32 TIM3）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 实现 pwm_output.h 接口，封装 TIM PWM 占空比设置。
 * set_duty 接收归一化占空比 0.0 ~ 1.0，内部乘 ARR 转换为定时器比较值。
 */

#ifndef __PWM_3PHASE_H__
#define __PWM_3PHASE_H__

#include "tim.h"
#include "pwm_output.h"

/**
 * @brief  创建三相 PWM 输出实例
 * @param  htim : 定时器句柄（如 &htim3）
 * @return 已填充 PwmOutput_t 结构体
 */
PwmOutput_t PWM3Phase_Create(TIM_HandleTypeDef *htim);

/** @brief 设置三相占空比（0.0 ~ 1.0），作为 set_duty 函数指针的实现 */
void PWM3Phase_SetDuty(void *ctx, float a, float b, float c);

/** @brief 使能三相 PWM 输出（启动通道 + 使能定时器） */
void PWM3Phase_Enable(void *ctx);

/** @brief 关闭三相 PWM 输出（停止全部通道） */
void PWM3Phase_Disable(void *ctx);

#endif
