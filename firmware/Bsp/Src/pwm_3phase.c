/**
 * @file    pwm_3phase.c
 * @brief   三相 PWM 输出实现（基于 STM32 TIM3）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 实现 pwm_output.h 接口，封装 TIM PWM 占空比设置。
 *
 * 归一化占空比 → 比较值换算：compare = duty × ARR
 * 通道映射（与原始设计一致）：TIM_CH1 = A相, TIM_CH3 = B相, TIM_CH2 = C相
 *
 * 注：set_duty 接收 0.0 ~ 1.0 归一化值，不再关心母线电压。
 */

#include "pwm_3phase.h"

/**
 * @brief  创建三相 PWM 输出实例
 * @param  htim : 定时器句柄（如 &htim3）
 * @return 已填充 PwmOutput_t 结构体
 */
PwmOutput_t PWM3Phase_Create(TIM_HandleTypeDef *htim)
{
    PwmOutput_t pwm;
    pwm.set_duty    = PWM3Phase_SetDuty;
    pwm.enable      = PWM3Phase_Enable;
    pwm.disable     = PWM3Phase_Disable;
    pwm.ctx         = (void *)htim;

    /* 启动三路 PWM 通道 */
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);

    return pwm;
}

/**
 * @brief  设置三相占空比（归一化值 0.0 ~ 1.0）
 *
 * 将归一化占空比乘以 ARR 得到定时器比较值。
 * 通道映射：TIM_CH1 = A相, TIM_CH3 = B相, TIM_CH2 = C相
 */
void PWM3Phase_SetDuty(void *ctx, float a, float b, float c)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ctx;
    float arr = (float)htim->Init.Period;

    __HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, (uint32_t)(a * arr));
    __HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, (uint32_t)(b * arr));
    __HAL_TIM_SetCompare(htim, TIM_CHANNEL_2, (uint32_t)(c * arr));
}

/**
 * @brief  使能三相 PWM 输出（启动通道 + 使能定时器）
 *
 * 与 Disable 完全对称：重启三路 PWM 通道。
 * 从待机态切回运行态时由 FOC_Enable() 调用。
 */
void PWM3Phase_Enable(void *ctx)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ctx;

    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
}

/**
 * @brief  关闭三相 PWM 输出（停止全部通道）
 *
 * 从运行态切到待机态时由 FOC_Disable() 调用。
 */
void PWM3Phase_Disable(void *ctx)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)ctx;

    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
}
