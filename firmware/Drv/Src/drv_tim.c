/**
 * @file    drv_tim.c
 * @brief   定时器驱动实现（纯时基，不含业务逻辑）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 核心改动：中断回调中不再直接调用 FOC 函数，改为呼叫已注册的回调。
 * LED 由 led_indicator.c 的软件 PWM 接管（亮度+模式分离）。
 * 原文件名 bsp_tim.c → drv_tim.c（正名：本文件属于驱动层，非板级支持包）
 */

#include "drv_tim.h"
#include "tim.h"
#include "main.h"
#include "led_indicator.h"

/** 频率分频计数器（文件作用域已初始化） */
uint16_t FreqDiv_Cnt[3]  = {0};
uint16_t FreqDiv_Base[3] = {200, 18, 5};
/* FreqDiv_Base[1]=18 → TIM1 硬件 20kHz / (18+2) = 软件 1kHz FOC */
/* 改 FOC_RATE_HZ（main.c）时要同步调整此值，保持 20kHz / (Base[1]+2) = FOC_RATE_HZ */

/** 注册的回调（由 motor_ctrl 注册为 Motor_Loop） */
static TIM_Callback_t user_callback = NULL;

/**
 * @brief  初始化定时器时基中断
 */
void TIM_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim1);
}

/**
 * @brief  注册定时器回调（由 motor_ctrl 调用）
 */
void TIM_RegisterCallback(TIM_Callback_t cb)
{
    user_callback = cb;
}

/* ================================================================
 *  HAL TIM1 周期中断回调
 *
 *  分频列表：
 *    [0] = 每 200  tick → 预留
 *    [1] = 每 20   tick → FOC 控制周期 → 调用注册的回调
 *    [2] = 每 5    tick → 预留
 *
 *  注：LED_Tick() 每周期（20kHz）都调，不做分频——内部自有 PWM 计数器。
 * ================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        /* LED 指示器驱动（含软件 PWM + 模式状态机）—— 每 20kHz tick */
        LED_Tick();

        /* 分频通道 0：预留 */
        if (FreqDiv_Cnt[0]++ > FreqDiv_Base[0])
        {
            FreqDiv_Cnt[0] = 0;
        }

        /* 分频通道 1：FOC 控制周期 → 调用注册的回调 */
        if (FreqDiv_Cnt[1]++ > FreqDiv_Base[1])
        {
            FreqDiv_Cnt[1] = 0;
            if (user_callback != NULL)
            {
                user_callback();    /* → Motor_Loop() */
            }
        }

        /* 分频通道 2：预留 */
        if (FreqDiv_Cnt[2]++ > FreqDiv_Base[2])
        {
            FreqDiv_Cnt[2] = 0;
        }
    }
}
