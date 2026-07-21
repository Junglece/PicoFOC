/**
 * @file    pwm_output.h
 * @brief   PWM 输出抽象接口
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 只定义函数指针，不实现任何具体硬件操作。
 *
 * 注：set_duty 接收归一化占空比 0.0 ~ 1.0，PWM 驱动内部自行换算为定时器比较值，
 *     不再关心母线电压。电压相关的量纲由 FOC 层管理。
 *
 * 更换 PWM 方案的方法：
 *   1. 新建 pwm_xxx.c，实现以下四个函数指针
 *   2. 在 main.c 中将 PwmOutput_t 的函数指针指向新实现
 *   3. 其余文件（Foc.c / motor_ctrl.c）一行不用改
 */

#ifndef __PWM_OUTPUT_H__
#define __PWM_OUTPUT_H__

typedef struct PwmOutput_t PwmOutput_t;

struct PwmOutput_t
{
    /**
     * @brief  设置三相占空比（0.0 ~ 1.0，归一化值）
     * @param  ctx    : 实现方的上下文（通常是 TIM 句柄）
     * @param  a/b/c  : A / B / C 相占空比 (0.0 ~ 1.0)
     */
    void    (*set_duty)(void *ctx, float a, float b, float c);

    /**
     * @brief  使能 PWM 输出（启动通道 + 使能定时器）
     */
    void    (*enable)(void *ctx);

    /**
     * @brief  关闭 PWM 输出（停止全部通道）
     */
    void    (*disable)(void *ctx);

    void    *ctx;          /**< 上下文指针，如 &htim3 */
};

#endif
