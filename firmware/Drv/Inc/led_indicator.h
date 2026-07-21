/**
 * @file    led_indicator.h
 * @brief   LED 指示器 —— 亮度与开关模式分离
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 两个独立维度合成最终效果：
 *   LED_SetBrightness(pct)  →  PWM 占空比 0~100%（通常映射到扭矩）
 *   LED_SetPattern(mode)    →  开关模式（心跳 / 快闪 / 常亮 / 常灭）
 *
 * 合成规则：
 *   模式说"亮" → 灯以当前亮度值亮
 *   模式说"灭" → 灯灭（无视亮度）
 *
 * 数据流：
 *   TIM1 ISR (20kHz) → LED_Tick() → PWM 比较 + 模式状态机
 *   Motor_Loop (1kHz) → LED_SetBrightness + LED_SetPattern
 */

#ifndef __LED_INDICATOR_H__
#define __LED_INDICATOR_H__

#include <stdint.h>

/* ================================================================
 *  引脚定义 —— PA5 板载 LED
 * ================================================================ */
#define LED_GPIO_Port  GPIOA
#define LED_Pin        GPIO_PIN_5

/* ================================================================
 *  开关模式
 * ================================================================ */
typedef enum {
    LED_PATTERN_OFF,            /**< 常灭（亮度被覆盖，不亮） */
    LED_PATTERN_ON,             /**< 常亮（亮度由 SetBrightness 决定） */
    LED_PATTERN_HEARTBEAT,      /**< 心跳（200ms 亮 + 800ms 灭） */
    LED_PATTERN_FAST_BLINK,     /**< 快闪（100ms 亮 + 100ms 灭，校准用） */
    LED_PATTERN_COUNT,
} LED_Pattern_t;

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  设置亮度（PWM 占空比 0~100%）
 * @param  pct  0=灭, 100=最亮, 超出自动限幅
 *
 * 通常 Motor_Loop 里根据 Motor.Uq / Motor.Umax × 100 填入。
 */
void LED_SetBrightness(uint8_t pct);

/**
 * @brief  设置开关模式
 * @param  pattern  模式枚举（OFF/ON/HEARTBEAT/FAST_BLINK）
 */
void LED_SetPattern(LED_Pattern_t pattern);

/**
 * @brief  定时器中断驱动（20kHz，TIM1 ISR 中每周期调用）
 *
 * 内部完成两件事：
 *   1. PWM 比较（200 级，100Hz 刷新率）
 *   2. 模式状态机（心跳 / 快闪）
 */
void LED_Tick(void);

#endif
