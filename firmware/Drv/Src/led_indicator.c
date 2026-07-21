/**
 * @file    led_indicator.c
 * @brief   LED 指示器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 核心机制：
 *   软件 PWM 基于 TIM1 20kHz 中断，200 级 → 100Hz PWM 刷新率。
 *   模式状态机在每轮 PWM 周期末尾推进（即 100Hz 步进）。
 *
 * 模式时间参数（以 100Hz tick 为单位）：
 *   HEARTBEAT: 亮 20 ticks (200ms) + 灭 80 ticks (800ms)
 *   FAST_BLINK: 亮 10 ticks (100ms) + 灭 10 ticks (100ms)
 */

#include "led_indicator.h"
#include "gpio.h"

/* ================================================================
 *  常量
 * ================================================================ */
#define PWM_CYCLES      200     /**< PWM 级数（100Hz = 20kHz / 200） */

#define HB_ON_TICKS     20      /**< 心跳亮持续 tick 数（200ms） */
#define HB_OFF_TICKS    80      /**< 心跳灭持续 tick 数（800ms） */
#define BLINK_TICKS     10      /**< 快闪半周期 tick 数（100ms） */

/* ================================================================
 *  全局状态
 * ================================================================ */
static uint8_t       g_brightness  = 0;        /**< 0~100% */
static LED_Pattern_t g_pattern     = LED_PATTERN_OFF;
static LED_Pattern_t g_last_pattern = LED_PATTERN_OFF;  /**< 检测切换用 */

/* ---- 模式状态机（仅在 LED_Tick 中读写，中断上下文） ---- */
static uint16_t      pat_tick      = 0;        /**< 模式内 tick 计数 */
static uint8_t       pat_on        = 0;        /**< 模式当前是否亮 */

/* ================================================================
 *  接口实现
 * ================================================================ */

void LED_SetBrightness(uint8_t pct)
{
    g_brightness = (pct > 100) ? 100 : pct;
}

void LED_SetPattern(LED_Pattern_t pattern)
{
    g_pattern = pattern;
}

/* ---- 模式切换时重置状态机 ---- */
static void pattern_reset(void)
{
    pat_tick = 0;
    switch (g_pattern) {
    case LED_PATTERN_OFF:   pat_on = 0; break;
    case LED_PATTERN_ON:    pat_on = 1; break;
    default:                pat_on = 1; break;   /* 闪烁类先亮 */
    }
}

/* ---- 模式状态机步进（100Hz，每次 PWM 周期末尾调） ---- */
static void pattern_step(void)
{
    pat_tick++;

    switch (g_pattern) {
    case LED_PATTERN_OFF:
        pat_on = 0;
        break;

    case LED_PATTERN_ON:
        pat_on = 1;
        break;

    case LED_PATTERN_HEARTBEAT:
        if (pat_on) {
            if (pat_tick >= HB_ON_TICKS) {
                pat_on    = 0;
                pat_tick  = 0;
            }
        } else {
            if (pat_tick >= HB_OFF_TICKS) {
                pat_on    = 1;
                pat_tick  = 0;
            }
        }
        break;

    case LED_PATTERN_FAST_BLINK:
        if (pat_tick >= BLINK_TICKS) {
            pat_on    = !pat_on;
            pat_tick  = 0;
        }
        break;

    default:
        break;
    }
}

/* ================================================================
 *  LED_Tick —— TIM1 ISR 中调，20kHz
 * ================================================================ */
void LED_Tick(void)
{
    static uint16_t pwm_cnt = 0;

    /* ---- 检测模式切换，重置状态机 ---- */
    if (g_pattern != g_last_pattern) {
        g_last_pattern = g_pattern;
        pattern_reset();
    }

    /* ---- PWM 比较值（0~200） ---- */
    uint16_t cmp = (uint16_t)g_brightness * 2;   /* 0~100 → 0~200 */

    /* ---- 合成输出 ---- */
    if (pat_on && pwm_cnt < cmp) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }

    /* ---- PWM 计数器 + 模式步进 ---- */
    if (++pwm_cnt >= PWM_CYCLES) {
        pwm_cnt = 0;
        pattern_step();
    }
}
