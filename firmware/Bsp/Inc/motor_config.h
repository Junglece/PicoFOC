/**
 * @file    motor_config.h
 * @brief   电机硬件参数统一配置
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 集中管理极对数、母线电压、控制频率等硬件参数。
 * 所有模块从此处读取唯一数据源，消除重复配置。
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include <stdint.h>

/**
 * @brief 速度限幅 (rpm)
 *
 * target_speed 被钳制在 ±此值内。
 * 龙伯格观测器的 x2_max 独立设为此值 x5（10000 rpm），
 * 确保观测器自身不限幅控制目标。
 */
#define SPEED_LIMIT_RPM     2000.0f

/**
 * @brief 速度模式软启动斜率 (rpm/s)
 *
 * 限制 target_speed 每秒最大变化量，防止指令跳变导致电流冲击。
 * 每 1ms 控制周期步进 = SPEED_RAMP_RPM_S / 1000。
 *
 * 例如：从 0→2000 rpm 需要 2000/500 = 4 秒。
 * 设为 0 可禁用软启动（target_speed 立即跟随指令）。
 */
#define SPEED_RAMP_RPM_S    500.0f

/**
 * @brief 电角度前馈补偿延迟时间 (s)
 *
 * 补偿从 AS5600 角度采样到 PWM 输出实际生效之间的总延迟。
 * 包括: I2C 读取 + 观测器计算 + PWM 影子寄存器更新。
 *
 * 在 FOC_Output 的反 Park 变换前，根据当前转速推算此期间转子转过的
 * 电角度，提前补偿，使电压矢量始终对准真实转子位置。
 *
 * 公式: angle_advance_deg = speed_rpm × pole_pairs × 6 × 此值
 * 1600 RPM, 7 极对: advance = 1600 × 7 × 6 × 0.00021 = 14.1°
 * 2000 RPM, 7 极对: advance = 2000 × 7 × 6 × 0.00021 = 17.6°
 *
 * 调节方法:
 *   太小 (<0.00015): 补偿不足, 高速时 Id 偏负, 速度上不去
 *   太大 (>0.00035): 过补偿, 高速时 Id 偏正, 可能振荡
 *   推荐从 0.00021 开始, 用 VOFA+ 观察 Id 随速度变化的趋势
 */
#define ANGLE_COMP_DELAY_S  0.00021f

/**
 * @brief 电压调制系数 —— 决定线性调制区最大相电压
 *
 * 实际限制值为 bus_voltage × 此系数。
 * 当 UqUd 矢量模长超过此值时做等比例缩小，避免进入过调制区。
 *
 * 选值：
 *   0.5f      = SPWM  线性调制上限 (Udc/2)
 *   0.57735f  = SVPWM 线性调制上限 (Udc/√3)
 *
 * 当前使用 SVPWM，故设为 0.57735f。
 * 若将来切回 SPWM，改成 0.5f 即可。
 */
#define VOLTAGE_MODULATION_FACTOR  0.577350269f

/**
 * @brief 电机硬件参数
 *
 * 在 main.c 中初始化后传递给 FOC_Init，PWM 驱动层不再关心母线电压。
 */
typedef struct
{
    uint8_t  pole_pairs;      /**< 极对数                      */
    float    bus_voltage;     /**< 母线电压 (V)                */
    float    rate_hz;         /**< 控制频率 (Hz)               */

} MotorConfig_t;

#endif
