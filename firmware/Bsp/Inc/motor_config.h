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
