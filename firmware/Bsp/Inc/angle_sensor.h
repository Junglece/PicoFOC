/**
 * @file    angle_sensor.h
 * @brief   角度传感器抽象接口
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 只定义函数指针，不实现任何具体硬件操作。
 *
 * 更换编码器的方法：
 *   1. 新建 angle_xxx.c，实现 read_angle 函数指针
 *   2. 在 main.c 中将 AngleSensor_t 的函数指针指向新实现
 *   3. 其余文件（Foc.c / motor_ctrl.c）一行不用改
 */

#ifndef __ANGLE_SENSOR_H__
#define __ANGLE_SENSOR_H__

typedef struct AngleSensor_t AngleSensor_t;

struct AngleSensor_t
{
    /**
     * @brief  读取机械角度（0 ~ 360°）
     * @param  ctx : 实现方的上下文（通常是 I2C 句柄）
     * @return 机械角度（度）
     */
    float   (*read_angle)(void *ctx);

    void    *ctx;   /**< 上下文指针，如 &hi2c1 */
};

#endif
