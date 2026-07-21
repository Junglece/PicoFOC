/**
 * @file    angle_as5600.h
 * @brief   AS5600 磁编码器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * AS5600：12 位 I2C 磁编码器，地址 0x6C，角度寄存器 0x0C。
 * 实现 angle_sensor.h 接口，封装 I2C 读取和角度换算。
 */

#ifndef __ANGLE_AS5600_H__
#define __ANGLE_AS5600_H__

#include "i2c.h"
#include "angle_sensor.h"

/**
 * @brief  创建 AS5600 传感器实例（填充 AngleSensor_t 接口）
 * @param  hi2c : I2C 句柄
 * @return 已填充 AngleSensor_t 结构体
 */
AngleSensor_t AS5600_Create(I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取原始角度值（0 ~ 4095，12 位）
 */
uint16_t AS5600_ReadRaw(I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取机械角度（0 ~ 360°），作为 read_angle 函数指针的实现
 */
float AS5600_ReadAngle(void *ctx);

#endif
