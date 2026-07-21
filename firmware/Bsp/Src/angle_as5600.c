/**
 * @file    angle_as5600.c
 * @brief   AS5600 磁编码器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * AS5600：12 位 I2C 磁编码器
 *   - I2C 地址：0x6C
 *   - 角度寄存器：0x0C
 *   - 接口：实现 angle_sensor.h 的 read_angle 函数指针
 */

#include "angle_as5600.h"
#include "i2c.h"

#define AS5600_ADDR      0x6C
#define AS5600_ANGLE_REG 0x0C

/**
 * @brief  读取原始 ADC 值（0 ~ 4095）
 */
uint16_t AS5600_ReadRaw(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, AS5600_ANGLE_REG,
                     I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * @brief  读取机械角度（0 ~ 360°），作为 read_angle 函数指针的实现
 */
float AS5600_ReadAngle(void *ctx)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)ctx;
    return (float)AS5600_ReadRaw(hi2c) * (360.0f / 4096.0f);
}

/**
 * @brief  创建 AS5600 传感器实例
 *
 * 用法：
 *   AngleSensor_t sensor = AS5600_Create(&hi2c1);
 *   float angle = sensor.read_angle(sensor.ctx);
 */
AngleSensor_t AS5600_Create(I2C_HandleTypeDef *hi2c)
{
    AngleSensor_t sensor;
    sensor.read_angle = AS5600_ReadAngle;
    sensor.ctx        = (void *)hi2c;
    return sensor;
}
