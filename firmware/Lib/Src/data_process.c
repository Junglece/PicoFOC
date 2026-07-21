/**
 * @file    data_process.c
 * @brief   数据处理工具 —— 角度限幅 & 数值限幅
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 纯函数，零硬件依赖。
 *
 * ANGLE_limit()  将角度值通过加减整圆周包装到 [angle_min, angle_max]。
 * DATA_limit()   将浮点数钳制到 [data_min, data_max]。
 */

#include "data_process.h"

/**
 * @brief  角度限幅
 *
 * 通过加减完整圆周周期将角度值包装到目标范围。
 * 典型用法：ANGLE_limit(delta, 360, -180, 180) 求最短路径角度差。
 */
float ANGLE_limit(float angle, float full_circle_angle,
                  float angle_min, float angle_max)
{
    while (angle > angle_max)
    {
        angle -= full_circle_angle;
    }
    while (angle < angle_min)
    {
        angle += full_circle_angle;
    }
    return angle;
}

/**
 * @brief  数值钳制
 *
 * 将 data 限制在 [data_min, data_max] 闭区间。
 */
float DATA_limit(float data, float data_min, float data_max)
{
    if (data > data_max)
    {
        data = data_max;
    }
    else if (data < data_min)
    {
        data = data_min;
    }
    return data;
}
