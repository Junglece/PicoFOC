/**
 * @file    data_process.h
 * @brief   数据处理工具 —— 角度限幅 & 数值限幅
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 纯函数，零硬件依赖，可跨项目复用。
 */

#ifndef __DATA_PROCESS_H__
#define __DATA_PROCESS_H__

/* ================================================================
 *  单位换算常量
 * ================================================================ */

#define DEG_TO_RAD     (0.017453293f)  /**< PI / 180              */
#define RAD_TO_DEG     (57.29578f)     /**< 180 / PI              */
#define RPM_TO_RAD_S   (0.104719755f)  /**< PI / 30  — rpm→rad/s */
#define RAD_S_TO_RPM   (9.5492966f)    /**< 30 / PI  — rad/s→rpm */

/* ================================================================
 *  函数声明
 * ================================================================ */

/**
 * @brief  将角度包装到 [angle_min, angle_max] 范围内
 * @param  angle             原始角度值
 * @param  full_circle_angle 圆周周期（如 360°）
 * @param  angle_min         范围下界
 * @param  angle_max         范围上界
 * @return 范围 [angle_min, angle_max] 内的角度值
 */
float ANGLE_limit(float angle, float full_circle_angle,
                  float angle_min, float angle_max);

/**
 * @brief  将数值钳制到 [data_min, data_max]
 */
float DATA_limit(float data, float data_min, float data_max);

#endif
