/**
 * @file    Foc.h
 * @brief   单电机 FOC 控制器
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 核心设计思路：传感器和执行器通过函数指针接口注入，不直接调用 HAL。
 *
 * 更换编码器（AS5600 → MA730 等）：
 *   1. 新建 angle_xxx.c，实现 AngleSensor_t 接口
 *   2. 在 main.c 中将 sensor 指向新实现
 *   3. 本文件不动
 *
 * 更换 PWM 方案：
 *   1. 新建 pwm_xxx.c，实现 PwmOutput_t 接口
 *   2. 在 main.c 中将 motor 指向新实现
 *   3. 本文件不动
 */

#ifndef __FOC_H__
#define __FOC_H__

/* ================================================================
 *  电角度零点偏移（单位：度，范围 0 ~ pole_pairs×360）
 *
 *  由首次校准得出后填在此处，之后上电自动生效，无需重复校准。
 *  修改值 → 重新编译烧录即可。
 * ================================================================ */
#define FOC_ELEC_OFFSET     0.0f

#include <stdint.h>
#include "angle_sensor.h"
#include "pwm_output.h"
#include "motor_config.h"
#include "nv_storage.h"
#include "pid.h"
#include "foc_math.h"
#include "data_process.h"
#include "observer.h"

typedef struct
{
    /* ==== 硬件接口（函数指针注入） ==== */
    AngleSensor_t   *sensor;        /**< 角度传感器接口 */
    PwmOutput_t     *motor;         /**< PWM 输出接口 */
    const NvStorage_t *storage;     /**< 非易失存储接口（校准参数持久化） */
    Observer_t      *observer;      /**< 角度/速度观测器（滤波+速度估计） */

    /* ==== 电机参数 ==== */
    uint8_t  pole_pairs;            /**< 极对数 */
    float    Umax;                  /**< 母线电压 / PWM 满幅值 (V) */
    int8_t   dir;                   /**< 方向（暂未使用） */

    /* ==== 三相电压 ==== */
    float    Ua, Ub, Uc;
    float    U_alpha, U_beta;
    float    Uq, Ud;

    /* ==== PID 控制器 ==== */
    PID_t    PosPID;                /**< 位置环 PD（串级外环） */
    PID_t    SpdPID;                /**< 速度环 PI（串级内环，与 PosPID 联调） */
    PID_t    SpdPID_Ext;            /**< 独立速度环 PI（速度模式专用，独立调参） */

    /* ==== 控制参数 ==== */
    float    rate;                  /**< 控制频率 (Hz) */

    /* ==== 电机状态 ==== */
    float    mech_angle;            /**< 机械角度 (0 ~ 360°) */
    float    last_mech_angle;
    float    elec_angle;            /**< 电角度 (0 ~ pole_pairs×360°) */
    float    speed;                 /**< 机械转速 (rpm) */
    float    last_speed;

    /* ==== 目标值 ==== */
    float    target_angle;          /**< 目标机械角度 (0 ~ 360°) */
    float    target_speed;          /**< 目标速度 (rpm) */

    /* ==== 三相电流 + 坐标变换输出 ==== */
    float    Ia;                    /**< Phase A 电流 (A) */
    float    Ib;                    /**< Phase B 电流 (A) */
    float    Ic;                    /**< Phase C 电流 (A) */
    float    I_alpha;               /**< α 轴电流 (A) — Clarke 输出 */
    float    I_beta;                /**< β 轴电流 (A) — Clarke 输出 */
    float    Id;                    /**< D 轴电流 (A) — Park 输出 */
    float    Iq;                    /**< Q 轴电流 (A) — Park 输出 */

    /* ==== 三相电流缓冲区（当前恒为 0）==== */
    float    current_raw[3];        /**< 三相电流 (A)，当前未使用恒为 0 */

    /* ==== 电角度校准 ==== */
    uint16_t calib_flag;            /**< 校准标志：1 = 正在校准 */
    uint32_t calib_count;           /**< 校准计数 */
    float    elec_offset;           /**< 电角度零点偏移 */

} FOC_t;

/** 全局单电机实例 */
extern FOC_t Motor;

/* ===== API ===== */

void FOC_Init(FOC_t *foc,
              AngleSensor_t *sensor, PwmOutput_t *motor,
              const MotorConfig_t *cfg,
              const NvStorage_t *storage,
              Observer_t *observer);

/** @brief 设置位置环 PD 参数（串级外环） */
void FOC_SetPosPID(FOC_t *foc, float kp, float ki, float kd, float out_max);

/** @brief 设置串级速度环 PI 参数（与位置环联调的内环） */
void FOC_SetSpdPID(FOC_t *foc, float kp, float ki, float kd, float out_max);

/** @brief 设置独立速度环 PI 参数（速度模式专用，不与位置环耦合） */
void FOC_SetSpdExtPID(FOC_t *foc, float kp, float ki, float kd, float out_max);

/** @brief 设置目标角度（自动限幅到 0~360°） */
void FOC_SetTarget(FOC_t *foc, float target_angle);

void FOC_UpdateSensor(FOC_t *foc);  /**< 读传感器 + 算电角度 + 算转速       */
void FOC_ReadCurrents(FOC_t *foc);  /**< 读三相电流 → Clarke → Park → Id/Iq */
void FOC_PosCtrl(FOC_t *foc);       /**< 位置环（PD）                        */
void FOC_SpdCtrl(FOC_t *foc);       /**< 串级速度环（PI）                     */
void FOC_SpdCtrl_Ext(FOC_t *foc);   /**< 独立速度环（PI，使用 SpdPID_Ext）   */
void FOC_Output(FOC_t *foc);        /**< 反 Park + SVPWM + 输出到 PWM        */

/** @brief 使能电机输出（PB1 拉高 + PWM 启动） */
void FOC_Enable(FOC_t *foc);

/** @brief 关闭电机输出（PB1 拉低 + PWM 停止 + PID 积分清零） */
void FOC_Disable(FOC_t *foc);

/** @brief 执行单步电角度校准（注入 Ud=4V 拉转子到 d 轴对齐） */
void FOC_CalibStep(FOC_t *foc);

#endif
