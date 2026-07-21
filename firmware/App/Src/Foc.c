/**
 * @file    Foc.c
 * @brief   单电机 FOC 控制器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 本文件对应重构前的 Foc.c，核心变化：
 *   1. 不直接调用 HAL_I2C / HAL_TIM，改用 sensor->read_angle() 和 motor->set_duty()
 *   2. 坐标变换委托给 foc_math.c 的纯函数
 *   3. PID 使用 PID_Calc() 替代旧的 pid_output()
 *   4. 只保留一个电机实例 Motor
 */

#include "Foc.h"
#include "main.h"

/* ---- 全局单电机实例 ---- */
FOC_t Motor = {0};

/* ================================================================
 *  FOC_Init —— 初始化 FOC 控制器
 *
 *  存入硬件接口指针 + 观测器 + 电机配置 + 存储接口
 *  → 初始化观测器 → 读取初始角度 → 计算电角度 → 使能 PWM → 拉高 PB1
 *
 *  cfg 参数集中管理极对数、母线电压、控制频率，消除重复配置。
 *  storage 提供非易失存储抽象，换存储介质不影响 FOC 逻辑。
 *  observer 提供角度滤波和速度估计（差分/龙伯格/卡尔曼可互换）。
 * ================================================================ */
void FOC_Init(FOC_t *foc,
              AngleSensor_t *sensor, PwmOutput_t *motor,
              const MotorConfig_t *cfg,
              const NvStorage_t *storage,
              Observer_t *observer)
{
    foc->sensor     = sensor;
    foc->motor      = motor;
    foc->storage    = storage;
    foc->observer   = observer;
    foc->pole_pairs = cfg->pole_pairs;
    foc->Umax       = cfg->bus_voltage;
    foc->rate       = cfg->rate_hz;

    /* 读取初始角度并喂给观测器 */
    float initial = sensor->read_angle(sensor->ctx);
    foc->observer->update(foc->observer->ctx, initial);
    foc->mech_angle      = foc->observer->get_position(foc->observer->ctx);
    foc->last_mech_angle = foc->mech_angle;
    foc->speed           = foc->observer->get_velocity(foc->observer->ctx);
    foc->last_speed      = foc->speed;
    foc->elec_angle      = (float)cfg->pole_pairs * foc->mech_angle;

    /* 优先从非易失存储读取已保存的零点偏移，失败回退到宏默认值 */
    if (!foc->storage->read(foc->storage->ctx,
                             &foc->elec_offset,
                             sizeof(foc->elec_offset)))
    {
        foc->elec_offset = FOC_ELEC_OFFSET;
    }
    foc->calib_flag   = 0;

    /* 使能 PWM 输出 */
    motor->enable(motor->ctx);

    /* PB1 拉高 —— 驱动芯片使能信号 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    /* 初始化独立速度环 PI（默认参数，速度模式可通过 CAN 覆盖） */
    PID_Init(&foc->SpdPID_Ext, 0.05f, 0.01f, 0.0f, cfg->rate_hz,
             cfg->bus_voltage, cfg->bus_voltage);
}

/* ================================================================
 *  FOC_SetPosPID / FOC_SetSpdPID / FOC_SetSpdExtPID —— 设置 PID 参数
 *
 * 注意：PID 已做时间归一化（integral × dt），频率改变时 ki 行为不变。
 * ================================================================ */
void FOC_SetPosPID(FOC_t *foc, float kp, float ki, float kd, float out_max)
{
    PID_Init(&foc->PosPID, kp, ki, kd, foc->rate, out_max, out_max);
}

void FOC_SetSpdPID(FOC_t *foc, float kp, float ki, float kd, float out_max)
{
    PID_Init(&foc->SpdPID, kp, ki, kd, foc->rate, foc->Umax, out_max);
}

void FOC_SetSpdExtPID(FOC_t *foc, float kp, float ki, float kd, float out_max)
{
    PID_Init(&foc->SpdPID_Ext, kp, ki, kd, foc->rate, foc->Umax, out_max);
}

/* ================================================================
 *  FOC_SetTarget —— 设置目标角度（自动限幅 0~360°）
 * ================================================================ */
void FOC_SetTarget(FOC_t *foc, float target_angle)
{
    foc->target_angle = ANGLE_limit(target_angle, 360.0f, 0.0f, 360.0f);
}

/* ================================================================
 *  FOC_UpdateSensor —— 刷新传感器数据
 *
 *  通过 Observer 接口获取滤波后的角度和估计速度。
 *  无论底层是差分法、龙伯格还是卡尔曼，此处接口不变。
 *
 *  三件事：
 *    1. 读取原始角度 → 喂给观测器
 *    2. 从观测器拿滤波后角度 → 计算电角度
 *    3. 从观测器拿估计速度（无噪声放大）
 *
 *  注意：last_mech_angle / last_speed 保留给 PID 微分项使用，
 *  不受观测器影响。
 * ================================================================ */
void FOC_UpdateSensor(FOC_t *foc)
{
    /* 保存上一周期值（给 PID 微分项用） */
    foc->last_mech_angle = foc->mech_angle;
    foc->last_speed      = foc->speed;

    /* 读取原始角度 → 喂给观测器 */
    float raw = foc->sensor->read_angle(foc->sensor->ctx);
    foc->observer->update(foc->observer->ctx, raw);

    /* 位置和速度都来自观测器
     *
     * 之前在 原始值（噪声→微分抖）和 观测值（滞后→过冲）间反复横跳。
     * 现在：提高 l1 让观测器快速收敛，控制滞后在 1~2ms 内，
     * 同时观测器内置的一阶滞后本身就在滤除传感器噪声，
     * 比原始值直通更适合进 PD 控制环的微分项。
     */
    foc->mech_angle = foc->observer->get_position(foc->observer->ctx);
    foc->speed      = foc->observer->get_velocity(foc->observer->ctx);

    /* 电角度 = 极对数 × 机械角度 - 零点偏移 */
    foc->elec_angle = ANGLE_limit(
        (float)foc->pole_pairs * foc->mech_angle - foc->elec_offset,
        (float)foc->pole_pairs * 360.0f,
        0.0f,
        (float)foc->pole_pairs * 360.0f);
}

/* ================================================================
 *  FOC_PosCtrl —— 位置环 PD
 *
 *  误差 = 目标角度 - 当前角度（最短路径）
 *  微分项采用 Derivative-on-Measurement：传入 -Δθ 起阻尼作用（抑制超调）
 * ================================================================ */
void FOC_PosCtrl(FOC_t *foc)
{
    float error = ANGLE_limit(
        foc->target_angle - foc->mech_angle,
        360.0f, -180.0f, 180.0f);

    float angle_delta = ANGLE_limit(
        foc->mech_angle - foc->last_mech_angle,
        360.0f, -180.0f, 180.0f);

    foc->target_speed = PID_Calc(&foc->PosPID, error, -angle_delta);
}

/* ================================================================
 *  FOC_SpdCtrl —— 串级速度环 PI
 *
 *  误差 = 目标速度 - 当前速度
 *  输出 = Uq（交轴电压）
 *  使用 SpdPID（与位置环联调），用于位置模式的串级内环。
 * ================================================================ */
void FOC_SpdCtrl(FOC_t *foc)
{
    float error = foc->target_speed - foc->speed;
    foc->Uq = PID_Calc(&foc->SpdPID, error, foc->speed - foc->last_speed);
}

/* ================================================================
 *  FOC_SpdCtrl_Ext —— 独立速度环 PI
 *
 *  使用 SpdPID_Ext，与串级 SpdPID 互不影响。
 *  默认 PI 控制（kd=0），参数通过 CAN 或 FOC_SetSpdExtPID 配置。
 * ================================================================ */
void FOC_SpdCtrl_Ext(FOC_t *foc)
{
    float error = foc->target_speed - foc->speed;
    foc->Uq = PID_Calc(&foc->SpdPID_Ext, error, foc->speed - foc->last_speed);
}

/* ================================================================
 *  FOC_Enable —— 使能电机输出
 *
 *  拉高 PB1（驱动芯片使能） + 启动 PWM 输出。
 *  从待机态切回运行态时调用。
 * ================================================================ */
void FOC_Enable(FOC_t *foc)
{
    foc->motor->enable(foc->motor->ctx);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

/* ================================================================
 *  FOC_Disable —— 关闭电机输出
 *
 *  拉低 PB1 → 驱动芯片失能 → 内部自动关断 MOSFET。
 *  同时清零所有 PID 积分项，避免切回运行态时积分 windup。
 * ================================================================ */
void FOC_Disable(FOC_t *foc)
{
    /* PB1 拉低 → 驱动芯片自动关断全部输出 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    /* 停止 PWM 输出 */
    foc->motor->disable(foc->motor->ctx);

    /* 清零全部 PID 积分（防 windup） */
    foc->PosPID.integral     = 0.0f;
    foc->SpdPID.integral     = 0.0f;
    foc->SpdPID_Ext.integral = 0.0f;

    /* 清零输出电压 */
    foc->Uq = 0.0f;
    foc->Ud = 0.0f;
}

/* ================================================================
 *  FOC_Output —— FOC 输出
 *
 *  流水线：
 *    1. 校准检测（calib_flag=1 时注入 Ud=4V 拉转子对齐）
 *    2. 反 Park 变换：d-q → α-β
 *    3. 反 Clarke 变换：α-β → 三相电压（量纲为 V）
 *    4. 限幅 → 归一化到 0.0~1.0 → 输出到 PWM
 *
 *  set_duty 接收归一化值，PWM 驱动内部自行换算为定时器比较值。
 * ================================================================ */
void FOC_Output(FOC_t *foc)
{
    /* ---- 电角度校准流程 ---- */
    if (foc->calib_flag == 1)
    {
        foc->elec_offset = 0.0f;
        FOC_CalibStep(foc);
        if (foc->calib_count++ > 3 * (uint32_t)foc->rate)
        {
            /* 校准完成：记录当前电角度作为零点偏移 */
            foc->elec_offset = (float)foc->pole_pairs
                             * foc->sensor->read_angle(foc->sensor->ctx);
            foc->calib_flag = 0;
        }
    }

    /* ---- 反 Park 变换：d-q → α-β ---- */
    float u_alpha, u_beta;
    FOC_Math_InvPark(foc->Ud, foc->Uq,
                     foc->elec_angle * DEG_TO_RAD,
                     &u_alpha, &u_beta);

    /* ---- 反 Clarke 变换：α-β → 三相电压（量纲为 V） ---- */
    float duty_a, duty_b, duty_c;
    FOC_Math_InvClarke(u_alpha, u_beta, foc->Umax,
                        &duty_a, &duty_b, &duty_c);

    /* ---- 限幅到 [0, Umax] 并归一化到 [0.0, 1.0] ---- */
    float inv_umax = 1.0f / foc->Umax;
    foc->Ua = DATA_limit(duty_a, 0.0f, foc->Umax) * inv_umax;
    foc->Ub = DATA_limit(duty_b, 0.0f, foc->Umax) * inv_umax;
    foc->Uc = DATA_limit(duty_c, 0.0f, foc->Umax) * inv_umax;

    /* ---- 输出到 PWM（归一化值 0.0 ~ 1.0） ---- */
    foc->motor->set_duty(foc->motor->ctx, foc->Ua, foc->Ub, foc->Uc);
}

/* ================================================================
 *  FOC_CalibStep —— 执行单步电角度校准
 *
 *  注入 Ud=4V, Uq=0 → 转子被拉到 d 轴对齐 → 此时编码器读数 = 电角度零点偏移。
 *  校准持续约 3 秒（3 × rate 个周期）。
 * ================================================================ */
void FOC_CalibStep(FOC_t *foc)
{
    foc->elec_angle = 0.0f;
    foc->Uq = 0.0f;
    foc->Ud = 4.0f;
}
