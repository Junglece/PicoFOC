/**
 * @file    motor_ctrl.h
 * @brief   电机控制任务调度 —— CAN 指令分发 + FOC 流水线
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 由 TIM1 周期中断回调触发，运行频率 ~1 kHz。
 * 不包含任何硬件依赖，仅调用 FOC / CAN Proto API。
 *
 * 四种控制模式：
 *   STANDBY    (0) : 关断驱动芯片（PB1 拉低，PWM 停止，PID 积分清零）
 *   TORQUE     (1) : 电流环（预留）
 *   SPEED      (2) : 独立速度 PI 环  →  使用 SpdPID_Ext
 *   POSITION   (3) : 串级位置 PD + 速度 PI 环  →  使用 PosPID + SpdPID
 *   CALIBRATE  (4) : 电角度自校准 → 注入 Ud=4V 拉转子对齐，结果写入 Flash
 *
 * 状态回传：两个通道都每周期发送（1 kHz），CAN 直接发，UART 按 10 分频（100 Hz）。
 */

#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

void Motor_Loop(void);

#endif
