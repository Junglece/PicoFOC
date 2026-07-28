/**
 * @file    motor_ctrl.c
 * @brief   电机控制任务调度 —— 抽象指令分发 + FOC 流水线
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 由 TIM1 周期中断回调触发，运行频率 1 kHz。
 *
 * 与通信介质解耦：通过 motor_msg.h 统一接口收发指令/状态，
 * 不再直接依赖 can_proto 或 uart_proto。
 *
 * 每轮控制流程：
 *   1. 轮询消息总线获取最新指令 → 解析运行模式与目标值
 *   2. 处理模式切换（待机 ↔ 运行，使能 / 关断驱动芯片）
 *   3. 按当前模式执行对应控制策略
 *   4. 运行 FOC 流水线（传感器 → 控制环 → SVPWM → 输出）
 *   5. 更新状态 → 触发所有通道回传（1 kHz）
 */

#include "motor_ctrl.h"
#include "Foc.h"
#include "motor_msg.h"
#include "data_process.h"
#include "drv_tim.h"
#include "led_indicator.h"
#include "main.h"

/* SPEED_LIMIT_RPM 定义于 motor_config.h —— 所有模块共享 */

void Motor_Loop(void)
{
    static MotorMode_t   g_mode     = MOTOR_MODE_STANDBY;    /**< 当前运行模式 */
    static uint16_t      g_comms_timeout = 0;                /**< 通信超时计数器 */

    MotorCommand_t       cmd;
    uint8_t              new_cmd    = MotorMsg_PollCommand(&cmd);

    /* 喂独立看门狗 —— 如果此处卡死，IWDG ~2s 后自动复位 MCU */
    IWDG_REFRESH();      /* 喂独立看门狗 —— IWDG 寄存器直接操作 */

    /* ================================================================
     *  第 1 步：处理新指令 → 模式切换 + 参数更新
     *
     *  校准模式中忽略其他指令（避免异常切换破坏校准流程）。
     *  收到 STANDBY 可随时中止校准。
     * ================================================================ */
    if (new_cmd)
    {
        g_comms_timeout = 0;    /* 收到新指令 → 复位通信超时 */

        /* ---- 校准中收到待机：中止校准 ---- */
        if (g_mode == MOTOR_MODE_CALIBRATE && cmd.mode == MOTOR_MODE_STANDBY)
        {
            FOC_Disable(&Motor);
            g_mode = MOTOR_MODE_STANDBY;
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
        }
        /* ---- 校准中忽略其他指令 ---- */
        else if (g_mode == MOTOR_MODE_CALIBRATE)
        {
            /* 丢弃，保持校准状态 */
        }
        /* ---- 正常模式切换 ---- */
        else
        {
            /* 进入 / 退出待机 */
            if (cmd.mode == MOTOR_MODE_STANDBY && g_mode != MOTOR_MODE_STANDBY)
            {
                FOC_Disable(&Motor);
                LED_SetPattern(LED_PATTERN_HEARTBEAT);
            }
            else if (cmd.mode != MOTOR_MODE_STANDBY && g_mode == MOTOR_MODE_STANDBY)
            {
                FOC_Enable(&Motor);
            }

            g_mode = cmd.mode;

            switch (g_mode)
            {
            case MOTOR_MODE_STANDBY:
                break;

            case MOTOR_MODE_SPEED:
                Motor.target_speed = cmd.target * RAD_S_TO_RPM;
                Motor.target_speed = DATA_limit(Motor.target_speed,
                                                  -SPEED_LIMIT_RPM, SPEED_LIMIT_RPM);
                FOC_SetSpdExtPID(&Motor, cmd.spd_kp, cmd.spd_ki, 0.0f, Motor.Umax);
                break;

            case MOTOR_MODE_POSITION:
                FOC_SetTarget(&Motor, cmd.target * RAD_TO_DEG);
                FOC_SetPosPID(&Motor, cmd.pos_kp, 0.0f, cmd.pos_kd, 2000.0f);
                break;

            case MOTOR_MODE_CALIBRATE:
                /* 启动自校准：注入 Ud=4V 拉转子到 d 轴对齐 */
                Motor.calib_flag  = 1;
                Motor.calib_count = 0;
                LED_SetPattern(LED_PATTERN_FAST_BLINK);
                break;

            default:
                break;
            }
        }
    }

    /* ================================================================
     *  通信超时保护：超过 100ms 未收到指令 → 自动待机
     *
     *  （仅 SPEED / POSITION / TORQUE 模式生效，
     *   CALIBRATE 模式持续 ~3s 无指令，跳过超时检查）
     * ================================================================ */
    if (g_mode == MOTOR_MODE_SPEED || g_mode == MOTOR_MODE_POSITION
        || g_mode == MOTOR_MODE_TORQUE)
    {
        if (++g_comms_timeout >= 100)
        {
            FOC_Disable(&Motor);
            g_mode = MOTOR_MODE_STANDBY;
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
        }
    }
    else
    {
        g_comms_timeout = 0;
    }

    /* ================================================================
     *  第 2 步：FOC 流水线（按模式执行不同控制环）
     * ================================================================ */
    FOC_UpdateSensor(&Motor);
    FOC_ReadCurrents(&Motor);       /* 读三相电流 → Clarke → Park → Id/Iq */

    switch (g_mode)
    {
    case MOTOR_MODE_STANDBY:
        break;

    case MOTOR_MODE_SPEED:
        FOC_SpdCtrl_Ext(&Motor);
        break;

    case MOTOR_MODE_POSITION:
        FOC_PosCtrl(&Motor);
        FOC_SpdCtrl(&Motor);
        break;

    case MOTOR_MODE_CALIBRATE:
        /* 跳过控制环 —— FOC_Output 内部的 calib_flag 会自动注入 Ud=4V */
        break;

    default:
        break;
    }

    FOC_Output(&Motor);

    /* ================================================================
     *  LED 指示器 —— 亮度来自扭矩，模式由事件驱动
     *
     *  亮度（Brightness）：
     *    待机/校准：强制 100%，不受扭矩影响（要闪烁可见）
     *    运行态：20%~100% 线性映射到 0~12V Uq
     *  模式（Pattern）：
     *    待机/校准时由模式切换事件设置，运行态默认常亮。
     * ================================================================ */
    {
        uint8_t brightness;
        if (g_mode == MOTOR_MODE_CALIBRATE) {
            brightness = 100;
            /* 校准已在进入时设为 FAST_BLINK */
        } else if (g_mode == MOTOR_MODE_STANDBY) {
            brightness = 100;
            /* 待机已在切换事件中设为 HEARTBEAT */
        } else {
            /* 运行态：5%~100% 对应 0~12V */
            float ratio = Motor.Uq / Motor.Umax;
            if (ratio < 0.0f)  ratio = -ratio;
            if (ratio > 1.0f)  ratio = 1.0f;
            brightness = 5 + (uint8_t)(ratio * 95.0f);
            LED_SetPattern(LED_PATTERN_ON);
        }
        LED_SetBrightness(brightness);
    }

    /* ================================================================
     *  第 3 步：校准完成检测
     *
     *  FOC_Output 中 calib_count 超过 3s 后自动清零 calib_flag，
     *  并将当前 pole_pairs × mech_angle 写入 elec_offset。
     *  此处检测到 flag 清零 → 写入 Flash → 恢复 LED → 关断 → 回待机。
     * ================================================================ */
    if (g_mode == MOTOR_MODE_CALIBRATE && Motor.calib_flag == 0)
    {
        /* 写入非易失存储永久保存 */
        Motor.storage->write(Motor.storage->ctx,
                             &Motor.elec_offset,
                             sizeof(Motor.elec_offset));

        LED_SetPattern(LED_PATTERN_HEARTBEAT);

        /* 关断驱动 → 回到待机 */
        FOC_Disable(&Motor);
        g_mode = MOTOR_MODE_STANDBY;
    }

    /* ================================================================
     *  第 4 步：更新状态 → 触发所有通道回传
     *
     *  MotorMsg_TxStatus() 内部会编码为 8 字节统一 payload，
     *  然后分别调 CANProto_SendPayload() 和 UARTProto_SendPayload()。
     *  两个通道每 1ms 都发（1 kHz），各自独立。
     * ================================================================ */
    MotorMsg_UpdateStatus(
        Motor.mech_angle * DEG_TO_RAD,
        Motor.speed * RPM_TO_RAD_S,
        Motor.Uq
    );
    MotorMsg_TxStatus();
}
