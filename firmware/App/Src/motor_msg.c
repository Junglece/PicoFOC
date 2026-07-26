/**
 * @file    motor_msg.c
 * @brief   消息总线实现 —— 中央解码 + 状态编码 + TX 分发
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ════════════════════════════════════════════════════════════════════
 *  本文件是整个通信架构的"心脏"
 * ════════════════════════════════════════════════════════════════════
 *
 *  【RX】can_proto/uart_proto 收到 8 字节 → OnFrameReceived()
 *        → DecodeCommand() 解析 → 写入命令缓冲
 *
 *  【TX】motor_ctrl 每 1ms 调 UpdateStatus() + TxStatus()
 *        → 编码 8 字节 → 分别调 CAN/UART 的发送函数
 *
 *  CAN 和 UART 的 payload 格式一样，所以解码/编码都是同一份代码。
 *  这就是"中央解码"——你只用一个地方拆包/打包，不用在两个通信文件里各写一遍。
 *
 *  如果要加 SPI 通信：
 *    1. 新建 spi_proto.c，在 RX 回调中调 MotorMsg_OnFrameReceived()
 *    2. 在 MotorMsg_TxStatus() 里加一句 SPIProto_SendPayload()
 *    3. motor_ctrl.c 一行不用改
 * ════════════════════════════════════════════════════════════════════
 */

#include "motor_msg.h"
#include "can_proto.h"
#include "uart_proto.h"
#include "main.h"   /* for __disable_irq / __enable_irq */

/* ================================================================
 *  编码 / 解码常数
 * ================================================================ */

/* 状态帧编码系数（float → int16 LE） */
#define POS_ENC_SCALE    (834.4f)    /* 32767 / (12.5 × π) ≈ 834.4   */
#define SPD_ENC_SCALE    (131.07f)   /* 32767 / 250 ≈ 131.07         */
                                     /* rad/s 范围 ±250  →  覆盖 ±2387 rpm   */
                                     /* 分辨率 0.0076 rad/s ≈ 0.073 rpm       */
#define VQ_ENC_SCALE     (819.175f)  /* 32767 / 40 ≈ 819.175        */

/* 控制帧解码系数（原始整型 → float） */
#define KP_DEC_SCALE     (0.00015259f)   /* 10.0 / 65535 */
#define KD_DEC_SCALE     (0.00392157f)   /*  1.0 / 255   */

/* ================================================================
 *  内部状态
 * ================================================================ */

/** 命令缓冲区 (ISR 写, Poll 读, 互斥通过关中断保护) */
static MotorCommand_t  g_cmd       = {MOTOR_MODE_STANDBY};
static volatile uint8_t g_new_frame = 0;

/** 当前状态快照 (供 TX 编码用) */
static float g_pos_rad  = 0.0f;     /**< 机械位置 (rad) */
static float g_spd_rads = 0.0f;     /**< 机械速度 (rad/s) */
static float g_vq       = 0.0f;     /**< 交轴电压 (V) */

/** 最近一次收到 CAN 帧的时刻 (ms)，用于判断 CAN/UART 模式切换 */
static uint32_t g_last_can_rx = 0;

/* ================================================================
 *  中央解码 —— CAN 和 UART 共用（两者 payload 格式一致）
 *
 *  input : data[8]  = 与 CAN 控制帧完全相同的 8 字节布局
 *  output: *cmd     = 填充好的 MotorCommand_t
 * ================================================================ */
static void DecodeCommand(const uint8_t data[8], MotorCommand_t *cmd)
{
    /* ---- 运行模式 ---- */
    cmd->mode = (MotorMode_t)data[0];

    /* ---- 目标值：float 小端 (data[1] ~ data[4]) ---- */
    {
        union { float f; uint8_t b[4]; uint32_t u; } fb;
        fb.b[0] = data[1];
        fb.b[1] = data[2];
        fb.b[2] = data[3];
        fb.b[3] = data[4];
        cmd->target = fb.f;

        /* NaN/Inf 保护：若 target 的指数位全为 1，说明是 NaN 或 Inf */
        if ((fb.u & 0x7F800000UL) == 0x7F800000UL)
            cmd->target = 0.0f;
    }

    /* ---- 位置/速度环 Kp：uint16 小端 (data[5] ~ data[6]) ---- */
    uint16_t kp_raw = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
    cmd->pos_kp = (float)kp_raw * KP_DEC_SCALE;
    cmd->spd_kp = (float)kp_raw * KP_DEC_SCALE;

    /* ---- 位置环 Kd / 速度环 Ki：uint8 (data[7]) ---- */
    cmd->pos_kd = (float)data[7] * KD_DEC_SCALE;
    cmd->spd_ki = (float)data[7] * KD_DEC_SCALE;
}

/* ================================================================
 *  API 实现
 * ================================================================ */

/* ================================================================
 *  初始化
 *  谁调我：main.c 的初始化流程
 *  我做什么：清零所有内部变量，确保初始状态是 MOTOR_MODE_STANDBY
 * ================================================================ */
void MotorMsg_Init(void)
{
    g_new_frame = 0;
    g_cmd.mode  = MOTOR_MODE_STANDBY;
    g_pos_rad   = 0.0f;
    g_spd_rads  = 0.0f;
    g_vq        = 0.0f;
}

/* ================================================================
 *  【RX】通信驱动层收到字节 → 拆包解码 → 存进命令缓冲
 *
 *  谁调我：can_proto.c（CAN 中断）或 uart_proto.c（UART 中断）
 *  我下一步干啥：DecodeCommand() 拆 8 字节 → 写 g_cmd → 置标志
 *  注意：ch 参数目前用不上（CAN/UART 格式一样），留着以后万一格式分化
 * ================================================================ */
void MotorMsg_OnFrameReceived(MotorMsg_Channel_t ch, uint8_t *payload)
{
    if (ch == MSG_CH_CAN)
        g_last_can_rx = HAL_GetTick();

    DecodeCommand(payload, &g_cmd);
    g_new_frame = 1;
}

/* ================================================================
 *  【RX】motor_ctrl 每 1ms 问"有新指令吗？"
 *
 *  谁调我：motor_ctrl.c 的 Motor_Loop()
 *  返回 0 → 没有新指令，保持当前状态
 *  返回 1 → cmd 里有最新指令（mode/target/kp/kd）
 *
 *  关中断保护：g_cmd 可能被 ISR 同时写，关中断避免读到一半的数据
 * ================================================================ */
uint8_t MotorMsg_PollCommand(MotorCommand_t *cmd)
{
    if (!g_new_frame)
        return 0;

    __disable_irq();
    g_new_frame = 0;
    *cmd = g_cmd;
    __enable_irq();

    return 1;
}

/* ================================================================
 *  【TX】motor_ctrl 把当前电机状态存到这儿
 *
 *  谁调我：motor_ctrl.c 的 Motor_Loop()
 *  参数：position_rad 弧度, speed_rads 弧度/秒, vq 伏
 *  存到哪里：静态变量，等会儿 TxStatus() 打包发出去
 * ================================================================ */
void MotorMsg_UpdateStatus(float position_rad, float speed_rads, float vq)
{
    g_pos_rad  = position_rad;
    g_spd_rads = speed_rads;
    g_vq       = vq;
}

/* ================================================================
 *  【TX】打包状态 → 发往所有通信通道（CAN + UART）
 *
 *  谁调我：motor_ctrl.c 的 Motor_Loop() 结尾，每 1ms 一次
 *
 *  流程：
 *    1. 3 个 float（位置/速度/Vq）→ 编码成 int16（省带宽）
 *    2. 填入 8 字节 payload（跟 CAN 状态帧布局完全一样）
 *    3. 分发给各通道：CAN 1kHz + UART 条件分发
 *
 *  【双模策略】
 *    模式判定：500ms 内收到过 CAN 帧 = CAN 模式，否则 UART 模式
 *
 *    CAN 模式（VOFA+ 调试 / 开发者模式）：
 *      CAN Proto → 1kHz（始终发送）
 *      UART Proto → 停发（省出带宽给 VOFA+ 全速调试）
 *      VOFA+ → 全速 36B/帧（在 main.c while(1) 中发送）
 *
 *    UART 模式（日常控制 / 普通用户）：
 *      CAN Proto → 1kHz（始终发送——CAN 总线不占 UART 带宽）
 *      UART Proto → 1kHz（12B/帧 跑满 115200 绰绰有余）
 *      VOFA+ → 停发
 * ================================================================ */
void MotorMsg_TxStatus(void)
{
    uint8_t payload[8];
    int16_t raw;

    raw = (int16_t)(g_pos_rad * POS_ENC_SCALE);
    payload[0] = (uint8_t)(raw & 0xFF);
    payload[1] = (uint8_t)((raw >> 8) & 0xFF);

    raw = (int16_t)(g_spd_rads * SPD_ENC_SCALE);
    payload[2] = (uint8_t)(raw & 0xFF);
    payload[3] = (uint8_t)((raw >> 8) & 0xFF);

    raw = (int16_t)(g_vq * VQ_ENC_SCALE);
    payload[4] = (uint8_t)(raw & 0xFF);
    payload[5] = (uint8_t)((raw >> 8) & 0xFF);

    payload[6] = 0;  /* 保留 */
    payload[7] = 0;  /* 保留 */

    CANProto_SendPayload(payload);   /* → CAN 总线，1kHz */

    /* UART 自定义协议：仅 UART 模式下发送（1kHz） */
    if (!MotorMsg_IsCANMode())
    {
        UARTProto_SendPayload(payload);
    }
}

/* ------------------------------------------------------------------
 *  调试接口
 * ------------------------------------------------------------------ */
MotorMode_t MotorMsg_GetLastMode(void)
{
    MotorMode_t mode;
    __disable_irq();
    mode = g_cmd.mode;
    __enable_irq();
    return mode;
}

float MotorMsg_GetLastTarget(void)
{
    float t;
    __disable_irq();
    t = g_cmd.target;
    __enable_irq();
    return t;
}

uint8_t MotorMsg_IsCANMode(void)
{
    return (HAL_GetTick() - g_last_can_rx) < CAN_TIMEOUT_MS;
}
