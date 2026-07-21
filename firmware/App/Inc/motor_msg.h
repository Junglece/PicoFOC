/**
 * @file    motor_msg.h
 * @brief   消息总线 —— 连接通信驱动层(can_proto/uart_proto) 和 控制层(motor_ctrl)
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ════════════════════════════════════════════════════════════════════
 *  整个工程的通信数据流（建议看明白这个再读其他文件）
 * ════════════════════════════════════════════════════════════════════
 *
 *  【RX 方向】外部 → 你的电机控制代码
 *
 *     主机发 CAN 帧 → PA11(CAN_RX) → 中断
 *                                     ↓
 *            can_proto.c: CANProto_RxCallback()
 *                         只做 ID 过滤，不解码
 *                                     ↓
 *     主机发 UART 帧 → PA10(UART_RX) → 中断
 *                                     ↓
 *            uart_proto.c: 接收状态机（找0xAA→读8字节→CRC校验）
 *                          CRC 失败就丢弃，静默
 *                                     ↓
 *           这两条路径最终都调用同一个函数 ↓
 *                                     ↓
 *            motor_msg.c ───────────────────────────────────┐
 *            MotorMsg_OnFrameReceived(ch, payload)          │
 *              ↓                                            │
 *            DecodeCommand(payload, &g_cmd)   ← 中央解码   │
 *            把 8 字节拆开：mode/target/kp/kd               │
 *            CAN 和 UART 用的是同一份解码代码                │
 *              ↓                                            │
 *            g_new_frame = 1                               │
 *              ↓                                            │
 *            motor_ctrl.c: Motor_Loop() ← 每 1ms 一次       │
 *              MotorMsg_PollCommand(&cmd) → 读到最新指令    │
 *              → 切换运行模式/更新 PID 目标                  │
 *              → 执行 FOC 流水线                            │
 *              → ...                                       │
 *              → MotorMsg_UpdateStatus(位置,速度,Vq)       │
 *              → MotorMsg_TxStatus()  ← 发起发送            │
 *              └────────────────────────────────────────────┘
 *
 *  【TX 方向】你的电机控制代码 → 外部主机
 *
 *            MotorMsg_TxStatus()
 *              ↓
 *            编码 8 字节统一 payload
 *            (位置int16 + 速度int16 + Vq int16 + 保留2字节)
 *              ↓
 *            ├─→ CANProto_SendPayload(payload)
 *            │    → CAN_SendMsg() → PA12(CAN_TX) → CAN 总线
 *            │
 *            └─→ UARTProto_SendPayload(payload)
 *                 → 组帧 [0xAA][0x08][8字节][CRC16]
 *                 → PA9(UART_TX) → 上位机
 *
 *  【VOFA+ 调试通道】独立于以上流程
 *    main.c 的 while(1) 循环里单独调：
 *      VOFA_SendDebug() → UARTProto_SendVOFA(8个float)
 *      和上面的自定义协议在同一个 UART 口上互不干扰
 *
 * ════════════════════════════════════════════════════════════════════
 *  你的自由度（改动范围速查）
 * ════════════════════════════════════════════════════════════════════
 *
 *  想改什么                        → 只需要改这些文件
 *  ─────────────────────────────────────────────────
 *  换控制协议格式（8字节怎么拆）      → motor_msg.c
 *  加新运行模式                     → motor_msg.h + motor_ctrl.c
 *  改 CAN 总线速率 / CAN ID         → Core/Src/can.c + can_proto.h
 *  改 UART 波特率                   → Core/Src/usart.c
 *  换 UART 帧头 / CRC 算法          → uart_proto.c
 *  加 SPI 通信                     → 新建 spi_proto.c/.h + main.c
 *  改状态发送频率                   → motor_msg.c 的 TxStatus()
 *  换 VOFA+ 通道数                  → main.c 的 VOFA_SendDebug()
 *  换控制算法（FOC→别的）           → Foc.c + motor_ctrl.c
 *  ─────────────────────────────────────────────────
 *  通信驱动层（can_proto / uart_proto）和控制层（motor_ctrl）
 *  通过本模块解耦——改一边不影响另一边。
 * ════════════════════════════════════════════════════════════════════
 */

#ifndef __MOTOR_MSG_H__
#define __MOTOR_MSG_H__

#include <stdint.h>

/* ================================================================
 *  电机运行模式（与通信介质无关）
 * ================================================================ */
typedef enum {
    MOTOR_MODE_STANDBY    = 0,   /**< 待机失能 */
    MOTOR_MODE_TORQUE     = 1,   /**< 力矩模式（预留） */
    MOTOR_MODE_SPEED      = 2,   /**< 速度模式 */
    MOTOR_MODE_POSITION   = 3,   /**< 位置模式 */
    MOTOR_MODE_CALIBRATE  = 4,   /**< 电角度自校准 */
} MotorMode_t;

/* ================================================================
 *  统一控制指令结构体（与 CAN_CtrlCmd_t 数据布局相同）
 *
 *  8 字节 payload 布局（CAN / UART 通用）：
 *    [0]    : mode (MotorMode_t)
 *    [1]~[4]: target (float, 小端)
 *    [5]~[6]: Kp (uint16, 小端)
 *    [7]    : Kd/Ki (uint8)
 * ================================================================ */
typedef struct {
    MotorMode_t  mode;       /**< 运行模式 */
    float        target;     /**< 目标值 (rad / rad/s / Nm) */
    float        pos_kp;     /**< 位置环比例增益 (V/rad) */
    float        pos_kd;     /**< 位置环阻尼增益 (V/(rad/s)) */
    float        spd_kp;     /**< 速度环比例增益 (V/(rad/s)) */
    float        spd_ki;     /**< 速度环积分增益 (V/(rad·s)) */
} MotorCommand_t;

/* ================================================================
 *  通信通道 ID（标记消息来源）
 * ================================================================ */
typedef enum {
    MSG_CH_CAN  = 0,    /**< CAN 通道 */
    MSG_CH_UART = 1,    /**< UART 通道 */
} MotorMsg_Channel_t;

/* ================================================================
 *  API 说明
 *
 *  这些函数被谁调用：
 *    MotorMsg_Init()               → main.c 的初始化流程调一次
 *    MotorMsg_OnFrameReceived()    → can_proto.c / uart_proto.c 的 RX 回调调
 *    MotorMsg_PollCommand()        → motor_ctrl.c 每 1ms 调一次
 *    MotorMsg_UpdateStatus()       → motor_ctrl.c 每 1ms 调一次
 *    MotorMsg_TxStatus()           → motor_ctrl.c 每 1ms 调一次
 *    MotorMsg_GetLastMode/Target() → main.c 的 VOFA_SendDebug() 调
 * ================================================================ */

void MotorMsg_Init(void);
void MotorMsg_OnFrameReceived(MotorMsg_Channel_t ch, uint8_t *payload);
uint8_t MotorMsg_PollCommand(MotorCommand_t *cmd);
void MotorMsg_UpdateStatus(float position_rad, float speed_rads, float vq);
void MotorMsg_TxStatus(void);
MotorMode_t MotorMsg_GetLastMode(void);
float       MotorMsg_GetLastTarget(void);

#endif /* __MOTOR_MSG_H__ */
