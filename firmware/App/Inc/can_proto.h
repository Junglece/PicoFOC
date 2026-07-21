/**
 * @file    can_proto.h
 * @brief   CAN 通信驱动 —— 只做收发，不做协议解析
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ════════════════════════════════════════════════════════════════════
 *  本文件在整个架构中扮演的角色
 * ════════════════════════════════════════════════════════════════════
 *  CAN 总线 ←→ CANProto_SendPayload() / CANProto_RxCallback()
 *                ↓                          ↑
 *           motor_msg.c 消息总线（拆包/打包/分发）
 *                ↓                          ↑
 *           motor_ctrl.c 电机控制（完全不知道 CAN 的存在）
 *
 *  本文件不做任何协议解析，收到字节就往上送，要发字节就从上面拿。
 *  协议格式（8 字节怎么拆）是 motor_msg.c 的事——跟本文件无关。
 *
 *  【CAN ID 分配规则】
 *    控制帧（主机→电机）：CAN ID = 0x000 + 电机ID
 *    状态帧（电机→主机）：CAN ID = 0x100 + 电机ID
 *    电机 ID 在 Init() 时传入，运行时决定，不依赖宏
 * ════════════════════════════════════════════════════════════════════
 */

#ifndef __CAN_PROTO_H__
#define __CAN_PROTO_H__

#include <stdint.h>

/* CAN ID 计算宏（将电机ID转为标准帧ID） */
#define CAN_CTRL_ID(id)     ((uint32_t)(0x000 + (id)))   /* 主机→电机 */
#define CAN_STAT_ID(id)     ((uint32_t)(0x100 + (id)))   /* 电机→主机 */

/* 兼容宏（旧代码用 CAN_MODE_xxx，新代码用 MOTOR_MODE_xxx） */
#define CAN_MODE_STANDBY    0
#define CAN_MODE_TORQUE     1
#define CAN_MODE_SPEED      2
#define CAN_MODE_POSITION   3
#define CAN_MODE_CALIBRATE  4

/* ================================================================
 *  API 说明
 *
 *  CANProto_Init(电机ID)  → main.c 调一次，传入本机 ID
 *  CANProto_SendPayload() → motor_msg.c 的 TxStatus() 调
 *
 *  示例（main.c）：
 *    CANProto_Init(1);     // RX=0x001, TX=0x101
 *    CANProto_Init(5);     // RX=0x005, TX=0x105
 * ================================================================ */

void CANProto_Init(uint8_t motor_id);
void CANProto_SendPayload(uint8_t data[8]);

#endif
