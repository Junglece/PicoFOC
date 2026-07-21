/**
 * @file    can_proto.c
 * @brief   CAN 通信驱动实现 —— 最底层的收发
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ════════════════════════════════════════════════════════════════════
 *  本文件的职责就两件
 * ════════════════════════════════════════════════════════════════════
 *
 *  【收】CAN 中断来 → HAL_CAN_RxFifo0MsgPendingCallback
 *        → CANProto_RxCallback(id, data[8])
 *        → 判断是不是发给本电机的 ID
 *        → 是 → 直接转发给 MotorMsg_OnFrameReceived(MSG_CH_CAN, data)
 *        → 否 → 丢弃
 *        （本文件不拆包，让 motor_msg.c 去拆）
 *
 *  【发】motor_msg.c 编码完 8 字节 payload 后调我
 *        → CANProto_SendPayload(payload)
 *        → 加上标准 CAN ID → HAL_CAN_AddTxMessage 发出去
 *        （本文件不打包，让 motor_msg.c 去打包）
 * ════════════════════════════════════════════════════════════════════
 */

#include "can_proto.h"
#include "motor_msg.h"
#include "drv_can.h"

/* 本机电机 ID（运行时配置，Init 时传入） */
static uint8_t g_motor_id = 1;

/* ================================================================
 *  RX 中断回调
 *
 *  谁调我：drv_can.c 的 HAL_CAN_RxFifo0MsgPendingCallback
 *          （它在 CAN 中断上下文里调我）
 *  我干啥：检查 ID 是否是 0x000 + 本机ID，对的送 motor_msg，错的不管
 *  我调谁：MotorMsg_OnFrameReceived(MSG_CH_CAN, data)
 * ================================================================ */
static void CANProto_RxCallback(uint32_t id, uint8_t data[8])
{
    if (id != CAN_CTRL_ID(g_motor_id))
        return;

    MotorMsg_OnFrameReceived(MSG_CH_CAN, data);
}

/* ================================================================
 *  初始化
 *
 *  谁调我：main.c
 *  参数：motor_id = 本电机 ID（1~15），决定了 RX_ID = 0x000+ID，TX_ID = 0x100+ID
 *  我干啥：记住电机 ID，注册 RX 回调到 drv_can 驱动
 * ================================================================ */
void CANProto_Init(uint8_t motor_id)
{
    g_motor_id = motor_id;
    CAN_RegisterRxCB(CANProto_RxCallback);
}

/* ================================================================
 *  发一帧 CAN 数据
 *
 *  谁调我：motor_msg.c 的 MotorMsg_TxStatus()
 *  参数：data[8] 是 motor_msg 已经编码好的 8 字节 payload
 *  我干啥：加上 CAN ID（0x100 + 本机ID），塞进 CAN 硬件发送
 * ================================================================ */
void CANProto_SendPayload(uint8_t data[8])
{
    CAN_SendMsg(CAN_STAT_ID(g_motor_id), data);
}
