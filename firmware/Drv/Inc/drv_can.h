/**
 * @file    drv_can.h
 * @brief   CAN 驱动（纯收发，不解析协议）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 改动说明：
 *   - Originally hardcoded a specific protocol (ID=0x2) in the CAN IRQ callback,
 *     now refactored to callback registration; protocol parsing moved to can_proto.c.
 *   - 原文件名 bsp_can.h → drv_can.h（正名：本文件属于驱动层，非板级支持包）
 */

#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#include "can.h"

/** CAN 接收回调函数类型 */
typedef void (*CAN_RxCallback_t)(uint32_t id, uint8_t data[8]);

/* ===== API ===== */

/**
 * @brief  初始化 CAN 滤波器并启动
 */
void CAN_Init(void);

/**
 * @brief  注册 CAN 接收回调（由协议层调用）
 * @param  cb : void cb(uint32_t id, uint8_t data[8])
 */
void CAN_RegisterRxCB(CAN_RxCallback_t cb);

/**
 * @brief  发送一帧 CAN 消息（8 字节数据）
 * @param  id   : 标准帧 ID
 * @param  data : 8 字节数据
 * @return HAL 状态
 */
uint8_t CAN_SendMsg(uint16_t id, uint8_t data[8]);

#endif
