/**
 * @file    uart_proto.h
 * @brief   UART 通信驱动 —— 自定义协议 + VOFA+ 调试，共用 USART1
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ════════════════════════════════════════════════════════════════════
 *  本文件提供两个独立的发送接口，共用同一根 UART 线
 * ════════════════════════════════════════════════════════════════════
 *
 *  接口1：UARTProto_SendPayload() — 自定义控制协议帧
 *  谁调我：motor_msg.c 的 MotorMsg_TxStatus()
 *  帧格式：
 *    ┌──────┬──────┬──────────────────────────┬────────────┐
 *    │ 0xAA │ 0x08 │  8 bytes payload (同CAN)  │ CRC16-CCITT│
 *    │  1B   │  1B  │       8B                 │  2B         │
 *    └──────┴──────┴──────────────────────────┴────────────┘
 *    用途：发电机状态给上位机/另一个 MCU，1kHz 频率
 *
 *  接口2：UARTProto_SendVOFA() — VOFA+ 示波器调试帧
 *  谁调我：main.c 的 VOFA_SendDebug()，在主循环里调
 *  帧格式：
 *    [N×float (4N bytes)] + [0x00,0x00,0x80,0x7f]
 *    用途：发给 PC 端的 VOFA+ 软件画波形，100Hz 频率（有限速）
 *
 *  ！！！两个接口在同一个 USART1 上互不干扰 ！！！
 *  VOFA+ 上位机只认尾部 0x00,0x00,0x80,0x7f，看到 0xAA 开头的帧当无效浮点数忽略
 *  自定义上位机只认 0xAA 开头，看到 VOFA 帧 CRC 校验不通过就丢弃
 *
 *  接收（主机→电机）：
 *    USART1 中断 → uart_rx_handler(字节) 状态机
 *    找 0xAA → 读 0x08 → 收 8 字节 payload → 收 2 字节 CRC
 *    CRC 通过 → MotorMsg_OnFrameReceived(MSG_CH_UART, payload)
 *    CRC 失败 → 静默丢弃
 *
 *  收发都用中断方式（非阻塞），busy 时新帧自动丢弃（不阻塞控制环）
 * ════════════════════════════════════════════════════════════════════
 */

#ifndef __UART_PROTO_H__
#define __UART_PROTO_H__

#include <stdint.h>

/**
 * @brief 初始化 UART 协议层
 * 谁调我：main.c
 * 我干啥：使能 USART1 RX 中断，启动第一个字节接收
 */
void UARTProto_Init(void);

/**
 * @brief 发送自定义控制协议帧（仿 CAN 8 字节 + CRC）
 * 谁调我：motor_msg.c 的 MotorMsg_TxStatus()
 * 参数：data[8] 已编码好的 8 字节 payload（跟 CAN 状态帧一样）
 * 我干啥：组帧 [0xAA][0x08][data][CRC16] → 中断方式发出
 * 注意：busy 时静默丢弃（1kHz 频率下丢几帧影响不大）
 */
void UARTProto_SendPayload(uint8_t data[8]);

/**
 * @brief 发送 VOFA+ just-float 调试帧
 * 谁调我：main.c 的 VOFA_SendDebug()
 * 参数：data[] float 数组, count float 个数（最大 8）
 * 我干啥：组帧 [N×float][0x00,0x00,0x80,0x7f] → 中断方式发出
 * 注意：自带 100Hz 速率限制，避免抢占自定义协议的带宽
 */
void UARTProto_SendVOFA(const float *data, uint8_t count);

#endif /* __UART_PROTO_H__ */
