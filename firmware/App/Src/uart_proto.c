/**
 * @file    uart_proto.c
 * @brief   UART 通信驱动实现 —— 接收状态机 + 两路独立发送
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ════════════════════════════════════════════════════════════════════
 *  本文件做三件事
 * ════════════════════════════════════════════════════════════════════
 *
 *  【收】USART1 中断 → uart_rx_handler(byte)
 *        四状态状态机：找SOF → 读LEN → 收8字节payload → 收2字节CRC
 *        CRC 通过 → MotorMsg_OnFrameReceived(MSG_CH_UART, payload)
 *        CRC 失败 → 直接扔掉，回到找SOF
 *
 *  【发-自定义协议】motor_msg.c 的 TxStatus() 调 UARTProto_SendPayload()
 *        组帧 [0xAA][0x08][8字节][CRC16] → 中断方式发出
 *        （和 CAN 的 payload 完全一样，让上位机可以一套代码兼容两种介质）
 *
 *  【发-VOFA+】main.c 的 VOFA_SendDebug() 调 UARTProto_SendVOFA()
 *        组帧 [8×float][0x00,0x00,0x80,0x7f] → 中断方式发出
 *        内置 100Hz 限速，防止淹掉自定义协议帧
 *
 *  两个发送共用 uart_tx_busy 标志互斥：busy 时新帧直接丢弃
 *  —115200 波特率下，自定义帧 12 字节约 1ms，VOFA 帧 36 字节约 3ms
 *  —1kHz 的自定义帧基本每次都能发出，VOFA 被限到 100Hz，不会冲突
 * ════════════════════════════════════════════════════════════════════
 */

#include "uart_proto.h"
#include "motor_msg.h"
#include "usart.h"
#include <string.h>

/* ================================================================
 *  协议常数
 * ================================================================ */
#define UART_SOF            0xAA        /**< 帧起始字节 */
#define UART_PAYLOAD_LEN    0x08        /**< payload 固定 8 字节 */
#define UART_FRAME_LEN      12          /**< 整帧长度 = 1+1+8+2 */

/* ================================================================
 *  RX 状态机
 * ================================================================ */
typedef enum {
    UART_RX_WAIT_SOF,       /**< 等待 SOF */
    UART_RX_WAIT_LEN,       /**< 等待 LEN */
    UART_RX_WAIT_PAYLOAD,   /**< 收取 payload */
    UART_RX_WAIT_CRC,       /**< 收取 CRC */
} UART_RxState_t;

static volatile UART_RxState_t  uart_rx_state = UART_RX_WAIT_SOF;
static          uint8_t         uart_rx_buf[UART_FRAME_LEN];
static          uint8_t         uart_rx_idx;
static          uint8_t         uart_rx_byte;       /**< 单字节接收缓冲 */

/* ================================================================
 *  TX 状态
 * ================================================================ */
static volatile uint8_t uart_tx_busy = 0;       /**< 1 = 有发送在进行 */
static          uint8_t uart_tx_custom_frame[UART_FRAME_LEN]; /**< 自定义帧缓冲 */
#define VOFA_MAX_BYTES  (8 * sizeof(float) + 4)  /**< 8 float + 4 tail */
static          uint8_t uart_tx_vofa_buf[VOFA_MAX_BYTES]; /**< VOFA+ 帧缓冲 */

/* ================================================================
 *  CRC16-CCITT (软件查表法，多项式 0x1021)
 *
 * 对 SOF + LEN + 8 字节 payload 共 10 字节计算 CRC。
 * 初始值 0xFFFF，输出不移位。
 * ================================================================ */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ================================================================
 *  RX 字节处理（在 USART1 中断上下文中被调用）
 *
 *  状态机流程：
 *    UART_RX_WAIT_SOF     → 等 0xAA
 *    UART_RX_WAIT_LEN     → 读 LEN，必须是 0x08
 *    UART_RX_WAIT_PAYLOAD → 收 8 字节 payload
 *    UART_RX_WAIT_CRC     → 收 2 字节 CRC → 校验
 *      ├─ CRC 对 → MotorMsg_OnFrameReceived(MSG_CH_UART, payload)
 *      └─ CRC 错 → 静默丢弃
 *
 *  任何一步出错（比如 LEN≠0x08）就立刻回到 WAIT_SOF
 *  这是最简单的"找帧头"策略——不会因为噪声数据死锁
 * ================================================================ */
static void uart_rx_handler(uint8_t byte)
{
    switch (uart_rx_state)
    {
    case UART_RX_WAIT_SOF:
        if (byte == UART_SOF)
        {
            uart_rx_buf[0] = byte;
            uart_rx_idx    = 1;
            uart_rx_state  = UART_RX_WAIT_LEN;
        }
        break;

    case UART_RX_WAIT_LEN:
        uart_rx_buf[1] = byte;
        if (byte == UART_PAYLOAD_LEN)
        {
            uart_rx_state = UART_RX_WAIT_PAYLOAD;
            uart_rx_idx   = 2;
        }
        else
        {
            uart_rx_state = UART_RX_WAIT_SOF;   /* 非法长度，复位 */
        }
        break;

    case UART_RX_WAIT_PAYLOAD:
        uart_rx_buf[uart_rx_idx++] = byte;
        if (uart_rx_idx >= 10)                  /* SOF + LEN + 8B payload */
        {
            uart_rx_state = UART_RX_WAIT_CRC;
        }
        break;

    case UART_RX_WAIT_CRC:
        uart_rx_buf[uart_rx_idx++] = byte;
        if (uart_rx_idx >= UART_FRAME_LEN)
        {
            /* CRC 校验：收到 CRC vs 计算 CRC */
            uint16_t crc_calc = crc16_ccitt(uart_rx_buf, 10);
            uint16_t crc_recv = (uint16_t)uart_rx_buf[10]
                              | ((uint16_t)uart_rx_buf[11] << 8);
            if (crc_calc == crc_recv)
            {
                /* CRC 通过 → 验证 mode 合法性（0~4），防止噪声帧污染指令 */
                uint8_t mode = uart_rx_buf[2];
                if (mode <= 4)
                {
                    MotorMsg_OnFrameReceived(MSG_CH_UART, &uart_rx_buf[2]);
                }
                /* mode 非法 → 静默丢弃（噪声生成的有效CRC但内容垃圾） */
            }
            /* CRC 失败 → 静默丢弃 */

            uart_rx_state = UART_RX_WAIT_SOF;   /* 无论成功与否都复位 */
        }
        break;
    }
}

/* ================================================================
 *  HAL UART 回调（覆盖 weak 定义）
 * ================================================================ */

/**
 * @brief  USART1 RX 完成回调（每收到一个字节触发一次）
 *
 * 处理接收字节后重新使能下一个字节的接收中断。
 * 调用 HAL_UART_Receive_IT 再次使能可能导致频繁中断，
 * 但 USART1 中断优先级设为 0（最高），不会被其他中断打断。
 * 对于 115200 波特率，每约 86μs 触发一次中断，CPU 负担可接受。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_rx_handler(uart_rx_byte);

        /* 重新使能单字节接收中断 */
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

/**
 * @brief  USART1 TX 完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_tx_busy = 0;
    }
}

/**
 * @brief  USART1 错误回调
 *
 * RX 悬空时的噪声会导致 UART 帧错误/溢出错误/噪声错误。
 * HAL 默认处理完错误后可能不触发 RxCpltCallback，导致 RX 中断永久停摆。
 * 此回调确保错误发生后重新使能 RX 接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 清除错误标志 */
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_OREFLAG(&huart1);

        /* 如果之前有挂起的 Receive_IT 因错误终止，重新使能 */
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

        /* 重置状态机 */
        uart_rx_state = UART_RX_WAIT_SOF;
    }
}

/* ================================================================
 *  公开 API
 * ================================================================ */

/* ================================================================
 *  初始化
 *
 *  谁调我：main.c（初始化流程）
 *  我干啥：
 *    1. 重置 RX 状态机，清除 TX busy
 *    2. 使能 USART1 全局中断（优先级 0，和 CAN 同级——不互相抢占，安全）
 *    3. 启动单字节接收中断——之后每个字节到都调 uart_rx_handler()
 * ================================================================ */
void UARTProto_Init(void)
{
    uart_rx_state = UART_RX_WAIT_SOF;
    uart_tx_busy  = 0;

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* 使能错误中断，确保 RX 悬空产生噪声帧/溢出时能通过 ErrorCallback 恢复 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/* ================================================================
 *  发送自定义控制协议帧
 *
 *  谁调我：motor_msg.c 的 MotorMsg_TxStatus()（每 1ms）
 *  参数：data[8] 已编码好的 8 字节 payload（位置/速度/Vq）
 *
 *  我干啥：组帧 [0xAA][0x08][data][CRC16 LE] → 中断发出
 *    0xAA      = 帧头，上位机用它识别新帧开始
 *    0x08      = payload 长度（固定 8 字节）
 *    data[0..7]= 跟 CAN 状态帧完全一样的 8 字节
 *    CRC16     = SOF+LEN+payload 共 10 字节的 CRC16-CCITT
 *
 *  如果上一帧还没发完（uart_tx_busy=1），本帧直接丢弃
 *  1kHz 频率下连续两三帧被丢的概率极低，不影响控制
 * ================================================================ */
void UARTProto_SendPayload(uint8_t data[8])
{
    /* 关中断保护 uart_tx_busy 检查-设置原子性！
     * 不加保护时：TIM1 中断和主循环可能同时通过检查，
     * 两次 HAL_UART_Transmit_IT 覆盖了 HAL 内部缓冲区指针，
     * 导致发出去的是两帧拼接的垃圾，VOFA+ 认不出来。 */
    __disable_irq();
    if (uart_tx_busy) { __enable_irq(); return; }
    uart_tx_busy = 1;
    __enable_irq();

    /* 组帧（不需要关中断保护，uart_tx_custom_frame 不会被中断修改） */
    uart_tx_custom_frame[0] = UART_SOF;
    uart_tx_custom_frame[1] = UART_PAYLOAD_LEN;
    for (uint8_t i = 0; i < 8; i++)
        uart_tx_custom_frame[2 + i] = data[i];

    uint16_t crc = crc16_ccitt(uart_tx_custom_frame, 10);
    uart_tx_custom_frame[10] = (uint8_t)(crc & 0xFF);
    uart_tx_custom_frame[11] = (uint8_t)((crc >> 8) & 0xFF);

    HAL_UART_Transmit_IT(&huart1, uart_tx_custom_frame, UART_FRAME_LEN);
}

/******************************************************************************
 *  发送 VOFA+ just-float 调试帧
 *
 *  谁调我：main.c 的 VOFA_SendDebug()（在 while(1) 循环里调）
 *  参数：data[] float 数组, count float 个数（最大 8 个）
 *
 *  我干啥：组帧 [N×float][0x00,0x00,0x80,0x7f] → 中断发出
 *    STM32 发 8 个 float（32 字节）→ VOFA+ 在上位机画 8 条波形
 *    帧尾 0x00,0x00,0x80,0x7f 是 VOFA+ 的 just-float 引擎标记
 *
 *  触发时机：仅 CAN 模式下通过 main.c 的条件判断才调到这里
 *    UART 模式下此函数不被调用（main.c 中短路返回）
 *    这里不做限速，让 while(1) 全速发出
 *    与 UARTProto_SendPayload 共用 uart_tx_busy 锁互斥
 ******************************************************************************/
void UARTProto_SendVOFA(const float *data, uint8_t count)
{
    __disable_irq();
    if (uart_tx_busy) { __enable_irq(); return; }
    uart_tx_busy = 1;
    __enable_irq();

    if (count > 8)
        count = 8;

    uint16_t data_bytes = count * sizeof(float);   /* 最大 32 */
    memcpy(uart_tx_vofa_buf, data, data_bytes);

    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    memcpy(uart_tx_vofa_buf + data_bytes, tail, 4);

    HAL_UART_Transmit_IT(&huart1, uart_tx_vofa_buf, data_bytes + 4);
}
