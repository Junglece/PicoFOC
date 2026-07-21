/**
 * @file    drv_can.c
 * @brief   CAN 驱动实现（纯收发，不解析协议）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 核心改动：
 *   1. 中断回调中不再硬编码协议解析
 *   2. 收帧后调用已注册的回调函数
 *   3. 滤波器保持原设置（全通）
 * 原文件名 bsp_can.c → drv_can.c（正名：本文件属于驱动层，非板级支持包）
 */

#include "drv_can.h"
#include "main.h"
#include "can.h"

/* ===== 内部变量 ===== */
static CAN_RxHeaderTypeDef  rx_header;
static CAN_RxCallback_t     rx_callback = NULL;   /**< 注册的 RX 回调 */

/**
 * @brief  初始化 CAN（配置滤波器 + 启动）
 */
void CAN_Init(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterBank           = 1;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_16BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x00;
    filter.FilterMaskIdLow      = 0x00;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;

    HAL_CAN_ConfigFilter(&hcan, &filter);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan);
}

/**
 * @brief  注册 CAN 接收回调
 */
void CAN_RegisterRxCB(CAN_RxCallback_t cb)
{
    rx_callback = cb;
}

/**
 * @brief  发送 CAN 消息（8 字节数据）
 */
uint8_t CAN_SendMsg(uint16_t id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId               = id;
    tx_header.ExtId               = 0x00;
    tx_header.RTR                 = CAN_RTR_DATA;
    tx_header.IDE                 = CAN_ID_STD;
    tx_header.DLC                 = 8;
    tx_header.TransmitGlobalTime  = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox);
}

/* ================================================================
 *  HAL CAN FIFO0 收帧中断回调
 *
 *  不做协议解析，只转发给注册的回调。
 * ================================================================ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    uint8_t data[8] = {0};

    if (hcan_ptr == &hcan)
    {
        HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rx_header, data);
    }

    /* 转发给协议解析层 */
    if (rx_callback != NULL)
    {
        rx_callback(rx_header.StdId, data);
    }
}
