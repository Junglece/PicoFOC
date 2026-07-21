/**
 * @file    drv_adc.c
 * @brief   ADC 驱动 —— DMA 电流采样（预留）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 提供 3 通道 ADC DMA 循环采样，结果存入 adc_dma_buf[]。
 * 当前 FOC 控制环中未使用（编码器模式，不需要电流采样）。
 * 原文件名 bsp_adc.c → drv_adc.c（正名：本文件属于驱动层，非板级支持包）
 */

#include "drv_adc.h"
#include "adc.h"

int16_t adc_dma_buf[3]      = {0};
int16_t adc_dma_buf_show[3] = {0};

/**
 * @brief  启动 ADC DMA 循环采样（3 通道）
 */
void ADC_DMA_Mode_Init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, 3);
}
