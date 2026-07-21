/**
 * @file    drv_adc.h
 * @brief   ADC 驱动 —— DMA 电流采样（预留）
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * adc_dma_buf[3]      : DMA 循环传输的原始 ADC 数据（三相电流）
 * adc_dma_buf_show[3] : 供外部读取的快照副本
 * ADC_DMA_Mode_Init() : 启动 ADC-DMA 循环采样（ADC 初始化后调用一次即可）
 *
 * 原文件名 bsp_adc.h → drv_adc.h（正名：本文件属于驱动层，非板级支持包）
 */

#ifndef __DRV_ADC_H__
#define __DRV_ADC_H__

#include "stm32f1xx_hal.h"

extern int16_t adc_dma_buf[3];
extern int16_t adc_dma_buf_show[3];

void ADC_DMA_Mode_Init(void);

#endif
