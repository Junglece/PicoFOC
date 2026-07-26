/**
 * @file    drv_adc.c
 * @brief   [已封存] ADC 驱动实现 —— DMA 循环采样 + 原子快照
 *
 * ═══════════════════════════════════════════════════════════════════
 *  封存说明 —— 2026-07-26
 * ═══════════════════════════════════════════════════════════════════
 *
 *  本文件是 drv_adc.h 的实现，与头文件同时封存。详细原因见头文件。
 *
 *  当前状态：main.c 中不再调用 ADC_Init()，本文件所有函数
 *  不会被任何模块调用。s_adc_dma_buf[3] 不再更新（ADC 未启动）。
 *
 *  恢复时：
 *    1. main.c 取消注释 ADC_Init() 调用
 *    2. 确认 ADC 触发配置正确（见 retrospective .md）
 *    3. 本文无需修改
 *
 * ═══════════════════════════════════════════════════════════════════
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ═══════════════════════════════════════════════════════════════════
 *  旧版 vs 新版对照
 * ═══════════════════════════════════════════════════════════════════
 *
 *  旧                         新
 *   ───────────────────────── ─────────────────────────────
 *  adc_dma_buf[3] (全局)     static s_adc_dma_buf[3] (内部)
 *  adc_dma_buf_show[3] (全局) static s_adc_snapshot[3] (内部)
 *  ADC_DMA_Mode_Init()       ADC_Init()
 *  (无)                      ADC_TakeSnapshot()
 *  (无)                      ADC_GetRawCurrents()
 *  (无)                      ADC_ReadPhaseCurrents()
 *
 *  全局变量不再暴露到头文件中，通过 API 函数访问。
 */

#include "drv_adc.h"
#include "adc.h"        /* hadc1, MX_ADC1_Init */

/* ================================================================
 *  内部缓冲区（static，不暴露到头文件）
 * ================================================================ */

/** DMA 实时循环缓冲区（外设→内存，TIM3 TRGO 持续更新） */
static int16_t s_adc_dma_buf[3] = {0};

/** 快照缓冲区（FOC 周期开始时冻结，避免 DMA 写入与 CPU 读取交叉） */
static int16_t s_adc_snapshot[3] = {0};

/* ================================================================
 *  初始化
 *
 *  启动 DMA 循环模式，等待首批数据稳定后取初始快照。
 *  HAL_Delay(1) 在 main() 初始化阶段调用是安全的（SysTick 已运行）。
 * ================================================================ */
void ADC_Init(void)
{
    /* 启动 DMA 循环模式（TIM3 TRGO 触发，无需 CPU 干预） */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma_buf, 3);

    /* 等待首批数据稳定（1ms >> 1 个 PWM 周期 50us，足够 DMA 完成首轮） */
    HAL_Delay(1);

    /* 取初始快照，避免首次读取全 0 */
    ADC_TakeSnapshot();
}

/* ================================================================
 *  快照机制
 *
 *  DMA 在后端持续更新 s_adc_dma_buf（40kHz），
 *  FOC 在前端周期读取（1kHz）。
 *  关中断拷贝防止 CPU 读到一半时 DMA 写入新值。
 * ================================================================ */

void ADC_TakeSnapshot(void)
{
    __disable_irq();
    s_adc_snapshot[0] = s_adc_dma_buf[0];
    s_adc_snapshot[1] = s_adc_dma_buf[1];
    s_adc_snapshot[2] = s_adc_dma_buf[2];
    __enable_irq();
}

void ADC_GetRawCurrents(ADC_PhaseCurrents_t *currents)
{
    currents->raw[0] = s_adc_snapshot[0];
    currents->raw[1] = s_adc_snapshot[1];
    currents->raw[2] = s_adc_snapshot[2];
}

void ADC_ReadPhaseCurrents(ADC_PhaseCurrents_t *currents)
{
    ADC_TakeSnapshot();
    ADC_GetRawCurrents(currents);
}

void ADC_GetRawDMA(int16_t *a, int16_t *b, int16_t *c)
{
    __disable_irq();
    *a = s_adc_dma_buf[0];
    *b = s_adc_dma_buf[1];
    *c = s_adc_dma_buf[2];
    __enable_irq();
}

void ADC_ReadRawFromISR(int16_t *a, int16_t *b, int16_t *c)
{
    /* ISR 中不关中断——硬件已屏蔽同优先级及以下中断。
     * Cortex-M3 的 16-bit 半字读取本身是原子的。 */
    *a = s_adc_dma_buf[0];
    *b = s_adc_dma_buf[1];
    *c = s_adc_dma_buf[2];
}

/* ================================================================
 *  零电流偏移校准（Phase 2 实现）
 *
 *  Phase 1 函数体为空，只做接口预留。
 *  Phase 2 将实现：
 *    1. 暂停 DMA → 软件触发轮询采样 N 次取平均
 *    2. 写入 Flash → 恢复 DMA 模式
 * ================================================================ */
void ADC_CalibrateOffset(int16_t *offset_a, int16_t *offset_b,
                         int16_t *offset_c, uint32_t samples)
{
    (void)offset_a;
    (void)offset_b;
    (void)offset_c;
    (void)samples;
}
