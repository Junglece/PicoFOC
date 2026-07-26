/**
 * @file    drv_adc.h
 * @brief   [已封存] ADC 驱动 —— DMA 循环采样 + 原子快照
 *
 * ═══════════════════════════════════════════════════════════════════
 *  封存说明 —— 2026-07-26
 * ═══════════════════════════════════════════════════════════════════
 *
 *  本文件是为 Phase 1 电流 ADC 电流采样开发的驱动。
 *  核心功能：
 *    1. ADC 连续自由转换 + DMA CIRCULAR（3 通道 ~98kHz）
 *    2. 关中断快照拷贝（ADC_TakeSnapshot）
 *    3. ISR 直接读 DMA 缓冲（ADC_ReadRawFromISR）
 *    4. 零电流校准接口预留（ADC_CalibrateOffset）
 *
 *  放弃原因：小电机空载/轻载下 MP6541 SOx 信噪比不足以做可靠
 *  电流反馈，且 ADC 自由转换无法与 PWM 同步 → 电流波形削波。
 *
 *  当前状态：main.c 中不再调用 ADC_Init()，本驱动所有函数
 *  不会被执行。保留代码供后续硬件升级后复用。
 *
 *  恢复使用时需注意：
 *    1. main.c 中恢复 ADC_Init() 调用
 *    2. TIM3 CC4 中断配置需重新启用
 *    3. 如果改回硬件触发，必须在 USER CODE 加 CC4E=1
 *    见 memory/current-sensing-retrospective.md
 *
 * ═══════════════════════════════════════════════════════════════════
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * ═══════════════════════════════════════════════════════════════════
 *  设计原则
 * ═══════════════════════════════════════════════════════════════════
 *
 *  1. 所有内部变量 static，不暴露 extern 全局变量
 *  2. ADC 连续自由转换（ContinuousConvMode=ENABLE），由 TIM3 CC4 中断
 *     在 V0 时刻（20kHz）读取 DMA 缓冲区最新值
 *  3. DMA CIRCULAR 模式持续更新内部缓冲区（~285kHz 扫描率）
 *  4. 快照机制：关中断拷贝 DMA 缓冲区，避免竞态（兼容旧接口）
 *  5. ISR 专用接口 ADC_ReadRawFromISR() 不关中断（ISR 中已屏蔽）
 *  6. FOC 控制环通过 FOC_ReadCurrents() 获取 20kHz 重构后的 Id/Iq
 *
 *  ═══════════════════════════════════════════════════════════════
 *  数据流
 *  ═══════════════════════════════════════════════════════════════
 *
 *    ADC1 连续自由转换（~285kHz）
 *      → DMA1_CH1 CIRCULAR → s_adc_dma_buf[3]（持续更新）
 *        → TIM3 CC4 @ V0 (20kHz) → ADC_ReadRawFromISR()
 *          → 占空比排序，取最低两相电流 → 重构 Ia/Ib/Ic
 *            → Motor.current_raw[3]
 *              → FOC_ReadCurrents (1kHz) → Clarke → Park → Id/Iq
 *
 *  快照路径（兼容旧接口）：
 *    ADC_TakeSnapshot() 关中断拷贝 → s_adc_snapshot[3]
 *      → ADC_GetRawCurrents() 读快照
 *
 *  竞态分析（快照路径）：关中断拷贝 6 字节约 0.08us，DMA 每 3.5us 更新一次。
 *  碰撞概率 ~2.3%，即使发生也只错一个通道的一个采样周期值。
 */

#ifndef __DRV_ADC_H__
#define __DRV_ADC_H__

#include <stdint.h>

/**
 * @brief 三相 ADC 原始值快照
 *
 * raw[0] = SOA (PA0 / ADC1_IN0)
 * raw[1] = SOB (PA1 / ADC1_IN1)
 * raw[2] = SOC (PA2 / ADC1_IN2)
 *
 * 由 ADC_TakeSnapshot() 从 DMA 缓冲区原子拷贝。
 */
typedef struct {
    int16_t raw[3];     /**< ADC 原始值 (0~4095, int16_t 可负偏移运算) */
} ADC_PhaseCurrents_t;

/**
 * @brief 初始化 ADC DMA 循环采样（PWM 触发模式）
 *
 * 配置：
 *   - 触发源: TIM3 TRGO (Update Event)
 *   - 传输:   DMA1_CH1 CIRCULAR → 内部缓冲区
 *
 * 前置条件: MX_ADC1_Init() 和 MX_TIM3_Init() 已执行
 *
 * 调用位置: main.c 初始化流程（TIM_RegisterCallback 之前）
 */
void ADC_Init(void);

/**
 * @brief 取三相电流快照（原子拷贝: DMA 缓冲区 → 快照缓冲区）
 *
 * 关中断保护（约 0.08us，不影响 FOC 1ms 周期）
 * 调用位置: FOC_ReadCurrents() 第一行
 */
void ADC_TakeSnapshot(void);

/**
 * @brief 读取快照中的三相电流原始值
 *
 * @param currents [out] 快照数据
 */
void ADC_GetRawCurrents(ADC_PhaseCurrents_t *currents);

/**
 * @brief 一步完成快照 + 读取（组合接口）
 *
 * 等价于 ADC_TakeSnapshot() + ADC_GetRawCurrents()
 *
 * @param currents [out] 快照数据
 */
void ADC_ReadPhaseCurrents(ADC_PhaseCurrents_t *currents);

/**
 * @brief 读取 DMA 循环缓冲区的最新三相原始值（调试用）
 *
 * 不在 ISR 中调用时使用，关中断保护 16-bit 半字读取。
 * 用于 VOFA+ 调试通道，直接观察 ADC 是否采到有效值。
 *
 * @param a [out] Phase A 原始值 (PA0)
 * @param b [out] Phase B 原始值 (PA1)
 * @param c [out] Phase C 原始值 (PA2)
 */
void ADC_GetRawDMA(int16_t *a, int16_t *b, int16_t *c);

/**
 * @brief ISR 中直接读取 DMA 缓冲区的最新三相原始值（不关中断）
 *
 * 在 TIM3_IRQHandler（20kHz V0 同步）中调用，直接从 DMA 缓冲区拷贝。
 * 不关中断，因为 ISR 执行期间中断已由硬件屏蔽，且 16-bit 读在
 * Cortex-M3 上是原子操作。
 *
 * @param a [out] Phase A 原始值 (PA0)
 * @param b [out] Phase B 原始值 (PA1)
 * @param c [out] Phase C 原始值 (PA2)
 */
void ADC_ReadRawFromISR(int16_t *a, int16_t *b, int16_t *c);

/**
 * @brief 零电流偏移校准（Phase 2 实现）
 *
 * Phase 1 函数体为空，仅声明接口。
 * Phase 2 将实现：暂停 DMA → 软件触发轮询采样 N 次取平均 → 恢复 DMA
 */
void ADC_CalibrateOffset(int16_t *offset_a, int16_t *offset_b,
                         int16_t *offset_c, uint32_t samples);

#endif /* __DRV_ADC_H__ */
