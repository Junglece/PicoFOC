/**
 * @file    flash_storage.h
 * @brief   Flash 参数存储 —— 利用 MCU 内 Flash 保存运行参数
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 实现 nv_storage.h 接口，封装 STM32 HAL Flash 擦写 API。
 *
 * ================================================================
 *  存储布局
 * ================================================================
 *
 *  按页对齐，每页起始布局：
 *     [0 ~ 3] : magic    (uint32_t, 用于校验数据有效性)
 *     [4 ~ N] : 用户数据 (N 字节，由调用方指定)
 *
 *  注意：写入时需先擦除整页（~20ms），擦写期间 CPU 暂停（Flash 总线 stall）。
 *       与正常程序区完全独立，烧录固件不会覆盖此区域。
 *
 * ================================================================
 *  存储模式：全量读写
 * ================================================================
 *
 *  Flash 擦除以扇区为单位（最小 1KB），无法单独擦除一个字段。
 *  因此每次 write 会擦除整扇区再写入，未被写入的字节将恢复为 0xFF（已擦除状态）。
 *
 *  ── 如果要存储多个参数 ──
 *
 *  请定义一个结构体，总是读写完整的结构体：
 *
 *     // 所有参数打包成一个结构体
 *     typedef struct {
 *         float   elec_offset;
 *         float   user_kp;
 *         uint8_t crc;
 *     } MotorParams_t;
 *
 *     MotorParams_t p;
 *
 *     // 读出全部
 *     if (storage->read(ctx, &p, sizeof(p)))
 *     {
 *         // 使用已有的值
 *     }
 *     else
 *     {
 *         // 首次使用，写入默认值
 *         p.elec_offset = 0.0f;
 *         p.user_kp     = 0.05f;
 *         p.crc         = 0;
 *         storage->write(ctx, &p, sizeof(p));
 *     }
 *
 *     // 修改某一个字段 → 写回完整结构体
 *     p.user_kp = 0.08f;
 *     storage->write(ctx, &p, sizeof(p));
 *     // ↑ 整页擦除后写回全部字段，elec_offset 和 crc 不会丢失
 *
 *  ── 永远不要 ──
 *
 *     // ❌ 只写入一个字段的片段
 *     storage->write(ctx, &p.user_kp, sizeof(p.user_kp));
 *     // ↑ 整页擦除后只写了 4 字节，elec_offset 恢复为 0xFF
 *     //   下次读出时 magic 仍可能匹配，但 elec_offset 已损坏
 *
 *  ── 一句话总结 ──
 *
 *     write 什么长度，read 就用什么长度。
 *     先 read 再改再 write，始终读写全量结构体。
 */

#ifndef __FLASH_STORAGE_H__
#define __FLASH_STORAGE_H__

#include "nv_storage.h"

/**
 * @brief  创建 Flash 存储实例（填充 NvStorage_t 接口）
 * @param  sector_addr : Flash 扇区起始地址（如 0x0800FC00）
 * @param  magic       : 魔数 —— 用于区分有效数据和空扇区
 * @param  sector_size : 扇区大小（字节，STM32F1 典型值 1024）
 * @return 已填充 NvStorage_t 结构体
 */
NvStorage_t FlashStorage_Create(uint32_t sector_addr,
                                uint32_t magic,
                                uint32_t sector_size);

#endif
