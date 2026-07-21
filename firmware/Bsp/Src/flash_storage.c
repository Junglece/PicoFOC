/**
 * @file    flash_storage.c
 * @brief   STM32 Flash 参数存储实现 —— 实现 NvStorage_t 接口
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * STM32F103C8T6 典型用法：
 *   0x08000000 ~ 0x0800FBFF : 主程序区（63 页，每页 1KB）
 *   0x0800FC00 ~ 0x0800FFFF : 参数存储区（第 64 页，1KB）
 *
 * 存储布局：
 *   [0 ~ 3] : magic  (uint32_t)
 *   [4 ~ N] : 用户参数 (N 字节)
 *
 * 任意长度数据均可存储，由工厂函数调用者指定扇区地址和大小。
 */

#include "flash_storage.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ================================================================
 *  上下文结构体 —— 由 FlashStorage_Create 填充
 * ================================================================ */
typedef struct {
    uint32_t sector_addr;       /**< Flash 扇区起始地址          */
    uint32_t magic;             /**< 魔数 —— 数据有效性校验      */
    uint32_t sector_size;       /**< 扇区大小 (字节)             */
} FlashCtx_t;

/* ================================================================
 *  FlashStorage_Read —— NvStorage_t.read 实现
 *
 *  从 Flash 扇区起始处读取魔数校验，通过后拷贝后续数据到 buf。
 *  返回 1 表示有效，0 表示未写入或数据损坏。
 * ================================================================ */
static uint8_t FlashStorage_Read(void *ctx, void *buf, uint32_t len)
{
    FlashCtx_t *fc = (FlashCtx_t *)ctx;
    const uint8_t *flash = (const uint8_t *)fc->sector_addr;

    /* 检查魔数 */
    uint32_t stored_magic;
    memcpy(&stored_magic, flash, sizeof(stored_magic));
    if (stored_magic != fc->magic)
        return 0;

    /* 跳过前 4 字节魔数，拷贝数据 */
    memcpy(buf, flash + sizeof(stored_magic), len);
    return 1;
}

/* ================================================================
 *  FlashStorage_Write —— NvStorage_t.write 实现
 *
 *  流程：
 *    1. 解锁 Flash 控制寄存器
 *    2. 擦除整页
 *    3. 依次写入 magic（4 字节） + 用户数据（N 字节，按 word 对齐）
 *    4. 锁定 Flash
 *
 *  注意：擦除约 20ms，期间 Flash 总线 stall，所有代码暂停执行。
 * ================================================================ */
static void FlashStorage_Write(void *ctx, const void *buf, uint32_t len)
{
    FlashCtx_t *fc = (FlashCtx_t *)ctx;

    /* ---- 解锁 Flash ---- */
    HAL_FLASH_Unlock();

    /* ---- 擦除整页 ---- */
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_PAGES;
    erase.PageAddress  = fc->sector_addr;
    erase.NbPages      = fc->sector_size / 1024;
    uint32_t page_error = 0;
    HAL_FLASHEx_Erase(&erase, &page_error);

    /* ---- 写入魔数 ---- */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, fc->sector_addr, fc->magic);

    /* ---- 写入用户数据（按 4 字节 word 写入） ---- */
    const uint32_t *src = (const uint32_t *)buf;
    uint32_t addr = fc->sector_addr + 4;
    uint32_t words = (len + 3) / 4;            /* 向上取整到 word 边界 */

    for (uint32_t i = 0; i < words; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, src[i]);
    }

    /* ---- 锁定 Flash ---- */
    HAL_FLASH_Lock();
}

/* ================================================================
 *  FlashStorage_Create —— 工厂函数
 *
 *  用法：
 *     NvStorage_t flash = FlashStorage_Create(0x0800FC00, 0xF0C0F0C0, 1024);
 *     float offset;
 *     flash.read(flash.ctx, &offset, sizeof(offset));    // 读取
 *     flash.write(flash.ctx, &offset, sizeof(offset));   // 写入
 * ================================================================ */
NvStorage_t FlashStorage_Create(uint32_t sector_addr,
                                uint32_t magic,
                                uint32_t sector_size)
{
    static FlashCtx_t ctx_storage;

    ctx_storage.sector_addr  = sector_addr;
    ctx_storage.magic        = magic;
    ctx_storage.sector_size  = sector_size;

    NvStorage_t storage;
    storage.read  = FlashStorage_Read;
    storage.write = FlashStorage_Write;
    storage.ctx   = &ctx_storage;

    return storage;
}
