/**
 * @file    nv_storage.h
 * @brief   非易失存储抽象接口
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 只定义函数指针，不实现任何具体硬件操作。
 *
 * 支持任意长度数据的读写（内部自动处理魔数校验），
 * 更换存储介质（Flash → EEPROM → FRAM）只需新建一个实现文件。
 *
 * 用法：
 *   1. 新建 flash_storage_f1.c / eeprom_storage.c，实现 read/write 函数指针
 *   2. 在 main.c 中将 NvStorage_t 的函数指针指向新实现
 *   3. 其余文件（Foc.c / motor_ctrl.c）一行不用改
 */

#ifndef __NV_STORAGE_H__
#define __NV_STORAGE_H__

#include <stdint.h>

typedef struct NvStorage_t NvStorage_t;

struct NvStorage_t
{
    /**
     * @brief  读取参数块
     * @param  ctx : 实现方的上下文（如 Flash 地址信息）
     * @param  buf : [out] 输出缓冲区
     * @param  len : 期望读取的字节数
     * @return 1 = 读取成功（介质中存有有效数据）
     *         0 = 未写入过 / 数据损坏
     */
    uint8_t (*read)(void *ctx, void *buf, uint32_t len);

    /**
     * @brief  写入参数块（先擦除再编程）
     * @param  ctx : 实现方的上下文
     * @param  buf : 要写入的数据
     * @param  len : 数据长度（字节）
     */
    void    (*write)(void *ctx, const void *buf, uint32_t len);

    void    *ctx;   /**< 上下文指针，如 Flash 地址/页大小结构体 */
};

#endif
