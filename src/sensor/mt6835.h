/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_MT6835_H
#define HPM_MT6835_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 传感器接口层 —— MT6835 绝对值磁编码器芯片协议
 *
 * 只实现芯片级通信（寄存器读写、角度拼接、通信自检），
 * 不涉及 FOC 语义（电角度、极对数）—— 那些在 sens_encoder。
 *
 * 角度分辨率：21 bit，满量程 2097152。
 */

#define MT6835_CMD_READ_REG     0x3     /* 4-bit 命令 '0011' */
#define MT6835_REG_ANGLE_MSB    0x003
#define MT6835_REG_ANGLE_MID    0x004
#define MT6835_REG_ANGLE_LSB    0x005
#define MT6835_REG_USER_ID      0x001   /* 用户 ID 寄存器（可用于验证通信） */

#define MT6835_ANGLE_RESOLUTION (2097152U)   /* 2^21 */

/**
 * @brief 芯片初始化：执行通信自检并读取 USER_ID
 *
 * @note 必须在电机出力之前、轴静止时调用 —— 自检要求轴静止。
 *       自检结果写入 g_seq3_ok（1=单次连读生效）。
 */
void mt6835_init(void);

/**
 * @brief 读取 21bit 原始角度
 * @param[out] raw 0 ~ 2097151
 * @return true 成功；false SPI 连续失败
 */
bool mt6835_read_angle_raw(uint32_t *raw);

/* ---------------- 调试观测变量（J-Scope 可直接抓符号） ---------------- */
extern volatile uint8_t  g_seq3_ok;      /* 1=单次连读生效(约5us)，0=回退三次单字节读(约25us) */
extern volatile uint8_t  g_spi_err;      /* 0=正常, 3=SPI 传输失败 */
extern volatile uint8_t  g_byte0;        /* 角度寄存器 0x003 */
extern volatile uint8_t  g_byte1;        /* 角度寄存器 0x004 */
extern volatile uint8_t  g_byte2;        /* 角度寄存器 0x005 */
extern volatile uint8_t  g_user_id;      /* USER_ID(0x001)，用于验证通信是否建立 */
extern volatile uint8_t  g_rx_byte;      /* 最近一次单字节读的返回值 */
extern volatile uint32_t g_spi_fail_cnt; /* SPI 连续失败计数 */

#endif /* HPM_MT6835_H */
