/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file hal_bsp.h
 * @brief 板级支持层：引脚/时钟初始化与延时适配
 *
 * 只做"板子相关的脏活"（引脚复用、时钟门控），不含任何控制语义。
 * 具体外设的功能配置在 driver 层。
 */
#ifndef HAL_BSP_H
#define HAL_BSP_H

#include <stdint.h>

/**
 * @brief 板级引脚与时钟初始化
 *
 * 必须在任何外设驱动初始化之前调用。
 */
void bsp_init(void);

/**
 * @brief MCL 中间件需要的微秒级延时回调
 * @param tick 微秒数
 */
void mcl_user_delay_us(uint64_t tick);

#endif /* HAL_BSP_H */
