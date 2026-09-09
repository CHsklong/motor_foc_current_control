/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_DRV_TIMER_H
#define HPM_DRV_TIMER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 外设驱动层 —— 周期定时器（1ms 滴答）
 *
 * 用于故障检测等周期性后台任务。本层不知道"故障检测"是什么，
 * 只提供滴答回调注册 —— 具体行为由 control 层注册。
 */

/**
 * @brief 初始化 1ms 周期定时器并注册中断
 */
void drv_timer_init(void);

/**
 * @brief 注册滴答回调（在定时器中断上下文中调用）
 * @param cb 回调函数，NULL 表示取消注册
 */
void drv_timer_register_tick_callback(void (*cb)(void));

#endif /* HPM_DRV_TIMER_H */
