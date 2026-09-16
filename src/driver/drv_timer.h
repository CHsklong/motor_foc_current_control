/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file drv_timer.h
 * @brief GPTMR 驱动：1ms 周期中断，用于 MCL 故障检测轮询
 */
#ifndef DRV_TIMER_H
#define DRV_TIMER_H

#include "hw_map.h"

/**
 * @brief 初始化 1ms 定时器并使能比较中断
 */
void timer_init(void);

#endif /* DRV_TIMER_H */
