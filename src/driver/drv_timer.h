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

/**
 * @brief 系统毫秒时基（1ms GPTMR 中断里累加）
 *
 * 用途：运行监护等周期性逻辑必须按"真实时间"计时，而不能靠自己的调用计数——
 * 一旦主循环被 20kHz 中断饿死、改由 ISR 代跑，调用频率就不再是 1kHz，
 * 按调用次数算的延时会被拉长十几倍，自愈窗口反而错过。
 */
extern volatile uint32_t g_sys_ms;

#endif /* DRV_TIMER_H */
