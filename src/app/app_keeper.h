/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file app_keeper.h
 * @brief 运行监护：让"单向锁死"的故障有恢复通道
 *
 * 解决的问题见 app_cfg.h 里【运行监护】那一节的说明：PWM 硬件故障锁存、
 * MCL detect 关断输出、母线电压一次性读取，这三处都没有恢复路径，
 * 一旦在上电爬升期被误触发，表现就是"第一次能转、之后怎么上电都不转"。
 *
 * 安全边界：只有在 MCL 从未判定过真实故障（g_fault_src == 0）时才动作。
 * 真实故障一律不自愈，停机等待人工确认，避免把已经出问题的功率级反复重启。
 */
#ifndef APP_KEEPER_H
#define APP_KEEPER_H

#include "hw_map.h"

/**
 * @brief 在主循环里每毫秒调一次的监护
 *
 * 内部自带启动延时与动作冷却，可以直接无脑调用。
 */
void app_keeper_update(void);

#endif /* APP_KEEPER_H */
