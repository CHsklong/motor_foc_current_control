/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file app_monitor.h
 * @brief 应用层低速监控（主循环 1kHz）
 *
 * 只放"刷新慢、允许被中断打断"的观测量；电流/转速等 20kHz 快量在中断里刷新。
 */
#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#include "hw_map.h"

/**
 * @brief 主循环调用：刷新低速观测量
 *
 * 注意：母线电压 read_vbus() 内部有 20us 忙等，禁止放进 20kHz 中断。
 */
void app_monitor_update(void);

#endif /* APP_MONITOR_H */
