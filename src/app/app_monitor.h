/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_APP_MONITOR_H
#define HPM_APP_MONITOR_H

/**
 * @brief 应用层 —— 运行观测（J-Scope / 串口）
 *
 * 集中管理所有调试观测变量。这样"观测"不会散落到各层污染业务代码，
 * 也方便一次性关闭（发布版本可不调用 app_monitor_update）。
 */

void app_monitor_update(void);

/* ---------------- 观测变量（J-Scope 可直接抓符号） ---------------- */
extern volatile float g_ia;             /* A 相电流(A) */
extern volatile float g_ib;             /* B 相电流(A)（由 A+C 重构） */
extern volatile float g_ic;             /* C 相电流(A)（由 -Ia-Ib 推算） */
extern volatile float g_vbus;           /* 母线电压(V) */
extern volatile float g_raw_u;          /* U 相 ADC 原始值 */
extern volatile float g_raw_v;          /* V 相 ADC 原始值 */
extern volatile float g_theta_initial;  /* 零点偏移（机械弧度） */
extern volatile float g_probe_keep;     /* 占位：防止链接器优化掉未使用探针 */

#endif /* HPM_APP_MONITOR_H */
