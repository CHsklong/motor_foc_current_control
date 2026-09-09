/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_APP_CFG_H
#define HPM_APP_CFG_H

/**
 * @brief 应用层配置 —— 运行时策略与标定值
 *
 * 这里放"这台设备要怎么跑"的决策，不涉及算法与外设细节。
 */

/* ---------------- 电流给定 ---------------- */
#define APP_CURRENT_Q_REF       (0.15f)   /* q 轴电流给定(A)，即转矩电流 */
#define APP_CURRENT_D_REF       (0.0f)    /* d 轴电流给定(A)，即励磁电流 */

/* ---------------- 上电跳过对齐 ----------------
 *
 * 装配固定后 theta_initial 就是常数（程序在 flash 里，掉电保持），
 * 不必每次上电重新对齐，转子也就不会"顿"一下。
 *
 * 取值步骤：
 *   1) 保持 ENC_SKIP_ALIGN = 0，烧录运行一次
 *   2) J-Scope 读 g_theta_initial（单位：机械弧度）
 *   3) 把读到的值填进 ENC_THETA_INITIAL，再把 ENC_SKIP_ALIGN 改成 1，重新烧录
 *
 * ⚠️ 4 对极电机有 4 个相差 π/2(1.5708) 的等效值，填【任意一个】都可以 —— 电角度完全等价。
 * ⚠️ 换电机 / 重装编码器 / 编码器与轴打滑后必须重新测一次。
 * ⚠️ 值填 0 或漏填会自动回退到上电对齐，不会用错角度。
 */
#define ENC_SKIP_ALIGN          (0)
#define ENC_THETA_INITIAL       (4.9862f)   /* 实测的初始机械弧度 */

/* ---------------- 主循环周期 ---------------- */
#define APP_MAIN_LOOP_DELAY_MS  (1)

#endif /* HPM_APP_CFG_H */
