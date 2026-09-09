/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_CTRL_ALIGN_H
#define HPM_CTRL_ALIGN_H

/**
 * @brief 控制算法层 —— 初始角（零点偏移）对齐
 *
 * 对齐的目的：测出"编码器机械零位"与"电机 A 相绕组轴线"之间的夹角 theta_initial。
 *
 * 原理：强制电角度为 0，给定 d 轴电流把转子吸到 A 相轴线，此时读一次编码器即为偏移值。
 * 三阶段设计：大电流吸合 → 加 q 轴扰动脱离静摩擦 → 小电流稳定 → 读数。
 */

/**
 * @brief 执行三阶段对齐
 *
 * @note 调用前必须已使能控制环（否则无电流输出，转子不吸合，
 *       读到的 theta_initial 是随机位置）。
 */
void ctrl_align_run(void);

#endif /* HPM_CTRL_ALIGN_H */
