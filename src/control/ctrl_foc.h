/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_CTRL_FOC_H
#define HPM_CTRL_FOC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 控制算法层 —— FOC 电流环调度
 *
 * 本层决定"什么时候跑什么算法"，算法本体由 HPM MCL 提供。
 * ADC 采样完成中断是本系统的实时心跳：20kHz，电流环与编码器更新都挂在这里。
 */

/* 运行模式（编译期选择，ISR 内不产生额外分支开销）
 *   电流闭环：CTRL_FOC_CURRENT_MODE = 1
 *   开环 SVPWM：CTRL_FOC_SVPWM_MODE = 1（调试用，验证相序与 PWM 输出） */
#ifndef CTRL_FOC_CURRENT_MODE
#define CTRL_FOC_CURRENT_MODE   (1)
#endif
#ifndef CTRL_FOC_SVPWM_MODE
#define CTRL_FOC_SVPWM_MODE     (0)
#endif

/**
 * @brief 注册 ADC 中断并使能
 */
void ctrl_foc_init(void);

/**
 * @brief 允许 / 禁止在中断里更新编码器角度
 *
 * 初始角对齐期间必须关闭 —— 对齐要求电角度冻结为 0（等价于 force_theta(0)），
 * 否则中断刷新真实角度后，转子会被恒力矩拖走，theta_initial 记到随机位置。
 *
 * @param enable true=由 ADC 中断按电流环频率刷新角度
 */
void ctrl_foc_encoder_update_enable(bool enable);

#endif /* HPM_CTRL_FOC_H */
