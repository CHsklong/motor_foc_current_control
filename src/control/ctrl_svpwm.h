/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_CTRL_SVPWM_H
#define HPM_CTRL_SVPWM_H

#include <stdint.h>

/**
 * @brief 控制算法层 —— 开环 SVPWM
 *
 * 用自行递增的虚拟电角度生成旋转电压矢量，不依赖编码器与电流采样。
 * 用途：验证相序、PWM 输出极性与功率级是否正常，是定位"不转"问题的第一步。
 */

/* 开环运行参数 */
#define CTRL_SVPWM_VDC_DEFAULT      (36.0f)    /* 母线电压(V) */
#define CTRL_SVPWM_VREF             (2.0f)     /* 给定电压矢量幅值 */
#define CTRL_SVPWM_TARGET_FREQ      (15.0f)    /* 目标电频率(Hz) */
#define CTRL_SVPWM_RAMP_STEP        (0.0005f)  /* 每步电频率增量（软启动） */

/**
 * @brief 开环步进：由电流环中断按 PWM 频率调用
 */
void ctrl_svpwm_step(void);

/**
 * @brief 获取最近一次计算的三相占空比
 */
float ctrl_svpwm_get_duty(uint8_t phase);

/**
 * @brief 标准七段式 SVPWM 计算（不依赖 MCL，可独立使用）
 */
void ctrl_svpwm_calc(float u_alpha, float u_beta, float vdc,
                     float *duty_a, float *duty_b, float *duty_c);

#endif /* HPM_CTRL_SVPWM_H */
