/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file ctrl_svpwm.h
 * @brief SVPWM 调制与开环拖动
 */
#ifndef CTRL_SVPWM_H
#define CTRL_SVPWM_H

#include "hw_map.h"

/** 母线电压（V），SVPWM 归一化用 */
extern float Vdc;

/** 开环给定电压矢量幅值（归一化到 Vdc 的比例） */
extern float Vref;

/** 三相占空比输出（MCL 结构体形式） */
extern mcl_control_svpwm_duty_t svpwm_duty;

/** 三相占空比观测值（J-Scope） */
extern float svpwma;
extern float svpwmb;
extern float svpwmc;

/**
 * @brief 标准七段式 SVPWM：αβ 电压 → 三相占空比
 *
 * @param u_alpha α 轴电压
 * @param u_beta  β 轴电压
 * @param Vdc     母线电压
 * @param duty_a/b/c 输出占空比 0~1
 */
void svpwm(float u_alpha, float u_beta, float Vdc, float *duty_a, float *duty_b, float *duty_c);

/**
 * @brief 开环拖动一个步进（SVPWM_MODE=1 时由电流环中断调用）
 *
 * 内含软启动：电频率按 FREQ_RAMP_STEP 爬升到目标频率，避免启动冲击。
 */
void svpwm_openloop_step(void);

#endif /* CTRL_SVPWM_H */
