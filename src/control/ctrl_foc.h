/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file ctrl_foc.h
 * @brief FOC 控制层：20kHz 电流环中断与各环调度
 */
#ifndef CTRL_FOC_H
#define CTRL_FOC_H

#include "hw_map.h"

/** 三相采样电流 A（由 ADC 中断以 20kHz 刷新） */
extern float g_ia;
extern float g_ib;
extern float g_ic;

/** 机械角速度 rad/s */
extern float speed_rad_s;

/** 转速 r/min */
extern float rpm;

/** 过流故障引脚电平（调试用） */
extern float fault_level;

/**
 * @brief 电流环中断入口（20kHz，由 ADC 转换完成触发）
 *
 * 职责：编码器角度更新 → 电流采样 → 模式调度（开环 SVPWM / FOC 闭环）
 */
void isr_adc(void);

#endif /* CTRL_FOC_H */
