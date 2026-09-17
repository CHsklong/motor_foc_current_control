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

/**
 * @brief 位置环初始化（必须在 adc_isr_enable() 之前调用）
 *
 * 换算位置环分频比，并把 MCL 的位置环 PID 换成带 S 曲线速度前馈的包装。
 */
void ctrl_pos_init(void);

/**
 * @brief 启动一段位置运动
 * @param delta 相对当前位置的位移 rad（有符号）
 *
 * 启用 S 曲线时按 v/a/j 约束规划七段轨迹，位置环跟踪移动的目标点；
 * 未启用时行为与原来一致（直接给终点 + 接近限速）。
 */
void ctrl_pos_start(float delta);

#endif /* CTRL_FOC_H */
