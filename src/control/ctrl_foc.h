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
 * @brief 注册"主循环停滞"时的代办监护函数
 * @param hook 由应用层实现的监护函数；NULL = 不启用
 *
 * main 一旦被 20kHz 中断饿死就完全得不到 CPU，而负责解除 PWM 锁存、恢复输出的
 * 运行监护原本跑在 main 里 → 等于把唯一的救援通道堵死了。
 * 注册后，20kHz 中断每 20ms 检查一次 main 心跳，发现停滞就代为调用该钩子，
 * 保证自愈逻辑在最坏情况下依然执行得到。
 */
void ctrl_set_keeper_hook(void (*hook)(void));

#endif /* CTRL_FOC_H */
