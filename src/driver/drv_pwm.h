/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file drv_pwm.h
 * @brief 三相 PWM 驱动：周期/死区/故障保护/DMA 占空比更新
 */
#ifndef DRV_PWM_H
#define DRV_PWM_H

#include "hw_map.h"

/** PWM 比较值影子缓冲区，由 DMA 在周期 75% 处搬进 CMP 寄存器 */
extern volatile uint32_t pwm_buff[6];

/** PWM 模块时钟频率，main() 初始化时填入，PWM_RELOAD 宏依赖它 */
extern int32_t motor_clock_hz;

/**
 * @brief PWM 初始化（三相六路互补 + 死区 + DMA + 硬件过流保护）
 */
void pwm_init(void);

/**
 * @brief 设置某一相占空比
 * @param chn  A/B/C 相
 * @param duty 0~1
 */
hpm_mcl_stat_t pwm_duty_set(mcl_drivers_channel_t chn, float duty);

/** 解除 PWM 强制输出，恢复正常调制 */
hpm_mcl_stat_t enable_all_pwm_output(void);

/** 强制六路输出为低，关断电机（过流/故障/停机时调用） */
hpm_mcl_stat_t disable_all_pwm_output(void);

/**
 * @brief 查询 PWM 当前是否处于正常调制状态
 * @return 1=已使能（正常调制），0=被 disable_all_pwm_output() 强制关断
 *
 * MCL 的 detect 一旦判故障就回调 disable_all_pwm_output()，
 * 电机"突然不动"但代码里没有任何提示。把这个状态挂到 J-Scope 上，
 * 一眼就能区分"控制环没出力"和"出力了但被保护关断"。
 */
bool pwm_output_is_enabled(void);

/**
 * @brief 查询 PWM 的硬件过流故障是否被锁存
 * @return true = SR 的 FAULT 位已置，六路输出被硬件强制拉低
 *
 * 【为什么必须有这个】外部过流故障源（PA10，低有效）在 pwm_init() 里使能，
 * 且配成 fault_recovery_on_fault_clear —— 锁存后不会自动恢复，必须软件写
 * FAULTCLR。冷上电时电流采样运放/比较器还在上电爬升，输出可能是低电平，
 * 于是故障在 PWM 启动的瞬间就被锁住，六路输出永远是 0：
 * CPU 正常、20kHz 中断正常、心跳正常、g_pwm_enabled 仍显示 1，
 * 唯独电机不出力，而且现象随机（取决于上电瞬间比较器的状态）。
 */
bool pwm_fault_is_latched(void);

/**
 * @brief 清除 PWM 的硬件故障锁存，恢复六路调制输出
 *
 * 只允许在【上电初始化阶段】调用。运行期间若真的过流，锁存是保护，不要清。
 */
void pwm_fault_clear(void);

#endif /* DRV_PWM_H */
