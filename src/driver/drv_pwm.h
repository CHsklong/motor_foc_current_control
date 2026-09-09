/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_DRV_PWM_H
#define HPM_DRV_PWM_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 外设驱动层 —— 三相 PWM 输出
 *
 * 对外只暴露"通道 + 占空比"的语义化接口，不暴露 mcl_drivers_channel_t 之类的
 * 中间件类型 —— 与 MCL 的适配放在 motor 层，从而保持本层可被非 MCL 方案复用。
 */

#define DRV_PWM_FREQUENCY_HZ        (20000U)
#define DRV_PWM_DEAD_AREA_TICK      (100U)

typedef enum {
    drv_pwm_chn_a = 0,
    drv_pwm_chn_b,
    drv_pwm_chn_c,
} drv_pwm_chn_t;

/**
 * @brief 获取并缓存 PWM 模块时钟频率
 * @note 必须在 drv_pwm_init() 与使用 drv_pwm_get_reload() 之前调用
 */
void drv_pwm_clock_init(void);

/**
 * @brief 初始化 PWM（三相六路互补 + 死区 + DMA 占空比搬运 + 硬件过流保护）
 */
void drv_pwm_init(void);

/**
 * @brief 获取 PWM 重载值（周期）
 */
uint32_t drv_pwm_get_reload(void);

/**
 * @brief 获取 PWM 频率
 */
uint32_t drv_pwm_get_frequency(void);

/**
 * @brief 设置某相占空比
 * @param chn  相通道
 * @param duty 占空比 0.0 ~ 1.0
 */
void drv_pwm_set_duty(drv_pwm_chn_t chn, float duty);

/**
 * @brief 使能 PWM 输出（解除软件强制）
 */
void drv_pwm_enable_output(void);

/**
 * @brief 关闭 PWM 输出（软件强制六路输出无效电平）
 */
void drv_pwm_disable_output(void);

#endif /* HPM_DRV_PWM_H */
