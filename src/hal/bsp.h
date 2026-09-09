/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_BSP_H
#define HPM_BSP_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 硬件抽象层 —— 引脚、时钟、基础延时
 *
 * 本层是唯一允许直接调用 board_init() / init_*_pins() 的地方。
 * 向上只暴露语义化接口（"初始化编码器接口"而不是"配置 PA26 复用为 SPI1_CS"）。
 */

/* 系统级 */
void bsp_init(void);

/* 引脚与时钟分组初始化 */
void bsp_encoder_interface_init(void);   /* SPI1 引脚 + 时钟（MT6835） */
void bsp_adc_pins_init(void);            /* 相电流 / 母线电压采样引脚 */
void bsp_pwm_pins_init(void);            /* 三相六路 PWM 输出引脚 */
void bsp_acmp_pins_init(void);           /* 比较器（过流保护）引脚 */

/* 基础延时（busy-wait，仅用于初始化与非实时路径） */
void bsp_delay_ms(uint32_t ms);
void bsp_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* HPM_BSP_H */
