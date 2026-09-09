/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_SENS_ANALOG_H
#define HPM_SENS_ANALOG_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 传感器接口层 —— 模拟量采样（相电流 + 母线电压）
 *
 * 职责边界：把 ADC 码值换算成"带物理意义"的量，并负责零漂校准。
 * 电流的 Park 变换、PI 调节属于 control 层，不在这里。
 */

/* 零漂校准采样次数（每次间隔 1ms） */
#define SENS_CURRENT_CAL_MS     (200)

/**
 * @brief 零漂校准：在上电时 PWM 未输出、相电流为零的状态下取平均
 * @note 必须在 PWM 输出使能之前调用，否则会采到真实电流导致零点漂移
 */
void sens_current_calibrate(void);

/**
 * @brief 读取 A 相电流（已减去零漂的 ADC 码值）
 */
bool sens_current_read_phase_a(int32_t *value);

/**
 * @brief 读取 B 相电流（已减去零漂的 ADC 码值）
 *
 * @note 硬件实际采样 A 相与 C 相，B 相由 Ia+Ib+Ic=0 重构，
 *       因此必须先调用 sens_current_read_phase_a() 再调用本函数。
 */
bool sens_current_read_phase_b(int32_t *value);

/**
 * @brief 读取母线电压
 * @return 母线电压(V)；读取失败返回 -1.0f
 */
float sens_vbus_read(void);

/* 零漂校准值（J-Scope 可观测） */
extern volatile uint32_t adc_u_midpoint;
extern volatile uint32_t adc_v_midpoint;

#endif /* HPM_SENS_ANALOG_H */
