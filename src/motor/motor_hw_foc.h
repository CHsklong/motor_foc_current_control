/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file motor_hw_foc.h
 * @brief 硬件 FOC / 混合环路加速模块（CLC + VSC + QEO + TRGMUX）
 *
 * 仅在定义 HW_CURRENT_FOC_ENABLE 或 MCL_HARDWARE_HYBRID_LOOP_ENABLE 时参与编译，
 * 当前工程走软件 FOC，这些接口不会被链接进来。
 */
#ifndef MOTOR_HW_FOC_H
#define MOTOR_HW_FOC_H

#include "hw_map.h"

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
/**
 * @brief 浮点 d/q → CLC 硬件定点格式
 *
 * 输入 ±15.0 → 输出 0x80000000~0x7FFFFFFF
 */
void clc_convert_input(float d, float q, uint32_t *d_hardware, uint32_t *q_hardware);

/**
 * @brief CLC 硬件定点格式 → 浮点 d/q
 *
 * 输入 0x80000000~0x7FFFFFFF → 输出 ±7.0（与输入的 15.0 不同，为系统增益考虑）
 */
void clc_convert_output(uint32_t ud_hardware, uint32_t uq_hardware, float *ud, float *uq);
#endif

#if defined(HW_CURRENT_FOC_ENABLE)
void trigmux_init_1(void);
void trigmux_init_2(void);
void trigmux_init_3(void);
void vsc_init(void);
void clc_init(void);
void qeov2_init(void);
void motor0_clc_set_currentloop_value(mcl_loop_chn_t chn, int32_t val);
int32_t motor0_clc_float_convert_clc(float realdata);
#endif

#endif /* MOTOR_HW_FOC_H */
