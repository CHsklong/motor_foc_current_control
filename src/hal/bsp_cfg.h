/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_BSP_CFG_H
#define HPM_BSP_CFG_H

#include "board.h"

/**
 * @brief 板级资源映射
 *
 * 换板子时只需要改这里：把 BOARD_* 重新指向新板子的外设与引脚。
 * 上层（driver / sensor / motor / control / app）一律通过本文件的别名访问硬件，
 * 不直接出现 BOARD_* 宏，从而实现硬件可替换。
 */

/* ---------- PWM ---------- */
#define MOTOR0_BLDCPWM              BOARD_BLDCPWM
#define MOTOR0_PWM_CLOCK_NAME       BOARD_BLDC_MOTOR_CLOCK_SOURCE

/* ---------- ADC 采样缓冲索引 ----------
 * 硬件实际采集 A 相与 C 相，B 相由 Ia+Ib+Ic=0 重构（见 sensor/sens_analog.c）。
 * 这里用"ADC 模块序号"命名，避免与相序混淆。 */
#define ADCU_INDEX                  0
#define ADCV_INDEX                  1

/* ---------- 母线电压分压比 ---------- */
#define MOTOR0_VBUS_DIVIDER         BOARD_BLDC_VBUS_DIVIDER

#endif /* HPM_BSP_CFG_H */
