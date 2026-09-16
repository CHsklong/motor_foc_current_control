/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file sens_analog.h
 * @brief 模拟量传感器层：相电流换算、母线电压读取（MCL 回调）
 */
#ifndef SENS_ANALOG_H
#define SENS_ANALOG_H

#include "hw_map.h"

/**
 * @brief MCL 回调：取某一路模拟量的物理值
 *
 * analog_a_current：U 相电流（A）；
 * analog_b_current：由 A、C 两路重构的 B 相电流（硬件实际采的是 A 相与 C 相）。
 */
hpm_mcl_stat_t adc_value_get(mcl_analog_chn_t chn, int32_t *value);

/**
 * @brief MCL 回调：更新采样点位置（本工程采样点由 PWM 比较器固定，无需调整）
 */
hpm_mcl_stat_t analog_update_sample_location(mcl_analog_chn_t chn, uint32_t tick);

/**
 * @brief 软件触发一次母线电压采样并返回电压值
 * @return 母线电压 V；读取失败返回 -1.0f
 *
 * 注意：内部有 20us 忙等，禁止在 20kHz 中断里调用。
 */
float read_vbus(void);

/** 单次读取可能因首帧未落定而失败，最多重试这么多次 */
#define VBUS_READ_RETRY (5U)

#endif /* SENS_ANALOG_H */
