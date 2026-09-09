/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "sens_analog.h"
#include "drv_adc.h"
#include "bsp.h"
#include "board.h"

ATTR_PLACE_AT_FAST_RAM_INIT volatile uint32_t adc_u_midpoint;
ATTR_PLACE_AT_FAST_RAM_INIT volatile uint32_t adc_v_midpoint;

/* 缓存 A 相电流码值，供 B 相重构使用 */
static int32_t s_current_a_code = 0;

void sens_current_calibrate(void)
{
    uint32_t adc_u_sum = 0;
    uint32_t adc_v_sum = 0;
    uint32_t times = 0;

    /* 确保电机静止、PWM 未输出时电流为零 */
    do {
        adc_u_sum += drv_adc_read_phase_raw(drv_adc_chn_u);
        adc_v_sum += drv_adc_read_phase_raw(drv_adc_chn_v);
        times++;
        bsp_delay_ms(1);
        if (times >= SENS_CURRENT_CAL_MS) {
            break;
        }
    } while (1);

    adc_u_midpoint = adc_u_sum / SENS_CURRENT_CAL_MS;
    adc_v_midpoint = adc_v_sum / SENS_CURRENT_CAL_MS;
}

bool sens_current_read_phase_a(int32_t *value)
{
    int32_t sens_value = (int32_t)drv_adc_read_phase_raw(drv_adc_chn_u);

    s_current_a_code = (int32_t)adc_u_midpoint - sens_value;
    *value = s_current_a_code;
    return true;
}

bool sens_current_read_phase_b(int32_t *value)
{
    int32_t sens_value = (int32_t)drv_adc_read_phase_raw(drv_adc_chn_v);
    int32_t current_c;

    /* 硬件采样的是 C 相电流（ADCV_INDEX 对应 C 相） */
    current_c = (int32_t)adc_v_midpoint - sens_value;
    *value = -(s_current_a_code + current_c);   /* B 相 = -(A + C) */
    return true;
}

float sens_vbus_read(void)
{
    uint16_t raw;
    float pin_voltage;

    if (!drv_adc_read_vbus_raw(&raw)) {
        return -1.0f;   /* 布局不符，返回负数表示读取失败 */
    }

    /* ADC 读数 → ADC 引脚电压 → 实际母线电压 */
    pin_voltage = (float)raw * 3.3f / 4095.0f;
    return pin_voltage * MOTOR0_VBUS_DIVIDER;
}
