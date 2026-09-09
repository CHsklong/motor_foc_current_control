/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "sens_encoder.h"
#include "mt6835.h"
#include "drv_pwm.h"

volatile uint32_t g_angle_raw = 0;
volatile float    theta_r = 0.0f;

void sens_encoder_init(void)
{
    mt6835_init();
}

bool sens_encoder_get_theta(float *theta)
{
#if SENSOR_USE_VIRTUAL_ANGLE
    /* 虚拟角度：每次调用增加固定步长，用于隔离"角度源"是否正确。
       步长 = 2π × 频率 / 电流环频率 */
    static float s_virtual_theta = 0.0f;
    static float s_step = 2.0f * SENS_ENCODER_PI * SENSOR_VIRTUAL_FREQ_HZ / (float)DRV_PWM_FREQUENCY_HZ;

    s_virtual_theta += s_step;
    if (s_virtual_theta >= 2.0f * SENS_ENCODER_PI) {
        s_virtual_theta -= 2.0f * SENS_ENCODER_PI;
    }
    *theta = -s_virtual_theta;
    if (*theta < 0) {
        *theta += 2.0f * SENS_ENCODER_PI;
    }
    theta_r = *theta;
    return true;
#else
    uint32_t angle_raw;

    if (!mt6835_read_angle_raw(&angle_raw)) {
        return false;
    }
    g_angle_raw = angle_raw;

    /* MT6835 为 21 位分辨率：ANGLE[20:0]，满量程 = 2^21 = 2097152（见数据手册 7.6.8）
     * 注意：angle_raw 已右移 3 位得到 21bit，这里必须除 2^21，
     * 除 2^23 会让机械角只有真实值的 1/4。 */
    float theta_raw = (float)angle_raw * 2.0f * SENS_ENCODER_PI / (float)MT6835_ANGLE_RESOLUTION;

    /* 极性反转：顺时针旋转 → 角度递增 */
    *theta = -theta_raw;
    if (*theta < 0) {
        *theta += 2.0f * SENS_ENCODER_PI;
    }
    theta_r = *theta;
    return true;
#endif
}

bool sens_encoder_get_abs_theta(float *theta)
{
    return sens_encoder_get_theta(theta);
}
