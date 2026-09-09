/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <math.h>
#include "ctrl_svpwm.h"
#include "motor.h"
#include "drv_pwm.h"
#include "hpm_mcl_common.h"

static mcl_control_svpwm_duty_t s_svpwm_duty;
static volatile float s_theta = 0.0f;
static float s_current_freq = 0.0f;
static float s_duty[3] = {0.5f, 0.5f, 0.5f};

void ctrl_svpwm_calc(float u_alpha, float u_beta, float vdc,
                     float *duty_a, float *duty_b, float *duty_c)
{
    float Ts = 1.0f / (float)DRV_PWM_FREQUENCY_HZ;
    float sqrt3 = 1.7320508f;

    float Vref = sqrtf(u_alpha * u_alpha + u_beta * u_beta);
    float Vmax = vdc / sqrt3;   /* SVPWM 最大线性调制电压 */

    if (Vref < 1e-6f || Vref > Vmax) {
        *duty_a = *duty_b = *duty_c = 0.5f;
        return;
    }

    float theta = atan2f(u_beta, u_alpha);
    if (theta < 0) {
        theta += 2.0f * (float)MCL_PI;
    }
    int sector = (int)(theta / ((float)MCL_PI / 3.0f)) + 1;
    if (sector > 6) {
        sector = 1;
    }

    float T1 = 0.0f, T2 = 0.0f, T0;
    float Vdc_inv = 1.0f / vdc;
    float Ta, Tb, Tc;

    switch (sector) {
    case 1:
        T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
        T2 = Ts * (2.0f * u_beta / sqrt3) * Vdc_inv;
        break;
    case 2:
        T1 = Ts * (-u_alpha + u_beta / sqrt3) * Vdc_inv;
        T2 = Ts * (u_alpha + u_beta / sqrt3) * Vdc_inv;
        break;
    case 3:
        T1 = Ts * (-u_alpha - u_beta / sqrt3) * Vdc_inv;
        T2 = Ts * (u_beta / sqrt3) * Vdc_inv * 2.0f;
        break;
    case 4:
        T1 = Ts * (u_beta / sqrt3 - u_alpha) * Vdc_inv;
        T2 = Ts * (-u_alpha - u_beta / sqrt3) * Vdc_inv;
        break;
    case 5:
        T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
        T2 = Ts * (-2.0f * u_beta / sqrt3) * Vdc_inv;
        break;
    case 6:
        T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
        T2 = Ts * (u_alpha + u_beta / sqrt3) * Vdc_inv;
        break;
    default:
        T1 = T2 = 0;
        break;
    }

    /* 过调制限制（T1+T2 <= Ts） */
    if (T1 + T2 > Ts) {
        float k = Ts / (T1 + T2);
        T1 *= k;
        T2 *= k;
    }
    T0 = Ts - T1 - T2;

    /* 七段式中心对齐 */
    switch (sector) {
    case 1:
        Ta = (T1 + T2 + T0 / 2) / Ts;
        Tb = (T2 + T0 / 2) / Ts;
        Tc = (T0 / 2) / Ts;
        break;
    case 2:
        Ta = (T1 + T0 / 2) / Ts;
        Tb = (T1 + T2 + T0 / 2) / Ts;
        Tc = (T0 / 2) / Ts;
        break;
    case 3:
        Ta = (T0 / 2) / Ts;
        Tb = (T1 + T2 + T0 / 2) / Ts;
        Tc = (T2 + T0 / 2) / Ts;
        break;
    case 4:
        Ta = (T0 / 2) / Ts;
        Tb = (T1 + T0 / 2) / Ts;
        Tc = (T1 + T2 + T0 / 2) / Ts;
        break;
    case 5:
        Ta = (T2 + T0 / 2) / Ts;
        Tb = (T0 / 2) / Ts;
        Tc = (T1 + T2 + T0 / 2) / Ts;
        break;
    case 6:
        Ta = (T1 + T2 + T0 / 2) / Ts;
        Tb = (T0 / 2) / Ts;
        Tc = (T1 + T0 / 2) / Ts;
        break;
    default:
        Ta = Tb = Tc = 0.5f;
        break;
    }

    *duty_a = (Ta < 0.0f) ? 0.0f : ((Ta > 1.0f) ? 1.0f : Ta);
    *duty_b = (Tb < 0.0f) ? 0.0f : ((Tb > 1.0f) ? 1.0f : Tb);
    *duty_c = (Tc < 0.0f) ? 0.0f : ((Tc > 1.0f) ? 1.0f : Tc);
}

void ctrl_svpwm_step(void)
{
    /* 软启动：电频率逐步爬升到目标值 */
    if (s_current_freq < CTRL_SVPWM_TARGET_FREQ) {
        s_current_freq += CTRL_SVPWM_RAMP_STEP;
        if (s_current_freq > CTRL_SVPWM_TARGET_FREQ) {
            s_current_freq = CTRL_SVPWM_TARGET_FREQ;
        }
    }

    s_theta += 2.0f * (float)MCL_PI * s_current_freq * (1.0f / (float)DRV_PWM_FREQUENCY_HZ);
    if (s_theta > 2.0f * (float)MCL_PI) {
        s_theta -= 2.0f * (float)MCL_PI;
    }

    float u_alpha = CTRL_SVPWM_VREF * cosf(s_theta);
    float u_beta  = CTRL_SVPWM_VREF * sinf(s_theta);

    motor_get_loop()->control->method.svpwm(u_alpha, u_beta, motor_get_vbus(), &s_svpwm_duty);

    motor_set_duty_raw(0, s_svpwm_duty.a);
    motor_set_duty_raw(1, s_svpwm_duty.b);
    motor_set_duty_raw(2, s_svpwm_duty.c);

    s_duty[0] = s_svpwm_duty.a;
    s_duty[1] = s_svpwm_duty.b;
    s_duty[2] = s_svpwm_duty.c;
}

float ctrl_svpwm_get_duty(uint8_t phase)
{
    if (phase > 2) {
        return 0.0f;
    }
    return s_duty[phase];
}
