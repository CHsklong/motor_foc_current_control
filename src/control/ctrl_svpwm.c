/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_pwm.h"
#include "motor.h"
#include "ctrl_svpwm.h"
#include "sens_analog.h"

float Vdc = 0.0f;               /* 母线电压 V */
float Vref = 1.0f;                 /* 给定电压矢量幅值（归一化） */
mcl_control_svpwm_duty_t svpwm_duty;
float svpwma;
float svpwmb;
float svpwmc;

/* 开环拖动状态 */
static float target_freq = 5.0f;    /* 目标电频率 Hz */
static float current_freq = 0.0f;   /* 当前电频率 Hz */
volatile static float theta = 0.0f; /* 开环电角度 */

#define FREQ_RAMP_STEP  0.0005f     /* 每个 PWM 周期的电频率增量 */

/* SVPWM 实现：七段式中心对齐，带过调制与零矢量保护 */
void svpwm(float u_alpha, float u_beta, float Vdc,
                          float *duty_a, float *duty_b, float *duty_c)
{
    float Ts = 1.0f / PWM_FREQUENCY;  /* 50us @20kHz */
    float sqrt3 = 1.7320508f;
    Vdc =read_vbus();  

    /* 电压归一化（调制比范围 0~1） */
    float Vref = sqrtf(u_alpha*u_alpha + u_beta*u_beta);
    float Vmax = Vdc / sqrt3;  /* SVPWM 最大线性调制电压 */

    if (Vref < 1e-6f || Vref > Vmax) {
        /* 零矢量或过调制保护 */
        *duty_a = *duty_b = *duty_c = 0.5f;
        return;
    }

    /* 计算扇区（使用角度） */
    float theta = atan2f(u_beta, u_alpha);
    if (theta < 0) theta += 2.0f * MCL_PI;
    int sector = (int)(theta / (MCL_PI / 3.0f)) + 1;
    if (sector > 6) sector = 1;

    /* 计算 X, Y, Z（标准 SVPWM 时间变量） */
    float T1, T2, T0;
    float Vdc_inv = 1.0f / Vdc;
    float Ta, Tb, Tc;

    switch(sector) {
        case 1:  /* 0°~60° */
            T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (2.0f * u_beta / sqrt3) * Vdc_inv;
            break;
        case 2:  /* 60°~120° */
            T1 = Ts * (-u_alpha + u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (u_alpha + u_beta / sqrt3) * Vdc_inv;
            break;
        case 3:  /* 120°~180° */
            T1 = Ts * (-u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (u_beta / sqrt3) * Vdc_inv * 2.0f;
            break;
        case 4:  /* 180°~240° */
            T1 = Ts * (u_beta / sqrt3 - u_alpha) * Vdc_inv;
            T2 = Ts * (-u_alpha - u_beta / sqrt3) * Vdc_inv;
            break;
        case 5:  /* 240°~300° */
            T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (-2.0f * u_beta / sqrt3) * Vdc_inv;
            break;
        case 6:  /* 300°~360° */
            T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (u_alpha + u_beta / sqrt3) * Vdc_inv;
            break;
        default:
            T1 = T2 = 0;
    }

    /* 过调制限制（T1+T2 <= Ts） */
    if (T1 + T2 > Ts) {
        float k = Ts / (T1 + T2);
        T1 *= k;
        T2 *= k;
    }
    T0 = Ts - T1 - T2;

    /* 计算三相占空比（七段式中心对齐） */
    switch(sector) {
        case 1:
            Ta = (T1 + T2 + T0/2) / Ts;
            Tb = (T2 + T0/2) / Ts;
            Tc = (T0/2) / Ts;
            break;
        case 2:
            Ta = (T1 + T0/2) / Ts;
            Tb = (T1 + T2 + T0/2) / Ts;
            Tc = (T0/2) / Ts;
            break;
        case 3:
            Ta = (T0/2) / Ts;
            Tb = (T1 + T2 + T0/2) / Ts;
            Tc = (T2 + T0/2) / Ts;
            break;
        case 4:
            Ta = (T0/2) / Ts;
            Tb = (T1 + T0/2) / Ts;
            Tc = (T1 + T2 + T0/2) / Ts;
            break;
        case 5:
            Ta = (T2 + T0/2) / Ts;
            Tb = (T0/2) / Ts;
            Tc = (T1 + T2 + T0/2) / Ts;
            break;
        case 6:
            Ta = (T1 + T2 + T0/2) / Ts;
            Tb = (T0/2) / Ts;
            Tc = (T1 + T0/2) / Ts;
            break;
        default:
            Ta = Tb = Tc = 0.5f;
    }

    /* 限幅保护（0~1） */
    *duty_a = (Ta < 0.0f) ? 0.0f : ((Ta > 1.0f) ? 1.0f : Ta);
    *duty_b = (Tb < 0.0f) ? 0.0f : ((Tb > 1.0f) ? 1.0f : Tb);
    *duty_c = (Tc < 0.0f) ? 0.0f : ((Tc > 1.0f) ? 1.0f : Tc);
}

void svpwm_openloop_step(void)
{
    if (current_freq < target_freq) {          /* 软启动 */
        current_freq += FREQ_RAMP_STEP;
        if (current_freq > target_freq) current_freq = target_freq;
    }
    theta += 2.0f * MCL_PI * current_freq * (1.0f / PWM_FREQUENCY);
    if (theta > 2.0f * MCL_PI) {
        theta -= 2.0f * MCL_PI;
    }
    float u_alpha = Vref * cosf(theta);
    float u_beta  = Vref * sinf(theta);

    motor0.loop.control->method.svpwm(u_alpha, u_beta, motor0.cfg.mcl.physical.motor.vbus, &svpwm_duty);
    pwm_duty_set(mcl_drivers_chn_a, svpwm_duty.a);
    pwm_duty_set(mcl_drivers_chn_b, svpwm_duty.b);
    pwm_duty_set(mcl_drivers_chn_c, svpwm_duty.c);
    svpwma = svpwm_duty.a;
    svpwmb = svpwm_duty.b;
    svpwmc = svpwm_duty.c;
}
