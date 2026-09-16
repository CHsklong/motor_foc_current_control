/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_adc.h"
#include "drv_pwm.h"
#include "motor.h"
#include "sens_analog.h"
#include "dbg_probe.h"
#include "app_monitor.h"

void app_monitor_update(void)
{
    /* ADC 原始码值 */
    g_raw_u = (float)adc_buff[0][0];
    g_raw_v = (float)adc_buff[1][0];

    /* 位置环观测：给定、反馈、误差、位置环输出的速度给定 */
    g_pos_err   = g_pos_ref - g_pos_abs;
    g_ref_speed = motor0.loop.exec_ref.speed;

    /* 位置环积分项,单位是 "rad·拍" ,判断是否抗饱和。 */
    g_pos_integral = motor0.cfg.control.position_pid_cfg.integral;

    /* 故障状态：电机"突然不转"时先看这三个 */
    g_loop_status = (uint32_t)motor0.loop.status;      /* 2=run，3=fail */
    g_enc_status  = (uint32_t)motor0.encoder.status;   /* 非 0 即编码器异常 */
    g_ana_status  = (uint32_t)motor0.analog.status;    /* 非 0 即采样异常（含过流） */
    g_pwm_enabled = pwm_output_is_enabled() ? 1U : 0U; /* 0 = 已被保护关断输出 */

    /* PWM 硬件过流锁存（只读，不清）。注意它和 g_pwm_enabled 是两回事：
       g_pwm_enabled 只是我们自己的软件标志，硬件故障锁存会绕过它，
       直接把六路输出钉死为 0 —— "一切正常但电机不出力"时先看这个。 */
    g_pwm_fault = pwm_fault_is_latched() ? 1U : 0U;

    /* 母线电压（const_vbus 指向 mcl_cfg 里的那个值，上电读一次后不再刷新） */
    g_vbus = motor0.cfg.mcl.physical.motor.vbus;

    /* 20kHz 节拍耗时的周期数→us 换算已移到 isr_timed_end() 里做：
       main 被饿死/卡死时 J-Scope 也能直接看到 g_isr_us_max / g_enc_us_max / g_loop_us_max */
}
