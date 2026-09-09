/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <stdio.h>
#include "app_cfg.h"
#include "app_monitor.h"
#include "bsp.h"
#include "drv_spi.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "drv_timer.h"
#include "sens_encoder.h"
#include "sens_analog.h"
#include "motor.h"
#include "ctrl_foc.h"
#include "ctrl_align.h"

/**
 * @brief MCL 中间件所需的微秒延时（由应用层提供实现）
 */
void mcl_user_delay_us(uint64_t tick)
{
    bsp_delay_us((uint32_t)tick);
}

/**
 * @brief 应用层 —— 启动编排
 *
 * 顺序说明（改动前请先看注释，很多步骤的先后是有依赖的）：
 *
 *  1. drv_adc_init() 必须在 sens_current_calibrate() 之前 —— 零漂校准要读 ADC
 *  2. drv_pwm_clock_init() 必须在 motor_init() 之前 —— MCL 需要 PWM 重载值
 *  3. motor_init() 必须在 motor_disable_output() 之前 —— 先装配再封输出
 *  4. sens_encoder_init() 必须在电机出力之前 —— 通信自检要求轴静止
 *  5. motor_enable_loop() 必须在 ctrl_align_run() 之前 —— 环没使能时对齐无电流输出，
 *     转子不吸合，theta_initial 会取到随机位置
 */
int main(void)
{
    /* ---------- 板级与外设 ---------- */
    bsp_init();
    bsp_encoder_interface_init();     /* SPI1 引脚 + 时钟 */
    drv_spi_init();                   /* SPI 外设参数 */
    bsp_adc_pins_init();
    bsp_pwm_pins_init();

    drv_adc_init();                   /* ADC：相电流 + 母线电压 */
    drv_pwm_clock_init();             /* 获取 PWM 时钟频率，计算重载值 */

    /* ---------- 电机对象装配 ---------- */
    motor_init();
    motor_disable_output();

    sens_current_calibrate();         /* 电流采样零漂校准（要求 PWM 未输出） */
    ctrl_foc_init();                  /* 使能 ADC 采样完成中断（20kHz 心跳） */

    drv_timer_register_tick_callback(motor_detect_loop);
    drv_timer_init();                 /* 1ms 故障检测滴答 */
    bsp_acmp_pins_init();

    drv_pwm_init();                   /* PWM：三相六路 + 死区 + DMA */
    motor_enable_output();

    /* ---------- 编码器（轴静止时自检） ---------- */
    sens_encoder_init();

    /* ---------- 使能控制环 ---------- */
    motor_enable_loop();

    /* ---------- 零点偏移：跳过对齐 or 上电对齐 ---------- */
    if (ENC_SKIP_ALIGN && (ENC_THETA_INITIAL > 0.0f)) {
        /* 跳过对齐：直接用已标定的零点偏移，转子不再被强行吸到 d 轴 */
        motor_set_initial_theta(ENC_THETA_INITIAL);
        g_theta_initial = ENC_THETA_INITIAL;
        ctrl_foc_encoder_update_enable(true);
    } else {
        /* 对齐期间必须冻结电角度为 0（等价于 force_theta(0)）。
           若此时中断仍在刷新真实角度，转子会被恒力矩拖走，
           theta_initial 记到随机位置 —— 所以先关中断更新。 */
        ctrl_foc_encoder_update_enable(false);
        ctrl_align_run();
        ctrl_foc_encoder_update_enable(true);
        g_theta_initial = motor_get_initial_theta();
    }

    /* ---------- 电流给定 ---------- */
    motor_set_current_q(APP_CURRENT_Q_REF);
    motor_set_current_d(APP_CURRENT_D_REF);

    /* ---------- 主循环：观测与后台任务 ---------- */
    while (1) {
        app_monitor_update();
        bsp_delay_ms(APP_MAIN_LOOP_DELAY_MS);
    }

    return 0;
}
