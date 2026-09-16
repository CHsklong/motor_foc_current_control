/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file main.c
 * @brief 应用入口：初始化各层、选择运行模式、主循环监控
 */
#include "hw_map.h"

#include "hal_bsp.h"
#include "drv_spi.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "drv_timer.h"
#include "sens_encoder.h"
#include "sens_analog.h"
#include "mt6835.h"
#include "motor.h"
#include "motor_hw_foc.h"
#include "ctrl_foc.h"
#include "ctrl_svpwm.h"
#include "dbg_probe.h"
#include "drv_flash.h"
#include "app_cfg.h"
#include "app_monitor.h"
#include "app_keeper.h"

/**
 * @brief 启动流程
 *
 * 顺序不能随意调整：
 *   1) 引脚/时钟 → SPI → ADC，之后才能读编码器与相电流；
 *   2) motor_clock_hz 必须在 motor_init() 之前填好，PWM_RELOAD 依赖它；
 *   3) 静态零点校准必须在 PWM 无输出、电机静止时做；
 *   4) 控制环必须先使能再对齐，否则 hpm_mcl_loop() 内部所有计算与 PWM 输出
 *      都在 if(loop->enable) 里，对齐不会输出电流，转子不会吸合；
 *   5) 对齐完成后要等 20ms 让测速滤波器的假速度尖峰耗散，再正式出力。
 *   6) 复位后必须先等电源/模拟前端/编码器稳定，再做零点校准、再开 PWM 输出。
 *      调试器下载时板子已经上电很久，这段爬升期天然被跳过；冷上电则恰恰相反 ——
 *      "烧录后必转、重新上电时好时坏"的绝大部分原因就在这一步。
 *   7) 【上电快慢】pwm_init() 必须在零点校准之前：相电流 ADC 由 PWM 触发，
 *      PWM 不跑就没有触发，校准里"等首帧"会耗满上限白等。详见下面 pwm_init 处注释。
 */

int main(void)
{
    board_init();                     /* 开发板基础初始化 */

    /* 【冷上电稳定期】等母线电容充电、电流采样运放/ADC 基准建立、
       MT6835 完成 POR。之后才允许取零点、读母线、开输出。 */
    board_delay_ms(POWER_UP_SETTLE_MS);

    bsp_init();                       /* 引脚与时钟：SPI1/ADC/PWM */
    spi1_config();                    /* SPI 外设参数（MT6835 模式 3、8MHz） */
    adc_init();                       /* ADC 初始化 */
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_MOTOR_CLOCK_SOURCE);  /* PWM 时钟频率，PWM_RELOAD 依赖它 */

    /* 零点偏移从 FLASH 取：走 XIP 指针直读，不调任何 ROM API（不会动 XPI 配置）。
       FLASH_PARAM_CALIB_ONCE=1 时才会探测几何并擦写，用于专门烧一次标定值。 */
#if FLASH_PARAM_ENABLE
    float theta_init = drv_flash_load_theta(ENC_THETA_INITIAL, FLASH_PARAM_CALIB_ONCE);
#else
    float theta_init = ENC_THETA_INITIAL;   /* 排查用：完全不碰 FLASH */
#endif

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    clc_init();
#endif

    g_boot_step = 1U;                 /*故障排除序号*/
    motor_init();                     /* 电机与 MCL 参数初始化 */
    g_boot_step = 2U;
    motor_set_loop_mode(MOTOR_RUN_LOOP_MODE);  /* 按 app_cfg 选择闭环结构（必须在 enable 之前） */

    g_boot_step = 3U;
    (void)0;                          /* 位置环无初始化：限幅在 20kHz 中断里每拍算 */

    g_boot_step = 4U;
    pwm_init();                       /* PWM 初始化：计数器与 ADC 触发链从这里开始跑 */
    g_boot_step = 5U;
    disable_all_pwm_output();         /* 零点校准期间必须无输出（pwm_init 的 deinit 会清掉之前的 force 设置） */


    g_boot_step = 6U;
    board_delay_ms(PWM_FAULT_SETTLE_MS);   /* 先等比较器输出稳定，再判断 */
    if (pwm_fault_is_latched())
    {
        g_pwm_fault = 1U;
        pwm_fault_clear();
        g_pwm_fault_clr++;
    }

    g_boot_step = 7U;
    motor_adc_midpoint();             /* 静态零点校准（此时 PWM 触发链已跑起来，DMA 每 50us 刷新一帧） */
    g_boot_step = 8U;
    adc_isr_enable();                 /* 使能 ADC 中断，即 20kHz 控制节拍 */
    g_boot_step = 9U;
    timer_init();                     /* 1ms 故障检测定时器 */

    g_boot_step = 10U;
    enable_all_pwm_output();          /* 使能 PWM 输出 */

    g_boot_step = 11U;
    mt6835_read_byte(MT6835_REG_USER_ID, &g_user_id);   /* 读 USER_ID，结果存入 g_user_id */

    g_boot_step = 12U;
    mt6835_seq3_init();                 /*角度读取*/

    g_boot_step = 13U;
    hpm_mcl_loop_enable(&motor0.loop);  /* 使能控制环（对齐需要出力） */

/* ---- 角度对齐 ---- */
    int theta_need_save = 0;            /* 1=本次跑了对齐，需要把结果存进 FLASH */

    g_boot_step = 14U;
    if (ENC_SKIP_ALIGN && (theta_init > 0.0f))
    {
        /* 跳过对齐：直接用 FLASH 里已标定的零点偏移（首次上电则是 ENC_THETA_INITIAL），
           转子不再被强行吸到 d 轴 */
        hpm_mcl_encoder_set_initial_theta(&motor0.encoder, theta_init);
        g_theta_initial = theta_init;
        g_encoder_isr_enable = 1;
    }
    else
    {
        g_encoder_isr_enable = 0;
        motor_angle_align();          /* 三段式对齐 */
        g_encoder_isr_enable = 1;     /* 之后由 ADC 中断按 20kHz 刷新角度 */
        g_theta_initial = motor0.encoder.theta_initial;   /* 供 J-Scope 观察零点偏移 */
        theta_need_save = 1;          /* 对齐结果存 FLASH，下次上电可直接跳过对齐 */
    }

    /* ===== 上电第一拍假速度尖峰：先冲洗，再出力 */
    g_boot_step = 15U;
    hpm_mcl_loop_disable(&motor0.loop);
    if (theta_need_save && FLASH_PARAM_CALIB_ONCE)
    {
        /* 借这 20ms 冲洗窗口写 FLASH：输出已断、控制环已停，擦写关中断不会失控。
           与已存值相差不到 0.001rad 时会自动跳过，不会每次上电都擦一遍。
           注意：擦写要调 ROM 的 XPI 探测 API，会把 XPI 重配、拖慢取指，
           所以只在 FLASH_PARAM_CALIB_ONCE=1（标定固件）时才做。 */
        drv_flash_save_theta(g_theta_initial, false);
    }
    g_boot_step = 16U;
    board_delay_ms(20);
    hpm_mcl_loop_enable(&motor0.loop);  /* 正式使能，开始出力 */

/* ---- 运行模式给定 ---- */
    mcl_user_value_t user_speed;      /* 速度环速度给定 */
    mcl_user_value_t position;        /* 位置环位置给定 */
    mcl_user_value_t id, iq;          /* 电流环电流给定 */
    if (FOC_CURRENT_MODE)
    {

        /* Q 轴电流 = 转矩电流；D 轴 = 励磁（表贴式 Ld≈Lq，id 不产生转矩，
           只用来抬高相电流幅值以提高 ADC 信噪比） */
        iq.enable = true;
        iq.value = 0.1f;
        hpm_mcl_loop_set_current_q(&motor0.loop, iq);
        id.enable = true;
        id.value = 0.0f;
        hpm_mcl_loop_set_current_d(&motor0.loop, id); 


    }

    /* ===== 位置环 / 速度环必须互斥 */
    g_boot_step = 17U;
    if (FOC_POSITION_MODE)
    {
        user_speed.enable = false;      /* 防止速度给定盖掉位置环输出 */
        user_speed.value  = 0.0f;
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
        motor0.cfg.control.position_pid_cfg.integral = 0.0f;   /* 位置环积分清零 */

        encoder_abs_rebase();           /* 以当前机械位置为 0 基准 */

        /* 目标取"相对基准的增量"：方向唯一确定，不会因多圈角回绕而反向 */
        position.enable = true;
        position.value  = POS_TARGET_DELTA;   /* 相对基准的位置增量（机械角 rad） */
        hpm_mcl_loop_set_position(&motor0.loop, position);
        g_pos_ref = position.value;
    }
    else if (FOC_SPEED_MODE)
    {
        user_speed.enable = true;
        user_speed.value = 50;    /* 机械角速度 rad/s */
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
    }


/* ---- 主循环（约 1kHz）---- */
    while (1)
    {
        g_main_step = 1U;        /* 1=观测/探针段 */
        g_heart_main++;
        dbg_probe_keep();        /* 兜住 J-Scope 探针符号，防止 --gc-sections 删除 */

        app_monitor_update();    /* 观测量 */

        g_main_step = 2U;        /* 2=运行监护段 */
        app_keeper_update();     /* 运行监护：把上电期的锁死类故障自动救回来 */

        g_main_step = 3U;        /* 3=motor_ban 分支 */
        if (motor_ban)
        {
            /* 保持电机静止：输出 50% 占空比 */
            pwm_duty_set(mcl_drivers_chn_a, 0.5);
            pwm_duty_set(mcl_drivers_chn_b, 0.5);
            pwm_duty_set(mcl_drivers_chn_c, 0.5);
        }

        g_main_step = 4U;        /* 4=电流环阶跃测试块 */
        if(1)                   //电流环阶跃响应测试
        {

          id.value = 0.5f;          // 阶跃到 0.5A
          hpm_mcl_loop_set_current_d(&motor0.loop, id);
          board_delay_ms(1);
          id.value = 0.0f;          // 阶跃到 0.0A
          hpm_mcl_loop_set_current_d(&motor0.loop, id);
        }
        g_main_step = 5U;        /* 5=1ms 延时段 */
        board_delay_ms(1);
    }

    return 0;
}
