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
#include "traj_s_curve.h"
#include "dbg_probe.h"
#include "drv_flash.h"
#include "app_cfg.h"
#include "app_monitor.h"
#include "app_keeper.h"
#include "can_app.h"
#include "motor_scope.h"


int main(void)
{
    board_init();                                   /* 开发板初始化 */
    board_delay_ms(POWER_UP_SETTLE_MS);             /* 等母线电容充电、电流采样运放/ADC 基准建立、MT6835 完成 POR。之后才允许取零点、读母线、开输出。 */
    bsp_init();                                     /* 引脚配置 */
    spi1_config();                                  /* 编码器SPI 参数设置 */
    adc_init();                                     /* ADC 初始化 */
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_MOTOR_CLOCK_SOURCE);  /* PWM 时钟频率 */

#if FLASH_PARAM_ENABLE                              /* 编码器零点偏移从 FLASH 取 */
    float theta_init = drv_flash_load_theta(ENC_THETA_INITIAL, FLASH_PARAM_CALIB_ONCE);
#else
    float theta_init = ENC_THETA_INITIAL;   /* 排查FLASH */
#endif

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    clc_init();
#endif

    g_boot_step = 1U;                               /* 故障排除序号 */
    motor_init();                                   /* 电机参数初始化 */
    g_boot_step = 2U;
    if(FOC_MODE)
    {
      motor_set_loop_mode(MOTOR_RUN_LOOP_MODE);       /* 闭环下切换三环模式 */
    }
    g_boot_step = 3U;
    pwm_init();                                     /* PWM 初始化 */
    g_boot_step = 4U;
    disable_all_pwm_output();                       /* 零点校准期间关闭输出 */
    g_boot_step = 5U;
    ctrl_set_keeper_hook(app_keeper_update);        /* 主循环停滞时由中断代跑 */
    board_delay_ms(PWM_FAULT_SETTLE_MS);            /* 先等比较器输出稳定，再判断 */
    if (pwm_fault_is_latched())                     /* 如果pwm锁存，清除故障标志*/
    {
        g_pwm_fault = 1U;
        pwm_fault_clear();
        g_pwm_fault_clr++;
    }
    g_boot_step = 6U;
    motor_adc_midpoint();                                /* 采样电流零点校准 */
    g_boot_step = 7U; 
    adc_isr_enable();                                    /* 使能 ADC 中断 */
    g_boot_step = 8U;
    timer_init();                                        /* 1ms 定时器，触发中断里的故障检测 */
    g_boot_step = 9U;
    enable_all_pwm_output();                             /* 使能 PWM 输出 */
    g_boot_step = 10U;
    mt6835_read_byte(MT6835_REG_USER_ID, &g_user_id);    /* 从指定ID寄存器读取数据 */
    g_boot_step = 11U;
    mt6835_seq3_init();                                  /* 上电自检 */
    can_app_init();                                      /* CAN初始化 */
    motor_scope_init();                                  /* ELF 变量录波：Build ID + 默认六路通道 */
    g_boot_step = 12U;
    if (!(ENC_SKIP_ALIGN && (theta_init > 0.0f)))
    {
#if !ENCODER_ONLY_DEMO
        hpm_mcl_loop_enable(&motor0.loop);               /* 对齐路径提前使能 */
#endif
    }
/* ---- 角度对齐 ---- */
    int theta_need_save = 0;                            /* 1=本次跑了对齐，需要把结果存进 FLASH */
    g_boot_step = 13U;
    if (ENC_SKIP_ALIGN && (theta_init > 0.0f))
    {
        hpm_mcl_encoder_set_initial_theta(&motor0.encoder, theta_init);      /* 跳过对齐：直接用 FLASH 里已标定的零点偏移 */
        g_theta_initial = theta_init; 
        g_encoder_isr_enable = 1;
    }
    else
    {
        g_encoder_isr_enable = 0;
        motor_angle_align();                              /* 三段式对齐 */
        g_encoder_isr_enable = 1;                         /* 之后由 ADC 中断按 20kHz 刷新角度 */
        g_theta_initial = motor0.encoder.theta_initial;   /* 供 J-Scope 观察零点偏移 */
        theta_need_save = 1;                              /* 对齐结果存 FLASH，下次上电可直接跳过对齐 */
    }

    /* 上电第一拍假速度尖峰：先冲洗，再出力 */
    g_boot_step = 14U;
    hpm_mcl_loop_disable(&motor0.loop);
    if (theta_need_save && FLASH_PARAM_CALIB_ONCE)
    {
        drv_flash_save_theta(g_theta_initial, false);
    }
    g_boot_step = 15U;
    board_delay_ms(20);

/* ---- 运行模式给定 ---- */
    mcl_user_value_t user_speed;      /* 速度环速度给定 */
    mcl_user_value_t id, iq;          /* 电流环电流给定 */
#if !SCURVE_ENABLE
    mcl_user_value_t position;        /* 位置环位置给定 */
#endif
    if (FOC_CURRENT_MODE)             /* 电流环 */
    {
        iq.enable = true;
        iq.value = 0.0f;
        hpm_mcl_loop_set_current_q(&motor0.loop, iq);
        id.enable = true;
        id.value = 0.0f;
        hpm_mcl_loop_set_current_d(&motor0.loop, id); 
    }

#if !SCURVE_ENABLE && !CAN_CTRL_ENABLE
    float pos_debug = 0.0f;
#endif
#if SCURVE_ENABLE && !CAN_CTRL_ENABLE       /*无can通信时的位置曲线规划配置*/
    uint32_t sc_dwell = SCURVE_DWELL_MS;  /* 停留计数（主循环约 1ms 一次）；初值=起始等待窗口 */
    float    sc_dir   = 1.0f;   /* 运动方向：+1 正向，-1 反向（仅往复模式使用） */
    uint32_t sc_armed = 1U;     /* 1=还允许下发运动；单向单次模式下走完一次即置 0 */
#endif
    g_boot_step = 16U;
    if (FOC_POSITION_MODE)
    {
        motor0.cfg.control.position_pid_cfg.integral = 0.0f;   /* 位置环积分清零 */
        encoder_abs_rebase();           /* 以当前机械位置为 0 基准 */
        g_boot_step = 18U;

#if SCURVE_ENABLE                       /* 位置S型曲线规划 */
        motor0.cfg.loop.enable_position_loop = false;         /* 关掉 position_loop，S 曲线模式下位置外环由规划决定（速度前馈 + P/PI 修正） */
        scurve_init(1.0f / (float)PWM_FREQUENCY, SCURVE_VMAX, SCURVE_AMAX, SCURVE_JMAX);  /* 约束值设置 */
        scurve_reset(0.0f);                                   /* 规划器输出对齐到刚 rebase 的 0 基准 */
        g_boot_step = 17U;
        motor0.cfg.control.position_pid_cfg.cfg.kp = SCURVE_POS_KP;        /* 位置S型曲线前馈结构 P 增益单独整定 */
        user_speed.enable = true;
        user_speed.value  = 0.0f;       /* 配置速度为0，防止速度给定盖掉位置环输出 */                       
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);   
        g_pos_ref = 0.0f;
        g_boot_step = 18U;
#else
        user_speed.enable = false;      /* 防止速度给定盖掉位置环输出 */
        user_speed.value  = 0.0f;
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
        g_boot_step = 19U;
        /* 目标取"相对基准的增量"：方向唯一确定，不会因多圈角回绕而反向 */
        position.enable = true;
        position.value  = POS_TARGET_DELTA;   /* 相对基准的位置增量POS_TARGET_DELTA（机械角 rad） */
        hpm_mcl_loop_set_position(&motor0.loop, position);
        g_boot_step = 20U;
        g_pos_ref = position.value;
#endif
    }
    else if (FOC_SPEED_MODE)
    {
#if SCURVE_SPD_ENABLE
        /* 速度 S 曲线：只设定"目标速度"，由规划器按 jerk 受限斜坡平滑过渡过去。
           斜坡推进在 20kHz 中断（scurve_spd_isr_tick），下发在 speed_ramp_pre()（4kHz）。 */
        scurve_spd_init(1.0f / (float)PWM_FREQUENCY,
                        SCURVE_SPD_AMAX_ACC, SCURVE_SPD_AMAX_DEC,
                        SCURVE_SPD_JMAX, SCURVE_SPD_VMAX);
        scurve_spd_reset(0.0f);
        scurve_spd_move(SCURVE_SPD_TARGET);
#else
        user_speed.enable = true;
        user_speed.value = 50;                /* 机械角速度 rad/s */
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
#endif
    }
#if ENCODER_ONLY_DEMO
    /* 只读编码器演示：不使能闭环、不输出 PWM（编码器在 ADC 中断里照常刷新）。
       注意 app_cfg.h 已把 RUN_KEEPER_ENABLE 关掉，否则 PWM 会被监护反复救回。 */
    disable_all_pwm_output();
#else
    hpm_mcl_loop_enable(&motor0.loop);        /* 命令侧全部就绪后才正式出力 */
#endif
g_boot_step = 19U;

/* ---- 主循环（约 1kHz）---- */
    while (1)
    {
        g_main_step = 1U;        /* 1=观测/探针段 */
        g_heart_main++;
        dbg_probe_keep();        /* 兜住 J-Scope 探针符号，防止 --gc-sections 删除 */
        app_monitor_update();    /* 观测量 */
        g_main_step = 2U;        /* 2=运行监护段 */
        app_keeper_update();     /* 运行监护：把上电期的锁死类故障自动救回来 */
        g_main_step = 22U;       /* 22=CAN 段：取命令 + 100ms 遥测（CAN_CTRL_ENABLE=0 时空实现） */
        can_app_poll();
        g_main_step = 3U;        /* 3=motor_ban 分支 */
        if (motor_ban)           /* 保持电机静止：输出 50% 占空比 */
        {
            disable_all_pwm_output();
        }
        g_main_step = 4U;        /* 4=电流环阶跃测试块 */
        if(step_response_c)      /* 电流环阶跃响应测试 */
        {
          board_delay_ms(2000);
          id.value = 0.5f;          /* 阶跃到 0.5A */
          hpm_mcl_loop_set_current_d(&motor0.loop, id);
          board_delay_ms(2000);
          id.value = 0.0f;          /* 阶跃到 0.0A */
          hpm_mcl_loop_set_current_d(&motor0.loop, id);
        }
#if !CAN_CTRL_ENABLE   
  #if !FOC_POSITION_MODE
        if(step_response_s)
        {
          board_delay_ms(2000);
          if(user_speed.value<50)
          {
            user_speed.value += 10;    /* 机械角速度 rad/s */
          }
          g_ref_speed = user_speed.value;
          hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
        }
  #endif
        if(step_response_p)
        {
#if SCURVE_ENABLE
            if (sc_dwell > 0U) {
                sc_dwell--;
            } else if ((sc_armed != 0U) && scurve_is_idle()) {
                if (scurve_move_delta(sc_dir * POS_TARGET_DELTA) == 0) {
#if SCURVE_REPEAT_ENABLE
                    sc_dir = -sc_dir;       /* 往复：反向，下一段往回走 */
#else
                    sc_armed = 0U;          /* 单向单次：本次行程即全部 */
#endif
                }
                sc_dwell = SCURVE_DWELL_MS;
            }
            g_sc_dwell = sc_dwell;      /* J-Scope：递减到 0 触发一次；往复模式下 0↔800 循环 */
#else
            pos_debug += 5;
            board_delay_ms(2000);
            encoder_abs_rebase();           /* 以当前机械位置为 0 基准 */
            position.value = pos_debug;
            g_pos_ref = position.value;
            hpm_mcl_loop_set_position(&motor0.loop, position);
#endif
        }
#endif /* !CAN_CTRL_ENABLE */
        g_main_step = 5U;        /* 5=节流段：等下一个 1ms 节拍 */
        {
            uint32_t t_ms = g_sys_ms;                           /* 忙等改等系统毫秒节拍，避免中断占满 CPU 时主循环饿死。 */
            uint32_t t_cy = (uint32_t)hpm_csr_get_core_cycle();
            while ((g_sys_ms == t_ms) &&(((uint32_t)hpm_csr_get_core_cycle() - t_cy) < 1000000U))
            {
            }
        }
    }

    return 0;
}
