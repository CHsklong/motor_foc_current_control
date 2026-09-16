/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "motor.h"
#include "motor_params.h"
#include "motor_hw_foc.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "drv_qei.h"
#include "sens_encoder.h"
#include "sens_analog.h"
#include "dbg_probe.h"

ATTR_PLACE_AT_FAST_RAM_INIT motor0_t motor0;
ATTR_PLACE_AT_FAST_RAM_INIT mcl_user_value_t user_set_theta;
float abs_position_theta;

/**
 * @brief MCL 故障检测的用户回调
 *
 * 【必须装】hpm_mcl_detect_loop() 判出故障后是这么写的：
 *     detect->cfg->callback.disable_output();
 *     detect->cfg->callback.user_process(detect->status);
 * 而工程里从来没给 user_process 赋过值 —— 它是 NULL。
 * 也就是说：任何一次故障（过流/编码器超时/环路异常）发生时的完整后果是
 * "先关断 PWM，紧接着跳转到地址 0" → 非法指令异常 → CPU 原地 trap。
 * 表现为"电机突然不动了"，但真正的死因被埋掉。装上这个回调后，
 * 故障源会写进 g_fault_src 位图，J-Scope 一眼可辨：
 *   bit0=analog(采样/过流) bit1=loop(环路) bit2=drivers(驱动) bit3=encoder(编码器)
 *
 * 这里只记录不自动恢复：过流锁存是保护，擅自恢复可能烧功率级。
 */
static void motor0_fault_process(mcl_detect_status status)
{
    if ((status.analog != NULL)       && (*status.analog == analog_status_fail))       { g_fault_src |= 1U; }
    if ((status.control_loop != NULL) && (*status.control_loop == loop_status_fail))   { g_fault_src |= 2U; }
    if ((status.drivers != NULL)      && (*status.drivers == drivers_status_fail))     { g_fault_src |= 4U; }
    if ((status.encoder != NULL)      && (*status.encoder == encoder_status_fail))     { g_fault_src |= 8U; }
    g_fault_cnt++;
}

void motor0_control_init(void)
{
}

void motor_init(void)
{
    motor0.cfg.mcl.physical.board.analog[analog_a_current].adc_reference_vol = 3.3;  /* A 相 ADC 参考电压 */
    motor0.cfg.mcl.physical.board.analog[analog_a_current].opamp_gain = 20;          /* A 相运放增益(CSA240L 放大 20 倍，原理图注明) */
    motor0.cfg.mcl.physical.board.analog[analog_a_current].sample_precision = 4095;  /* 采样精度 */
    motor0.cfg.mcl.physical.board.analog[analog_a_current].sample_res = 0.005;       /* 采样电阻 10mΩ */

    motor0.cfg.mcl.physical.board.analog[analog_b_current].adc_reference_vol = 3.3;  /* B 相 */
    motor0.cfg.mcl.physical.board.analog[analog_b_current].opamp_gain = 20;
    motor0.cfg.mcl.physical.board.analog[analog_b_current].sample_precision = 4095;
    motor0.cfg.mcl.physical.board.analog[analog_b_current].sample_res = 0.005;

    motor0.cfg.mcl.physical.board.num_current_sample_res = 2;                       /* A、B 相采样电阻数量 */
    motor0.cfg.mcl.physical.board.pwm_dead_time_tick = PWM_DEAD_AREA_TICK;          /* PWM 死区（防上下桥直通） */
    motor0.cfg.mcl.physical.board.pwm_frequency = PWM_FREQUENCY;
    motor0.cfg.mcl.physical.board.pwm_reload = PWM_RELOAD;
    motor0.cfg.mcl.physical.motor.i_max = 3;                                       /* 最大电流（过流保护阈值） */
    motor0.cfg.mcl.physical.motor.inertia = 6.2e-6;                                 /* 转动惯量 62g·cm² */
    motor0.cfg.mcl.physical.motor.ls =  0.00127f;                                    /* 定子电感 1.27mH(线间)/2 */
    motor0.cfg.mcl.physical.motor.pole_num = 4;                                     /* 极对数 8极/2 */
    motor0.cfg.mcl.physical.motor.power = 63;                                       /* 额定功率 63W */
    motor0.cfg.mcl.physical.motor.res = 2.49f;                                      /* 定子电阻 2.49Ω(线间)/2 @20℃ */
    motor0.cfg.mcl.physical.motor.rpm_max = 3000;                                   /* 额定转速 3000r/min */
    /* 母线电压：冷上电时 ADC 首帧可能还没落定，读失败要兜底，
       否则 const_vbus 变成 -1，dq 解耦和电压限幅全错。
       g_boot_vbus 记下上电瞬间的值：明显低于标称说明还在爬升。 */
    {
        float vbus = read_vbus();
        if (vbus < 1.0f) {
            vbus = MOTOR_VBUS_DEFAULT;
        }
        motor0.cfg.mcl.physical.motor.vbus = vbus;
        g_boot_vbus = vbus;
    }
    motor0.cfg.mcl.physical.motor.flux = 0.009;                                     /* 永磁体磁链 6.5V@1000r/min(线间)→0.009Wb */
    motor0.cfg.mcl.physical.motor.ld =  0.00127f;                                    /* d 轴电感（表贴式 Ld≈Lq） */
    motor0.cfg.mcl.physical.motor.lq =  0.00127f;
    motor0.cfg.mcl.physical.time.adc_sample_ts = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY);
    motor0.cfg.mcl.physical.time.current_loop_ts = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY);
    motor0.cfg.mcl.physical.time.encoder_process_ts = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY);
    motor0.cfg.mcl.physical.time.speed_loop_ts = (MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY)) * 5;
    motor0.cfg.mcl.physical.time.position_loop_ts = (MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY)) * 20;
    motor0.cfg.mcl.physical.time.mcu_clock_tick = clock_get_frequency(clock_cpu0);
    motor0.cfg.mcl.physical.time.pwm_clock_tick = clock_get_frequency(BOARD_BLDC_MOTOR_CLOCK_SOURCE);

    motor0.cfg.analog.enable_a_current = true;          /* 使能 A 相电流采样 */
    motor0.cfg.analog.enable_b_current = true;          /* B */
    memset(motor0.cfg.analog.enable_filter, false, MCL_ANALOG_CHN_NUM);  /* 滤波器默认禁用 */
    motor0.cfg.analog.enable_vbus = true;               /* 使能母线电压 */

    /* encoder 设置 */
    motor0.cfg.encoder.communication_interval_us = 0;   /* 编码器通信间隔 */
    motor0.cfg.encoder.disable_start_sample_interrupt = true; /* 禁用编码器开始采样中断 */
    motor0.cfg.encoder.period_call_time_s = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY); /* 编码器调用周期 */
    motor0.cfg.encoder.precision = BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV; /* 编码器精度 */
    motor0.cfg.encoder.speed_abs_switch_m_t = 5;              /* M/T 法自动切换的绝对速度阈值 */
    motor0.cfg.encoder.speed_cal_method = encoder_method_m;   /* 速度计算方法为 M 法 */
    motor0.cfg.encoder.timeout_s = 0.5;            /* 编码器超时时间 */

    /* loop pass fpass 100 fstop 2000，二阶低通滤波器 */
    motor0.cfg.encoder_iir.section = 2;
    motor0.cfg.encoder_iir.matrix = motor0.cfg.encoder_iir_mat;
    motor0.cfg.encoder_iir_mat[0].a1 = -1.947404031871316831825424742419272661209f;
    motor0.cfg.encoder_iir_mat[0].a2 = 0.95152023575172306468772376319975592196f;
    motor0.cfg.encoder_iir_mat[0].b0 = 1;
    motor0.cfg.encoder_iir_mat[0].b1 = 2;
    motor0.cfg.encoder_iir_mat[0].b2 = 1;
    motor0.cfg.encoder_iir_mat[0].scale = 0.001029050970101526990552187612593115773f;

    motor0.cfg.encoder_iir_mat[1].a1 = -1.88285893096534651114382086234400048852f;
    motor0.cfg.encoder_iir_mat[1].a2 = 0.886838706662149367510039610351668670774f;
    motor0.cfg.encoder_iir_mat[1].b0 = 1;
    motor0.cfg.encoder_iir_mat[1].b1 = 2;
    motor0.cfg.encoder_iir_mat[1].b2 = 1;
    motor0.cfg.encoder_iir_mat[1].scale = 0.000994943924200649039424337871651005116f;

    motor0.cfg.control.callback.init = motor0_control_init;

    /* ---- 电流环 PI ---- */
    motor0.cfg.control.currentd_pid_cfg.cfg.integral_max = 100;                  /* 积分限幅 */
    motor0.cfg.control.currentd_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.currentd_pid_cfg.cfg.output_max = 15;                     /* 输出限幅 */
    motor0.cfg.control.currentd_pid_cfg.cfg.output_min = -15;
#if 0   /* 源码 PI 参数（旧公式：kp/ki 都多乘了 ωc 与 ts，2026-09-10 停用，仅备查） */
    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
#else   /* 理论 PI 参数：标准整定 Kp=Ls*ωc、Ki=Rs*ωc*ts（PI 零点对消电机极点 R/L） */
    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls * MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts;
#endif

    motor0.cfg.control.currentq_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.currentq_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.currentq_pid_cfg.cfg.output_max = 15;
    motor0.cfg.control.currentq_pid_cfg.cfg.output_min = -15;
#if 0   /* 源码 PI 参数（旧公式，2026-09-10 停用，仅备查） */
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
#else
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls * MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts;
#endif

    motor0.cfg.control.dead_area_compensation_cfg.cfg.lowpass_k = 0.1;   /* 死区补偿低通滤波系数 */

    /* ---- 速度环 PI ---- */
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 20;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -20;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 10;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -10;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_HW_FOC_SPEED_KI;
#else
    /* 限幅（2026-09-11 收紧）：
     * output_max 原为 5A，但 physical.motor.i_max = 3A —— 速度环可以要到 5A，
     * 直接顶到过流保护阈值，起转瞬间会被 detect 打断。
     * integral_max 原为 100，而 MCL 的 hpm_mcl_control_pi() 没有抗饱和：
     * 输出被 output_max 削掉后积分仍在累加，最多能攒到 100，退饱和要几十秒，
     * 表现为起转时"猛冲一下"然后长时间回不来。把积分限幅压到和输出限幅一致，
     * 是最省事的抗饱和办法。 */
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 3;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -3;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 3;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -3;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI;
#endif

    /* ---- 位置环 PI ----
       注意：位置环走的是 hpm_mcl_position_pid()，积分语义与速度环不同
       （integral += err，out = kp*err + ki*integral），参数整定见 motor_params.h。
       SDK 原值 kp=154.7 / ki=0.113 / integral_max=10 完全不匹配弧度反馈，
       会导致起步 bang-bang 猛冲 + 到位后被静摩擦卡住。 */
    motor0.cfg.control.position_pid_cfg.cfg.integral_max = BOARD_BLDC_SW_FOC_POSITION_INTEGRAL_MAX;
    motor0.cfg.control.position_pid_cfg.cfg.integral_min = -BOARD_BLDC_SW_FOC_POSITION_INTEGRAL_MAX;
    motor0.cfg.control.position_pid_cfg.cfg.output_max = BOARD_BLDC_SW_FOC_POSITION_OUTPUT_MAX;
    motor0.cfg.control.position_pid_cfg.cfg.output_min = -BOARD_BLDC_SW_FOC_POSITION_OUTPUT_MAX;
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.position_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_POSITION_KP;
#else
    motor0.cfg.control.position_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_POSITION_KP;
#endif
    motor0.cfg.control.position_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_POSITION_KI;

    /* ---- 回调注册 ---- */
    motor0.cfg.drivers.callback.init = pwm_init;
    motor0.cfg.drivers.callback.enable_all_drivers = enable_all_pwm_output;
    motor0.cfg.drivers.callback.disable_all_drivers = disable_all_pwm_output;
    motor0.cfg.drivers.callback.update_duty_cycle = pwm_duty_set;
    motor0.cfg.analog.callback.init = adc_init;
    motor0.cfg.analog.callback.update_sample_location = analog_update_sample_location;
    motor0.cfg.analog.callback.get_value = adc_value_get;
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.encoder.callback.init = NULL;
#else
    motor0.cfg.encoder.callback.init = NULL;   /* 使用 SPI 绝对编码器，不用 QEI */
#endif
    motor0.cfg.encoder.callback.start_sample = encoder_start_sample;
    motor0.cfg.encoder.callback.get_theta = encoder_get_theta;
    motor0.cfg.encoder.callback.get_absolute_theta = encoder_get_abs_theta;

#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.clc.clc_set_val = motor0_clc_set_currentloop_value;
    motor0.cfg.clc.convert_float_to_clc_val = motor0_clc_float_convert_clc;
    motor0.loop.hardware = &motor0.cfg.clc;
#endif

#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.loop.mode = mcl_mode_hardware_foc;
#else
    motor0.cfg.loop.mode = mcl_mode_foc;
#endif
    motor0.cfg.loop.enable_speed_loop = true;    /* 速度环 */
    motor0.cfg.loop.enable_position_loop = true; /* 位置环 */

    motor0.cfg.detect.enable_detect = true;
    motor0.cfg.detect.en_submodule_detect.analog = true;
    motor0.cfg.detect.en_submodule_detect.drivers = true;
    motor0.cfg.detect.en_submodule_detect.encoder = true;
    motor0.cfg.detect.en_submodule_detect.loop = true;
    motor0.cfg.detect.callback.disable_output = disable_all_pwm_output;
    motor0.cfg.detect.callback.user_process = motor0_fault_process;   /* 不装就是 NULL 调用 → 死机 */

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    /* 硬件混合环路模式配置 */
    motor0.cfg.loop.mode = mcl_mode_hybrid_foc;
    motor0.cfg.hw_loop.clc_cfg.base = BOARD_CLC;
    motor0.cfg.hw_loop.callback.clc_convert_input = clc_convert_input;
    motor0.cfg.hw_loop.callback.clc_convert_output = clc_convert_output;
    hpm_mcl_hw_loop_init(&motor0.hw_loop, &motor0.cfg.hw_loop);
    hpm_mcl_enable_clc_hardware_loop(&motor0.hw_loop);
#endif

    hpm_mcl_analog_init(&motor0.analog, &motor0.cfg.analog, &motor0.cfg.mcl);
    hpm_mcl_filter_iir_df1_init(&motor0.encoder_iir, &motor0.cfg.encoder_iir, &motor0.encoder_iir_mem[0]);
    hpm_mcl_encoder_init(&motor0.encoder, &motor0.cfg.mcl, &motor0.cfg.encoder, &motor0.encoder_iir);
    hpm_mcl_drivers_init(&motor0.drivers, &motor0.cfg.drivers);
    hpm_mcl_control_init(&motor0.control, &motor0.cfg.control);
#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                    &motor0.encoder, &motor0.analog, &motor0.control, &motor0.drivers, NULL, &motor0.hw_loop);
#else
    hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                    &motor0.encoder, &motor0.analog, &motor0.control, &motor0.drivers, NULL, NULL);
#endif
    hpm_mcl_detect_init(&motor0.detect, &motor0.cfg.detect, &motor0.loop, &motor0.encoder, &motor0.analog, &motor0.drivers);
    hpm_mcl_enable_dq_axis_decoupling(&motor0.loop);
    hpm_mcl_enable_dead_area_compensation(&motor0.loop);
}

void motor0_speed_loop_para_init(void)
{
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 20;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -20;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 10;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -10;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_HW_FOC_SPEED_KI;
#else
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 5;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -5;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI;

    hpm_mcl_disable_dead_area_compensation(&motor0.loop);

    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
#endif
}

void motor0_position_loop_para_init(void)
{
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 20;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -20;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 10;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -10;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_HW_FOC_SPEED_KI;
#else
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 5;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -5;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KI;

    hpm_mcl_enable_dead_area_compensation(&motor0.loop);

    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
#endif
}

void motor_set_loop_mode(motor_loop_mode_t mode)
{
    switch (mode) {
    case MOTOR_LOOP_POSITION:
        motor0.cfg.loop.enable_speed_loop    = true;
        motor0.cfg.loop.enable_position_loop = true;
        break;
    case MOTOR_LOOP_SPEED:
        motor0.cfg.loop.enable_speed_loop    = true;
        motor0.cfg.loop.enable_position_loop = false;   /* 必须关，否则位置环空转拖累速度环 */
        break;
    case MOTOR_LOOP_CURRENT:
    default:
        motor0.cfg.loop.enable_speed_loop    = false;
        motor0.cfg.loop.enable_position_loop = false;
        break;
    }
}

void motor_angle_align(void)
{
    mcl_motor_alignment_cfg_t alignment_cfg;

    /* 三段式对齐算法 */
    alignment_cfg.algorithm = mcl_alignment_algorithm_three_stage;

#if defined(HW_CURRENT_FOC_ENABLE)
    /* 硬件 FOC 专有初始化 */
    qeo_enable_software_position_inject(BOARD_BLDC_QEO);
    qeo_software_position_inject(BOARD_BLDC_QEO, 0);
    qeo_disable_software_position_inject(BOARD_BLDC_QEO);
    vsc_sw_inject_pos_value(BOARD_VSC, 0);

    alignment_cfg.config.three_stage.stage1.d_current = 8.0f;    /* 阶段 1 大电流 */
    alignment_cfg.config.three_stage.stage1.q_current = 0.5f;    /* 阶段 1 q 轴扰动 */
    alignment_cfg.config.three_stage.stage1.delay_ms = 500;

    alignment_cfg.config.three_stage.stage2.d_current = 5.0f;    /* 阶段 2 中电流 */
    alignment_cfg.config.three_stage.stage2.delay_ms = 800;

    alignment_cfg.config.three_stage.stage3.d_current = 3.0f;    /* 阶段 3 小电流 */
    alignment_cfg.config.three_stage.stage3.delay_ms = 400;
#else
    /* 软件 FOC 三段式参数 */
    alignment_cfg.config.three_stage.stage1.d_current = 4.0f;
    alignment_cfg.config.three_stage.stage1.q_current = 0.6f;
    alignment_cfg.config.three_stage.stage1.delay_ms = 500;

    alignment_cfg.config.three_stage.stage2.d_current = 1.5f;
    alignment_cfg.config.three_stage.stage2.delay_ms = 800;

    alignment_cfg.config.three_stage.stage3.d_current = 1.0f;
    alignment_cfg.config.three_stage.stage3.delay_ms = 400;
#endif

    alignment_cfg.config.three_stage.final_delay_ms = 100;

    hpm_mcl_motor_angle_alignment(&motor0.loop, &alignment_cfg);

#if defined(HW_CURRENT_FOC_ENABLE)
    qei_init();
    trigmux_init_3();
#endif
}
