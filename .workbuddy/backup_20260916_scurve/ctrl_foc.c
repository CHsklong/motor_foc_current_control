/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "motor.h"
#include "motor_params.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "sens_encoder.h"
#include "sens_analog.h"
#include "ctrl_svpwm.h"
#include "ctrl_foc.h"
#include "hpm_mcl_control.h"
#include "planner_scurve.h"
#include "dbg_probe.h"
#include "app_cfg.h"
#include <math.h>

float g_ia = 0.0f;  /* A 相采样电流 A */
float g_ib = 0.0f;  /* B 相采样电流 A */
float g_ic = 0.0f;  /* C 相采样电流 A */
float speed_rad_s;  /* 机械角速度 rad/s */
float rpm;          /* 转/分钟 */
float fault_level;

#if FOC_POSITION_MODE
static float s_pos_vlim = 0.0f;          /* 本拍位置环输出限幅 */
static float s_pos_integral_prv = 0.0f;  /* 本拍位置环执行前的积分值 */
static float s_omega_ff = 0.0f;          /* 本拍 S 曲线速度前馈 rad/s */
static float s_pos_pid_out = 0.0f;       /* 本拍 PID 自身输出（加前馈之前），抗饱和判定用 */

/* S 曲线规划器 + 20kHz → 位置环周期的分频计数 */
static scurve_t   s_sc;
static uint32_t   s_pos_div = 0U;        /* 几个 20kHz 拍才执行一次位置环 */
static uint32_t   s_pos_cnt = 0U;

/* 原 MCL 位置环 PID 的函数指针。我们把它换成带前馈的包装，
   因为 MCL 里 position_pid() 写完 exec_ref.speed 后，同一拍速度环立刻消费，
   在 hpm_mcl_loop() 之后再注入前馈就晚了一拍（且会被下一拍的 PID 输出覆盖）。 */
static hpm_mcl_stat_t (*s_pos_pid_orig)(hpm_mcl_type_t, hpm_mcl_type_t,
                                        mcl_control_pid_t *, hpm_mcl_type_t *) = NULL;

/** 位置环 PID 包装：原输出 + S 曲线速度前馈，再统一限幅 */
static hpm_mcl_stat_t position_pid_with_ff(hpm_mcl_type_t setpoint, hpm_mcl_type_t feedback,
                                           mcl_control_pid_t *pid_x, hpm_mcl_type_t *output)
{
    hpm_mcl_stat_t st;
    float v;
    float lim = BOARD_BLDC_SW_FOC_POSITION_OUTPUT_MAX;

    if (s_pos_pid_orig != NULL) 
    {
        st = s_pos_pid_orig(setpoint, feedback, pid_x, output);
    } else 
    {
        st = hpm_mcl_position_pid(setpoint, feedback, pid_x, output);
    }
    if (st != mcl_success) {
        return st;
    }

    /* 先留一份"PID 自己的输出"，抗饱和判定要用它，
       不能用加完前馈的 *output —— 见 position_loop_post() 的注释 */
    s_pos_pid_out = *output;

    v = *output + s_omega_ff * POS_SCURVE_FF_GAIN;
    if (v >  lim) { v =  lim; }
    if (v < -lim) { v = -lim; }
    *output = v;

    return st;
}

void ctrl_pos_init(void)
{
    /* 位置环周期是 speed_loop_ts（MCL 里 position_ts 直接指向它），
       用它在运行时换算分频比，避免把 4kHz / 20kHz 写死。 */
    float pos_ts = *motor0.loop.const_time.position_ts;
    float f      = (float)PWM_FREQUENCY * pos_ts;

    s_pos_div = (f >= 1.0f) ? ((uint32_t)(f + 0.5f)) : 1U;
    /* 兜底：配置异常时 div 可能算成 0 或巨大值。
       div=0  → 每个 20kHz 拍都推进，轨迹被加速 5 倍跑完；
       div 过大 → 规划器几乎不推进，前馈恒为 0、给定恒为起点，电机完全不动。 */
    if (s_pos_div == 0U)   { s_pos_div = 1U; }
    if (s_pos_div > 50U)   { s_pos_div = 50U; }
    s_pos_cnt = 0U;

    scurve_init(&s_sc, pos_ts);

    /* 只包一层，重复调用不会套娃 */
    if (motor0.loop.control->method.position_pid != position_pid_with_ff) {
        s_pos_pid_orig = motor0.loop.control->method.position_pid;
        motor0.loop.control->method.position_pid = position_pid_with_ff;
    }
}

void ctrl_pos_start(float delta)
{
    mcl_user_value_t up;
    float th_now = 0.0f;

    if (s_pos_div == 0U) 
    {
        ctrl_pos_init();          /* 未初始化就先补一次，避免 div=0 把轨迹加速跑完 */
    }

    /* 轨迹起点直接取 g_pos_abs：20kHz 中断里的 MCL 编码器回调每拍都在刷新它，
       32 位对齐 float 读取天然原子，且 encoder_abs_rebase() 已把它清零作为新基准。
       绝不在 main 上下文里做 SPI 事务（ISR 才是 SPI 的唯一拥有者）——
       之前"关中断 + 主上下文 SPI 读"的写法实测导致 CPU 冻结在 step1（2026-09-15）。 */
    th_now = g_pos_abs;

    up.enable = true;
    /* 给定先写成终点：规划器一旦运行会被逐拍覆盖；万一它没跑起来
       （状态停留在 IDLE），至少还能按原来的"固定目标 + 接近限速"走到终点，
       不会像早期版本那样把给定写死在起点、电机一动不动。 */
    up.value  = th_now + delta;

    motor0.cfg.control.position_pid_cfg.integral = 0.0f;
    /* 【顺序很重要】先把给定写进 MCL，再去规划轨迹。
       反过来写的话，只要 scurve_start() 因故没返回（参数异常/被打断），
       hpm_mcl_loop_set_position() 就永远执行不到 → ref_position.enable 一直 false
       → MCL 退回用 exec_ref.position(恒 0) 当给定 → 误差恒 0 → 电机一动不动，
       而中断和心跳都正常，极难排查。先写给定就把这条路径堵死了。 */
    hpm_mcl_loop_set_position(&motor0.loop, up);
    g_pos_ref = up.value;

#if POS_SCURVE_ENABLE
    scurve_start(&s_sc, th_now, delta,
                 POS_SCURVE_V_MAX, POS_SCURVE_A_MAX, POS_SCURVE_J_MAX);
    s_omega_ff = 0.0f;
#else
    scurve_init(&s_sc, *motor0.loop.const_time.position_ts);
#endif
}

/** hpm_mcl_loop() 之前调用：推进轨迹、写入本拍给定、设定限幅 */
static void position_loop_pre(void)
{
    mcl_control_pid_t *p = &motor0.cfg.control.position_pid_cfg;
    float out_max = BOARD_BLDC_SW_FOC_POSITION_OUTPUT_MAX;
    float err;

    /* 只在位置环真正执行的那一拍推进规划器（与 MCL 的 position_ts 累加同相位） */
    if (++s_pos_cnt >= s_pos_div) {
        s_pos_cnt = 0U;
    #if POS_SCURVE_ENABLE
        scurve_step(&s_sc);
        s_omega_ff = s_sc.w;
    #else
        s_omega_ff = 0.0f;
    #endif
    }

#if POS_SCURVE_ENABLE
    /* 只有规划器真的在跑才接管给定。IDLE 时保持 main 里写入的终点目标，
       退化为原来的固定目标模式 —— 否则会把给定写死在起点、电机一动不动。 */
    if (s_sc.state != SCURVE_IDLE) {
        motor0.loop.ref_position.value = s_sc.th;   /* 移动的目标位置 */
        g_pos_ref = s_sc.th;
    }

    /* ---- 失速保护（必须在上面改写完给定之后再判） ----
       轨迹在跑，但电机明显没跟上（跟随误差超过 POS_SCURVE_ABORT_ERR）。
       再跟着规划点走只会让误差越拉越大；起步段前馈本身很小，一旦破不了静摩擦
       就再也追不回来。这里直接放弃轨迹 → 退回"固定目标 + 接近限速"
       （SCURVE=0 时验证过能转的那条路径），保证电机一定会转起来并走到终点。
       正常跟踪时跟随误差只有 0.0x rad，阈值留了几十倍余量，不会误触发。 */
    err = g_pos_ref - g_pos_abs;
    if ((s_sc.state == SCURVE_RUN) &&
        ((err > POS_SCURVE_ABORT_ERR) || (err < -POS_SCURVE_ABORT_ERR))) {
        scurve_abort(&s_sc);        /* → DONE，th 恒等于 th0+delta */
        s_omega_ff    = 0.0f;
        s_pos_pid_out = 0.0f;
        motor0.loop.ref_position.value = s_sc.th;
        g_pos_ref = s_sc.th;
        err = g_pos_ref - g_pos_abs;
        g_sc_abort++;
    }

    /* 观测：规划位置 / 规划速度（前馈量）。实际位置看 g_pos_abs，
       实际速度看 g_spd_fdb —— 两两对照即可在 J-Scope 里看出跟随情况。 */
    g_sc_theta = s_sc.th;
    g_sc_omega = s_sc.w;
    g_sc_state = (uint32_t)s_sc.state;
#endif

    err = g_pos_ref - g_pos_abs;
    s_pos_integral_prv = p->integral;

#if POS_SCURVE_ENABLE
    if (s_sc.state == SCURVE_RUN) {
        /* 轨迹跟踪阶段：减速已由 S 曲线负责，前馈承担主要运动，
           PID 只需用"剩下的额度"补跟随误差。
           【2026-09-15】原先这里还用一个固定上限（8rad/s）再截一刀，
           起步阶段前馈≈0、预算本来是满的 30，被砍到 8 之后
           一旦静摩擦偏大就没有裕度把电机拽动 —— 这正是"SCURVE=1 完全不转"的主因。
           现在只保一个下限 POS_TRACK_VLIM_MIN；总和不会超上限，
           前馈包装 position_pid_with_ff() 最后统一限到 ±OUTPUT_MAX。 */
        float budget = out_max - ((s_omega_ff >= 0.0f) ? s_omega_ff : -s_omega_ff);
        if (budget < POS_TRACK_VLIM_MIN) { budget = POS_TRACK_VLIM_MIN; }
        if (budget > out_max)            { budget = out_max; }
        s_pos_vlim = budget;
    } else
#endif
    {
        /* 轨迹结束后（或未启用规划）：回到接近限速逻辑收拾残差 */
        float aerr = ((err >= 0.0f) ? err : -err) - POS_ARRIVE_DEADBAND;
        if (aerr <= 0.0f) {
            s_pos_vlim = 0.0f;                              /* 到位：输出 0，积分在 post 里泄放 */
        } else {
            float v = sqrtf(2.0f * POS_DECEL_MAX * aerr);   /* 接近限速 */
            s_pos_vlim = (v < out_max) ? v : out_max;
        }
    }

    p->cfg.output_max =  s_pos_vlim;
    p->cfg.output_min = -s_pos_vlim;

    g_pos_err  = err;
}

/** hpm_mcl_loop() 之后调用：抗饱和回退与观测刷新 */
static void position_loop_post(void)
{
    mcl_control_pid_t *p = &motor0.cfg.control.position_pid_cfg;
    float out = motor0.loop.exec_ref.speed;   /* 位置环本拍输出（速度给定） */

    if (s_pos_vlim <= 0.0f) 
    {
        /* 到位：输出 0，积分线性泄放到 0，
           避免减速段攒下的积分在下次微小扰动时把电机推离目标 */
        float i = p->integral;
        if (i > POS_INTEGRAL_LEAK) 
        {
            p->integral = i - POS_INTEGRAL_LEAK;
        } else if (i < -POS_INTEGRAL_LEAK)
        {
            p->integral = i + POS_INTEGRAL_LEAK;
        } else 
        {
            p->integral = 0.0f;
        }
    } 
    else if (((s_pos_pid_out >=  s_pos_vlim - 0.001f) || (s_pos_pid_out <= -s_pos_vlim + 0.001f))
             && (g_pos_err * s_pos_pid_out > 0.0f))
    {
        /* 撞限幅且误差还在同方向推：撤销本拍的积分累加
           【2026-09-15】判饱和必须用【PID 自己的输出 s_pos_pid_out】，不能用 out。
           out 是加完前馈的 exec_ref.speed，S 曲线模式下它≈前馈量（可达 25rad/s），
           而 s_pos_vlim 只是"留给 PID 的预算"（巡航时只有 5），
           拿 out 去比会每拍都误判为饱和 → 积分被永久冻结在 0，
           位置环退化成纯 P，起步和末端都没有积分可用。 */
        p->integral = s_pos_integral_prv;
    }

    g_pos_integral = p->integral;
    g_ref_speed    = out;
}
#endif

/**
 * @brief ADC 转换完成中断 —— 整个电机控制的节拍源（20kHz）
 *
 * 编码器角度必须与电流环同频更新：放在主循环里只有 ~1kHz，
 * 而且传给 hpm_mcl_encoder_process() 的 tick 是按 50us 算的，
 * 速度会被放大约 20 倍，导致预测角和 dq 解耦前馈
 * uq += w*pole_num*(Ld*iq+flux) 严重失真。
 */
SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_ADC_IRQn, isr_adc)
void isr_adc(void)
{
    uint32_t status;
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);

    g_heart_isr++;   /* 存活心跳：J-Scope 里不递增即说明 20kHz 节拍根本没起来 */

    status = hpm_adc_v2_get_status_flags(adc_u);
    if ((status & HPM_ADC_V2_EVENT_TRIG_COMPLETE) != 0)
    {
        hpm_adc_v2_clear_status_flags(adc_u, HPM_ADC_V2_EVENT_TRIG_COMPLETE);

        if (g_encoder_isr_enable) 
        {
            hpm_mcl_encoder_process(&motor0.encoder, motor0.cfg.mcl.physical.time.mcu_clock_tick / PWM_FREQUENCY);
        }
        hpm_mcl_analog_get_value(&motor0.analog, analog_a_current, &g_ia);
        hpm_mcl_analog_get_value(&motor0.analog, analog_b_current, &g_ib);    /* 实际采样为 C 相 */
        g_ic = -g_ia - g_ib;
        speed_rad_s = motor0.encoder.result.speed;                    /* 机械角速度 rad/s */
        rpm = motor0.encoder.result.speed * 60.0f / (2.0f * MCL_PI);  /* 转/分钟 */

        /* 实际机械角速度探针：与 g_sc_omega（S 曲线规划速度）同图对照，
           即可看出"速度跟不跟得上规划"。必须在 20kHz 里刷新，否则 J-Scope
           HSS 采到的是冻结值。 */
        g_spd_fdb = motor0.encoder.result.speed;

        if (SVPWM_MODE) 
        {
            svpwm_openloop_step();
        }

        if (FOC_MODE) 
        {
        #if FOC_POSITION_MODE
                    position_loop_pre();     /* 位置环：设定本拍动态限幅 */
        #endif
                    hpm_mcl_loop(&motor0.loop);  /* 电流环 / 速度环 / 位置环 */
        #if FOC_POSITION_MODE
                    position_loop_post();    /* 位置环：抗饱和回退 + 观测刷新 */
        #endif
        }
    }
}
