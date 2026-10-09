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
#include "traj_s_curve.h"
#include "hpm_mcl_control.h"
#include "dbg_probe.h"
#include "motor_scope.h"     /* motor_scope_isr_tick：20kHz 里刷新录波通道并采样 */
#include "app_cfg.h"
#include <math.h>

float g_ia = 0.0f;  /* A 相采样电流 A */
float g_ib = 0.0f;  /* B 相采样电流 A */
float g_ic = 0.0f;  /* C 相采样电流 A */
float speed_rad_s;  /* 机械角速度 rad/s */
float rpm;          /* 转/分钟 */
float fault_level;

#if FOC_POSITION_MODE && SCURVE_ENABLE
/* ==================== S 曲线规划 + 位置外环（前馈 + 修正） ==================== */
static uint8_t s_pos_decim;          /* 20kHz -> 4kHz 分频计数 */
static float   s_v_cmd;              /* 本拍最终速度给定 rad/s */
static float   s_corr;               /* 本拍位置外环修正量（原始值）rad/s */
static float   s_corr_lp;            /* 修正量低通输出（实际叠加进速度给定）rad/s */
static float   s_corr_alpha;         /* 低通系数（首次调用时按 SCURVE_CORR_FC 算好） */
static float   s_pos_integral_prv;   /* 本拍外环执行前的积分值 */

/** hpm_mcl_loop() 之前：推进 S 曲线 → 位置外环 → 下发速度给定 */
static void position_loop_pre(void)                             /*S 曲线轨迹跟踪 + 前馈 + 受限 PID 修正 + 滤波*/
{
    mcl_control_pid_t *p = &motor0.cfg.control.position_pid_cfg;
    mcl_user_value_t   cmd;
    float pos_fb, err, out;

    /* ---- 1) 位置参考 = S 曲线输出---- */
    g_pos_ref = g_sc.p;

    /* ---- 2) 位置外环 4kHz ---- */
    if (++s_pos_decim < POS_LOOP_DIV) 
    {
        return;
    }
    s_pos_decim = 0;

    if (encoder_get_abs_theta(&pos_fb) != mcl_success) 
    {
        pos_fb = g_pos_abs;          /* 偶发读失败：沿用上一拍 */
    }
    g_pos_abs = pos_fb;

    err = g_pos_ref - pos_fb;
    g_pos_err = err;
    s_pos_integral_prv = p->integral;

    /* 修正器权限：只给"修正权"，主体速度由前馈出 */
    p->cfg.output_max =  POS_CORR_MAX;
    p->cfg.output_min = -POS_CORR_MAX;

    hpm_mcl_position_pid(g_pos_ref, pos_fb, p, &out);
    s_corr = out;

    /* 修正量一阶低通（4kHz）：把修正里的高频毛刺滤掉再叠加进速度给定。
       外环交点在几 Hz 量级，30Hz 的低通对它几乎无相位损失，
       但能把"打摆"的高频成分和编码器量化毛刺挡在速度环外面。
       设 SCURVE_CORR_FC=0 可旁路（完全等价于不做滤波）。 */
    if (SCURVE_CORR_FC > 0.0f)                            /* 低通滤波 */
    {
        if (s_corr_alpha == 0.0f) 
        {
            float w = 2.0f * MCL_PI * SCURVE_CORR_FC * (MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY) * POS_LOOP_DIV);
            s_corr_alpha = w / (1.0f + w);
        }
        s_corr_lp += s_corr_alpha * (out - s_corr_lp);
    } else 
    {
        s_corr_lp = out;
    }

    /* 前馈 + 修正，再按"规划速度 + 修正权限"做总限幅 */
    s_v_cmd = s_corr_lp + g_sc.v + SCURVE_ACC_FF_TAU * g_sc.a;

    /* 非有限值兜底：NaN/Inf 与任何数的比较都为假，下面的限幅会原样把它放过去，
       一路穿透到速度环→电流环→PWM 比较值，表现就是"上电偶发过流"。
       这里先判掉并强制 0（保持静止），同时计数，便于定位是哪个分量先出现的。 */
    if ((s_v_cmd != s_v_cmd) || (s_v_cmd > 1.0e30f) || (s_v_cmd < -1.0e30f))
    {
        s_v_cmd = 0.0f;
        s_corr_lp = 0.0f;               /* 修正器状态也要清掉，否则 NaN 会永久粘住 */
        g_sc_nan++;
    }

    if (s_v_cmd >  SCURVE_V_CMD_MAX) { s_v_cmd =  SCURVE_V_CMD_MAX; }
    if (s_v_cmd < -SCURVE_V_CMD_MAX) { s_v_cmd = -SCURVE_V_CMD_MAX; }

    cmd.enable = true;               /* 必须 enable，否则速度环会取 exec_ref.speed(=0) */
    cmd.value  = s_v_cmd;
    hpm_mcl_loop_set_speed(&motor0.loop, cmd);
}

/** hpm_mcl_loop() 之后：抗饱和回退 + 到位后的积分泄放 + 探针刷新 */
static void position_loop_post(void)
{
    mcl_control_pid_t *p = &motor0.cfg.control.position_pid_cfg;
    float aerr = ((g_pos_err >= 0.0f) ? g_pos_err : -g_pos_err) - POS_ARRIVE_DEADBAND;

    if (((s_corr >=  POS_CORR_MAX - 0.001f) || (s_corr <= -POS_CORR_MAX + 0.001f))
        && (g_pos_err * s_corr > 0.0f))
    {
        /* 修正撞限幅且误差还在同方向推：撤销本拍的积分累加 */
        p->integral = s_pos_integral_prv;
    }

    if ((g_sc.state != SCURVE_RUN) && (aerr <= 0.0f))
    {
        /* 到位：积分线性泄放到 0，避免残余积分在下次扰动时把电机推离目标 */
        float i = p->integral;
        if (i > POS_INTEGRAL_LEAK) {
            p->integral = i - POS_INTEGRAL_LEAK;
        } else if (i < -POS_INTEGRAL_LEAK) {
            p->integral = i + POS_INTEGRAL_LEAK;
        } else {
            p->integral = 0.0f;
        }
    }

    g_pos_integral = p->integral;
    g_ref_speed    = s_v_cmd;
    /* ---- S 曲线探针（4kHz 刷新，J-Scope 直接看规划器在干什么）---- */
    g_sc_tick    = g_sc.tick;           /* 中断喂拍镜像：不涨说明 scurve_isr_tick() 没被调用 */
    g_sc_t       = scurve_elapsed();
    g_sc_v_ref   = g_sc.v;
    g_sc_a_ref   = g_sc.a;
    g_sc_v_cmd   = s_v_cmd;
    g_sc_corr    = s_corr;
    g_sc_state   = (uint32_t)g_sc.state;
    g_sc_shape   = (uint32_t)g_sc.shape;
    g_sc_t_all   = g_sc.t_all;
    g_sc_vp      = g_sc.vp;
    g_sc_move_cnt = g_sc.move_cnt;
    g_sc_busy_cnt = g_sc.busy_cnt;
}
#elif FOC_POSITION_MODE
/* ============ 原逻辑：阶跃给定 + 接近限速（SCURVE_ENABLE=0 的回退路径） ============ */
static float s_pos_vlim = 0.0f;          /* 本拍位置环输出限幅 */
static float s_pos_integral_prv = 0.0f;  /* 本拍位置环执行前的积分值 */

/** hpm_mcl_loop() 之前调用：按剩余距离设定本拍输出限幅（接近限速） */
static void position_loop_pre(void)
{
    mcl_control_pid_t *p = &motor0.cfg.control.position_pid_cfg;
    float out_max = BOARD_BLDC_SW_FOC_POSITION_OUTPUT_MAX;
    float err  = g_pos_ref - g_pos_abs;
    float aerr = ((err >= 0.0f) ? err : -err) - POS_ARRIVE_DEADBAND;

    s_pos_integral_prv = p->integral;

    /* 以减速度 POS_DECEL_MAX 从当前位置刚好停到目标的速度上限。
       死区是"对误差的抵扣"而不是硬开关，所以跨过死区边界时限幅从 0 连续长起来。 */
    if (aerr <= 0.0f)
    {
        s_pos_vlim = 0.0f;                              /* 到位：输出 0，积分在 post 里泄放 */
    }
    else
    {
        float v = sqrtf(2.0f * POS_DECEL_MAX * aerr);   /* 接近限速 */
        s_pos_vlim = (v < out_max) ? v : out_max;
    }

    p->cfg.output_max =  s_pos_vlim;
    p->cfg.output_min = -s_pos_vlim;

    g_pos_err = err;
}

/** hpm_mcl_loop() 之后调用：抗饱和回退与到位后的积分泄放 */
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
        }
        else if (i < -POS_INTEGRAL_LEAK)
        {
            p->integral = i + POS_INTEGRAL_LEAK;
        }
        else
        {
            p->integral = 0.0f;
        }
    }
    else if (((out >=  s_pos_vlim - 0.001f) || (out <= -s_pos_vlim + 0.001f))
             && (g_pos_err * out > 0.0f))
    {
        /* 撞限幅且误差还在同方向推：撤销本拍的积分累加 */
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
/**
 * @brief 20kHz 中断耗时统计
 *
 * 节拍预算是 1/PWM_FREQUENCY（20kHz → 50us）。一旦单拍超时：
 *   - 下一次 ADC 触发紧接着就到，CPU 几乎全在中断里 → main 被饿死
 *     （表现为 g_heart_isr 猛涨、g_heart_main 一动不动）；
 *   - 控制时序抖动，电流环出力异常，电机异响或干脆不起转。
 * 所以这个数字是判断"到底是 main 卡住，还是被中断饿死"的决定性依据。
 */
/**
 * @brief 周期数换算成 us 的除数（首次调用时按 CPU 主频算好，之后恒定）
 *
 * 换算放在 ISR 里而不是主循环：main 被饿死/卡死时，J-Scope 里照样能看到
 * 中断耗时，不必再靠"main 活着"才能出数。
 */
static uint32_t isr_cycles_per_us(void)
{
    static uint32_t per_us;

    if (per_us == 0U)
    {
        per_us = motor0.cfg.mcl.physical.time.mcu_clock_tick / 1000000U;
        if (per_us == 0U)
        {
            per_us = 1U;    /* 防除零：配置没跑时就按 1 处理 */
        }
    }
    return per_us;
}

/* ============ 节拍守护：窗口统计 / main 停滞接管 / 应急让路 =============
 *
 * 【为什么要这些】实测出现过"每一拍都超出 50us 预算"的持续状态：CPU 100% 吃在
 * 20kHz 中断里，主循环一条指令都抢不到 → g_heart_main 冻结、系统看着像死机
 * （g_boot_step=17 之后再没动过）。而那时负责自愈的运行监护恰恰跑在被饿死的
 * main 里，等于把唯一的救援通道堵死了。
 *
 * 三道措施：
 *   ① 窗口化计时（20ms 一窗）：把"开机以来的最大值"换成"本窗口最大值"，
 *      从而区分启动瞬间的冷峰值与运行期反复出现的持续超载。
 *   ② 停滞接管：每 20ms 检查 main 心跳还推不推进；不推进就由本 ISR 代跑
 *      运行监护（解除 PWM 锁存、恢复输出仍有机会执行），不再依赖 main。
 *   ③ 应急让路：连续 3 个窗口（60ms）仍停滞 → 每 4 拍跳过 1 次控制计算，
 *      强行给 main 留出时间片。
 */
#define ISR_WINDOW_TICKS   (PWM_FREQUENCY / 50U)   /* 20ms 一个统计窗口 */
#define ISR_RELIEF_DIV     (4U)                    /* 让路模式：每 4 拍跳过 1 拍 */
#define ISR_STARVE_WIN_MAX (3U)                    /* 连续停滞几个窗口后开始让路 */

static uint32_t s_win_ticks;
static uint32_t s_win_over;
static uint32_t s_win_isr_max;
static uint32_t s_win_enc_max;
static uint32_t s_win_loop_max;
static uint32_t s_hm_last;        /* 上次检查时的 main 心跳值 */
static uint32_t s_starve_win;     /* 连续检出 main 停滞的窗口数 */
static uint8_t  s_relief;         /* 1 = 让路模式生效中 */
static void (*s_keeper_hook)(void);   /* main 停摆时由 ISR 代办的运行监护 */

/**
 * @brief 注册运行监护钩子
 * @param hook 监护函数（应用层实现），NULL 表示不启用
 *
 * 注册后，一旦检测到主循环停滞，本模块会在 20kHz 中断里代跑监护逻辑。
 */
void ctrl_set_keeper_hook(void (*hook)(void))
{
    s_keeper_hook = hook;
}

/** 每拍更新窗口统计；窗口结束时结算窗口峰值并做停滞检查 */
static void isr_window_update(uint32_t dt, uint32_t budget)
{
    s_win_ticks++;
    if (dt > budget) {
        s_win_over++;
    }
    if (dt > s_win_isr_max) {
        s_win_isr_max = dt;
    }

    if (s_win_ticks < ISR_WINDOW_TICKS) {
        return;
    }

    /* ---- 窗口结算：只看本窗口，不再被开机冷峰值污染 ---- */
    {
        uint32_t per_us = isr_cycles_per_us();
        g_isr_us_win_max  = s_win_isr_max / per_us;
        g_enc_us_win_max  = s_win_enc_max / per_us;
        g_loop_us_win_max = s_win_loop_max / per_us;
        g_isr_win_over    = (s_win_over * 100U) / s_win_ticks;   /* 本窗口超时占比 % */
        g_isr_win_cnt++;
    }
    s_win_ticks     = 0U;
    s_win_over      = 0U;
    s_win_isr_max   = 0U;
    s_win_enc_max   = 0U;
    s_win_loop_max  = 0U;

    /* ---- 判断主循环是否正常 ---- */
    if (g_heart_main != s_hm_last)
    {
        s_hm_last   = g_heart_main;
        s_starve_win = 0U;
        s_relief     = 0U;              /* main 已恢复，退出让路 */
        return;
    }

    s_starve_win++;
    g_isr_starve_cnt++;

    /* main 停摆：立即由中断代跑运行监护，保住自愈通道 */
    if (s_keeper_hook != NULL) {
        s_keeper_hook();
    }

    /* 连续停滞才开始让路——偶发的一两个窗口（如启动瞬间）不触发 */
#if ISR_RELIEF_ENABLE
    if (s_starve_win >= ISR_STARVE_WIN_MAX) {
        s_relief = 1U;
    }
#endif
}

static void isr_timed_end(uint32_t t0)
{
    uint32_t dt     = (uint32_t)hpm_csr_get_core_cycle() - t0;
    uint32_t budget = motor0.cfg.mcl.physical.time.mcu_clock_tick / PWM_FREQUENCY;
    uint32_t per_us = isr_cycles_per_us();

    g_isr_cycles_last = dt;
    if (dt > g_isr_cycles_max) {
        g_isr_cycles_max = dt;
    }
    if (dt > budget) {
        g_isr_over_cnt++;    /* 超时拍数：持续增加就是节拍跑不完 */
    }
    g_isr_us_last = dt / per_us;
    if (g_isr_us_last > g_isr_us_max) {
        g_isr_us_max = g_isr_us_last;
    }

    /* 分段耗时：编码器段慢 → SPI 问题；环路段慢 → 控制计算问题 */
    g_enc_us_max  = g_enc_cycles_max / per_us;
    g_loop_us_max = g_loop_cycles_max / per_us;

    isr_window_update(dt, budget);
}

#if FOC_SPEED_MODE && SCURVE_SPD_ENABLE
/* ============ 速度 S 曲线：把规划器输出下发给速度环 ============
 *
 * 与位置模式的前馈结构不同：速度模式没有位置外环，速度给定直接由规划器出，
 * 所以这里只做一件事 —— 把 scurve_spd_out() 写进速度环给定
 * 目标速度（CAN 0x101 / main 里的常量）由 scurve_spd_move() 设定，
 * 斜坡本身在 20kHz 的 scurve_spd_isr_tick() 里推进。
 * 下发速率取 4kHz（POS_LOOP_DIV 分频）：与速度环节拍一致，再快没有意义
 * （速度环本来就 4kHz 才取一次给定），还能省一点中断时间。 */
static uint8_t s_spd_decim;

static void speed_ramp_pre(void)
{
    mcl_user_value_t cmd;

    if (++s_spd_decim < POS_LOOP_DIV) {
        return;
    }
    s_spd_decim = 0;

    cmd.enable = true;              /* 必须 enable，否则速度环会取 exec_ref.speed(=0) */
    /* 给定 = 斜坡本体 + 加速度前馈 tau*a（补速度环跟踪斜坡的一阶滞后）。 */
    cmd.value  = scurve_spd_out() + SCURVE_SPD_ACC_FF_TAU * g_scs.a;
    hpm_mcl_loop_set_speed(&motor0.loop, cmd);

    /* g_ref_speed 统一表示"本拍真正下发给速度环的给定"
       注意别把它当目标速度看：目标速度在 g_can_spd_cmd，
       这里赋值的就是 g_scs.v斜坡输出（S 曲线本体）。 */
    g_ref_speed = cmd.value;
}
#endif

void isr_adc(void)
{
    uint32_t status;
    uint32_t t0 = (uint32_t)hpm_csr_get_core_cycle();   /* 计时起点 */
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);

    g_heart_isr++;   /* 存活心跳：J-Scope 里不递增即说明 20kHz 节拍根本没起来 */

    /* ELF 变量录波：刷新六路通道 + 按分频采样（上位机"ELF 与录波"页的数据源）。
       只在 ARMED/TRIGGERED 状态下写缓冲，IDLE 时开销是 10 次 volatile 存，可忽略。 */
    motor_scope_isr_tick();

#if FOC_POSITION_MODE && SCURVE_ENABLE
    scurve_isr_tick();
#endif

#if FOC_SPEED_MODE && SCURVE_SPD_ENABLE
    scurve_spd_isr_tick();      /* 速度域斜坡推进（时间基准必须每拍连续） */

    /* J-scope变量储存 */
    g_scs_v        = g_scs.v;
    g_scs_a        = g_scs.a;
    g_scs_j        = g_scs.j;
    g_scs_state    = (uint32_t)g_scs.state;
    g_scs_move_cnt = g_scs.move_cnt;
    g_scs_tick     = g_scs.tick;
    g_scs_v0       = g_scs.v0;
    g_scs_dv       = g_scs.dv;
    g_scs_ta       = g_scs.ta;
    /* 加/减速限值与其生效结果：CAN SET_ACCEL / SET_DECEL 有没有落到规划器，看这四个 */
    g_scs_amax     = g_scs.amax;
    g_scs_ap       = g_scs.ap;
    g_scs_lim_acc  = g_scs.amax_acc;
    g_scs_lim_dec  = g_scs.amax_dec;
    g_scs_stage2   = (uint32_t)g_scs.stage2;
#endif

    status = hpm_adc_v2_get_status_flags(adc_u);
    if ((status & HPM_ADC_V2_EVENT_TRIG_COMPLETE) != 0)
    {
        hpm_adc_v2_clear_status_flags(adc_u, HPM_ADC_V2_EVENT_TRIG_COMPLETE);

        if (g_encoder_isr_enable)
        {
            uint32_t t_enc = (uint32_t)hpm_csr_get_core_cycle();
            hpm_mcl_encoder_process(&motor0.encoder, motor0.cfg.mcl.physical.time.mcu_clock_tick / PWM_FREQUENCY);
            t_enc = (uint32_t)hpm_csr_get_core_cycle() - t_enc;
            if (t_enc > g_enc_cycles_max) {
                g_enc_cycles_max = t_enc;
            }
            if (t_enc > s_win_enc_max) {
                s_win_enc_max = t_enc;
            }
        }
        hpm_mcl_analog_get_value(&motor0.analog, analog_a_current, &g_ia);
        hpm_mcl_analog_get_value(&motor0.analog, analog_b_current, &g_ib);    /* 实际采样为 C 相 */
        g_ic = -g_ia - g_ib;
        speed_rad_s = motor0.encoder.result.speed;                            /* 机械角速度 rad/s */
        g_spd_fdb   = speed_rad_s;
        rpm = motor0.encoder.result.speed * 60.0f / (2.0f * MCL_PI);          /* 转/分钟 */


        if (SVPWM_MODE) 
        {
            svpwm_openloop_step();
        }

        if (FOC_MODE)
        {
            /* 应急让路：main 已连续停滞时，每 4 拍跳过 1 拍完整控制计算，
               强行给主循环留出执行时间，打破"没人救 → 一直饿死"的死锁 */
            bool run_loop = true;
        #if ISR_RELIEF_ENABLE
            if (s_relief != 0U) {
                if ((g_heart_isr & (ISR_RELIEF_DIV - 1U)) == 0U) {
                    run_loop = false;
                    g_isr_relief_cnt++;
                }
            }
        #endif
            if (run_loop)
            {
                uint32_t t_loop = (uint32_t)hpm_csr_get_core_cycle();
        #if FOC_POSITION_MODE
                    position_loop_pre();     /* 位置环：设定本拍动态限幅 */  /* 前馈给定S型曲线的参考输入 */
        #endif
        #if FOC_SPEED_MODE && SCURVE_SPD_ENABLE
                    speed_ramp_pre();        /* 速度 S 曲线：下发本拍速度给定（4kHz） */
        #endif
                    hpm_mcl_loop(&motor0.loop);  /* 电流环 / 速度环 / 位置环 */
        #if FOC_POSITION_MODE
                    position_loop_post();    /* 位置环：抗饱和回退 + 观测刷新 */
        #endif
                t_loop = (uint32_t)hpm_csr_get_core_cycle() - t_loop;
                if (t_loop > g_loop_cycles_max) {
                    g_loop_cycles_max = t_loop;
                }
                if (t_loop > s_win_loop_max) {
                    s_win_loop_max = t_loop;
                }
            }
        }
    }

    isr_timed_end(t0);
}
