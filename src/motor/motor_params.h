/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file motor_params.h
 * @brief 电机物理参数与控制环 PI 参数
 *
 * 所有可调参数集中在此，改增益不用翻业务代码。
 */
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include "hw_map.h"

/**
 * @brief 母线电压兜底值 V（read_vbus() 连续读失败时用它）
 *
 * 冷上电时 ADC 首帧可能还没落定，read_vbus() 会返回 -1；而 const_vbus 是
 * dq 解耦与电压限幅的输入，喂个负值进去比喂个标称值危险得多。
 * 填你的实际电源值（本工程 24V 供电）。
 */
#define MOTOR_VBUS_DEFAULT  (24.0f)

/** 闭环结构选择：决定 MCL 里 enable_speed_loop / enable_position_loop 的组合
 *
 * 放在本文件而不是 motor.h，是为了让 app_cfg.h 能引用它而不会让
 * sensor/control 层间接依赖 motor 层的结构体定义。 */
typedef enum {
    MOTOR_LOOP_CURRENT = 0,   /**< 只跑电流环（exec_ref.speed 恒 0） */
    MOTOR_LOOP_SPEED,         /**< 电流环 + 速度环 */
    MOTOR_LOOP_POSITION       /**< 电流环 + 速度环 + 位置环（位置环输出即速度给定） */
} motor_loop_mode_t;

/* ============================================================================
 * 重要：下面这一组 BOARD_BLDC_* 宏在 boards/hpm5300evk/board.h 里已经定义过，
 * 而 hw_map.h 会先 #include "board.h"，所以本文件被展开时它们全都是"已定义"状态。
 * 早先这里写的是 #ifndef 守卫，结果整块被跳过、board.h 里的旧值反而生效 ——
 * 期间所有"改了参数却没变化"的困惑都源于此（速度环 0.02/0.00008 从未真正生效，
 * 实际一直跑的是 board.h 的 0.0074/0.0001；位置环也一直是 154.7/0.113）。
 *
 * 本工程的参数必须以本文件为准，所以改成先 #undef 再 #define 强制覆盖。
 * 如果你确实想让 board.h 的默认值生效，把下面的 #undef 段注释掉即可。
 * ========================================================================== */
#undef BOARD_BLDC_HW_FOC_SPEED_KP
#undef BOARD_BLDC_HW_FOC_SPEED_KI
#undef BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP
#undef BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI
#undef BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KP
#undef BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KI
#undef BOARD_BLDC_HW_FOC_POSITION_KP
#undef BOARD_BLDC_HW_FOC_POSITION_KI
#undef BOARD_BLDC_SW_FOC_POSITION_KP
#undef BOARD_BLDC_SW_FOC_POSITION_KI

#define BOARD_BLDC_HW_FOC_SPEED_KP (0.015f)
#define BOARD_BLDC_HW_FOC_SPEED_KI (0.0005f)
/* 速度环 PI（2026-09-11 重算）
 * 依据：Kt = 1.5 * pole_num * flux = 1.5*4*0.009 = 0.054 N·m/A，J = 6.2e-6 kg·m^2
 *       => Kt/J = 8710 (rad/s)/A ；速度环 4kHz，Ts = 250us
 * 无延迟理论整定（ωn=94.2rad/s=15Hz，ζ=1）：
 *   kp      = 2ζωn / (Kt/J) = 2*94.2/8710 = 0.0216 -> 取 0.02
 *   ki_cont = ωn^2 / (Kt/J) = 94.2^2/8710 = 1.019
 *   ki_disc = ki_cont * Ts  = 1.019*2.5e-4 = 2.55e-4
 *   （MCL 的 PI 是 integral += ki*err，ki 已含 Ts，不要再除周期）
 *
 * 但速度反馈要过 encoder_iir（2 段二阶、fpass=100Hz @ 20kHz），等效时延约 5ms，
 * 在 15Hz 带宽上就是 ~28° 的相位滞后。按上面 ζ=1 直接整定，阶跃响应实测会超调 50%，
 * 起转瞬间表现为"猛冲一下"。所以 ki 从 2.55e-4 退到 8e-5，kp 保留 0.02。
 * 仿真（J=6.2e-6, 库仑摩擦 0.004N·m, 反馈时延 5ms, 给定斜率 500rad/s^2）：
 *   起转 11ms，峰值 52.8rad/s（超调 5.6%）。
 * 旧值 kp=0.0074/ki=0.0001 对应 ωn≈59rad/s(9.4Hz)、ζ≈0.55 —— 太软，
 * 起步阶段几乎不出转矩，是"顿一下再起转"的主因。
 * 若实测带载惯量远大于 6.2e-6，kp/ki 按 J 同比例放大。 */
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP (0.02f)
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI (0.00008f)

/* 位置环 PI
 *
 * ============ 一个必须知道的前提：位置环和速度环的 PID 实现不一样 ============
 * MCL 里两个函数，语义完全不同，混用整定公式会得到差几十倍的错误参数：
 *
 *   速度环 hpm_mcl_control_pi()：
 *       integral += ki * err;              // ki 在积分里
 *       out      =  kp * err + integral;
 *
 *   位置环 hpm_mcl_position_pid()：
 *       integral += err;                   // 积分累加的是"误差本身"，不含 ki
 *       out      =  kp * err + ki * integral;
 *
 * 对位置环做连续化：integral ≈ (1/Ts)·∫err dt，所以等效积分增益 Ki_eff = ki/Ts。
 * 位置环实际周期是 speed_loop_ts = 250us（4kHz，见 hpm_mcl_loop_init 的坑），Ts=2.5e-4。
 *
 * 整定：位置环 →(速度给定)→ 速度环闭环(≈一阶滞后 τ=62ms) →(积分)→ 位置
 *       特征方程  τs³ + s² + kp·s + Ki_eff
 *   取 ωn=10rad/s(1.6Hz)、ζ=1：kp = 2ζωn = 20，Ki_eff = ωn² = 100
 *       => ki = Ki_eff · Ts = 100 × 2.5e-4 = 0.025
 *   劳斯判据稳定条件 kp > τ·Ki_eff = 0.062×100 = 6.2，kp=20 有 3 倍裕度。
 *
 * integral_max 的单位是"rad·拍"，不是输出量：
 *   积分项对输出的贡献 = ki × integral，要能补出克服静摩擦所需的速度给定（约 0.5rad/s）
 *   => integral 至少 0.5/0.025 = 20；取 output_max/ki = 30/0.025 = 1200 留足余量。
 *
 * 历史坑（2026-09-14 曾按速度环语义整定，得到 kp=6/ki=0.0015，是错的；
 * 更早的 SDK 原值 kp=154.7/ki=0.113/integral_max=10 更是完全不匹配弧度反馈）：
 *   kp=154.7 → 误差 0.25rad 就把速度给顶到 output_max，等效 bang-bang，起步"顿一下"；
 *   integral_max=10 → 积分项最多贡献 0.113×10=1.13rad/s，远不够克服摩擦。 */
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KP (0.05f)
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KI (0.001f)
#define BOARD_BLDC_HW_FOC_POSITION_KP (34.7f)
#define BOARD_BLDC_HW_FOC_POSITION_KI (0.113f)
#define BOARD_BLDC_SW_FOC_POSITION_KP (20.0f)
#define BOARD_BLDC_SW_FOC_POSITION_KI (0.025f)

/* 位置环输出限幅：输出是"速度给定"，上限取 30rad/s（约 286rpm），
   不要再大到 50 —— 起步即饱和到 50rad/s，速度环积分会一路顶到 i_max=3A，
   触发过流检测后 disable_output，表现就是"顿一下然后不转"。*/
#define BOARD_BLDC_SW_FOC_POSITION_OUTPUT_MAX   (30.0f)

/* 位置环积分限幅（单位 rad·拍，不是输出量纲，积分项贡献 = ki × integral）
 * 这里只取 output_max/ki 的 1/4，即积分单独最多贡献 7.5rad/s：
 *   - 取满 1200 时积分项 = 0.025×1200 = 30 = output_max，
 *     意味着积分自己就能把输出顶满，长行程下退饱和要 0.3s，必然大幅超调。
 *   - 积分的作用只是在末端补出克服静摩擦的速度给定（约 0.5rad/s，即 integral≈20），
 *     300 已经留了 15 倍余量。 */
#define BOARD_BLDC_SW_FOC_POSITION_INTEGRAL_MAX (300.0f)

/* ---- 接近限速（2026-09-14）：消除"正转→反转→正转"----
 * 位置环输出限幅不能是固定值。39.27rad(6.25 圈)的行程里，固定 30rad/s 会让电机
 * 一路全速冲到目标附近才开始减速，而速度反馈要过 encoder_iir（等效滞后 ~5ms），
 * 减速指令晚了 5ms 就等于多走 v×5ms 的距离 —— 到 30rad/s 时约 0.15rad，
 * 冲过目标后 err 反号，积分反向，电机反转，再被拉回。
 * 对策：每拍把输出限幅改成 sqrt(2*a*|err|)，即"以减速度 a 从当前位置刚好停到目标"
 * 若嫌到位太慢，可提到 300~500。 */
#define POS_DECEL_MAX          (150.0f)

/* ---- 到位死区 rad（机械角，0.01rad = 0.57°）----
 * 限速用的是 sqrt(2*a*(|err| - 死区))，"死区"体现为对误差的抵扣而不是硬开关，
 * 所以 |err| 跨过死区边界时限幅是从 0 连续长起来的，不会突然跳到 1.7rad/s。
 *
 * 死区内还要把积分泄放到 0（下面的 POS_INTEGRAL_LEAK）。
 * 这一步不能省，也不能只"冻结"：减速段结束时 integral 已攒到 ~130（约 3.3rad/s），
 * 而此时 P 项只有零点几 —— 一旦 err 因扰动越过死区，积分会主导输出把电机推走，
 * 于是"停稳几秒后又转一下"、来回小幅摆动，仿真里表现为 39.23~39.30 的持续极限环。 */
#define POS_ARRIVE_DEADBAND    (0.01f)

/* 到位后位置环积分的泄放速率（单位 rad·拍，位置环 4kHz）
 * 2.0 意味着 130 的积分约 66 拍 = 16ms 泄完，足够快且不会造成冲击。 */
#define POS_INTEGRAL_LEAK      (2.0f)

#endif /* MOTOR_PARAMS_H */
