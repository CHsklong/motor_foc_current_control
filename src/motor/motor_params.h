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

#define BOARD_BLDC_HW_FOC_SPEED_KP (0.035f)
#define BOARD_BLDC_HW_FOC_SPEED_KI (0.001f)
/* 速度环 PI */
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP (0.035f)
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI (0.00005f)

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
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KP (0.022f)     
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KI (0.00015f)     
#define BOARD_BLDC_HW_FOC_POSITION_KP (34.7f)
#define BOARD_BLDC_HW_FOC_POSITION_KI (0.113f)
#define BOARD_BLDC_SW_FOC_POSITION_KP (20.0f)
/* ki 2026-09-20 降：0.025(Ki_eff=100) 配 kp=10 时闭环阻尼比只有 0.39（特征方程
 * 0.024s^3+s^2+10s+100，τ=速度环 24ms），实测 1.8Hz 欠阻尼振荡、到位超调 1 格。
 * 降到 0.012（Ki_eff=48），配合 SCURVE_POS_KP=4，ζ 回到 0.5 以上。
 * 积分作用只剩到位末端补摩擦（integral≈42 即可出 0.5rad/s），够用。 */
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

/* 到位后位置环积分的泄放速率（单位 rad·拍，位置外环 4kHz）
 * 2.0 意味着 130 的积分约 66 拍 = 16ms 泄完，足够快且不会造成冲击。 */
#define POS_INTEGRAL_LEAK      (2.0f)

/* ============================================================================
 * S 曲线（jerk 受限）轨迹规划参数 —— 见 control/traj_s_curve.h
 * ==========================================================================*/
/** 速度上限 rad/s（机械角）。约 239rpm。
 *  参考：额定 3000rpm = 314rad/s，但位置环原来 output_max 只有 30rad/s，
 *  这里取略小一点，给修正器留出叠加余量。 */
#define SCURVE_VMAX          (25.0f)
/** 加速度上限 rad/s^2。
 *  能力核算：Kt/J = 8710 (rad/s)/A（Kt=1.5*pole_num*flux=0.054，J=6.2e-6），
 *  i_max=3A 对应 26130rad/s^2 —— 电机完全做得到，这里是为"平顺"和"不给电流环
 *  添堵"而主动限的。想更快就往上加，但别一次加到 1000 以上：
 *  加速段会短到只剩 jerk 段，S 曲线退化成梯形的锐角起步。 */
#define SCURVE_AMAX          (150.0f)

/** 加加速度上限 rad/s^3。jerk 段时长 tj = amax/jmax = 37.5ms。
 *  jmax 越小起步/停止越柔（电流冲击小、无异响），越大越干脆。
 *  这是"手感"旋钮：抖就调小，嫌慢就调大。 */
#define SCURVE_JMAX          (4000.0f)

/** 加速度前馈系数 s（补速度环的一阶滞后 τ）。
 *  速度环闭环近似一阶滞后，跟踪斜坡输入会滞后 a*τ 的速度，
 *  所以在速度前馈上再加 a_ff*τ 把它提前补掉。
 *  实测调：J-Scope 看加速段 g_spd_fdb 与 g_sc_v_ref 的差值，差多少就补多少。
 *  设 0 即关闭加速度前馈（只靠外环积分慢慢追，加速段会有位置滞后）。
 *
 *  2026-09-21 修正（过冲根因）：原 0.02 是"按设计带宽 ~15Hz、τ≈20ms"拍的，
 *  但实际速度环 SW 增益 kp=0.012/ki=5e-5 软得多（实测带宽仅 ~3~4Hz，τ≈60ms）。
 *  J-Scope 实测段末 spd_fdb(23.1) 比 g_ref_speed(13.6) 高约 9rad/s 的滞后，
 *  对应 a*τ = 150*0.06 = 9 —— 即实际 τ≈0.06s，原 0.02 只补了 1/3。
 *  结果：S 曲线减速段速度环跟不上指令，电机带着惯性冲过目标 ~3.23rad（段末过冲）。
 *  提到 0.06 让前馈大致吃满这段滞后，电机即可贴着规划曲线停住。
 *  注意：此量只在 g_sc.a≠0（加/减速段）生效，对阶跃/匀速命令零影响，
 *  不会引入位置环的阶跃超调（那是当初把速度环 kp 从 0.02 降到 0.012 的原因）。
 *  若仍过冲：继续提到 0.07~0.08；若反向欠调/打摆：回落到 0.04~0.05。 */
#define SCURVE_ACC_FF_TAU    (0.06f)

/** 位置外环修正权限 rad/s。
 *  S 曲线模式下外环只负责"修正"，主体速度由前馈给出。
 *  【实测教训】6 时 J-Scope 里 g_sc_corr 会在 ±6 之间打满（等效 bang-bang），
 *  速度给定 19↔31 来回摆 → 电机全程抖。权限越小，打摆的幅度越小。
 *  若还打摆：降 SCURVE_POS_KP，而不是无脑加大这里的值。 */
#define POS_CORR_MAX         (6.0f)

/** 位置外环 P 增益（覆盖 motor_init() 里的 BOARD_BLDC_SW_FOC_POSITION_KP）。
 *  20 是按"阶跃给定"整定的，前馈结构下 err 天生很小、用不到那么高的增益，
 *  高增益反而让修正量打满 → 速度给定打摆 → 电机抖。
 *  到位精度由积分保证（Ki_eff = ki/Ts = 100 不变），kp 只影响抗扰快慢：
 *  到位变慢/带载压不住误差就往 15、20 回调。 */
#define SCURVE_POS_KP        (11.0f)

/** 位置外环修正量低通截止频率 Hz（0=旁路）。
 *  外环交点在几 Hz 量级，30Hz 低通对外环几乎无相位损失，
 *  却能把修正里的高频毛刺挡在速度环外面。修正量仍在低频打摆时，
 *  说明问题不是毛刺而是增益/前馈失配，别指望这个滤波器。 */
#define SCURVE_CORR_FC       (30.0f)

/** 速度给定总限幅 rad/s = 规划速度 + 修正权限 */
#define SCURVE_V_CMD_MAX     (SCURVE_VMAX + POS_CORR_MAX)

/** 位置外环分频：20kHz / 5 = 4kHz。
 *  必须等于 speed_loop_ts / current_loop_ts，不能照 position_loop_ts(1ms) 取 20：
 *  MCL 里 position_ts 指针指向的是 speed_loop_ts（见 ctrl_foc.c 的说明），
 *  位置环 ki 也是按 4kHz 整定的，分频取错等效积分增益差 4 倍。 */
#define POS_LOOP_DIV         (5)

/* ============================================================================
 * 速度域 S 曲线（速度给定的 jerk 受限斜坡）—— scurve_spd_*，见 traj_s_curve.h
 * ==========================================================================*/
/** 直接复用位置域的两个约束，不另设一组：
 *  速度域里"加速度" = 速度的变化率 rad/s^2（对应 SCURVE_AMAX 的量纲），
 *           "jerk"  = 加速度的变化率 rad/s^3（对应 SCURVE_JMAX 的量纲），
 *  两者与位置域同量纲，且电机能力核算（i_max=3A ⇒ 26130rad/s^2）远大于此，够用。
 *  想让斜坡更柔：调小 SCURVE_SPD_JMAX（jerk 段变长）；想更快：调大它。 */

/** 速度域斜坡的 jerk 上限 rad/s^3（柔顺度旋钮，独立可调，不动位置域的 SCURVE_JMAX） */
#define SCURVE_SPD_JMAX      (4000.0f)

/** 速度域的加速度 / 减速度上限 rad/s^2 —— 【上电默认值】，
 *  运行期由 CAN 0x221 的 SET_ACCEL / SET_DECEL 在线改写（scurve_spd_set_accel），
 *  改写只影响后续斜坡与"剩余行程的重新规划"，不会让当前速度跳变。
 *
 *  取用规则（见 traj_s_curve.c spd_plan）：按【速度幅值】判定而不是按 dv 符号 ——
 *    |目标| > |当前| → ACC（升速，含 0 → -15 这种反向启动）
 *    |目标| < |当前| → DEC（降速，含 STOP 归零、减速到更低速）
 *    跨零换向（+15 → -15）拆两段：先按 DEC 减到 0，再按 ACC 反向升上去。
 *
 *  上电默认与位置域 SCURVE_AMAX 同值，保证不改动既有手感；
 *  上位机下发后以 RAM 里的值为准（断电恢复）。 */
#define SCURVE_SPD_AMAX_ACC  (SCURVE_AMAX)
#define SCURVE_SPD_AMAX_DEC  (SCURVE_AMAX)

/** 速度模式加速度前馈系数 s（默认 0 = 关闭，行为与原版完全一致）。
 *  机理：速度环节拍 4kHz、闭环近似一阶滞后，跟踪斜率 a 的斜坡会滞后约 a*tau_loop，
 *  段末这个滞后回补 + 12~18Hz 电流侧振荡 ⇒ 可见过冲。在给定上提前加 a*tau 把它补掉。
 *  与位置域的 SCURVE_ACC_FF_TAU(0.06s) 不是一个量级：那里补的是"整个位置外环"，
 *  这里只补速度闭环，tau_loop ≈ 1/wc，wc = kp*Kt/J = 0.035*(2300~4000) ⇒ 13~22Hz
 *  ⇒ tau_loop ≈ 7~12ms ⇒ **从 0.005 起步试**。
 *  调法：J-Scope 看加速段的 g_spd_fdb 与 g_scs_v 差值；段末若仍冲高就往上加
 *  （0.005→0.008→0.012），若反向欠调/段末打摆就减。
 *  安全边界：a≠0 时才生效，且段末 a→0（jerk 受限）⇒ 给定无阶跃；
 *  稳态匀速时该项恒 0，不影响稳态跟随与噪声。 */
#define SCURVE_SPD_ACC_FF_TAU (0.0065f)

/* ============================================================================
 * 摩擦前馈（速度环输出侧，直接补电流）—— 治"加速前期跟不上"的特效药
 * ==========================================================================*/
/** 转矩常数 N·m/A。Kt = 1.5 * pole_num * flux = 1.5 * 4 * 0.009 = 0.054。
 *  前馈转矩 → 电流的换算全靠它，改这个数等于改所有前馈增益。 */
#define MOTOR_KT_NM_PER_A    (0.054f)

/** 库仑（干）摩擦转矩 N·m —— 与速度大小无关、只与转向有关的那一份。
 *  2026-09-23 定案（v=3 与 v=15 两点，见 docs/friction_ff_reference.md）：
 *    需求转矩 need(3)=0.00756 N·m、need(15)=0.00679 N·m ⇒ 斜率 B≈-6.4e-5（≈0，落在噪声里）
 *    ⇒ **摩擦以库仑项为主**，取保守值 Tc=0.0065、B=0。
 *  判据：与斜坡缺口反推（Tc/(Kp·Kt)≈2.5~3 rad/s）对照，0.0065 落在下沿，宁欠补不过补。
 *  若前期缺口仍明显 → 提到 0.0070；若起步冲过头/零速来回蹭 → 降到 0.0055。 */
/* *  【Kt 争议被标定本身消掉】上面这个"测 iq → 反填 Tc"的闭环标定法里，
 *        ff 输出 = Tc/Kt = (iq_mean·Kt)/Kt = iq_mean，
 *  即最终前馈电流恒等于你实测到的那个 iq_mean，与 Kt 取 0.054 还是 0.019 无关。
 *  所以别纠结 Kt 的绝对值，照这个流程标定即可。 */
#define FRIC_TC_NM           (0.0065f)

/** 黏滞摩擦系数 N·m/(rad/s) —— 随速度线性增长的那一份。
 *  2026-09-23 两点定案：B_fit = (need(15)-need(3))/12 = (0.00679-0.00756)/12 = -6.4e-5，
 *  即"负黏滞"——低速摩擦反而略大（Stribeck 特征）。但这个量级远小于噪声
 *  （v=3 时电流 p-p 摆动 0.45A，远大于两点之差 0.014A），**不可信，按 0 处理**。
 *  真要在模型里体现"低速摩擦更大"，正路是加 Stribeck 项而非用负 B（负 B 高速外推会翻负）。 */
#define FRIC_B_NMS           (0.0f)

/** 平滑符号函数的"软化宽度" rad/s：sgn = v / (|v| + eps)。
 *  不能直接用 sign()：零速附近给定在 ±1 个噪声 LSB 之间跳变会让它每拍翻转，
 *  等于往电流里注入 ±Tc/Kt 的方波 → 抖。eps 越大越柔（零速附近前馈线性衰减到 0），
 *  太小则接近理想继电器。取 0.3~1.0 rad/s，别取 0。 */
#define FRIC_FF_V_EPS        (0.5f)

/** 目标速度限幅 rad/s：与位置模式共用同一个速度上限，防止上位机发超限命令 */
#define SCURVE_SPD_VMAX      (SCURVE_VMAX)

/** main() 里速度模式（FOC_SPEED_MODE=1）上电后要走到的目标速度 rad/s。
 *  SCURVE_SPD_ENABLE=1 时按 S 形斜坡过渡过去，不是阶跃。 */
#define SCURVE_SPD_TARGET    (0.0f)

/** 往复演示的到位停留时间（主循环次数，约 1ms 一次） */
#define SCURVE_DWELL_MS      (800)

/** S 曲线位置命令（CAN 0x102）的"续发"门槛。
 *  上一段走完（规划器 DONE）并不等于电机实际停稳：速度环一阶滞后让实际速度、
 *  位置误差还要一小段时间才收敛，而位置外环残余修正量按 ~5.31ms(=1/SCURVE_CORR_FC)
 *  的时间常数泄放。若在残余未泄放时就续发下一段，修正量会叠加到新段前馈上 → 过冲。
 *  故续发须同时满足：规划器空闲 + 位置误差 < 此值 + 实际速度 < 下值。
 *  这两个阈值就是"实际停稳"的工程判据，比单纯看 scurve_is_idle() 更可靠。 */
#define SCURVE_SETTLE_POS   (0.05f)   /* rad，≈2.86°：允许的最大位置误差 */
#define SCURVE_SETTLE_SPD   (1.0f)    /* rad/s，≈9.5rpm：允许的最大实际机械角速度
                                        （源：ctrl_foc.c 每拍刷新的 g_spd_fdb） */

/** 上面两个条件必须【连续】成立这么多主循环拍（约 1ms 一拍）才允许续发。
 *  ① 残余修正量的时间常数 5.31ms，要求"安静"持续约 2τ 才算泄放干净；
 *  ② 实际速度是差分估出来的，运动中的瞬时过零会短暂满足阈值，连续判据滤掉它。 */
#define SCURVE_SETTLE_HOLD_MS (10)

#endif /* MOTOR_PARAMS_H */
