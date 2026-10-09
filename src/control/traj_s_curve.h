/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file traj_s_curve.h
 * @brief S 型加减速（jerk 受限）轨迹规划器
 *
 * 【为什么不用状态机】
 * 常见的 S 曲线写法是"每拍判断当前处于哪一段、积分推进到哪一段"，
 * 一个 7 段式规划器就是 7 个状态 + 一堆切换条件。这类实现有个致命弱点：
 * 只要有一个切换条件没覆盖到（比如匀加速段长度为 0、或者位移小于最小距离），
 * 状态机就会卡在某个中间态永远出不来 —— 表现为"给定发了但电机不动"。
 *
 * 这里换一种做法：**下发命令时一次性把整条曲线解析出来，运行时只做闭式求值**。
 *   - 规划阶段（scurve_plan，每次运动只跑一次）：用闭式公式算出各段时长
 *     tj / ta / tv / t_all 和峰值速度 vp、峰值加速度 ap；
 *   - 运行阶段（每拍）：位置只是"已过时间 t"的纯函数，
 *     按 t 落在哪一段直接代多项式，没有内部状态、没有累加、不会漂移、卡不住。
 *   - 所有 sqrt / cbrt 都在规划阶段算完，运行阶段只有几次乘加和比较。
 *
 * 【四种形状自适应】给定位移 D、速度上限 vmax、加速度上限 amax、加加速度上限 jmax：
 *   ① 七段：D 足够大，vmax 和 amax 都能达到；
 *   ② 五段(限速)：能达到 amax，但达不到 vmax（短行程，速度呈三角形）；
 *   ③ 四段：amax / vmax 都达不到（极短行程，加速度呈三角形）；
 *   ④ 五段(限加速)：能达到 vmax，但达不到 amax（vmax 很小时）。
 * 规划器自动挑，调用方不用管。
 *
 * 【约束】起止速度固定为 0（点对点定位）。运行中不接受重新规划（会拒绝并计数），
 * 因为带非零初速度的规划会引入 v0≠0 的另一套分支，得不偿失。
 */
#ifndef TRAJ_S_CURVE_H
#define TRAJ_S_CURVE_H

#include "hw_map.h"

/** 轨迹形状（规划结果，供 J-Scope 观察规划器到底走了哪套公式） */
#define SCURVE_SHAPE_7   (1U)   /**< 七段：vmax、amax 均达到 */
#define SCURVE_SHAPE_5V  (2U)   /**< 五段：达到 amax，未达到 vmax */
#define SCURVE_SHAPE_4   (3U)   /**< 四段：三角加速度，amax、vmax 均未达到 */
#define SCURVE_SHAPE_5A  (4U)   /**< 五段：达到 vmax，未达到 amax */

/** 规划器状态 */
typedef enum {
    SCURVE_IDLE = 0,    /**< 空闲：保持在当前位置 */
    SCURVE_RUN  = 1,    /**< 运行中：每拍刷新 p/v/a */
    SCURVE_DONE = 2     /**< 已到位：保持终点 */
} scurve_state_t;

/** 规划器实例 */
typedef struct {
    /* ---- 约束（scurve_init 写入，之后只读） ---- */
    float    vmax;      /**< 速度上限 rad/s（机械角） */
    float    amax;      /**< 加速度上限 rad/s^2 */
    float    jmax;      /**< 加加速度上限 rad/s^3 */
    float    ts;        /**< 规划器调用周期 s（= 1/PWM_FREQUENCY） */

    /* ---- 本次运动的规划结果（plan 时一次算完） ---- */
    float    p0;        /**< 起点 rad */
    float    delta;     /**< 位移 rad（含符号） */
    float    dir;       /**< 方向 +1 / -1 */
    float    tj;        /**< 加加速段（jerk 段）时长 s */
    float    ta;        /**< 加速段总时长 s（= 2*tj + 匀加速段） */
    float    tv;        /**< 匀速段时长 s（短行程时为 0） */
    float    td;        /**< 减速段总时长 s（与 ta 对称） */
    float    t_all;     /**< 总时长 s = ta + tv + td */
    float    vp;        /**< 实际峰值速度 rad/s（<= vmax） */
    float    ap;        /**< 实际峰值加速度 rad/s^2（<= amax） */
    float    d_acc;     /**< 加速段位移 rad = vp*ta/2 */
    uint8_t  shape;     /**< 形状，见 SCURVE_SHAPE_* */
    uint8_t  state;     /**< 状态，见 scurve_state_t */
    uint32_t t0;        /**< 本次运动起始节拍 */

    /* ---- 运行量 ---- */
    uint32_t tick;      /**< 节拍计数（每拍 +1，规划器的时间基准） */
    float    p;         /**< 本拍位置输出 rad */
    float    v;         /**< 本拍速度前馈 rad/s */
    float    a;         /**< 本拍加速度前馈 rad/s^2 */

    /* ---- 统计 ---- */
    uint32_t move_cnt;  /**< 成功启动的运动次数 */
    uint32_t busy_cnt;  /**< 因上一次还没走完而被拒绝的次数 */
} scurve_t;

/** 全局规划器实例（单电机） */
extern scurve_t g_sc;

/**
 * @brief 初始化规划器约束
 * @param ts   规划器节拍周期 s（填 1/PWM_FREQUENCY）
 * @param vmax 速度上限 rad/s
 * @param amax 加速度上限 rad/s^2
 * @param jmax 加加速度上限 rad/s^3
 */
void scurve_init(float ts, float vmax, float amax, float jmax);

/**
 * @brief 置位到指定位置（空闲态）
 *
 * 上电 / encoder_abs_rebase() 之后调用，把规划器输出对齐到当前机械位置。
 * 只改输出，不产生运动。
 */
void scurve_reset(float p_now);

/**
 * @brief 启动一次相对位移运动
 * @param delta 位移 rad（正/负决定方向）
 * @retval  0 已启动
 * @retval -1 上一次运动还没走完（busy_cnt++），本次被忽略
 * @retval -2 参数非法（NaN / Inf）
 */
int scurve_move_delta(float delta);

/**
 * @brief 启动一次绝对定位（相对当前规划位置的差值）
 */
int scurve_move_abs(float p_abs);

/**
 * @brief 就地保持：立刻停在当前规划位置（取消后续行程）
 */
void scurve_hold(void);

/**
 * @brief 20kHz 中断里每拍调用一次：节拍 +1 并刷新 p/v/a
 *
 * 必须在 hpm_mcl_loop() **之前**调用（位置环用它的输出做给定）。
 * 放在中断最前面、不受"应急让路"跳拍影响 —— 时间基准必须连续。
 */
void scurve_isr_tick(void);

/**
 * @brief 查询规划器是否空闲（IDLE 或 DONE）
 * @retval 1 空闲，可以下发新运动；0 正在运动
 */
uint8_t scurve_is_idle(void);

/** @brief 已过时间 s（供 J-Scope 观察） */
float scurve_elapsed(void);

/* ============================================================================
 * 速度域 S 曲线（jerk 受限的【速度给定】斜坡）
 * ============================================================================
 * 与上面位置规划器的分工：
 *   - 位置规划器输出"位置 p" → 喂位置外环（点对点定位，起止速度为 0）；
 *   - 这里输出"速度 v"      → 直接喂速度环，做的是【目标速度变化时的平滑过渡】。
 * 即：上位机/主循环给出目标速度后不再阶跃，而是按加速度/加加速度限制
 * 走一条 S 形斜坡过去，避免阶跃冲击电流环、也方便观察速度环跟踪能力。
 *
 * 结构（标准 jerk 受限三段，加速度呈梯形或三角形）：
 *   ① [0, tj)          jerk = +jmax，加速度 0 → ap
 *   ② [tj, tj+tc)      jerk = 0，    加速度恒为 ap（斜坡太短时 tc = 0）
 *   ③ [tj+tc, ta)      jerk = -jmax，加速度 ap → 0
 *
 * 与位置规划器同一套思想：规划期闭式解、运行期纯函数求值，
 * 无状态机、无累加、不会漂移也不会卡死。
 *
 * 判据：能从 amax 走满再收回所需的最小速度变化量 = amax^2/jmax
 *   Δv >= amax^2/jmax → 加速度梯形（ap = amax，中间有匀加速段 tc）
 *   Δv <  amax^2/jmax → 加速度三角形（ap = sqrt(Δv*jmax)，tc = 0）
 *
 * 【运行中改目标】允许：新目标以"当前速度给定"为新起点重新规划，
 * 不打断、不跳变（速度命令本就是可随时改的，不需要 busy 拒绝）。
 */

/** 速度域规划器实例 */
typedef struct {
    /* ---- 约束（scurve_spd_init 写入，运行期可被 scurve_spd_set_accel 改写） ---- */
    float    amax_acc;  /**< 加速度上限 rad/s^2（|v| 增大的方向，CAN SET_ACCEL 在线改） */
    float    amax_dec;  /**< 减速度上限 rad/s^2（|v| 减小的方向，CAN SET_DECEL 在线改） */
    float    jmax;      /**< 加加速度上限 rad/s^3 */
    float    vmax;      /**< 目标速度限幅 rad/s（防止上位机发超限命令） */
    float    ts;        /**< 规划器节拍周期 s（= 1/PWM_FREQUENCY） */

    /* ---- 本次斜坡的规划结果（move 时一次算完） ---- */
    float    v0;        /**< 起点速度 rad/s */
    float    dv;        /**< 速度增量 rad/s（含符号） */
    float    dir;       /**< 方向 +1 / -1 */
    float    tj;        /**< jerk 段时长 s */
    float    tc;        /**< 匀加速段时长 s（未达到 amax 时为 0） */
    float    ta;        /**< 总时长 s = 2*tj + tc */
    float    ap;        /**< 实际峰值加速度 rad/s^2（<= 本段生效的 amax） */
    float    amax;      /**< 本段实际生效的加速度上限 rad/s^2（= amax_acc 或 amax_dec） */
    uint8_t  state;     /**< 状态，见 scurve_state_t（复用同一套枚举） */
    uint8_t  stage2;    /**< 1 = 本次 move 要跨零换向：第一段减到 0，第二段再反向升到 v_final */
    uint32_t t0;        /**< 本次斜坡起始节拍 */
    float    v_final;   /**< 最终目标速度 rad/s（跨零换向时保存第二段的目标） */

    /* ---- 运行量 ---- */
    uint32_t tick;      /**< 节拍计数（每拍 +1，规划器的时间基准） */
    float    v;         /**< 本拍速度给定 rad/s（喂速度环） */
    float    a;         /**< 本拍加速度 rad/s^2（观测 / 前馈用） */
    float    j;         /**< 本拍 jerk rad/s^3（观测） */

    /* ---- 统计 ---- */
    uint32_t move_cnt;  /**< 成功启动的斜坡次数 */
} scurve_spd_t;

/** 全局速度域规划器实例（单电机） */
extern scurve_spd_t g_scs;

/**
 * @brief 初始化速度域规划器约束
 *
 * 约束全部由参数传入：规划器不反向依赖 motor_params.h，
 * 与位置规划器（scurve_init）保持同样的分层。
 *
 * 加速度与减速度分开给：上位机 SET_ACCEL / SET_DECEL 是两条独立命令，
 * 对应"升速"和"降速"两个方向（见 scurve_spd_move 的取值规则）。
 *
 * @param ts       规划器节拍周期 s（填 1/PWM_FREQUENCY）
 * @param amax_acc 加速度上限 rad/s^2（填 SCURVE_SPD_AMAX_ACC）
 * @param amax_dec 减速度上限 rad/s^2（填 SCURVE_SPD_AMAX_DEC）
 * @param jmax     加加速度上限 rad/s^3（填 SCURVE_SPD_JMAX）
 * @param vmax     目标速度限幅 rad/s（填 SCURVE_SPD_VMAX）
 */
void scurve_spd_init(float ts, float amax_acc, float amax_dec, float jmax, float vmax);

/**
 * @brief 在线改写加/减速上限（CAN 0x221 SET_ACCEL / SET_DECEL 的落点）
 *
 * 只改约束，不改当前给定：正在跑的斜坡从下一拍起按新 amax 重新规划剩余行程
 * （起点取当前给定 v，终点仍是本次 move 的最终目标），所以不会跳变。
 * 非正 / 非有限值直接忽略（返回后约束保持原值），避免把规划器搞成 NaN。
 *
 * @param a_acc 加速度上限 rad/s^2（>0）
 * @param a_dec 减速度上限 rad/s^2（>0）
 */
void scurve_spd_set_accel(float a_acc, float a_dec);

/**
 * @brief 置位到指定速度（空闲态，不产生斜坡）
 *
 * 上电 / 停机后调用，把规划器输出对齐到当前速度给定。
 */
void scurve_spd_reset(float v_now);

/**
 * @brief 以 S 形斜坡过渡到目标速度
 *
 * 以"当前速度给定"为起点规划一条 jerk 受限斜坡；运行中可再次调用改目标，
 * 会以当前给定为新起点重新规划（不跳变）。
 * 目标速度按 SCURVE_SPD_VMAX 限幅。
 *
 * 【本段用哪个 amax】速度域的"加速/减速"按【速度幅值】判定，不是按 dv 的符号：
 *   |v_target| > |v_start|  → 幅值在变大 → 用 amax_acc（SET_ACCEL）
 *   |v_target| < |v_start|  → 幅值在变小 → 用 amax_dec（SET_DECEL，含 STOP 归零）
 *
 * 【跨零换向】v_start 与 v_target 反号时整段只有一个 amax 是没有定义的
 * （前一半在刹车、后一半在反向加速），故拆成两段依次执行：
 *   第一段：v_start → 0（幅值减小，用 amax_dec）
 *   第二段：0 → v_target（幅值增大，用 amax_acc）
 * 衔接在 20kHz 的 scurve_spd_isr_tick() 里完成，速度连续、无停顿。
 *
 * @param v_target 目标速度 rad/s（机械角）
 * @retval  0 已启动
 * @retval -2 参数非法（NaN，或约束非正）
 */
int scurve_spd_move(float v_target);

/**
 * @brief 就地保持：立刻停在当前速度给定（取消后续斜坡）
 */
void scurve_spd_hold(void);

/**
 * @brief 20kHz 中断里每拍调用一次：节拍 +1 并刷新 v/a/j
 *
 * 必须在 hpm_mcl_loop() **之前**调用（速度环用它的输出做给定）。
 * 放在中断最前面、不受"应急让路"跳拍影响 —— 时间基准必须连续。
 */
void scurve_spd_isr_tick(void);

/**
 * @brief 查询规划器是否空闲（IDLE 或 DONE）
 * @retval 1 空闲（已在目标速度）；0 正在斜坡中
 */
uint8_t scurve_spd_is_idle(void);

/** @brief 本拍速度给定 rad/s（喂速度环） */
float scurve_spd_out(void);

#endif /* TRAJ_S_CURVE_H */
