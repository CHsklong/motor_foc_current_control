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

#endif /* TRAJ_S_CURVE_H */
