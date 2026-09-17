/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file planner_scurve.h
 * @brief 位置环 S 曲线轨迹规划（恒定 jerk 七段式）
 *
 * 原理见 docs/scurve_planner.md，对应 CSDN《PMSM FOC位置环S曲线控制算法(恒定急动度)》
 * 的"恒定急动度"推导：对加速度再微分得到 jerk J = da/dt，让加速度走梯形，
 * 速度自然成为 S 形，位置则是平滑的"S 曲线"。
 *
 * 文章给的是"总时间均分 9 段"的特例。这里做成了更通用的形式：
 * 给定 位移 / 最大速度 / 最大加速度 / 最大 jerk，自动求解七段时长，
 * 并在条件不足时退化为五段（无匀速）、四段（无匀加速，三角形加速度）。
 *
 * 关键设计：每拍用**闭式分段公式**求值，而不是逐拍积分 a/w/theta。
 * 文章代码里那种 `a += j*Ts; w += a*Ts; th += w*Ts` 的写法会累积浮点误差，
 * 多圈后终点偏差明显（文章"待解决"一节也提到了这个问题）。
 * 这里只在规划时算好 7 个段起点的状态，运行时用 t 直接闭式求值，
 * 且 t 由整数 tick 换算，零累积误差。
 */
#ifndef PLANNER_SCURVE_H
#define PLANNER_SCURVE_H

#include "hw_map.h"

/** 规划器状态 */
typedef enum {
    SCURVE_IDLE = 0,   /**< 未启动 */
    SCURVE_RUN,        /**< 轨迹执行中 */
    SCURVE_DONE        /**< 已到终点（th 恒等于 th0+delta，w/a 恒为 0） */
} scurve_state_t;

/**
 * @brief S 曲线规划器
 *
 * 段划分（以 delta > 0 为例，delta < 0 时整体取反）：
 *   段0 [0,      tj     )  j = +J   加速度爬升
 *   段1 [tj,     ta-tj  )  j =  0   匀加速
 *   段2 [ta-tj,  ta     )  j = -J   加速度回落
 *   段3 [ta,     ta+tv  )  j =  0   匀速（v = v_peak）
 *   段4 [ta+tv,  +tj    )  j = -J   减速：加速度反向爬升
 *   段5 [...,    -tj    )  j =  0   匀减速
 *   段6 [...,    2ta+tv )  j = +J   减速：加速度回到 0
 */
typedef struct {
    /* ---- 规划结果（时间参数） ---- */
    float tj;        /**< jerk 段时长 s */
    float ta;        /**< 加速段总时长 s（= 减速段时长，对称） */
    float tv;        /**< 匀速段时长 s */
    float t_total;   /**< 总时长 = 2*ta + tv */
    float j;         /**< jerk 幅值 rad/s^3（恒正） */
    float a_peak;    /**< 峰值加速度 rad/s^2 */
    float v_peak;    /**< 巡航速度 rad/s */

    /* ---- 起终点 ---- */
    float th0;       /**< 起点位置 rad */
    float delta;     /**< 位移（有符号）rad */
    float sign;      /**< +1 / -1 */

    /* ---- 7 个段的起点状态（thk[7] 为终点） ---- */
    float tk[8];     /**< 各段起点时刻 s */
    float ak[8];     /**< 各段起点加速度 rad/s^2 */
    float wk[8];     /**< 各段起点速度 rad/s */
    float thk[8];    /**< 各段起点位移（相对起点） */
    float jk[7];     /**< 各段的 jerk */

    /* ---- 运行态 ---- */
    uint32_t tick;    /**< 已推进拍数（配合 ts 换算当前时刻，避免浮点累加漂移） */
    float    ts;      /**< 规划器步长 s（= 位置环周期） */
    uint8_t  state;   /**< scurve_state_t */

    /* ---- 本拍输出 ---- */
    float th;        /**< 规划位置 rad（绝对，已含 th0 与 sign） */
    float w;         /**< 规划速度 rad/s（用于前馈） */
} scurve_t;

/**
 * @brief 初始化规划器（只设步长与状态，不做规划）
 * @param s  规划器对象
 * @param ts 步长 s，应等于位置环周期（本工程 250us）
 */
void scurve_init(scurve_t *s, float ts);

/**
 * @brief 规划并启动一段轨迹
 * @param s    规划器对象
 * @param th0  起点位置 rad（通常取当前实际位置）
 * @param delta 位移 rad（有符号）
 * @param vmax 最大速度 rad/s（>0）
 * @param amax 最大加速度 rad/s^2（>0）
 * @param jmax 最大 jerk rad/s^3（>0）
 *
 * 三个约束会按需退化：
 *   - 位移足够长  -> 七段（含匀速）
 *   - 到不了 vmax -> 五段（无匀速）
 *   - 到不了 amax -> 四段（三角形加速度，无匀加速）
 */
void scurve_start(scurve_t *s, float th0, float delta, float vmax, float amax, float jmax);

/**
 * @brief 推进一拍并重算本拍的 th / w / a
 * @note 调用频率必须等于 1/ts；非位置环执行拍不要调用
 */
void scurve_step(scurve_t *s);

/** 中止（保持当前位置为终点） */
void scurve_abort(scurve_t *s);

#endif /* PLANNER_SCURVE_H */
