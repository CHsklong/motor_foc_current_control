/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file planner_scurve.c
 * @brief 位置环 S 曲线轨迹规划实现
 *
 * 数学基础（对应文章推导）：段内 jerk 恒定，故该段是时间的三次多项式
 *   a(τ) = a0 + j·τ
 *   w(τ) = w0 + a0·τ + ½·j·τ²
 *   θ(τ) = θ0 + w0·τ + ½·a0·τ² + (1/6)·j·τ³
 * j = 0 时自动退化为匀加速公式，所以七段可以共用这一个函数。
 *
 * 对称梯形加速度的加速段有两个不变量（用它们可以避免冗长的分段积分）：
 *   v_peak = a_peak·(ta - tj)          加速段末速度
 *   d_acc  = v_peak·ta / 2            加速段位移（平均速度 v/2 × 时间 ta）
 * 代入文章的特例 tj=A、ta=3A 即得 v=2JA²、d_acc=3JA³，与文章结果一致。
 */
#include "planner_scurve.h"
#include <math.h>

/** 段内恒定 jerk 推进（注意顺序：先位移，再速度，最后加速度） */
static void seg_advance(float j, float tau, float *a, float *w, float *th)
{
    float t2 = tau * tau;

    *th += (*w) * tau + 0.5f * (*a) * t2 + (1.0f / 6.0f) * j * t2 * tau;
    *w  += (*a) * tau + 0.5f * j * t2;
    *a  += j * tau;
}

void scurve_init(scurve_t *s, float ts)
{
    uint32_t i;

    s->ts = (ts > 0.0f) ? ts : 1.0e-3f;
    s->tj = s->ta = s->tv = s->t_total = 0.0f;
    s->j = s->a_peak = s->v_peak = 0.0f;
    s->th0 = s->delta = 0.0f;
    s->sign = 1.0f;
    s->tick = 0U;
    s->state = SCURVE_IDLE;
    s->th = s->w = 0.0f;

    for (i = 0U; i < 8U; i++) {
        s->tk[i] = s->ak[i] = s->wk[i] = s->thk[i] = 0.0f;
    }
    for (i = 0U; i < 7U; i++) {
        s->jk[i] = 0.0f;
    }
}

void scurve_start(scurve_t *s, float th0, float delta, float vmax, float amax, float jmax)
{
    float d     = (delta >= 0.0f) ? delta : -delta;   /* 位移绝对值 */
    float tj, ta, tv, v, a, j;
    float tj_a, tj_v, d2, p;
    uint32_t i;

    s->th0   = th0;
    s->delta = delta;
    s->sign  = (delta >= 0.0f) ? 1.0f : -1.0f;
    s->tick  = 0U;
    s->th    = th0;
    s->w     = 0.0f;

    /* 位移太小没必要规划，直接算到位 */
    if ((d < 1.0e-5f) || (vmax <= 0.0f) || (amax <= 0.0f) || (jmax <= 0.0f)) {
        s->state = SCURVE_DONE;
        s->th    = th0 + delta;
        return;
    }

    j = jmax;

    /* ---- 1) 先按"能达到 amax 且有匀速段"试算 ---- */
    tj_a = amax / jmax;                 /* 加速度升到 amax 所需时间 */
    tj_v = sqrtf(vmax / jmax);          /* 三角形加速度达到 vmax 所需时间 */

    if (tj_a <= tj_v) {
        /* 梯形加速度：能升到 amax */
        tj = tj_a;
        a  = jmax * tj;                 /* = amax */
        ta = tj + vmax / a;
        d2 = vmax * ta;                 /* 两段加速位移之和 = 2*d_acc */
        if (d2 <= d) {
            v  = vmax;                  /* 七段：有匀速 */
            tv = (d - d2) / v;
        } else {
            /* 到不了 vmax：解 v² + v·(a·tj) − d·a = 0 */
            tv = 0.0f;
            p  = a * tj;                /* = amax²/jmax，梯形/三角形的分界速度 */
            v  = 0.5f * (-p + sqrtf(p * p + 4.0f * d * a));
            if (v >= p) {
                ta = tj + v / a;        /* 五段：有匀加速，无匀速 */
            } else {
                /* 连匀加速段都没有：退化成三角形加速度（四段） */
                tj = powf(d / (2.0f * jmax), 1.0f / 3.0f);
                a  = jmax * tj;
                v  = jmax * tj * tj;
                ta = 2.0f * tj;
            }
        }
    } else {
        /* 三角形加速度：vmax 太小，还没到 amax 就得开始降 */
        tj = tj_v;
        a  = jmax * tj;
        ta = 2.0f * tj;
        d2 = vmax * ta;
        if (d2 <= d) {
            v  = vmax;                  /* 七段（三角形加速 + 匀速） */
            tv = (d - d2) / v;
        } else {
            /* 四段：全程三角形 */
            tj = powf(d / (2.0f * jmax), 1.0f / 3.0f);
            a  = jmax * tj;
            v  = jmax * tj * tj;
            ta = 2.0f * tj;
            tv = 0.0f;
        }
    }

    /* 数值保护：ta 至少要容下两个 tj，否则段边界会出现时间倒流 */
    if (ta < 2.0f * tj) {
        ta = 2.0f * tj;
        if (tv > 0.0f) {
            /* 加速段被拉长后位移变了，重新算匀速时间，允许被截到 0 */
            d2 = v * ta;
            tv = (d > d2) ? ((d - d2) / v) : 0.0f;
        }
    }

    s->tj = tj; s->ta = ta; s->tv = tv;
    s->j  = j;  s->a_peak = a; s->v_peak = v;
    s->t_total = 2.0f * ta + tv;

    /* ---- 2) 构建 7 个段：时刻边界 + 段内 jerk ---- */
    s->tk[0] = 0.0f;
    s->jk[0] =  j;  s->tk[1] = tj;
    s->jk[1] =  0.0f; s->tk[2] = ta - tj;
    s->jk[2] = -j;  s->tk[3] = ta;
    s->jk[3] =  0.0f; s->tk[4] = ta + tv;
    s->jk[4] = -j;  s->tk[5] = ta + tv + tj;
    s->jk[5] =  0.0f; s->tk[6] = ta + tv + ta - tj;
    s->jk[6] =  j;  s->tk[7] = 2.0f * ta + tv;

    /* ---- 3) 递推各段起点的状态 ---- */
    s->ak[0] = 0.0f; s->wk[0] = 0.0f; s->thk[0] = 0.0f;
    for (i = 0U; i < 7U; i++) {
        s->ak[i + 1]  = s->ak[i];
        s->wk[i + 1]  = s->wk[i];
        s->thk[i + 1] = s->thk[i];
        seg_advance(s->jk[i], s->tk[i + 1] - s->tk[i],
                    &s->ak[i + 1], &s->wk[i + 1], &s->thk[i + 1]);
    }

    /* 强制收尾：递推会有浮点残差，终点必须精确等于目标位移 */
    s->thk[7] = d;
    s->wk[7]  = 0.0f;
    s->ak[7]  = 0.0f;

    s->state = SCURVE_RUN;
}

void scurve_step(scurve_t *s)
{
    float t, tau, a, w, th;
    uint32_t i;

    if (s->state == SCURVE_IDLE) {
        return;
    }
    if (s->state == SCURVE_DONE) {
        s->th = s->th0 + s->delta;
        s->w  = 0.0f;
        return;
    }

    /* 整数 tick 换算时间，避免浮点累加漂移 */
    t = (float)s->tick * s->ts;

    if (t >= s->t_total) {
        s->state = SCURVE_DONE;
        s->th    = s->th0 + s->delta;
        s->w     = 0.0f;
        return;
    }

    /* 定位当前段（段数少，线性查找足够；段边界重合时取后一段） */
    i = 0U;
    while ((i < 6U) && (t >= s->tk[i + 1])) {
        i++;
    }

    tau = t - s->tk[i];
    a   = s->ak[i];
    w   = s->wk[i];
    th  = s->thk[i];
    seg_advance(s->jk[i], tau, &a, &w, &th);

    s->w  = w * s->sign;
    s->th = s->th0 + th * s->sign;

    s->tick++;
}

void scurve_abort(scurve_t *s)
{
    s->state = SCURVE_DONE;
    s->th    = s->th0 + s->delta;
    s->w     = 0.0f;
}
