/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file traj_s_curve.c
 * @brief S 型加减速（jerk 受限）轨迹规划器实现
 *
 * 详见 traj_s_curve.h 的头部说明：规划期闭式解，运行期纯函数求值，无状态机。
 */
#include "hw_map.h"
#include "traj_s_curve.h"

/** 小于这个位移就认为"已经在位"，直接置位不做规划 rad（约 0.006°） */
#define SCURVE_MIN_DIST   (1.0e-4f)

scurve_t g_sc;

/* ============================================================================
 * 加速段形状函数
 *
 * 加速段的结构固定为三段：
 *   ① [0, tj)          加加速：j = +jmax，a 从 0 线性升到 ap
 *   ② [tj, ta-tj)      匀加速：a = ap（三角加速度时这一段长度为 0）
 *   ③ [ta-tj, ta)      减加速：j = -jmax，a 从 ap 线性降回 0
 *
 * 其中 ap = jmax * tj（若受 amax 限制则 tj = amax/jmax，ap 恰好 = amax）。
 * 四种形状共用这一套公式，只靠 tj / ta / vp 的取值区分。
 *
 * @param s    规划器
 * @param t    加速段内的局部时间 [0, ta]
 * @param pp   输出：本段已走过的位移（>=0）
 * @param vv   输出：速度（>=0）
 * @param aa   输出：加速度（>=0）
 * ========================================================================== */
static void scurve_shape(const scurve_t *s, float t, float *pp, float *vv, float *aa)
{
    float jmax = s->jmax;
    float tj   = s->tj;
    float ta   = s->ta;

    if (t <= 0.0f)                                //段前
    {
        *pp = 0.0f; *vv = 0.0f; *aa = 0.0f;
        return;
    }
    if (t >= ta)                                  //段后
    {
        *pp = s->d_acc; *vv = s->vp; *aa = 0.0f;
        return;
    }

    if (t < tj) 
    {
        /* ① 加加速：a = jmax*t，v = 1/2*jmax*t^2，p = 1/6*jmax*t^3 */
        *aa = jmax * t;
        *vv = 0.5f * jmax * t * t;
        *pp = (jmax * t * t * t) / 6.0f;
    } else if (t < (ta - tj)) 
    {
        /* ② 匀加速：以 tj 时刻的 (p1, v1) 为初值做匀加速直线外推 */
        float x  = t - tj;
        float v1 = 0.5f * jmax * tj * tj;              /* = 0.5 * ap * tj */
        float p1 = (jmax * tj * tj * tj) / 6.0f;
        *aa = s->ap;
        *vv = v1 + s->ap * x;
        *pp = p1 + v1 * x + 0.5f * s->ap * x * x;
    } else 
    {
        /* ③ 减加速：利用加速段速度曲线的中心对称性 v(ta-t) = vp - v(t)，
              位移用"整段位移减去剩余部分"求，避免再递归一次形状函数。
              剩余部分 = vp*u - p_shape(u)，而此处 u <= tj，p_shape(u)=jmax*u^3/6。 */
        float u = ta - t;                              /* u: tj -> 0 */
        *aa = jmax * u;
        *vv = s->vp - 0.5f * jmax * u * u;
        *pp = s->d_acc - s->vp * u + (jmax * u * u * u) / 6.0f;
    }
}

/**
 * @brief 整条曲线在 t 时刻的取值（含方向）
 *
 * 减速段不另写公式：它是加速段的时间反演 ——
 * 用"剩余时间 u = t_all - t"去查加速段形状，再从终点倒扣位移即可。
 *   p(t) = p_end - dir * p_shape(u)
 *   v(t) = dir * v_shape(u)                      （方向与加速段相同，为正）
 *   a(t) = -dir * a_shape(u)                     （减速，符号相反）
 */
static void scurve_eval_at(const scurve_t *s, float t, float *pp, float *vv, float *aa)       //计算取值
{
    float ps, vs, as;

    if (t <= 0.0f) 
    {
        *pp = s->p0; *vv = 0.0f; *aa = 0.0f;
        return;
    }
    if (t >= s->t_all) 
    {
        *pp = s->p0 + s->delta; *vv = 0.0f; *aa = 0.0f;
        return;
    }

    if (t < s->ta) {                                      //加速段
        scurve_shape(s, t, &ps, &vs, &as);
        *pp = s->p0 + s->dir * ps;
        *vv = s->dir * vs;
        *aa = s->dir * as;
    } else if (t < (s->ta + s->tv)) {                     //匀速段
        *pp = s->p0 + s->dir * (s->d_acc + s->vp * (t - s->ta));
        *vv = s->dir * s->vp;
        *aa = 0.0f;
    } else {                                              //减速段
        float u = s->t_all - t;
        scurve_shape(s, u, &ps, &vs, &as);
        *pp = s->p0 + s->delta - s->dir * ps;
        *vv = s->dir * vs;
        *aa = -s->dir * as;
    }
}

/**
 * @brief 规划：根据位移和三个上限，解析出各段时长
 *
 * 判据推导（v0 = v1 = 0，D = |delta|）：
 *   - 达到 amax 至少需要速度 va = amax^2 / jmax；
 *   - 达到 vmax 至少需要位移
 *       vmax >= va 时：Dmin = vmax * (vmax/amax + amax/jmax)
 *       vmax <  va 时：Dmin = 2 * vmax * sqrt(vmax/jmax)
 *   - 达到 amax 至少需要位移 Da = 2 * amax^3 / jmax^2
 * 于是：
 *   D >= Dmin                 -> 形状①/④（视 vmax 与 va 的大小关系）
 *   D <  Dmin 且 D >= Da      -> 形状②
 *   D <  Dmin 且 D <  Da      -> 形状③
 * （vmax < va 时必有 Da > Dmin，所以形状②不会被误选，分支自洽。）
 */
static int scurve_plan(scurve_t *s, uint32_t tick, float p_start, float delta)
{
    float vmax = s->vmax;                             //约束
    float amax = s->amax;
    float jmax = s->jmax;
    float D    = fabsf(delta);
    float va   = (amax * amax) / jmax;              /* 达到 amax 所需的最小峰值速度 */
    float Da   = (2.0f * amax * amax * amax) / (jmax * jmax);  /* 达到 amax 所需的最小位移 */
    float Dmin;

    if (!(D >= 0.0f) || (D > 1.0e12f)) {             /* NaN / 负值 / 超大值 */
        return -2;
    }
    if (D < SCURVE_MIN_DIST) {                      /* 已经在位：直接置位 */
        s->p    = p_start + delta;
        s->v    = 0.0f;
        s->a    = 0.0f;
        s->p0   = s->p;
        s->delta = 0.0f;
        s->dir  = 1.0f;
        s->state = SCURVE_DONE;
        return 0;
    }
    if ((vmax <= 0.0f) || (amax <= 0.0f) || (jmax <= 0.0f)) {
        return -2;
    }

    s->p0    = p_start;
    s->delta = delta;
    s->dir   = (delta >= 0.0f) ? 1.0f : -1.0f;

    Dmin = (vmax >= va) ? (vmax * (vmax / amax + amax / jmax))
                        : (2.0f * vmax * sqrtf(vmax / jmax));

    if (D >= Dmin) {
        /* ---- vmax 能达到 ---- */
        if (vmax >= va) {
            s->tj    = amax / jmax;                 /* 受 amax 限制 */
            s->ta    = vmax / amax + s->tj;
            s->shape = SCURVE_SHAPE_7;
        } else {
            s->tj    = sqrtf(vmax / jmax);          /* 受 vmax 限制，amax 达不到 */
            s->ta    = 2.0f * s->tj;
            s->shape = SCURVE_SHAPE_5A;
        }
        s->vp = vmax;
        s->tv = D / vmax - s->ta;
        if (s->tv < 0.0f) {
            s->tv = 0.0f;                           /* 数值误差兜底 */
        }
    } else if (D >= Da) {
        /* ---- 形状②：达到 amax，达不到 vmax（速度三角形）----
         * 由 D = vp*ta、ta = vp/amax + tj 得  vp^2/amax + vp*tj - D = 0，取正根。 */
        s->tj    = amax / jmax;
        s->vp    = 0.5f * (sqrtf(amax * amax * s->tj * s->tj + 4.0f * amax * D) - amax * s->tj);
        s->ta    = s->vp / amax + s->tj;
        s->tv    = 0.0f;
        s->shape = SCURVE_SHAPE_5V;
    } else {
        /* ---- 形状③：加速度三角形，D = 2*jmax*tj^3 ---- */
        s->tj    = powf(D / (2.0f * jmax), 1.0f / 3.0f);
        s->ta    = 2.0f * s->tj;
        s->vp    = jmax * s->tj * s->tj;
        s->tv    = 0.0f;
        s->shape = SCURVE_SHAPE_4;
    }

    s->ap    = jmax * s->tj;
    s->td    = s->ta;
    s->d_acc = 0.5f * s->vp * s->ta;
    s->t_all = 2.0f * s->ta + s->tv;
    s->t0    = tick;
    s->state = SCURVE_RUN;

    return 0;
}

void scurve_init(float ts, float vmax, float amax, float jmax)
{
    scurve_t *s = &g_sc;

    s->ts   = ts;
    s->vmax = vmax;
    s->amax = amax;
    s->jmax = jmax;
    s->tick = 0U;
    s->move_cnt = 0U;
    s->busy_cnt = 0U;
    scurve_reset(0.0f);
}

void scurve_reset(float p_now)
{
    scurve_t *s = &g_sc;

    s->p0    = p_now;
    s->delta = 0.0f;
    s->dir   = 1.0f;
    s->tj    = 0.0f;
    s->ta    = 0.0f;
    s->tv    = 0.0f;
    s->td    = 0.0f;
    s->t_all = 0.0f;
    s->vp    = 0.0f;
    s->ap    = 0.0f;
    s->d_acc = 0.0f;
    s->shape = 0U;
    s->p     = p_now;
    s->v     = 0.0f;
    s->a     = 0.0f;
    s->state = SCURVE_IDLE;
}

int scurve_move_delta(float delta)
{
    scurve_t *s = &g_sc;
    int rc;

    if (s->state == SCURVE_RUN) {
        s->busy_cnt++;                  /* 上一次还没走完：拒绝，不打断 */
        return -1;
    }

    rc = scurve_plan(s, s->tick, s->p, delta);
    if (rc != 0) {
        return rc;
    }
    s->move_cnt++;
    scurve_eval_at(s, 0.0f, &s->p, &s->v, &s->a);
    return 0;
}

int scurve_move_abs(float p_abs)
{
    return scurve_move_delta(p_abs - g_sc.p);
}

void scurve_hold(void)
{
    scurve_t *s = &g_sc;

    if (s->state == SCURVE_RUN) {
        s->delta = s->p - s->p0;        /* 已走的部分记为本次行程 */
    }
    s->v     = 0.0f;
    s->a     = 0.0f;
    s->tv    = 0.0f;
    s->t_all = 0.0f;
    s->state = SCURVE_DONE;             /* 保持在当前规划位置 */
}

void scurve_isr_tick(void)                  //20khz每拍唯一的入口
{
    scurve_t *s = &g_sc;

    s->tick++;

    if (s->state == SCURVE_RUN) {
        float t = (float)(s->tick - s->t0) * s->ts;
        scurve_eval_at(s, t, &s->p, &s->v, &s->a);
        if (t >= s->t_all) {
            s->p    = s->p0 + s->delta; /* 收尾对齐，消除浮点残差 */
            s->v    = 0.0f;
            s->a    = 0.0f;
            s->state = SCURVE_DONE;
        }
    }
    /* IDLE / DONE：p/v/a 保持不变（= 保持位置） */
}

uint8_t scurve_is_idle(void)
{
    return (g_sc.state == SCURVE_RUN) ? 0U : 1U;
}

float scurve_elapsed(void)
{
    if (g_sc.state != SCURVE_RUN) {
        return 0.0f;
    }
    return (float)(g_sc.tick - g_sc.t0) * g_sc.ts;
}

/* ============================================================================
 * 速度域 S 曲线（jerk 受限速度斜坡）—— 实现
 * 说明与公式推导见 traj_s_curve.h 尾部
 * ========================================================================== */

/** 小于这个速度变化量就认为"已经到位"，直接置位不做规划 rad/s */
#define SCURVE_SPD_MIN_DV   (1.0e-4f)

scurve_spd_t g_scs;

/* 单段规划的内部实现（scurve_spd_move / set_accel / isr_tick 换向衔接都要用） */
static int spd_plan(float v_start, float v_target);

void scurve_spd_init(float ts, float amax_acc, float amax_dec, float jmax, float vmax)
{
    scurve_spd_t *s = &g_scs;

    s->ts       = ts;
    s->amax_acc = amax_acc;
    s->amax_dec = amax_dec;
    s->amax     = amax_acc;             /* 本段生效值：reset 后先按加速限值算 */
    s->jmax     = jmax;
    s->vmax     = vmax;
    s->tick     = 0U;
    s->move_cnt = 0U;
    s->stage2   = 0U;
    s->v_final  = 0.0f;
    scurve_spd_reset(0.0f);
}

void scurve_spd_reset(float v_now)
{
    scurve_spd_t *s = &g_scs;

    s->v0    = v_now;
    s->dv    = 0.0f;
    s->dir   = 1.0f;
    s->tj    = 0.0f;
    s->tc    = 0.0f;
    s->ta    = 0.0f;
    s->ap    = 0.0f;
    s->state = SCURVE_IDLE;
    s->stage2 = 0U;
    s->v_final = v_now;
    s->v     = v_now;
    s->a     = 0.0f;
    s->j     = 0.0f;
}

void scurve_spd_set_accel(float a_acc, float a_dec)
{
    scurve_spd_t *s = &g_scs;

    /* 非法值一律拒绝：非正会让 d_reach<=0 把段长算成 0/负，
       NaN 会穿透后面所有比较一路带到电流环。保持原约束是最省事的安全行为。 */
    if (!(a_acc > 0.0f) || !(a_acc < 1.0e30f)) { return; }
    if (!(a_dec > 0.0f) || !(a_dec < 1.0e30f)) { return; }

    s->amax_acc = a_acc;
    s->amax_dec = a_dec;

    /* 斜坡正在跑：以【当前给定】为新起点、按新限值重新规划剩下的行程。
       终点仍是本次 move 的最终目标（跨零换向时是第二段的 v_final），
       所以只是把剩余那段重新拉长/缩短，速度本身连续，不会跳变。 */
    if (s->state == SCURVE_RUN)
    {
        float v_end = (s->stage2 != 0U) ? 0.0f : s->v_final;
        (void)spd_plan(s->v, v_end);
    }
    else if (s->state == SCURVE_DONE)
    {
        s->amax = (fabsf(s->v_final) > fabsf(s->v)) ? s->amax_acc : s->amax_dec;
    }
}

/** 内部：以 v_start 为起点、v_target 为终点启动一段 jerk 受限斜坡
 *
 *  本段生效的 amax 按【速度幅值】选取（不是按 dv 的符号）：
 *    |v_target| > |v_start| → 幅值变大 → amax_acc
 *    |v_target| < |v_start| → 幅值变小 → amax_dec
 *  这样"升速命令走 SET_ACCEL、降速/停机走 SET_DECEL"与上位机语义一致，
 *  而反向启动（0 → -15）这种 dv<0 但幅值变大的情形也归到 ACC，符合直觉。
 */
static int spd_plan(float v_start, float v_target)
{
    scurve_spd_t *s = &g_scs;
    float dv = v_target - v_start;
    float D  = fabsf(dv);
    float amax, d_reach;

    if ((v_target != v_target) || (v_target > 1.0e30f) || (v_target < -1.0e30f))
    {
        return -2;                      /* NaN / Inf */
    }
    if ((s->amax_acc <= 0.0f) || (s->amax_dec <= 0.0f) || (s->jmax <= 0.0f))
    {
        return -2;
    }

    /* 目标速度限幅：上限由 scurve_spd_init() 传入（不反向依赖 motor_params.h） */
    if (v_target >  s->vmax) { v_target =  s->vmax; }
    if (v_target < -s->vmax) { v_target = -s->vmax; }
    dv = v_target - v_start;
    D  = fabsf(dv);

    if (D < SCURVE_SPD_MIN_DV) {        /* 已经在位：直接置位 */
        s->v     = v_target;
        s->a     = 0.0f;
        s->j     = 0.0f;
        s->state = SCURVE_DONE;
        if (s->stage2 != 0U)            /* 换向第一段退化成"已在 0"：直接接第二段 */
        {
            s->stage2 = 0U;
            return spd_plan(s->v, s->v_final);
        }
        return 0;
    }

    amax    = (fabsf(v_target) > fabsf(v_start)) ? s->amax_acc : s->amax_dec;
    d_reach = (amax * amax) / s->jmax;

    s->v0    = v_start;
    s->dv    = dv;
    s->dir   = (dv >= 0.0f) ? 1.0f : -1.0f;
    s->amax  = amax;

    if (D >= d_reach) {
        /* 加速度梯形：峰值加速度顶到 amax，中间有一段匀加速 */
        s->ap = amax;
        s->tj = amax / s->jmax;
        s->tc = (D - d_reach) / amax;
    } else {
        /* 加速度三角形：Δv = ap^2/jmax => ap = sqrt(Δv*jmax)，无匀加速段 */
        s->ap = sqrtf(D * s->jmax);
        s->tj = sqrtf(D / s->jmax);
        s->tc = 0.0f;
    }

    s->ta    = 2.0f * s->tj + s->tc;
    s->t0    = s->tick;
    s->state = SCURVE_RUN;

    return 0;
}

int scurve_spd_move(float v_target)
{
    scurve_spd_t *s = &g_scs;
    float v_start = s->v;               /* 以当前给定为起点：支持斜坡中途改目标 */

    if ((v_target != v_target) || (v_target > 1.0e30f) || (v_target < -1.0e30f))
    {
        return -2;                      /* NaN / Inf */
    }
    if ((s->amax_acc <= 0.0f) || (s->amax_dec <= 0.0f) || (s->jmax <= 0.0f))
    {
        return -2;
    }

    /* 目标速度限幅：上限由 scurve_spd_init() 传入（不反向依赖 motor_params.h） */
    if (v_target >  s->vmax) { v_target =  s->vmax; }
    if (v_target < -s->vmax) { v_target = -s->vmax; }

    s->v_final = v_target;
    s->move_cnt++;

    /* 跨零换向（v_start 与 v_target 反号，且两端都离 0 足够远）：
       拆两段 —— 先按 amax_dec 减到 0，再按 amax_acc 反向升到目标。
       整段共用一个 amax 时"前半刹车、后半反向加速"没法同时满足两个限值。 */
    if ((v_start * v_target < 0.0f)
        && (fabsf(v_start)  > SCURVE_SPD_MIN_DV)
        && (fabsf(v_target) > SCURVE_SPD_MIN_DV))
    {
        s->stage2 = 1U;
        return spd_plan(v_start, 0.0f);
    }

    s->stage2 = 0U;
    return spd_plan(v_start, v_target);
}

void scurve_spd_hold(void)
{
    scurve_spd_t *s = &g_scs;

    s->ta      = 0.0f;                  /* 本斜坡作废，保持当前给定 */
    s->state   = SCURVE_DONE;
    s->a       = 0.0f;
    s->j       = 0.0f;
    s->stage2  = 0U;                    /* 换向第二段一并取消 */
    s->v_final = s->v;
    /* s->v 保持不动：就地保持当前速度 */
}

void scurve_spd_isr_tick(void)              //20khz每拍唯一的入口
{
    scurve_spd_t *s = &g_scs;
    float t;

    s->tick++;

    if (s->state != SCURVE_RUN) 
    {
        return;                             /* IDLE / DONE：保持当前速度给定 */
    }

    t = (float)(s->tick - s->t0) * s->ts;

    if (t <= 0.0f) 
    {
        s->a = 0.0f;
        s->j = 0.0f;
        return;
    }

    if (t >= s->ta) {                       /* 收尾对齐，消除浮点残差 */
        s->v     = s->v0 + s->dv;
        s->a     = 0.0f;
        s->j     = 0.0f;
        s->state = SCURVE_DONE;

        /* 跨零换向：第一段（减到 0）刚走完 → 立刻接第二段（0 → v_final）。
           衔接点在 20kHz 里，速度连续、中间不停顿；第二段按 amax_acc 规划。
           这里会走到 spd_plan() 里的 sqrtf —— 只在换向那一刻执行一次，
           不是每拍开销；常态路径（①②③ 三段求值）仍然是纯乘加。 */
        if (s->stage2 != 0U)
        {
            s->stage2 = 0U;
            (void)spd_plan(s->v, s->v_final);
        }
        return;
    }

    if (t < s->tj) 
    {
        /* ① 加加速：jerk = +jmax，a = jmax*t，Δv = 1/2*jmax*t^2 */
        s->j =  s->dir * s->jmax;
        s->a =  s->dir * s->jmax * t;
        s->v =  s->v0 + s->dir * 0.5f * s->jmax * t * t;
    } else if (t < (s->tj + s->tc)) 
    {
        /* ② 匀加速：a 恒为 ap，Δv = d1 + ap*(t-tj) */
        float d1 = 0.5f * s->jmax * s->tj * s->tj;      /* = 0.5*ap*tj */
        s->j = 0.0f;
        s->a = s->dir * s->ap;
        s->v = s->v0 + s->dir * (d1 + s->ap * (t - s->tj));
    } else 
    {
        /* ③ 减加速：用"剩余时间 u"倒扣，与加速段对称 */
        float u  = s->ta - t;                            /* u: tj -> 0 */
        s->j = -s->dir * s->jmax;
        s->a =  s->dir * s->jmax * u;
        s->v = (s->v0 + s->dv) - s->dir * 0.5f * s->jmax * u * u;
    }
}

uint8_t scurve_spd_is_idle(void)
{
    return (g_scs.state == SCURVE_RUN) ? 0U : 1U;
}

float scurve_spd_out(void)
{
    return g_scs.v;
}
