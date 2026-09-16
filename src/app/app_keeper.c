/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_pwm.h"
#include "sens_analog.h"
#include "motor.h"
#include "dbg_probe.h"
#include "app_cfg.h"
#include "app_keeper.h"

/**
 * 运行监护实现
 *
 * 每毫秒被调用一次，按顺序处理三类"没有恢复通道"的锁死：
 *   ① PWM 硬件故障锁存 → 自动清标志
 *   ② PWM 输出被软件关断（detect 的 disable_output）→ 自动恢复
 *   ③ 控制环 status=fail → 软重启一次
 * 外加一次上电母线电压复核修正。
 *
 * 【为什么不无条件自愈】单纯的 PWM 锁存清掉就没事了；但如果 MCL 已经判定
 * 过真实故障（g_fault_src 非 0），说明采样/驱动/编码器里确有问题，
 * 这时反复使能输出可能直接烧功率级。所以这里只在 g_fault_src==0 时动作，
 * 否则只记录 g_keeper_blocked 并保持停机。
 */

#if RUN_KEEPER_ENABLE

/** 自愈动作原因码（写进 g_keeper_last_cause，便于 J-Scope 分辨发生了什么） */
enum {
    KEEPER_CAUSE_NONE        = 0,
    KEEPER_CAUSE_FAULT_CLR   = 1,   /* 清掉了 PWM 硬件故障锁存 */
    KEEPER_CAUSE_OUT_RECOVER = 2,   /* 恢复被关断的 PWM 输出 */
    KEEPER_CAUSE_LOOP_REINIT = 3,   /* 软重启了控制环 */
    KEEPER_CAUSE_VBUS_FIX    = 4    /* 修正了偏低的母线电压 */
};

static uint32_t s_tick_ms;          /* 调用计数，主循环约 1kHz，可直接当毫秒用 */
static uint32_t s_cooldown_until;   /* 冷却结束时刻 */
static uint32_t s_action_cnt;       /* 已执行的自愈动作总数 */
static uint8_t  s_vbus_checked;     /* 母线电压复核是否已做过 */
static uint8_t  s_soft_retry_done;  /* 上电窗口内的软恢复机会是否已用掉 */

/** g_fault_src 位图里"可能烧毁功率级"的源：bit0=采样/过流 bit2=驱动
 *  —— 这两类一律不自愈；bit1=环路 bit3=编码器在上电窗口内允许试一次。 */
#define KEEPER_HARD_SRC_MASK  ((1U << 0) | (1U << 2))
#define KEEPER_SOFT_SRC_MASK  ((1U << 1) | (1U << 3))
#define KEEPER_BOOT_WINDOW_MS (3000U)   /* 上电 Window：认为编码器/PLL 还在建立 */

/** 是否还有自愈额度：超额后彻底停机，防止在无人在场时反复重启 */
static bool s_budget_left(void)
{
    if (KEEPER_MAX_ACTION == 0U) {
        return true;
    }
    return (s_action_cnt < (uint32_t)KEEPER_MAX_ACTION);
}

/**
 * @brief 安全门槛：什么时候允许自愈
 * @retval true 可以动作
 *
 * 规则：
 *   1) MCL 从未判过故障（g_fault_src==0）→ 允许（上电毛刺误锁的常见情形）；
 *   2) 判过故障，但只涉及 环路/编码器，且还在上电窗口内 → 允许【一次】。
 *      冷上电时 MT6835 没完成 POR、PLL 没锁好，detect 很容易上来就判 encoder/loop fail
 *      并永久关断输出 —— 而那时输出根本还没真正出过力，重启不会烧机；
 *   3) 涉及 采样/过流 或 驱动 的故障 → 永远不自愈，保持停机查因。
 */
static bool s_safe_to_recover(void)
{
    if (!s_budget_left()) {
        return false;
    }
    if (g_fault_src == 0U) {
        return true;
    }
    if ((g_fault_src & KEEPER_HARD_SRC_MASK) != 0U) {
        return false;
    }
    if ((g_fault_src & KEEPER_SOFT_SRC_MASK) == 0U) {
        return false;
    }
    if (s_soft_retry_done != 0U) {
        return false;
    }
    if (s_tick_ms > (uint32_t)KEEPER_BOOT_WINDOW_MS) {
        return false;
    }
    s_soft_retry_done = 1U;
    return true;
}

void app_keeper_update(void)
{
    s_tick_ms++;

    /* 上电刚开始的一段窗口不做任何干预：让零点校准、首次出力、滤波器收敛先跑完，
       否则监护会把正常的上电瞬态误判成故障去"救"。 */
    if (s_tick_ms < (uint32_t)KEEPER_START_DELAY_MS) {
        return;
    }

    /* ---- 母线电压复核（只做一次） ----
       motor_init() 里那次 read_vbus() 发生在 pwm_init() 之前，
       此时母线的可能还没爬到标称值，而 const_vbus 一旦定下就不再刷新，
       后续 dq 解耦与电压限幅会一直按偏小的母线算 → 出力不足，表现为"不转"。
       这里等系统稳定后重读：只有读到明显更高的合理值才覆盖，
       避免把噪声毛刺当成母线电压。 */
#if KEEPER_VBUS_RECHECK
    if (s_vbus_checked == 0U)
    {
        float v = read_vbus();
        g_vbus_now = v;

        if ((v > 1.0f) && (v < 60.0f) && (v > (motor0.cfg.mcl.physical.motor.vbus + 0.5f)))
        {
            /* const_vbus 就是指向这个成员，改它即刻生效 */
            motor0.cfg.mcl.physical.motor.vbus = v;
            g_keeper_vbus_upd++;
            g_keeper_last_cause = KEEPER_CAUSE_VBUS_FIX;
        }
        s_vbus_checked = 1U;
    }
#endif

    if (s_tick_ms < s_cooldown_until) {
        return;
    }

    /* ---- ① PWM 硬件故障锁存 ----
       硬件配的是"故障后需软件清标志"，不清就永远钉死输出。
       清之前必须确认不是真实过流：真实过流时 MCL 的 detect 一定判过并置起 g_fault_src。 */
#if KEEPER_FAULT_AUTO_CLR
    if (pwm_fault_is_latched())
    {
        if (s_safe_to_recover())
        {
            pwm_fault_clear();
            g_keeper_fault_clr++;
            g_keeper_last_cause = KEEPER_CAUSE_FAULT_CLR;
            s_action_cnt++;
            s_cooldown_until = s_tick_ms + (uint32_t)KEEPER_COOLDOWN_MS;
            return;
        }
        g_keeper_blocked = 1U;      /* 有真实故障记录，拒绝自愈，保持关断 */
    }
#endif

    /* ---- ② 输出被软件关断（detect 的 disable_output 走过） ---- */
#if KEEPER_OUT_AUTO_REC
    if (!pwm_output_is_enabled())
    {
        if (s_safe_to_recover())
        {
            enable_all_pwm_output();
            g_keeper_out_rec++;
            g_keeper_last_cause = KEEPER_CAUSE_OUT_RECOVER;
            s_action_cnt++;
            s_cooldown_until = s_tick_ms + (uint32_t)KEEPER_COOLDOWN_MS;
            return;
        }
        g_keeper_blocked = 2U;
    }
#endif

    /* ---- ③ 控制环 status=fail：软重启一次 ----
       注意只重启环路使能，不动已写入的位置/速度给定，避免每次自愈都丢目标。 */
#if KEEPER_LOOP_AUTO_REC
    if (motor0.loop.status == loop_status_fail)
    {
        if (s_safe_to_recover())
        {
            hpm_mcl_loop_disable(&motor0.loop);
            hpm_mcl_loop_enable(&motor0.loop);
            g_keeper_loop_rec++;
            g_keeper_last_cause = KEEPER_CAUSE_LOOP_REINIT;
            s_action_cnt++;
            s_cooldown_until = s_tick_ms + (uint32_t)KEEPER_COOLDOWN_MS;
            return;
        }
        g_keeper_blocked = 3U;
    }
#endif
}

#else

void app_keeper_update(void)
{
    /* 监护关闭：什么都不做 */
}

#endif /* RUN_KEEPER_ENABLE */
