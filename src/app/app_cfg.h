/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file app_cfg.h
 * @brief 应用层配置：运行模式与策略开关
 *
 * 改运行模式只需要动这个文件。
 */
#ifndef APP_CFG_H
#define APP_CFG_H

#include "hw_map.h"
#include "motor_params.h"   /* MOTOR_LOOP_* 枚举（在参数头里，避免分层倒挂） */

/* ---------------- 运行模式 ---------------- */
#define SVPWM_MODE        0    /* 开环 SVPWM 拖动（不带反馈，用于验证功率级） */
#define FOC_MODE          1    /* FOC 闭环总开关（下面的模式依赖它） */
#define FOC_CURRENT_MODE  1    /* 电流环：直接给 id/iq */
#define FOC_SPEED_MODE    0    /* 速度环：给机械角速度 rad/s */
#define FOC_POSITION_MODE 0    /* 位置环：给相对基准的位置增量 rad */

#if (FOC_CURRENT_MODE + FOC_SPEED_MODE + FOC_POSITION_MODE) > 1
#error "FOC_CURRENT_MODE / FOC_SPEED_MODE / FOC_POSITION_MODE 同时只能有一个为 1"
#endif

/** 由上面的宏推导出当前闭环结构，供 motor_set_loop_mode() 使用 */
#if FOC_POSITION_MODE
#define MOTOR_RUN_LOOP_MODE  MOTOR_LOOP_POSITION
#elif FOC_SPEED_MODE
#define MOTOR_RUN_LOOP_MODE  MOTOR_LOOP_SPEED
#else
#define MOTOR_RUN_LOOP_MODE  MOTOR_LOOP_CURRENT
#endif

/* ---------------- 上电时序 ----------------
 * 冷上电时母线电容、电流采样运放、ADC 基准、磁编码器都有自己的上电爬升时间，
 * 而调试器下载是在板子已上电数分钟之后才让程序跑起来的 —— 两者唯一的区别就在这。
 * "烧录后必转、重新上电时好时坏" 基本都是这段稳定期被跳过导致的。
 * 注意：等待是纯忙等，每一项都会 1:1 变成"上电到开始转"的延迟，够用即可。
 * 当前预算 ≈ 120 + 10 + 2 + 6.4 + 20 + 20 ≈ 180ms（改之前是 500+100+64+200+50+20 ≈ 930ms）。
 * 关键不在把这些数调小，而在【pwm_init() 提到零点校准之前】——
 * 相电流 ADC 由 PWM 触发，PWM 没跑起来时等首帧必然耗满探测上限，
 * 那才是原来那近 1 秒里最冤的一截。详见 main.c 里的启动顺序注释。 */
/** 复位后等待电源/模拟前端/编码器稳定的时间 ms（含母线电容充电） */
#define POWER_UP_SETTLE_MS  (120)
/** pwm_init() 之后、开输出之前，等待过流比较器输出稳定的时间 ms
 *  （上电稳定期已经把模拟前端等稳了，这里只需覆盖"故障输入刚使能"的毛刺） */
#define PWM_FAULT_SETTLE_MS (10)

/* ---------------- 位置环目标 ---------------- */
#define POS_TARGET_DELTA  (1.0f * MCL_PI)


#define ENC_SKIP_ALIGN    1   /* 1=跳过对齐，用下面的常量；0=每次上电对齐 */
#define ENC_THETA_INITIAL 4.9862f   /* 实测的初始机械弧度 */

/* ---------------- 运行监护 app_keeper（解决"冷上电不转 / 转过一次后就不转"） ----------------
 * 背景：这套闭环里有三处"单向锁死"，一旦触发就再也回不来，而且代码里没有任何提示：
 *   ① PWM 硬件故障锁存：pwm_init() 配的是 禁止硬件自动恢复、需软件清标志。
 *      外部过流比较器在上电毛刺期拉一下低，六路输出就被永久钉死为 0；
 *      此时 CPU、20kHz 中断、心跳全正常，只有 PWM 的 SR.FAULT 位能看出来。
 *   ② MCL detect 回调 disable_output：判出任何故障（含上电瞬间的误判）就关断输出，没有恢复路径。
 *   ③ 母线电压只在 motor_init() 里读一次，之后 const_vbus 不再刷新；
 *      冷上电若在模拟前端没稳定时读到偏低值，后续所有电压限幅都偏小，力矩不足表现出"不转"。
 * 前两类的触发窗口恰好在上电爬升期 —— 这正是"调试器下载后必转、重新上电不转"的来源。
 * 监护的策略：**只有在 MCL 从未判定过真实故障（g_fault_src==0）时才自愈**；
 * 一旦 MCL 明确判过 采样/环路/驱动/编码器 故障，就停机不自愈（宁可不转也不能烧功率级），
 * 并把拒绝原因写进 g_keeper_blocked 供 J-Scope 查看。 */
#define RUN_KEEPER_ENABLE    1   /* 1=在主循环里跑运行监护；0=完全关闭 */
/** 下列子项仅在 RUN_KEEPER_ENABLE=1 时有效 */
#define KEEPER_FAULT_AUTO_CLR 1  /* 1=自动清除 PWM 硬件故障锁存（无真实故障时） */
#define KEEPER_OUT_AUTO_REC   1  /* 1=PWM 输出被意外关断时自动恢复 */
#define KEEPER_LOOP_AUTO_REC  1  /* 1=控制环 status=fail 时软重启一次 */
#define KEEPER_VBUS_RECHECK   1  /* 1=上电稳定后复核并修正母线电压 */
#define KEEPER_START_DELAY_MS (1000)  /* 上电后先观察这么久再开始监护（ms），让起转/校准先跑完 */
#define KEEPER_COOLDOWN_MS    (500)   /* 两次自愈动作之间的最小间隔 ms */
#define KEEPER_MAX_ACTION     (8)     /* 自愈总次数上限，超过后停机待查；0=不限 */

/* 主循环停滞兜底：main 被 20kHz 中断饿死时怎么办
 *
 * 1=停滞超过 60ms 后，每 4 拍跳过 1 拍完整控制计算，强制给 main 留出时间片，
 *   同时由 ISR 代跑运行监护。代价是被跳过的那拍没有新的控制输出（维持上一拍），
 *   但这是"已经完全冻死"时的降级保命选项，不是常态。
 * 0=只做观测与监护接管，不降控制频率。 */
#define ISR_RELIEF_ENABLE     (1)

/* ---------------- FLASH 参数区 ---------------- */
/** 1=运行时从片内 FLASH 读零点偏移（走 XIP 指针直读，不调任何 ROM API，零风险）；
    0=完全不碰 FLASH，直接用 ENC_THETA_INITIAL。 */
#define FLASH_PARAM_ENABLE       1

/** 标定开关：1=本次固件专门用来把零点偏移烧进 FLASH（上电擦写一次），0=常规运行。
 *
 *  【为什么不能常开】XIP 下代码就在 FLASH 里执行，取指速率靠 BootROM 启动时配好的
 *  XPI 状态。而 ROM 的 XPI 探测类 API（rom_xpi_nor_get_config / auto_config /
 *  get_property）都会把 XPI 控制器重新初始化一遍，覆盖掉那个状态。程序不会死，
 *  但取指变慢，20kHz 控制节拍跑不完 → 电流环时序抖动 → 电机异响、速度环起不来。
 *  实测：只要上电调一次这类 API，即便最终没擦写（g_flash_erase_ms=0）也照样异响。
 *
 *  所以常规运行一律不碰这些 API，只做 XIP 指针读；需要更新 FLASH 时才置 1。
 *
 *  用法：置 1 烧录并运行一次（J-Scope 看 g_flash_status==0 即写入成功），
 *        然后改回 0 重新烧录。调试器下载不擦末尾 sector，参数会保留。 */
#define FLASH_PARAM_CALIB_ONCE   0

/* ---------------- 调试开关 ---------------- */
#define motor_ban         0   /* 闭环静止模式 */
#define step_response     1   /* 电流环阶跃响应测试 */

#endif /* APP_CFG_H */
