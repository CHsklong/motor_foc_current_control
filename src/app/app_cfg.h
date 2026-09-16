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
#define FOC_CURRENT_MODE  0    /* 电流环：直接给 id/iq */
#define FOC_SPEED_MODE    0    /* 速度环：给机械角速度 rad/s */
#define FOC_POSITION_MODE 1    /* 位置环：给相对基准的位置增量 rad */

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
#define POS_TARGET_DELTA  (12.5f * MCL_PI)


#define ENC_SKIP_ALIGN    1   /* 1=跳过对齐，用下面的常量；0=每次上电对齐 */
#define ENC_THETA_INITIAL 4.9862f   /* 实测的初始机械弧度 */

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
#define step_response     0   /* 电流环阶跃响应测试 */

#endif /* APP_CFG_H */
