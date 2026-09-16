/*
 * Copyright (c) 2021-2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file hw_map.h
 * @brief 硬件映射层：统一 SDK 头引用 + 工程级硬件常量
 *
 * 分层约定：所有 .c 只包含 "hw_map.h" + 本模块头文件 + 所依赖的下层头文件，
 * 不再各自堆一长串 SDK include，避免新增模块时漏引、重复引。
 *
 * 层级依赖（上层可包含下层，反之禁止）：
 *   app -> control -> motor -> sensor -> driver -> hal
 *   debug 为横切层，任何层都可引用，自身不依赖任何业务层
 */
#ifndef HW_MAP_H
#define HW_MAP_H

#include "board.h"
#include "clock.h"
#include "hpm_spi_drv.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "hpm_debug_console.h"
#include "hpm_sysctl_drv.h"
#include "hpm_mcl_control.h"
#if defined(HPMSOC_HAS_HPMSDK_PWM)
#include "hpm_pwm_drv.h"
#endif
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
#include "hpm_pwmv2_drv.h"
#endif
#include "hpm_trgm_drv.h"
#if defined(HPMSOC_HAS_HPMSDK_QEI)
#include "hpm_qei_drv.h"
#endif
#ifdef HPMSOC_HAS_HPMSDK_GPTMRV2
#include "hpm_gptmrv2_drv.h"
#else
#include "hpm_gptmr_drv.h"
#endif
#if defined(HPMSOC_HAS_HPMSDK_QEIV2)
#include "hpm_qeiv2_drv.h"
#endif
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_adc_v2.h"
#include "hpm_mcl_loop.h"
#include "hpm_mcl_abz.h"
#include "hpm_mcl_detect.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif
#include "hpm_dmamux_drv.h"

/* 硬件 FOC / 混合环路由 SES 或 CMake 侧定义，默认关闭（当前走软件 FOC） */
/*#define HW_CURRENT_FOC_ENABLE   1*/
/*#define MCL_HARDWARE_HYBRID_LOOP_ENABLE   1*/

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
#include "hpm_clc_drv.h"
#include "hpm_synt_drv.h"
#endif
#if defined(HW_CURRENT_FOC_ENABLE)
#include "hpm_vsc_drv.h"
#include "hpm_clc_drv.h"
#include "hpm_trgm_soc_drv.h"
#include "hpm_qeov2_drv.h"
#include "hpm_synt_drv.h"
#endif
#if defined(CONFIG_HPM_MONITOR)
#include "monitor.h"
#endif

/* ---------------- PWM ---------------- */
#define PWM_FREQUENCY               (20000)     /* PWM 载波频率 20kHz */
#define PWM_RELOAD                  ((motor_clock_hz / PWM_FREQUENCY) - 1)
#define PWM_DEAD_AREA_TICK          (50)        /* 死区（单位：PWM 输入时钟周期 /2） */
#define MOTOR0_BLDCPWM              BOARD_BLDCPWM

/* ---------------- 电流环 ---------------- */
#define MOTOR0_CURRENT_LOOP_BANDWIDTH (200)     /* 电流环设计带宽 Hz */

/* ---------------- ADC 通道索引（adc_buff 的行号） ---------------- */
#define ADCU_INDEX 0        /* ADC1：U 相 */
#define ADCV_INDEX 1        /* ADC0：V 相（实际硬件接 C 相，见 sens_analog.c 注释） */

/* ---------------- 通用 ---------------- */
#define MOTOR0_SPD                  (20.0)      /* r/s，delta 0.1r/s，范围 1~40 */

/* ---- 静态零点校准 ----
 * CURRENT_SET_TIME_MS 只是"平均次数"，不等于耗时。真正决定耗时的是采样间隔
 * ADC_MID_SAMPLE_US：PWM 触发是 20kHz（每 50us 一帧），原来用 board_delay_ms(1)
 * 等于 20 帧才取 1 个样本，200 次就白白等了 200ms —— 这是"上电要等一下"的大头。
 * 改成 100us（每 2 帧取 1 个）后同样 200 次平均只要 20ms，降噪效果不变。 */
#define CURRENT_SET_TIME_MS         (200)       /* 静态零点校准平均次数 */
#define ADC_MID_SAMPLE_US           (100)       /* 零点校准的采样间隔 us（>= 2 个 PWM 周期） */

/* ---- 静态零点校准的上电加固（冷上电时模拟前端还在爬升，别拿爬升期的数据当零点）---- */
#define ADC_MID_DISCARD_TIMES       (64)        /* 开始平均之前先丢掉的采样数（等运放/基准稳定） */
#define ADC_MID_WAIT_CNT_MAX        (100)       /* 等 ADC DMA 首帧的最大探测次数（耗时 = 次数×ADC_MID_SAMPLE_US，上限 10ms）；耗满说明触发链没起来 */
#define ADC_MID_RETRY_MAX           (3)         /* 结果不合理时的重试次数 */
#define ADC_MID_MIN                 (1024U)     /* 合理零点下限：1/4 量程 */
#define ADC_MID_MAX                 (3072U)     /* 合理零点上限：3/4 量程 */

/* PWM 时钟频率，由 main() 在初始化阶段填入，PWM_RELOAD 宏依赖它 */
extern int32_t motor_clock_hz;

#endif /* HW_MAP_H */
