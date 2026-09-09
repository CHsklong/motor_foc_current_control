/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_MOTOR_H
#define HPM_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "hpm_mcl_loop.h"
#include "hpm_mcl_detect.h"

/**
 * @brief 电机抽象层
 *
 * 把"一台电机"封装成对象：包含 MCL 的 encoder / analog / drivers / control / loop / detect
 * 六个子模块及其配置。上层（control / app）只通过本层接口操作电机，
 * 不直接接触 MCL 内部结构体。
 *
 * 本层同时负责把 sensor 与 driver 提供的语义化接口，适配成 MCL 要求的回调签名。
 */

typedef struct {
    mcl_encoder_t encoder;
    mcl_filter_iir_df1_t encoder_iir;
    mcl_filter_iir_df1_memory_t encoder_iir_mem[2];
    mcl_analog_t analog;
    mcl_drivers_t drivers;
    mcl_control_t control;
    mcl_loop_t loop;
    mcl_detect_t detect;
    struct {
        mcl_cfg_t mcl;
        mcl_encoer_cfg_t encoder;
        mcl_filter_iir_df1_cfg_t encoder_iir;
        mcl_filter_iir_df1_matrix_t encoder_iir_mat[2];
        mcl_analog_cfg_t analog;
        mcl_drivers_cfg_t drivers;
        mcl_control_cfg_t control;
        mcl_loop_cfg_t loop;
        mcl_detect_cfg_t detect;
    } cfg;
} motor_t;

extern motor_t motor0;

/**
 * @brief 初始化电机对象：装配参数、注册回调、初始化 MCL 各子模块
 * @note 调用前必须先完成 drv_pwm_clock_init()（需要 PWM 重载值）
 */
void motor_init(void);

/**
 * @brief 获取 MCL 控制环句柄（供 control 层调用 MCL 算法）
 */
mcl_loop_t *motor_get_loop(void);

/**
 * @brief 获取母线电压(V)（初始化时实测，用于 SVPWM 调制与解耦前馈）
 */
float motor_get_vbus(void);

/**
 * @brief 设置 q 轴电流给定（转矩电流）
 */
void motor_set_current_q(float iq);

/**
 * @brief 设置 d 轴电流给定（励磁电流）
 */
void motor_set_current_d(float id);

/**
 * @brief 使能控制环（必须先使能，对齐阶段才会有电流输出）
 */
void motor_enable_loop(void);

/**
 * @brief 编码器状态更新（与电流环同频调用）
 */
void motor_encoder_process(uint32_t tick);

/**
 * @brief 获取一个电流环周期对应的 CPU tick 数
 *
 * 编码器 M 法测速需要"距离上次调用过了多少个 CPU 周期"作为时间基准，
 * 传错会让速度成比例失真（在 1kHz 主循环里传 50us 会放大 20 倍）。
 */
uint32_t motor_get_loop_tick(void);

/**
 * @brief 直接设置零点偏移（跳过对齐时使用）
 */
void motor_set_initial_theta(float theta);

/**
 * @brief 获取零点偏移
 */
float motor_get_initial_theta(void);

/**
 * @brief 故障检测循环（由 1ms 定时器滴答驱动）
 */
void motor_detect_loop(void);

/**
 * @brief 使能 / 关闭 PWM 输出
 */
void motor_enable_output(void);
void motor_disable_output(void);

/**
 * @brief 设置某相占空比（供开环 SVPWM 使用）
 */
void motor_set_duty_raw(uint8_t phase, float duty);

/**
 * @brief 读取 A/B 相电流物理值（单位 A）
 * @note 必须按 A → B 顺序读取：B 相由 A+C 重构，依赖 A 相的采样缓存
 */
bool motor_get_phase_current(float *ia, float *ib);

#endif /* HPM_MOTOR_H */
