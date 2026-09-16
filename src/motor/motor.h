/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file motor.h
 * @brief 电机抽象层：MCL 各子模块句柄聚合 + 参数整定 + 初始角对齐
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "hw_map.h"
#include "motor_params.h"

/** MCL 全部子模块句柄与配置的聚合体 */
typedef struct {
    mcl_encoder_t encoder;
    mcl_filter_iir_df1_t encoder_iir;
    mcl_filter_iir_df1_memory_t encoder_iir_mem[2];
    mcl_analog_t analog;
    mcl_drivers_t drivers;
    mcl_control_t control;
    mcl_loop_t loop;
    mcl_detect_t detect;
#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    mcl_hw_loop_t hw_loop;
#endif
    struct
    {
        mcl_cfg_t mcl;
        mcl_encoer_cfg_t encoder;
        mcl_filter_iir_df1_cfg_t encoder_iir;
        mcl_filter_iir_df1_matrix_t encoder_iir_mat[2];
        mcl_analog_cfg_t analog;
        mcl_drivers_cfg_t drivers;
        mcl_control_cfg_t control;
        mcl_loop_cfg_t loop;
        mcl_detect_cfg_t detect;
        mcl_hardware_clc_cfg_t clc;
#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
        mcl_hw_loop_cfg_t hw_loop;
#endif
    } cfg;
} motor0_t;

/** 全局电机实例（放 FAST_RAM，20kHz 中断高频访问） */
extern motor0_t motor0;

/** 强制注入的电角度（开环调试用） */
extern mcl_user_value_t user_set_theta;

/** ABZ 绝对位置角（硬件 QEI 路径用） */
extern float abs_position_theta;

/* ---------------- MCL 回调 ---------------- */
void motor0_control_init(void);

/**
 * @brief 电机与 MCL 全参数初始化：物理参数、环周期、PI 增益、滤波器、回调注册
 */
void motor_init(void);

/**
 * @brief 速度环参数整定（切换到速度模式时调用）
 *
 * 注意：会把电流环 PI 改成"旧公式"（带宽平方项），与 motor_init() 里的
 * 标准整定不一致；切到速度模式前请确认这是你想要的。
 */
void motor0_speed_loop_para_init(void);

/**
 * @brief 位置环参数整定（切换到位置模式时调用）
 *
 * 当前未被 main() 调用，位置环增益以 motor_init() 里的配置为准。
 */
void motor0_position_loop_para_init(void);

/**
 * @brief 选择闭环结构
 *
 * MCL 的 hpm_mcl_current_foc_loop() 里两个环是串联的：
 *   - enable_position_loop=false 时，exec_ref.speed 被强制置 0；
 *   - enable_position_loop=true 但 ref_position.enable=false 时，位置环**照样在跑**，
 *     把 ref_position 取成 exec_ref.position(通常为 0)、反馈取成连续多圈角，
 *     于是 err 是个巨大负值 → 输出饱和 → 积分顶死，同时每拍多一次 SPI 读编码器。
 *     跑速度环时这是纯粹的副作用，必须关掉。
 * 所以速度环模式一定要显式关掉位置环，不能"两个都 enable 靠给定开关切换"。
 */
void motor_set_loop_mode(motor_loop_mode_t mode);

/**
 * @brief 编码器初始角对齐（三段式：大电流吸合 → 中电流 → 小电流）
 */
void motor_angle_align(void);

#endif /* MOTOR_H */
