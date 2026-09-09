/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "ctrl_align.h"
#include "motor.h"
#include "hpm_mcl_loop.h"

void ctrl_align_run(void)
{
    mcl_motor_alignment_cfg_t alignment_cfg;

    alignment_cfg.algorithm = mcl_alignment_algorithm_three_stage;

    /* 三阶段对齐参数（软件 FOC）
     *   stage1: 大电流吸合 + q 轴扰动（帮助脱离静摩擦）
     *   stage2: 中等电流稳定
     *   stage3: 小电流收敛 */
    alignment_cfg.config.three_stage.stage1.d_current = 4.0f;
    alignment_cfg.config.three_stage.stage1.q_current = 0.6f;
    alignment_cfg.config.three_stage.stage1.delay_ms  = 500;

    alignment_cfg.config.three_stage.stage2.d_current = 1.5f;
    alignment_cfg.config.three_stage.stage2.delay_ms  = 800;

    alignment_cfg.config.three_stage.stage3.d_current = 1.0f;
    alignment_cfg.config.three_stage.stage3.delay_ms  = 400;

    alignment_cfg.config.three_stage.final_delay_ms = 100;

    hpm_mcl_motor_angle_alignment(motor_get_loop(), &alignment_cfg);
}
