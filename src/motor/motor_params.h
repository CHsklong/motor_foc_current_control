/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_MOTOR_PARAMS_H
#define HPM_MOTOR_PARAMS_H

/**
 * @brief 电机与采样电路参数 —— 换电机时只需要改这个文件
 *
 * ⚠️ 当前值来自 HPM SDK 样例模板，并非雷赛 BLM57050 的实测值。
 *    改为真实参数时必须同步核对电流环 PI 整定（见 motor.c 的 motor_tune_current_loop()），
 *    否则可能因相位裕度不足而振荡。
 */

/* ---------------- 电流环 ---------------- */
#define MOTOR0_CURRENT_LOOP_BANDWIDTH   (200)      /* 电流环设计带宽(Hz) */

/* ---------------- 采样电路 ---------------- */
#define MOTOR0_ADC_REF_VOL              (3.3f)     /* ADC 参考电压(V) */
#define MOTOR0_OPAMP_GAIN               (20.0f)    /* CSA240L 运放增益（原理图注明） */
#define MOTOR0_ADC_PRECISION            (4095.0f)  /* 12bit */
#define MOTOR0_SAMPLE_RES               (0.005f)   /* 采样电阻(Ω) */
#define MOTOR0_NUM_SAMPLE_RES           (2)        /* 采样电阻数量（A、B 两相） */

/* ---------------- 电机本体 ---------------- */
#define MOTOR0_I_MAX                    (9.0f)     /* 最大电流，过流保护阈值(A) */
#define MOTOR0_INERTIA                  (0.075f)   /* 转动惯量（影响速度环响应） */
#define MOTOR0_LS                       (0.00263f) /* 定子电感(H) */
#define MOTOR0_POLE_NUM                 (4)        /* 极对数（不是极数！） */
#define MOTOR0_POWER                    (50.0f)    /* 额定功率(W) */
#define MOTOR0_RES                      (0.0011f)  /* 定子电阻(Ω) */
#define MOTOR0_RPM_MAX                  (3500.0f)  /* 最大转速 */
#define MOTOR0_FLUX                     (0.0015f)  /* 永磁体磁链(Wb) */
#define MOTOR0_LD                       (0.0026f)  /* d 轴电感(H) */
#define MOTOR0_LQ                       (0.0026f)  /* q 轴电感(H) */

/* ---------------- 速度环 / 位置环 PID ---------------- */
#ifndef BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP     (0.0074f)
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI     (0.0001f)
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KP  (0.05f)
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KI  (0.001f)
#define BOARD_BLDC_SW_FOC_POSITION_KP             (154.7f)
#define BOARD_BLDC_SW_FOC_POSITION_KI             (0.113f)
#endif

#endif /* HPM_MOTOR_PARAMS_H */
