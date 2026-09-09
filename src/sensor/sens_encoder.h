/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_SENS_ENCODER_H
#define HPM_SENS_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 传感器接口层 —— 转子角度
 *
 * 职责边界：把"芯片原始码值"翻译成"机械弧度"。
 * 不涉及极对数、电角度 —— 那是 motor/control 层的事。
 */

#ifndef SENS_ENCODER_PI
#define SENS_ENCODER_PI 3.14159265358979323846f
#endif

/* 调试用虚拟角度：置 1 时不读芯片，改为自行递增的角度（用于隔离"角度源"问题） */
#ifndef SENSOR_USE_VIRTUAL_ANGLE
#define SENSOR_USE_VIRTUAL_ANGLE 0
#endif

#define SENSOR_VIRTUAL_FREQ_HZ (10.0f)

/**
 * @brief 编码器初始化（含芯片通信自检）
 * @note 必须在电机出力之前、轴静止时调用
 */
void sens_encoder_init(void);

/**
 * @brief 获取转子机械角度
 * @param[out] theta 机械弧度，范围 [0, 2π)
 * @return true 有效；false 读取失败
 */
bool sens_encoder_get_theta(float *theta);

/**
 * @brief 获取多圈绝对角度（MT6835 为单圈绝对值芯片，等价于机械角）
 */
bool sens_encoder_get_abs_theta(float *theta);

/* ---------------- 调试观测变量 ---------------- */
extern volatile uint32_t g_angle_raw;   /* 21bit 原始角度 */
extern volatile float    theta_r;       /* 换算后的机械弧度 */

#endif /* HPM_SENS_ENCODER_H */
