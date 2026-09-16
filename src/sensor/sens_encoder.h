/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file sens_encoder.h
 * @brief 编码器传感器层：单圈机械角 / 多圈连续角（MCL 回调）
 */
#ifndef SENS_ENCODER_H
#define SENS_ENCODER_H

#include "hw_map.h"

/** 1=用虚拟角度代替真实编码器（脱离电机验证电流环/SVPWM 链路用），0=读真实 MT6835
 *
 * 放在 sensor 层而不是 app_cfg.h：这是编码器自身的行为配置，
 * 若留在 app 层会让 sensor 反向依赖 app，破坏分层。 */
#ifndef USE_VIRTUAL_ANGLE
#define USE_VIRTUAL_ANGLE 0
#endif

/**
 * @brief 单拍最大可信角度增量 rad（抗"字节撕裂"与干扰毛刺）
 *
 * 3000rpm（50 转/秒）在 20kHz 一拍内只转 0.9° ≈ 0.016rad，这里给到 0.35rad
 * （约 20°、20 倍余量），正常运动绝不会触发。
 * 只有"三次单字节读在进位边界拼出横跨半个量程的假值"这类故障才会撞上——
 * 那种假值会让速度尖峰 → 电流环冲击 → 起步过冲，甚至触发过流保护关断输出。
 * 撞上就丢弃本帧、沿用上一次的角度，并计入 g_angle_jump。
 */
#define ENC_MAX_STEP_RAD  (0.35f)

/** 1=由 ADC 中断按 20kHz 刷新角度；对齐阶段保持 0 */
extern volatile uint8_t g_encoder_isr_enable;

/**
 * @brief MCL 回调：读取单圈机械角 [0, 2π)
 *
 * 由 hpm_mcl_encoder_process() 在 20kHz 中断里调用。
 * SPI 偶发失败时用上一帧角度顶住，连续失败超过 5 次才上报失败
 * （一旦上报，MCL 会把 encoder->status 永久置 fail，角度再也不更新）。
 */
hpm_mcl_stat_t encoder_get_theta(float *theta);

/**
 * @brief MCL 回调：读取连续多圈机械角（位置环反馈）
 *
 * MT6835 是单圈绝对编码器，而 MCL 的 hpm_mcl_position_pid() 直接算
 * err = setpoint - feedback，不做 ±π 归一化。若反馈用单圈值，
 * 在 0/2π 回绕处 err 会瞬间跳 ±2π，位置环输出翻符号 ——
 * 表现为"转过目标却以为还差一圈"。这里把单圈角展开成连续多圈角。
 */
hpm_mcl_stat_t encoder_get_abs_theta(float *theta);

/**
 * @brief 把当前机械位置设为多圈角的 0 基准
 *
 * 位置环目标用"相对该基准的增量"，方向唯一确定，避免每次上电转向随机。
 */
void encoder_abs_rebase(void);

/**
 * @brief MCL 回调：启动采样（本编码器为轮询读取，无需操作）
 */
hpm_mcl_stat_t encoder_start_sample(void);

#endif /* SENS_ENCODER_H */
