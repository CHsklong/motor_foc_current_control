/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file mt6835.h
 * @brief MT6835 磁编码器芯片驱动（SPI 寄存器访问层）
 *
 * 只负责"把角度字节读出来"，不做角度换算/多圈展开（见 sens_encoder.c）。
 */
#ifndef MT6835_H
#define MT6835_H

#include "hw_map.h"

/* MT6835 SPI 命令与寄存器地址 */
#define MT6835_CMD_READ_REG   0x3   /* 4-bit 命令 '0011' */
#define MT6835_REG_ANGLE_MSB  0x003 /* 角度高字节 */
#define MT6835_REG_ANGLE_MID  0x004
#define MT6835_REG_ANGLE_LSB  0x005
#define MT6835_REG_USER_ID    0x001 /* 用户 ID 寄存器（可用于通信验证） */

/** 上电自检的重试次数与间隔
 *
 * 自检要求 4 轮"连读 vs 三次单读"完全一致。冷上电时 MT6835 自身的 POR
 * 可能还没结束，第一轮读到的是垃圾 → 整轮自检判失败 → 永久退回三次单字节读
 * （慢 5 倍，且三个字节来自不同时刻锁存，在字节进位边界会拼出横跨半个量程的假角度
 *  → 速度尖峰 / 位置跳变 / 起步过冲）。所以必须重试，别一锤子买卖。 */
#define MT6835_SEQ3_RETRY     (5U)
#define MT6835_SEQ3_RETRY_MS  (20U)

/** 上一帧角度原始码值；SPI 偶发失败时用它顶住，避免角度突变 */
extern uint32_t g_last_angle_raw;

/**
 * @brief 读单个寄存器字节
 * @param reg_addr 寄存器地址
 * @param data     读回的字节
 */
hpm_stat_t mt6835_read_byte(uint16_t reg_addr, uint8_t *data);

/**
 * @brief 单次连读 0x003/0x004/0x005 三个角度字节（同一时刻锁存，约 5us @8MHz）
 * @param b 3 字节输出缓冲
 * @return status_success 成功；status_fail 失败（无应答或 MISO 高阻）
 */
hpm_stat_t mt6835_read_angle_seq3(uint8_t b[3]);

/**
 * @brief 上电自检：判断芯片是否支持"普通读命令 + 地址自增"
 *
 * 必须在电机出力之前、轴静止时调用。自检通过则后续走 mt6835_read_angle_seq3()，
 * 否则回退到三次单字节读。结果写入 g_seq3_ok，重试次数写入 g_seq3_retry。
 */
void mt6835_seq3_init(void);

#endif /* MT6835_H */
