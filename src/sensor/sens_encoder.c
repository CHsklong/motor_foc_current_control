/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "dbg_probe.h"
#include "mt6835.h"
#include "sens_encoder.h"

volatile uint8_t g_encoder_isr_enable = 0;

/* 多圈展开状态（模块内私有） */
static float g_abs_theta     = 0.0f;
static float g_abs_theta_prv = 0.0f;
static bool  g_abs_theta_rdy = false;

/* 虚拟角度：USE_VIRTUAL_ANGLE=1 时用固定频率递增的角度代替真实编码器，
   用于在不接电机的情况下验证电流环/SVPWM 链路 */
#define VIRTUAL_FREQ_HZ  10.0f

hpm_mcl_stat_t encoder_start_sample(void)
{
    return mcl_success;
}

hpm_mcl_stat_t encoder_get_theta(float *theta)
{
#if USE_VIRTUAL_ANGLE
    static float step = 2.0f * MCL_PI * VIRTUAL_FREQ_HZ / PWM_FREQUENCY;
    static float virtual_theta = 0.0f;

    virtual_theta += step;
    if (virtual_theta >= 2.0f * MCL_PI) {
        virtual_theta -= 2.0f * MCL_PI;
    }
    *theta = -virtual_theta;
    if (*theta < 0) *theta += 2.0f * MCL_PI;
    theta_r = *theta;
    return mcl_success;
#else
    uint8_t rx[3] = {0};
    uint32_t angle_raw;
    bool read_ok = true;

    if (g_seq3_ok) 
    {
        /* 单次连读：一次 CS 拿到同一时刻锁存的 3 个字节（约 5us @8MHz） */
        if (mt6835_read_angle_seq3(rx) != status_success)
        {
            read_ok = false;
        }
    } 
    else 
    {
        /* 回退：三次独立单字节读，每次一次独立 CS */
        if (mt6835_read_byte(MT6835_REG_ANGLE_MSB, &rx[0]) != status_success) 
        {
            read_ok = false;
        } else if (mt6835_read_byte(MT6835_REG_ANGLE_MID, &rx[1]) != status_success) 
        {
            read_ok = false;
        } else if (mt6835_read_byte(MT6835_REG_ANGLE_LSB, &rx[2]) != status_success) 
        {
            read_ok = false;
        }
    }

    if (!read_ok) 
    {
        /* 偶发 SPI 失败不要直接返回 mcl_fail：
           hpm_mcl_encoder_process() 一旦收到失败会把 encoder->status 永久置成 fail，
           之后每次调用都在入口 MCL_ASSERT 直接返回 not_ready，角度再也不更新、电机锁死。
           这里用上一帧角度顶住，连续失败 5 次才真正上报。 */
        if (++g_spi_fail_cnt > 5) 
        {
            return mcl_fail;
        }
        angle_raw = g_last_angle_raw;
    } 
    else 
    {
        g_spi_fail_cnt = 0;
        g_byte0 = rx[0];
        g_byte1 = rx[1];
        g_byte2 = rx[2];
        angle_raw = ((uint32_t)rx[0] << 13) | ((uint32_t)rx[1] << 5) | (rx[2] >> 3);
        g_last_angle_raw = angle_raw;
    }
    g_angle_raw = angle_raw;

    /* MT6835 为 21 位分辨率：ANGLE[20:0]，满量程 = 2^21 = 2097152（数据手册 7.6.8）
       注意：angle_raw 已右移 3 位得到 21bit，这里必须除 2^21，
       除 2^23 会让机械角只有真实的 1/4 */
    float theta_raw = (float)angle_raw * 2.0f * MCL_PI / 2097152.0f;
    /* 极性反转：顺时针旋转 → 角度递增 */
    *theta = -theta_raw;
    if (*theta < 0) *theta += 2.0f * MCL_PI;
    theta_r = *theta;
    return mcl_success;
#endif
}

void encoder_abs_rebase(void)
{
    g_abs_theta     = 0.0f;
    g_abs_theta_rdy = false;    /* 下一拍 get_abs_theta 会把当前角作为新的 0 基准 */
    g_pos_abs       = 0.0f;     /* 反馈同步清零：main 里读它做轨迹起点时不必再碰 SPI */
}

hpm_mcl_stat_t encoder_get_abs_theta(float *theta)
{
    float th;
    float d;

    if (encoder_get_theta(&th) != mcl_success)
    {
        return mcl_fail;
    }
    if (!g_abs_theta_rdy) 
    {
        g_abs_theta_prv = th;
        g_abs_theta_rdy = true;
    }
    d = th - g_abs_theta_prv;
    if (d > MCL_PI) 
    {
        d -= 2.0f * MCL_PI;
    } 
    else if (d < -MCL_PI) 
    {
        d += 2.0f * MCL_PI;
    }

    /* 抗撕裂：单拍位移超过物理上限就是脏帧，丢弃并沿用上一次的角度。
       见 sens_encoder.h 里 ENC_MAX_STEP_RAD 的说明。 */
    if ((d > ENC_MAX_STEP_RAD) || (d < -ENC_MAX_STEP_RAD))
    {
        g_angle_jump++;
        *theta = g_abs_theta;
        return mcl_success;
    }

    g_abs_theta += d;
    g_abs_theta_prv = th;
    g_pos_abs = g_abs_theta;
    *theta = g_abs_theta;
    return mcl_success;
}
