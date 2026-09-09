/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_DRV_ADC_H
#define HPM_DRV_ADC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 外设驱动层 —— ADC（相电流 + 母线电压）
 *
 * 本层负责 ADC 模块、通道、触发矩阵、抢占队列与 DMA 搬运，以及"取 12bit 有效值"。
 * 电流/电压的物理换算、零漂校准属于 sensor 层，不在这里。
 */

typedef enum {
    drv_adc_chn_u = 0,   /* ADC 模块 U（硬件实际采 A 相电流） */
    drv_adc_chn_v = 1,   /* ADC 模块 V（硬件实际采 C 相电流） */
} drv_adc_chn_t;

/* 从 ADC 结果字中取 12bit 有效数据（与 MCL_GET_ADC_12BIT_VALID_DATA 等价） */
#define DRV_ADC_GET_12BIT(x)        (((x) & 0xffffU) >> 4)

/**
 * @brief 初始化 ADC（模块/通道/触发矩阵/抢占队列/DMA）
 */
void drv_adc_init(void);

/**
 * @brief 使能 ADC 转换完成中断（相电流采样完成触发中断，用于驱动电流环）
 */
void drv_adc_isr_enable(void);

/**
 * @brief 读取相电流通道的原始 12bit 值
 */
uint16_t drv_adc_read_phase_raw(drv_adc_chn_t chn);

/**
 * @brief 软件触发一次母线电压采样并读取
 * @param[out] raw 12bit 原始值
 * @return true 成功；false 触发通道/ADC 通道布局校验失败
 */
bool drv_adc_read_vbus_raw(uint16_t *raw);

#endif /* HPM_DRV_ADC_H */
