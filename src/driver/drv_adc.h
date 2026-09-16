/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file drv_adc.h
 * @brief ADC 驱动：相电流/母线电压采样、DMA 缓冲、静态零点校准
 */
#ifndef DRV_ADC_H
#define DRV_ADC_H

#include "hw_map.h"

/** ADC 抢占模式 DMA 缓冲区：[0]=U 相(ADC1) [1]=V 相(ADC0) */
extern volatile uint32_t adc_buff[3][BOARD_BLDC_ADC_PMT_DMA_SIZE_IN_4BYTES];

/** 静态零点校准值（上电时电机静止采样 200 次取平均） */
extern uint32_t adc_u_midpoint;
extern uint32_t adc_v_midpoint;

/**
 * @brief ADC 模块初始化：分辨率/转换模式/通道/触发/DMA
 */
hpm_mcl_stat_t adc_init(void);

/**
 * @brief 使能 ADC 转换完成中断（20kHz 电流环节拍源）
 */
void adc_isr_enable(void);

/**
 * @brief 静态零点校准
 *
 * 必须在 PWM 无输出、电机静止时调用；连续采样 CURRENT_SET_TIME_MS 次取平均。
 */
void motor_adc_midpoint(void);

/**
 * @brief 把 PWM 触发信号路由到 ADC 模块
 */
void init_trigger_mux(TRGM_Type *ptr);

/**
 * @brief 配置 ADC 抢占模式触发通道（U 相产生中断，V 相与母线不产生）
 */
void init_trigger_cfg(void);

#endif /* DRV_ADC_H */
