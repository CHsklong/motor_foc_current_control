/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file drv_qei.h
 * @brief QEI 正交编码器接口驱动
 *
 * 注意：当前工程用 SPI 接口的 MT6835 绝对编码器（见 sensor/mt6835.c），
 * QEI 未作为角度源使用；本模块保留给硬件 FOC 路径与备用板卡。
 */
#ifndef DRV_QEI_H
#define DRV_QEI_H

#include "hw_map.h"

#if defined(HPMSOC_HAS_HPMSDK_QEI)
#define BLDC_MOTOR_QEI_BASE   BOARD_BLDC_QEI_BASE
#endif
#if defined(HPMSOC_HAS_HPMSDK_QEIV2)
#define BLDC_MOTOR_QEI_BASE   BOARD_BLDC_QEIV2_BASE
#endif

/** QEI 初始化：引脚 TRGM 路由 + ABZ 工作模式 */
hpm_mcl_stat_t qei_init(void);

#endif /* DRV_QEI_H */
