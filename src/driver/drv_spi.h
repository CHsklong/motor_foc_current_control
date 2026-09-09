/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef HPM_DRV_SPI_H
#define HPM_DRV_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "hpm_spi_drv.h"

/**
 * @brief 外设驱动层 —— SPI（供 MT6835 绝对值编码器使用）
 *
 * 本层只负责 SPI 外设本身（格式、波特率、传输控制字），
 * 不理解任何编码器协议 —— 协议在 sensor/mt6835.c。
 */

#define DRV_SPI_ENCODER_BASE        HPM_SPI1
#define DRV_SPI_ENCODER_CLOCK       clock_spi1

/* MT6835 手册支持最高 16MHz。8MHz 是信号完整性与速度的折中：
   单次连读 5 字节约 5us，若线路较长/波形变差可降到 4MHz（约 10us，仍在中断预算内）。 */
#define DRV_SPI_BAUDRATE_HZ         (8000000U)

/**
 * @brief 初始化 SPI 外设
 */
void drv_spi_init(void);

/**
 * @brief 获取传输控制配置（供上层发起 spi_transfer 使用）
 * @return 控制配置结构体指针
 */
spi_control_config_t *drv_spi_get_ctrl(void);

#endif /* HPM_DRV_SPI_H */
