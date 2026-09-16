/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file drv_spi.h
 * @brief SPI1 主机驱动（MT6835 磁编码器通信用）
 */
#ifndef DRV_SPI_H
#define DRV_SPI_H

#include "hw_map.h"

/** 固定传输控制配置，由 spi1_config() 填好，后续所有传输复用 */
extern spi_control_config_t g_spi_ctrl_cfg;

/**
 * @brief 配置 SPI1：模式 3（CPOL=1/CPHA=1）、8MHz、MSB 优先、CS0
 */
void spi1_config(void);

#endif /* DRV_SPI_H */
