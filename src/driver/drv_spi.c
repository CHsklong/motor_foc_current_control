/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "drv_spi.h"
#include "hpm_clock_drv.h"

static spi_control_config_t s_spi_ctrl_cfg;

void drv_spi_init(void)
{
    /* 1. 格式配置
     * MT6835 使用 SPI 模式 3：CPOL=1（空闲高电平）, CPHA=1（在第二个边沿采样） */
    spi_format_config_t format_cfg;
    spi_master_get_default_format_config(&format_cfg);

    format_cfg.common_config.data_len_in_bits = 8;                       /* MT6835 按字节通信 */
    format_cfg.common_config.cpol = spi_sclk_high_idle;                  /* CPOL = 1 */
    format_cfg.common_config.cpha = spi_sclk_sampling_even_clk_edges;    /* CPHA = 1（偶数边沿采样） */
    format_cfg.common_config.mode = spi_master_mode;
    format_cfg.common_config.lsb = false;                                /* MSB 优先 */
    spi_format_init(DRV_SPI_ENCODER_BASE, &format_cfg);

    /* 2. 时序配置（波特率） */
    spi_timing_config_t timing_cfg;
    spi_master_get_default_timing_config(&timing_cfg);
    timing_cfg.master_config.clk_src_freq_in_hz = clock_get_frequency(DRV_SPI_ENCODER_CLOCK);
    timing_cfg.master_config.sclk_freq_in_hz = DRV_SPI_BAUDRATE_HZ;
    spi_master_timing_init(DRV_SPI_ENCODER_BASE, &timing_cfg);

    /* 3. 控制配置（固定，供后续每次传输复用） */
    spi_master_get_default_control_config(&s_spi_ctrl_cfg);
    s_spi_ctrl_cfg.master_config.cmd_enable = false;
    s_spi_ctrl_cfg.master_config.addr_enable = false;
    s_spi_ctrl_cfg.common_config.trans_mode = spi_trans_write_read_together;   /* 同时读写 */
    s_spi_ctrl_cfg.common_config.data_phase_fmt = spi_single_io_mode;          /* 四线 SPI */
    s_spi_ctrl_cfg.common_config.dummy_cnt = spi_dummy_count_1;                /* 传输前虚拟字节数 */
    s_spi_ctrl_cfg.common_config.cs_index = spi_cs_0;                          /* CS0 */
}

spi_control_config_t *drv_spi_get_ctrl(void)
{
    return &s_spi_ctrl_cfg;
}
