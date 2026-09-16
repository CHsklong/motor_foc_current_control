/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_spi.h"

spi_control_config_t g_spi_ctrl_cfg;

void spi1_config(void)
{
    /* 1. 格式配置 */
    spi_format_config_t format_cfg;
    spi_master_get_default_format_config(&format_cfg);    /* 预配置 */

    /* MT6835 使用 SPI 模式 3：CPOL=1（空闲高电平）, CPHA=1（在第二个边沿采样） */
    format_cfg.common_config.data_len_in_bits = 8;        /* MT6835 按字节通信 */
    format_cfg.common_config.cpol = spi_sclk_high_idle;   /* CPOL = 1 */
    format_cfg.common_config.cpha = spi_sclk_sampling_even_clk_edges; /* CPHA = 1（偶数边沿采样） */
    format_cfg.common_config.mode = spi_master_mode;
    format_cfg.common_config.lsb = false;                 /* MSB 优先 */
    spi_format_init(HPM_SPI1, &format_cfg);

    /* 2. 时序配置（波特率） */
    spi_timing_config_t timing_cfg;
    spi_master_get_default_timing_config(&timing_cfg);
    timing_cfg.master_config.clk_src_freq_in_hz = clock_get_frequency(clock_spi1);
    /* 线路较差时可降回 4MHz（此时单次连读约 10us，仍在预算内） */
    timing_cfg.master_config.sclk_freq_in_hz = 8000000;   /* 8MHz（手册支持最大 16MHz） */
    spi_master_timing_init(HPM_SPI1, &timing_cfg);

    /* 3. 控制配置（固定，供后续传输使用） */
    spi_master_get_default_control_config(&g_spi_ctrl_cfg);
    g_spi_ctrl_cfg.master_config.cmd_enable = false;
    g_spi_ctrl_cfg.master_config.addr_enable = false;
    g_spi_ctrl_cfg.common_config.trans_mode = spi_trans_write_read_together;   /* 同时读写 */
    g_spi_ctrl_cfg.common_config.data_phase_fmt = spi_single_io_mode;          /* 四线 SPI */
    g_spi_ctrl_cfg.common_config.dummy_cnt = spi_dummy_count_1;                /* 传输前的虚拟字节数 */
    g_spi_ctrl_cfg.common_config.cs_index = spi_cs_0;                          /* 明确使用 CS0（PA26） */
}
