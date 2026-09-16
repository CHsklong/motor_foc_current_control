/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "hal_bsp.h"

void bsp_init(void)
{
    board_init();                     /* 开发板基础初始化（时钟/console） */
    init_spi1_pins();                 /* 编码器 SPI1 引脚 */
    init_spi1_clock();                /* 编码器 SPI1 时钟 */
    init_adc_bldc_pins();             /* 相电流采样 ADC 引脚 */
    init_pwm_pins(MOTOR0_BLDCPWM);    /* 三相 PWM 输出引脚 */
}

void mcl_user_delay_us(uint64_t tick)
{
    board_delay_us(tick);
}
