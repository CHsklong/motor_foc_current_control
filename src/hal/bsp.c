/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "bsp.h"
#include "board.h"
#include "clock.h"

void bsp_init(void)
{
    board_init();
}

void bsp_encoder_interface_init(void)
{
    init_spi1_pins();     /* SPI1 引脚复用（SCK/MISO/MOSI/CS） */
    init_spi1_clock();    /* SPI1 时钟使能 */
}

void bsp_adc_pins_init(void)
{
    init_adc_bldc_pins();
}

void bsp_pwm_pins_init(void)
{
    init_pwm_pins(MOTOR0_BLDCPWM);
}

void bsp_acmp_pins_init(void)
{
    init_acmp_pins();
}

void bsp_delay_ms(uint32_t ms)
{
    board_delay_ms(ms);
}

void bsp_delay_us(uint32_t us)
{
    board_delay_us(us);
}
