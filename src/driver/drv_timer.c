/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "drv_timer.h"
#include "board.h"
#ifdef HPMSOC_HAS_HPMSDK_GPTMRV2
#include "hpm_gptmrv2_drv.h"
#else
#include "hpm_gptmr_drv.h"
#endif

/* 滴答回调：由上层（control 层）注册，避免 driver 反向依赖 motor */
static void (*s_tick_callback)(void) = NULL;

void drv_timer_register_tick_callback(void (*cb)(void))
{
    s_tick_callback = cb;
}

SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_TMR_IRQ, isr_gptmr)
void isr_gptmr(void)
{
    if (gptmr_check_status(BOARD_BLDC_TMR_1MS, GPTMR_CH_CMP_IRQ_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP))) {
        gptmr_clear_status(BOARD_BLDC_TMR_1MS, GPTMR_CH_CMP_IRQ_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP));
        if (s_tick_callback != NULL) {
            s_tick_callback();
        }
    }
}

void drv_timer_init(void)
{
    gptmr_channel_config_t config;

    clock_add_to_group(BOARD_BLDC_TMR_CLOCK, 0);
    gptmr_channel_get_default_config(BOARD_BLDC_TMR_1MS, &config);
    config.debug_mode = 0;
    config.reload = BOARD_BLDC_TMR_RELOAD + 1;
    config.cmp[0] = BOARD_BLDC_TMR_RELOAD;
    config.cmp[1] = config.reload;
    gptmr_enable_irq(BOARD_BLDC_TMR_1MS, GPTMR_CH_CMP_IRQ_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP));
    gptmr_channel_config(BOARD_BLDC_TMR_1MS, BOARD_BLDC_TMR_CH, &config, true);
    intc_m_enable_irq_with_priority(BOARD_BLDC_TMR_IRQ, 1);
}
