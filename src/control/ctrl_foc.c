/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "ctrl_foc.h"
#include "ctrl_svpwm.h"
#include "motor.h"
#include "drv_adc.h"
#include "board.h"
#include "hpm_adc_v2.h"

static volatile bool s_encoder_update_enable = false;

void ctrl_foc_encoder_update_enable(bool enable)
{
    s_encoder_update_enable = enable;
}

void ctrl_foc_init(void)
{
    drv_adc_isr_enable();
}

SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_ADC_IRQn, isr_adc)
void isr_adc(void)
{
    uint32_t status;
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);

    status = hpm_adc_v2_get_status_flags(adc_u);
    if ((status & HPM_ADC_V2_EVENT_TRIG_COMPLETE) != 0) {
        hpm_adc_v2_clear_status_flags(adc_u, HPM_ADC_V2_EVENT_TRIG_COMPLETE);

        /* 编码器角度必须和电流环同频(20kHz)更新。
           若放在主循环里只有 ~1kHz，而 tick 仍按 50us 传，速度会被放大约 20 倍，
           导致预测角与 dq 解耦前馈 uq += ω·pole_num·(Ld·iq+flux) 严重失真。 */
        if (s_encoder_update_enable) {
            motor_encoder_process(motor_get_loop_tick());
        }

#if CTRL_FOC_SVPWM_MODE
        ctrl_svpwm_step();
#endif

#if CTRL_FOC_CURRENT_MODE
        hpm_mcl_loop(motor_get_loop());
#endif
    }
}
