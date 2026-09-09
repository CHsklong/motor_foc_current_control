/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "app_monitor.h"
#include "motor.h"
#include "drv_adc.h"
#include "sens_analog.h"

volatile float g_ia = 0.0f;
volatile float g_ib = 0.0f;
volatile float g_ic = 0.0f;
volatile float g_vbus = 0.0f;
volatile float g_raw_u = 0.0f;
volatile float g_raw_v = 0.0f;
volatile float g_theta_initial = 0.0f;

/*
 * ⚠️ 不要删除这个变量。
 *
 * 下面这些探针（g_ref_q / g_sens_q / g_theta_e ...）定义在 hpm_mcl_loop.c，
 * 在本工程中并没有被真正"使用"，链接器 --gc-sections 会把它们从 elf 中删掉，
 * 导致 J-Scope 找不到符号地址。这里做一次求和引用把它们"锚定"住。
 */
volatile float g_probe_keep = 0.0f;

void app_monitor_update(void)
{
    float ia, ib;

    g_probe_keep = g_ref_q + g_sens_q + g_ref_d + g_sens_d + g_ud + g_uq + g_theta_e + g_theta_initial;

    g_raw_u = (float)drv_adc_read_phase_raw(drv_adc_chn_u);
    g_raw_v = (float)drv_adc_read_phase_raw(drv_adc_chn_v);

    if (motor_get_phase_current(&ia, &ib)) {
        g_ia = ia;
        g_ib = ib;
        g_ic = -g_ia - g_ib;
    }

    g_vbus = sens_vbus_read();
}
