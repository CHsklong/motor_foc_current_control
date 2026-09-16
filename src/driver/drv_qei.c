/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_qei.h"

#if defined(HPMSOC_HAS_HPMSDK_QEI)
hpm_mcl_stat_t qei_init(void)
{
    trgm_output_t trgm_config = {0};
    qei_mode_config_t mode_config = {0};

    init_qei_trgm_pins();

    trgm_config.invert = false;
    trgm_config.input = BOARD_BLDC_QEI_TRGM_QEI_A_SRC;
    trgm_output_config(BOARD_BLDC_QEI_TRGM, TRGM_TRGOCFG_QEI_A, &trgm_config);
    trgm_config.input = BOARD_BLDC_QEI_TRGM_QEI_B_SRC;
    trgm_output_config(BOARD_BLDC_QEI_TRGM, TRGM_TRGOCFG_QEI_B, &trgm_config);

    mode_config.work_mode = qei_work_mode_abz;
    mode_config.z_count_inc_mode = qei_z_count_inc_on_phase_count_max;
    mode_config.phcnt_max = BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV;
    mode_config.z_cali_enable = false;
    mode_config.phcnt_idx = 0;
    qei_config_mode(BLDC_MOTOR_QEI_BASE, &mode_config);

    return mcl_success;
}
#endif

#if defined(HPMSOC_HAS_HPMSDK_QEIV2)
hpm_mcl_stat_t qei_init(void)
{
    qeiv2_mode_config_t mode_config = {0};

    init_qei_trgm_pins();

    qeiv2_config_filter(BLDC_MOTOR_QEI_BASE, qeiv2_filter_phase_a, false, qeiv2_filter_mode_delay, true, 100);
    qeiv2_config_filter(BLDC_MOTOR_QEI_BASE, qeiv2_filter_phase_b, false, qeiv2_filter_mode_delay, true, 100);

    mode_config.work_mode = qeiv2_work_mode_abz;
    mode_config.spd_tmr_content_sel = qeiv2_spd_tmr_as_spd_tm;
    mode_config.z_count_inc_mode = qeiv2_z_count_inc_on_phase_count_max;
    mode_config.phcnt_max = BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV;
    mode_config.z_cali_enable = false;
    mode_config.z_cali_ignore_ab = false;
    mode_config.phcnt_idx = 0;
    qeiv2_config_mode(BLDC_MOTOR_QEI_BASE, &mode_config);

    return mcl_success;
}
#endif
