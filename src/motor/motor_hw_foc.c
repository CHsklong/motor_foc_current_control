/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "motor.h"
#include "motor_hw_foc.h"
#include "drv_adc.h"

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
/**
 * @brief Convert floating point d/q values to hardware format for CLC module
 *
 * Input range: ±15.0 (float) -> 0x80000000~0x7FFFFFFF (signed 32-bit as uint32_t)
 * Note: 15.0 is a scaling factor, calibrate it for your system.
 */
void clc_convert_input(float d, float q, uint32_t *d_hardware, uint32_t *q_hardware)
{
    *d_hardware = (uint32_t)(int32_t)((d / 15.0f) * 2147483647.0f);
    *q_hardware = (uint32_t)(int32_t)((q / 15.0f) * 2147483647.0f);
}

/**
 * @brief Convert hardware format values back to floating point d/q values
 *
 * Output range: ±7.0 (float)。与输入的 15.0 不同，为系统增益与饱和限制考虑。
 */
void clc_convert_output(uint32_t ud_hardware, uint32_t uq_hardware, float *ud, float *uq)
{
    *ud = ((float)(int32_t)ud_hardware / 2147483647) * 7.0f;
    *uq = ((float)(int32_t)uq_hardware / 2147483647) * 7.0f;
}
#endif

#if defined(HW_CURRENT_FOC_ENABLE)
void trigmux_init_1(void)
{
    trgm_output_t trgm_config;

    /* pwm trigout0 trig adc and vsc */
    trgm_config.invert = false;
    trgm_config.type = trgm_output_same_as_input;
    trgm_config.input = BOARD_BLDC_PWM_TRG_ADC;
    trgm_output_config(HPM_TRGM0, BOARD_BLDC_TRG_ADC, &trgm_config);
    trgm_output_config(HPM_TRGM0, BOARD_BLDC_TRG_VSC, &trgm_config);
}

void trigmux_init_2(void)
{
    /* vsc adc data from adc */
    trgm_adc_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_ADC_MATRIX_TO_VSC_ADC0, BOARD_BLDC_TRGM_ADC_MATRIX_FROM_ADC_U, false);
    trgm_adc_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_ADC_MATRIX_TO_VSC_ADC1, BOARD_BLDC_TRGM_ADC_MATRIX_FROM_ADC_V, false);
    trgm_adc_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_ADC_MATRIX_TO_VSC_ADC2, BOARD_BLDC_TRGM_ADC_MATRIX_FROM_ADC_W, false);

    /* clc id/iq data from vsc */
    trgm_adc_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_ADC_MATRIX_TO_CLC_ID_ADC, BOARD_BLDC_TRGM_ADC_MATRIX_FROM_VSC_ID_ADC, false);
    trgm_adc_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_ADC_MATRIX_TO_CLC_IQ_ADC, BOARD_BLDC_TRGM_ADC_MATRIX_FROM_VSC_IQ_ADC, false);

    /* qeo vd/vq data from clc */
    trgm_dac_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_DAC_MATRIX_TO_QEO_VD_DAC, BOARD_BLDC_TRGM_DAC_MATRIX_FROM_CLC_VD_DAC, false);
    trgm_dac_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_DAC_MATRIX_TO_QEO_VQ_DAC, BOARD_BLDC_TRGM_DAC_MATRIX_FROM_CLC_VQ_DAC, false);

    /* pwm duty data from qeo */
    trgm_dac_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_DAC_MATRIX_TO_PWM_DAC0, BOARD_BLDC_TRGM_DAC_MATRIX_FROM_QEO_DAC0, false);
    trgm_dac_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_DAC_MATRIX_TO_PWM_DAC1, BOARD_BLDC_TRGM_DAC_MATRIX_FROM_QEO_DAC1, false);
    trgm_dac_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_DAC_MATRIX_TO_PWM_DAC2, BOARD_BLDC_TRGM_DAC_MATRIX_FROM_QEO_DAC2, false);
}

void trigmux_init_3(void)
{
    /* pos data from qei */
    trgm_pos_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_POS_MATRIX_TO_VSC, BOARD_BLDC_TRGM_POS_MATRIX_FROM_QEI, true);
    trgm_pos_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_POS_MATRIX_TO_QEO, BOARD_BLDC_TRGM_POS_MATRIX_FROM_QEI, true);
}

void vsc_init(void)
{
    vsc_config_t vsc_config;

    vsc_get_default_config(BOARD_VSC, &vsc_config);
    vsc_config.phase_mode = vsc_ab_phase;
    vsc_config.pole_pairs = motor0.cfg.mcl.physical.motor.pole_num;
    vsc_config.a_adc_config.adc_sel = vsc_sel_adc0;
    vsc_config.b_adc_config.adc_sel = vsc_sel_adc1;
    vsc_config.a_adc_config.adc_chn = BOARD_BLDC_ADC_CH_U;
    vsc_config.b_adc_config.adc_chn = BOARD_BLDC_ADC_CH_V;

    /* adc software use 12bit, so need shift more four bit */
    vsc_config.a_adc_config.adc_offset = (adc_u_midpoint) << (16 + 4);
    vsc_config.b_adc_config.adc_offset = (adc_v_midpoint) << (16 + 4);

    vsc_config.a_data_cnt = 1;
    vsc_config.b_data_cnt = 1;

    vsc_config_init(BOARD_VSC, &vsc_config);
    vsc_set_enable(BOARD_VSC, true);
}

void clc_init(void)
{
    clc_param_config_t clc_param;
    clc_coeff_config_t clc_coeff0;
    mcl_control_pid_cfg_t pid;

    clc_param.eadc_lowth = 0xA0000000;
    clc_param.eadc_mid_lowth = 0xD0000000;
    clc_param.eadc_mid_highth = 0x30000000;
    clc_param.eadc_highth = 0x60000000;
    clc_param._2p2z_clamp_lowth = 0x80000000;
    clc_param._2p2z_clamp_highth = 0x7FFFFFFF;
    clc_param._3p3z_clamp_lowth = 0x80000000;
    clc_param._3p3z_clamp_highth = 0x7FFFFFFF;
    clc_param.output_forbid_lowth = 0;
    clc_param.output_forbid_mid = 0;
    clc_param.output_forbid_highth = 0;
    clc_config_param(BOARD_CLC, clc_vd_chn, &clc_param);
    clc_config_param(BOARD_CLC, clc_vq_chn, &clc_param);

    /**
     * @brief Value of incremental pid, this value is different from the positional pid
     */
    pid.kp = 1.94678;
    pid.ki = 0.000081414;
    pid.kd = 0;

    hpm_mcl_pid_to_3p3z(&pid, (mcl_clc_coeff_cfg_t *)&clc_coeff0);
    clc_config_coeff(BOARD_CLC, clc_vd_chn, clc_coeff_zone_0, &clc_coeff0);
    clc_config_coeff(BOARD_CLC, clc_vq_chn, clc_coeff_zone_0, &clc_coeff0);

    clc_config_coeff(BOARD_CLC, clc_vd_chn, clc_coeff_zone_1, &clc_coeff0);
    clc_config_coeff(BOARD_CLC, clc_vq_chn, clc_coeff_zone_1, &clc_coeff0);

    clc_config_coeff(BOARD_CLC, clc_vd_chn, clc_coeff_zone_2, &clc_coeff0);
    clc_config_coeff(BOARD_CLC, clc_vq_chn, clc_coeff_zone_2, &clc_coeff0);

    clc_set_adc_chn_offset(BOARD_CLC, clc_vd_chn, 0, 0);
    clc_set_adc_chn_offset(BOARD_CLC, clc_vq_chn, 0, 0);
    clc_set_pwm_period(BOARD_CLC, clc_vd_chn, 0);
    clc_set_pwm_period(BOARD_CLC, clc_vq_chn, 0);

    clc_set_expect_adc_value(BOARD_CLC, clc_vd_chn, 0);
    clc_set_expect_adc_value(BOARD_CLC, clc_vq_chn, 0);

    clc_set_enable(BOARD_CLC, clc_vd_chn, true);
    clc_set_enable(BOARD_CLC, clc_vq_chn, true);
}

void qeov2_init(void)
{
    qeo_wave_mode_t config;
    qeo_wave_get_default_mode_config(BOARD_BLDC_QEO, &config);
    config.dq_valid_trig_enable = true;
    config.pos_valid_trig_enable = true;
    config.vd_vq_inject_enable = true;
    config.vd_vq_from_sw = false;
    config.wave_type = qeo_wave_saddle;
    config.saddle_type = qeo_saddle_standard;
    qeo_wave_config_mode(BOARD_BLDC_QEO, &config);
    qeo_wave_set_resolution_lines(BOARD_BLDC_QEO, motor0.cfg.mcl.physical.motor.pole_num);

    qeo_wave_set_phase_shift(BOARD_BLDC_QEO, 0, 180.0);
    qeo_wave_set_phase_shift(BOARD_BLDC_QEO, 1, 60.0);
    qeo_wave_set_phase_shift(BOARD_BLDC_QEO, 2, 300.0);

    qeo_wave_set_pwm_cycle(BOARD_BLDC_QEO, (PWM_RELOAD << 8));
}

void motor0_clc_set_currentloop_value(mcl_loop_chn_t chn, int32_t val)
{
    const clc_chn_t clc_chn[2] = {clc_vd_chn, clc_vq_chn};

    clc_set_expect_adc_value(BOARD_CLC, clc_chn[chn], val);
}

int32_t motor0_clc_float_convert_clc(float realdata)
{
    int32_t data0;
    double data1 = realdata;
    data0 = data1 * 0x6400000;

    return data0;
}
#endif
