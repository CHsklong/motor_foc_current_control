/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_pwm.h"

volatile ATTR_PLACE_AT_FAST_RAM_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t pwm_buff[6];
int32_t motor_clock_hz;

hpm_mcl_stat_t pwm_duty_set(mcl_drivers_channel_t chn, float duty)
{
#if !defined(HW_CURRENT_FOC_ENABLE)     /* 硬件 FOC 时占空比由硬件闭环接管*/
    uint32_t pwm_reload;
    uint32_t pwm_cmp_half, pwm_reload_half;
    uint32_t index0, index1;

    pwm_reload = (int32_t)(PWM_RELOAD * 0.98f);
    pwm_cmp_half = (uint32_t)(duty * pwm_reload) >> 1;
    pwm_reload_half =  PWM_RELOAD >> 1;
    switch (chn) {
    case mcl_drivers_chn_a:
            index0 = BOARD_BLDCPWM_CMP_INDEX_0;
            index1 = BOARD_BLDCPWM_CMP_INDEX_1;
        break;
    case mcl_drivers_chn_b:
            index0 = BOARD_BLDCPWM_CMP_INDEX_2;
            index1 = BOARD_BLDCPWM_CMP_INDEX_3;
        break;
    case mcl_drivers_chn_c:
            index0 = BOARD_BLDCPWM_CMP_INDEX_4;
            index1 = BOARD_BLDCPWM_CMP_INDEX_5;
        break;
    default:
        return mcl_fail;
    }
#if defined(HPMSOC_HAS_HPMSDK_PWM)
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
    pwm_buff[index0] = PWM_CMP_CMP_SET((pwm_reload_half - pwm_cmp_half));
    pwm_buff[index1] = PWM_CMP_CMP_SET((pwm_reload_half + pwm_cmp_half));
#else
    pwm_cmp_force_value(MOTOR0_BLDCPWM, index0, PWM_CMP_CMP_SET((pwm_reload_half - pwm_cmp_half)));
    pwm_cmp_force_value(MOTOR0_BLDCPWM, index1, PWM_CMP_CMP_SET((pwm_reload_half + pwm_cmp_half)));
#endif
#endif
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
    pwmv2_shadow_register_unlock(MOTOR0_BLDCPWM);
    pwmv2_set_shadow_val(MOTOR0_BLDCPWM, (index0 + 1), (pwm_reload_half - pwm_cmp_half), 0, false);
    pwmv2_set_shadow_val(MOTOR0_BLDCPWM, (index1 + 1), (pwm_reload_half + pwm_cmp_half), 0, false);
    pwmv2_shadow_register_lock(MOTOR0_BLDCPWM);
#endif
#else
    (void)chn;
    (void)duty;
#endif
    return mcl_success;
}

/** PWM 输出使能状态：1=正常调制，0=被强制关断 */
static volatile uint8_t g_pwm_out_enabled = 0;

hpm_mcl_stat_t enable_all_pwm_output(void)
{
    g_pwm_out_enabled = 1;
#if defined(HPMSOC_HAS_HPMSDK_PWM)
    pwm_disable_sw_force(MOTOR0_BLDCPWM);
#endif
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
    pwmv2_disable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwmv2_disable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwmv2_disable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwmv2_disable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwmv2_disable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwmv2_disable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN);
#endif
    return mcl_success;
}

hpm_mcl_stat_t disable_all_pwm_output(void)
{
    g_pwm_out_enabled = 0;
#if defined(HPMSOC_HAS_HPMSDK_PWM)
    pwm_config_force_cmd_timing(MOTOR0_BLDCPWM, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN);
    pwm_set_force_output(MOTOR0_BLDCPWM,
                        PWM_FORCE_OUTPUT(BOARD_BLDC_UH_PWM_OUTPIN, pwm_output_0)
                        | PWM_FORCE_OUTPUT(BOARD_BLDC_UL_PWM_OUTPIN, pwm_output_0)
                        | PWM_FORCE_OUTPUT(BOARD_BLDC_VH_PWM_OUTPIN, pwm_output_0)
                        | PWM_FORCE_OUTPUT(BOARD_BLDC_VL_PWM_OUTPIN, pwm_output_0)
                        | PWM_FORCE_OUTPUT(BOARD_BLDC_WH_PWM_OUTPIN, pwm_output_0)
                        | PWM_FORCE_OUTPUT(BOARD_BLDC_WL_PWM_OUTPIN, pwm_output_0));
    pwm_enable_sw_force(MOTOR0_BLDCPWM);
#endif
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
    pwmv2_set_force_update_time(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(MOTOR0_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(MOTOR0_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, pwm_force_immediately);
    pwmv2_set_force_update_time(MOTOR0_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN, pwm_force_immediately);

    pwmv2_force_update_time_by_shadow(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(MOTOR0_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(MOTOR0_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, pwm_force_update_shadow_immediately);
    pwmv2_force_update_time_by_shadow(MOTOR0_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN, pwm_force_update_shadow_immediately);

    pwmv2_enable_force_by_software(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwmv2_enable_force_by_software(MOTOR0_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwmv2_enable_force_by_software(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwmv2_enable_force_by_software(MOTOR0_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwmv2_enable_force_by_software(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwmv2_enable_force_by_software(MOTOR0_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN);

    pwmv2_enable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwmv2_enable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwmv2_enable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwmv2_enable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwmv2_enable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwmv2_enable_software_force(MOTOR0_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN);

    pwmv2_shadow_register_unlock(MOTOR0_BLDCPWM);
    pwmv2_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_force_output(MOTOR0_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN, pwm_force_output_1, false);
    pwmv2_shadow_register_lock(MOTOR0_BLDCPWM);
#endif
    return mcl_success;
}

#if defined(HPMSOC_HAS_HPMSDK_PWM)
/* PWM v1 初始化（三相六路互补 + 死区 + DMA + 硬件过流保护） */
void pwm_init(void)
{
    uint8_t cmp_index = BOARD_BLDCPWM_CMP_INDEX_0;          /* 起始比较器编号 */
    pwm_cmp_config_t cmp_config[4] = {0};                   /* 4 个比较器配置 */
    pwm_pair_config_t pwm_pair_config = {0};                /* 互补 PWM 配置结构体 */
    pwm_output_channel_t pwm_output_ch_cfg = {0};           /* 触发输出通道配置 */
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
    dma_channel_config_t config = {0};                      /* DMA 通道配置 */
    trgm_output_t trgm_output_cfg;                          /* TRGM 输出配置 */
#endif

    /* 1. 复位并停止 PWM 模块 */
    pwm_deinit(MOTOR0_BLDCPWM);
    pwm_stop_counter(MOTOR0_BLDCPWM);
    pwm_enable_reload_at_synci(MOTOR0_BLDCPWM);
    pwm_set_reload(MOTOR0_BLDCPWM, 0, PWM_RELOAD);         /* 设置周期（20kHz） */
    pwm_set_start_count(MOTOR0_BLDCPWM, 0, 0);

    /* 2. 配置比较器 0/1（U 相上下桥），初始值大于重载值，上电时关断 */
    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = PWM_RELOAD + 1;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = PWM_RELOAD + 1;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;

    /* 3. 配置比较器 2（ADC 触发） */
    cmp_config[2].enable_ex_cmp  = false;
    cmp_config[2].mode           = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp = 50;                                  /* PWM 周期起点触发 ADC */
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_shlk;

    /* 4. 配置比较器 3（影子寄存器加载触发） */
    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = PWM_RELOAD;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_modify;

    /* 5. 配置互补输出（死区、使能、故障保护） */
    pwm_get_default_pwm_pair_config(MOTOR0_BLDCPWM, &pwm_pair_config);

    pwm_pair_config.pwm[0].enable_output = true;
    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
    pwm_pair_config.pwm[0].invert_output = false;
    pwm_pair_config.pwm[0].fault_mode             = pwm_fault_mode_force_output_0; /* 故障时强制输出低 */
    pwm_pair_config.pwm[0].fault_recovery_trigger = pwm_fault_recovery_on_fault_clear;

    pwm_pair_config.pwm[1].enable_output = true;
    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
    pwm_pair_config.pwm[1].invert_output = false;
    pwm_pair_config.pwm[1].fault_mode             = pwm_fault_mode_force_output_0;
    pwm_pair_config.pwm[1].fault_recovery_trigger = pwm_fault_recovery_on_fault_clear;

    /* 6. 配置 ADC 触发输出通道 */
    pwm_output_ch_cfg.cmp_start_index = BOARD_BLDC_PWM_TRIG_CMP_INDEX;
    pwm_output_ch_cfg.cmp_end_index   = BOARD_BLDC_PWM_TRIG_CMP_INDEX;
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(MOTOR0_BLDCPWM, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &pwm_output_ch_cfg);

    /* 7. 配置三相六路输出（U/V/W 各一对互补通道） */
    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while(1);
    }
    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, &pwm_pair_config, cmp_index+2, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while(1);
    }
    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, &pwm_pair_config, cmp_index+4, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while(1);
    }
    pwm_load_cmp_shadow_on_match(MOTOR0_BLDCPWM, BOARD_BLDCPWM_CMP_TRIG_CMP, &cmp_config[3]);
    pwm_config_cmp(MOTOR0_BLDCPWM, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &cmp_config[2]);

    /* 8. 配置硬件过流故障源（PA10 = PWM1_FAULT_0）*/
    pwm_fault_source_config_t fault_config = {0};
    fault_config.source_mask              = pwm_fault_source_external_0;  /* 启用外部故障通道 0 */
    fault_config.fault_external_0_active_low = true;                      /* TL331 过流拉低 → 低有效 */
    fault_config.fault_external_1_active_low = false;                     /* 未使用 fault_1 */
    fault_config.fault_recover_at_rising_edge = false;                    /* 禁止硬件自动恢复，需软件清除故障标志 */
    fault_config.fault_output_recovery_trigger = 0;                       /* 恢复触发由通道级配置决定 */
    pwm_config_fault_source(MOTOR0_BLDCPWM, &fault_config);

    /* 9. DMA 相关配置（若启用） */
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
    pwm_output_ch_cfg.cmp_start_index = BOARD_BLDC_DMA_TRG_CMP_INDEX;
    pwm_output_ch_cfg.cmp_end_index   = BOARD_BLDC_DMA_TRG_CMP_INDEX;
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(MOTOR0_BLDCPWM, BOARD_BLDC_DMA_TRG_CMP_INDEX, &pwm_output_ch_cfg);

    cmp_config[2].cmp = PWM_RELOAD * 0.75;   /* 在周期 75% 处触发 DMA */
    pwm_config_cmp(MOTOR0_BLDCPWM, BOARD_BLDC_DMA_TRG_CMP_INDEX, &cmp_config[2]);
#endif

    /* 10. 启动 PWM 计数器，加载影子寄存器 */
    pwm_start_counter(MOTOR0_BLDCPWM);
    pwm_issue_shadow_register_lock_event(MOTOR0_BLDCPWM);

    /* 11. DMA 通道和 TRGM 配置（若启用） */
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_pulse_at_input_both_edge;
    trgm_output_cfg.input  = BOARD_BLDC_DMA_TRG_IN;
    trgm_output_config(BOARD_BLDCPWM_TRGM, BOARD_BLDC_DMA_TRG_DST, &trgm_output_cfg);

    dmamux_config(BOARD_APP_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN),
                BOARD_BLDC_DMA_MUX_SRC, true);

    trgm_dma_request_config(BOARD_BLDCPWM_TRGM, BOARD_BLDC_DMA_TRG_INDEX, BOARD_BLDC_DMA_TRG_SRC);

    dma_default_channel_config(BOARD_APP_DMA0, &config);

    config.src_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&pwm_buff);
    config.dst_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&MOTOR0_BLDCPWM->CMP[BOARD_BLDCPWM_CMP_INDEX_0]);
    config.src_width = DMA_TRANSFER_WIDTH_WORD;
    config.dst_width = DMA_TRANSFER_WIDTH_WORD;
    config.src_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    config.size_in_byte = 24;   /* 6 个 32 位比较寄存器 */
    config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;

    dma_setup_channel(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, &config, true);
    dma_set_infinite_loop_mode(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, true);
    dma_set_handshake_option(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, DMA_HANDSHAKE_OPT_ALL_TRANSIZE);
#endif
}
#endif

#if defined(HPMSOC_HAS_HPMSDK_PWMV2)

#if defined(HW_CURRENT_FOC_ENABLE)
/* PWM v2 初始化：硬件 FOC 路径，比较值来自 CLC 计算结果 */
void pwm_init(void)
{
    pwmv2_cmp_config_t cmp_cfg[2] = {0};
    pwmv2_pair_config_t pwm_cfg = {0};
    pwmv2_cmp_calculate_cfg_t cal = {0};

    pwmv2_get_default_config(&pwm_cfg.pwm[0]);
    pwmv2_get_default_config(&pwm_cfg.pwm[1]);

    cmp_cfg[0].cmp = PWM_RELOAD;
    cmp_cfg[0].enable_half_cmp = false;
    cmp_cfg[0].enable_hrcmp = false;
    cmp_cfg[0].cmp_source = cmp_value_from_calculate;
    cmp_cfg[0].cmp_source_index = PWMV2_CALCULATE_INDEX(0);
    cmp_cfg[0].update_trigger = pwm_shadow_register_update_on_reload;
    cmp_cfg[1].cmp = PWM_RELOAD;
    cmp_cfg[1].enable_half_cmp = false;
    cmp_cfg[1].enable_hrcmp = false;
    cmp_cfg[1].cmp_source = cmp_value_from_calculate;
    cmp_cfg[1].cmp_source_index = PWMV2_CALCULATE_INDEX(1);
    cmp_cfg[1].update_trigger = pwm_shadow_register_update_on_reload;

    pwm_cfg.pwm[0].enable_output = true;
    pwm_cfg.pwm[0].enable_async_fault = false;
    pwm_cfg.pwm[0].enable_sync_fault = false;
    pwm_cfg.pwm[0].invert_output = false;
    pwm_cfg.pwm[0].enable_four_cmp = false;
    pwm_cfg.pwm[0].update_trigger = pwm_reload_update_on_reload;
    pwm_cfg.pwm[0].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
    pwm_cfg.pwm[1].enable_output = true;
    pwm_cfg.pwm[1].enable_async_fault = false;
    pwm_cfg.pwm[1].enable_sync_fault = false;
    pwm_cfg.pwm[1].invert_output = false;
    pwm_cfg.pwm[1].enable_four_cmp = false;
    pwm_cfg.pwm[1].update_trigger = pwm_reload_update_on_reload;
    pwm_cfg.pwm[1].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;

    pwmv2_disable_counter(MOTOR0_BLDCPWM, pwm_counter_0);
    pwmv2_reset_counter(MOTOR0_BLDCPWM, pwm_counter_0);

    pwmv2_shadow_register_unlock(MOTOR0_BLDCPWM);
    pwmv2_set_shadow_val(MOTOR0_BLDCPWM, PWMV2_SHADOW_INDEX(0), PWM_RELOAD, 0, false);
    pwmv2_set_shadow_val(MOTOR0_BLDCPWM, PWMV2_SHADOW_INDEX(1), 1, 0, false);
    pwmv2_shadow_register_lock(MOTOR0_BLDCPWM);

    pwmv2_setup_waveform_in_pair(MOTOR0_BLDCPWM, pwm_channel_0, &pwm_cfg, PWMV2_CMP_INDEX(0), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_CALCULATE_INDEX(2);
    cmp_cfg[1].cmp_source_index = PWMV2_CALCULATE_INDEX(3);
    pwmv2_setup_waveform_in_pair(MOTOR0_BLDCPWM, pwm_channel_2, &pwm_cfg, PWMV2_CMP_INDEX(4), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_CALCULATE_INDEX(4);
    cmp_cfg[1].cmp_source_index = PWMV2_CALCULATE_INDEX(5);
    pwmv2_setup_waveform_in_pair(MOTOR0_BLDCPWM, pwm_channel_4, &pwm_cfg, PWMV2_CMP_INDEX(8), &cmp_cfg[0], 2);

    pwmv2_counter_select_data_offset_from_shadow_value(MOTOR0_BLDCPWM, pwm_counter_0, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(MOTOR0_BLDCPWM, pwm_counter_0);
    pwmv2_set_reload_update_time(MOTOR0_BLDCPWM, pwm_counter_0, pwm_reload_update_on_reload);

    pwmv2_counter_select_data_offset_from_shadow_value(MOTOR0_BLDCPWM, pwm_counter_1, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(MOTOR0_BLDCPWM, pwm_counter_1);
    pwmv2_set_reload_update_time(MOTOR0_BLDCPWM, pwm_counter_1, pwm_reload_update_on_reload);

    pwmv2_counter_select_data_offset_from_shadow_value(MOTOR0_BLDCPWM, pwm_counter_2, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(MOTOR0_BLDCPWM, pwm_counter_2);
    pwmv2_set_reload_update_time(MOTOR0_BLDCPWM, pwm_counter_2, pwm_reload_update_on_reload);

    pwmv2_select_cmp_source(MOTOR0_BLDCPWM, 16, cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(1));
    if (status_success != pwmv2_set_trigout_cmp_index(MOTOR0_BLDCPWM, BOARD_BLDC_PWM_TRIG_OUT_CHN, 16)) {
        printf("failed to set PWMv2 trigger output compare index\n");
        while (1) {
        }
    }

    pwmv2_issue_shadow_register_lock_event(MOTOR0_BLDCPWM);

    cal.t_param = 4;
    cal.d_param = -4;
    cal.enbale_low_limit = false;
    cal.enable_up_limit = false;
    cal.counter_index = pwm_counter_0;
    cal.in_index = pwm_dac_channel_0;
    cal.in_offset_index = PWMV2_CAL_SHADOW_OFFSET_ZERO;
    pwmv2_setup_cmp_calculate(MOTOR0_BLDCPWM, PWMV2_CALCULATE_INDEX(0), &cal);
    cal.d_param = 4;
    pwmv2_setup_cmp_calculate(MOTOR0_BLDCPWM, PWMV2_CALCULATE_INDEX(1), &cal);

    cal.counter_index = pwm_counter_1;
    cal.in_index = pwm_dac_channel_1;
    cal.d_param = -4;
    pwmv2_setup_cmp_calculate(MOTOR0_BLDCPWM, PWMV2_CALCULATE_INDEX(2), &cal);
    cal.d_param = 4;
    pwmv2_setup_cmp_calculate(MOTOR0_BLDCPWM, PWMV2_CALCULATE_INDEX(3), &cal);

    cal.counter_index = pwm_counter_2;
    cal.in_index = pwm_dac_channel_2;
    cal.d_param = -4;
    pwmv2_setup_cmp_calculate(MOTOR0_BLDCPWM, PWMV2_CALCULATE_INDEX(4), &cal);
    cal.d_param = 4;
    pwmv2_setup_cmp_calculate(MOTOR0_BLDCPWM, PWMV2_CALCULATE_INDEX(5), &cal);

    /* start counter0,counter1,counter2 */
    pwmv2_enable_multi_counter_sync(MOTOR0_BLDCPWM, 0x07);
    pwmv2_start_pwm_output_sync(MOTOR0_BLDCPWM, 0x07);
}

#else
/* PWM v2 初始化：软件 FOC 路径，比较值来自影子寄存器（由 pwm_duty_set 写入） */
void pwm_init(void)
{
    pwmv2_cmp_config_t cmp_cfg[2] = {0};
    pwmv2_pair_config_t pwm_cfg = {0};

    cmp_cfg[0].cmp = PWM_RELOAD;
    cmp_cfg[0].enable_half_cmp = false;
    cmp_cfg[0].enable_hrcmp = false;
    cmp_cfg[0].cmp_source = cmp_value_from_shadow_val;
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(1);
    cmp_cfg[0].update_trigger = pwm_shadow_register_update_on_reload;
    cmp_cfg[1].cmp = PWM_RELOAD;
    cmp_cfg[1].enable_half_cmp = false;
    cmp_cfg[1].enable_hrcmp = false;
    cmp_cfg[1].cmp_source = cmp_value_from_shadow_val;
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(2);
    cmp_cfg[1].update_trigger = pwm_shadow_register_update_on_reload;

    pwmv2_get_default_config(&pwm_cfg.pwm[0]);
    pwmv2_get_default_config(&pwm_cfg.pwm[1]);
    pwm_cfg.pwm[0].enable_output = true;
    pwm_cfg.pwm[0].enable_async_fault = false;
    pwm_cfg.pwm[0].enable_sync_fault = false;
    pwm_cfg.pwm[0].invert_output = false;
    pwm_cfg.pwm[0].enable_four_cmp = false;
    pwm_cfg.pwm[0].update_trigger = pwm_reload_update_on_reload;
    pwm_cfg.pwm[0].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
    pwm_cfg.pwm[1].enable_output = true;
    pwm_cfg.pwm[1].enable_async_fault = false;
    pwm_cfg.pwm[1].enable_sync_fault = false;
    pwm_cfg.pwm[1].invert_output = false;
    pwm_cfg.pwm[1].enable_four_cmp = false;
    pwm_cfg.pwm[1].update_trigger = pwm_reload_update_on_reload;
    pwm_cfg.pwm[1].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;

    pwmv2_disable_counter(MOTOR0_BLDCPWM, pwm_counter_0);
    pwmv2_reset_counter(MOTOR0_BLDCPWM, pwm_counter_0);

    pwmv2_shadow_register_unlock(MOTOR0_BLDCPWM);
    pwmv2_set_shadow_val(MOTOR0_BLDCPWM, PWMV2_SHADOW_INDEX(0), PWM_RELOAD, 0, false);
    pwmv2_set_shadow_val(MOTOR0_BLDCPWM, PWMV2_SHADOW_INDEX(9), 1, 0, false);
    pwmv2_shadow_register_lock(MOTOR0_BLDCPWM);

    pwmv2_setup_waveform_in_pair(MOTOR0_BLDCPWM, pwm_channel_0, &pwm_cfg, PWMV2_CMP_INDEX(0), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(3);
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(4);
    pwmv2_setup_waveform_in_pair(MOTOR0_BLDCPWM, pwm_channel_2, &pwm_cfg, PWMV2_CMP_INDEX(4), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(5);
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(6);
    pwmv2_setup_waveform_in_pair(MOTOR0_BLDCPWM, pwm_channel_4, &pwm_cfg, PWMV2_CMP_INDEX(8), &cmp_cfg[0], 2);

    pwmv2_counter_select_data_offset_from_shadow_value(MOTOR0_BLDCPWM, pwm_counter_0, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(MOTOR0_BLDCPWM, pwm_counter_0);
    pwmv2_set_reload_update_time(MOTOR0_BLDCPWM, pwm_counter_0, pwm_reload_update_on_reload);

    pwmv2_counter_select_data_offset_from_shadow_value(MOTOR0_BLDCPWM, pwm_counter_1, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(MOTOR0_BLDCPWM, pwm_counter_1);
    pwmv2_set_reload_update_time(MOTOR0_BLDCPWM, pwm_counter_1, pwm_reload_update_on_reload);

    pwmv2_counter_select_data_offset_from_shadow_value(MOTOR0_BLDCPWM, pwm_counter_2, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(MOTOR0_BLDCPWM, pwm_counter_2);
    pwmv2_set_reload_update_time(MOTOR0_BLDCPWM, pwm_counter_2, pwm_reload_update_on_reload);

    pwmv2_select_cmp_source(MOTOR0_BLDCPWM, BOARD_BLDCPWM_CMP_TRIG_CMP, cmp_value_from_shadow_val, PWMV2_SHADOW_INDEX(9));
    if (status_success != pwmv2_set_trigout_cmp_index(MOTOR0_BLDCPWM, BOARD_BLDC_PWM_TRIG_OUT_CHN, BOARD_BLDCPWM_CMP_TRIG_CMP)) {
        printf("failed to set PWMv2 trigger output compare index\n");
        while (1) {
        }
    }
    pwmv2_cmp_select_counter(MOTOR0_BLDCPWM, BOARD_BLDCPWM_CMP_TRIG_CMP, pwm_counter_0);
    pwmv2_issue_shadow_register_lock_event(MOTOR0_BLDCPWM);

    /* start counter0,counter1,counter2 */
    pwmv2_enable_multi_counter_sync(MOTOR0_BLDCPWM, 0x07);
    pwmv2_start_pwm_output_sync(MOTOR0_BLDCPWM, 0x07);
}
#endif
#endif

bool pwm_output_is_enabled(void)
{
    return (g_pwm_out_enabled != 0);
}

bool pwm_fault_is_latched(void)
{
#if defined(HPMSOC_HAS_HPMSDK_PWM)
    return ((pwm_get_status(MOTOR0_BLDCPWM) & (uint32_t)PWM_IRQ_FAULT) != 0U);
#elif defined(HPMSOC_HAS_HPMSDK_PWMV2)
    /* PWMv2 的故障标志按通道分开，这里只查当前使用的 counter_0 */
    return ((pwmv2_get_status(MOTOR0_BLDCPWM) & PWMV2_IRQ_FAULT(0)) != 0U);
#else
    return false;
#endif
}

void pwm_fault_clear(void)
{
#if defined(HPMSOC_HAS_HPMSDK_PWM)
    pwm_clear_fault(MOTOR0_BLDCPWM);
    pwm_clear_status(MOTOR0_BLDCPWM, (uint32_t)PWM_IRQ_FAULT);
#elif defined(HPMSOC_HAS_HPMSDK_PWMV2)
    pwmv2_clear_fault(MOTOR0_BLDCPWM, 0);
    pwmv2_clear_status(MOTOR0_BLDCPWM, PWMV2_IRQ_FAULT(0));
#else
    /* 没有 PWM 外设（纯逻辑仿真），无操作 */
#endif
}
