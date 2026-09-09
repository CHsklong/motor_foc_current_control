/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <stdio.h>
#include "drv_pwm.h"
#include "bsp.h"
#include "board.h"
#include "hpm_pwm_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_dmamux_drv.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif
#include "hpm_adc_v2.h"   /* ADC_SOC_DMA_ADDR_ALIGNMENT */

/* PWM 模块时钟频率（drv_pwm_clock_init 时更新） */
static int32_t s_motor_clock_hz;
/* 缓存的重载值，避免在 20kHz 中断里反复做除法 */
static uint32_t s_pwm_reload;

/* DMA 搬运源：6 个比较寄存器值，由 drv_pwm_set_duty() 填充，硬件在周期 75% 处搬入 CMP 寄存器 */
volatile ATTR_PLACE_AT_FAST_RAM_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t pwm_buff[6];

void drv_pwm_clock_init(void)
{
    s_motor_clock_hz = clock_get_frequency(MOTOR0_PWM_CLOCK_NAME);
    s_pwm_reload = (uint32_t)((s_motor_clock_hz / DRV_PWM_FREQUENCY_HZ) - 1);
}

uint32_t drv_pwm_get_reload(void)
{
    return s_pwm_reload;
}

uint32_t drv_pwm_get_frequency(void)
{
    return DRV_PWM_FREQUENCY_HZ;
}

void drv_pwm_init(void)
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
    pwm_set_reload(MOTOR0_BLDCPWM, 0, s_pwm_reload);
    pwm_set_start_count(MOTOR0_BLDCPWM, 0, 0);

    /* 2. 配置比较器 0/1（U 相上下桥），初始值大于重载值，上电时关断 */
    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = s_pwm_reload + 1;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = s_pwm_reload + 1;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;

    /* 3. 配置比较器 2（ADC 触发）
     * 采样点保持 cmp=5（计数器≈0，零矢量区间）：该时刻三相下桥均导通续流，
     * 低边采样电阻可同时采到两相电流。不要改成 reload/2（有效矢量区），
     * 否则只有占空比<50% 的相下桥导通，采样失真。 */
    cmp_config[2].enable_ex_cmp  = false;
    cmp_config[2].mode           = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp            = 5;
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_shlk;

    /* 4. 配置比较器 3（影子寄存器加载触发） */
    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = s_pwm_reload;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_modify;

    /* 5. 配置互补输出（死区、使能、故障保护） */
    pwm_get_default_pwm_pair_config(MOTOR0_BLDCPWM, &pwm_pair_config);

    pwm_pair_config.pwm[0].enable_output = true;
    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = DRV_PWM_DEAD_AREA_TICK;
    pwm_pair_config.pwm[0].invert_output = false;
    pwm_pair_config.pwm[0].fault_mode             = pwm_fault_mode_force_output_0;         /* 故障时强制输出低 */
    pwm_pair_config.pwm[0].fault_recovery_trigger = pwm_fault_recovery_on_fault_clear;     /* 清除故障后恢复 */

    pwm_pair_config.pwm[1].enable_output = true;
    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = DRV_PWM_DEAD_AREA_TICK;
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
        while (1) {
        }
    }
    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 2, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        }
    }
    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, &pwm_pair_config, cmp_index + 4, &cmp_config[0], 2)) {
        printf("failed to setup waveform\n");
        while (1) {
        }
    }
    pwm_load_cmp_shadow_on_match(MOTOR0_BLDCPWM, BOARD_BLDCPWM_CMP_TRIG_CMP, &cmp_config[3]);
    pwm_config_cmp(MOTOR0_BLDCPWM, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &cmp_config[2]);

    /* 8. 配置硬件过流故障源 */
    pwm_fault_source_config_t fault_config = {0};
    fault_config.source_mask                  = pwm_fault_source_external_0;  /* 外部故障通道 0 */
    fault_config.fault_external_0_active_low  = true;                         /* TL331 过流拉低 → 低有效 */
    fault_config.fault_external_1_active_low  = false;                        /* 未使用 fault_1 */
    fault_config.fault_recover_at_rising_edge = false;                        /* 禁止硬件自动恢复，需软件清除 */
    fault_config.fault_output_recovery_trigger = 0;                           /* 恢复触发由通道级配置决定 */
    pwm_config_fault_source(MOTOR0_BLDCPWM, &fault_config);

    /* 9. DMA 触发比较器配置（若启用） */
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
    pwm_output_ch_cfg.cmp_start_index = BOARD_BLDC_DMA_TRG_CMP_INDEX;
    pwm_output_ch_cfg.cmp_end_index   = BOARD_BLDC_DMA_TRG_CMP_INDEX;
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(MOTOR0_BLDCPWM, BOARD_BLDC_DMA_TRG_CMP_INDEX, &pwm_output_ch_cfg);

    cmp_config[2].cmp = s_pwm_reload * 0.75;   /* 在周期 75% 处触发 DMA */
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

void drv_pwm_set_duty(drv_pwm_chn_t chn, float duty)
{
    uint32_t pwm_reload;
    uint32_t pwm_cmp_half, pwm_reload_half;
    uint32_t index0, index1;

    pwm_reload = (int32_t)(s_pwm_reload * 0.98f);
    pwm_cmp_half = (uint32_t)(duty * pwm_reload) >> 1;
    pwm_reload_half = s_pwm_reload >> 1;

    switch (chn) {
    case drv_pwm_chn_a:
        index0 = BOARD_BLDCPWM_CMP_INDEX_0;
        index1 = BOARD_BLDCPWM_CMP_INDEX_1;
        break;
    case drv_pwm_chn_b:
        index0 = BOARD_BLDCPWM_CMP_INDEX_2;
        index1 = BOARD_BLDCPWM_CMP_INDEX_3;
        break;
    case drv_pwm_chn_c:
        index0 = BOARD_BLDCPWM_CMP_INDEX_4;
        index1 = BOARD_BLDCPWM_CMP_INDEX_5;
        break;
    default:
        return;
    }

    /* 不直接写 CMP 寄存器，而是写 DMA 源缓冲 —— 硬件在周期 75% 处统一搬运，
       保证三相占空比在同一时刻更新，避免换相毛刺。 */
    pwm_buff[index0] = PWM_CMP_CMP_SET((pwm_reload_half - pwm_cmp_half));
    pwm_buff[index1] = PWM_CMP_CMP_SET((pwm_reload_half + pwm_cmp_half));
}

void drv_pwm_enable_output(void)
{
    pwm_disable_sw_force(MOTOR0_BLDCPWM);
}

void drv_pwm_disable_output(void)
{
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
}
