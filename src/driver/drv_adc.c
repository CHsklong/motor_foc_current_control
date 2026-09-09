/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "drv_adc.h"
#include "bsp.h"
#include "board.h"
#include "hpm_adc_v2.h"
#include "hpm_trgm_drv.h"

/* ADC 抢占模式 DMA 目标缓冲：[ADC模块][每触发通道 4 word] */
volatile ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
    uint32_t adc_buff[3][BOARD_BLDC_ADC_PMT_DMA_SIZE_IN_4BYTES];

/**
 * @brief 将 PWM 触发信号路由到 ADC 模块
 */
static void adc_trigger_mux_init(TRGM_Type *ptr)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_same_as_input;
    trgm_output_cfg.input  = BOARD_BLDC_PWM_TRG_ADC;
    trgm_output_config(ptr, BOARD_BLDC_TRG_ADC, &trgm_output_cfg);
}

/**
 * @brief 配置 ADC 抢占模式触发通道
 *
 * - U 相（ADC1）: 硬件触发，产生中断 —— 由它驱动 20kHz 电流环
 * - V 相（ADC0）: 硬件触发，不产生中断
 * - 母线电压（ADC0 CH11）: 独立软件触发通道 TRG1A，不产生中断
 */
static void adc_trigger_cfg_init(void)
{
    adc_v2_preempt_config_t pmt_cfg = {0};

    /* ------ U 相（ADC1，通道 U） ------ */
    pmt_cfg.trig_ch  = BOARD_BLDC_ADC_TRG;
    pmt_cfg.trig_len = BOARD_BLDC_ADC_PREEMPT_TRIG_LEN;
    pmt_cfg.inten[0] = true;                  /* 使能中断（用于电流采样） */
    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_U;
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE), &pmt_cfg);

    /* ------ V 相（ADC0，通道 V） ------ */
    pmt_cfg.inten[0] = false;                 /* 只有 U 相驱动采样中断 */
    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_V;
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE), &pmt_cfg);

    /* ------ 母线电压（ADC0，通道 VBUS，独立触发通道） ------ */
    pmt_cfg.trig_ch  = BOARD_BLDC_ADC_VBUS_TRIG;
    pmt_cfg.trig_len = 1;
    pmt_cfg.inten[0] = false;
    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_VBUS;
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE), &pmt_cfg);
}

void drv_adc_init(void)
{
    adc_v2_config_t cfg;
    adc_v2_channel_config_t ch_cfg;
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);
    adc_v2_handle_t adc_v = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE);

    hpm_adc_v2_get_default_config(adc_u, &cfg);
    board_init_adc_clock(BOARD_BLDC_ADC_U_BASE, true);
    board_init_adc_clock(BOARD_BLDC_ADC_V_BASE, true);
    board_init_adc_clock(BOARD_BLDC_ADC_W_BASE, true);
    cfg.resolution_bits = BOARD_BLDC_ADC_RES_BITS;
    cfg.conv_mode       = adc_v2_conv_mode_preemption;
    cfg.signal_mode     = adc_v2_signal_mode_single_ended;
    cfg.clock_div       = BOARD_BLDC_ADC_CLOCK_DIV;
    cfg.sel_sync_ahb    = false;
    hpm_adc_v2_init(adc_u, &cfg);
    hpm_adc_v2_init(adc_v, &cfg);

    hpm_adc_v2_get_channel_default_config(adc_u, &ch_cfg);
    ch_cfg.signal_mode = adc_v2_signal_mode_single_ended;
    ch_cfg.sample_cycle = BOARD_BLDC_ADC_CHANNEL_SAMPLE_CYCLE;
    ch_cfg.ch = BOARD_BLDC_ADC_CH_U;
    hpm_adc_v2_init_channel(adc_u, &ch_cfg);
    ch_cfg.ch = BOARD_BLDC_ADC_CH_V;
    hpm_adc_v2_init_channel(adc_v, &ch_cfg);
    ch_cfg.ch = BOARD_BLDC_ADC_CH_VBUS;      /* 母线电压通道 */
    hpm_adc_v2_init_channel(adc_v, &ch_cfg);

    adc_trigger_mux_init(BOARD_BLDCPWM_TRGM);
    adc_trigger_cfg_init();

    hpm_adc_v2_enable_pmt_queue(adc_u, BOARD_BLDC_ADC_TRG);
    hpm_adc_v2_enable_pmt_queue(adc_v, BOARD_BLDC_ADC_TRG);
    hpm_adc_v2_enable_pmt_queue(adc_v, BOARD_BLDC_ADC_VBUS_TRIG);

    hpm_adc_v2_init_pmt_dma(adc_u, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[ADCU_INDEX]));
    hpm_adc_v2_init_pmt_dma(adc_v, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[ADCV_INDEX]));
}

void drv_adc_isr_enable(void)
{
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);

    hpm_adc_v2_enable_interrupts(adc_u, HPM_ADC_V2_EVENT_TRIG_COMPLETE);
    intc_m_enable_irq_with_priority(BOARD_BLDC_ADC_IRQn, 1);
}

uint16_t drv_adc_read_phase_raw(drv_adc_chn_t chn)
{
    uint32_t idx = (chn == drv_adc_chn_u) ? ADCU_INDEX : ADCV_INDEX;

    return (uint16_t)DRV_ADC_GET_12BIT(adc_buff[idx][BOARD_BLDC_ADC_TRG * 4]);
}

bool drv_adc_read_vbus_raw(uint16_t *raw)
{
    adc_v2_handle_t adc_v = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE);
    adc_v2_dma_sample_t sample;
    uint32_t word;

    /* 软件触发 TRG1A 开始采样母线电压 */
    hpm_adc_v2_trigger_pmt_by_sw(adc_v, BOARD_BLDC_ADC_VBUS_TRIG);

    /* 等待转换 + DMA 搬运完成（母线电压变化慢，20us 足够） */
    bsp_delay_us(20);

    /* 从 DMA buffer 读结果：每个 TRG 通道占 4 个 word，TRG1A → offset = trig*4
     * 用 parse 校验 trig_ch 与 adc_ch，防止布局假设错导致读到脏数据 */
    word = adc_buff[ADCV_INDEX][BOARD_BLDC_ADC_VBUS_TRIG * 4];
    hpm_adc_v2_parse_pmt_dma_word(adc_v, word, &sample);
    if ((sample.trig_ch != BOARD_BLDC_ADC_VBUS_TRIG) || (sample.adc_ch != BOARD_BLDC_ADC_CH_VBUS)) {
        return false;
    }

    *raw = (uint16_t)DRV_ADC_GET_12BIT(sample.result);
    return true;
}
