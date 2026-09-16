/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_adc.h"
#include "dbg_probe.h"

volatile ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc_buff[3][BOARD_BLDC_ADC_PMT_DMA_SIZE_IN_4BYTES];
ATTR_PLACE_AT_FAST_RAM_INIT uint32_t adc_u_midpoint, adc_v_midpoint;

void init_trigger_mux(TRGM_Type * ptr)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_same_as_input;
    trgm_output_cfg.input  = BOARD_BLDC_PWM_TRG_ADC;                /* PWM 触发 */
    trgm_output_config(ptr, BOARD_BLDC_TRG_ADC, &trgm_output_cfg);   /* PWM触发路由到 ADC 模块 */
}

void init_trigger_cfg(void)     /* ADC 抢占模式配置 */
{
    adc_v2_preempt_config_t pmt_cfg = {0};

    /* ------ 配置 U 相（ADC1，通道 U） ------ */
    pmt_cfg.trig_ch = BOARD_BLDC_ADC_TRG;     /* 触发源 */
    pmt_cfg.trig_len = BOARD_BLDC_ADC_PREEMPT_TRIG_LEN;
    pmt_cfg.inten[0] = true;                  /* 使能中断（用于电流采样） */

    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_U;
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE), &pmt_cfg);

    /* ------ 配置 V 相（ADC0，通道 V） ------ */
    pmt_cfg.inten[0] = false;                 /* V 相不产生中断，只有 U 相(ADC1)驱动采样中断 */
    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_V;
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE), &pmt_cfg);

    /* ------ 配置母线电压（ADC0，通道 VBUS，独立触发通道） ------ */
    pmt_cfg.trig_ch = BOARD_BLDC_ADC_VBUS_TRIG;   /* 使用独立触发 TRG1A */
    pmt_cfg.trig_len = 1;
    pmt_cfg.inten[0] = false;                     /* 不产生中断 */
    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_VBUS;   /* ADC0_IN11（PB08） */
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE), &pmt_cfg);
}

hpm_mcl_stat_t adc_init(void)
{
    adc_v2_config_t cfg;                                                   /* ADC 配置结构体 */
    adc_v2_channel_config_t ch_cfg;                                        /* ADC 通道配置结构体 */
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);      /* 根据 ADC 基地址获取句柄 */
    adc_v2_handle_t adc_v = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE);

    hpm_adc_v2_get_default_config(adc_u, &cfg);                           /* ADC 模块参数初始化 */
    board_init_adc_clock(BOARD_BLDC_ADC_U_BASE, true);                    /* ADC 时钟使能 */
    board_init_adc_clock(BOARD_BLDC_ADC_V_BASE, true);
    board_init_adc_clock(BOARD_BLDC_ADC_W_BASE, true);
    cfg.resolution_bits = BOARD_BLDC_ADC_RES_BITS;                        /* 分辨率设置 */
    cfg.conv_mode = adc_v2_conv_mode_preemption;                          /* 转换模式 */
    cfg.signal_mode = adc_v2_signal_mode_single_ended;                    /* 信号模式 */
    cfg.clock_div = BOARD_BLDC_ADC_CLOCK_DIV;                             /* 时钟分频 */
    cfg.sel_sync_ahb = false;                                             /* AHB 同步 */
    hpm_adc_v2_init(adc_u, &cfg);
    hpm_adc_v2_init(adc_v, &cfg);

    hpm_adc_v2_get_channel_default_config(adc_u, &ch_cfg);                /* 通道参数初始化 */
    ch_cfg.signal_mode = adc_v2_signal_mode_single_ended;
    ch_cfg.sample_cycle = BOARD_BLDC_ADC_CHANNEL_SAMPLE_CYCLE;
    ch_cfg.ch = BOARD_BLDC_ADC_CH_U;
    hpm_adc_v2_init_channel(adc_u, &ch_cfg);
    ch_cfg.ch = BOARD_BLDC_ADC_CH_V;
    hpm_adc_v2_init_channel(adc_v, &ch_cfg);
    /* 母线电压通道（ADC0 CH11, PB08） */
    ch_cfg.ch = BOARD_BLDC_ADC_CH_VBUS;
    hpm_adc_v2_init_channel(adc_v, &ch_cfg);

    init_trigger_mux(BOARD_BLDCPWM_TRGM);                                 /* PWM 触发信号路由到 ADC */
    init_trigger_cfg();                                                    /* 配置 ADC 抢占模式触发通道 */
    hpm_adc_v2_enable_pmt_queue(adc_u, BOARD_BLDC_ADC_TRG);               /* 使能抢占队列 */
    hpm_adc_v2_enable_pmt_queue(adc_v, BOARD_BLDC_ADC_TRG);
    hpm_adc_v2_enable_pmt_queue(adc_v, BOARD_BLDC_ADC_VBUS_TRIG);
    hpm_adc_v2_init_pmt_dma(adc_u, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[ADCU_INDEX]));
    hpm_adc_v2_init_pmt_dma(adc_v, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[ADCV_INDEX]));
#if defined(HW_CURRENT_FOC_ENABLE) && HPM_ADC_V2_HAS_MOTOR_MODE
    hpm_adc_v2_enable_motor_mode(adc_u);
    hpm_adc_v2_enable_motor_mode(adc_v);
#endif

    return mcl_success;
}

void adc_isr_enable(void)    /* 使能 ADC 转换完成中断，ADC 完成采样后触发中断 */
{
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);

    hpm_adc_v2_enable_interrupts(adc_u, HPM_ADC_V2_EVENT_TRIG_COMPLETE);
    intc_m_enable_irq_with_priority(BOARD_BLDC_ADC_IRQn, 1);     /* 中断优先级 */
}

/**
 * @brief 静态零点校准
 *
 * 三个针对"冷上电"的加固（原实现只有一个裸的 200 次循环）：
 *   1) 先等 DMA 真的刷新过 —— 相电流 ADC 由 PWM 触发，PWM 没跑起来时
 *      adc_buff 全是上电残值，200 次"平均"其实是把同一个脏数加了 200 遍；
 *   2) 丢掉前 ADC_MID_DISCARD_TIMES 次 —— 避开运放/基准的上电爬升段；
 *   3) 结果做合理性校验（1/4~3/4 量程），不合理就重来。
 *      零点错 = 电流反馈带直流偏置 = 出力不足 / 起步过冲，且冷上电才犯病。
 */
void motor_adc_midpoint(void)    /* 采样获取零点偏移值 */
{
    uint32_t retry;

    for (retry = 0U; retry < ADC_MID_RETRY_MAX; retry++)
    {
        uint32_t adc_u_sum = 0;
        uint32_t adc_v_sum = 0;
        uint32_t times;
        uint32_t mid_u;
        uint32_t mid_v;

        /* ① 等首帧有效数据
           adc_buff 在 BSS 里，初值全 0；只要 DMA 搬运过一次，12bit 通道值就不会再是 0
           （零电流时 ADC 码值在 2048 附近）。所以"非零"就足以证明触发链通了。
           【2026-09-16】原来还要求"前后两次值不同"，本意是确认在刷新，但电流恒定时
           ADC 码值完全可以连续几拍一模一样 —— 判据过严，每次都要耗满
           等满上限才退出，这是上电延迟的大头之一。 */
        g_mid_wait_us = 0U;
        while (g_mid_wait_us < (ADC_MID_WAIT_CNT_MAX * ADC_MID_SAMPLE_US))
        {
            if (MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[0][BOARD_BLDC_ADC_TRG*4]) != 0U)
            {
                break;
            }
            board_delay_us(ADC_MID_SAMPLE_US);
            g_mid_wait_us += ADC_MID_SAMPLE_US;
        }

        /* ② 丢掉爬升段（避开运放/基准的上电爬升） */
        for (times = 0U; times < ADC_MID_DISCARD_TIMES; times++)
        {
            board_delay_us(ADC_MID_SAMPLE_US);
        }

        /* ③ 正式平均（间隔用 us 而不是 ms：20kHz 触发下 1ms 会漏掉 19 帧，
              同样的平均次数却要多等 10 倍时间） */
        adc_u_sum = 0;
        adc_v_sum = 0;
        for (times = 0U; times < CURRENT_SET_TIME_MS; times++)
        {
            adc_u_sum += MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[0][BOARD_BLDC_ADC_TRG*4]);
            adc_v_sum += MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[1][BOARD_BLDC_ADC_TRG*4]);
            board_delay_us(ADC_MID_SAMPLE_US);
        }
        mid_u = adc_u_sum / CURRENT_SET_TIME_MS;
        mid_v = adc_v_sum / CURRENT_SET_TIME_MS;

        /* ④ 合理性校验：偏离中点太远说明读到的是脏数据，重来 */
        if ((mid_u >= ADC_MID_MIN) && (mid_u <= ADC_MID_MAX) &&
            (mid_v >= ADC_MID_MIN) && (mid_v <= ADC_MID_MAX))
        {
            adc_u_midpoint = mid_u;
            adc_v_midpoint = mid_v;
            g_mid_u = mid_u;
            g_mid_v = mid_v;
            g_mid_retry = retry;
            return;
        }
        g_mid_retry = retry + 1U;
    }

    /* 三次都不合理：仍然写进去（否则恒 0 更糟），但把重试次数留在探针里给人看 */
    adc_u_midpoint = 2048U;
    adc_v_midpoint = 2048U;
    g_mid_u = adc_u_midpoint;
    g_mid_v = adc_v_midpoint;
}
