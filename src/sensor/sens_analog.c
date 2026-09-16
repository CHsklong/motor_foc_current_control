/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_adc.h"
#include "dbg_probe.h"
#include "sens_analog.h"

static int32_t current_a = 0;   /* 缓存 A 相电流码值，供 B 相重构使用 */

hpm_mcl_stat_t analog_update_sample_location(mcl_analog_chn_t chn, uint32_t tick)
{
    (void)chn;
    (void)tick;
    return mcl_success;
}

hpm_mcl_stat_t adc_value_get(mcl_analog_chn_t chn, int32_t *value)
{
    int32_t sens_value;

    switch (chn) {
    case analog_a_current:
        sens_value = MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[ADCU_INDEX][BOARD_BLDC_ADC_TRG*4]);
        g_a = sens_value;
        current_a = adc_u_midpoint - sens_value;   /* 计算并缓存 A 相电流 */
        *value = current_a;
        break;

    case analog_b_current:
        /* 实际硬件采样的是 C 相电流（ADCV_INDEX 对应 C 相），
           B 相通过 Ia + Ib + Ic = 0 重构：Ib = -(Ia + Ic) */
        {
            int32_t sens_a = MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[ADCU_INDEX][BOARD_BLDC_ADC_TRG*4]);
            int32_t sens_c = MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[ADCV_INDEX][BOARD_BLDC_ADC_TRG*4]);
            int32_t current_a_local = adc_u_midpoint - sens_a;
            int32_t current_c = adc_v_midpoint - sens_c;
            *value = -(current_a_local + current_c);
        }
        break;
    default:
        return mcl_fail;
    }

    return mcl_success;
}

/**
 * @brief 读一次母线电压
 *
 * 加了重试：冷上电时 ADC/DMA 首帧可能还没落定，单次读取会撞上"布局校验失败 → -1"。
 * 而 motor_init() 里只读这一次并把结果写进 const_vbus（dq 解耦与限幅要用），
 * 读到 -1 会让整机的电压模型全错，所以这里必须多试几次。
 */
float read_vbus(void)
{
    adc_v2_handle_t adc_v = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE);   /* 获取 ADC 句柄 */

    for (uint32_t i = 0U; i < VBUS_READ_RETRY; i++)
    {
        uint32_t raw;                       /* 从 DMA buffer 读出的原始 32 位数据 */
        adc_v2_dma_sample_t sample;         /* 解析 DMA 数据：触发通道、ADC 通道、结果 */
        uint16_t adc_val;                   /* 12 位有效 ADC 数值 */
        float pin_voltage, vbus;            /* 引脚电压和最终母线电压（V） */

        /* ① 软件触发 TRG1A，开始采样母线电压 */
        hpm_adc_v2_trigger_pmt_by_sw(adc_v, BOARD_BLDC_ADC_VBUS_TRIG);

        /* ② 等待转换 + DMA 搬运完成（母线电压变化慢，20µs 足够） */
        board_delay_us(20);

        /* ③ 从 DMA buffer 读结果
         *    布局: 每个 TRG 通道占 4 个 word
         *    用 parse 校验 trig_ch 和 adc_ch，防止布局假设错导致读到脏数据 */
        raw = adc_buff[ADCV_INDEX][BOARD_BLDC_ADC_VBUS_TRIG * 4];
        hpm_adc_v2_parse_pmt_dma_word(adc_v, raw, &sample);
        if ((sample.trig_ch != BOARD_BLDC_ADC_VBUS_TRIG) || (sample.adc_ch != BOARD_BLDC_ADC_CH_VBUS)) {
            board_delay_us(50);
            continue;   /* 布局不符（多半是首帧未落定），重试 */
        }

        /* ④ 换算: ADC 读数 → ADC 引脚电压 → 实际母线电压 */
        adc_val = MCL_GET_ADC_12BIT_VALID_DATA(sample.result);   /* 取 12bit 有效位 */
        pin_voltage = (float)adc_val * 3.3f / 4095.0f;
        vbus = pin_voltage * BOARD_BLDC_VBUS_DIVIDER;

        return vbus;
    }

    return -1.0f;   /* 多次都失败，返回负数表示读取失败，由调用方兜底 */
}
