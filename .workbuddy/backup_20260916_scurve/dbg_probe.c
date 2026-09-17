/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "dbg_probe.h"

/* ---------------- 异常捕获 ----------------
 * 覆盖 soc/HPM5300/HPM5361/toolchains/trap.c 里的 weak exception_handler。
 * SDK 默认版本 return epc 会让 CPU 在出错指令上无限重复 trap（假死机，变量冻结）。
 * 这里记下 mcause/mepc 后原地停机：J-Scope 读 g_exc_cause/g_exc_epc 即可定位
 * 异常类型和出错指令地址（用 objdump 反汇编该地址找所属函数）。 */
volatile uint32_t g_exc_cause = 0xFFFFFFFFu;  /* 0xFFFFFFFF = 本次上电从未踩过异常 */
volatile uint32_t g_exc_epc = 0;

long exception_handler(long cause, long epc)
{
    disable_global_irq(CSR_MSTATUS_MIE_MASK);   /* 冻结现场，防止中断再改探针 */
    g_exc_cause = (uint32_t)cause;
    g_exc_epc   = (uint32_t)epc;
    for (;;) {
        /* 原地停机：绝不能 return，返回会让 CPU 回到出错指令再次 trap */
    }
}

/* ---------------- 编码器 / SPI ---------------- */
uint32_t      g_spi_err = 0;
uint32_t      g_spi_fail_cnt = 0;
uint8_t       g_byte0 = 0;
uint8_t       g_byte1 = 0;
uint8_t       g_byte2 = 0;
uint8_t       g_user_id = 0;
uint8_t       g_rx_byte = 0;
volatile uint8_t g_seq3_ok = 0;
uint32_t      g_angle_raw = 0;
float         theta_r = 0.0f;

/* ---------------- 相电流 / ADC ---------------- */
volatile int32_t g_a = 0;
volatile int32_t g_b = 0;
volatile int32_t g_c = 0;
float         g_raw_u = 0.0f;
float         g_raw_v = 0.0f;
float         g_vbus = 0.0f;
volatile uint32_t g_u16 = 0;
volatile uint32_t g_v16 = 0;
volatile uint32_t g_mid_u = 0;
volatile uint32_t g_mid_v = 0;
volatile uint32_t g_cmp4 = 0;
volatile uint32_t g_cmp5 = 0;

/* ---------------- 位置环 ---------------- */
volatile float g_pos_abs = 0.0f;
volatile float g_pos_ref = 0.0f;
volatile float g_pos_err = 0.0f;
volatile float g_ref_speed = 0.0f;
volatile float g_pos_integral = 0.0f;
volatile float g_spd_fdb = 0.0f;

/* ---------------- S 曲线规划 ---------------- */
volatile uint32_t g_sc_state = 0;
volatile float g_sc_theta = 0.0f;
volatile float g_sc_omega = 0.0f;
volatile uint32_t g_sc_abort = 0;

/* ---------------- 存活心跳 ---------------- */
volatile uint32_t g_heart_isr = 0;
volatile uint32_t g_heart_main = 0;

/* ---------------- 启动步进追踪 ---------------- */
volatile uint32_t g_boot_step = 0;

/* ---------------- 故障状态 ---------------- */
volatile uint32_t g_loop_status = 0;
volatile uint32_t g_enc_status = 0;
volatile uint32_t g_ana_status = 0;
volatile uint32_t g_pwm_enabled = 0;

/* ---------------- 上电 / 硬件故障诊断 ---------------- */
volatile uint32_t g_pwm_fault = 0;
volatile uint32_t g_pwm_fault_clr = 0;
volatile uint32_t g_fault_src = 0;
volatile uint32_t g_fault_cnt = 0;
volatile uint32_t g_mid_retry = 0;
volatile uint32_t g_mid_wait_us = 0;
volatile uint32_t g_seq3_retry = 0;
volatile uint32_t g_angle_jump = 0;
volatile float    g_boot_vbus = 0.0f;

/* ---------------- 其他 ---------------- */
volatile float g_theta_initial = 0.0f;
volatile float g_probe_keep = 0.0f;

void dbg_probe_keep(void)
{
    /* MCL 侧探针（定义在 hpm_mcl_loop.c，头文件里有 extern 声明） */
    g_probe_keep = g_ref_q + g_sens_q + g_ref_d + g_sens_d + g_ud + g_uq + g_theta_e + g_theta_initial;

    /* 本工程探针：做一次加法，让链接器认为它们被使用 */
    g_probe_keep += (float)g_spi_err + (float)g_spi_fail_cnt + (float)g_byte0 + (float)g_byte1 +
                    (float)g_byte2 + (float)g_user_id + (float)g_rx_byte + (float)g_seq3_ok +
                    (float)g_angle_raw + theta_r +
                    (float)g_a + (float)g_b + (float)g_c + g_raw_u + g_raw_v + g_vbus +
                    (float)g_u16 + (float)g_v16 + (float)g_mid_u + (float)g_mid_v +
                    (float)g_cmp4 + (float)g_cmp5 +
                    g_pos_abs + g_pos_ref + g_pos_err + g_ref_speed + g_pos_integral +
                    g_spd_fdb +
                    (float)g_sc_state + g_sc_theta + g_sc_omega + (float)g_sc_abort +
                    (float)g_heart_isr + (float)g_heart_main +
                    (float)g_loop_status + (float)g_enc_status + (float)g_ana_status +
                    (float)g_pwm_enabled +
                    (float)g_pwm_fault + (float)g_pwm_fault_clr + (float)g_fault_src +
                    (float)g_fault_cnt + (float)g_mid_retry + (float)g_mid_wait_us +
                    (float)g_seq3_retry + (float)g_angle_jump + g_boot_vbus +
                    (float)g_flash_status + (float)g_flash_valid + g_flash_theta +
                    (float)g_flash_erase_ms + (float)g_flash_erase_cycles +
                    (float)g_flash_ofs;
}
