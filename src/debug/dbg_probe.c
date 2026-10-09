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
volatile float g_iq_ref = 0.0f;
volatile float g_spd_ref_in = 0.0f;
volatile float g_speed_err = 0.0f;
volatile float g_iq_ff = 0.0f;

/* ---------------- S 曲线轨迹规划 ---------------- */
volatile float    g_sc_t = 0.0f;
volatile float    g_sc_v_ref = 0.0f;
volatile float    g_sc_a_ref = 0.0f;
volatile float    g_sc_v_cmd = 0.0f;
volatile float    g_sc_corr = 0.0f;
volatile uint32_t g_sc_state = 0;
volatile uint32_t g_sc_shape = 0;
volatile float    g_sc_t_all = 0.0f;
volatile float    g_sc_vp = 0.0f;
volatile uint32_t g_sc_move_cnt = 0;
volatile uint32_t g_sc_busy_cnt = 0;
volatile uint32_t g_sc_tick  = 0;      /* 规划器时间拍镜像（20kHz 递增） */
volatile uint32_t g_sc_dwell = 0;      /* 主循环停留计数镜像 */
volatile uint32_t g_sc_nan   = 0;      /* s_v_cmd 非有限值次数 */

/* ---------------- 速度域 S 曲线（g_scs 成员平铺，J-Scope 才点得开） ---------------- */
volatile float    g_scs_v        = 0.0f;
volatile float    g_scs_a        = 0.0f;
volatile float    g_scs_j        = 0.0f;
volatile uint32_t g_scs_state    = 0;
volatile uint32_t g_scs_move_cnt = 0;
volatile uint32_t g_scs_tick     = 0;
volatile float    g_scs_v0       = 0.0f;
volatile float    g_scs_dv       = 0.0f;
volatile float    g_scs_ta       = 0.0f;
volatile float    g_scs_amax     = 0.0f;   /* 本段生效的加速度上限（CAN SET_ACCEL/SET_DECEL 的落点） */
volatile float    g_scs_ap       = 0.0f;   /* 本段实际峰值加速度（三角形分支时 < amax） */
volatile float    g_scs_lim_acc  = 0.0f;   /* 当前 amax_acc 约束 rad/s^2 */
volatile float    g_scs_lim_dec  = 0.0f;   /* 当前 amax_dec 约束 rad/s^2 */
volatile uint32_t g_scs_stage2   = 0;      /* 1 = 跨零换向：第一段减到 0 中 */

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

/* ---------------- 运行监护 app_keeper ---------------- */
volatile uint32_t g_keeper_fault_clr  = 0;
volatile uint32_t g_keeper_out_rec    = 0;
volatile uint32_t g_keeper_loop_rec   = 0;
volatile uint32_t g_keeper_vbus_upd   = 0;
/* ---------------- 20kHz 节拍耗时 ---------------- */
volatile uint32_t g_isr_cycles_last = 0;
volatile uint32_t g_isr_cycles_max  = 0;
volatile uint32_t g_isr_over_cnt    = 0;
volatile uint32_t g_isr_us_max      = 0;
volatile uint32_t g_isr_us_last     = 0;
volatile uint32_t g_enc_cycles_max  = 0;
volatile uint32_t g_loop_cycles_max = 0;
volatile uint32_t g_enc_us_max      = 0;
volatile uint32_t g_loop_us_max     = 0;
volatile uint32_t g_isr_us_win_max  = 0;
volatile uint32_t g_enc_us_win_max  = 0;
volatile uint32_t g_loop_us_win_max = 0;
volatile uint32_t g_isr_win_over    = 0;
volatile uint32_t g_isr_win_cnt     = 0;
volatile uint32_t g_isr_starve_cnt  = 0;
volatile uint32_t g_isr_relief_cnt  = 0;
volatile uint32_t g_main_step       = 0;

volatile uint32_t g_keeper_blocked    = 0;
volatile uint32_t g_keeper_last_cause = 0;
volatile float    g_vbus_now          = 0.0f;

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
                    g_spd_fdb + g_iq_ref + g_spd_ref_in + g_speed_err +
                    g_sc_t + g_sc_v_ref + g_sc_a_ref + g_sc_v_cmd + g_sc_corr +
                    g_sc_t_all + g_sc_vp +
                    (float)g_sc_state + (float)g_sc_shape +
                    (float)g_sc_move_cnt + (float)g_sc_busy_cnt +
                    (float)g_sc_tick + (float)g_sc_dwell + (float)g_sc_nan +
                    g_scs_v + g_scs_a + g_scs_j + g_scs_v0 + g_scs_dv + g_scs_ta +
                    g_scs_amax + g_scs_ap + g_scs_lim_acc + g_scs_lim_dec +
                    (float)g_scs_state + (float)g_scs_move_cnt + (float)g_scs_tick +
                    (float)g_scs_stage2 +
                    (float)g_heart_isr + (float)g_heart_main +
                    (float)g_loop_status + (float)g_enc_status + (float)g_ana_status +
                    (float)g_pwm_enabled +
                    (float)g_pwm_fault + (float)g_pwm_fault_clr + (float)g_fault_src +
                    (float)g_fault_cnt + (float)g_mid_retry + (float)g_mid_wait_us +
                    (float)g_seq3_retry + (float)g_angle_jump + g_boot_vbus +
                    (float)g_isr_cycles_last + (float)g_isr_cycles_max +
                    (float)g_isr_over_cnt + (float)g_isr_us_max + (float)g_isr_us_last +
                    (float)g_enc_cycles_max + (float)g_loop_cycles_max +
                    (float)g_enc_us_max + (float)g_loop_us_max + (float)g_main_step +
                    (float)g_isr_us_win_max + (float)g_enc_us_win_max +
                    (float)g_loop_us_win_max + (float)g_isr_win_over +
                    (float)g_isr_win_cnt + (float)g_isr_starve_cnt + (float)g_isr_relief_cnt +
                    (float)g_keeper_fault_clr + (float)g_keeper_out_rec +
                    (float)g_keeper_loop_rec + (float)g_keeper_vbus_upd +
                    (float)g_keeper_blocked + (float)g_keeper_last_cause + g_vbus_now +
                    (float)g_flash_status + (float)g_flash_valid + g_flash_theta +
                    (float)g_flash_erase_ms + (float)g_flash_erase_cycles +
                    (float)g_flash_ofs;
}
