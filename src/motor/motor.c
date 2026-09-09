/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <string.h>
#include "motor.h"
#include "motor_params.h"
#include "bsp.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "sens_encoder.h"
#include "sens_analog.h"
#include "hpm_clock_drv.h"
#include "board.h"

ATTR_PLACE_AT_FAST_RAM_INIT motor_t motor0;

/* ==================== MCL 回调适配 ====================
 * 把 driver / sensor 提供的语义化接口，包成 MCL 要求的签名。
 * 这样 driver 与 sensor 两层都不需要知道 MCL 的存在。 */

/* 注意：drivers.callback.init 的签名是 void(*)(void)，
 * 而 enable/disable/update_duty_cycle 是 hpm_mcl_stat_t(*)(...) —— 两者不一致，按各自类型定义 */
static void mcl_pwm_init(void)
{
    drv_pwm_init();
}

static hpm_mcl_stat_t mcl_pwm_enable_all(void)
{
    drv_pwm_enable_output();
    return mcl_success;
}

static hpm_mcl_stat_t mcl_pwm_disable_all(void)
{
    drv_pwm_disable_output();
    return mcl_success;
}

static hpm_mcl_stat_t mcl_pwm_duty_set(mcl_drivers_channel_t chn, float duty)
{
    switch (chn) {
    case mcl_drivers_chn_a:
        drv_pwm_set_duty(drv_pwm_chn_a, duty);
        break;
    case mcl_drivers_chn_b:
        drv_pwm_set_duty(drv_pwm_chn_b, duty);
        break;
    case mcl_drivers_chn_c:
        drv_pwm_set_duty(drv_pwm_chn_c, duty);
        break;
    default:
        return mcl_fail;
    }
    return mcl_success;
}

static hpm_mcl_stat_t mcl_adc_init(void)
{
    drv_adc_init();
    return mcl_success;
}

static hpm_mcl_stat_t mcl_adc_value_get(mcl_analog_chn_t chn, int32_t *value)
{
    switch (chn) {
    case analog_a_current:
        return sens_current_read_phase_a(value) ? mcl_success : mcl_fail;
    case analog_b_current:
        return sens_current_read_phase_b(value) ? mcl_success : mcl_fail;
    default:
        return mcl_fail;
    }
}

static hpm_mcl_stat_t mcl_adc_update_sample_location(mcl_analog_chn_t chn, uint32_t tick)
{
    (void)chn;
    (void)tick;
    return mcl_success;
}

static hpm_mcl_stat_t mcl_encoder_start_sample(void)
{
    return mcl_success;
}

static hpm_mcl_stat_t mcl_encoder_get_theta(float *theta)
{
    return sens_encoder_get_theta(theta) ? mcl_success : mcl_fail;
}

static hpm_mcl_stat_t mcl_encoder_get_abs_theta(float *theta)
{
    return sens_encoder_get_abs_theta(theta) ? mcl_success : mcl_fail;
}

/* 注意：control.callback.init 的签名是 void(*)(void)，与 encoder/drivers 的
 * hpm_mcl_stat_t(*)(void) 不同，这里按实际类型定义，避免不兼容指针告警。 */
static void mcl_control_init(void)
{
}

/* ==================== 参数配置 ==================== */

static void motor_cfg_sampling(void)
{
    motor0.cfg.mcl.physical.board.analog[analog_a_current].adc_reference_vol = MOTOR0_ADC_REF_VOL;
    motor0.cfg.mcl.physical.board.analog[analog_a_current].opamp_gain = MOTOR0_OPAMP_GAIN;
    motor0.cfg.mcl.physical.board.analog[analog_a_current].sample_precision = MOTOR0_ADC_PRECISION;
    motor0.cfg.mcl.physical.board.analog[analog_a_current].sample_res = MOTOR0_SAMPLE_RES;

    motor0.cfg.mcl.physical.board.analog[analog_b_current].adc_reference_vol = MOTOR0_ADC_REF_VOL;
    motor0.cfg.mcl.physical.board.analog[analog_b_current].opamp_gain = MOTOR0_OPAMP_GAIN;
    motor0.cfg.mcl.physical.board.analog[analog_b_current].sample_precision = MOTOR0_ADC_PRECISION;
    motor0.cfg.mcl.physical.board.analog[analog_b_current].sample_res = MOTOR0_SAMPLE_RES;

    motor0.cfg.mcl.physical.board.num_current_sample_res = MOTOR0_NUM_SAMPLE_RES;
    motor0.cfg.mcl.physical.board.pwm_dead_time_tick = DRV_PWM_DEAD_AREA_TICK;
    motor0.cfg.mcl.physical.board.pwm_frequency = drv_pwm_get_frequency();
    motor0.cfg.mcl.physical.board.pwm_reload = drv_pwm_get_reload();
}

static void motor_cfg_physical(void)
{
    motor0.cfg.mcl.physical.motor.i_max = MOTOR0_I_MAX;
    motor0.cfg.mcl.physical.motor.inertia = MOTOR0_INERTIA;
    motor0.cfg.mcl.physical.motor.ls = MOTOR0_LS;
    motor0.cfg.mcl.physical.motor.pole_num = MOTOR0_POLE_NUM;
    motor0.cfg.mcl.physical.motor.power = MOTOR0_POWER;
    motor0.cfg.mcl.physical.motor.res = MOTOR0_RES;
    motor0.cfg.mcl.physical.motor.rpm_max = MOTOR0_RPM_MAX;
    motor0.cfg.mcl.physical.motor.vbus = sens_vbus_read();
    motor0.cfg.mcl.physical.motor.flux = MOTOR0_FLUX;
    motor0.cfg.mcl.physical.motor.ld = MOTOR0_LD;
    motor0.cfg.mcl.physical.motor.lq = MOTOR0_LQ;
}

static void motor_cfg_time(void)
{
    uint32_t pwm_freq = drv_pwm_get_frequency();

    motor0.cfg.mcl.physical.time.adc_sample_ts = MCL_FREQUENCY_TO_PERIOD(pwm_freq);
    motor0.cfg.mcl.physical.time.current_loop_ts = MCL_FREQUENCY_TO_PERIOD(pwm_freq);
    motor0.cfg.mcl.physical.time.encoder_process_ts = MCL_FREQUENCY_TO_PERIOD(pwm_freq);
    motor0.cfg.mcl.physical.time.speed_loop_ts = MCL_FREQUENCY_TO_PERIOD(pwm_freq) * 5;
    motor0.cfg.mcl.physical.time.position_loop_ts = MCL_FREQUENCY_TO_PERIOD(pwm_freq) * 20;
    motor0.cfg.mcl.physical.time.mcu_clock_tick = clock_get_frequency(clock_cpu0);
    motor0.cfg.mcl.physical.time.pwm_clock_tick = clock_get_frequency(MOTOR0_PWM_CLOCK_NAME);
}

static void motor_cfg_analog(void)
{
    motor0.cfg.analog.enable_a_current = true;
    motor0.cfg.analog.enable_b_current = true;
    memset(motor0.cfg.analog.enable_filter, false, MCL_ANALOG_CHN_NUM);
    motor0.cfg.analog.enable_vbus = true;
}

static void motor_cfg_encoder(void)
{
    motor0.cfg.encoder.communication_interval_us = 0;
    motor0.cfg.encoder.disable_start_sample_interrupt = true;
    motor0.cfg.encoder.period_call_time_s = MCL_FREQUENCY_TO_PERIOD(drv_pwm_get_frequency());
    motor0.cfg.encoder.precision = BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV;
    motor0.cfg.encoder.speed_abs_switch_m_t = 5;
    motor0.cfg.encoder.speed_cal_method = encoder_method_m;
    motor0.cfg.encoder.timeout_s = 0.5;

    /* 编码器速度低通：fpass 100 fstop 2000，二阶 */
    motor0.cfg.encoder_iir.section = 2;
    motor0.cfg.encoder_iir.matrix = motor0.cfg.encoder_iir_mat;
    motor0.cfg.encoder_iir_mat[0].a1 = -1.947404031871316831825424742419272661209f;
    motor0.cfg.encoder_iir_mat[0].a2 = 0.95152023575172306468772376319975592196f;
    motor0.cfg.encoder_iir_mat[0].b0 = 1;
    motor0.cfg.encoder_iir_mat[0].b1 = 2;
    motor0.cfg.encoder_iir_mat[0].b2 = 1;
    motor0.cfg.encoder_iir_mat[0].scale = 0.001029050970101526990552187612593115773f;

    motor0.cfg.encoder_iir_mat[1].a1 = -1.88285893096534651114382086234400048852f;
    motor0.cfg.encoder_iir_mat[1].a2 = 0.886838706662149367510039610351668670774f;
    motor0.cfg.encoder_iir_mat[1].b0 = 1;
    motor0.cfg.encoder_iir_mat[1].b1 = 2;
    motor0.cfg.encoder_iir_mat[1].b2 = 1;
    motor0.cfg.encoder_iir_mat[1].scale = 0.000994943924200649039424337871651005116f;
}

/**
 * @brief 电流环 PI 整定
 *
 * ⚠️ 这里沿用 HPM SDK 官方样例的公式：kp = L·ωc²·ts·1.5, ki = R·ωc²·ts·1.5
 *    该公式与注释（K_p = W_c*L）不符，且使得 ki/(kp·ts) = R/(L·ts)，
 *    与设定的带宽 ωc 无关 —— 用真实电机参数（L/R 在毫秒级）会导致相位裕度严重不足。
 *    若更换电机参数后电流环振荡，应改按 kp = L·ωc、ki = R·ωc·ts 重新整定。
 */
static void motor_tune_current_loop(void)
{
    float wc_sq = powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2);
    float ts = motor0.cfg.mcl.physical.time.current_loop_ts;

    motor0.cfg.control.currentd_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.currentd_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.currentd_pid_cfg.cfg.output_max = 15;
    motor0.cfg.control.currentd_pid_cfg.cfg.output_min = -15;
    motor0.cfg.control.currentd_pid_cfg.cfg.kp = MOTOR0_LS * wc_sq * ts * 1.5f;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = MOTOR0_RES * wc_sq * ts * 1.5f;

    motor0.cfg.control.currentq_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.currentq_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.currentq_pid_cfg.cfg.output_max = 15;
    motor0.cfg.control.currentq_pid_cfg.cfg.output_min = -15;
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = MOTOR0_LS * wc_sq * ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = MOTOR0_RES * wc_sq * ts * 1.5f;
}

static void motor_cfg_speed_position_loop(void)
{
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 5;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -5;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI;

    motor0.cfg.control.position_pid_cfg.cfg.integral_max = 10;
    motor0.cfg.control.position_pid_cfg.cfg.integral_min = -10;
    motor0.cfg.control.position_pid_cfg.cfg.output_max = 50;
    motor0.cfg.control.position_pid_cfg.cfg.output_min = -50;
    motor0.cfg.control.position_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_POSITION_KP;
    motor0.cfg.control.position_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_POSITION_KI;
}

static void motor_cfg_callbacks(void)
{
    motor0.cfg.control.callback.init = mcl_control_init;
    motor0.cfg.control.dead_area_compensation_cfg.cfg.lowpass_k = 0.1;

    motor0.cfg.drivers.callback.init = mcl_pwm_init;
    motor0.cfg.drivers.callback.enable_all_drivers = mcl_pwm_enable_all;
    motor0.cfg.drivers.callback.disable_all_drivers = mcl_pwm_disable_all;
    motor0.cfg.drivers.callback.update_duty_cycle = mcl_pwm_duty_set;

    motor0.cfg.analog.callback.init = mcl_adc_init;
    motor0.cfg.analog.callback.update_sample_location = mcl_adc_update_sample_location;
    motor0.cfg.analog.callback.get_value = mcl_adc_value_get;

    motor0.cfg.encoder.callback.init = NULL;
    motor0.cfg.encoder.callback.start_sample = mcl_encoder_start_sample;
    motor0.cfg.encoder.callback.get_theta = mcl_encoder_get_theta;
    motor0.cfg.encoder.callback.get_absolute_theta = mcl_encoder_get_abs_theta;
}

void motor_init(void)
{
    motor_cfg_sampling();
    motor_cfg_physical();
    motor_cfg_time();
    motor_cfg_analog();
    motor_cfg_encoder();
    motor_tune_current_loop();
    motor_cfg_speed_position_loop();
    motor_cfg_callbacks();

    motor0.cfg.loop.mode = mcl_mode_foc;
    motor0.cfg.loop.enable_speed_loop = false;

    motor0.cfg.detect.enable_detect = true;
    motor0.cfg.detect.en_submodule_detect.analog = true;
    motor0.cfg.detect.en_submodule_detect.drivers = true;
    motor0.cfg.detect.en_submodule_detect.encoder = true;
    motor0.cfg.detect.en_submodule_detect.loop = true;
    motor0.cfg.detect.callback.disable_output = mcl_pwm_disable_all;

    hpm_mcl_analog_init(&motor0.analog, &motor0.cfg.analog, &motor0.cfg.mcl);
    hpm_mcl_filter_iir_df1_init(&motor0.encoder_iir, &motor0.cfg.encoder_iir, &motor0.encoder_iir_mem[0]);
    hpm_mcl_encoder_init(&motor0.encoder, &motor0.cfg.mcl, &motor0.cfg.encoder, &motor0.encoder_iir);
    hpm_mcl_drivers_init(&motor0.drivers, &motor0.cfg.drivers);
    hpm_mcl_control_init(&motor0.control, &motor0.cfg.control);
    hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                    &motor0.encoder, &motor0.analog, &motor0.control, &motor0.drivers, NULL, NULL);
    hpm_mcl_detect_init(&motor0.detect, &motor0.cfg.detect, &motor0.loop,
                        &motor0.encoder, &motor0.analog, &motor0.drivers);
    hpm_mcl_enable_dq_axis_decoupling(&motor0.loop);
    hpm_mcl_enable_dead_area_compensation(&motor0.loop);
}

/* ==================== 对外接口 ==================== */

mcl_loop_t *motor_get_loop(void)
{
    return &motor0.loop;
}

float motor_get_vbus(void)
{
    return motor0.cfg.mcl.physical.motor.vbus;
}

void motor_set_current_q(float iq)
{
    mcl_user_value_t val;

    val.enable = true;
    val.value = iq;
    hpm_mcl_loop_set_current_q(&motor0.loop, val);
}

void motor_set_current_d(float id)
{
    mcl_user_value_t val;

    val.enable = true;
    val.value = id;
    hpm_mcl_loop_set_current_d(&motor0.loop, val);
}

void motor_enable_loop(void)
{
    hpm_mcl_loop_enable(&motor0.loop);
}

void motor_encoder_process(uint32_t tick)
{
    hpm_mcl_encoder_process(&motor0.encoder, tick);
}

uint32_t motor_get_loop_tick(void)
{
    return (uint32_t)(motor0.cfg.mcl.physical.time.mcu_clock_tick / drv_pwm_get_frequency());
}

void motor_set_initial_theta(float theta)
{
    hpm_mcl_encoder_set_initial_theta(&motor0.encoder, theta);
}

float motor_get_initial_theta(void)
{
    return motor0.encoder.theta_initial;
}

void motor_detect_loop(void)
{
    hpm_mcl_detect_loop(&motor0.detect);
}

void motor_enable_output(void)
{
    drv_pwm_enable_output();
}

void motor_disable_output(void)
{
    drv_pwm_disable_output();
}

bool motor_get_phase_current(float *ia, float *ib)
{
    if (hpm_mcl_analog_get_value(&motor0.analog, analog_a_current, ia) != mcl_success) {
        return false;
    }
    if (hpm_mcl_analog_get_value(&motor0.analog, analog_b_current, ib) != mcl_success) {
        return false;
    }
    return true;
}

void motor_set_duty_raw(uint8_t phase, float duty)
{
    switch (phase) {
    case 0:
        drv_pwm_set_duty(drv_pwm_chn_a, duty);
        break;
    case 1:
        drv_pwm_set_duty(drv_pwm_chn_b, duty);
        break;
    case 2:
        drv_pwm_set_duty(drv_pwm_chn_c, duty);
        break;
    default:
        break;
    }
}
