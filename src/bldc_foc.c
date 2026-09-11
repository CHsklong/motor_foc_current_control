/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "board.h"
#include "clock.h"
#include "hpm_spi_drv.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "hpm_debug_console.h"
#include "hpm_sysctl_drv.h"
#include "hpm_mcl_control.h"
#if defined(HPMSOC_HAS_HPMSDK_PWM)
#include "hpm_pwm_drv.h"
#endif
#if defined(HPMSOC_HAS_HPMSDK_PWMV2)
#include "hpm_pwmv2_drv.h"
#endif
#include "hpm_trgm_drv.h"
#if defined(HPMSOC_HAS_HPMSDK_QEI)
#include "hpm_qei_drv.h"
#endif
#ifdef HPMSOC_HAS_HPMSDK_GPTMRV2
#include "hpm_gptmrv2_drv.h"
#else
#include "hpm_gptmr_drv.h"
#endif
#if defined(HPMSOC_HAS_HPMSDK_QEIV2)
#include "hpm_qeiv2_drv.h"
#endif
#include "hpm_clock_drv.h"
#include "hpm_uart_drv.h"
#include "hpm_gpio_drv.h"
//#include "hpm_acmp_drv.h"
#include "hpm_adc_v2.h"
#include "hpm_mcl_loop.h"
#include "hpm_mcl_abz.h"
#include "hpm_mcl_detect.h"
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
#include "hpm_dmav2_drv.h"
#else
#include "hpm_dma_drv.h"
#endif
#include "hpm_dmamux_drv.h"

/*#if defined(HPMSOC_HAS_HPMSDK_VSC) && defined(HPMSOC_HAS_HPMSDK_CLC) && defined(HPMSOC_HAS_HPMSDK_QEOV2)
#define HW_CURRENT_FOC_ENABLE   1*/
/**
 * @brief Enable hardware hybrid loop functionality
 * This macro controls the hardware hybrid loop features including CLC, VSC, and QEO hardware acceleration
 * #define MCL_HARDWARE_HYBRID_LOOP_ENABLE   1
 */
//#endif
#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
#include "hpm_clc_drv.h"
#include "hpm_synt_drv.h"
#endif
#if defined(HW_CURRENT_FOC_ENABLE)
#include "hpm_vsc_drv.h"
#include "hpm_clc_drv.h"
#include "hpm_trgm_soc_drv.h"
#include "hpm_qeov2_drv.h"
#include "hpm_synt_drv.h"
#endif
#if defined(CONFIG_HPM_MONITOR)
#include "monitor.h"
#endif

#define MOTOR0_SPD                  (20.0)  /*r/s   delta:0.1r/s    1-40r/s */
#define CURRENT_SET_TIME_MS    (200)
#define SPEED_MAX              (40)
#define PWM_FREQUENCY               (20000)    //PWM频率
#define PWM_RELOAD                  ((motor_clock_hz/PWM_FREQUENCY) - 1)
#define PWM_DEAD_AREA_TICK   (100)          //pwm死区时间（x/2*PWM输入时钟频率motor_clock_hz）
#define MOTOR0_BLDCPWM              BOARD_BLDCPWM
#define MOTOR0_CURRENT_LOOP_BANDWIDTH (200)
#define ADCU_INDEX 0
#define ADCV_INDEX 1

//模式切换
#define SVPWM_MODE 0
#define FOC_CURRENT_MODE 1
#define USE_VIRTUAL_ANGLE 0
#define motor_ban 0


#ifndef BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP
#define BOARD_BLDC_HW_FOC_SPEED_KP (0.01f)
#define BOARD_BLDC_HW_FOC_SPEED_KI (0.001f)
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP (0.0074f)
#define BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI (0.0001f)
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KP (0.05f)
#define BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KI (0.001f)
#define BOARD_BLDC_HW_FOC_POSITION_KP (34.7f)
#define BOARD_BLDC_HW_FOC_POSITION_KI (0.113f)
#define BOARD_BLDC_SW_FOC_POSITION_KP (154.7f)
#define BOARD_BLDC_SW_FOC_POSITION_KI (0.113f)
#endif

#if defined(HPMSOC_HAS_HPMSDK_QEI)
#define BLDC_MOTOR_QEI_BASE              BOARD_BLDC_QEI_BASE
#endif
#if defined(HPMSOC_HAS_HPMSDK_QEIV2)
#define BLDC_MOTOR_QEI_BASE            BOARD_BLDC_QEIV2_BASE
#endif

volatile ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc_buff[3][BOARD_BLDC_ADC_PMT_DMA_SIZE_IN_4BYTES];  //存放adc采样的缓冲区
volatile ATTR_PLACE_AT_FAST_RAM_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t pwm_buff[6];
int32_t motor_clock_hz;
float abs_position_theta;
typedef struct {
    mcl_encoder_t encoder;
    mcl_filter_iir_df1_t encoder_iir;
    mcl_filter_iir_df1_memory_t encoder_iir_mem[2];
    mcl_analog_t analog;
    mcl_drivers_t drivers;
    mcl_control_t control;
    mcl_loop_t loop;
    mcl_detect_t detect;
#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    mcl_hw_loop_t hw_loop;
#endif
    struct 
    {
        mcl_cfg_t mcl;
        mcl_encoer_cfg_t encoder;
        mcl_filter_iir_df1_cfg_t encoder_iir;
        mcl_filter_iir_df1_matrix_t encoder_iir_mat[2];
        mcl_analog_cfg_t analog;
        mcl_drivers_cfg_t drivers;
        mcl_control_cfg_t control;
        mcl_loop_cfg_t loop;
        mcl_detect_cfg_t detect;
        mcl_hardware_clc_cfg_t clc;
        #if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
                mcl_hw_loop_cfg_t hw_loop;
        #endif
    } cfg;
} motor0_t;

ATTR_PLACE_AT_FAST_RAM_INIT motor0_t motor0;
ATTR_PLACE_AT_FAST_RAM_INIT uint32_t adc_u_midpoint, adc_v_midpoint;
ATTR_PLACE_AT_FAST_RAM_INIT mcl_user_value_t user_set_theta;

hpm_mcl_stat_t enable_all_pwm_output(void);
hpm_mcl_stat_t disable_all_pwm_output(void);
hpm_mcl_stat_t pwm_duty_set(mcl_drivers_channel_t chn, float duty);
void pwm_init(void);
hpm_mcl_stat_t qei_init(void);
hpm_mcl_stat_t adc_init(void);
hpm_mcl_stat_t adc_value_get(mcl_analog_chn_t chn, int32_t *value);
hpm_mcl_stat_t encoder_get_theta(float *theta);           //编码器获取单圈角度
hpm_mcl_stat_t encoder_get_abs_theta(float *theta);       //编码器获取多圈绝对角度
#if defined(HW_CURRENT_FOC_ENABLE)
void motor0_clc_set_currentloop_value(mcl_loop_chn_t chn, int32_t val);
int32_t motor0_clc_float_convert_clc(float realdata);
#endif
void motor0_control_init(void)
{
}

hpm_mcl_stat_t analog_update_sample_location(mcl_analog_chn_t chn, uint32_t tick)
{
    (void)chn;
    (void)tick;
    return mcl_success;
}

hpm_mcl_stat_t encoder_start_sample(void)
{
    return mcl_success;
}


float read_vbus(void)
{
    adc_v2_handle_t adc_v = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE);   //  获取 ADC 句柄
    uint32_t raw;                       //  存放从 DMA buffer 读出的原始 32 位数据
    adc_v2_dma_sample_t sample;         //  用于解析 DMA 数据，提取出触发通道、ADC 通道和结果
    uint16_t adc_val;                   //  12 位有效 ADC 数值（去除对齐/填充位）
    float pin_voltage, vbus;            //  引脚电压和最终母线电压（单位：V）

    /* ① 软件触发 TRG1A，开始采样母线电压 */
    hpm_adc_v2_trigger_pmt_by_sw(adc_v, BOARD_BLDC_ADC_VBUS_TRIG);

    /* ② 等待转换 + DMA 搬运完成(母线电压变化慢，20µs 足够) */
    board_delay_us(20);

    /* ③ 从 DMA buffer 读结果
     *    布局: 每个 TRG 通道占 4 个 word，TRG1A=3 → offset = 3×4 = 12
     *    用 parse 校验 trig_ch 和 adc_ch，防止布局假设错导致读到脏数据 */
    raw = adc_buff[ADCV_INDEX][BOARD_BLDC_ADC_VBUS_TRIG * 4];
    hpm_adc_v2_parse_pmt_dma_word(adc_v, raw, &sample);
    if ((sample.trig_ch != BOARD_BLDC_ADC_VBUS_TRIG) || (sample.adc_ch != BOARD_BLDC_ADC_CH_VBUS)) {
        return -1.0f;   /* 布局不符，返回负数表示读取失败 */
    }

    /* ④ 换算: ADC读数 → ADC引脚电压 → 实际母线电压 */
    adc_val = MCL_GET_ADC_12BIT_VALID_DATA(sample.result);   /* 取 12bit 有效位 */
    pin_voltage = (float)adc_val * 3.3f / 4095.0f;
    vbus = pin_voltage * BOARD_BLDC_VBUS_DIVIDER;

    return vbus;
}

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
/**
 * @brief Convert floating point d/q values to hardware format for CLC module
 *
 * Conversion Logic:
 * - Input range: ±15.0 (floating point)
 * - Output range: 0x80000000 to 0x7FFFFFFF (signed 32-bit integer as uint32_t)
 * - Zero point: 0x00000000 corresponds to 0.0
 * - Maximum: 0x7FFFFFFF corresponds to +15.0
 * - Minimum: 0x80000000 corresponds to -15.0
 *
 * Note: These values have been processed through multiple conversion stages
 * (ADC -> Clarke/Park transforms -> PI controllers) and no longer represent
 * direct physical quantities. Users should adjust the scaling factor (15.0)
 * based on their specific system requirements and control loop characteristics.
 *
 * @param d D-axis current reference (floating point)
 * @param q Q-axis current reference (floating point)
 * @param d_hardware Pointer to store D-axis hardware format value
 * @param q_hardware Pointer to store Q-axis hardware format value
 */
void clc_convert_input(float d, float q, uint32_t *d_hardware, uint32_t *q_hardware)
{
    *d_hardware = (uint32_t)(int32_t)((d / 15.0f) * 2147483647.0f);
    *q_hardware = (uint32_t)(int32_t)((q / 15.0f) * 2147483647.0f);
}

/**
 * @brief Convert hardware format values back to floating point d/q values
 *
 * Conversion Logic:
 * - Input range: 0x80000000 to 0x7FFFFFFF (hardware format as int32_t)
 * - Output range: ±7.0 (floating point)
 * - Zero point: 0x00000000 corresponds to 0.0
 * - Maximum: 0x7FFFFFFF corresponds to +7.0
 * - Minimum: 0x80000000 corresponds to -7.0
 *
 * Note: The output scaling (7.0) is different from input scaling (15.0) to
 * account for system gain and saturation limits. These are normalized values
 * that have lost direct physical meaning through multiple processing stages.
 * Users should calibrate these scaling factors based on actual system behavior
 * and desired control performance.
 *
 * @param ud_hardware D-axis hardware format value
 * @param uq_hardware Q-axis hardware format value
 * @param ud Pointer to store D-axis floating point value
 * @param uq Pointer to store Q-axis floating point value
 */
void clc_convert_output(uint32_t ud_hardware, uint32_t uq_hardware, float *ud, float *uq)
{
    *ud = ((float)(int32_t)ud_hardware / 2147483647) * 7.0f;
    *uq = ((float)(int32_t)uq_hardware / 2147483647) * 7.0f;
}
#endif

void motor_init(void)
{
    motor0.cfg.mcl.physical.board.analog[analog_a_current].adc_reference_vol = 3.3;  //A相adc参考电压   ADC电流换算系数计算系数
    motor0.cfg.mcl.physical.board.analog[analog_a_current].opamp_gain = 20;          //A相运放增益(实际CSA240L放大20倍, 原理图注明; 原代码误写10导致电流偏小一半)
    motor0.cfg.mcl.physical.board.analog[analog_a_current].sample_precision = 4095;  //采样精度
    motor0.cfg.mcl.physical.board.analog[analog_a_current].sample_res = 0.005;        //采样分辨率(10mΩ采样电阻)

    motor0.cfg.mcl.physical.board.analog[analog_b_current].adc_reference_vol = 3.3;   //B相
    motor0.cfg.mcl.physical.board.analog[analog_b_current].opamp_gain = 20;           //CSA240L放大20倍
    motor0.cfg.mcl.physical.board.analog[analog_b_current].sample_precision = 4095;
    motor0.cfg.mcl.physical.board.analog[analog_b_current].sample_res = 0.005;

    motor0.cfg.mcl.physical.board.num_current_sample_res = 2;                       //A、B相采样电阻数量
    motor0.cfg.mcl.physical.board.pwm_dead_time_tick = PWM_DEAD_AREA_TICK;          //pwm死区时间（防止上下桥臂直通）
    motor0.cfg.mcl.physical.board.pwm_frequency = PWM_FREQUENCY;                    //pwm频率
    motor0.cfg.mcl.physical.board.pwm_reload = PWM_RELOAD;                          //pwm重载值
    motor0.cfg.mcl.physical.motor.i_max = 3;                                       //电机最大电流（过流保护阈值；峰值转矩0.6N·m/Kt0.06=10A）
    motor0.cfg.mcl.physical.motor.inertia = 6.2e-6;                                 //电机转动惯量 62g·cm^2=6.2e-6 kg·m^2（当前软件FOC路径未使用）
    motor0.cfg.mcl.physical.motor.ls =  0.00127f;                                    //定子电感 1.27mH(线间)/2=0.635mH/Phase
    motor0.cfg.mcl.physical.motor.pole_num = 4;                                     //电机极对数 8极/2=4
    motor0.cfg.mcl.physical.motor.power = 63;                                       //电机额定功率 63W
    motor0.cfg.mcl.physical.motor.res = 2.49f;                                      //定子电阻 2.49Ω(线间)/2=1.245Ω/Phase (20℃)
    motor0.cfg.mcl.physical.motor.rpm_max = 3000;                                   //电机额定转速 3000r/min
    motor0.cfg.mcl.physical.motor.vbus = read_vbus();                                        //母线电压
    motor0.cfg.mcl.physical.motor.flux = 0.009;                                     //永磁体磁链 6.5V@1000r/min(线间)→0.009Wb
    motor0.cfg.mcl.physical.motor.ld =  0.00127f;                                    //d轴电感 = 相电感（表贴式）
    motor0.cfg.mcl.physical.motor.lq =  0.00127f;                                    //q轴电感 = 相电感（表贴式）
    motor0.cfg.mcl.physical.time.adc_sample_ts = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY);                  //ADC采样周期
    motor0.cfg.mcl.physical.time.current_loop_ts = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY);                //电流环周期
    motor0.cfg.mcl.physical.time.encoder_process_ts = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY);             //编码器处理周期
    motor0.cfg.mcl.physical.time.speed_loop_ts = (MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY)) * 5;            //速度环周期
    motor0.cfg.mcl.physical.time.position_loop_ts = (MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY)) * 20;        //位置环周期
    motor0.cfg.mcl.physical.time.mcu_clock_tick = clock_get_frequency(clock_cpu0);                        //获取cpu0的主频，mcu时钟计算@@@@
    motor0.cfg.mcl.physical.time.pwm_clock_tick = clock_get_frequency(BOARD_BLDC_MOTOR_CLOCK_SOURCE);     //获取pwm时钟源频率

    motor0.cfg.analog.enable_a_current = true;          //使能A相电流采样
    motor0.cfg.analog.enable_b_current = true;          // B
    memset(motor0.cfg.analog.enable_filter, false, MCL_ANALOG_CHN_NUM);  //滤波器初始化为禁用状态
    motor0.cfg.analog.enable_vbus = true;               //使能母线电压

    //encoder设置

    motor0.cfg.encoder.communication_interval_us = 0;   //编码器通信间隔
    motor0.cfg.encoder.disable_start_sample_interrupt = true; //禁用编码器开始采样中断
    motor0.cfg.encoder.period_call_time_s = MCL_FREQUENCY_TO_PERIOD(PWM_FREQUENCY); //编码器调用周期
    motor0.cfg.encoder.precision = BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV; //编码器精度
    motor0.cfg.encoder.speed_abs_switch_m_t = 5;              //M/T法自动切换的绝对速度阈值
    motor0.cfg.encoder.speed_cal_method = encoder_method_m;   //速度计算方法为m法
    motor0.cfg.encoder.timeout_s = 0.5;            //编码器超时时间

    /**
     * @brief loop pass fpass 100 fstop 2000，二阶低通滤波器
     *
     */
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
    
    motor0.cfg.control.callback.init = motor0_control_init;
    //PI控制参数
    motor0.cfg.control.currentd_pid_cfg.cfg.integral_max = 100;                  //积分限幅
    motor0.cfg.control.currentd_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.currentd_pid_cfg.cfg.output_max = 15;                     //输出限幅
    motor0.cfg.control.currentd_pid_cfg.cfg.output_min = -15;
if(0)   //源码PI参数设定（旧公式：kp/ki 都多乘了 ωc 与 ts，2026-09-10 停用，仅备查）
{
    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;         //K_p=电感*(带宽）^2*50us*1.5
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;         //K_i=电阻*(带宽）^2*50us*1.5
}
if(1)   //理论PI参数设定：标准整定 Kp=Ls*ωc、Ki=Rs*ωc*ts（PI零点对消电机极点 R/L）
{
    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls * MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI;    //K_p=电感*带宽
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts;          //K_i=电阻*带宽*50us
}
    motor0.cfg.control.currentq_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.currentq_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.currentq_pid_cfg.cfg.output_max = 15;
    motor0.cfg.control.currentq_pid_cfg.cfg.output_min = -15;
if(0)   //源码PI参数设定（旧公式，2026-09-10 停用，仅备查）
{
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
}
if(1)   //理论PI参数设定：标准整定 Kp=Ls*ωc、Ki=Rs*ωc*ts（PI零点对消电机极点 R/L）
{
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls * MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI;    //K_p=电感*带宽
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts;          //K_i=电阻*带宽*50us
}
    motor0.cfg.control.dead_area_compensation_cfg.cfg.lowpass_k = 0.1;   //死区补偿低通滤波器系数
 
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 20;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -20;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 10;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -10;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_HW_FOC_SPEED_KI;
#else
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 5;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -5;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI;
#endif

    motor0.cfg.control.position_pid_cfg.cfg.integral_max = 10;
    motor0.cfg.control.position_pid_cfg.cfg.integral_min = -10;
    motor0.cfg.control.position_pid_cfg.cfg.output_max = 50;
    motor0.cfg.control.position_pid_cfg.cfg.output_min = -50;
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.position_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_POSITION_KP;
#else
    motor0.cfg.control.position_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_POSITION_KP;
#endif
    motor0.cfg.control.position_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_POSITION_KI;

    motor0.cfg.drivers.callback.init = pwm_init;    //回调函数调用PWM初始化函数
    motor0.cfg.drivers.callback.enable_all_drivers = enable_all_pwm_output;
    motor0.cfg.drivers.callback.disable_all_drivers = disable_all_pwm_output;
    motor0.cfg.drivers.callback.update_duty_cycle = pwm_duty_set;
    motor0.cfg.analog.callback.init = adc_init;      
    motor0.cfg.analog.callback.update_sample_location = analog_update_sample_location;
    motor0.cfg.analog.callback.get_value = adc_value_get;
#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.encoder.callback.init = NULL;
#else
    //motor0.cfg.encoder.callback.init = qei_init;
    motor0.cfg.encoder.callback.init = NULL;
#endif
    motor0.cfg.encoder.callback.start_sample = encoder_start_sample;
    motor0.cfg.encoder.callback.get_theta = encoder_get_theta;                        //回调函数获取角度值
    motor0.cfg.encoder.callback.get_absolute_theta = encoder_get_abs_theta;

#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.clc.clc_set_val = motor0_clc_set_currentloop_value;
    motor0.cfg.clc.convert_float_to_clc_val = motor0_clc_float_convert_clc;
    motor0.loop.hardware = &motor0.cfg.clc;
#endif

#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.loop.mode = mcl_mode_hardware_foc;
#else
    motor0.cfg.loop.mode = mcl_mode_foc;
#endif
    motor0.cfg.loop.enable_speed_loop = false;      //关闭速度环

    motor0.cfg.detect.enable_detect = true;
    motor0.cfg.detect.en_submodule_detect.analog = true;
    motor0.cfg.detect.en_submodule_detect.drivers = true;
    motor0.cfg.detect.en_submodule_detect.encoder = true;
    motor0.cfg.detect.en_submodule_detect.loop = true;
    motor0.cfg.detect.callback.disable_output = disable_all_pwm_output;

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    /* Hardware hybrid loop mode configuration */
    motor0.cfg.loop.mode = mcl_mode_hybrid_foc;
    motor0.cfg.hw_loop.clc_cfg.base = BOARD_CLC;
    motor0.cfg.hw_loop.callback.clc_convert_input = clc_convert_input;
    motor0.cfg.hw_loop.callback.clc_convert_output = clc_convert_output;
    hpm_mcl_hw_loop_init(&motor0.hw_loop, &motor0.cfg.hw_loop);
    hpm_mcl_enable_clc_hardware_loop(&motor0.hw_loop);
#endif

    hpm_mcl_analog_init(&motor0.analog, &motor0.cfg.analog, &motor0.cfg.mcl);
    hpm_mcl_filter_iir_df1_init(&motor0.encoder_iir, &motor0.cfg.encoder_iir, &motor0.encoder_iir_mem[0]);
    hpm_mcl_encoder_init(&motor0.encoder, &motor0.cfg.mcl, &motor0.cfg.encoder, &motor0.encoder_iir);
    hpm_mcl_drivers_init(&motor0.drivers, &motor0.cfg.drivers);
    hpm_mcl_control_init(&motor0.control, &motor0.cfg.control);
#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                    &motor0.encoder, &motor0.analog, &motor0.control, &motor0.drivers, NULL, &motor0.hw_loop);
#else
    hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                    &motor0.encoder, &motor0.analog, &motor0.control, &motor0.drivers, NULL, NULL);
#endif
    hpm_mcl_detect_init(&motor0.detect, &motor0.cfg.detect, &motor0.loop, &motor0.encoder, &motor0.analog, &motor0.drivers);
    hpm_mcl_enable_dq_axis_decoupling(&motor0.loop);
    hpm_mcl_enable_dead_area_compensation(&motor0.loop);
}

void motor0_speed_loop_para_init(void)
{

#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 20;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -20;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 10;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -10;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_HW_FOC_SPEED_KI;
#else
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 5;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -5;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_SPEED_LOOP_SPEED_KI;

    hpm_mcl_disable_dead_area_compensation(&motor0.loop);

    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
#endif
}

void motor0_position_loop_para_init(void)
{

#if defined(HW_CURRENT_FOC_ENABLE)
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 20;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -20;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 10;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -10;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_HW_FOC_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_HW_FOC_SPEED_KI;
#else
    motor0.cfg.control.speed_pid_cfg.cfg.integral_max = 100;
    motor0.cfg.control.speed_pid_cfg.cfg.integral_min = -100;
    motor0.cfg.control.speed_pid_cfg.cfg.output_max = 5;
    motor0.cfg.control.speed_pid_cfg.cfg.output_min = -5;
    motor0.cfg.control.speed_pid_cfg.cfg.kp = BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KP;
    motor0.cfg.control.speed_pid_cfg.cfg.ki = BOARD_BLDC_SW_FOC_POSITION_LOOP_SPEED_KI;

    hpm_mcl_enable_dead_area_compensation(&motor0.loop);

    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentq_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
                                                (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
                                                motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
#endif
}

hpm_mcl_stat_t pwm_duty_set(mcl_drivers_channel_t chn, float duty)
{
#if !defined(HW_CURRENT_FOC_ENABLE)     //未定义HW_CURRENT_FOC_ENABLE，执行函数；定义了则为空函数，占空比由硬件自动控制
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
//#if 0
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






//hpm_mcl_stat_t encoder_get_theta(float *theta)
//{
//    return hpm_mcl_abz_get_theta(BLDC_MOTOR_QEI_BASE, BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV,\
//        -((MCL_PI * 2) / BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV), theta);
//}
spi_control_config_t g_spi_ctrl_cfg; 
void spi1_config(void)
{
    // 1. 格式配置
    spi_format_config_t format_cfg;
    spi_master_get_default_format_config(&format_cfg);    //预配置

    // MT6835 使用 SPI 模式 3：CPOL=1（空闲高电平）, CPHA=1（在第二个边沿采样）
    format_cfg.common_config.data_len_in_bits = 8;        // MT6835 按字节通信
    format_cfg.common_config.cpol = spi_sclk_high_idle;   // CPOL = 1  SPI使用模式3
    format_cfg.common_config.cpha = spi_sclk_sampling_even_clk_edges; // CPHA = 1 (偶数边沿采样)
    format_cfg.common_config.mode = spi_master_mode; 
    format_cfg.common_config.lsb = false;                 // MSB 优先（最高有效位）
    spi_format_init(HPM_SPI1, &format_cfg);

    // 2. 时序配置（波特率）
    spi_timing_config_t timing_cfg;
    spi_master_get_default_timing_config(&timing_cfg);
    // 获取 SPI1 时钟源频率
    timing_cfg.master_config.clk_src_freq_in_hz = clock_get_frequency(clock_spi1);
    // 线路较差时可降回 4MHz（此时单次连读约 10us，仍在预算内）。
    timing_cfg.master_config.sclk_freq_in_hz = 8000000;  // 4MHz（手册支持最大 16 MHz）
    spi_master_timing_init(HPM_SPI1, &timing_cfg);

    // 3. 控制配置（固定，供后续传输使用） =====
    spi_master_get_default_control_config(&g_spi_ctrl_cfg);
    g_spi_ctrl_cfg.master_config.cmd_enable = false;
    g_spi_ctrl_cfg.master_config.addr_enable = false;
    g_spi_ctrl_cfg.common_config.trans_mode = spi_trans_write_read_together;   // 同时读写
    g_spi_ctrl_cfg.common_config.data_phase_fmt = spi_single_io_mode;  //四线spi
    g_spi_ctrl_cfg.common_config.dummy_cnt = spi_dummy_count_1;      //数据传输前的虚拟字节数量
    g_spi_ctrl_cfg.common_config.cs_index = spi_cs_0;               // 明确使用 CS0（PA26）
}
// 定义 MT6835 的 SPI 命令和寄存器地址
#define MT6835_CMD_READ_REG   0x3   // 4-bit 命令 '0011'
#define MT6835_REG_ANGLE_MSB  0x003 // 角度高字节寄存器地址
#define MT6835_REG_ANGLE_MID  0x004
#define MT6835_REG_ANGLE_LSB  0x005
#define MT6835_REG_USER_ID    0x001   // 用户 ID 寄存器（可用于测试）
// 调试变量（供 J‑Scope 观察）
uint32_t g_spi_err = 0;      // 0=正常, 1=第一步失败, 2=第二步失败
uint8_t  g_byte0 = 0;        // 角度寄存器 0x003 读到的值
uint8_t  g_byte1 = 0;        // 角度寄存器 0x004 读到的值
uint8_t  g_byte2 = 0;        // 角度寄存器 0x005 读到的值
uint8_t  g_user_id = 0;      // 读 USER_ID(0x001) 的值，用于验证通信
uint8_t g_rx_byte = 0; 

// 通过 SPI 读取 MT6835 单字节寄存器函数
static hpm_stat_t mt6835_read_byte(uint16_t reg_addr, uint8_t *data)
{
    uint8_t tx_buf[3] = 
    {
        (MT6835_CMD_READ_REG << 4) | ((reg_addr >> 8) & 0x0F),
        reg_addr & 0xFF,
        0x00   // dummy 字节
    };
    uint8_t rx_buf[3] = {0};
    if (spi_transfer(HPM_SPI1, &g_spi_ctrl_cfg, NULL, NULL, tx_buf, 3, rx_buf, 3) != status_success) 
    {
        g_spi_err = 3;
        return status_fail;
    }
    *data = rx_buf[2];
    g_rx_byte = rx_buf[2];
    return status_success;
}

uint32_t g_spi_fail_cnt = 0;         /* SPI 连续失败次数 */
static uint32_t g_last_angle_raw = 0;/* 上一帧角度，SPI 偶发失败时用它顶住 */

#if 0   /* ================= MT6835 burst read：已停用，改回三次单字节读 =================
 * 以下为 burst 方案（命令 C3~C0='1010'，一次 CS 连读 0x003~0x006，6 字节 @4MHz ≈ 12us），
 * 代码保留备用，需要时把本行 #if 0 改成 #if 1 即可。
 * 注意：启用 burst 时，encoder_get_theta() 里的读角度代码也要一起切回 mt6835_read_angle_raw()。
 */
#define MT6835_CMD_BURST_READ  0xA
uint8_t  g_burst_ok     = 0;   /* 1=burst 生效, 0=已回退到单字节读 */
static uint8_t  g_burst_disable = 0;   /* burst 连续失败后永久回退，避免 12us+30us 叠加拖垮中断 */
static uint8_t  g_burst_miss    = 0;

static bool mt6835_read_angle_raw(uint32_t *raw)
{
    if (!g_burst_disable) {
        uint8_t tx[6] = {
            (uint8_t)((MT6835_CMD_BURST_READ << 4) | ((MT6835_REG_ANGLE_MSB >> 8) & 0x0F)),
            (uint8_t)(MT6835_REG_ANGLE_MSB & 0xFF),
            0, 0, 0, 0                 /* 4 个空字节把 0x003~0x006 时钟出来 */
        };
        uint8_t rx[6] = {0};

        if (spi_transfer(HPM_SPI1, &g_spi_ctrl_cfg, NULL, NULL, tx, 6, rx, 6) == status_success) {
            /* 全 0 = 没应答，全 0xFF = MISO 还在 Hi-Z（命令后需要 shadow 时间），都是无效帧 */
            uint8_t all_zero = ((rx[2] | rx[3] | rx[4]) == 0);
            uint8_t all_ff   = ((rx[2] & rx[3] & rx[4]) == 0xFF);
            if (!all_zero && !all_ff) {
                g_byte0 = rx[2];
                g_byte1 = rx[3];
                g_byte2 = rx[4];       /* rx[5]=CRC, rx[4]&0x07=STATUS(bit1 磁场弱告警) */
                g_burst_ok = 1;
                g_burst_miss = 0;
                *raw = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3);
                return true;
            }
        }
        if (++g_burst_miss >= 3) {
            g_burst_disable = 1;       /* 锁定为单字节模式，不再每帧白试 burst */
        }
    }

    /* 回退：三次单字节读（3 次独立 CS，慢但兼容性最好） */
    uint8_t b[3] = {0};
    if (mt6835_read_byte(MT6835_REG_ANGLE_MSB, &b[0]) != status_success) return false;
    if (mt6835_read_byte(MT6835_REG_ANGLE_MID, &b[1]) != status_success) return false;
    if (mt6835_read_byte(MT6835_REG_ANGLE_LSB, &b[2]) != status_success) return false;
    g_byte0 = b[0];
    g_byte1 = b[1];
    g_byte2 = b[2];
    g_burst_ok = 0;
    *raw = ((uint32_t)b[0] << 13) | ((uint32_t)b[1] << 5) | (b[2] >> 3);
    return true;
}
#endif  /* ------------- MT6835 burst read 结束 ------------- */

/* ============ MT6835 单次连读：一次 CS 读出 0x003 / 0x004 / 0x005 ============
 * 发完"命令+地址"这 2 字节后继续给时钟，若芯片支持地址自增，会依次吐出
 * 0x003/0x004/0x005 三个角度字节，整帧 5 字节 @8MHz ≈ 5us 传输 + 一次固定开销。
 *
 * 相比三次独立单字节读：
 *   1) 省掉 2 次 spi_control_init() 的 FIFO/控制器复位轮询 + 2 次 CS 切换；
 *   2) 三个字节来自同一次角度锁存 —— 不会像分次读那样在字节进位边界上
 *      拼出横跨半个量程的假值（例如 MSB 已翻到 0x80 而 MID/LSB 还是旧值）。
 * 上电自检不通过会自动回退到三次单字节读，见 mt6835_seq3_init()。
 */
static hpm_stat_t mt6835_read_angle_seq3(uint8_t b[3])
{
    uint8_t tx[5] = {
        (uint8_t)((MT6835_CMD_READ_REG << 4) | ((MT6835_REG_ANGLE_MSB >> 8) & 0x0F)),
        (uint8_t)(MT6835_REG_ANGLE_MSB & 0xFF),
        0, 0, 0                        /* 3 个空字节把 0x003/0x004/0x005 时钟出来 */
    };
    uint8_t rx[5] = {0};

    if (spi_transfer(HPM_SPI1, &g_spi_ctrl_cfg, NULL, NULL, tx, 5, rx, 5) != status_success) {
        return status_fail;
    }
    /* 全 0 = 芯片没应答；全 0xFF = MISO 仍高阻（地址未自增，后续字节无效） */
    if (((rx[2] | rx[3] | rx[4]) == 0) || ((rx[2] & rx[3] & rx[4]) == 0xFF)) {
        return status_fail;
    }
    b[0] = rx[2];
    b[1] = rx[3];
    b[2] = rx[4];
    return status_success;
}

volatile uint8_t g_seq3_ok = 0;   /* 1=单次连读生效（J-Scope 可直接观测这个符号） */

/* 上电自检：必须在电机出力之前、轴静止时调用。
 * 静止时"单次连读"与"三次单字节读"应当读到同一个角度；若不一致，说明该芯片
 * 不支持普通读命令下的地址自增（此时 rx[3]/rx[4] 是垃圾），永久回退，
 * 避免在 ADC 中断里把错角度喂给 FOC。
 */
void mt6835_seq3_init(void)
{
    uint8_t pass = 0;

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t a, m, l, s[3];
        if (mt6835_read_byte(MT6835_REG_ANGLE_MSB, &a) != status_success) break;
        if (mt6835_read_byte(MT6835_REG_ANGLE_MID, &m) != status_success) break;
        if (mt6835_read_byte(MT6835_REG_ANGLE_LSB, &l) != status_success) break;
        if (mt6835_read_angle_seq3(s) != status_success) break;

        /* 无符号差值比较以容忍回绕；允许 MT6835 的 LSB 本底抖动 */
        if (((uint8_t)(s[0] - a) > 2) && ((uint8_t)(a - s[0]) > 2)) break;
        if (((uint8_t)(s[1] - m) > 2) && ((uint8_t)(m - s[1]) > 2)) break;
        /* 0x005 的低 3 位是状态位(含磁场弱告警)，只比较高 5 位 */
        if (((uint8_t)((s[2] >> 3) - (l >> 3)) > 1) &&
            ((uint8_t)((l >> 3) - (s[2] >> 3)) > 1)) break;
        pass++;
    }
    g_seq3_ok = (pass == 4) ? 1 : 0;
}

float theta_r = 0.0f;
uint32_t g_angle_raw;
volatile uint8_t g_encoder_isr_enable = 0;   /* 1=由 ADC 中断更新角度，对齐阶段保持 0 */

// 角度读取函数
#define VIRTUAL_FREQ_HZ  10.0f
hpm_mcl_stat_t encoder_get_theta(float *theta)
{
#if USE_VIRTUAL_ANGLE
    // ========== 虚拟角度生成（每次调用增加固定步长）==========
    // 每次ADC中断调用该函数，步长 = 2*pi * 频率 / PWM频率
    static float step = 2.0f * MCL_PI * VIRTUAL_FREQ_HZ / PWM_FREQUENCY;
    static float virtual_theta = 0.0f;
    virtual_theta += step;
    if (virtual_theta >= 2.0f * MCL_PI) {
        virtual_theta -= 2.0f * MCL_PI;
    }
    *theta = -virtual_theta;
    //*theta = 0;                              //测试电流方向
    if (*theta < 0) *theta += 2.0f * MCL_PI;
    theta_r = *theta;  
    return mcl_success;
#else
    uint8_t rx[3] = {0};
    uint32_t angle_raw;
    bool read_ok = true;

    if (g_seq3_ok) 
    {
        /* 单次连读：一次 CS 拿到同一时刻锁存的 3 个字节（约 5us @8MHz） */
        if (mt6835_read_angle_seq3(rx) != status_success) 
        {
            read_ok = false;
        }
    } 
    else 
    {
        /* 回退：三次独立单字节读 0x003 / 0x004 / 0x005，每次一次独立 CS */
        if (mt6835_read_byte(MT6835_REG_ANGLE_MSB, &rx[0]) != status_success) {
            read_ok = false;
        } else if (mt6835_read_byte(MT6835_REG_ANGLE_MID, &rx[1]) != status_success) {
            read_ok = false;
        } else if (mt6835_read_byte(MT6835_REG_ANGLE_LSB, &rx[2]) != status_success) {
            read_ok = false;
        }
    }

    if (!read_ok) {
        /* 偶发 SPI 失败不要直接返回 mcl_fail：
           hpm_mcl_encoder_process() 一旦收到失败会把 encoder->status 永久置成 fail，
           之后每次调用都在入口 MCL_ASSERT 直接返回 not_ready，角度再也不更新、电机锁死。
           这里用上一帧角度顶住，连续失败 5 次才真正上报。 */
        if (++g_spi_fail_cnt > 5) {
            return mcl_fail;
        }
        angle_raw = g_last_angle_raw;
    } else {
        g_spi_fail_cnt = 0;
        g_byte0 = rx[0];
        g_byte1 = rx[1];
        g_byte2 = rx[2];
        angle_raw = ((uint32_t)rx[0] << 13) | ((uint32_t)rx[1] << 5) | (rx[2] >> 3);
        g_last_angle_raw = angle_raw;
    }
    g_angle_raw = angle_raw;
    // 原始角度（0~2π）
    // MT6835 为 21 位分辨率：ANGLE[20:0]，满量程 = 2^21 = 2097152（见数据手册 7.6.8）
    // 注意：angle_raw 已右移 3 位得到 21bit，这里必须除 2^21，除 2^23 会让机械角只有真实的 1/4
    float theta_raw = (float)angle_raw * 2.0f * MCL_PI / 2097152.0f;
    // 极性反转：顺时针旋转 → 角度递增
    *theta = -theta_raw;
    if (*theta < 0) *theta += 2.0f * MCL_PI;
    theta_r = *theta;  // 更新全局变量
    return mcl_success;
#endif
}


hpm_mcl_stat_t encoder_get_abs_theta(float *theta)
{
    //return hpm_mcl_abz_get_abs_theta(BLDC_MOTOR_QEI_BASE, BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV,\
    //    ((MCL_PI * 2) / BOARD_BLDC_QEI_FOC_PHASE_COUNT_PER_REV), -abs_position_theta, theta);
    return encoder_get_theta(theta);   // 直接读取绝对角度

}
volatile int32_t g_a;
volatile int32_t g_b;
volatile int32_t g_c;
static int32_t current_a = 0;   // 缓存 A 相电流，供 B 相重构使用
hpm_mcl_stat_t adc_value_get(mcl_analog_chn_t chn, int32_t *value)
{
    int32_t sens_value;


    switch (chn) {
    case analog_a_current:
        sens_value = MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[ADCU_INDEX][BOARD_BLDC_ADC_TRG*4]);
        g_a = sens_value;
        current_a = adc_u_midpoint - sens_value;   // 计算并缓存 A 相电流
        *value = current_a;
        break;

    case analog_b_current:
        // 实际硬件采样的是 C 相电流（ADCV_INDEX 对应 C 相）
        
        //sens_value = MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[ADCV_INDEX][BOARD_BLDC_ADC_TRG*4]);
        //g_b = sens_value;
        //int32_t current_c = adc_v_midpoint - sens_value;   // C 相电流
        //*value = -(current_a + current_c);                 // B 相 = - (A + C)
        int32_t sens_a = MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[ADCU_INDEX][BOARD_BLDC_ADC_TRG*4]);
        int32_t sens_c = MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[ADCV_INDEX][BOARD_BLDC_ADC_TRG*4]);
        int32_t current_a_local = adc_u_midpoint - sens_a;
        int32_t current_c = adc_v_midpoint - sens_c;
        *value = -(current_a_local + current_c);
        break;
    default:
        return mcl_fail;
    }

    return mcl_success;
}



hpm_mcl_stat_t enable_all_pwm_output(void)
{
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
//#if defined(HPMSOC_HAS_HPMSDK_PWM)
//void pwm_init(void)
//{
//    uint8_t cmp_index = BOARD_BLDCPWM_CMP_INDEX_0;
//    pwm_cmp_config_t cmp_config[4] = {0};
//    pwm_pair_config_t pwm_pair_config = {0};
//    pwm_output_channel_t pwm_output_ch_cfg = {0};
//#ifdef HPMSOC_HAS_HPMSDK_DMAV2
//    dma_channel_config_t config = {0};
//    trgm_output_t trgm_output_cfg;
//#endif
//    pwm_deinit(MOTOR0_BLDCPWM);
//    pwm_stop_counter(MOTOR0_BLDCPWM);
//    pwm_enable_reload_at_synci(MOTOR0_BLDCPWM);
//    pwm_set_reload(MOTOR0_BLDCPWM, 0, PWM_RELOAD);
//    pwm_set_start_count(MOTOR0_BLDCPWM, 0, 0);
//    cmp_config[0].mode = pwm_cmp_mode_output_compare;
//    cmp_config[0].cmp = PWM_RELOAD + 1;
//    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

//    cmp_config[1].mode = pwm_cmp_mode_output_compare;
//    cmp_config[1].cmp = PWM_RELOAD + 1;
//    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;

//    cmp_config[2].enable_ex_cmp  = false;
//    cmp_config[2].mode           = pwm_cmp_mode_output_compare;
//    cmp_config[2].cmp = 5;// PWM_RELOAD/2;                //原来为5
//    cmp_config[2].update_trigger = pwm_shadow_register_update_on_shlk;

//    cmp_config[3].mode = pwm_cmp_mode_output_compare;
//    cmp_config[3].cmp = PWM_RELOAD;
//    cmp_config[3].update_trigger = pwm_shadow_register_update_on_modify;

//    pwm_get_default_pwm_pair_config(MOTOR0_BLDCPWM, &pwm_pair_config);
//    pwm_pair_config.pwm[0].enable_output = true;
//    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
//    pwm_pair_config.pwm[0].invert_output = false;

//    pwm_pair_config.pwm[1].enable_output = true;
//    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = PWM_DEAD_AREA_TICK;
//    pwm_pair_config.pwm[1].invert_output = false;

//    /* Set comparator channel for trigger a */
//    pwm_output_ch_cfg.cmp_start_index = BOARD_BLDC_PWM_TRIG_CMP_INDEX;
//    pwm_output_ch_cfg.cmp_end_index   = BOARD_BLDC_PWM_TRIG_CMP_INDEX;
//    pwm_output_ch_cfg.invert_output   = false;
//    pwm_config_output_channel(MOTOR0_BLDCPWM, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &pwm_output_ch_cfg);

//    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN, &pwm_pair_config, cmp_index, &cmp_config[0], 2)) {
//        printf("failed to setup waveform\n");
//        while(1);
//    }
//    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, &pwm_pair_config, cmp_index+2, &cmp_config[0], 2)) {
//        printf("failed to setup waveform\n");
//        while(1);
//    }
//    if (status_success != pwm_setup_waveform_in_pair(MOTOR0_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, &pwm_pair_config, cmp_index+4, &cmp_config[0], 2)) {
//        printf("failed to setup waveform\n");
//        while(1);
//    }
//    pwm_load_cmp_shadow_on_match(MOTOR0_BLDCPWM, BOARD_BLDCPWM_CMP_TRIG_CMP,  &cmp_config[3]);

//    pwm_config_cmp(MOTOR0_BLDCPWM, BOARD_BLDC_PWM_TRIG_CMP_INDEX, &cmp_config[2]);
//#ifdef HPMSOC_HAS_HPMSDK_DMAV2                                         
//    /* Set comparator channel for trigger a */
//    pwm_output_ch_cfg.cmp_start_index = BOARD_BLDC_DMA_TRG_CMP_INDEX;                              //配置DMA触发比较器
//    pwm_output_ch_cfg.cmp_end_index   = BOARD_BLDC_DMA_TRG_CMP_INDEX;
//    pwm_output_ch_cfg.invert_output   = false;
//    pwm_config_output_channel(MOTOR0_BLDCPWM, BOARD_BLDC_DMA_TRG_CMP_INDEX, &pwm_output_ch_cfg);
//    cmp_config[2].cmp = PWM_RELOAD * 0.75;
//    pwm_config_cmp(MOTOR0_BLDCPWM, BOARD_BLDC_DMA_TRG_CMP_INDEX, &cmp_config[2]);
//#endif
//    /* ADC采样触发点保持cmp=5(计数器≈0,零矢量区间): 该时刻三相下桥均导通续流,
//       低边采样电阻可同时采到A/B两相电流。不要改成PWM_RELOAD/2(有效矢量区),
//       否则只有占空比<50%的相下桥导通,采样失真(表现为A相有波形、B相又小又畸变) */
//    pwm_start_counter(MOTOR0_BLDCPWM);                                           //启动pwm计数
//    pwm_issue_shadow_register_lock_event(MOTOR0_BLDCPWM);
//#ifdef HPMSOC_HAS_HPMSDK_DMAV2
//    trgm_output_cfg.invert = false;                     
//    trgm_output_cfg.type   = trgm_output_pulse_at_input_both_edge;                         //配置触发复用器触发信号
//    trgm_output_cfg.input  = BOARD_BLDC_DMA_TRG_IN;
//    trgm_output_config(BOARD_BLDCPWM_TRGM, BOARD_BLDC_DMA_TRG_DST, &trgm_output_cfg);

//    dmamux_config(BOARD_APP_DMAMUX, DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN),     //配置DMA多路复用器
//                BOARD_BLDC_DMA_MUX_SRC, true);
//    trgm_dma_request_config(BOARD_BLDCPWM_TRGM, BOARD_BLDC_DMA_TRG_INDEX, BOARD_BLDC_DMA_TRG_SRC);

//    dma_default_channel_config(BOARD_APP_DMA0, &config);                                          //配置DMA通道参数

//    config.src_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&pwm_buff);
//    config.dst_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)&MOTOR0_BLDCPWM->CMP[BOARD_BLDCPWM_CMP_INDEX_0]);
//    config.src_width = DMA_TRANSFER_WIDTH_WORD;
//    config.dst_width = DMA_TRANSFER_WIDTH_WORD;
//    config.src_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
//    config.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
//    config.size_in_byte = 24;
//    config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;

//    dma_setup_channel(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, &config, true);
//    dma_set_infinite_loop_mode(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, true);
//    dma_set_handshake_option(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, DMA_HANDSHAKE_OPT_ALL_TRANSIZE);
//#endif
//}
//#endif


#if defined(HPMSOC_HAS_HPMSDK_PWM)
/* PWM v1 初始化（三相六路互补 + 死区 + DMA + 硬件过流保护） */
void pwm_init(void)
{
    uint8_t cmp_index = BOARD_BLDCPWM_CMP_INDEX_0;          // 起始比较器编号
    pwm_cmp_config_t cmp_config[4] = {0};                   // 4个比较器配置
    pwm_pair_config_t pwm_pair_config = {0};                // 互补 PWM 配置结构体
    pwm_output_channel_t pwm_output_ch_cfg = {0};           // 触发输出通道配置
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
    dma_channel_config_t config = {0};                      // DMA 通道配置
    trgm_output_t trgm_output_cfg;                          // TRGM 输出配置
#endif

    /* 1. 复位并停止 PWM 模块 */
    pwm_deinit(MOTOR0_BLDCPWM);
    pwm_stop_counter(MOTOR0_BLDCPWM);
    pwm_enable_reload_at_synci(MOTOR0_BLDCPWM);
    pwm_set_reload(MOTOR0_BLDCPWM, 0, PWM_RELOAD);         // 设置周期（20kHz）
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
    cmp_config[2].cmp = 50;                                  // PWM 周期起点触发 ADC
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
    pwm_pair_config.pwm[0].fault_mode             = pwm_fault_mode_force_output_0; // 故障时强制输出低
    pwm_pair_config.pwm[0].fault_recovery_trigger = pwm_fault_recovery_on_fault_clear; // 清除故障后恢复

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
    fault_config.source_mask              = pwm_fault_source_external_0;  // 启用外部故障通道 0
    fault_config.fault_external_0_active_low = true;                      // TL331 过流拉低 → 低有效
    fault_config.fault_external_1_active_low = false;                     // 未使用 fault_1
    fault_config.fault_recover_at_rising_edge = false;                    // 禁止硬件自动恢复，需软件清除故障标志
    fault_config.fault_output_recovery_trigger = 0;                       // 恢复触发由通道级配置决定
    pwm_config_fault_source(MOTOR0_BLDCPWM, &fault_config);

    /* 9. DMA 相关配置（若启用） */
#ifdef HPMSOC_HAS_HPMSDK_DMAV2
    pwm_output_ch_cfg.cmp_start_index = BOARD_BLDC_DMA_TRG_CMP_INDEX;
    pwm_output_ch_cfg.cmp_end_index   = BOARD_BLDC_DMA_TRG_CMP_INDEX;
    pwm_output_ch_cfg.invert_output   = false;
    pwm_config_output_channel(MOTOR0_BLDCPWM, BOARD_BLDC_DMA_TRG_CMP_INDEX, &pwm_output_ch_cfg);

    cmp_config[2].cmp = PWM_RELOAD * 0.75;   // 在周期 75% 处触发 DMA
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
    config.size_in_byte = 24;   // 6 个 32 位比较寄存器
    config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;

    dma_setup_channel(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, &config, true);
    dma_set_infinite_loop_mode(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, true);
    dma_set_handshake_option(BOARD_APP_DMA0, BOARD_BLDC_DMA_CHN, DMA_HANDSHAKE_OPT_ALL_TRANSIZE);
#endif
}
#endif






#if defined(HPMSOC_HAS_HPMSDK_PWMV2)

#if defined(HW_CURRENT_FOC_ENABLE)

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



float vbus_t;
float vbus_low_threshold;
SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_TMR_IRQ, isr_gptmr)
void isr_gptmr(void)      //定时器中断触发故障检测
{
    if (gptmr_check_status(BOARD_BLDC_TMR_1MS, GPTMR_CH_CMP_IRQ_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP))) {
        gptmr_clear_status(BOARD_BLDC_TMR_1MS, GPTMR_CH_CMP_IRQ_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP));
        hpm_mcl_detect_loop(&motor0.detect);   //检查模拟量、控制环路、功率驱动、编码器/霍尔
    }
    ////模拟欠压故障
    //vbus_t = read_vbus();
    ////vbus_t = 15.0f;             //模拟欠压情况
    //vbus_low_threshold = 18.0f;  //假设欠压下界为18v
    //if(vbus_t<=vbus_low_threshold)
    //{
    // disable_all_pwm_output();
    //}
}




static void timer_init(void)
{
    gptmr_channel_config_t config;

    clock_add_to_group(BOARD_BLDC_TMR_CLOCK, 0);
    gptmr_channel_get_default_config(BOARD_BLDC_TMR_1MS, &config);
    config.debug_mode = 0;
    config.reload = BOARD_BLDC_TMR_RELOAD + 1;
    config.cmp[0] = BOARD_BLDC_TMR_RELOAD;
    config.cmp[1] = config.reload;
    gptmr_enable_irq(BOARD_BLDC_TMR_1MS, GPTMR_CH_CMP_IRQ_MASK(BOARD_BLDC_TMR_CH, BOARD_BLDC_TMR_CMP));
    gptmr_channel_config(BOARD_BLDC_TMR_1MS, BOARD_BLDC_TMR_CH, &config, true);
    intc_m_enable_irq_with_priority(BOARD_BLDC_TMR_IRQ, 1);
}

void init_trigger_mux(TRGM_Type * ptr)      
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_same_as_input;
    trgm_output_cfg.input  = BOARD_BLDC_PWM_TRG_ADC;                //PWM触发√
    trgm_output_config(ptr, BOARD_BLDC_TRG_ADC, &trgm_output_cfg);   //将pwm触发信号路由到ADC模块上
}
 
void init_trigger_cfg(void)     //adc抢占模式配置
{
    adc_v2_preempt_config_t pmt_cfg = {0};

    // ------ 配置 U 相（ADC1，通道 U） ------
    pmt_cfg.trig_ch = BOARD_BLDC_ADC_TRG;     //触发源
    pmt_cfg.trig_len = BOARD_BLDC_ADC_PREEMPT_TRIG_LEN;
    pmt_cfg.inten[0] = true;      // 使能中断（用于电流采样）

    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_U;
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE), &pmt_cfg);
    // ------ 配置 V 相（ADC0，通道 V） ------

    pmt_cfg.inten[0] = false;                 // V相不产生中断，只有U相(ADC1)驱动采样中断
    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_V;       //通道
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE), &pmt_cfg);  //模块与通道

        // ------ 配置母线电压（ADC0，通道 VBUS，独立触发通道） ------
    pmt_cfg.trig_ch = BOARD_BLDC_ADC_VBUS_TRIG;   // 使用独立触发 TRG1A
    pmt_cfg.trig_len = 1;
    pmt_cfg.inten[0] = false;                     // 不产生中断
    pmt_cfg.adc_ch[0] = BOARD_BLDC_ADC_CH_VBUS;   // ADC0_IN11（PB08）
    hpm_adc_v2_set_preempt_config(HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE), &pmt_cfg);
}



// SVPWM 实现
static void svpwm(float u_alpha, float u_beta, float Vdc,
                          float *duty_a, float *duty_b, float *duty_c)
{
    float Ts = 1.0f / PWM_FREQUENCY;  // 50us @ 20kHz 周期时间
    float sqrt3 = 1.7320508f;
    
    // 电压归一化（调制比范围 0~1）
    float Vref = sqrtf(u_alpha*u_alpha + u_beta*u_beta);
    float Vmax = Vdc / sqrt3;  // SVPWM 最大线性调制电压
    
    if (Vref < 1e-6f || Vref > Vmax) {
        // 零矢量或过调制保护
        *duty_a = *duty_b = *duty_c = 0.5f;
        return;
    }
    
    // 计算扇区（使用角度）
    float theta = atan2f(u_beta, u_alpha);
    if (theta < 0) theta += 2.0f * MCL_PI;
    int sector = (int)(theta / (MCL_PI / 3.0f)) + 1;
    if (sector > 6) sector = 1;
    
    // 计算 X, Y, Z（标准 SVPWM 时间变量）
    float T1, T2, T0;
    float Vdc_inv = 1.0f / Vdc;
    float Ta, Tb, Tc;
    
    switch(sector) {
        case 1:  // 0°~60°
            T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (2.0f * u_beta / sqrt3) * Vdc_inv;
            break;
        case 2:  // 60°~120°
            T1 = Ts * (-u_alpha + u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (u_alpha + u_beta / sqrt3) * Vdc_inv;
            break;
        case 3:  // 120°~180°
            T1 = Ts * (-u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (u_beta / sqrt3) * Vdc_inv * 2.0f;
            break;
        case 4:  // 180°~240°
            T1 = Ts * (u_beta / sqrt3 - u_alpha) * Vdc_inv;
            T2 = Ts * (-u_alpha - u_beta / sqrt3) * Vdc_inv;
            break;
        case 5:  // 240°~300°
            T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (-2.0f * u_beta / sqrt3) * Vdc_inv;
            break;
        case 6:  // 300°~360°
            T1 = Ts * (u_alpha - u_beta / sqrt3) * Vdc_inv;
            T2 = Ts * (u_alpha + u_beta / sqrt3) * Vdc_inv;
            break;
        default:
            T1 = T2 = 0;
    }
    
    // 过调制限制（T1+T2 <= Ts）
    if (T1 + T2 > Ts) {
        float k = Ts / (T1 + T2);
        T1 *= k;
        T2 *= k;
    }
    T0 = Ts - T1 - T2;
    
    // 计算三相占空比（七段式中心对齐）
    switch(sector) {
        case 1:
            Ta = (T1 + T2 + T0/2) / Ts;
            Tb = (T2 + T0/2) / Ts;
            Tc = (T0/2) / Ts;
            break;
        case 2:
            Ta = (T1 + T0/2) / Ts;
            Tb = (T1 + T2 + T0/2) / Ts;
            Tc = (T0/2) / Ts;
            break;
        case 3:
            Ta = (T0/2) / Ts;
            Tb = (T1 + T2 + T0/2) / Ts;
            Tc = (T2 + T0/2) / Ts;
            break;
        case 4:
            Ta = (T0/2) / Ts;
            Tb = (T1 + T0/2) / Ts;
            Tc = (T1 + T2 + T0/2) / Ts;
            break;
        case 5:
            Ta = (T2 + T0/2) / Ts;
            Tb = (T0/2) / Ts;
            Tc = (T1 + T2 + T0/2) / Ts;
            break;
        case 6:
            Ta = (T1 + T2 + T0/2) / Ts;
            Tb = (T0/2) / Ts;
            Tc = (T1 + T0/2) / Ts;
            break;
        default:
            Ta = Tb = Tc = 0.5f;
    }
    
    // 限幅保护（0~1）
    *duty_a = (Ta < 0.0f) ? 0.0f : ((Ta > 1.0f) ? 1.0f : Ta);
    *duty_b = (Tb < 0.0f) ? 0.0f : ((Tb > 1.0f) ? 1.0f : Tb);
    *duty_c = (Tc < 0.0f) ? 0.0f : ((Tc > 1.0f) ? 1.0f : Tc);
}




//ADC中断，执行电机运转文件
mcl_control_svpwm_duty_t svpwm_duty;
float Vdc = 36.0f;
float Vref = 1.0f;                //给定电压矢量幅值
static float target_freq = 5.0f;   // 目标电频率
static float current_freq = 0.0f;  // 当前实际频率
#define FREQ_RAMP_STEP  0.0005f      // 每步电频率增量
volatile static float theta = 0.0f;
SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_ADC_IRQn, isr_adc)
float svpwma;
float svpwmb;
float svpwmc;
float fault_level;
float g_ia = 0.0f;  // A 相采样电流（安培）
float g_ib = 0.0f;  // B 相采样电流（安培）
float g_ic = 0.0f;  // B 相采样电流（安培
float speed_rad_s;    // 机械角速度，单位 rad/s
float rpm ;           // 转/分钟

void isr_adc(void)
{   
    ////设为0，模拟硬件过流
    //fault_level = gpio_read_pin(HPM_GPIO0, 0,10);
    //if (fault_level == 0)   // 引脚为低电平的时候触发过流保护
    //{ 
    //    disable_all_pwm_output();
    //}
    uint32_t status;
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);
    status = hpm_adc_v2_get_status_flags(adc_u);
    if ((status & HPM_ADC_V2_EVENT_TRIG_COMPLETE) != 0)
    {
        hpm_adc_v2_clear_status_flags(adc_u, HPM_ADC_V2_EVENT_TRIG_COMPLETE);         //清除中断标志

        /* 编码器角度必须和电流环同频(20kHz)更新。放在 main 的 while(1) 里只有 ~1kHz，
           而且传给它的 tick 是按 50us 算的，速度会被放大约 20 倍，
           导致预测角和 dq 解耦前馈 uq += w*pole_num*(Ld*iq+flux) 严重失真。 */
        if (g_encoder_isr_enable) {
            hpm_mcl_encoder_process(&motor0.encoder, motor0.cfg.mcl.physical.time.mcu_clock_tick / PWM_FREQUENCY);
        }
        hpm_mcl_analog_get_value(&motor0.analog, analog_a_current, &g_ia);
        hpm_mcl_analog_get_value(&motor0.analog, analog_b_current, &g_ib);    //实际采样为c相
        g_ic = -g_ia-g_ib ;
        rpm = motor0.encoder.result.speed * 60.0f / (2.0f * MCL_PI); // 转/分钟
        //SVPWM开环转动
        if(SVPWM_MODE)
        {
            if (current_freq < target_freq)                   //软启动
            {
                current_freq += FREQ_RAMP_STEP;
                if (current_freq > target_freq) current_freq = target_freq;
            }
            theta += 2.0f * MCL_PI * current_freq * (1.0f / PWM_FREQUENCY);
            if (theta > 2.0f * MCL_PI)
            {
                theta -= 2.0f * MCL_PI;
            }
            float u_alpha = Vref* cosf(theta); 
            float u_beta  = Vref * sinf(theta);
    
            motor0.loop.control->method.svpwm(u_alpha, u_beta, motor0.cfg.mcl.physical.motor.vbus, &svpwm_duty);
            pwm_duty_set(mcl_drivers_chn_a,svpwm_duty.a);
            pwm_duty_set(mcl_drivers_chn_b,svpwm_duty.b);
            pwm_duty_set(mcl_drivers_chn_c, svpwm_duty.c);
            svpwma = svpwm_duty.a;
            svpwmb = svpwm_duty.b;
            svpwmc = svpwm_duty.c;
        }
        //电流环闭环
        if (FOC_CURRENT_MODE) 
        {   
            hpm_mcl_loop(&motor0.loop);   
        }
    }
    
}


uint32_t test_val;
hpm_mcl_stat_t adc_init(void)
{
    adc_v2_config_t cfg;                                                   //ADC配置结构体
    adc_v2_channel_config_t ch_cfg;                                        //ADC通道配置结构体
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);      //根据ADC基地址获取句柄
    adc_v2_handle_t adc_v = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_V_BASE);
    adc_v2_handle_t adc_vsen = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_CH_VBUS);  //获取母线电压检测句柄

    hpm_adc_v2_get_default_config(adc_u, &cfg);                           //adc模块配置初始化函数（参数初始化）
    board_init_adc_clock(BOARD_BLDC_ADC_U_BASE, true);                    //adc时钟使能
    board_init_adc_clock(BOARD_BLDC_ADC_V_BASE, true);
    board_init_adc_clock(BOARD_BLDC_ADC_W_BASE, true);
    cfg.resolution_bits = BOARD_BLDC_ADC_RES_BITS;                        //分辨率设置（参数配置）
    cfg.conv_mode = adc_v2_conv_mode_preemption;                          //转换模式
    cfg.signal_mode = adc_v2_signal_mode_single_ended;                    //信号模式
    cfg.clock_div = BOARD_BLDC_ADC_CLOCK_DIV;                             //时钟分频
    cfg.sel_sync_ahb = false;                                             //AHB同步
    hpm_adc_v2_init(adc_u, &cfg);                                         //将通用配置结构体转化为底层硬件寄存器配置，并写入硬件
    hpm_adc_v2_init(adc_v, &cfg);

    hpm_adc_v2_get_channel_default_config(adc_u, &ch_cfg);                //ADC通道配置初始化（参数初始化）
    ch_cfg.signal_mode = adc_v2_signal_mode_single_ended;                 //通道参数配置
    ch_cfg.sample_cycle = BOARD_BLDC_ADC_CHANNEL_SAMPLE_CYCLE;
    ch_cfg.ch = BOARD_BLDC_ADC_CH_U;
    hpm_adc_v2_init_channel(adc_u, &ch_cfg);                              //将ADC通道的通用配置转换为硬件寄存器配置并写入硬件。
    ch_cfg.ch = BOARD_BLDC_ADC_CH_V;
    hpm_adc_v2_init_channel(adc_v, &ch_cfg);
      /* 【E249新增】初始化母线电压通道(ADC0 CH11, PB08) */
    ch_cfg.ch = BOARD_BLDC_ADC_CH_VBUS;
    hpm_adc_v2_init_channel(adc_v, &ch_cfg);

    init_trigger_mux(BOARD_BLDCPWM_TRGM);                                 //配置触发信号多路复用器，将PWM的触发信号路由到ADC模块
    //init_trigger_cfg(BOARD_BLDC_ADC_TRG, true);                          
    init_trigger_cfg();                                                    //配置ADC抢占模式触发通道
    hpm_adc_v2_enable_pmt_queue(adc_u, BOARD_BLDC_ADC_TRG);               //使能ADC的抢占模式队列
    hpm_adc_v2_enable_pmt_queue(adc_v, BOARD_BLDC_ADC_TRG);
    hpm_adc_v2_enable_pmt_queue(adc_v, BOARD_BLDC_ADC_VBUS_TRIG);
    hpm_adc_v2_init_pmt_dma(adc_u, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[ADCU_INDEX])); //配置ADC抢占模式的DMA，将ADC的转换结果自动搬运到指定的内存地址
    hpm_adc_v2_init_pmt_dma(adc_v, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)adc_buff[ADCV_INDEX]));
#if defined(HW_CURRENT_FOC_ENABLE) && HPM_ADC_V2_HAS_MOTOR_MODE    //使能ADC的电机模式
    hpm_adc_v2_enable_motor_mode(adc_u);
    hpm_adc_v2_enable_motor_mode(adc_v);
#endif
    ADC16_Type *adc0 = HPM_ADC0;



    return mcl_success;

}


#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
void clc_init(void)
{
    clc_param_config_t clc_param;
    clc_coeff_config_t clc_coeff0;
    mcl_control_pid_cfg_t pid;

    synt_enable_timestamp(HPM_SYNT, true);

    clc_set_sw_inject_dq_mode_enable(BOARD_CLC, clc_vd_chn, true);
    clc_set_sw_inject_dq_mode_enable(BOARD_CLC, clc_vq_chn, true);

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
     *
     */
    pid.kp = 1.4678;
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

    clc_set_expect_adc_value(BOARD_CLC, clc_vd_chn, 1);
    clc_set_expect_adc_value(BOARD_CLC, clc_vq_chn, 1);

    clc_set_irq_enable(BOARD_CLC, clc_vd_chn, clc_irq_calc_done, true);
    clc_set_irq_enable(BOARD_CLC, clc_vq_chn, clc_irq_calc_done, true);

    clc_set_enable(BOARD_CLC, clc_vd_chn, true);
    clc_set_enable(BOARD_CLC, clc_vq_chn, true);
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
    trgm_pos_matrix_config(HPM_TRGM0, BOARD_BLDC_TRGM_POS_MATRIX_TO_VSC, BOARD_BLDC_TRGM_POS_MATRIX_FROM_QEI, true);    /* position invert: motor foreward rotation, position decrease */
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

    /*
    * adc software use 12bit, so need shif more four bit
    */
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
     *
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
    /* config.wave_type = qeo_wave_cosine; */
    config.wave_type = qeo_wave_saddle;
    config.saddle_type = qeo_saddle_standard;
    /* config.saddle_type = qeo_saddle_triple; */
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

void adc_isr_enable(void)    //使能ADC转换完成中断，ADC完成采样之后触发中断
{
    adc_v2_handle_t adc_u = HPM_ADC_V2_HANDLE(BOARD_BLDC_ADC_U_BASE);

    hpm_adc_v2_enable_interrupts(adc_u, HPM_ADC_V2_EVENT_TRIG_COMPLETE);
    intc_m_enable_irq_with_priority(BOARD_BLDC_ADC_IRQn, 1);     //中断优先级
}

void motor_angle_align(void)
{
    mcl_motor_alignment_cfg_t alignment_cfg;

    /* Select three-stage alignment algorithm */
    alignment_cfg.algorithm = mcl_alignment_algorithm_three_stage;

#if defined(HW_CURRENT_FOC_ENABLE)
    /* Hardware FOC specific initialization */
    qeo_enable_software_position_inject(BOARD_BLDC_QEO);
    qeo_software_position_inject(BOARD_BLDC_QEO, 0);
    qeo_disable_software_position_inject(BOARD_BLDC_QEO);
    vsc_sw_inject_pos_value(BOARD_VSC, 0);

    /* Configure three-stage alignment parameters for hardware mode */
    alignment_cfg.config.three_stage.stage1.d_current = 8.0f;    /* High current for stage 1 */
    alignment_cfg.config.three_stage.stage1.q_current = 0.5f;    /* Q-axis disturbance for stage 1 */
    alignment_cfg.config.three_stage.stage1.delay_ms = 500;      /* Stage 1 delay time */

    alignment_cfg.config.three_stage.stage2.d_current = 5.0f;    /* Moderate current for stage 2 */
    alignment_cfg.config.three_stage.stage2.delay_ms = 800;      /* Stage 2 delay time */

    alignment_cfg.config.three_stage.stage3.d_current = 3.0f;    /* Low current for stage 3 */
    alignment_cfg.config.three_stage.stage3.delay_ms = 400;      /* Stage 3 delay time */
#else
    /* Configure three-stage alignment parameters for software mode */
    alignment_cfg.config.three_stage.stage1.d_current = 4.0f;    /* High current for stage 1 */
    alignment_cfg.config.three_stage.stage1.q_current = 0.6f;    /* Q-axis disturbance for stage 1 */
    alignment_cfg.config.three_stage.stage1.delay_ms = 500;      /* Stage 1 delay time */

    alignment_cfg.config.three_stage.stage2.d_current = 1.5f;    /* Moderate current for stage 2 */
    alignment_cfg.config.three_stage.stage2.delay_ms = 800;      /* Stage 2 delay time */

    alignment_cfg.config.three_stage.stage3.d_current = 1.0f;    /* Low current for stage 3 */
    alignment_cfg.config.three_stage.stage3.delay_ms = 400;      /* Stage 3 delay time */
#endif

    alignment_cfg.config.three_stage.final_delay_ms = 100;       /* Final delay */

    /* Call the enhanced alignment function from middleware */
    hpm_mcl_motor_angle_alignment(&motor0.loop, &alignment_cfg);

#if defined(HW_CURRENT_FOC_ENABLE)
    /* Hardware FOC specific post-alignment setup */
    qei_init();
    trigmux_init_3();
#endif
}





void motor_adc_midpoint(void)    //采样获取零点偏移值
{
    uint32_t adc_u_sum = 0;
    uint32_t adc_v_sum = 0;
    uint32_t times = 0;

    do {                          //确保电机静止，PWM未输出时电流为零        
        adc_u_sum += MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[0][BOARD_BLDC_ADC_TRG*4]);
        adc_v_sum += MCL_GET_ADC_12BIT_VALID_DATA(adc_buff[1][BOARD_BLDC_ADC_TRG*4]);
        times++;
        board_delay_ms(1);
        if (times >= CURRENT_SET_TIME_MS) {
            break;
        }
    } while (1);                  
    adc_u_midpoint = adc_u_sum / CURRENT_SET_TIME_MS;
    adc_v_midpoint = adc_v_sum / CURRENT_SET_TIME_MS;  //ADC连续采样200次，取平均值为零点偏移值
}

void mcl_user_delay_us(uint64_t tick)
{
    board_delay_us(tick);
}

float g_vbus =0.0f; //母线电压 
float g_raw_u = 0;
float g_raw_v = 0;   // 在全局定义
volatile uint32_t g_u16 = 0;   // ADC1 ch1 完整16位结果
volatile uint32_t g_v16 = 0;   // ADC0 ch2 完整16位结果
volatile uint32_t g_mid_u = 0; // U相零点校准值(12bit有效)
volatile uint32_t g_mid_v = 0; // V相零点校准值(12bit有效)
volatile uint32_t g_cmp4 = 0;  // PWM CMP[4] 实际寄存器值(W相高侧比较值), 验证DMA是否写入
volatile uint32_t g_cmp5 = 0;  // PWM CMP[5] 实际寄存器值(W相低侧比较值)


volatile float g_probe_keep = 0.0f;   /* 仅用于防止 --gc-sections 丢弃上面的探针符号 */


volatile float g_theta_initial = 0.0f;
#define ENC_SKIP_ALIGN      1           /* 1=跳过对齐，用下面的常量；0=每次上电对齐 */
#define ENC_THETA_INITIAL   4.9862f     /*  实测的初始机械弧度 */

int main(void)
{
    char input_data[100], input_end;
    mcl_user_value_t  user_speed;     //定义结构体速度，包含数据和使能
    mcl_user_value_t user_position;   //定义结构体位置
    uint8_t i;
    uint8_t user_mode;
    float speed;
    int32_t position;
    board_init();                     //初始化开发板√
    init_spi1_pins();                 // 配置编码器引脚
    init_spi1_clock();                // 使能编码器SPI1 时钟
    spi1_config();                    //配置spi外设参数
    init_adc_bldc_pins();             //初始化ADC引脚，用于采集电机相电流√
    init_pwm_pins(MOTOR0_BLDCPWM);    //初始化PWM引脚，用于驱动电机√
    adc_init();                       //adc初始化
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_MOTOR_CLOCK_SOURCE);  //获取电机定时器的时钟频率，存入全局变量,用于计算pwm周期
#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)   //如果满足定义宏的条件，则调用clc_init函数，目前文件里面clc_init是个指针定义，后续需要写一个clc函数，并将函数地址赋给指针
    clc_init();          
#endif
    motor_init();                     //初始化电机FOC控制参数
    disable_all_pwm_output();
    motor_adc_midpoint();             //采样静态零点校准
    adc_isr_enable();                 //使能adc中断，即adc转换完成后会触发中断 
    timer_init();                     //初始化定时器
    init_acmp_pins();
    pwm_init();
////PWM输出
    enable_all_pwm_output();                //使能PWM输出
    mt6835_read_byte(MT6835_REG_USER_ID, &g_user_id);   // 读取用户ID，结果存入 g_user_id

    /* 探测 MT6835 是否支持"普通读命令 + 地址自增"，决定角度读取走单次连读还是三次单字节。
       必须在使能控制环、电机出力之前调用 —— 自检要求轴静止。
       结果写入 g_seq3_ok（J-Scope 可观测）：1=连读生效(约5us)，0=已回退三次单字节(约25us)。 */
    mt6835_seq3_init();
    // 必须先使能控制环再对齐：hpm_mcl_loop() 内部所有计算与 PWM 输出都在 if(loop->enable) 里，
    // 环没使能时对齐不会输出任何电流，转子不会吸合，theta_initial 会取到一个随机角度。
    hpm_mcl_loop_enable(&motor0.loop);  // 使能控制环

    /* 对齐期间必须冻结电角度为 0（等价于 SDK 里的 force_theta(0)）。
       注意 hpm_mcl_encoder.c 里 force_theta 分支被 if(0) 旁路了，一旦中断开始刷新角度，
       force_theta 就失效、转子会被恒力矩拖走，theta_initial 记到随机位置 —— 所以用这个开关替代。 */
    if (ENC_SKIP_ALIGN && (ENC_THETA_INITIAL > 0.0f)) 
    {
        /* 跳过对齐：直接用已标定的零点偏移，转子不再被强行吸到 d 轴 */
        hpm_mcl_encoder_set_initial_theta(&motor0.encoder, ENC_THETA_INITIAL);
        g_theta_initial = ENC_THETA_INITIAL;
        g_encoder_isr_enable = 1;
    } 
    else
    {
        /* 对齐期间必须冻结电角度为 0（等价于 SDK 里的 force_theta(0)）。
           注意 hpm_mcl_encoder.c 里 force_theta 分支被 if(0) 旁路了，一旦中断开始刷新角度，
           force_theta 就失效、转子会被恒力矩拖走，theta_initial 记到随机位置 —— 所以用这个开关替代。 */
        g_encoder_isr_enable = 0;
        motor_angle_align();   // 对齐编码器角度
        g_encoder_isr_enable = 1;   /* 之后由 ADC 中断按 20kHz 刷新角度 */
        g_theta_initial = motor0.encoder.theta_initial;   /* 供 J-Scope 观察零点偏移 */
    }
    mcl_user_value_t id, iq;
    // Q 轴电流 = 转矩电流。它同时决定转速：本工程没开速度环，转速由
    // "电磁转矩 = 摩擦转矩" 的平衡点决定，所以 iq 一加，转速就跟着涨。
    iq.enable = true;
    iq.value = 0.1f;
    hpm_mcl_loop_set_current_q(&motor0.loop, iq);
    // D 轴电流 = 励磁电流。表贴式(SPM)电机 Ld≈Lq，id 不产生转矩 ——
    // 所以加大 id 可以只把"相电流幅值"顶上去（提高 ADC 信噪比），
    // 而转速基本不变。这是在转矩受限、不能再加 iq 时放大电流波形的办法。
    // 相电流幅值 = sqrt(id^2 + iq^2) = sqrt(0.5^2 + 0.1^2) ≈ 0.51 A（≈63 个 ADC 码）
    // 若要退出该测试，把 0.5f 改回 0.0f 即可。
    id.enable = true;
    id.value = 0.5f;
    hpm_mcl_loop_set_current_d(&motor0.loop, id);
       while (1) 
      {
        /* 保持对 J-Scope 探针符号的引用，避免链接器 gc-sections 把它们删掉 */
        g_probe_keep = g_ref_q + g_sens_q + g_ref_d + g_sens_d + g_ud + g_uq + g_theta_e + g_theta_initial;
        g_raw_u = (float)adc_buff[0][0];   
        g_raw_v = (float)adc_buff[1][0];
        //hpm_mcl_encoder_process(&motor0.encoder, motor0.cfg.mcl.physical.time.mcu_clock_tick / PWM_FREQUENCY);

        if(0)//电流采样测试
        {
        ADC16_Type *adc0 = HPM_ADC0;
        g_v16 = adc0->BUS_RESULT[BOARD_BLDC_ADC_CH_V] & 0xFFFF;
        g_u16 = HPM_ADC1->BUS_RESULT[1] & 0xFFFF;
        g_mid_u = adc_u_midpoint;
        g_mid_v = adc_v_midpoint;
        g_cmp4 = MOTOR0_BLDCPWM->CMP[BOARD_BLDCPWM_CMP_INDEX_4];   // 回读W相高侧比较值,确认DMA写入
        g_cmp5 = MOTOR0_BLDCPWM->CMP[BOARD_BLDCPWM_CMP_INDEX_5];   // 回读W相低侧比较值
        }
        if(0)////电流方向确认、转子旋转方向确认
        {
              #define TEST_VREF  0.1f   // 电压幅值

              // 固定角度为 0，即电压矢量指向 A 轴正方向
              float theta_fixed = 0.0f; 
              float u_alpha = TEST_VREF * cosf(theta_fixed);  // = 0.3
              float u_beta  = TEST_VREF * sinf(theta_fixed);  // = 0

              // 调用你的 SVPWM 函数（你代码里已经有了 svpwm 函数）
              float duty_a, duty_b, duty_c;
              svpwm(u_alpha, u_beta, Vdc, &duty_a, &duty_b, &duty_c);

              // 更新占空比
              pwm_duty_set(mcl_drivers_chn_a, duty_a);
              pwm_duty_set(mcl_drivers_chn_b, duty_b);
              pwm_duty_set(mcl_drivers_chn_c, duty_c);

              // 将占空比赋值给全局变量，便于 J-Scope 观察
              svpwma = duty_a;
              svpwmb = duty_b;
              svpwmc = duty_c;
        }
        //保持电机静止
        if(motor_ban)
        {
              pwm_duty_set(mcl_drivers_chn_a,0.5);
              pwm_duty_set(mcl_drivers_chn_b,0.5);
              pwm_duty_set(mcl_drivers_chn_c,0.5);
        }
        board_delay_ms(1);
      }
      return 0;
}


