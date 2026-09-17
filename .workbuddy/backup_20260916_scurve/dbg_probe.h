/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file dbg_probe.h
 * @brief J-Scope 观测探针集中定义
 *
 * 本文件里的变量全部"只写不读"（仅供 J-Scope 通过 HSS 直接读 RAM），
 * 不参与任何控制运算。集中放在一处的好处：
 *   1) 加/删探针只改这一个文件，不会把调试变量散落到业务代码里；
 *   2) 链接器 --gc-sections 会因为没人引用而删掉它们，所以提供
 *      dbg_probe_keep() 在主循环里做一次空读，强制保留符号。
 */
#ifndef DBG_PROBE_H
#define DBG_PROBE_H

#include "hw_map.h"

/* ---------------- 编码器 / SPI ---------------- */
extern uint32_t      g_spi_err;        /* 0=正常, 1=第一步失败, 2=第二步失败, 3=传输失败 */
extern uint32_t      g_spi_fail_cnt;   /* SPI 连续失败次数 */
extern uint8_t       g_byte0;          /* 角度寄存器 0x003 */
extern uint8_t       g_byte1;          /* 角度寄存器 0x004 */
extern uint8_t       g_byte2;          /* 角度寄存器 0x005 */
extern uint8_t       g_user_id;        /* MT6835 USER_ID(0x001)，用于验证通信 */
extern uint8_t       g_rx_byte;        /* 最近一次单字节读的返回值 */
extern volatile uint8_t g_seq3_ok;     /* 1=单次连读生效（约 5us），0=回退三次单字节（约 25us） */
extern uint32_t      g_angle_raw;      /* 最近一次原始角度码值（21bit） */
extern float         theta_r;          /* 最近一次机械角 [0, 2π) */

/* ---------------- 相电流 / ADC ---------------- */
extern volatile int32_t g_a;           /* A 相 ADC 原始码值 */
extern volatile int32_t g_b;           /* B 相 ADC 原始码值 */
extern volatile int32_t g_c;           /* C 相 ADC 原始码值 */
extern float         g_raw_u;          /* adc_buff[0][0] */
extern float         g_raw_v;          /* adc_buff[1][0] */
extern float         g_vbus;           /* 母线电压 V */
extern volatile uint32_t g_u16;        /* ADC1 ch1 完整 16 位结果 */
extern volatile uint32_t g_v16;        /* ADC0 ch2 完整 16 位结果 */
extern volatile uint32_t g_mid_u;      /* U 相零点校准值（12bit 有效） */
extern volatile uint32_t g_mid_v;      /* V 相零点校准值（12bit 有效） */
extern volatile uint32_t g_cmp4;       /* PWM CMP[4]，验证 DMA 是否写入 */
extern volatile uint32_t g_cmp5;       /* PWM CMP[5] */

/* ---------------- 位置环 ----------------
 * 【S 曲线观测组合】把下面四个放同一张图即可看出轨迹跟随质量：
 *   位置：g_sc_theta（规划位置，本拍移动的目标点） vs g_pos_abs（实际位置）
 *   速度：g_sc_omega（规划速度，即速度前馈量）   vs g_spd_fdb（实际机械角速度）
 * 轨迹跑完后 g_sc_state 变 2，由接近限速逻辑收拾残差（g_pos_err 收敛到 0）。 */
extern volatile float g_pos_abs;       /* 位置环反馈：连续多圈机械角 rad */
extern volatile float g_pos_ref;       /* 位置给定：多圈机械角 rad */
extern volatile float g_pos_err;       /* 位置误差 rad（跟踪阶段即轨迹跟随误差） */
extern volatile float g_ref_speed;     /* 位置环输出的速度给定 rad/s（= PID + 前馈） */
extern volatile float g_pos_integral;  /* 位置环积分项（单位 rad·拍，不是输出量） */
extern volatile float g_spd_fdb;       /* 实际机械角速度 rad/s，20kHz ISR 刷新 */

/* ---------------- S 曲线规划 ---------------- */
extern volatile uint32_t g_sc_state;   /* 0=IDLE 1=RUN 2=DONE；一直为 0 说明 ctrl_pos_start 没跑到 */
extern volatile float g_sc_theta;      /* 规划位置 rad */
extern volatile float g_sc_omega;      /* 规划速度 rad/s（即速度前馈量） */
extern volatile uint32_t g_sc_abort;   /* 失速保护触发次数；>0 说明电机没跟上轨迹、已退回固定目标模式 */

/* ---------------- 异常捕获（覆盖 SDK weak exception_handler） ----------------
 * SDK 默认实现直接 return epc → CPU 回到出错指令 → 无限重复 trap，表现为"死机但变量冻结"。
 * 这里的强符号版本把 mcause/mepc 记进探针后原地停机，J-Scope 一看便知死在哪条指令。 */
extern volatile uint32_t g_exc_cause;  /* mcause：bit31=1 是中断，低 5 位是异常号（2=非法指令 5=读 fault 7=写 fault） */
extern volatile uint32_t g_exc_epc;    /* mepc：出错指令地址，反汇编 objdump --start-address 可定位到函数 */

/* ---------------- 存活心跳（排查"全 0 不动"用） ---------------- */
extern volatile uint32_t g_heart_isr;     /* 20kHz 中断计数，不递增 = 中断没起来 */
extern volatile uint32_t g_heart_main;    /* 主循环计数，不递增 = main 卡住了 */

/* ---------------- 启动步进追踪（排查 main 卡在哪个初始化调用） ---------------- */
/* main 每调一个初始化函数【之前】把编号写进来；卡死时最后停在第几号，
   就说明卡在那个调用里。编号含义见 main.c 里的注释。 */
extern volatile uint32_t g_boot_step;

/* ---------------- 故障状态 ---------------- */
/* 这三个是最容易被忽略、却最能解释"突然不转"的信号。
   MCL 的 detect 一旦判定故障就会调用 disable_output 关断 PWM，
   电机表现为"顿一下然后彻底不动"，但代码里没有任何提示。
   loop: 2=run(正常) 3=fail；encoder/analog 非 0 即异常（见各自 status 枚举）。 */
extern volatile uint32_t g_loop_status;  /* motor0.loop.status */
extern volatile uint32_t g_enc_status;   /* motor0.encoder.status */
extern volatile uint32_t g_ana_status;   /* motor0.analog.status */
extern volatile uint32_t g_pwm_enabled;  /* 1=PWM 输出使能有效，0=已被关断 */

/* ---------------- 上电 / 硬件故障诊断（冷上电"时转时不转"专用） ----------------
 * g_pwm_fault 是最容易被忽略、也最能解释"烧录后必转、冷上电随机不转"的信号：
 * PWM 的外部过流故障源（PA10，低有效）一旦在模拟前端上电爬升期间被比较器拉低，
 * 就会把六路输出硬件钉死为 0，而且配的是"不自动恢复、需软件清标志"。
 * 此时 CPU、20kHz 中断、心跳全部正常，g_pwm_enabled 仍显示 1，
 * 只有 PWM 的 SR.FAULT 位能看出来——所以必须挂探针。 */
extern volatile uint32_t g_pwm_fault;    /* 1=PWM SR 的 FAULT 位已置（输出被硬件锁死） */
extern volatile uint32_t g_pwm_fault_clr;/* 启动阶段清掉过的故障锁存次数 */
extern volatile uint32_t g_fault_src;    /* MCL detect 判定的故障源位图：bit0=analog bit1=loop bit2=drivers bit3=encoder */
extern volatile uint32_t g_fault_cnt;    /* detect 故障回调次数；>0 说明保护动作过 */
extern volatile uint32_t g_mid_retry;    /* 零点校准因结果不合理而重试的次数 */
extern volatile uint32_t g_mid_wait_us;  /* 等 ADC DMA 首帧刷新所耗 us；耗满上限(10ms)说明 PWM 触发链没起来 */
extern volatile uint32_t g_seq3_retry;   /* MT6835 连读自检重试次数；等于上限说明自检没过 */
extern volatile uint32_t g_angle_jump;   /* 被合理性检查拦下的角度跳变次数（字节撕裂/干扰） */
extern volatile float    g_boot_vbus;    /* 上电瞬间读到的母线电压 V；明显偏低说明还没爬起来 */

/* ---------------- 其他 ---------------- */
extern volatile float g_theta_initial; /* 对齐得到的编码器零点偏移 rad */

/* ---------------- FLASH 参数区 ---------------- */
/* 定义在 driver/drv_flash.c：判断"零点偏移到底是从 FLASH 读到的还是用的默认值" */
extern volatile int32_t g_flash_status;  /* 最近一次读/写的 hpm_stat_t，0=成功 */
extern volatile uint8_t g_flash_valid;   /* 1=FLASH 中有有效标定数据，0=用的编译期常量 */
extern volatile float   g_flash_theta;   /* 本次实际使用的零点偏移 rad */
extern volatile uint32_t g_flash_erase_ms;     /* 上次擦写耗时 ms，0=本次上电没擦写 */
extern volatile uint32_t g_flash_erase_cycles; /* 上次擦写耗时 CPU 周期 */
extern volatile uint32_t g_flash_ofs;          /* 实际使用的参数区偏移（相对 FLASH 起始） */

/** 用于兜住上述符号，防止被 --gc-sections 丢弃 */
extern volatile float g_probe_keep;

/**
 * @brief 在主循环里调用：空读一遍所有探针，强制链接器保留它们
 */
void dbg_probe_keep(void);

#endif /* DBG_PROBE_H */
