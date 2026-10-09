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

/* ---------------- 位置环 ---------------- */
extern volatile float g_pos_abs;       /* 位置环反馈：连续多圈机械角 rad */
extern volatile float g_pos_ref;       /* 位置给定：多圈机械角 rad */
extern volatile float g_pos_err;       /* 位置误差 rad */
extern volatile float g_ref_speed;     /* 位置环/主循环输出的速度给定 rad/s */
extern volatile float g_pos_integral;  /* 位置环积分项（单位 rad·拍，不是输出量） */
extern volatile float g_spd_fdb;       /* 实际机械角速度 rad/s，20kHz ISR 刷新 */
extern volatile float g_iq_ref;        /* 速度环输出 = q 轴电流给定 A */
extern volatile float g_spd_ref_in;    /* 速度环实际收到的给定 rad/s（验证 ref 链路） */
extern volatile float g_speed_err;     /* 速度环误差 = 给定 - 反馈 rad/s */
extern volatile float g_iq_ff;         /* 摩擦前馈分量 A（J-Scope 标定用：看它是否≈iq_need） */

/* ---------------- S 曲线轨迹规划 ----------------
 * 判读顺序（"给了目标但不动"时按这个看）：
 *   1) g_sc_move_cnt 有没有涨 —— 没涨说明主循环根本没下发（或 busy_cnt 在涨：
 *      上一次运动还没走完，规划器按设计拒绝打断）；
 *   2) g_sc_state    0=空闲 1=运行中 2=已到位；恒为 1 说明 t_all 算出来异常大；
 *   3) g_sc_p_ref(=g_pos_ref) 有没有在动 —— 规划器在跑但电机不动，是外环/电流环的事；
 *   4) g_sc_shape    1=七段 2=五段(达amax) 3=四段(三角加速度) 4=五段(达vmax)，
 *      用来确认规划器挑了哪套公式；
 *   5) g_sc_v_cmd 与前馈 g_sc_v_ref 的差 = 外环修正 g_sc_corr，
 *      稳态下 g_sc_corr 应接近 0；长期贴着 ±POS_CORR_MAX 说明前馈不够或负载超预期。 */
extern volatile float    g_sc_t;        /* 本次运动已过时间 s */
extern volatile float    g_sc_v_ref;    /* S 曲线速度前馈 rad/s */
extern volatile float    g_sc_a_ref;    /* S 曲线加速度前馈 rad/s^2 */
extern volatile float    g_sc_v_cmd;    /* 最终速度给定 = 前馈 + 修正 rad/s */
extern volatile float    g_sc_corr;     /* 位置外环修正量 rad/s */
extern volatile uint32_t g_sc_state;    /* 0=空闲 1=运行 2=到位 */
extern volatile uint32_t g_sc_shape;    /* 轨迹形状 1/2/3/4 */
extern volatile float    g_sc_t_all;    /* 本次运动总时长 s */
extern volatile float    g_sc_vp;       /* 本次运动的峰值速度 rad/s */
extern volatile uint32_t g_sc_move_cnt; /* 已成功启动的运动次数 */
extern volatile uint32_t g_sc_busy_cnt; /* 因上一运动未完而被拒绝的次数 */

/* ---- S 曲线"不动/过流"定位三件套（2026-09-20 加）----
 *   g_sc_tick  ：规划器时间拍镜像。必须由 scurve_isr_tick() 在 20kHz 中断里递增。
 *                恒为 0/不涨 ⇒ 中断里根本没调 scurve_isr_tick()，轨迹时间不走，
 *                s_v_cmd 里只有修正项，表现为"给了目标但不动"。
 *   g_sc_dwell ：主循环里 step_response_p 块的停留计数镜像。
 *                在 SCURVE_DWELL_MS→0 之间往复 ⇒ 触发块在正常跑；
 *                恒等于 800 ⇒ 规划器一直没 idle（state 卡 RUN）；
 *                恒等于 0   ⇒ 主循环压根没进到这个块。
 *   g_sc_nan   ：速度给定出现非有限值(NaN/Inf)的次数。
 *                NaN 会让下面的限幅比较全部为假、原样穿透到电流环 → 上电过流。 */
extern volatile uint32_t g_sc_tick;     /* 规划器时间拍（20kHz 递增），不涨即中断没喂它 */
extern volatile uint32_t g_sc_dwell;    /* 主循环停留计数镜像 */
extern volatile uint32_t g_sc_nan;      /* s_v_cmd 非有限值次数（>0 说明穿透限幅，会过流） */

/* ---------------- 速度域 S 曲线（g_scs 的平铺镜像） ----------------
 * 为什么要镜像：g_scs 是结构体（struct scurve_spd_t，72 字节），本工程的
 * 调试信息里 struct 成员没被展开，J-Scope 的 Select Symbols 只能列到 "g_scs"
 * 这一层、点不开 .v/.a/.state，所以按位置域 g_sc_* 的做法把成员摊平成标量。
 * 由 ctrl_foc.c 在 20kHz 中断里每拍镜像一次（scurve_spd_isr_tick() 之后）。
 *
 * 判读（速度模式"看不到 S 曲线"时按这个看）：
 *   1) g_scs_move_cnt 涨没涨 —— 没涨说明 scurve_spd_move() 没被调到
 *      （CAN 0x101 没进来 / 没在 FOC_SPEED_MODE）；
 *   2) g_scs_state  0=空闲 1=斜坡运行中 2=已到位（到位后 v 保持不动）；
 *   3) g_scs_v 应从旧给定平滑爬到新目标；配合 g_scs_a / g_scs_j 看三段：
 *      ① j 为 ±jmax 且 a 线性上升  ② j=0 且 a=±ap（梯形才有）  ③ j 反号且 a 回落；
 *      只有 g_scs_a 一路尖三角(j 只有两个相位) = 走了"三角加速度"分支（Δv 小）；
 *   4) 目标速度在 g_can_spd_cmd，本拍给定 g_ref_speed = g_scs_v。 */
extern volatile float    g_scs_v;        /* 本拍速度给定 rad/s（= S 曲线本体） */
extern volatile float    g_scs_a;        /* 本拍加速度给定 rad/s^2 */
extern volatile float    g_scs_j;        /* 本拍 jerk rad/s^3（±jmax 或 0） */
extern volatile uint32_t g_scs_state;    /* 0=空闲 1=运行 2=到位 */
extern volatile uint32_t g_scs_move_cnt; /* 已启动的斜坡次数（每次改目标 +1） */
extern volatile uint32_t g_scs_tick;     /* 规划器时间拍（20kHz 递增），不涨即中断没喂它 */
extern volatile float    g_scs_v0;       /* 本次斜坡起点速度 rad/s */
extern volatile float    g_scs_dv;       /* 本次斜坡速度增量（带符号）rad/s */
extern volatile float    g_scs_ta;       /* 本次斜坡总时长 s（= 2*tj + tc） */
extern volatile float    g_scs_amax;     /* 本段【生效】的加速度上限 rad/s^2：
                                            升速段 = lim_acc，降速/停机段 = lim_dec */
extern volatile float    g_scs_ap;       /* 本段实际峰值加速度 rad/s^2（Δv 小时 < amax，三角形分支） */
extern volatile float    g_scs_lim_acc;  /* 当前 amax_acc 约束 rad/s^2（CAN SET_ACCEL 下发值） */
extern volatile float    g_scs_lim_dec;  /* 当前 amax_dec 约束 rad/s^2（CAN SET_DECEL 下发值） */
extern volatile uint32_t g_scs_stage2;   /* 1 = 跨零换向的两段式：第一段（减到 0）进行中 */

/*  判读 SET_ACCEL / SET_DECEL 有没有真正落进规划器：
 *   1) 发 SET_ACCEL=xxx 后 g_scs_lim_acc 应立刻变成对应 rad/s^2（0.001rpm/s × 1.0472e-4）；
 *   2) 再发一个升速命令，g_scs_amax 应等于 g_scs_lim_acc，且加速段 g_scs_a 峰值 = g_scs_ap；
 *   3) 发 STOP 或降速命令，g_scs_amax 应切到 g_scs_lim_dec（通常比 acc 大）；
 *   4) 从 +v 直接打到 -v：g_scs_stage2 先变 1（按 dec 减到 0），再变 0（按 acc 反向升）。 */

/* ---------------- 异常捕获（覆盖 SDK weak exception_handler） ----------------
 * SDK 默认实现直接 return epc → CPU 回到出错指令 → 无限重复 trap，表现为"死机但变量冻结"。
 * 这里的强符号版本把 mcause/mepc 记进探针后原地停机，J-Scope 一看便知死在哪条指令。 */
extern volatile uint32_t g_exc_cause;  /* mcause：bit31=1 是中断，低 5 位是异常号（2=非法指令 5=读 fault 7=写 fault） */
extern volatile uint32_t g_exc_epc;    /* mepc：出错指令地址，反汇编 objdump --start-address 可定位到函数 */

/* ---------------- 存活心跳（排查"全 0 不动"用） ---------------- */
extern volatile uint32_t g_heart_isr;     /* 20kHz 中断计数，不递增 = 中断没起来 */
extern volatile uint32_t g_heart_main;    /* 主循环计数，不递增 = main 卡住了或被中断饿死 */

/* ---------------- 20kHz 节拍耗时（区分"main 卡住"与"被中断饿死"） ----------------
 * 判读：g_heart_isr 在涨而 g_heart_main 不动时先看这一组。
 *   预算 = 1/PWM_FREQUENCY = 50us。
 *   g_isr_us_max 接近或超过 50 → 单拍跑不完，main 被饿死属正常结果，
 *     此时要去查中断里什么变慢了（典型：编码器 SPI 从 5us 连读退化成 25us 单字节读，
 *     看 g_seq3_ok 是否为 0）；
 *   g_isr_us_max 远小于 50 而 main 仍不动 → 真是 main 卡在某个调用里，看 g_boot_step。 */
extern volatile uint32_t g_isr_cycles_last; /* 上一拍中断消耗的 CPU 周期 */
extern volatile uint32_t g_isr_cycles_max;  /* 开机以来最长的中断消耗 CPU 周期 */
extern volatile uint32_t g_isr_over_cnt;    /* 超过节拍预算的拍数；持续增加 = 节拍跑不完 */
extern volatile uint32_t g_isr_us_max;      /* 最长中断耗时 us（ISR 里换算，main 死了也有数） */
extern volatile uint32_t g_isr_us_last;     /* 上一拍中断耗时 us */

/* ---- ISR 分段耗时：区分"编码器 SPI 慢"还是"控制环算得慢" ---- */
extern volatile uint32_t g_enc_cycles_max;  /* 编码器读取段（含 SPI）最长周期数 */
extern volatile uint32_t g_loop_cycles_max; /* hpm_mcl_loop 段（含 FOC/三环）最长周期数 */
extern volatile uint32_t g_enc_us_max;      /* 编码器段最长耗时 us */
extern volatile uint32_t g_loop_us_max;     /* 环路段最长耗时 us */

/* ---- 窗口化耗时（20ms 一窗）：区分"启动冷峰值"与"运行期持续超载" ----
 * 上面带 _max 的是开机以来的历史峰值，容易被启动瞬间的一次冷访问污染；
 * 下面这组是【当前窗口】的值，稳定运行后仍在增长才说明真有问题。 */
extern volatile uint32_t g_isr_us_win_max;  /* 本窗口最大整拍耗时 us（预算 50） */
extern volatile uint32_t g_enc_us_win_max;  /* 本窗口最大编码器段耗时 us */
extern volatile uint32_t g_loop_us_win_max; /* 本窗口最大环路段耗时 us */
extern volatile uint32_t g_isr_win_over;    /* 本窗口超时拍数占比 %（>75 即持续超载） */
extern volatile uint32_t g_isr_win_cnt;     /* 已结算的窗口数 */

/* ---- 停滞治理 ---- */
extern volatile uint32_t g_isr_starve_cnt;  /* 检出"主循环停止推进"的窗口数 */
extern volatile uint32_t g_isr_relief_cnt;  /* 应急让路：已跳过的控制拍数 */

/* ---- 主循环步进（排查 while(1) 卡在哪一段） ----
 * 自由跑时 J-Scope 里看它在 1~5 之间跳；**恒等于某个值就是卡在那一段**：
 *   1=dbg_probe_keep/app_monitor  2=app_keeper  3=motor_ban 分支
 *   4=电流环阶跃测试块            5=board_delay_ms(1) */
extern volatile uint32_t g_main_step;

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
extern volatile uint32_t g_mid_retry;    /* 零点校准因结果不合理而重试的次数 */extern volatile uint32_t g_mid_wait_us;  /* 等 ADC DMA 首帧刷新所耗 us；耗满上限(10ms)说明 PWM 触发链没起来 */
extern volatile uint32_t g_seq3_retry;   /* MT6835 连读自检重试次数；等于上限说明自检没过 */
extern volatile uint32_t g_angle_jump;   /* 被合理性检查拦下的角度跳变次数（字节撕裂/干扰） */
extern volatile float    g_boot_vbus;    /* 上电瞬间读到的母线电压 V；明显偏低说明还没爬起来 */

/* ---------------- 运行监护 app_keeper ----------------
 * 判读顺序（"上电不转"时按这个顺序看）：
 *   1) g_keeper_blocked != 0  → MCL 判过真实故障，监护拒绝自愈（保护性停机）；
 *      此时看 g_fault_src 位图：bit0=采样/过流 bit1=环路 bit2=驱动 bit3=编码器。
 *   2) g_keeper_fault_clr > 0 → 自动清过 PWM 硬件故障锁存（上电毛刺误锁），已自愈。
 *   3) g_keeper_out_rec  > 0  → 输出被 detect 的 disable_output 关断过，已恢复。
 *   4) g_keeper_loop_rec > 0  → 控制环 status 曾 fail，已软重启。
 *   5) g_keeper_vbus_upd > 0  → 母线电压冷上电读低了，已按复核值修正；对比 g_boot_vbus 与 g_vbus_now。
 *   四个计数全为 0 且电机不转 → 不是锁死类问题，去看位置环给定与 PWM duty。 */
extern volatile uint32_t g_keeper_fault_clr; /* 自动清除 PWM 硬件故障锁存的次数 */
extern volatile uint32_t g_keeper_out_rec;   /* 恢复被关断 PWM 输出的次数 */
extern volatile uint32_t g_keeper_loop_rec;  /* 控制环软重启次数 */
extern volatile uint32_t g_keeper_vbus_upd;  /* 母线电压被修正的次数 */
extern volatile uint32_t g_keeper_blocked;   /* 0=可自愈；1/2/3=因真实故障拒绝自愈（对应上面①②③） */
extern volatile uint32_t g_keeper_last_cause;/* 最近一次自愈动作原因码：1=清锁存 2=恢复输出 3=重启环 4=修正母线 */
extern volatile float    g_vbus_now;         /* 稳定运行后复核的母线电压 V；与 g_boot_vbus 对比 */

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
