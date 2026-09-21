/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file main.c
 * @brief 应用入口：初始化各层、选择运行模式、主循环监控
 */
#include "hw_map.h"

#include "hal_bsp.h"
#include "drv_spi.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "drv_timer.h"
#include "sens_encoder.h"
#include "sens_analog.h"
#include "mt6835.h"
#include "motor.h"
#include "motor_hw_foc.h"
#include "ctrl_foc.h"
#include "ctrl_svpwm.h"
#include "traj_s_curve.h"
#include "dbg_probe.h"
#include "drv_flash.h"
#include "app_cfg.h"
#include "app_monitor.h"
#include "app_keeper.h"
#include "can_app.h"

/**
 * @brief 启动流程
 *
 * 顺序不能随意调整：
 *   1) 引脚/时钟 → SPI → ADC，之后才能读编码器与相电流；
 *   2) motor_clock_hz 必须在 motor_init() 之前填好，PWM_RELOAD 依赖它；
 *   3) 静态零点校准必须在 PWM 无输出、电机静止时做；
 *   4) 控制环必须先使能再对齐，否则 hpm_mcl_loop() 内部所有计算与 PWM 输出
 *      都在 if(loop->enable) 里，对齐不会输出电流，转子不会吸合；
 *   5) 对齐完成后要等 20ms 让测速滤波器的假速度尖峰耗散，再正式出力。
 *   6) 复位后必须先等电源/模拟前端/编码器稳定，再做零点校准、再开 PWM 输出。
 *      调试器下载时板子已经上电很久，这段爬升期天然被跳过；冷上电则恰恰相反 ——
 *      "烧录后必转、重新上电时好时坏"的绝大部分原因就在这一步。
 *   7) 【上电快慢】pwm_init() 必须在零点校准之前：相电流 ADC 由 PWM 触发，
 *      PWM 不跑就没有触发，校准里"等首帧"会耗满上限白等。详见下面 pwm_init 处注释。
 */

int main(void)
{
    board_init();                                   /* 开发板基础初始化 */
    board_delay_ms(POWER_UP_SETTLE_MS);             /* 等母线电容充电、电流采样运放/ADC 基准建立、MT6835 完成 POR。之后才允许取零点、读母线、开输出。 */
    bsp_init();                                     /* 引脚配置 */
    spi1_config();                                  /* SPI 外设参数 */
    adc_init();                                     /* ADC 初始化 */
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_MOTOR_CLOCK_SOURCE);  /* PWM 时钟频率 */

#if FLASH_PARAM_ENABLE                              /* 编码器零点偏移从 FLASH 取 */
    float theta_init = drv_flash_load_theta(ENC_THETA_INITIAL, FLASH_PARAM_CALIB_ONCE);
#else
    float theta_init = ENC_THETA_INITIAL;   /* 排查FLASH */
#endif

#if defined(MCL_HARDWARE_HYBRID_LOOP_ENABLE)
    clc_init();
#endif

    g_boot_step = 1U;                               /* 故障排除序号 */
    motor_init();                                   /* 电机参数初始化 */
    g_boot_step = 2U;
    motor_set_loop_mode(MOTOR_RUN_LOOP_MODE);       /* 切换闭环模式 */
    g_boot_step = 3U;
    (void)0;                                        /* 位置环初始化*/
    g_boot_step = 4U;
    pwm_init();                                     /* PWM 初始化 */
    g_boot_step = 5U;
    disable_all_pwm_output();                       /* 零点校准期间关闭输出 */
    g_boot_step = 6U;
    ctrl_set_keeper_hook(app_keeper_update);        /* 主循环停滞时由中断代跑 */
    board_delay_ms(PWM_FAULT_SETTLE_MS);            /* 先等比较器输出稳定，再判断 */
    if (pwm_fault_is_latched())                     /* 如果pwm锁存，清除故障标志*/
    {
        g_pwm_fault = 1U;
        pwm_fault_clear();
        g_pwm_fault_clr++;
    }
    g_boot_step = 7U;
    motor_adc_midpoint();                            /* 采样电流零点校准 */
    g_boot_step = 8U; 
    adc_isr_enable();                                /* 使能 ADC 中断 */
    g_boot_step = 9U;
    timer_init();                                    /* 1ms 定时器，触发中断里的故障检测 */
    g_boot_step = 10U;
    enable_all_pwm_output();                         /* 使能 PWM 输出 */
    g_boot_step = 11U;
    mt6835_read_byte(MT6835_REG_USER_ID, &g_user_id);   /* 读 USER_ID，结果存入 g_user_id */
    g_boot_step = 12U;
    mt6835_seq3_init();                               /*角度读取*/
    can_app_init();                                 /* CAN（MCAN0, 500kbps）：CANTest 遥测与命令 */

    g_boot_step = 13U;
    /* 【2026-09-20 实锤修复】J-Scope 抓到：过流发生时 g_sc_state=0 / g_sc_move_cnt=0 /
       g_sc_v_ref=0 —— 规划器从未启动，过流来自这里的"提前使能"。
       skip 对齐路径下，此刻（step13）零点偏移还没应用（step14）、编码器 20kHz 刷新还没开，
       控制环使能后位置环/速度环拿着未初始化的角度出力 → 按转子停的位置随机地"踢一脚"，
       轻则抖动/不转，重则上电过流。所以只有对齐路径（motor_angle_align 需要吸转子）才提前使能；
       skip 路径统一等 step16（零点已应用 + 20ms 冲洗完成）再使能。 */
    if (!(ENC_SKIP_ALIGN && (theta_init > 0.0f)))
    {
        hpm_mcl_loop_enable(&motor0.loop);  /* 对齐需要出力：仅对齐路径提前使能 */
    }
/* ---- 角度对齐 ---- */
    int theta_need_save = 0;            /* 1=本次跑了对齐，需要把结果存进 FLASH */

    g_boot_step = 14U;
    if (ENC_SKIP_ALIGN && (theta_init > 0.0f))
    {
        /* 跳过对齐：直接用 FLASH 里已标定的零点偏移（首次上电则是 ENC_THETA_INITIAL），
           转子不再被强行吸到 d 轴 */
        hpm_mcl_encoder_set_initial_theta(&motor0.encoder, theta_init);
        g_theta_initial = theta_init;
        g_encoder_isr_enable = 1;
    }
    else
    {
        g_encoder_isr_enable = 0;
        motor_angle_align();          /* 三段式对齐 */
        g_encoder_isr_enable = 1;     /* 之后由 ADC 中断按 20kHz 刷新角度 */
        g_theta_initial = motor0.encoder.theta_initial;   /* 供 J-Scope 观察零点偏移 */
        theta_need_save = 1;          /* 对齐结果存 FLASH，下次上电可直接跳过对齐 */
    }

    /* ===== 上电第一拍假速度尖峰：先冲洗，再出力 */
    g_boot_step = 15U;
    hpm_mcl_loop_disable(&motor0.loop);
    if (theta_need_save && FLASH_PARAM_CALIB_ONCE)
    {
        /* 借这 20ms 冲洗窗口写 FLASH：输出已断、控制环已停，擦写关中断不会失控。
           与已存值相差不到 0.001rad 时会自动跳过，不会每次上电都擦一遍。
           注意：擦写要调 ROM 的 XPI 探测 API，会把 XPI 重配、拖慢取指，
           所以只在 FLASH_PARAM_CALIB_ONCE=1（标定固件）时才做。 */
        drv_flash_save_theta(g_theta_initial, false);
    }
    g_boot_step = 16U;
    board_delay_ms(20);
    /* 【2026-09-21】出力使能已下移到 g_boot_step=21 之前（原来就在这一行）。
       原因：此刻 MCL 自带的位置环还是开着的（motor.c 里 enable_position_loop=true），
       位置参考也还没 rebase，而 motor.h 已写明这种组合的后果是"位置环照样在跑……
       err 是个巨大负值 → 输出饱和 → 积分顶死"。本来关掉位置环就在下面几微秒处，
       正常跑没问题；但只要 main 在这段窗口里被 20kHz 中断卡住（实测停在 g_boot_step=17），
       饱和力矩就会一直被保持 → 速度环积分顶到 i_max=3A → 稳恒 3A → 上电过流。
       使能放到命令侧全部就绪之后，这段窗口内环路是关的，卡住也只是不转，不会过流。 */

/* ---- 运行模式给定 ---- */
    mcl_user_value_t user_speed;      /* 速度环速度给定 */
    mcl_user_value_t id, iq;          /* 电流环电流给定 */
#if !SCURVE_ENABLE
    mcl_user_value_t position;        /* 位置环位置给定（S 曲线模式下由规划器逐拍刷新 g_pos_ref） */
#endif
    if (FOC_CURRENT_MODE)
    {

        /* Q 轴电流 = 转矩电流；D 轴 = 励磁（表贴式 Ld≈Lq，id 不产生转矩，
           只用来抬高相电流幅值以提高 ADC 信噪比） */
        iq.enable = true;
        iq.value = 0.1f;
        hpm_mcl_loop_set_current_q(&motor0.loop, iq);
        id.enable = true;
        id.value = 0.0f;
        hpm_mcl_loop_set_current_d(&motor0.loop, id); 


    }

    /* ===== 位置环 / 速度环必须互斥 */
#if !SCURVE_ENABLE && !CAN_CTRL_ENABLE
    float pos_debug = 0.0f;
#endif
#if SCURVE_ENABLE && !CAN_CTRL_ENABLE
    uint32_t sc_dwell = SCURVE_DWELL_MS;  /* 停留计数（主循环约 1ms 一次）；初值=起始等待窗口 */
    float    sc_dir   = 1.0f;   /* 运动方向：+1 正向，-1 反向（仅往复模式使用） */
    uint32_t sc_armed = 1U;     /* 1=还允许下发运动；单向单次模式下走完一次即置 0 */
#endif
    g_boot_step = 17U;
    if (FOC_POSITION_MODE)
    {
        motor0.cfg.control.position_pid_cfg.integral = 0.0f;   /* 位置环积分清零 */

        encoder_abs_rebase();           /* 以当前机械位置为 0 基准 */
        g_boot_step = 18U;

#if SCURVE_ENABLE
        /* S 曲线模式下位置外环由 ctrl_foc.c 自建（速度前馈 + P/PI 修正），
           MCL 自带的 position_loop 必须关掉：它直接把输出写进 exec_ref.speed，
           中间没有前馈注入点，而 ref_speed 一旦 enable 又是整体覆盖而非叠加。 */
        motor0.cfg.loop.enable_position_loop = false;

        scurve_init(1.0f / (float)PWM_FREQUENCY, SCURVE_VMAX, SCURVE_AMAX, SCURVE_JMAX);  /*约束值设置*/
        scurve_reset(0.0f);             /* 规划器输出对齐到刚 rebase 的 0 基准 */
g_boot_step = 19U;
        /* 前馈结构下外环 P 增益单独整定 */
        motor0.cfg.control.position_pid_cfg.cfg.kp = SCURVE_POS_KP;
g_boot_step = 19U;
        /* 速度给定改由 ctrl_foc.c 每 4kHz 下发，这里必须先把 ref_speed 置为有效，
           否则速度环会取 exec_ref.speed（位置环关闭时恒为 0），电机不转。 */
        user_speed.enable = true;
        user_speed.value  = 0.0f;         //做阶跃为0
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
        g_pos_ref = 0.0f;
g_boot_step = 20U;
#else
        user_speed.enable = false;      /* 防止速度给定盖掉位置环输出 */
        user_speed.value  = 0.0f;
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
        g_boot_step = 19U;
        /* 目标取"相对基准的增量"：方向唯一确定，不会因多圈角回绕而反向 */
        position.enable = true;
        position.value  = POS_TARGET_DELTA;   /* 相对基准的位置增量POS_TARGET_DELTA（机械角 rad） */
        hpm_mcl_loop_set_position(&motor0.loop, position);
        g_boot_step = 20U;
        g_pos_ref = position.value;
#endif
    }
    else if (FOC_SPEED_MODE)
    {
        user_speed.enable = true;
        user_speed.value = 10;    /* 机械角速度 rad/s */
        hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
    }
    hpm_mcl_loop_enable(&motor0.loop);  /* 命令侧全部就绪后才正式出力（原在步 16） */
g_boot_step = 21U;

/* ---- 主循环（约 1kHz）---- */
    while (1)
    {
        g_main_step = 1U;        /* 1=观测/探针段 */
        g_heart_main++;
        dbg_probe_keep();        /* 兜住 J-Scope 探针符号，防止 --gc-sections 删除 */

        app_monitor_update();    /* 观测量 */
        
        g_main_step = 2U;        /* 2=运行监护段 */
        app_keeper_update();     /* 运行监护：把上电期的锁死类故障自动救回来 */

        g_main_step = 22U;       /* 22=CAN 段：取命令 + 100ms 遥测（CAN_CTRL_ENABLE=0 时空实现） */
        can_app_poll();

        g_main_step = 3U;        /* 3=motor_ban 分支 */
        if (motor_ban)
        {
            /* 保持电机静止：输出 50% 占空比 */
            pwm_duty_set(mcl_drivers_chn_a, 0.5);
            pwm_duty_set(mcl_drivers_chn_b, 0.5);
            pwm_duty_set(mcl_drivers_chn_c, 0.5);
        }

        g_main_step = 4U;        /* 4=电流环阶跃测试块 */
        if(step_response_c)                   //电流环阶跃响应测试
        {
        
          id.value = 0.5f;          // 阶跃到 0.5A
          hpm_mcl_loop_set_current_d(&motor0.loop, id);
          board_delay_ms(10);
          id.value = 0.0f;          // 阶跃到 0.0A
          hpm_mcl_loop_set_current_d(&motor0.loop, id);
        }
#if !CAN_CTRL_ENABLE   
  #if !FOC_POSITION_MODE
        if(step_response_s)
        {
          board_delay_ms(2000);
          if(user_speed.value<50)
          {
            user_speed.value += 10;    /* 机械角速度 rad/s */
          }
          g_ref_speed = user_speed.value;
          hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
        }
  #endif
        if(step_response_p)
        {
#if SCURVE_ENABLE
            /* S 曲线运动：走完一段 → 停留 SCURVE_DWELL_MS。
               SCURVE_REPEAT_ENABLE=1：停留后取反方向再走一段（往复，无限循环）；
               SCURVE_REPEAT_ENABLE=0：只走一次 POS_TARGET_DELTA，随后置 sc_armed=0
                 不再安排下一条，由位置环把终点保持住 —— 这样 J-Scope 上是一条完整、
                 可量测的 S 曲线（g_pos_ref 三次曲线 / g_sc_v_ref 梯形 /
                 g_sc_a_ref 分段恒定），g_sc_move_cnt 恒为 1。
               规划器忙时（scurve_move_delta 返回 -1）不打断，等它走完再发下一条。 */
            if (sc_dwell > 0U) {
                sc_dwell--;
            } else if ((sc_armed != 0U) && scurve_is_idle()) {
                if (scurve_move_delta(sc_dir * POS_TARGET_DELTA) == 0) {
#if SCURVE_REPEAT_ENABLE
                    sc_dir = -sc_dir;       /* 往复：反向，下一段往回走 */
#else
                    sc_armed = 0U;          /* 单向单次：本次行程即全部 */
#endif
                }
                sc_dwell = SCURVE_DWELL_MS;
            }
            g_sc_dwell = sc_dwell;      /* J-Scope：递减到 0 触发一次；往复模式下 0↔800 循环 */
#else
            pos_debug += 5;
            board_delay_ms(2000);
            encoder_abs_rebase();           /* 以当前机械位置为 0 基准 */
            position.value = pos_debug;
            g_pos_ref = position.value;
            hpm_mcl_loop_set_position(&motor0.loop, position);
#endif
        }
#endif /* !CAN_CTRL_ENABLE */
        g_main_step = 5U;        /* 5=节流段：等下一个 1ms 节拍 */
        {
            /* 【2026-09-21】原来是 board_delay_ms(1)：48 万拍的纯忙等。
             * 它的致命处不在"慢"，而在"必须由 main 独占累积 480000 个 CPU 拍才算走完"。
             * 20kHz 中断只要把 CPU 吃得比较满（本工程的"应急让路"只跳过控制环，
             * 编码器 SPI 读、S 曲线 tick、ADC 取样仍然逐拍执行，压不下去），
             * main 就永远凑不齐这 48 万拍 —— 实测 g_heart_main 冻在 1、g_main_step 冻在 1，
             * 而 g_heart_isr 一路正常上涨。S 曲线的往复触发只写在 main 里，
             * 于是"电机不转"。
             *
             * 改成"等 g_sys_ms 前进一格"：main 每圈只要求墙钟过 1ms，
             * 中断占多少 CPU 都只是让这一圈变慢，不再让 main 完全推进不了。
             * 节拍语义不变（1 圈 ≈ 1ms，sc_dwell=800 仍约 800ms）。
             *
             * 后面的周期数上限是兜底：万一定时器没起来、g_sys_ms 不走，
             * 也不会像空等那样死在这里（100 万拍 ≈ 2ms @480MHz）。 */
            uint32_t t_ms = g_sys_ms;
            uint32_t t_cy = (uint32_t)hpm_csr_get_core_cycle();
            while ((g_sys_ms == t_ms) &&
                   (((uint32_t)hpm_csr_get_core_cycle() - t_cy) < 1000000U))
            {
            }
        }
    }

    return 0;
}
