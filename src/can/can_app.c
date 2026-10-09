/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file can_app.c
 * @brief CAN 通信（MCAN0，CAN2.0A 500kbps）：E249 上位机协议实现
 *
 * 硬件：E249 板载 SN65HVD234 收发器，PA00=MCAN0_TXD(Pin12)、PA01=MCAN0_RXD(Pin11)，
 *       CANH/CANL 接 CANalyst-II 的 CAN1（或 CAN2），波特率双方都设 500kbps。
 *
 * 本工程走 MCAN0
 *
 * ==================== 协议====================
 *   PC -> 板  0x201 控制：B0 cmd  B1 seq  B2 arg0  B3 arg1(须0)  B4-7 extra(须0)
 *                   0x01 HEARTBEAT  0x02 SELECT_MODE(B2:1速度/2位置)  0x03 ENABLE
 *                   0x04 STOP       0x05 DISABLE      0x06 CLEAR_FAULT
 *   PC -> 板  0x221 运动：B0 cmd  B1 seq  B2-3 reserved(须0)  B4-7 value(int32 小端)
 *                   0x01 SET_SPEED(0.001rpm)  0x02/03 SET_ACCEL/DECEL(0.001rpm/s)
 *                   0x10/11 POS_ABS/POS_REL(0.001°)  0x12 POS_ZERO  0x13 POS_HOLD
 *                   0x20/21 JOG_POS/JOG_NEG（须 100ms 续发，300ms 未收到自动停）
 *   板 -> PC  0x181 ACK（每条命令）：source_class / cmd / seq / result / state / mode / fault
 *   板 -> PC  0x281 快速状态 20ms：state / mode_flags / 实际速度(0.1rpm) /
 *                                   目标速度(0.1rpm) / 实际 Iq(0.001A)
 *   板 -> PC  0x301 慢速状态 100ms：交替两个子帧
 *                   0x01 位置子帧：status_seq + 实际/目标位置 int24(0.001°)
 *                   0x02 诊断子帧：state / mode / test_type / test_stage / comm_status / fault
 *
 * 去重规则：相同「命令类别 + cmd + seq」的重发只回缓存 ACK，不重复执行。
 *
 * 【本阶段已实现 / 未实现】
 *   已实现：0x201 全部控制命令（除 START_TEST/STOP_TEST）、0x221 全部运动命令（速度侧全部生效）、
 *           0x181 ACK、0x281 快速状态、0x301 慢速状态。
 *   未实现：0x241/0x261/0x381 参数在线调试；0x10/0x11 循环测试；0x401 系波形采集。
 *           这几类帧当前静默忽略（上位机对应的功能用不了，但不影响连通与运行控制）。
 *   SET_ACCEL/SET_DECEL 已接入速度域 S 曲线规划器（scurve_spd_set_accel）：
 *   升速段（|目标|>|当前|）用 ACC，降速/STOP 归零用 DEC，跨零换向拆两段（先 DEC 到 0 再 ACC 反向）。
 *   上电默认取 motor_params.h 的 SCURVE_SPD_AMAX_ACC/DEC，下发后以 RAM 值为准（断电恢复）。
 *
 * 注意：PA00/PA01 同时是 UART0 控制台引脚，CAN 初始化会覆盖它 → printf 不可用，调试走 J-Scope。
 * =============================================================================
 */
#include "hw_map.h"
#include "hpm_mcan_drv.h"
#include "motor.h"              /* motor0 */
#include "traj_s_curve.h"       /* scurve_spd_move / scurve_move_delta / scurve_is_idle */
#include "dbg_probe.h"          /* g_spd_fdb / g_ref_speed / g_iq_ref / g_pos_abs / g_pos_ref
                                   g_fault_src / g_pwm_fault */
#include "drv_pwm.h"            /* enable/disable_all_pwm_output / pwm_output_is_enabled
                                   pwm_fault_is_latched / pwm_fault_clear */
#include "app_cfg.h"
#include "can_protocol.h"
#include "can_app.h"
#include "motor_scope.h"     /* 0x401 示波：motor_scope_handle_req / poll_1ms / set_live */
#include <string.h>
#include <math.h>            /* fabsf：位置命令续发判据用 */

#if CAN_CTRL_ENABLE

/* ---------------------------------------------------------------------------
 * 本固件编译期锁定的运行模式。
 * 协议要求 SELECT_MODE 在速度/位置间运行时切换，但本工程两个模式是【编译期宏】互斥的
 * （app_cfg.h:52 有 #error 强制互斥），位置环与速度环走不同的代码路径。
 * 所以这里只能接受编译时所选的那一个模式，另一个返回 ERR_MODE。
 * 真要运行时切模式，需要把两条路径同时编进来并做运行时分派 —— 属于后续重构项。
 * ------------------------------------------------------------------------- */
#if FOC_SPEED_MODE
#define CAN_MODE_SUPPORTED     MODE_SPEED
#elif FOC_POSITION_MODE
#define CAN_MODE_SUPPORTED     MODE_POSITION
#else
#define CAN_MODE_SUPPORTED     MODE_NONE
#endif

/* ---------------- 参数 ---------------- */
#ifndef CAN_BAUDRATE_KHZ
#define CAN_BAUDRATE_KHZ  500
#endif
#define CAN_BAUDRATE          ((uint32_t)CAN_BAUDRATE_KHZ * 1000U)
#define CAN_RX_RING_DEPTH     (8U)        /* 接收环形缓冲深度（命令速率低，足够） */

/* ---------------- 报文 RAM（HPM5361：MCAN 的 Message RAM 在 AHB RAM，必须先注册，
 * 否则 ram_size=0，mcan_init 直接返回 status_invalid_argument(2)。见 hpm_mcan_soc.h） ---------------- */
#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
static uint32_t s_mcan0_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS] ATTR_PLACE_AT(".ahb_sram");
#endif

/* ---------------- 观测计数（J-Scope） ---------------- */
volatile uint32_t g_can_rx_cnt   = 0U;
volatile uint32_t g_can_tx_cnt   = 0U;
volatile uint32_t g_can_drop_cnt = 0U;
volatile uint32_t g_can_overrun  = 0U;
volatile float    g_can_spd_cmd  = 0.0f;
volatile uint32_t g_can_err      = 0U;
volatile uint32_t g_can_init_stat   = 0U;
volatile uint32_t g_can_src_clk_khz = 0U;
volatile uint32_t g_can_state = 0U;   /* 主状态，见 can_protocol.h 的 ST_* */
volatile uint32_t g_can_mode  = 0U;   /* 0未选 / 1速度 / 2位置 */
volatile uint32_t g_can_comm  = 0U;   /* 0等待 / 1正常 / 2心跳超时 */
volatile uint32_t g_can_fault = 0U;   /* 故障位图 FLT_* */

/* ---------------- 接收环形缓冲（ISR 写 / 主循环读，单生产者单消费者） ---------------- */
static mcan_rx_message_t s_rx_msg[CAN_RX_RING_DEPTH];
static volatile uint32_t s_rx_w = 0U;
static volatile uint32_t s_rx_r = 0U;

static volatile bool     s_echo_pending = false;
static uint8_t           s_echo_payload[4];
static volatile uint16_t s_echo_cnt = 0U;

/* ---------------- 协议状态 ---------------- */
static uint32_t s_ms = 0U;                 /* 主循环节拍计数（约 1ms 一拍） */
static uint8_t  s_state = ST_STANDBY;
static uint8_t  s_mode  = MODE_NONE;
static uint8_t  s_comm  = COMM_WAIT;
static uint16_t s_fault = 0U;
static uint8_t  s_test_type  = 0U;         /* 循环测试未实现，恒 0 上报 */
static uint8_t  s_test_stage = 0U;
static uint32_t s_last_hb_ms = 0U;
static bool     s_hb_valid = false;

/* 点动看门狗 */
static uint32_t s_jog_last_ms = 0U;
static uint8_t  s_jog_active  = 0U;        /* 0=无 1=正向 2=反向 */

/* 加减速约束（rad/s^2）：上电取 motor_params.h 的默认值，
   SET_ACCEL / SET_DECEL 改写后立刻同步给速度域 S 曲线规划器（scurve_spd_set_accel）。
   为什么存 rad/s^2 而不是协议单位：规划器只认 rad/s^2，存协议单位每次用都要再换算一次，
   而且 0.001rpm/s 的整数会丢掉换算精度。协议单位只在收发那一刻出现。 */
static float s_accel_rad = SCURVE_SPD_AMAX_ACC;
static float s_decel_rad = SCURVE_SPD_AMAX_DEC;

/* ACK 去重缓存 */
static uint8_t  s_ack_cache[8];
static uint8_t  s_ack_cls = 0U;
static uint8_t  s_ack_cmd = 0U;
static uint8_t  s_ack_seq = 0U;
static bool     s_ack_valid = false;

/* 慢速状态子帧交替与序号 */
static uint8_t  s_slow_phase = 0U;
static uint8_t  s_status_seq = 0U;

/* ---------------- 0x221 位置命令缓存（停稳后续发，防过冲） ---------------- */
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
static volatile uint8_t s_pos_pending       = 0U;
static float            s_pos_pending_delta = 0.0f;
static float            s_pos_zero_offset   = 0.0f;   /* 用户零点（RAM），SET_POSITION_ZERO 设置 */
#endif

/* ---------------- 小端打包工具 ---------------- */
static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static int16_t  rd_i16(const uint8_t *p) { return (int16_t)rd_u16(p); }
static int32_t  rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static void wr_i16(uint8_t *p, int16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)((uint16_t)v >> 8); }
static void wr_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/** 写入有符号 int24（小端），超限饱和 */
static void put_i24(uint8_t *p, int32_t v)
{
    if (v >  8388607L) { v =  8388607L; }
    if (v < -8388608L) { v = -8388608L; }
    uint32_t u = (uint32_t)v & 0x00FFFFFFU;
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
    p[2] = (uint8_t)(u >> 16);
}

/* 浮点定标：四舍五入到 int16，防溢出 */
static int16_t f_to_i16(float x, float scale, float lim)
{
    float v = x * scale;
    if (v >  lim) { v =  lim; }
    if (v < -lim) { v = -lim; }
    v += (v >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)v;
}

/* ---------------- ISR：RXFIFO0 全部帧搬进环形缓冲（只搬运，不解析） ---------------- */
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, can_app_isr)
void can_app_isr(void)
{
    uint32_t flags = mcan_get_interrupt_flags(BOARD_APP_CAN_BASE);
    if ((flags & MCAN_INT_RXFIFO0_NEW_MSG) != 0U)
    {
        for (;;) {
            mcan_rx_message_t rx;
            if (mcan_read_rxfifo(BOARD_APP_CAN_BASE, 0, &rx) != status_success)
            {
                break;                      /* FIFO 读空 */
            }
            uint32_t next = (s_rx_w + 1U) % CAN_RX_RING_DEPTH;
            if (next != s_rx_r)
            {
                s_rx_msg[s_rx_w] = rx;
                s_rx_w = next;
            } else
            {
                g_can_overrun++;            /* 主循环没来得及取 */
            }
        }
    }
    mcan_clear_interrupt_flags(BOARD_APP_CAN_BASE, flags);
}

/* ---------------- 发送 ---------------- */
static void send_frame(uint32_t std_id, uint8_t dlc, const uint8_t *data)
{
    mcan_tx_frame_t tx;
    uint32_t idx = 0U;
    memset(&tx, 0, sizeof(tx));
    tx.std_id = std_id;
    tx.dlc    = dlc;
    memcpy(tx.data_8, data, dlc);
    /* 非阻塞发送：总线无节点应答时不会卡死主循环 */
    if (mcan_transmit_via_txfifo_nonblocking(BOARD_APP_CAN_BASE, &tx, &idx) == status_success) {
        g_can_tx_cnt++;
    }
}

/** 供 debug/motor_scope.c 复用本模块的发送通道（录波数据/趋势/响应都走这里） */
void can_app_send_raw(uint32_t std_id, uint8_t dlc, const uint8_t *data)
{
    send_frame(std_id, dlc, data);
}

/* ---------------- 故障扫描 ---------------- */
static uint16_t scan_fault(void)
{
    uint16_t f = 0U;
    if (pwm_fault_is_latched())  { f |= FLT_PWM_LATCH;    }
    if ((g_fault_src & 0x1U) != 0U) { f |= FLT_MCL_ANALOG;  }
    if ((g_fault_src & 0x2U) != 0U) { f |= FLT_MCL_LOOP;    }
    if ((g_fault_src & 0x4U) != 0U) { f |= FLT_MCL_DRIVER;  }
    if ((g_fault_src & 0x8U) != 0U) { f |= FLT_MCL_ENCODER; }
    if (s_comm == COMM_TIMEOUT)     { f |= FLT_COMM_TIMEOUT; }
    return f;
}

/* ---------------- ACK ---------------- */
static void send_ack(uint8_t cls, uint8_t cmd, uint8_t seq, uint8_t result)
{
    uint8_t d[8];
    d[0] = cls;
    d[1] = cmd;
    d[2] = seq;
    d[3] = result;
    d[4] = s_state;
    d[5] = s_mode;
    wr_u16(&d[6], s_fault);
    memcpy(s_ack_cache, d, 8);
    s_ack_cls = cls; s_ack_cmd = cmd; s_ack_seq = seq; s_ack_valid = true;
    send_frame(CAN_ID_ACK, 8U, d);
}

/** true=是重发（已回缓存 ACK），调用方直接 return，不重复执行 */
static bool dup_check(uint8_t cls, uint8_t cmd, uint8_t seq)
{
    if (s_ack_valid && (s_ack_cls == cls) && (s_ack_cmd == cmd) && (s_ack_seq == seq)) {
        send_frame(CAN_ID_ACK, 8U, s_ack_cache);
        return true;
    }
    return false;
}

/* ---------------- 控制执行体 ---------------- */
/** 速度目标统一入口：走 S 曲线斜坡（或直接给定，取决于 SCURVE_SPD_ENABLE） */
static void apply_speed_target(float v)
{
#if FOC_SPEED_MODE
  #if SCURVE_SPD_ENABLE
    scurve_spd_move(v);
  #else
    mcl_user_value_t cmd;
    cmd.enable = true;
    cmd.value  = v;
    hpm_mcl_loop_set_speed(&motor0.loop, cmd);
    g_ref_speed = v;
  #endif
#else
    (void)v;
#endif
    g_can_spd_cmd = v;
    /* 录波"下一次阶跃触发"：每次下发新目标都算一次阶跃起点 */
    motor_scope_notify_trigger();
}

static uint8_t ctrl_heartbeat(void)
{
    s_last_hb_ms = s_ms;
    s_hb_valid   = true;
    s_comm       = COMM_OK;
    s_fault     &= (uint16_t)~FLT_COMM_TIMEOUT;
    return ACK_OK;
}

static uint8_t ctrl_select_mode(uint8_t arg0)
{
    if (s_state != ST_STANDBY && s_state != ST_READY) {
        return ACK_ERR_STATE;               /* 运行中/测试中禁止切模式 */
    }
    if (arg0 != MODE_SPEED && arg0 != MODE_POSITION) {
        return ACK_ERR_RANGE;
    }
    if (arg0 != CAN_MODE_SUPPORTED) {
        return ACK_ERR_MODE;                /* 本固件编译期锁定为另一种模式 */
    }
    s_mode  = arg0;
    s_state = ST_READY;                     /* 装载对应参数组，等待 ENABLE */
    return ACK_OK;
}

static uint8_t ctrl_enable(void)
{
    if (s_fault != 0U)      { return ACK_ERR_FAULT; }
    if (s_mode == MODE_NONE) { return ACK_ERR_MODE;  }
    if (s_state == ST_TEST)  { return ACK_ERR_STATE; }

    /* 丢弃旧目标：速度目标 0；位置保持当前反馈（位置模式见下） */
#if FOC_SPEED_MODE
    apply_speed_target(0.0f);
#endif
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
    s_pos_pending = 0U;
#endif
    enable_all_pwm_output();
    s_state = ST_ACTIVE;
    return ACK_OK;
}

static uint8_t ctrl_stop(void)
{
    /* 安全归零：目标回到 0，PWM 不关（与 DISABLE 的区别） */
#if FOC_SPEED_MODE
    apply_speed_target(0.0f);
#endif
    s_jog_active = 0U;
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
    s_pos_pending = 0U;
#endif
    s_state = ST_READY;
    return ACK_OK;
}

static uint8_t ctrl_disable(void)
{
    disable_all_pwm_output();
    s_jog_active = 0U;
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
    s_pos_pending = 0U;
#endif
    s_state = ST_STANDBY;
    return ACK_OK;
}

static uint8_t ctrl_clear_fault(void)
{
    if (pwm_fault_is_latched()) {
        pwm_fault_clear();
    }
    s_fault = scan_fault();
    if (s_fault != 0U) {
        return ACK_ERR_FAULT;               /* 故障条件仍然存在 */
    }
    if (s_state == ST_FAULT) {
        s_state = (s_mode != MODE_NONE) ? ST_READY : ST_STANDBY;
    }
    return ACK_OK;
}

static uint8_t mov_set_speed(int32_t mrpm)
{
    if (s_mode != MODE_SPEED)  { return ACK_ERR_MODE;  }
    if (s_state != ST_ACTIVE)  { return ACK_ERR_STATE; }
    if (mrpm >  (int32_t)SPEED_LIMIT_MRPM || mrpm < -(int32_t)SPEED_LIMIT_MRPM) {
        return ACK_ERR_RANGE;               /* ±1000 rpm */
    }
    apply_speed_target((float)mrpm * MRPM_TO_RADS);
    s_jog_active = 0U;
    return ACK_OK;
}

/** 加速度 / 减速度只改【约束】，不改当前给定：
 *  规划器用它们决定下一段斜坡的 amax（升速取 acc、降速/停机取 dec），
 *  正在跑的斜坡会按新限值重新规划剩余行程，速度本身连续、不跳变。
 *  单位换算：协议 0.001 rpm/s × MRPM_TO_RADS(1.0472e-4) = rad/s^2
 *  （rpm/s → rad/s^2 与 rpm → rad/s 是同一个系数 2π/60，只是多一层 0.001 前缀）。 */
static uint8_t mov_set_accel(uint32_t mrps)
{
    if (s_mode != MODE_SPEED)  { return ACK_ERR_MODE; }
    if (mrps < (uint32_t)ACCEL_MIN_MRPMS || mrps > (uint32_t)ACCEL_MAX_MRPMS) {
        return ACK_ERR_RANGE;
    }
    s_accel_rad = (float)mrps * MRPM_TO_RADS;
    scurve_spd_set_accel(s_accel_rad, s_decel_rad);
    return ACK_OK;
}

static uint8_t mov_set_decel(uint32_t mrps)
{
    if (s_mode != MODE_SPEED)  { return ACK_ERR_MODE; }
    if (mrps < (uint32_t)ACCEL_MIN_MRPMS || mrps > (uint32_t)ACCEL_MAX_MRPMS) {
        return ACK_ERR_RANGE;
    }
    s_decel_rad = (float)mrps * MRPM_TO_RADS;
    scurve_spd_set_accel(s_accel_rad, s_decel_rad);
    return ACK_OK;
}

static uint8_t mov_jog(uint8_t dir)
{
    if (s_mode != MODE_SPEED)  { return ACK_ERR_MODE;  }
    if (s_state != ST_ACTIVE)  { return ACK_ERR_STATE; }
    int32_t mrpm = (dir == 1U) ? (int32_t)JOG_SPEED_MRPM : -(int32_t)JOG_SPEED_MRPM;
    apply_speed_target((float)mrpm * MRPM_TO_RADS);
    s_jog_active  = dir;
    s_jog_last_ms = s_ms;
    return ACK_OK;
}

static uint8_t mov_position(uint8_t cmd, int32_t mdeg)
{
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
    if (s_mode != MODE_POSITION) { return ACK_ERR_MODE;  }
    if (s_state != ST_ACTIVE)    { return ACK_ERR_STATE; }

    switch (cmd) {
    case MOV_POS_REL:
        if (mdeg > (int32_t)POS_REL_LIMIT_MDEG || mdeg < -(int32_t)POS_REL_LIMIT_MDEG) {
            return ACK_ERR_RANGE;           /* 相对命令限制 ±180° */
        }
        s_pos_pending_delta = (float)mdeg * MDEG_TO_RAD;
        s_pos_pending = 1U;                 /* 由 pos_cmd_service() 停稳后续发 */
        return ACK_OK;

    case MOV_POS_ABS:
        /* 绝对：单圈归一化 + 最短路径。当前位置以用户零点为基准 */
        {
            float cur = g_pos_abs - s_pos_zero_offset;
            float tgt = (float)mdeg * MDEG_TO_RAD;
            float err = tgt - cur;
            /* 折算到 ±π 取最短路径 */
            while (err >  (float)MCL_PI) { err -= 2.0f * (float)MCL_PI; }
            while (err < -(float)MCL_PI) { err += 2.0f * (float)MCL_PI; }
            s_pos_pending_delta = err;
            s_pos_pending = 1U;
        }
        return ACK_OK;

    case MOV_POS_ZERO:
        /* 仅设置用户位置零点（RAM），不改 FOC 电角度校准零点 */
        s_pos_zero_offset = g_pos_abs;
        return ACK_OK;

    case MOV_POS_HOLD:
        s_pos_pending = 0U;
        return ACK_OK;

    default:
        return ACK_ERR_COMMAND;
    }
#else
    (void)cmd; (void)mdeg;
    return ACK_ERR_MODE;        /* 本固件编译为速度模式，位置命令不可用 */
#endif
}

/* ---------------- 帧解析 ---------------- */
static void handle_ctrl(const mcan_rx_message_t *m)
{
    uint8_t cmd, seq, res;

    if (m->dlc != 8U) {
        send_ack(CLS_CTRL, (m->dlc > 0U) ? m->data_8[0] : 0U,
                           (m->dlc > 1U) ? m->data_8[1] : 0U, ACK_ERR_DLC);
        return;
    }
    cmd = m->data_8[0];
    seq = m->data_8[1];
    if (dup_check(CLS_CTRL, cmd, seq)) { return; }

    /* 保留字必须为零：B3(arg1) 与 B4-7(extra) */
    if ((m->data_8[3] != 0U) || (rd_u16(&m->data_8[4]) != 0U) || (rd_u16(&m->data_8[6]) != 0U)) {
        send_ack(CLS_CTRL, cmd, seq, ACK_ERR_PARAM);
        return;
    }

    switch (cmd) {
    case CMD_HEARTBEAT:   res = ctrl_heartbeat();            break;
    case CMD_SELECT_MODE: res = ctrl_select_mode(m->data_8[2]); break;
    case CMD_ENABLE:      res = ctrl_enable();               break;
    case CMD_STOP:        res = ctrl_stop();                 break;
    case CMD_DISABLE:     res = ctrl_disable();              break;
    case CMD_CLEAR_FAULT: res = ctrl_clear_fault();          break;
    case CMD_START_TEST:
    case CMD_STOP_TEST:   res = ACK_ERR_COMMAND;             break;  /* 循环测试未实现 */
    default:              res = ACK_ERR_COMMAND;             break;
    }
    send_ack(CLS_CTRL, cmd, seq, res);
}

static void handle_move(const mcan_rx_message_t *m)
{
    uint8_t cmd, seq, res;
    int32_t val;

    if (m->dlc != 8U) {
        send_ack(CLS_MOVE, (m->dlc > 0U) ? m->data_8[0] : 0U,
                           (m->dlc > 1U) ? m->data_8[1] : 0U, ACK_ERR_DLC);
        return;
    }
    cmd = m->data_8[0];
    seq = m->data_8[1];
    if (dup_check(CLS_MOVE, cmd, seq)) { return; }

    /* B2-3 reserved 必须为 0 */
    if (rd_u16(&m->data_8[2]) != 0U) {
        send_ack(CLS_MOVE, cmd, seq, ACK_ERR_PARAM);
        return;
    }
    val = rd_i32(&m->data_8[4]);

    switch (cmd) {
    case MOV_SET_SPEED: res = mov_set_speed(val);                 break;
    case MOV_SET_ACCEL: res = mov_set_accel((uint32_t)val);       break;
    case MOV_SET_DECEL: res = mov_set_decel((uint32_t)val);       break;
    case MOV_POS_ABS:
    case MOV_POS_REL:
    case MOV_POS_ZERO:
    case MOV_POS_HOLD:  res = mov_position(cmd, val);             break;
    case MOV_JOG_POS:   res = mov_jog(1U);                        break;
    case MOV_JOG_NEG:   res = mov_jog(2U);                        break;
    default:            res = ACK_ERR_COMMAND;                    break;
    }
    send_ack(CLS_MOVE, cmd, seq, res);
}

static void handle_frame(const mcan_rx_message_t *m)
{
    if (m->use_ext_id)
    {
        return;                             /* 本协议只用标准帧 */
    }
    g_can_rx_cnt++;

    switch (m->std_id)
    {
    case CAN_ID_CTRL:
        handle_ctrl(m);
        break;
    case CAN_ID_MOVE:
        handle_move(m);
        break;
    case CAN_ID_ECHO_REQ:
        /* 自留连通性自检：记下负载，主循环里回 0x1F0（发送不在 ISR 里做） */
        s_echo_payload[0] = m->data_8[0];
        s_echo_payload[1] = m->data_8[1];
        s_echo_payload[2] = m->data_8[2];
        s_echo_payload[3] = m->data_8[3];
        s_echo_pending = true;
        break;
    case SCOPE_ID_REQ:
        /* 0x401 示波请求：ELF 变量录波（见 debug/motor_scope.h） */
        if (m->dlc == 8U) { motor_scope_handle_req(&m->data_8[0]); }
        break;
    default:
        /* 0x241/0x261（参数调试）与 0x401 系（波形采集）本阶段未实现，静默忽略 */
        break;
    }
}

/* ---------------- 周期服务 ---------------- */
/** 心跳看门狗：1 小时无心跳 → 安全停止（速度归零后关 PWM） */
static void watchdog_service(void)
{
    if (!s_hb_valid) { return; }
    if ((uint32_t)(s_ms - s_last_hb_ms) > CAN_HB_TIMEOUT_MS)
    {
        s_comm  = COMM_TIMEOUT;
        s_fault |= FLT_COMM_TIMEOUT;
        apply_speed_target(0.0f);
        disable_all_pwm_output();
        s_jog_active = 0U;
        s_state = ST_STANDBY;
        s_hb_valid = false;
    }
}

/** 点动看门狗：300ms 未续发自动停 */
static void jog_service(void)
{
    if (s_jog_active == 0U) { return; }
    if ((uint32_t)(s_ms - s_jog_last_ms) > CAN_JOG_TIMEOUT_MS)
    {
        apply_speed_target(0.0f);
        s_jog_active = 0U;
    }
}

/** 故障监视：运行中出现故障立即关断并进 FAULT */
static void fault_service(void)
{
    s_fault = scan_fault();
    if ((s_fault != 0U) && (s_state == ST_ACTIVE))
    {
        apply_speed_target(0.0f);
        disable_all_pwm_output();
        s_jog_active = 0U;
        s_state = ST_FAULT;
    }
}

/* 位置命令续发服务：等电机实际停稳后再下发，防过冲 */
static void pos_cmd_service(void)
{
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
    static uint16_t s_settle_ms = 0U;

    if (!s_pos_pending) {
        s_settle_ms = 0U;
        return;
    }
    /* 三个条件同时成立并连续保持 SCURVE_SETTLE_HOLD_MS 拍：
     *   ① 规划器空闲 ② 位置误差已收敛 ③ 实际速度已归零
     * ③ 不能省：转子"穿过位置误差=0"的瞬间速度恰好最大。 */
    if (scurve_is_idle()
        && (fabsf(g_pos_err) < SCURVE_SETTLE_POS)
        && (fabsf(g_spd_fdb) < SCURVE_SETTLE_SPD))
    {
        if (s_settle_ms < (uint16_t)SCURVE_SETTLE_HOLD_MS) {
            s_settle_ms++;
            return;
        }
        int rc = scurve_move_delta(s_pos_pending_delta);
        if (rc == 0) {
            s_pos_pending = 0U;
        } else if (rc != -1) {
            s_pos_pending = 0U;             /* 非法命令：丢弃，不无限挂起 */
        }
        s_settle_ms = 0U;
    }
    else
    {
        s_settle_ms = 0U;
    }
#endif
}

/* ---------------- 状态帧 ---------------- */
/** 快速状态 0x281（20ms）：state / mode_flags / 实际速度 / 目标速度 / 实际 Iq */
static void send_fast_status(void)
{
    uint8_t d[8];
    uint8_t flags = 0U;

    if (pwm_output_is_enabled()) { flags |= FLAG_PWM;       }
    if (s_state == ST_ACTIVE)    { flags |= FLAG_ENABLE;    }
    if (s_comm == COMM_OK)       { flags |= FLAG_HEARTBEAT; }
    if (s_fault != 0U)           { flags |= FLAG_FAULT;     }
    if (s_state == ST_TEST)      { flags |= FLAG_TEST;      }
    flags |= FLAG_CALIB;   /* ENC_SKIP_ALIGN=1，用常量电角度零点，视为校准有效 */
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
    flags |= FLAG_POS_VALID;
#endif

    d[0] = s_state;
    d[1] = flags;
    wr_i16(&d[2], f_to_i16(g_spd_fdb,   RADS_TO_DRPM, 3276.7f));   /* 0.1 rpm  */
    wr_i16(&d[4], f_to_i16(g_ref_speed, RADS_TO_DRPM, 3276.7f));   /* 0.1 rpm  */
    wr_i16(&d[6], f_to_i16(g_iq_ref,    1000.0f,      32767.0f));  /* 0.001 A  */

    send_frame(CAN_ID_FAST, 8U, d);
}

/** 慢速状态 0x301（100ms）：位置子帧与诊断子帧交替 */
static void send_slow_status(void)
{
    uint8_t d[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

    if (s_slow_phase == 0U)
    {
        d[0] = 0x01U;                                   /* 位置子帧 */
        d[1] = s_status_seq++;
        put_i24(&d[2], (int32_t)(g_pos_abs * RAD_TO_MDEG));
        put_i24(&d[5], (int32_t)(g_pos_ref * RAD_TO_MDEG));
    }
    else
    {
        d[0] = 0x02U;                                   /* 诊断子帧 */
        d[1] = s_state;
        d[2] = s_mode;
        d[3] = s_test_type;
        d[4] = s_test_stage;
        d[5] = s_comm;
        wr_u16(&d[6], s_fault);
    }
    s_slow_phase ^= 1U;
    send_frame(CAN_ID_SLOW, 8U, d);
}

static void send_echo(void)
{
    uint8_t d[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint16_t c;
    d[0] = s_echo_payload[0];
    d[1] = s_echo_payload[1];
    d[2] = s_echo_payload[2];
    d[3] = s_echo_payload[3];
    c = s_echo_cnt++;
    d[4] = (uint8_t)c;
    d[5] = (uint8_t)(c >> 8);
    send_frame(CAN_ID_ECHO_RSP, 8U, d);
}

/* ---------------- 对外接口 ---------------- */
void can_app_init(void)
{
    /* 1) 时钟 + 引脚。board_init_can() 内部的 init_can_pins() 只认 MCAN3，
     *    MCAN0 的 PA00/PA01 要自己配（E249 移植：CAN 在 UART0 控制台引脚上）。 */
    board_init_can(BOARD_APP_CAN_BASE);
    init_mcan0_pins();
    uint32_t src_clk = board_init_can_clock(BOARD_APP_CAN_BASE);
    g_can_src_clk_khz = src_clk / 1000U;       /* J-Scope 诊断：正常应为 40000 */

#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
    mcan_msg_buf_attr_t msg_attr = { (uint32_t)s_mcan0_msg_buf, sizeof(s_mcan0_msg_buf) };
    if (mcan_set_msg_buf_attr(BOARD_APP_CAN_BASE, &msg_attr) != status_success) {
        g_can_init_stat = 2U;
        g_can_err = 2U;                        /* 2=报文 RAM 注册失败 */
        return;
    }
#endif

    /* 2) 经典 CAN，波特率同 CAN_BAUDRATE_KHZ。默认滤波接受所有标准帧进 RXFIFO0。 */
    mcan_config_t cfg;
    mcan_get_default_config(BOARD_APP_CAN_BASE, &cfg);
    cfg.baudrate = CAN_BAUDRATE;
#if CAN_LOOPBACK_TEST
    cfg.mode     = mcan_mode_loopback_external;
#else
    cfg.mode     = mcan_mode_normal;
#endif
    hpm_stat_t st = mcan_init(BOARD_APP_CAN_BASE, &cfg, src_clk);
    if (st != status_success) {
        g_can_init_stat = st;
        g_can_err = 1U;
        return;
    }

    /* 3) RX FIFO0 收到新帧即中断 */
    mcan_enable_interrupts(BOARD_APP_CAN_BASE, MCAN_INT_RXFIFO0_NEW_MSG);
    intc_m_enable_irq_with_priority(IRQn_MCAN0, 1);

    s_fault = scan_fault();
}

void can_app_poll(void)
{
    s_ms++;                                     /* 约 1ms 一拍 */

    /* 1) 取走环形缓冲里的所有帧并解析 */
    while (s_rx_r != s_rx_w)
    {
        mcan_rx_message_t m = s_rx_msg[s_rx_r];
        s_rx_r = (s_rx_r + 1U) % CAN_RX_RING_DEPTH;
        handle_frame(&m);
    }

    /* 2) 周期服务 */
    pos_cmd_service();
    jog_service();
    fault_service();
    watchdog_service();

    /* 3) 连通性自检应答 */
    if (s_echo_pending)
    {
        s_echo_pending = false;
        send_echo();
    }

    /* 4) 周期状态帧 */
    static uint16_t fast_div = 0U;
    static uint16_t slow_div = 0U;
    if (++fast_div >= CAN_FAST_PERIOD_MS) {
        fast_div = 0U;
        send_fast_status();
    }
    if (++slow_div >= CAN_SLOW_PERIOD_MS) {
        slow_div = 0U;
        send_slow_status();
    }

    /* 5) 观测镜像（J-Scope 直接看协议状态机） */
    g_can_state = s_state;
    g_can_mode  = s_mode;
    g_can_comm  = s_comm;
    g_can_fault = s_fault;

    /* 6) 录波模块：CRC 分片 + READ_BLOCK 排队回传 + 20ms 趋势流 */
    motor_scope_poll_1ms();
    motor_scope_set_live((uint8_t)s_state, (uint8_t)s_mode,
                         (uint8_t)s_test_type, (uint8_t)s_test_stage);
}

#else  /* CAN_CTRL_ENABLE == 0：空实现，调用点不用条件编译 */

void can_app_init(void) { }
void can_app_poll(void) { }
void can_app_send_raw(uint32_t std_id, uint8_t dlc, const uint8_t *data)
{
    (void)std_id; (void)dlc; (void)data;
}

#endif /* CAN_CTRL_ENABLE */
