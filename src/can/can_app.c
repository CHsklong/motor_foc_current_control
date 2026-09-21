/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file can_app.c
 * @brief CAN 通信（MCAN0，CAN2.0A 500kbps）：CANTest 上位机遥测与电机控制
 *
 * 硬件：E249 板载 SN65HVD234 收发器，PA00=MCAN0_TXD(Pin12)、PA01=MCAN0_RXD(Pin11)，
 *       CANH/CANL 接 CANalyst-II 的 CAN1（或 CAN2），波特率双方都设 500kbps。
 *
 * ============================ 协议（标准帧，小端） ============================
 *   板 -> PC   0x201 遥测（每 100ms）：
 *                b0-1  int16  实际机械角速度 ×0.1 rad/s
 *                b2-3  int16  相对基准机械角 ×0.01 rad（限幅 ±327.66 rad）
 *                b4-5  int16  q 轴电流给定 ×0.01 A
 *                b6    状态位：bit0=S曲线运行 bit1=位置模式
 *                              bit2=速度模式  bit3=PWM硬件故障
 *                b7    遥测帧计数（回绕）
 *   板 -> PC   0x202 诊断（每 500ms）：b0-1 收帧数 b2-3 发帧数 b4 丢弃数
 *                              b5 溢出数 b6 启动步骤 b7 错误码(1=初始化失败)
 *   PC -> 板   0x101 速度命令：b0-1 int16 目标速度 ×0.1 rad/s
 *                              b2   1=运行 / 0=停止(给定清零)
 *                —— 仅 FOC_SPEED_MODE=1 时生效
 *   PC -> 板   0x102 位置命令：b0-3 int32 位置增量 ×0.01 rad（S 曲线走过去）
 *                —— 仅 FOC_POSITION_MODE=1 且 SCURVE_ENABLE=1 时生效
 *   PC -> 板   0x100 连通性测试（负载任意）→ 板回 0x1F0：
 *                              b0-3=收到的 b0-3 原样回显，b4-5=应答计数
 *
 * 命令与当前模式不匹配、或 S 曲线正忙时，命令被丢弃并计入 g_can_drop_cnt。
 *
 * 注意：PA00/PA01 同时是 UART0 控制台引脚（pinmux.c 的 init_uart0_pins），
 *       CAN 引脚在 board_init() 之后初始化会覆盖它 → printf 不再可用。
 *       本工程调试走 J-Scope，无影响；若要串口打印，把 CAN_CTRL_ENABLE 置 0。
 * =============================================================================
 */
#include "hw_map.h"
#include "hpm_mcan_drv.h"
#include "motor.h"              /* motor0 */
#include "traj_s_curve.h"       /* scurve_move_delta / scurve_hold */
#include "dbg_probe.h"          /* g_spd_fdb / g_pos_abs / g_iq_ref / g_sc_state / g_pwm_fault */
#include "app_cfg.h"
#include "can_app.h"
#include <string.h>

#if CAN_CTRL_ENABLE

/* ---------------- 参数 ---------------- */
#ifndef CAN_BAUDRATE_KHZ
#define CAN_BAUDRATE_KHZ  500
#endif
#define CAN_BAUDRATE          ((uint32_t)CAN_BAUDRATE_KHZ * 1000U)  /* CANTest 设同样值 */
#define CAN_TELEM_ID          (0x201U)    /* 板 -> PC 遥测 */
#define CAN_CMD_SPEED_ID      (0x101U)    /* PC -> 板 速度命令 */
#define CAN_CMD_POS_ID        (0x102U)    /* PC -> 板 位置增量命令 */
#define CAN_TEST_CMD_ID       (0x100U)    /* PC -> 板 连通性测试 */
#define CAN_TEST_RSP_ID       (0x1F0U)    /* 板 -> PC 测试应答 */
#define CAN_TELEM_PERIOD_MS   (100U)      /* 遥测周期：主循环约 1ms 一拍 ×100 */
#define CAN_DBG_ID            (0x202U)    /* 板 -> PC 诊断遥测（每 500ms） */
#define CAN_DBG_PERIOD_MS     (500U)      /* 诊断帧周期 */
#define CAN_RX_RING_DEPTH     (8U)        /* 接收环形缓冲深度（命令速率低，足够） */

/* ---------------- 报文 RAM（HPM5361：MCAN 的 Message RAM 在 AHB RAM，必须先注册，
 * 否则 ram_size=0，mcan_init 直接返回 status_invalid_argument(2)。见 hpm_mcan_soc.h） ---------------- */
#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
static uint32_t s_mcan0_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS] ATTR_PLACE_AT(".ahb_sram");
#endif

/* ---------------- 观测计数 ---------------- */
volatile uint32_t g_can_rx_cnt   = 0U;
volatile uint32_t g_can_tx_cnt   = 0U;
volatile uint32_t g_can_drop_cnt = 0U;
volatile uint32_t g_can_overrun  = 0U;
volatile float    g_can_spd_cmd  = 0.0f;
volatile uint32_t g_can_err      = 0U;
volatile uint32_t g_can_init_stat   = 0U;   /* mcan_init 原始返回码：0=成功，3=通用超时，26013=MCAN超时，26014=位时序无效 */
volatile uint32_t g_can_src_clk_khz = 0U;   /* board_init_can_clock 返回的 CAN 功能时钟（kHz），正常应为 40000 */

/* ---------------- 接收环形缓冲（ISR 写 / 主循环读，单生产者单消费者） ---------------- */
static mcan_rx_message_t s_rx_msg[CAN_RX_RING_DEPTH];
static volatile uint32_t s_rx_w = 0U;   /*写指针 */
static volatile uint32_t s_rx_r = 0U;   /*环读指针*/

static volatile bool     s_echo_pending = false;      /* 0x100 测试帧待应答 */
static uint8_t           s_echo_payload[4];
static volatile uint16_t s_echo_cnt = 0U;
static uint8_t           s_telem_cnt = 0U;

/* ---------------- 小端打包工具 ---------------- */
static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static int16_t  rd_i16(const uint8_t *p) { return (int16_t)rd_u16(p); }
#if (FOC_POSITION_MODE && SCURVE_ENABLE)   /* 只有 S 曲线位置命令用得到，避免未使用告警 */
static int32_t  rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
#endif
static void wr_i16(uint8_t *p, int16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)((uint16_t)v >> 8); }
static void wr_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

/* 浮点定标：四舍五入到 int16，防溢出 */
static int16_t f_to_i16(float x, float scale, float lim)
{
    float v = x * scale;
    if (v >  lim) { v =  lim; }
    if (v < -lim) { v = -lim; }
    v += (v >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)v;
}

/* ---------------- ISR：RXFIFO0 全部帧搬进环形缓冲 ---------------- */
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, can_app_isr)
void can_app_isr(void)                                  //中断只做搬运
{
    uint32_t flags = mcan_get_interrupt_flags(BOARD_APP_CAN_BASE);
    if ((flags & MCAN_INT_RXFIFO0_NEW_MSG) != 0U) 
    {                                                 //读到空为止
        for (;;) {
            mcan_rx_message_t rx;                             //读取pc发来的数据
            if (mcan_read_rxfifo(BOARD_APP_CAN_BASE, 0, &rx) != status_success) 
            {
                break;                      /* FIFO 读空 */
            }
            uint32_t next = (s_rx_w + 1U) % CAN_RX_RING_DEPTH;
            if (next != s_rx_r)                 
            {
                s_rx_msg[s_rx_w] = rx;                  //存进环形缓冲
                s_rx_w = next;
            } else 
            {
                g_can_overrun++;            /* 主循环没来得及取，丢最旧的进不来 */
            }
        }
    }
    mcan_clear_interrupt_flags(BOARD_APP_CAN_BASE, flags);
}

/* ---------------- 命令处理 ---------------- */
static void handle_speed_cmd(const mcan_rx_message_t *m)
{
#if FOC_SPEED_MODE
    float v = (float)rd_i16(&m->data_8[0]) * 0.1f;
    if (m->dlc < 3 || m->data_8[2] == 0U) 
    {
        v = 0.0f;                           /* run=0：给定清零停机 */
    }
    mcl_user_value_t cmd;
    cmd.enable = true;
    cmd.value  = v;
    hpm_mcl_loop_set_speed(&motor0.loop, cmd);
    g_ref_speed   = v;
    g_can_spd_cmd = v;
#else
    (void)m;
    g_can_drop_cnt++;                       /* 当前不在速度模式 */
#endif
}

static void handle_pos_cmd(const mcan_rx_message_t *m)
{
#if (FOC_POSITION_MODE && SCURVE_ENABLE)
    float delta = (float)rd_i32(&m->data_8[0]) * 0.01f;   /* ×0.01 rad */
    if (scurve_move_delta(delta) == 0) {
        g_can_spd_cmd = 0.0f;
    } else {
        g_can_drop_cnt++;                   /* S 曲线正忙，不接受打断 */
    }
#else
    (void)m;
    g_can_drop_cnt++;                       /* 当前不在 S 曲线位置模式 */
#endif
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
    case CAN_CMD_SPEED_ID:
        handle_speed_cmd(m);
        break;
    case CAN_CMD_POS_ID:
        handle_pos_cmd(m);
        break;
    case CAN_TEST_CMD_ID:
        /* 连通性测试：记下负载，主循环里回 0x1F0（发送不在 ISR 里做） */
        s_echo_payload[0] = m->data_8[0];
        s_echo_payload[1] = m->data_8[1];
        s_echo_payload[2] = m->data_8[2];
        s_echo_payload[3] = m->data_8[3];
        s_echo_pending = true;
        break;
    default:
        break;                              /* 其他 ID 静默忽略 */
    }
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
    /* 非阻塞发送：总线无节点应答（CANTest 没开）时不会卡死主循环 */
    if (mcan_transmit_via_txfifo_nonblocking(BOARD_APP_CAN_BASE, &tx, &idx) == status_success) {
        g_can_tx_cnt++;
    }
}

static void send_telemetry(void)
{
    uint8_t d[8];
    int16_t v = f_to_i16(g_spd_fdb, 10.0f, 3276.7f);        /* ×0.1 rad/s */
    wr_i16(&d[0], v);
    v = f_to_i16(g_pos_abs, 100.0f, 327.66f);               /* ×0.01 rad */
    wr_i16(&d[2], v);
    v = f_to_i16(g_iq_ref, 100.0f, 327.67f);                /* ×0.01 A */
    wr_i16(&d[4], v);

    uint8_t st = 0U;
#if SCURVE_ENABLE
    if (g_sc_state == 1U) { st |= 0x01U; }                  /* S 曲线运行中 */
#endif
#if FOC_POSITION_MODE
    st |= 0x02U;
#endif
#if FOC_SPEED_MODE
    st |= 0x04U;
#endif
    if (g_pwm_fault != 0U) { st |= 0x08U; }
    d[6] = st;
    d[7] = s_telem_cnt++;

    send_frame(CAN_TELEM_ID, 8U, d);
}

/* 诊断帧：把 J-Scope 才能看的计数搬到 CAN 上。
 * 用途：仿真器与 CAN 盒不能同时接时，CANTest 本身就是判据（见下）。
 *   b0-1 收到帧数低16位   b2-3 发出帧数低16位
 *   b4 丢弃命令数         b5 接收溢出次数
 *   b6 启动步骤 g_boot_step（≥13 表示 CAN 初始化已完成）
 *   b7 g_can_err（1=MCAN 初始化失败，查时钟/接线） */
static void send_debug(void)
{
    uint8_t d[8];
    wr_u16(&d[0], (uint16_t)(g_can_rx_cnt   & 0xFFFFU));
    wr_u16(&d[2], (uint16_t)(g_can_tx_cnt   & 0xFFFFU));
    d[4] = (uint8_t)(g_can_drop_cnt & 0xFFU);
    d[5] = (uint8_t)(g_can_overrun  & 0xFFU);
    d[6] = (uint8_t)(g_boot_step    & 0xFFU);
    d[7] = (uint8_t)(g_can_err      & 0xFFU);
    send_frame(CAN_DBG_ID, 8U, d);
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
    send_frame(CAN_TEST_RSP_ID, 8U, d);
}

/* ---------------- 对外接口 ---------------- */
void can_app_init(void)
{
    /* 1) 时钟 + 引脚。board_init_can() 内部的 init_can_pins() 只认 MCAN3，
     *    MCAN0 的 PA00/PA01 要自己配（E249 移植：CAN 在 UART0 控制台引脚上）。 */
    board_init_can(BOARD_APP_CAN_BASE);                               //开时钟
    init_mcan0_pins();                                                // PA00/PA01 配成 CAN（覆盖掉 UART0）
    uint32_t src_clk = board_init_can_clock(BOARD_APP_CAN_BASE);
    g_can_src_clk_khz = src_clk / 1000U;       /* J-Scope 诊断：正常应为 40000（PLL1 400MHz/10） */

#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && (MCAN_SOC_MSG_BUF_IN_AHB_RAM == 1)
    /* 注册 MCAN0 报文 RAM 区（.ahb_sram 段，32KB AHB RAM 足够）。失败 = 地址/大小非法 */
    mcan_msg_buf_attr_t msg_attr = { (uint32_t)s_mcan0_msg_buf, sizeof(s_mcan0_msg_buf) };
    if (mcan_set_msg_buf_attr(BOARD_APP_CAN_BASE, &msg_attr) != status_success) {
        g_can_init_stat = 2U;
        g_can_err = 2U;                        /* 与 mcan_init 失败区分开：2=RAM 注册失败 */
        return;
    }
#endif

    /* 2) 经典 CAN，波特率同 CAN_BAUDRATE_KHZ。默认滤波配置接受所有标准帧进 RXFIFO0。
     *    CAN_LOOPBACK_TEST=1 时切外部回环：帧仍送到 CANH/CANL（万用表可测），
     *    但 MCU 内部自收、不要求 ACK —— 无需接 CAN 盒即可自检软件+引脚+收发器。 */
    mcan_config_t cfg;
    mcan_get_default_config(BOARD_APP_CAN_BASE, &cfg);              // 默认配置：接受所有标准帧进 RXFIFO0
    cfg.baudrate = CAN_BAUDRATE;                                    //波特率500k
#if CAN_LOOPBACK_TEST
    cfg.mode     = mcan_mode_loopback_external;
#else
    cfg.mode     = mcan_mode_normal;
#endif
    hpm_stat_t st = mcan_init(BOARD_APP_CAN_BASE, &cfg, src_clk);
    if (st != status_success) {
        g_can_init_stat = st;                  /* J-Scope 诊断：具体失败原因，见 can_app.h 注释 */
        g_can_err = 1U;
        return;
    }

    /* 3) RX FIFO0 收到新帧即中断 */
    mcan_enable_interrupts(BOARD_APP_CAN_BASE, MCAN_INT_RXFIFO0_NEW_MSG);   // 收到帧就中断
    intc_m_enable_irq_with_priority(IRQn_MCAN0, 1);
}
    
void can_app_poll(void)                               //取帧解析、应答 0x100、定时发遥测
{
    /* 1) 取走环形缓冲里的所有帧并处理 */
    while (s_rx_r != s_rx_w) 
    {
        mcan_rx_message_t m = s_rx_msg[s_rx_r];
        s_rx_r = (s_rx_r + 1U) % CAN_RX_RING_DEPTH;
        handle_frame(&m);
    }

    /* 2) 连通性测试应答 */
    if (s_echo_pending) 
    {
        s_echo_pending = false;
        send_echo();
    }

    /* 3) 周期遥测 */
    static uint16_t telem_div = 0U;
    static uint16_t dbg_div   = 0U;
    if (++telem_div >= CAN_TELEM_PERIOD_MS) {
        telem_div = 0U;
        send_telemetry();
    }
    if (++dbg_div >= CAN_DBG_PERIOD_MS) {
        dbg_div = 0U;
        send_debug();
    }
}

#else  /* CAN_CTRL_ENABLE == 0：空实现，调用点不用条件编译 */

void can_app_init(void) { }
void can_app_poll(void) { }

#endif /* CAN_CTRL_ENABLE */
