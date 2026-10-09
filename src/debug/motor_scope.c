/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file motor_scope.c
 * @brief ELF 变量录波实现（配合上位机"ELF 与录波"页）
 *
 * 设计约束见 motor_scope.h 文件头。这里只补充实现层面的三条：
 *
 * 1) 采样缓冲按【通道分块】存放（channel-major）：
 *      偏移 = channel * points * 4 + index * 4
 *    这样 READ_BLOCK(通道, 起点, 数量) 是一次连续内存读，中断里只做一次 4 字节写，
 *    不会在 20kHz 里做跨通道跳转。
 *
 * 2) CRC32（ISO-HDLC）不在中断里算：64 KiB 逐字节 CRC 约几十万次迭代，
 *    放进 50us 的中断必挂。做法：采满后状态转 CALCULATING_CRC，
 *    由 motor_scope_poll_1ms() 每拍算一片（SCOPE_CRC_SLICE 字节），
 *    算完才转 FROZEN。上位机轮询 GET_STATUS==FROZEN 即可，协议本来就允许这个中间态。
 *
 * 3) READ_BLOCK 一次最多 64 帧，一拍全发会撑爆 TX FIFO 且阻塞状态帧，
 *    所以改成排队：poll 每拍最多发 SCOPE_TX_PER_TICK 帧。
 */
#include <string.h>
#include <math.h>

#include "motor_scope.h"
#include "motor_scope_service.h"   /* 变量【定义】本体：上位机按这个文件名认变量 */
#include "can_app.h"
#include "dbg_probe.h"
#include "ctrl_foc.h"
#include "drv_pwm.h"               /* pwm_output_is_enabled：0xF1 环路启用标志用 */
#include "hpm_mcl_loop.h"          /* g_sens_q：q 轴电流【反馈】实测值 */

/* ---------------- CRC 分片与发送限流 ---------------- */
#define SCOPE_CRC_SLICE     (4096U)   /* 每拍最多算这么多字节的 CRC */
#define SCOPE_TX_PER_TICK   (8U)      /* 每拍最多发这么多帧录波数据（不阻塞状态帧） */
#define SCOPE_STREAM_PERIOD (20U)     /* 趋势流周期 ms（协议规定 20ms / 50Hz） */
#define SCOPE_REFRESH_DIV   (8U)      /* 空闲时通道刷新分频：20kHz/8 = 2.5kHz，趋势流够用 */

/* DLM 允许范围，与上位机内部常量一致：越界直接拒绝配置 */
#define SCOPE_DLM_BASE  (0x00080000U)
#define SCOPE_DLM_LIMIT (0x000A0000U)

/* 变量定义都在 motor_scope_service.h（含 build id / 六路通道 / 采样缓冲），
 * 这里只有内部状态与逻辑。 */

/* ---------------- 内部状态 ---------------- */
typedef struct {
    const void *addr;
    uint8_t     type;
    uint8_t     set;      /* 0=高速 1=趋势 */
    uint8_t     used;
} scope_ch_t;

static scope_ch_t s_ch[SCOPE_MAX_CHANNELS];

static uint8_t  s_state      = SCOPE_ST_IDLE;
static uint16_t s_capture_id = 0U;
static uint16_t s_points     = 1024U;   /* 默认 1024 点 */
static uint16_t s_divider    = 1U;      /* 默认不分频（20kHz） */
static uint16_t s_pretrigger = 0U;
static uint8_t  s_ch_count   = 1U;
static uint8_t  s_trig_mode  = SCOPE_TRIG_IMMEDIATE;
static uint8_t  s_test_type  = 0U;
static uint16_t s_trig_index = 0U;
static uint32_t s_trig_tick  = 0U;

static uint16_t s_wr_index;             /* 已写入点数 */
static uint16_t s_div_cnt;              /* 分频计数 */
static uint16_t s_refresh_div;          /* 空闲时通道刷新分频计数 */
static uint32_t s_crc;
static uint32_t s_crc_ctx;              /* 滚动 CRC 上下文 */
static uint32_t s_crc_done;             /* 已算字节数 */

static uint32_t s_ms;                   /* 1ms 计数（poll 里递增） */

/* 预触发环形缓冲（sample-major：一次采样的所有通道连续存放） */
static uint32_t s_pre_buf[SCOPE_PRETRIG_MAX * 4U];
static uint16_t s_pre_w;                /* 环形写指针 */
static uint16_t s_pre_n;                /* 已缓存点数 */

/* 趋势流 */
static bool     s_stream_on;
static uint16_t s_stream_seq;
static uint16_t s_stream_div;
static uint8_t  s_stream_id;        /* 每次重启流递增（协议：一组内 8 帧共用同一个 id） */

/* READ_BLOCK 排队 */
static bool     s_read_active;
static uint8_t  s_read_ch;
static uint16_t s_read_idx;
static uint16_t s_read_left;

/* 实时状态镜像（GET_LIVE_STATE） */
static uint8_t s_live_state;
static uint8_t s_live_mode;
static uint8_t s_live_test;
static uint8_t s_live_stage;

/* ---------------- 工具 ---------------- */
static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/** CRC32/ISO-HDLC：poly 0x04C11DB7，init/final xor 均为 0xFFFFFFFF，输入输出反射 */
static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0U; i < n; i++)
    {
        crc ^= p[i];
        for (uint32_t b = 0U; b < 8U; b++)
        {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return crc;
}

/** 按配置的类型读一个通道，统一返回 4 字节原始位（浮点按 IEEE754 位透传） */
static uint32_t chan_read(const scope_ch_t *c)
{
    uint32_t raw = 0U;
    switch (c->type)
    {
    case SCOPE_T_U8:   raw = (uint32_t)(*(const uint8_t  *)c->addr);                 break;
    case SCOPE_T_I8:   raw = (uint32_t)(uint8_t)(*(const int8_t   *)c->addr);        break;
    case SCOPE_T_U16:  raw = (uint32_t)(*(const uint16_t *)c->addr);                 break;
    case SCOPE_T_I16:  raw = (uint32_t)(uint16_t)(*(const int16_t *)c->addr);        break;
    case SCOPE_T_U32:  raw = *(const uint32_t *)c->addr;                             break;
    case SCOPE_T_I32:  raw = (uint32_t)(*(const int32_t *)c->addr);                  break;
    case SCOPE_T_BOOL: raw = (*(const uint8_t *)c->addr != 0U) ? 1U : 0U;            break;
    case SCOPE_T_F32:
    default:
    {
        float v = *(const float *)c->addr;
        (void)memcpy(&raw, &v, 4U);      /* 浮点按位透传，由上位机按 F32 解释 */
        break;
    }
    }
    return raw;
}

static uint32_t chan_offset(uint8_t ch, uint16_t idx)
{
    return ((uint32_t)ch * (uint32_t)s_points * 4U) + ((uint32_t)idx * 4U);
}

static void chan_store(uint8_t ch, uint16_t idx, uint32_t raw)
{
    uint32_t off = chan_offset(ch, idx);
    if ((off + 4U) <= SCOPE_BUF_SIZE)
    {
        wr_u32(&motor_scope_sample_buffer[off], raw);
    }
}

/** 采满 → 进入 CRC 分片计算 */
static void capture_finish(void)
{
    s_state    = SCOPE_ST_CALCULATING_CRC;
    s_crc_ctx  = 0xFFFFFFFFU;
    s_crc_done = 0U;
}

/* ---------------- 采样（20kHz 中断） ---------------- */
static void sample_into_pre(void)
{
    /* 预触发环形：一次采样的所有通道连续存（sample-major） */
    uint32_t base = (uint32_t)s_pre_w * (uint32_t)s_ch_count;
    for (uint8_t c = 0U; c < s_ch_count; c++)
    {
        if ((base + c) < (SCOPE_PRETRIG_MAX * 4U))
        {
            s_pre_buf[base + c] = chan_read(&s_ch[c]);
        }
    }
    s_pre_w = (uint16_t)((s_pre_w + 1U) % SCOPE_PRETRIG_MAX);
    if (s_pre_n < SCOPE_PRETRIG_MAX) { s_pre_n++; }
}

static void sample_into_buf(void)
{
    for (uint8_t c = 0U; c < s_ch_count; c++)
    {
        chan_store(c, s_wr_index, chan_read(&s_ch[c]));
    }
    s_wr_index++;
    if (s_wr_index >= s_points) { capture_finish(); }
}

/** 触发发生：先把预触发环形里的历史点搬到缓冲头部（最老的在前） */
static void flush_pre(void)
{
    uint16_t n = s_pretrigger;
    if (n > s_pre_n) { n = s_pre_n; }
    if (n > s_points) { n = s_points; }

    uint16_t start = (uint16_t)((s_pre_n >= SCOPE_PRETRIG_MAX)
                                ? s_pre_w
                                : 0U);                   /* 环形未满时从 0 开始 */
    for (uint16_t i = 0U; i < n; i++)
    {
        uint16_t slot = (uint16_t)((start + (s_pre_n - n) + i) % SCOPE_PRETRIG_MAX);
        uint32_t base = (uint32_t)slot * (uint32_t)s_ch_count;
        for (uint8_t c = 0U; c < s_ch_count; c++)
        {
            chan_store(c, i, s_pre_buf[base + c]);
        }
    }
    s_wr_index = n;
}

/** 六路预选通道刷新（上位机导入 ELF 后默认就看这六个） */
static void refresh_channels(void)
{
    float iamp = sqrtf(g_ia * g_ia + g_ib * g_ib + g_ic * g_ic);   /* 电流矢量幅值 A */

    jscope_iq_ref  = g_iq_ref;
    jscope_iq_amp  = iamp;
    motor_scope_signals.speed_ref_rad_s    = g_ref_speed;
    motor_scope_signals.speed_fdb_rad_s    = g_spd_fdb;
    motor_scope_signals.position_ref_relative_rad   = g_pos_ref;
    motor_scope_signals.position_fdb_relative_rad   = g_pos_abs;
    motor_scope_signals.iq_ref_a           = g_iq_ref;
    /* 通道 1 必须是【q 轴电流反馈】而不是"电流矢量幅值"：
       六路趋势协议里 kind=1 的物理意义是 Iq_actual（带符号，A），
       幅值 sqrt(ia²+ib²+ic²) 永远为正，看不出制动时的反向电流，
       也和上位机"电流环实际"窗的方向判读对不上。
       g_sens_q 是 MCL 电流环里 Park 变换后的实测 iq（见 hpm_mcl_loop.h）。 */
    motor_scope_signals.iq_fdb_a           = g_sens_q;
    jscope_encoder_speed_rad_s             = g_spd_fdb;
    jscope_position_ref_relative_rad       = g_pos_ref;
    jscope_position_feedback_relative_rad  = g_pos_abs;
}

void motor_scope_isr_tick(void)
{
    /* 1) 通道刷新：只在真正采样时才每拍刷（20kHz 录波需要最新值）；
     *    空闲时降到 2.5kHz 刷新即可 —— 趋势流是 50Hz，用不着 20kHz，
     *    这样省掉每拍一次 sqrtf（本工程有过中断超载导致主循环饿死的历史）。 */
    bool sampling = (s_state == SCOPE_ST_ARMED) || (s_state == SCOPE_ST_TRIGGERED);
    if (sampling)
    {
        refresh_channels();
    }
    else if (++s_refresh_div >= SCOPE_REFRESH_DIV)
    {
        s_refresh_div = 0U;
        refresh_channels();
    }

    /* 2) 采集状态机 */
    if (s_state == SCOPE_ST_ARMED)
    {
        /* 等待触发期间：维护预触发环形，触发后才有"触发前的波形" */
        if (s_pretrigger > 0U)
        {
            if (++s_div_cnt >= s_divider)
            {
                s_div_cnt = 0U;
                sample_into_pre();
            }
        }
    }
    else if (s_state == SCOPE_ST_TRIGGERED)
    {
        if (++s_div_cnt >= s_divider)
        {
            s_div_cnt = 0U;
            sample_into_buf();
        }
    }
    else
    {
        /* IDLE / FROZEN / CRC / ERROR：不采样 */
    }
}

void motor_scope_notify_trigger(void)
{
    if (s_state == SCOPE_ST_ARMED)
    {
        s_trig_index = s_wr_index;
        s_trig_tick  = s_ms;
        s_div_cnt    = 0U;
        if (s_pretrigger > 0U) { flush_pre(); } else { s_wr_index = 0U; }
        s_state = SCOPE_ST_TRIGGERED;
    }
}

/* ---------------- 发送 ---------------- */
static void send_resp(uint8_t op, uint8_t seq, uint8_t result, uint8_t field, uint32_t value)
{
    uint8_t d[8];
    d[0] = op;
    d[1] = seq;
    d[2] = result;
    d[3] = field;
    wr_u32(&d[4], value);
    can_app_send_raw(SCOPE_ID_RSP, 8U, d);
}

static void send_data(uint8_t capture_id, uint8_t ch, uint16_t idx, uint32_t raw)
{
    uint8_t d[8];
    d[0] = capture_id;
    d[1] = ch;
    d[2] = (uint8_t)idx;
    d[3] = (uint8_t)(idx >> 8);
    wr_u32(&d[4], raw);
    can_app_send_raw(SCOPE_ID_DATA, 8U, d);
}

static void send_stream(uint8_t kind, uint8_t stream_id, uint16_t seq, uint32_t value)
{
    uint8_t d[8];
    d[0] = kind;
    d[1] = stream_id;
    d[2] = (uint8_t)seq;
    d[3] = (uint8_t)(seq >> 8);
    wr_u32(&d[4], value);
    can_app_send_raw(SCOPE_ID_STREAM, 8U, d);
}

/* ---------------- op 处理 ---------------- */
static uint8_t decode_type(uint8_t arg1)
{
    uint8_t t = arg1 & 0x0FU;
    if ((t < 1U) || (t > 8U)) { t = (arg1 >> 4) & 0x0FU; }
    if ((t < 1U) || (t > 8U)) { t = SCOPE_T_F32; }   /* 认不出来就按 float，本工程通道全是 float */
    return t;
}
static uint8_t decode_set(uint8_t arg1)
{
    /* 上位机把 set(0高速/1趋势) 与类型拼在一个字节里，位序无文档：
     * 这里对 0x10 与 0x80 两种常见位都认作"趋势"，解析错也不致命 ——
     * 趋势流与高速录波都会回退到"所有已配置通道"，见 stream_channels()/arm()。 */
    return (((arg1 & 0x10U) != 0U) || ((arg1 & 0x80U) != 0U)) ? 1U : 0U;
}

static void handle_get_info(uint8_t seq, uint8_t field)
{
    uint32_t v;
    switch (field)
    {
    case 0U:
        /* ★ 协议版本必须是 2 —— 上位机【只在收到版本 2 后才发 STREAM 启动】，
           回 1 的表现就是"六路趋势窗口一片空白、也不报错"。
           （依据：《其他固件接入固定六路CAN波形操作手册》第 4 步）
           报 2 的前提：帧格式 0x521<kind,id,seq:u16,raw:u32>、每组 8 帧
           （0xF0 + 0xF1 + kind0..5）、单位与通道含义都与该手册一致。 */
        v = 0x0002U;
        break;
    case 1U: v = motor_scope_build_id[0];
        break;
    case 2U: v = motor_scope_build_id[1];
        break;
    case 3U: v = motor_scope_build_id[2];
        break;
    case 4U: v = motor_scope_build_id[3];
        break;
    case 5U: v = (uint32_t)SCOPE_BUF_SIZE;      /* 采样缓冲 64 KiB */
        break;
    case 6U: v = 20000U;                        /* 基础采样时钟 20 kHz */
        break;
    default:
        send_resp(SCOPE_OP_GET_INFO, seq, 0x09U, field, 0U);   /* ACK_ERR_PARAM 同义 */
        return;
    }
    send_resp(SCOPE_OP_GET_INFO, seq, 0U, field, v);
}

static void handle_set_channel(uint8_t seq, uint8_t slot, uint8_t arg1, uint32_t addr)
{
    if (slot >= SCOPE_MAX_CHANNELS)
    {
        send_resp(SCOPE_OP_SET_CHANNEL, seq, 0x04U, slot, 0U);     /* ERR_RANGE */
        return;
    }
    if ((addr < SCOPE_DLM_BASE) || (addr >= SCOPE_DLM_LIMIT))
    {
        send_resp(SCOPE_OP_SET_CHANNEL, seq, 0x04U, slot, addr);   /* 地址不在 DLM */
        return;
    }
    if ((addr & 3U) != 0U)                                          /* 4 字节对齐（float 必需） */
    {
        send_resp(SCOPE_OP_SET_CHANNEL, seq, 0x04U, slot, addr);
        return;
    }
    s_ch[slot].addr  = (const void *)(uintptr_t)addr;
    s_ch[slot].type  = decode_type(arg1);
    s_ch[slot].set   = decode_set(arg1);
    s_ch[slot].used  = 1U;
    send_resp(SCOPE_OP_SET_CHANNEL, seq, 0U, slot, addr);
}

static void handle_set_timing(uint8_t seq, uint32_t value)
{
    uint16_t points  = (uint16_t)(value >> 16);
    uint16_t divider = (uint16_t)(value & 0xFFFFU);
    if ((points == 0U) || (divider == 0U))
    {
        send_resp(SCOPE_OP_SET_TIMING, seq, 0x04U, 0U, value);
        return;
    }
    if (points > SCOPE_MAX_POINTS) { points = SCOPE_MAX_POINTS; }
    s_points  = points;
    s_divider = divider;
    send_resp(SCOPE_OP_SET_TIMING, seq, 0U, 0U, value);
}

static void handle_set_trigger(uint8_t seq, uint8_t mode, uint8_t test_type, uint32_t value)
{
    uint16_t pre = (uint16_t)(value & 0xFFFFU);
    if (mode > 1U)
    {
        send_resp(SCOPE_OP_SET_TRIGGER, seq, 0x04U, mode, value);
        return;
    }
    if (pre > SCOPE_PRETRIG_MAX) { pre = SCOPE_PRETRIG_MAX; }
    s_trig_mode  = mode;
    s_test_type  = test_type;
    s_pretrigger = pre;
    send_resp(SCOPE_OP_SET_TRIGGER, seq, 0U, mode, value);
}

static void handle_arm(uint8_t seq, uint8_t ch_count)
{
    if ((ch_count < 1U) || (ch_count > 4U))
    {
        send_resp(SCOPE_OP_ARM, seq, 0x04U, ch_count, 0U);
        return;
    }
    /* 缓冲是否装得下：ch * points * 4 <= 64 KiB */
    if (((uint32_t)ch_count * (uint32_t)s_points * 4U) > SCOPE_BUF_SIZE)
    {
        send_resp(SCOPE_OP_ARM, seq, 0x04U, ch_count, 0U);
        return;
    }
    /* 通道必须已配置；未配置则回退指向六路预选变量（上位机未发 SET_CHANNEL 也能用） */
    for (uint8_t c = 0U; c < ch_count; c++)
    {
        if (s_ch[c].used == 0U)
        {
            switch (c)
            {
            case 0U: s_ch[c].addr = (const void *)&jscope_iq_ref;                        break;
            case 1U: s_ch[c].addr = (const void *)&motor_scope_signals.iq_fdb_a;         break;
            case 2U: s_ch[c].addr = (const void *)&motor_scope_signals.speed_ref_rad_s;  break;
            case 3U:
            default: s_ch[c].addr = (const void *)&jscope_encoder_speed_rad_s;           break;
            }
            s_ch[c].type = SCOPE_T_F32;
            s_ch[c].used = 1U;
        }
    }

    s_ch_count  = ch_count;
    /* capture_id 只有 1 个字节能回给上位机（0x501 帧 B0 / READ_BLOCK 的 arg0），
       所以必须限制在 1..255 内循环，否则低 8 位与上位机持有的值对不上 */
    s_capture_id++;
    if ((s_capture_id == 0U) || (s_capture_id > 255U)) { s_capture_id = 1U; }
    s_wr_index  = 0U;
    s_div_cnt   = 0U;
    s_pre_w     = 0U;
    s_pre_n     = 0U;
    s_trig_index = 0U;
    s_trig_tick  = s_ms;

    if (s_trig_mode == SCOPE_TRIG_IMMEDIATE)
    {
        s_state = SCOPE_ST_TRIGGERED;
    }
    else
    {
        s_state = SCOPE_ST_ARMED;
    }
    send_resp(SCOPE_OP_ARM, seq, 0U, ch_count, (uint32_t)s_capture_id);
}

static void handle_get_status(uint8_t seq, uint8_t field)
{
    uint32_t v;
    switch (field)
    {
    case 0U: v = s_state;                 break;
    case 1U: v = s_capture_id;            break;
    case 2U: v = s_points;                break;
    case 3U: v = s_trig_index;            break;
    case 4U: v = s_trig_tick;             break;
    case 5U: v = s_divider;               break;
    case 6U: v = s_ch_count;              break;
    case 7U: v = s_crc;                   break;
    default:
        send_resp(SCOPE_OP_GET_STATUS, seq, 0x09U, field, 0U);
        return;
    }
    send_resp(SCOPE_OP_GET_STATUS, seq, 0U, field, v);
}

static void handle_read_block(uint8_t seq, uint8_t capture_id, uint8_t ch, uint32_t value)
{
    uint16_t count = (uint16_t)(value >> 16);
    uint16_t start = (uint16_t)(value & 0xFFFFU);

    if (s_state != SCOPE_ST_FROZEN)
    {
        send_resp(SCOPE_OP_READ_BLOCK, seq, 0x03U, ch, 0U);      /* ERR_STATE */
        return;
    }
    if ((uint8_t)(s_capture_id & 0xFFU) != capture_id)
    {
        send_resp(SCOPE_OP_READ_BLOCK, seq, 0x04U, ch, 0U);      /* capture 不匹配 */
        return;
    }
    if ((count < 1U) || (count > 64U) || (ch >= s_ch_count) || (start >= s_points))
    {
        send_resp(SCOPE_OP_READ_BLOCK, seq, 0x04U, ch, value);
        return;
    }
    if (((uint32_t)start + (uint32_t)count) > (uint32_t)s_points)
    {
        count = (uint16_t)(s_points - start);
    }
    s_read_active = true;
    s_read_ch     = ch;
    s_read_idx    = start;
    s_read_left   = count;
    send_resp(SCOPE_OP_READ_BLOCK, seq, 0U, ch, value);
}

void motor_scope_handle_req(const uint8_t d[8])
{
    uint8_t op   = d[0];
    uint8_t seq  = d[1];
    uint8_t arg0 = d[2];
    uint8_t arg1 = d[3];
    uint32_t val = rd_u32(&d[4]);

    switch (op)
    {
    case SCOPE_OP_GET_INFO:        handle_get_info(seq, arg0);                          break;
    case SCOPE_OP_SET_CHANNEL:     handle_set_channel(seq, arg0, arg1, val);            break;
    case SCOPE_OP_SET_TIMING:      handle_set_timing(seq, val);                         break;
    case SCOPE_OP_SET_TRIGGER:     handle_set_trigger(seq, arg0, arg1, val);            break;
    case SCOPE_OP_ARM:             handle_arm(seq, arg0);                               break;
    case SCOPE_OP_GET_STATUS:      handle_get_status(seq, arg0);                        break;
    case SCOPE_OP_READ_BLOCK:      handle_read_block(seq, arg0, arg1, val);             break;
    case SCOPE_OP_ABORT:
        if ((s_state == SCOPE_ST_ARMED) || (s_state == SCOPE_ST_TRIGGERED) ||
            (s_state == SCOPE_ST_CALCULATING_CRC))
        {
            s_state = SCOPE_ST_IDLE;
        }
        s_read_active = false;
        send_resp(SCOPE_OP_ABORT, seq, 0U, 0U, 0U);
        break;
    case SCOPE_OP_RELEASE:
        s_state       = SCOPE_ST_IDLE;
        s_read_active = false;
        send_resp(SCOPE_OP_RELEASE, seq, 0U, 0U, 0U);
        break;
    case SCOPE_OP_STREAM:
        if (arg0 != 0U)
        {
            /* 协议：每次重新启动流递增 stream_id（上位机靠它区分新老流，
               同一个 id 的组会被当成同一段数据，重启后会画成连续的一坨） */
            s_stream_id++;
            if (s_stream_id == 0U) { s_stream_id = 1U; }
            s_stream_div = SCOPE_STREAM_PERIOD - 1U;   /* 立刻出第一组，别让上位机干等 20ms */
        }
        s_stream_on = (arg0 != 0U);
        send_resp(SCOPE_OP_STREAM, seq, 0U, arg0, 0U);
        break;
    case SCOPE_OP_GET_LIVE_STATE:
        send_resp(SCOPE_OP_GET_LIVE_STATE, seq, 0U, 0U,
                  ((uint32_t)s_live_state)        |
                  ((uint32_t)s_live_mode  << 8)   |
                  ((uint32_t)s_live_test  << 16)  |
                  ((uint32_t)s_live_stage << 24));
        break;
    default:
        send_resp(op, seq, 0x02U, 0U, 0U);      /* ERR_COMMAND */
        break;
    }
}

/* ---------------- 主循环服务 ---------------- */
static void pump_crc(void)
{
    uint32_t total = (uint32_t)s_ch_count * (uint32_t)s_points * 4U;
    uint32_t n = total - s_crc_done;
    if (n > SCOPE_CRC_SLICE) { n = SCOPE_CRC_SLICE; }
    s_crc_ctx = crc32_update(s_crc_ctx, &motor_scope_sample_buffer[s_crc_done], n);
    s_crc_done += n;
    if (s_crc_done >= total)
    {
        s_crc  = ~s_crc_ctx;
        s_state = SCOPE_ST_FROZEN;
    }
}

static void pump_read_block(void)
{
    uint8_t n = 0U;
    while (s_read_active && (s_read_left > 0U) && (n < SCOPE_TX_PER_TICK))
    {
        uint32_t off = chan_offset(s_read_ch, s_read_idx);
        uint32_t raw = 0U;
        if ((off + 4U) <= SCOPE_BUF_SIZE)
        {
            raw = rd_u32(&motor_scope_sample_buffer[off]);
        }
        send_data((uint8_t)(s_capture_id & 0xFFU), s_read_ch, s_read_idx, raw);
        s_read_idx++;
        s_read_left--;
        n++;
        if (s_read_idx >= s_points) { s_read_active = false; }
    }
    if (s_read_left == 0U) { s_read_active = false; }
}

/** 0xF1 的语义是【环路启用标志】（不是"通道有效位图"）：
 *    电流环 0x03、速度环 0x0C、位置环 0x30，可相或。
 *  本工程电流环常开；速度/位置二选一由编译期宏决定。
 *  PWM 未使能（STOP/DISABLE/故障关断）时报 0 —— 手册明确"电机使能与否不应
 *  通过伪造波形数值来表示"，这个标志位才是表达它的正地方。 */
static uint32_t stream_loop_flags(void)
{
    uint32_t f = 0U;

    if (!pwm_output_is_enabled()) { return 0U; }

    f |= 0x03U;                     /* 电流环 */
#if FOC_SPEED_MODE
    f |= 0x0CU;                     /* 速度环 */
#endif
#if FOC_POSITION_MODE
    f |= 0x30U;                     /* 位置环 */
#endif
    return f;
}

/** 趋势组【固定六路】保底：任何未配置的槽位都回落到默认绑定。
 *  协议要求一组必须是 0xF0 + 0xF1 + kind0..5 共 8 帧，缺帧上位机会画成间断；
 *  而 SET_CHANNEL 可能把某些槽位清空（比如上位机自选变量时只配了 2 路），
 *  所以发送前先把六个槽位补齐。 */
static void stream_bind_default(void)
{
    static const void *const def[SCOPE_MAX_CHANNELS] = {
        &jscope_iq_ref,                             /* 0 Iq_ref */
        &motor_scope_signals.iq_fdb_a,              /* 1 Iq_actual */
        &motor_scope_signals.speed_ref_rad_s,       /* 2 速度参考 */
        &jscope_encoder_speed_rad_s,                /* 3 速度反馈 */
        &jscope_position_ref_relative_rad,          /* 4 位置参考 */
        &jscope_position_feedback_relative_rad      /* 5 位置反馈 */
    };
    for (uint8_t c = 0U; c < SCOPE_MAX_CHANNELS; c++)
    {
        if (s_ch[c].used == 0U)
        {
            s_ch[c].addr = def[c];
            s_ch[c].type = SCOPE_T_F32;
            s_ch[c].used = 1U;
        }
    }
}

static void send_stream_group(void)
{
    stream_bind_default();

    s_stream_seq++;
    /* 0xF0 = 板端 20 kHz 计数时间戳（协议规定，不是毫秒！）
       g_heart_isr 在 20kHz ADC 中断里每拍 +1，正是这个计数。 */
    send_stream(0xF0U, s_stream_id, s_stream_seq, g_heart_isr);
    send_stream(0xF1U, s_stream_id, s_stream_seq, stream_loop_flags());
    for (uint8_t c = 0U; c < SCOPE_MAX_CHANNELS; c++)
    {
        send_stream(c, s_stream_id, s_stream_seq, chan_read(&s_ch[c]));
    }
}

void motor_scope_poll_1ms(void)
{
    s_ms++;

    if (s_state == SCOPE_ST_CALCULATING_CRC)
    {
        pump_crc();
    }
    pump_read_block();

    if (s_stream_on)
    {
        if (++s_stream_div >= SCOPE_STREAM_PERIOD)
        {
            s_stream_div = 0U;
            send_stream_group();
        }
    }
}

void motor_scope_set_live(uint8_t state, uint8_t mode, uint8_t test, uint8_t stage)
{
    s_live_state = state;
    s_live_mode  = mode;
    s_live_test  = test;
    s_live_stage = stage;
}

uint8_t motor_scope_state(void) { return s_state; }

void motor_scope_init(void)
{
    /* Build ID 是 const（在 flash），这里只做一次读，防止链接器以"未被引用"为由丢弃符号 */
    (void)motor_scope_build_id[0];

    (void)memset(&s_ch[0], 0, sizeof(s_ch));

    /* 默认通道 = 固定六路协议规定的物理顺序（见 motor_scope.h 头注）：
       0 Iq_ref / 1 Iq_actual / 2 速度参考 / 3 速度反馈 / 4 位置参考 / 5 位置反馈 */
    s_ch[0].addr = (const void *)&jscope_iq_ref;
    s_ch[1].addr = (const void *)&motor_scope_signals.iq_fdb_a;
    s_ch[2].addr = (const void *)&motor_scope_signals.speed_ref_rad_s;
    s_ch[3].addr = (const void *)&jscope_encoder_speed_rad_s;
    s_ch[4].addr = (const void *)&jscope_position_ref_relative_rad;
    s_ch[5].addr = (const void *)&jscope_position_feedback_relative_rad;
    for (uint8_t c = 0U; c < SCOPE_MAX_CHANNELS; c++)
    {
        s_ch[c].type = SCOPE_T_F32;
        s_ch[c].set  = 1U;          /* 默认全部可用于趋势流 */
        s_ch[c].used = 1U;
    }

    /* 采样缓冲首字节写一次，避免某些工具链把整个 64 KiB 优化掉 */
    motor_scope_sample_buffer[0] = 0U;
    (void)motor_scope_sample_buffer[0];

    s_state = SCOPE_ST_IDLE;
}
