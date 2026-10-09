/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file motor_scope.h
 * @brief ELF 变量录波（上位机"ELF 与录波"页）：Build ID + 6 路通道 + 高速录波 + 趋势流
 *
 * ---------------------------------------------------------------------------
 * 【上位机三个硬要求，改代码前必须先看】
 * ---------------------------------------------------------------------------
 * 1) 128 位构建标识：上位机从【ELF 符号表】里找一个名为 motor_scope_build_id 的
 *    符号，取它 16 字节内容与板端 GET_INFO(field1..4) 回传值比对，
 *    不一致则"禁止按地址采集"（界面提示构建标识不匹配）。
 *    ⚠️ 不是读 ELF 的 .note.gnu.build-id —— 本工程的 GNU ld 与 SEGGER linker
 *       都不保证生成该 note，所以这里用显式符号，两种工具链通用。
 *
 * 2) 地址必须在允许的 DLM 范围：上位机内部常量为 [0x00080000, 0x000A0000)。
 *    本工程 .data/.bss 落在 0x00080300 之后，天然满足；
 *    ⚠️ 但若把变量声明成 const 放进 .rodata(0x80000000+，XIP flash) 就会被拒。
 *
 * 3) 变量类型靠 DWARF 解析（.debug_info）：必须用 Debug 构建（带 -g），
 *    Release 构建没有 DWARF，上位机认不出变量类型 → 变量"不可采集"。
 *
 * ---------------------------------------------------------------------------
 * 【上位机预选的 6 路通道（硬编码变量名，见快速使用说明 §ELF 与波形 2）】
 * ---------------------------------------------------------------------------
 *   槽位0 jscope_iq_ref
 *   槽位1 jscope_iq_amp
 *   槽位2 motor_scope_signals.speed_ref_rad_s   （结构体成员，靠 DWARF 展开）
 *   槽位3 jscope_encoder_speed_rad_s
 *   槽位4 jscope_position_ref_relative_rad
 *   槽位5 jscope_position_feedback_relative_rad
 * 六个符号都必须存在且为 float，否则对应通道空置（上位机不会用别的变量顶替）。
 *
 * ---------------------------------------------------------------------------
 * 【CAN 帧】
 * ---------------------------------------------------------------------------
 *   RX 0x400+NodeID (0x401) 示波请求  <BBBBI> op seq arg0 arg1 value:u32
 *   TX 0x480+NodeID (0x481) 示波响应  <BBBBI> op seq result field value:u32
 *   TX 0x500+NodeID (0x501) 录波数据  <BBHI>  capture_id channel index:u16 raw:u32
 *   TX 0x520+NodeID (0x521) 连续趋势  <BBHI>  kind stream_id sample_seq:u16 value:u32
 *        kind 0..5 = 通道槽位；0xF0 = 20kHz 计数时间戳；0xF1 = 环路启用标志
 *
 * ---------------------------------------------------------------------------
 * 【固定六路趋势（不导入 ELF 也能看的那个）协议要点】
 * ---------------------------------------------------------------------------
 * 依据《其他固件接入固定六路CAN波形操作手册》：
 *   1) GET_INFO(field=0) 必须回【版本 2】—— 上位机只在收到 2 之后才发 STREAM 启动。
 *      回 1 的表现是六个窗口全空白、且没有任何报错。
 *   2) 一组趋势必须是【8 帧】：0xF0 + 0xF1 + kind0..5，缺帧 = 波形间断。
 *      0xF1 是环路启用标志（电流 0x03 / 速度 0x0C / 位置 0x30），不是通道位图。
 *   3) 通道物理含义固定：0=Iq_ref 1=Iq_actual 2=速度参考 3=速度反馈 4=位置参考 5=位置反馈。
 *      单位：电流 A、速度【机械 rad/s】、位置【机械 rad，相对同一零点、跨圈连续】。
 *      上位机显示时自行换算成 rpm / 度。
 *   4) 每次重启流递增 stream_id；一组内 8 帧共用同一 stream_id + sample_seq。
 */
#ifndef MOTOR_SCOPE_H
#define MOTOR_SCOPE_H

#include <stdint.h>
#include <stdbool.h>

#include "can_protocol.h"

/* ---------------- CAN ID ---------------- */
#define SCOPE_ID_REQ     (0x400U + CAN_NODE_ID)   /* RX 示波请求 */
#define SCOPE_ID_RSP     (0x480U + CAN_NODE_ID)   /* TX 示波响应 */
#define SCOPE_ID_DATA    (0x500U + CAN_NODE_ID)   /* TX 录波数据（每帧一个采样点） */
#define SCOPE_ID_STREAM  (0x520U + CAN_NODE_ID)   /* TX 连续趋势 20ms */

/* ---------------- 示波操作码（op） ---------------- */
#define SCOPE_OP_GET_INFO       0x01U   /* 字段 0..6：版本/Build ID×4/缓冲区/20kHz */
#define SCOPE_OP_SET_CHANNEL    0x02U   /* arg0=槽位 arg1=set位|类型 value=变量地址 */
#define SCOPE_OP_SET_TIMING     0x03U   /* value 高16位=点数 低16位=分频 */
#define SCOPE_OP_SET_TRIGGER    0x04U   /* arg0=模式 arg1=test_type value低16=pretrigger */
#define SCOPE_OP_ARM            0x05U   /* arg0=通道数 value=host_request_id → 回 capture_id */
#define SCOPE_OP_GET_STATUS     0x06U   /* 字段 0..7 */
#define SCOPE_OP_READ_BLOCK     0x07U   /* arg0=capture_id arg1=通道 value 高16=数量 低16=起点 */
#define SCOPE_OP_ABORT          0x08U
#define SCOPE_OP_RELEASE        0x09U
#define SCOPE_OP_STREAM         0x0AU   /* arg0: 0关 1开 */
#define SCOPE_OP_GET_LIVE_STATE 0x0BU   /* 回 state/mode/test/stage，每字节一个字段 */

/* ---------------- 采集状态 ---------------- */
#define SCOPE_ST_IDLE            0U
#define SCOPE_ST_ARMED           1U
#define SCOPE_ST_TRIGGERED       2U
#define SCOPE_ST_FROZEN          3U
#define SCOPE_ST_CALCULATING_CRC 4U
#define SCOPE_ST_ERROR           5U

/* ---------------- 标量类型（与上位机 ScalarType 一致） ---------------- */
#define SCOPE_T_U8   1U
#define SCOPE_T_I8   2U
#define SCOPE_T_U16  3U
#define SCOPE_T_I16  4U
#define SCOPE_T_U32  5U
#define SCOPE_T_I32  6U
#define SCOPE_T_F32  7U
#define SCOPE_T_BOOL 8U

/* ---------------- 触发模式 ---------------- */
#define SCOPE_TRIG_IMMEDIATE 0U   /* 立即触发：ARM 后马上开始采 */
#define SCOPE_TRIG_TEST      1U   /* 等下一次阶跃/运动开始（motor_scope_notify_trigger） */

/* ---------------- 容量 ---------------- */
#define SCOPE_BUF_SIZE      (65536U)   /* 静态采样缓冲 64 KiB（协议规定值） */
#define SCOPE_MAX_CHANNELS  (6U)       /* 高速 1~4；趋势最多 6 */
#define SCOPE_MAX_POINTS   (16384U)   /* 单通道点数上限（4 通道×4 字节=64KiB 恰好用满） */
#define SCOPE_PRETRIG_MAX   (512U)     /* 预触发环形缓冲点数（×4 通道 ×4 字节 = 8 KiB） */

/* ---------------- 变量声明（定义见 motor_scope_service.h） ----------------
 * 上位机按【符号名 + DWARF 类型 + 声明文件】认变量，三点都要对得上：
 *   - 名字：motor_scope_signals / jscope_* / motor_scope_build_id
 *   - 类型：motor_scope_signals_t（结构体）、float、const uint32_t[4]
 *   - 声明文件：motor_scope_service.h
 * 所以定义统一放在 motor_scope_service.h，其他模块只 include 本文件拿 extern。 */

/** 128 位构建标识（const volatile → flash .rodata，必须能被 ELF 读到真实字节） */
extern const volatile uint32_t motor_scope_build_id[4];

/** 六路预选通道（全部 float、全部在 DLM 0x80000~0xA0000） */
typedef struct {
    float speed_ref_rad_s;              /* 槽位2：速度给定 */
    float speed_fdb_rad_s;              /* 速度反馈 */
    float position_ref_relative_rad;    /* 位置给定 */
    float position_fdb_relative_rad;    /* 位置反馈 */
    float iq_ref_a;                     /* q 轴电流给定 */
    float iq_fdb_a;                     /* 电流矢量幅值 */
} motor_scope_signals_t;

extern motor_scope_signals_t motor_scope_signals;

extern float jscope_iq_ref;
extern float jscope_iq_amp;
extern float jscope_encoder_speed_rad_s;
extern float jscope_position_ref_relative_rad;
extern float jscope_position_feedback_relative_rad;

/** 静态采样缓冲（协议规定名字与大小，64 KiB） */
extern uint8_t motor_scope_sample_buffer[SCOPE_BUF_SIZE];

/* ---------------- 对外 API ---------------- */
/** 上电初始化：填 Build ID、清状态、把默认通道指向六路预选变量 */
void motor_scope_init(void);

/** 20 kHz 中断里调用：刷新六路通道变量 + 按分频采样 */
void motor_scope_isr_tick(void);

/** 主循环 1ms 一拍：CRC 分块计算、趋势流发送、READ_BLOCK 排队回传 */
void motor_scope_poll_1ms(void);

/** 0x401 请求入口（8 字节负载） */
void motor_scope_handle_req(const uint8_t d[8]);

/** 触发源：下一次阶跃/运动开始时调用（触发模式=1 时用） */
void motor_scope_notify_trigger(void);

/** 由 can_app 每次状态变化时写入：GET_LIVE_STATE 回给上位机的 state/mode/test/stage */
void motor_scope_set_live(uint8_t state, uint8_t mode, uint8_t test, uint8_t stage);

/** 当前采集状态，J-Scope 排障用 */
uint8_t motor_scope_state(void);

#endif /* MOTOR_SCOPE_H */
