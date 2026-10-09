/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file can_app.h
 * @brief CAN 通信接口（MCAN0，CAN2.0A 500kbps，配 CANTest/CANalyst-II）
 *
 * 协议、ID 表见 can_app.c 文件头注释。
 * J-Scope 可观测：g_can_rx_cnt / g_can_tx_cnt / g_can_drop_cnt /
 *                 g_can_overrun / g_can_spd_cmd / g_can_err
 */
#ifndef CAN_APP_H
#define CAN_APP_H

#include <stdint.h>

#include "can_protocol.h"

/** 初始化 MCAN0：时钟 + 引脚(PA00/PA01) + 500kbps + RX FIFO0 中断。
 *  CAN_CTRL_ENABLE=0 时为空实现。 */
void can_app_init(void);

/** 主循环每拍（约 1ms）调用一次：解析 0x201/0x221 命令、
 *  回 0x181 ACK、周期发 0x281(20ms)/0x301(100ms) 状态帧。 */
void can_app_poll(void);

/** 给 debug/motor_scope.c 用的裸发送：直接把 8 字节负载按指定 ID 发出去。
 *  CAN_CTRL_ENABLE=0 时为空实现（录波静默失效，不影响控制）。 */
void can_app_send_raw(uint32_t std_id, uint8_t dlc, const uint8_t *data);

/* ---- 观测计数（J-Scope）---- */
extern volatile uint32_t g_can_rx_cnt;     /* 收到的命令帧总数 */
extern volatile uint32_t g_can_tx_cnt;     /* 发出的遥测+应答帧总数 */
extern volatile uint32_t g_can_drop_cnt;   /* 因模式不匹配/规划器忙被丢弃的命令数 */
extern volatile uint32_t g_can_overrun;    /* 接收环形缓冲溢出次数 */
extern volatile float    g_can_spd_cmd;    /* 最近一次速度命令 rad/s */
extern volatile uint32_t g_can_err;        /* 1=MCAN 初始化失败（查接线/时钟） */
extern volatile uint32_t g_can_init_stat;  /* mcan_init 原始返回码：
                                              0=成功  2=参数无效  3=通用超时(INIT应答没来，功能时钟没跑)
                                              26013=MCAN 超时  26014=位时序无效(时钟频率算不出目标波特率) */
extern volatile uint32_t g_can_src_clk_khz;/* CAN 功能时钟实测值 kHz，正常应为 40000 */

/* ---- 协议状态机镜像（J-Scope 排障用，枚举见 can_protocol.h） ---- */
extern volatile uint32_t g_can_state;  /* 主状态 ST_STANDBY/READY/ACTIVE/STOPPING/TEST/FAULT */
extern volatile uint32_t g_can_mode;   /* 0未选 / 1速度 / 2位置 */
extern volatile uint32_t g_can_comm;   /* 0等待 / 1心跳正常 / 2心跳超时 */
extern volatile uint32_t g_can_fault;  /* 故障位图 FLT_* */

#endif /* CAN_APP_H */
