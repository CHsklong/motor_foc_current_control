/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file can_protocol.h
 * @brief E249 CAN 控制协议常量（与 E249_CAN控制协议.xlsx v0.2 逐条对齐）
 *
 * 帧格式：经典 CAN 2.0A 标准帧 11-bit，DLC 固定 8，多字节字段小端。
 * 波特率 500 kbit/s（app_cfg.h 的 CAN_BAUDRATE_KHZ）。
 *
 * ID 分配（NodeID=1）：
 *   RX 0x201 控制/模式/心跳/测试     RX 0x221 运动指令
 *   RX 0x241 候选参数写入            RX 0x261 参数操作
 *   TX 0x181 命令应答 ACK            TX 0x281 快速状态 20ms
 *   TX 0x301 慢速状态 100ms          TX 0x381 参数响应
 *
 * ⚠️ 与旧自定协议（板->PC 0x201 遥测 / PC->板 0x101、0x102）不兼容：
 *    0x201 在本协议里是【上位机->板】的控制帧，方向正好相反，旧遥测必须删除。
 */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

/* ---------------- NodeID 与 CAN ID（协议总览 §2） ---------------- */
#ifndef CAN_NODE_ID
#define CAN_NODE_ID        (1U)          /* 1~15，与上位机 NodeID 设置一致 */
#endif

#define CAN_ID_CTRL        (0x200U + CAN_NODE_ID)   /* RX 0x201 控制/模式/心跳/测试 */
#define CAN_ID_MOVE        (0x220U + CAN_NODE_ID)   /* RX 0x221 运动指令（速度/位置/点动） */
#define CAN_ID_PARAM_WR    (0x240U + CAN_NODE_ID)   /* RX 0x241 候选参数写入 */
#define CAN_ID_PARAM_OP    (0x260U + CAN_NODE_ID)   /* RX 0x261 参数操作/读取/保存 */
#define CAN_ID_ACK         (0x180U + CAN_NODE_ID)   /* TX 0x181 命令应答 */
#define CAN_ID_FAST        (0x280U + CAN_NODE_ID)   /* TX 0x281 快速状态 20ms */
#define CAN_ID_SLOW        (0x300U + CAN_NODE_ID)   /* TX 0x301 慢速状态 100ms */
#define CAN_ID_PARAM_RSP   (0x380U + CAN_NODE_ID)   /* TX 0x381 参数响应 */

/* 保留：本工程自留的连通性自检帧（不属于协议，便于无上位机时排查链路） */
#define CAN_ID_ECHO_REQ    (0x100U)
#define CAN_ID_ECHO_RSP    (0x1F0U)

/* ---------------- 控制命令 0x201 B0（控制与运动 §2） ---------------- */
#define CMD_HEARTBEAT      0x01U   /* 刷新通信看门狗，100ms 周期          */
#define CMD_SELECT_MODE    0x02U   /* B2: 1=速度 2=位置                   */
#define CMD_ENABLE         0x03U   /* 使能已选模式                        */
#define CMD_STOP           0x04U   /* 安全停止：目标归零，回待机          */
#define CMD_DISABLE        0x05U   /* 关 PWM，转轴失去保持力              */
#define CMD_CLEAR_FAULT    0x06U   /* 清除可恢复故障                      */
#define CMD_START_TEST     0x10U   /* B2: 1=D电流 2=Q电流 3=速度 4=位置   */
#define CMD_STOP_TEST      0x11U   /* 停止循环测试                        */

/* ---------------- 运动命令 0x221 B0（控制与运动 §4） ---------------- */
#define MOV_SET_SPEED      0x01U   /* int32  0.001 rpm   ±1000 rpm        */
#define MOV_SET_ACCEL      0x02U   /* uint32 0.001 rpm/s                  */
#define MOV_SET_DECEL      0x03U   /* uint32 0.001 rpm/s                  */
#define MOV_POS_ABS        0x10U   /* int32  0.001°      归一化最短路径    */
#define MOV_POS_REL        0x11U   /* int32  0.001°      ±180°            */
#define MOV_POS_ZERO       0x12U   /* 设当前位置为用户 0°（仅 RAM）        */
#define MOV_POS_HOLD       0x13U   /* 目标 = 当前反馈位置                  */
#define MOV_JOG_POS        0x20U   /* 正向点动，需每 100ms 续发            */
#define MOV_JOG_NEG        0x21U   /* 反向点动，需每 100ms 续发            */

/* ---------------- ACK 结果码（状态与应答 §2） ---------------- */
#define ACK_OK             0x00U
#define ACK_ERR_DLC        0x01U   /* DLC 不是 8                          */
#define ACK_ERR_COMMAND    0x02U   /* 未知命令                            */
#define ACK_ERR_STATE      0x03U   /* 当前状态不允许                      */
#define ACK_ERR_RANGE      0x04U   /* 数值越界                            */
#define ACK_ERR_MODE       0x05U   /* 模式不匹配                          */
#define ACK_ERR_FAULT      0x06U   /* 存在故障                            */
#define ACK_ERR_BUSY       0x07U   /* 控制器忙                            */
#define ACK_ERR_TIMEOUT    0x08U   /* 通信超时                            */
#define ACK_ERR_PARAM      0x09U   /* 参数/保留字错误                      */
#define ACK_ERR_FLASH      0x0AU   /* Flash 保存失败                      */

/* ---------------- ACK source_class（状态与应答 §1） ---------------- */
#define CLS_CTRL           1U      /* 来自 0x201 */
#define CLS_MOVE           2U      /* 来自 0x221 */
#define CLS_PARAM_WR       3U      /* 来自 0x241 */
#define CLS_PARAM_OP       4U      /* 来自 0x261 */

/* ---------------- 主状态（数值由本实现固化） ---------------- */
#define ST_STANDBY         0U      /* 上电/停机，未使能，PWM 可关          */
#define ST_READY           1U      /* 模式已选，等待 ENABLE                */
#define ST_ACTIVE          2U      /* 已使能运行中                        */
#define ST_STOPPING        3U      /* 停止过程中（归零斜坡未完成）          */
#define ST_TEST            4U      /* 循环测试运行                        */
#define ST_FAULT           5U      /* 故障                                */

/* ---------------- 模式 ---------------- */
#define MODE_NONE          0U
#define MODE_SPEED         1U
#define MODE_POSITION      2U

/* ---------------- comm_status（状态与应答 §4） ---------------- */
#define COMM_WAIT          0U      /* 等待/离线 */
#define COMM_OK            1U      /* 通信正常 */
#define COMM_TIMEOUT       2U      /* 心跳超时 */

/* ---------------- mode_flags 位（状态与应答 §3） ---------------- */
#define FLAG_PWM           0x01U   /* bit0 PWM 输出使能 */
#define FLAG_ENABLE        0x02U   /* bit1 已使能       */
#define FLAG_HEARTBEAT     0x04U   /* bit2 心跳有效     */
#define FLAG_FAULT         0x08U   /* bit3 故障         */
#define FLAG_TEST          0x10U   /* bit4 测试中       */
#define FLAG_CALIB         0x20U   /* bit5 校准有效     */
#define FLAG_POS_VALID     0x40U   /* bit6 位置目标有效 */

/* ---------------- 故障位图（本实现定义，协议说"随故障模块固化"） ---------------- */
#define FLT_PWM_LATCH      0x0001U /* PWM 硬件过流锁存（g_pwm_fault）      */
#define FLT_MCL_ANALOG     0x0002U /* MCL: 采样/过流   g_fault_src bit0    */
#define FLT_MCL_LOOP       0x0004U /* MCL: 环路        bit1                */
#define FLT_MCL_DRIVER     0x0008U /* MCL: 驱动        bit2                */
#define FLT_MCL_ENCODER    0x0010U /* MCL: 编码器      bit3                */
#define FLT_COMM_TIMEOUT   0x0020U /* 心跳超时                             */

/* ---------------- 单位换算（协议用 rpm/deg，固件用 rad/s/rad） ---------------- */
/** 0.001 rpm -> rad/s：(milli_rpm/1000) * 2π/60 */
#define MRPM_TO_RADS       (1.0471975512e-4f)
/** rad/s -> 0.1 rpm（快速状态帧单位） */
#define RADS_TO_DRPM       (95.49296586f)
/** 0.001° -> rad */
#define MDEG_TO_RAD        (1.745329252e-5f)
/** rad -> 0.001°（慢速状态帧 int24 单位） */
#define RAD_TO_MDEG        (57295.77951f)

/* ---------------- 限值（控制与运动 §4） ---------------- */
#define SPEED_LIMIT_MRPM   (1000000L)    /* ±1000 rpm   */
#define POS_REL_LIMIT_MDEG (180000L)     /* ±180°       */
#define JOG_SPEED_MRPM     (30000L)      /* 点动默认 ±30 rpm */

/* ---------------- SET_ACCEL / SET_DECEL 的写入窗口 ----------------
 * 协议只规定了单位（0.001 rpm/s、uint32），没给上下限，这里按"电机能力 + 可用性"收口：
 *   上限 60000 rpm/s ≈ 6283 rad/s^2：电机能力约 26130 rad/s^2（i_max=3A / Kt / J），留足裕量；
 *   下限 1 rpm/s：再低就不是"斜坡"而是"几分钟爬不完"，多半是发错了。
 * 超出回 ACK_ERR_RANGE，上位机那边能立刻看出来。 */
#define ACCEL_MAX_MRPMS    (60000000L)   /* 60000 rpm/s */
#define ACCEL_MIN_MRPMS    (1000L)       /* 1 rpm/s     */

/* ---------------- 周期（协议总览 §2） ---------------- */
#define CAN_FAST_PERIOD_MS (20U)         /* 0x281 快速状态 */
#define CAN_SLOW_PERIOD_MS (100U)        /* 0x301 慢速状态 */
#define CAN_HB_TIMEOUT_MS  (3600000UL)   /* 心跳超时 1 小时 */
#define CAN_JOG_TIMEOUT_MS (300U)        /* 点动 300ms 未续发自动停 */

#endif /* CAN_PROTOCOL_H */
