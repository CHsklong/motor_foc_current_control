/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file motor_scope_service.h
 * @brief 录波变量的【定义】文件（上位机按 DWARF 的 decl_file 与类型名认变量）
 *
 * ---------------------------------------------------------------------------
 * 为什么单独一个头文件、而且把定义（不是 extern）写在里面？
 * ---------------------------------------------------------------------------
 * 上位机 app/elf/symbols.py 里硬编码了两个名字：
 *   - 变量名 motor_scope_signals（类型 motor_scope_signals_t），成员按 "结构体.成员"
 *     形式展开，对应通道 motor_scope_signals.speed_ref_rad_s；
 *   - 声明文件名 motor_scope_service.h —— 它用 DWARF 的 DW_AT_decl_file 做过滤，
 *     变量定义在别的文件里会被判成"不可采集"。
 * 所以本文件必须叫这个名字，且变量【定义】在这里（只被 motor_scope.c 包含一次，
 * 不会引发多重定义）。
 *
 * ---------------------------------------------------------------------------
 * 两段摆放位置的区别（很关键，别随手改）：
 * ---------------------------------------------------------------------------
 * 1) motor_scope_build_id 必须是【const → flash .rodata(PROGBITS)】。
 *    上位机 _read_build_id() 是 elf.get_section(st_shndx).data()[st_value-sh_addr : +16]，
 *    而 SES 生成的 ELF 里 DLM 上的 .data/.bss 段全是 SHT_NOBITS（文件里不存内容），
 *    放在 RAM 里会读到 16 个零 → 与板端回传值不一致 → 上位机"禁止按地址采集"。
 *    flash 段有真实字节，且 build id 的读取不做 DLM 地址校验，所以放这里。
 *
 * 2) 六路通道变量必须留在【RAM(DLM 0x80000~0xA0000)】。
 *    上位机只对采集变量做 DLM 范围校验；变量只送地址给板端、由板端读自己的 RAM，
 *    不需要 ELF 里有内容，所以放 .bss 没问题。
 */
#ifndef MOTOR_SCOPE_SERVICE_H
#define MOTOR_SCOPE_SERVICE_H

#include <stdint.h>

#include "motor_scope.h"   /* 复用 motor_scope_signals_t 类型定义，避免重复 typedef */

/* ---------------- 128 位构建标识（flash，PROGBITS，上位机要读这 16 字节） ----------
 * 改固件后想让上位机强制重新校验，把 W3 加一即可（板端与 ELF 同源，必然一致）。 */
#define MOTOR_SCOPE_BUILD_ID_W0  (0x45323439U)   /* 'E249' */
#define MOTOR_SCOPE_BUILD_ID_W1  (0x484D3533U)   /* 'HM53' */
#define MOTOR_SCOPE_BUILD_ID_W2  (0x464F4331U)   /* 'FOC1' */
#define MOTOR_SCOPE_BUILD_ID_W3  (0x00000001U)   /* 版本 */

/* ⚠️ volatile + used 都不能删：
 *   volatile —— 阻止编译器把"读 const 元素"常量传播成立即数（否则对数组的引用消失）；
 *   used     —— 阻止链接器 --gc-sections 把没被引用的符号从符号表里抹掉。
 * 两个都去掉的话，上位机会报"ELF缺少motor_scope_build_id"，然后禁止按地址采集。 */
const volatile uint32_t motor_scope_build_id[4] __attribute__((used)) = {
    MOTOR_SCOPE_BUILD_ID_W0,
    MOTOR_SCOPE_BUILD_ID_W1,
    MOTOR_SCOPE_BUILD_ID_W2,
    MOTOR_SCOPE_BUILD_ID_W3
};

/* ---------------- 采集变量（RAM / DLM） ----------------
 * 类型 motor_scope_signals_t 定义在 motor_scope.h（本文件已 include），
 * 这里只做定义。上位机按 "motor_scope_signals.speed_ref_rad_s" 找成员。 */
motor_scope_signals_t motor_scope_signals;

float jscope_iq_ref;                          /* 槽位0：q 轴电流给定 A */
float jscope_iq_amp;                          /* 槽位1：电流矢量幅值 A */
float jscope_encoder_speed_rad_s;             /* 槽位3：编码器速度 rad/s */
float jscope_position_ref_relative_rad;       /* 槽位4：位置给定 rad */
float jscope_position_feedback_relative_rad;  /* 槽位5：位置反馈 rad */

/** 静态采样缓冲 64 KiB（RAM / DLM，只送地址给板端，不在 ELF 里存内容） */
uint8_t motor_scope_sample_buffer[65536];

#endif /* MOTOR_SCOPE_SERVICE_H */
