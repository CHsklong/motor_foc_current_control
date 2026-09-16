/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file drv_flash.h
 * @brief 片内 FLASH 参数存储：编码器零点偏移的掉电保存
 *
 * 参数区占用片内 FLASH 的最后一个 sector（4KB，偏移 0xFF000 / 映射地址 0x800FF000）。
 * 程序本体只有 ~103KB，离末尾 sector 很远，不会被覆盖。
 *
 * 读：走 XIP 内存映射直接取指针，零开销，也无需初始化 XPI。
 * 写：走 ROM API（代码在 ROM / .ramfunc 中执行），擦除编程期间关全局中断。
 *
 * 上电流程见 drv_flash_load_theta()：FLASH 有有效数据就用它，
 * 没有就把编译期常量（app_cfg.h 的 ENC_THETA_INITIAL）烧进去，之后一直从 FLASH 读。
 */
#ifndef DRV_FLASH_H
#define DRV_FLASH_H

#include "hw_map.h"

#define FLASH_CALIB_MAGIC    (0x48454E43UL)   /**< 'HENC'，标记参数区有效 */
#define FLASH_CALIB_VERSION  (1U)             /**< 结构版本，改布局时递增 */

/**
 * @brief 存在 FLASH 里的编码器零点标定数据（16 字节，正好 4 个字）
 */
typedef struct {
    uint32_t magic;          /**< FLASH_CALIB_MAGIC */
    uint32_t version;        /**< FLASH_CALIB_VERSION */
    float    theta_initial;  /**< 初始机械弧度（零点偏移） */
    uint32_t checksum;       /**< 前三字的校验，防半写 / 位翻转 */
} flash_calib_t;

/**
 * @brief 探测 FLASH 几何（total_size / sector_size），确定参数区偏移
 *
 * 【不要在上电流程里主动调用】它内部的 ROM XPI 探测 API 会把 XPI 控制器重新
 * 初始化，覆盖 BootROM 为 XIP 配好的取指状态，导致控制节拍跑不完、电机异响。
 * 只有真正要擦写时才由 drv_flash_write_calib() 内部调用。
 *
 * 探测失败时回退到"1MB、末 4KB"的编译期假设，不影响纯读取路径。
 */
hpm_stat_t drv_flash_init(void);

/**
 * @brief 读参数区并校验
 * @param[out] out 读到的内容（校验失败时内容仍会填出，便于排查）
 * @retval status_success 数据有效
 * @retval status_fail    magic / version / checksum 任一项不对
 */
hpm_stat_t drv_flash_read_calib(flash_calib_t *out);

/**
 * @brief 擦除参数区所在 sector 并写入
 * @param[in] in 待写入的数据（checksum 由本函数补齐，调用方不必填）
 * @note 耗时几十 ms 且期间关中断，必须在 PWM 无输出、控制环未使能时调用
 */
hpm_stat_t drv_flash_write_calib(const flash_calib_t *in);

/**
 * @brief 上电加载零点偏移
 * @param default_val   FLASH 中无有效数据时使用的默认值（即 ENC_THETA_INITIAL）
 * @param auto_program  true=FLASH 无有效数据时把 default_val 回写进去；false=只读取
 * @return 实际使用的偏移值
 * @note auto_program=true 时会调用 ROM 的 XPI 探测 API（会重配 XPI、拖慢 XIP 取指），
 *       只应在专门的标定固件里用。常规运行必须传 false —— 那时本函数等价于
 *       一次 XIP 指针读，不碰任何 ROM API。
 */
float drv_flash_load_theta(float default_val, bool auto_program);

/**
 * @brief 保存零点偏移（自动对齐完成后调用）
 * @param theta 新的零点偏移（机械弧度）
 * @param force false=与已存值相差小于 0.001rad 就跳过，保护 FLASH 擦写寿命；true=强制写
 */
hpm_stat_t drv_flash_save_theta(float theta, bool force);

#endif /* DRV_FLASH_H */
