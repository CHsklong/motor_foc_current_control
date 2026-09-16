/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file drv_flash.c
 * @brief 片内 FLASH 参数存储实现（ROM API）
 */
#include "drv_flash.h"
#include "hpm_romapi.h"
#include "dbg_probe.h"

/* ---------------- 参数区地址 ---------------- */
/* 最后一个 sector。BOARD_FLASH_SIZE = 1MB，故默认偏移 0xFF000、映射地址 0x800FF000。
   ROM API 收的是"相对 FLASH 起始的偏移"，CPU 直接读要用 XPI 映射地址，两者别混。

   注意：HPM5361 没有 HPM_XPI_SOC_SUPPORT_HYBRID_MODE（只有 HPM5E00 有），
   所以不走 0xB0000000 那个 hybrid 窗口，擦写地址就是相对偏移，与
   samples/rom_api/xpi_nor_api 的写法一致。

   真实的 total_size / sector_size 由上电时的 drv_flash_init() 从 ROM API 探测，
   探测成功会覆盖 s_param_offset —— 片内 FLASH 的 sector 未必是 4KB，不能想当然。 */
#define FLASH_PARAM_SECTOR_SIZE (0x1000UL)
#define FLASH_PARAM_OFFSET_DFT  (BOARD_FLASH_SIZE - FLASH_PARAM_SECTOR_SIZE)
#define FLASH_PARAM_ADDR        (BOARD_FLASH_BASE_ADDRESS + s_param_offset)

/** 回写阈值：小于 0.001rad(约 0.057°) 认为没变化，不擦写 */
#define FLASH_THETA_EPS        (0.001f)

/* ---------------- J-Scope 探针（供 HSS 直读） ---------------- */
volatile int32_t g_flash_status = 0;   /* 最近一次读/写的 hpm_stat_t，0=成功 */
volatile uint8_t g_flash_valid = 0;   /* 1=FLASH 中有有效标定数据 */
volatile float   g_flash_theta = 0.0f; /* 本次实际使用的零点偏移 */
volatile uint32_t g_flash_erase_ms = 0;/* 上次擦写耗时 ms，0=本次上电没擦写过 */
volatile uint32_t g_flash_erase_cycles = 0; /* 上次擦写耗时 CPU 周期 */
volatile uint32_t g_flash_ofs = FLASH_PARAM_OFFSET_DFT; /* 实际使用的参数区偏移 */

/** 探测到的参数区偏移（相对 FLASH 起始） */
static uint32_t s_param_offset = FLASH_PARAM_OFFSET_DFT;
static bool     s_geom_probed = false;

static void flash_option_fill(xpi_nor_config_option_t *option)
{
    option->header.U  = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    option->option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    option->option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;
}

static uint32_t theta_to_bits(float f)
{
    union { float f; uint32_t u; } v;
    v.f = f;
    return v.u;
}

static uint32_t calib_checksum(uint32_t magic, uint32_t version, float theta)
{
    return magic ^ version ^ theta_to_bits(theta) ^ 0xA5A5A5A5UL;
}

hpm_stat_t drv_flash_read_calib(flash_calib_t *out)         //读取
{
    const flash_calib_t *p = (const flash_calib_t *)FLASH_PARAM_ADDR;

    out->magic         = p->magic;
    out->version       = p->version;
    out->theta_initial = p->theta_initial;
    out->checksum      = p->checksum;

    if ((out->magic != FLASH_CALIB_MAGIC) || (out->version != FLASH_CALIB_VERSION)) {
        return status_fail;
    }
    if (out->checksum != calib_checksum(out->magic, out->version, out->theta_initial)) {
        return status_fail;
    }
    return status_success;
}

/**
 * @brief 探测 FLASH 几何，确定参数区落在哪个偏移
 *
 * 【警告】本函数会调用 ROM 的 XPI 探测 API（get_config / get_property），
 * 它们会把 XPI 控制器重新初始化，覆盖 BootROM 为 XIP 配好的高速取指状态 →
 * 20kHz 控制节拍跑不完 → 电机异响、速度环起不来。
 *
 * 因此**不要在上电流程里主动调用**，只有真正要擦写时才由 drv_flash_write_calib()
 * 内部调用（配合 app_cfg.h 的 FLASH_PARAM_CALIB_ONCE 使用）。
 *
 * 探测失败时 s_param_offset 保持编译期假设的 1MB-4KB，纯读取路径不受影响。
 */
hpm_stat_t drv_flash_init(void)
{
    xpi_nor_config_t nor_cfg;
    xpi_nor_config_option_t option;
    uint32_t total = 0U, sector = 0U;

    flash_option_fill(&option);

    hpm_stat_t status = rom_xpi_nor_get_config(BOARD_APP_XPI_NOR_XPI_BASE, &nor_cfg, &option);
    if (status != status_success) {
        s_geom_probed = false;      /* 回退到编译期假设的 1MB-4KB */
        g_flash_ofs   = s_param_offset;
        return status;
    }

    hpm_stat_t st_total  = rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &nor_cfg,
                                                    xpi_nor_property_total_size, &total);
    hpm_stat_t st_sector = rom_xpi_nor_get_property(BOARD_APP_XPI_NOR_XPI_BASE, &nor_cfg,
                                                    xpi_nor_property_sector_size, &sector);
    if ((st_total == status_success) && (st_sector == status_success) &&
        (sector > 0U) && (total > sector)) {
        s_param_offset = total - sector;
        s_geom_probed  = true;
    }
    g_flash_ofs = s_param_offset;

    return status;
}

/**
 * @brief 擦除 + 编程，必须在 RAM 中执行
 *
 * 本工程是 flash_xip：代码在 FLASH 里跑。擦除/编程期间 FLASH 不可读，
 * 一旦 CPU 需要取指（哪怕是进中断查向量表）就会跑飞。所以：
 *   1) 整个函数放 .ramfunc（ATTR_RAMFUNC），ROM API 被内联进来也仍在 RAM；
 *   2) 期间关全局中断，杜绝任何取指请求；
 *   3) 函数体内不调用任何库函数（memcpy 等在 FLASH 中），数据由调用方准备好。
 *
 * 【关键前提】ROM 的 XPI 探测类 API（get_config / auto_config / get_property）
 * **全都会**把 XPI 控制器重新初始化，覆盖 BootROM 为 XIP 配好的高速取指状态。
 * 实测：只要上电调过一次，哪怕最终没擦写，电机也会异响、速度环起不来。
 *
 * 但擦写又绕不开这些 API（erase/program 需要 nor_cfg）。所以本函数只允许在
 * FLASH_PARAM_CALIB_ONCE=1 的标定固件里被调用一次，标定完就烧回常规固件
 * —— 常规运行时整条 flash 路径一个 ROM API 都不碰，只做 XIP 指针读。
 *
 * 这里先用 get_config（按字面语义只取配置），取不到才退回 auto_config。
 */
ATTR_RAMFUNC static hpm_stat_t flash_erase_program(const uint32_t *buf, uint32_t bytes)     //擦
{
    xpi_nor_config_t nor_cfg;
    xpi_nor_config_option_t option;
    hpm_stat_t status;
    /* 计时只用 inline 的 CSR 读取，换算放到 ramfunc 外面做 ——
       clock_get_frequency() 在 FLASH 里，不能在这里调。 */
    uint32_t t0 = (uint32_t)hpm_csr_get_core_cycle();

    flash_option_fill(&option);

    status = rom_xpi_nor_get_config(BOARD_APP_XPI_NOR_XPI_BASE, &nor_cfg, &option);
    if (status != status_success) {
        /* 连配置都取不到才退回 auto_config —— 它会重配 XPI，是下下策 */
        status = rom_xpi_nor_auto_config(BOARD_APP_XPI_NOR_XPI_BASE, &nor_cfg, &option);
        if (status != status_success) {
            return status;
        }
    }

    uint32_t level = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    status = rom_xpi_nor_erase(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto,
                               &nor_cfg, s_param_offset, bytes);
    if (status == status_success) {
        status = rom_xpi_nor_program(BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto,
                                     &nor_cfg, buf, s_param_offset, bytes);
    }
    restore_global_irq(level);
    fencei();

    g_flash_erase_cycles = (uint32_t)hpm_csr_get_core_cycle() - t0;

    return status;
}

hpm_stat_t drv_flash_write_calib(const flash_calib_t *in)           //擦写
{
    /* program 要求 4 字节对齐的字缓冲，长度也是字节数 */
    uint32_t buf[sizeof(flash_calib_t) / sizeof(uint32_t)];

    for (uint32_t i = 0; i < (sizeof(buf) / sizeof(buf[0])); i++) {
        buf[i] = 0xFFFFFFFFUL;
    }
    buf[0] = in->magic;
    buf[1] = in->version;
    buf[2] = theta_to_bits(in->theta_initial);
    buf[3] = calib_checksum(in->magic, in->version, in->theta_initial);

    /* 擦写前先探测几何（会调 ROM 的 XPI 探测 API，只在真正要擦写时才走到这里）。
       探测失败就用编译期假设的 1MB-4KB，不阻断流程。 */
    (void)drv_flash_init();

    hpm_stat_t status = flash_erase_program(buf, sizeof(buf));

    /* 周期数换算成 ms（在 FLASH 中执行，安全） */
    uint32_t khz = clock_get_frequency(clock_cpu0) / 1000U;
    g_flash_erase_ms = (khz > 0U) ? (g_flash_erase_cycles / khz) : 0U;

    return status;
}

float drv_flash_load_theta(float default_val, bool auto_program)
{
    flash_calib_t c;
    hpm_stat_t status = drv_flash_read_calib(&c);

    if (status == status_success) {
        g_flash_status = (int32_t)status;
        g_flash_valid  = 1;
        g_flash_theta  = c.theta_initial;
        return c.theta_initial;
    }

    /* 走到这里说明 FLASH 里没有有效数据 */
    g_flash_status = (int32_t)status;
    g_flash_valid  = 0;
    g_flash_theta  = default_val;

    if (!auto_program) {
        return default_val;   /* 只读取，不擦写 */
    }

    /* 首次上电（或数据损坏）：把编译期常量烧进 FLASH，之后每次上电都从 FLASH 命中 */
    c.magic         = FLASH_CALIB_MAGIC;
    c.version       = FLASH_CALIB_VERSION;
    c.theta_initial = default_val;
    c.checksum      = calib_checksum(c.magic, c.version, c.theta_initial);

    status          = drv_flash_write_calib(&c);
    g_flash_status  = (int32_t)status;
    g_flash_valid   = (status == status_success) ? 1u : 0u;

    return default_val;
}

hpm_stat_t drv_flash_save_theta(float theta, bool force)
{
    flash_calib_t c;

    if (!force && (drv_flash_read_calib(&c) == status_success)) {
        float diff = c.theta_initial - theta;
        if (diff < 0.0f) {
            diff = -diff;
        }
        if (diff < FLASH_THETA_EPS) {
            g_flash_status = (int32_t)status_success;
            g_flash_theta  = c.theta_initial;
            return status_success;    /* 没变化，不消耗擦写寿命 */
        }
    }

    c.magic         = FLASH_CALIB_MAGIC;
    c.version       = FLASH_CALIB_VERSION;
    c.theta_initial = theta;
    c.checksum      = calib_checksum(c.magic, c.version, c.theta_initial);

    hpm_stat_t status = drv_flash_write_calib(&c);
    g_flash_status    = (int32_t)status;
    if (status == status_success) {
        g_flash_valid = 1u;
        g_flash_theta = theta;
    }
    return status;
}
