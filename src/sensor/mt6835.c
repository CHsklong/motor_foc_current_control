/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "mt6835.h"
#include "drv_spi.h"
#include "hpm_spi_drv.h"

volatile uint8_t  g_seq3_ok = 0;
volatile uint8_t  g_spi_err = 0;
volatile uint8_t  g_byte0 = 0;
volatile uint8_t  g_byte1 = 0;
volatile uint8_t  g_byte2 = 0;
volatile uint8_t  g_user_id = 0;
volatile uint8_t  g_rx_byte = 0;
volatile uint32_t g_spi_fail_cnt = 0;

static uint32_t s_last_angle_raw = 0;   /* 上一帧角度，SPI 偶发失败时用它顶住 */

/**
 * @brief 通过 SPI 读取 MT6835 单字节寄存器
 */
static bool mt6835_read_byte(uint16_t reg_addr, uint8_t *data)
{
    uint8_t tx_buf[3] = {
        (uint8_t)((MT6835_CMD_READ_REG << 4) | ((reg_addr >> 8) & 0x0F)),
        (uint8_t)(reg_addr & 0xFF),
        0x00                       /* dummy 字节 */
    };
    uint8_t rx_buf[3] = {0};

    if (spi_transfer(DRV_SPI_ENCODER_BASE, drv_spi_get_ctrl(), NULL, NULL,
                     tx_buf, 3, rx_buf, 3) != status_success) {
        g_spi_err = 3;
        return false;
    }
    *data = rx_buf[2];
    g_rx_byte = rx_buf[2];
    return true;
}

/**
 * @brief 单次连读：一次 CS 读出 0x003 / 0x004 / 0x005
 *
 * 发完"命令+地址"这 2 字节后继续给时钟，若芯片支持地址自增，会依次吐出三个角度字节，
 * 整帧 5 字节 @8MHz 约 5us。
 *
 * 相比三次独立单字节读：
 *   1) 省掉 2 次 spi_control_init() 的 FIFO/控制器复位轮询 + 2 次 CS 切换；
 *   2) 三个字节来自同一次角度锁存 —— 不会像分次读那样在字节进位边界上
 *      拼出横跨半个量程的假值（例如 MSB 已翻到 0x80 而 MID/LSB 还是旧值）。
 */
static bool mt6835_read_angle_seq3(uint8_t b[3])
{
    uint8_t tx[5] = {
        (uint8_t)((MT6835_CMD_READ_REG << 4) | ((MT6835_REG_ANGLE_MSB >> 8) & 0x0F)),
        (uint8_t)(MT6835_REG_ANGLE_MSB & 0xFF),
        0, 0, 0                    /* 3 个空字节把 0x003/0x004/0x005 时钟出来 */
    };
    uint8_t rx[5] = {0};

    if (spi_transfer(DRV_SPI_ENCODER_BASE, drv_spi_get_ctrl(), NULL, NULL,
                     tx, 5, rx, 5) != status_success) {
        return false;
    }
    /* 全 0 = 芯片没应答；全 0xFF = MISO 仍高阻（地址未自增，后续字节无效） */
    if (((rx[2] | rx[3] | rx[4]) == 0) || ((rx[2] & rx[3] & rx[4]) == 0xFF)) {
        return false;
    }
    b[0] = rx[2];
    b[1] = rx[3];
    b[2] = rx[4];
    return true;
}

/**
 * @brief 上电自检：判断芯片是否支持"普通读命令 + 地址自增"
 *
 * 静止时"单次连读"与"三次单字节读"应当读到同一个角度；若不一致，说明该芯片
 * 不支持地址自增（此时 rx[3]/rx[4] 是垃圾），永久回退，避免把错角度喂给 FOC。
 */
static void mt6835_selfcheck(void)
{
    uint8_t pass = 0;

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t a, m, l, s[3];
        if (!mt6835_read_byte(MT6835_REG_ANGLE_MSB, &a)) break;
        if (!mt6835_read_byte(MT6835_REG_ANGLE_MID, &m)) break;
        if (!mt6835_read_byte(MT6835_REG_ANGLE_LSB, &l)) break;
        if (!mt6835_read_angle_seq3(s)) break;

        /* 无符号差值比较以容忍回绕；允许 MT6835 的 LSB 本底抖动 */
        if (((uint8_t)(s[0] - a) > 2) && ((uint8_t)(a - s[0]) > 2)) break;
        if (((uint8_t)(s[1] - m) > 2) && ((uint8_t)(m - s[1]) > 2)) break;
        /* 0x005 的低 3 位是状态位（含磁场弱告警），只比较高 5 位 */
        if (((uint8_t)((s[2] >> 3) - (l >> 3)) > 1) &&
            ((uint8_t)((l >> 3) - (s[2] >> 3)) > 1)) break;
        pass++;
    }
    g_seq3_ok = (pass == 4) ? 1 : 0;
}

void mt6835_init(void)
{
    mt6835_selfcheck();
    mt6835_read_byte(MT6835_REG_USER_ID, (uint8_t *)&g_user_id);
}

bool mt6835_read_angle_raw(uint32_t *raw)
{
    uint8_t rx[3] = {0};
    bool read_ok = true;

    if (g_seq3_ok) {
        /* 单次连读：一次 CS 拿到同一时刻锁存的 3 个字节（约 5us @8MHz） */
        if (!mt6835_read_angle_seq3(rx)) {
            read_ok = false;
        }
    } else {
        /* 回退：三次独立单字节读，每次一次独立 CS */
        if (!mt6835_read_byte(MT6835_REG_ANGLE_MSB, &rx[0])) {
            read_ok = false;
        } else if (!mt6835_read_byte(MT6835_REG_ANGLE_MID, &rx[1])) {
            read_ok = false;
        } else if (!mt6835_read_byte(MT6835_REG_ANGLE_LSB, &rx[2])) {
            read_ok = false;
        }
    }

    if (!read_ok) {
        /* 偶发 SPI 失败不要立刻上报失败：
           encoder 一旦收到失败会把状态永久置成 fail，之后角度再也不更新、电机锁死。
           这里用上一帧角度顶住，连续失败 5 次才真正上报。 */
        if (++g_spi_fail_cnt > 5) {
            return false;
        }
        *raw = s_last_angle_raw;
    } else {
        g_spi_fail_cnt = 0;
        g_byte0 = rx[0];
        g_byte1 = rx[1];
        g_byte2 = rx[2];
        *raw = ((uint32_t)rx[0] << 13) | ((uint32_t)rx[1] << 5) | (rx[2] >> 3);
        s_last_angle_raw = *raw;
    }
    return true;
}
