/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "hw_map.h"
#include "drv_spi.h"
#include "dbg_probe.h"
#include "mt6835.h"

uint32_t g_last_angle_raw = 0;   /* 上一帧角度，SPI 偶发失败时用它顶住 */

hpm_stat_t mt6835_read_byte(uint16_t reg_addr, uint8_t *data)
{
    uint8_t tx_buf[3] =
    {
        (uint8_t)((MT6835_CMD_READ_REG << 4) | ((reg_addr >> 8) & 0x0F)),
        (uint8_t)(reg_addr & 0xFF),
        0x00   /* dummy 字节 */
    };
    uint8_t rx_buf[3] = {0};

    if (spi_transfer(HPM_SPI1, &g_spi_ctrl_cfg, NULL, NULL, tx_buf, 3, rx_buf, 3) != status_success) {
        g_spi_err = 3;
        return status_fail;
    }
    *data = rx_buf[2];
    g_rx_byte = rx_buf[2];
    return status_success;
}

/* ============ MT6835 单次连读：一次 CS 读出 0x003 / 0x004 / 0x005 ============
 * 发完"命令+地址"这 2 字节后继续给时钟，若芯片支持地址自增，会依次吐出
 * 0x003/0x004/0x005 三个角度字节，整帧 5 字节 @8MHz ≈ 5us 传输 + 一次固定开销。
 *
 * 相比三次独立单字节读：
 *   1) 省掉 2 次 spi_control_init() 的 FIFO/控制器复位轮询 + 2 次 CS 切换；
 *   2) 三个字节来自同一次角度锁存 —— 不会像分次读那样在字节进位边界上
 *      拼出横跨半个量程的假值（例如 MSB 已翻到 0x80 而 MID/LSB 还是旧值）。
 * 上电自检不通过会自动回退到三次单字节读，见 mt6835_seq3_init()。 */
hpm_stat_t mt6835_read_angle_seq3(uint8_t b[3])
{
    uint8_t tx[5] = {
        (uint8_t)((MT6835_CMD_READ_REG << 4) | ((MT6835_REG_ANGLE_MSB >> 8) & 0x0F)),
        (uint8_t)(MT6835_REG_ANGLE_MSB & 0xFF),
        0, 0, 0                        /* 3 个空字节把 0x003/0x004/0x005 时钟出来 */
    };
    uint8_t rx[5] = {0};

    if (spi_transfer(HPM_SPI1, &g_spi_ctrl_cfg, NULL, NULL, tx, 5, rx, 5) != status_success) {
        return status_fail;
    }
    /* 全 0 = 芯片没应答；全 0xFF = MISO 仍高阻（地址未自增，后续字节无效） */
    if (((rx[2] | rx[3] | rx[4]) == 0) || ((rx[2] & rx[3] & rx[4]) == 0xFF)) {
        return status_fail;
    }
    b[0] = rx[2];
    b[1] = rx[3];
    b[2] = rx[4];
    return status_success;
}

/* 上电自检：必须在电机出力之前、轴静止时调用。
 * 静止时"单次连读"与"三次单字节读"应当读到同一个角度；若不一致，说明该芯片
 * 不支持普通读命令下的地址自增（此时 rx[3]/rx[4] 是垃圾），永久回退，
 * 避免在 ADC 中断里把错角度喂给 FOC。 */
void mt6835_seq3_init(void)
{
    for (uint32_t retry = 0U; retry < MT6835_SEQ3_RETRY; retry++)
    {
        uint8_t pass = 0;

        for (uint8_t i = 0; i < 4; i++) {
            uint8_t a, m, l, s[3];
            if (mt6835_read_byte(MT6835_REG_ANGLE_MSB, &a) != status_success) break;
            if (mt6835_read_byte(MT6835_REG_ANGLE_MID, &m) != status_success) break;
            if (mt6835_read_byte(MT6835_REG_ANGLE_LSB, &l) != status_success) break;
            if (mt6835_read_angle_seq3(s) != status_success) break;

            /* 无符号差值比较以容忍回绕；允许 MT6835 的 LSB 本底抖动 */
            if (((uint8_t)(s[0] - a) > 2) && ((uint8_t)(a - s[0]) > 2)) break;
            if (((uint8_t)(s[1] - m) > 2) && ((uint8_t)(m - s[1]) > 2)) break;
            /* 0x005 的低 3 位是状态位（含磁场弱告警），只比较高 5 位 */
            if (((uint8_t)((s[2] >> 3) - (l >> 3)) > 1) &&
                ((uint8_t)((l >> 3) - (s[2] >> 3)) > 1)) break;
            pass++;
        }

        if (pass == 4) {
            g_seq3_ok    = 1;
            g_seq3_retry = retry;
            return;
        }

        /* 芯片还没从上电复位里出来（或轴被碰了一下），等一会儿再试 */
        board_delay_ms(MT6835_SEQ3_RETRY_MS);
    }

    g_seq3_ok    = 0;
    g_seq3_retry = MT6835_SEQ3_RETRY;   /* 等于上限 = 自检没过 */
}
