# 项目长期记忆（bldc_foc / HPM5300 + E249 板）

## 硬件（来自 E249-v0.0 原理图 + HPM5300 数据手册）
- MCU：HPM5361IEG1（QFN48）。
- CAN 收发器 U5 = SN65HVD234（8 脚）：1=TXD 2=GND 3=VCC 4=RXD 5=VIO/EN 6=CANL 7=CANH 8=S(RS)。
  - RS(pin8) 经 R26=47K 接？：0V=斜率控制（压摆率约 4~6V/µs，500k 边缘）；3.3V=待机（驱动关，发不出帧）。
  - EN(pin5) 低电平 = sleep（收发全关）。
- CAN 走 **MCAN0**：原理图 MCU pin11=MCAN0_RXD(PA01)、pin12=MCAN0_TXD(PA00)；
  数据手册 p18 引脚表：QFN48 的 PA00(pin12)=MCAN0_TXD(ALT7)、PA01(pin11)=MCAN0_RXD(ALT7)。
  → 固件用 PA00/PA01 + MCAN0 正确，不要改成 MCAN3。
- 对外连接器 **JP1（HX2009-2X4Y，2×4）**引脚定义（CAN_H/CAN_L **不相邻**）：
  ```
  1=SK_CAN_L   5=SK_CAN_H
  2=SK_IN1     6=PE
  3=SK_IN2     7=GND      ← 可用作共地
  4=SK_IN3     8=DC12V
  ```
- **板上没有 120Ω 终端电阻**，需由 CANalyst-II 侧跳线提供，或在 CANH-CANL 外挂 120Ω。

## 时钟（E249 移植的大坑，2026-09-20 已修）
- `boards/hpm5300evk/clock.c` 是 Pinmux 工具生成的，原版 `init_board_clock_source()` 只初始化 PLL0(960MHz)；
  但 `init_can0_clock()` 的 CAN 时钟源是 `clk_src_pll1_clk0`（÷10）→ PLL1 没初始化时 `mcan_init()` 失败、`g_can_err=1`、总线无输出。
- **判读铁律：`g_can_tx_cnt` 涨 ≠ 总线有帧**（nonblocking 发送只是入 FIFO 就计数）；`g_can_err=1` 才是 mcan_init 失败的铁证。
- 已修：PLL1 = XTAL 参考 400MHz，clk0 postdiv div_1p0 → CAN0 功能时钟 40MHz（500kbps=80tq）。
- **坑 #2（09-20 确诊）：HPM5361 `MCAN_SOC_MSG_BUF_IN_AHB_RAM=1`，MCAN Message RAM 在 AHB RAM，
  必须在 mcan_init 前调 `mcan_set_msg_buf_attr()` 注册（否则 ram_size=0 → mcan_init 返回 2）。
  can_app.c 已加 2560B `.ahb_sram` 缓冲并注册。诊断变量：g_can_src_clk_khz(应40000)/g_can_init_stat。**

## 约定
- CAN 遥测：0x201（100ms，速度/位置/电流/状态）、0x202（500ms，诊断计数/启动步骤/错误码）。
- CAN_CTRL_ENABLE=1 时自动阶跃测试被跳过，电机上电不转，需发 0x101 才运行（设计行为）。
