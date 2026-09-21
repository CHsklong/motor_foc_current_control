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

## 启动序列铁律（2026-09-20 实锤，09-21 修订）
- **"step16 才使能"这条不够 —— 出力使能必须晚到"命令侧全部就绪"之后（现在的步 21 之前）。**
  步 16 时零点虽然已应用，但 MCL 位置环还没关（`main.c` 关它的那行在步 17 之后）⇒ 只要 main 卡在这段窗口里就会过流，
  详见下面"过流根因铁律"。只有对齐路径（`motor_angle_align` 要吸转子）才在步 13 提前使能。
  违反的症状：上电随机"过流/不转/抖动着转"，力度取决于转子停机位置。
- 判读方法：过流时看 `g_sc_state/g_sc_move_cnt/g_sc_v_ref` —— 全 0 而过流 = 启动路径踢腿，与 S 曲线无关。
- S 曲线 state 枚举：IDLE=0 / RUN=1 / DONE=2；`scurve_move_delta` 仅拒绝 RUN，
  main.c 触发块里 dwell=800 不依赖返回值，查重发失败要看 `g_sc_busy_cnt`。

## 诊断铁律（2026-09-21 第十四轮，最重要的一条）
- **★ 只有 S 曲线模式的运动依赖主循环，其它三种模式都不依赖。**
  电流环 / 速度环 / 位置环（非 S）的给定都在**上电时一次性下发**（`main.c:159-227`），之后完全由 20kHz 中断闭环执行；
  只有 S 曲线的 `scurve_move_delta()` 写在 `while(1)` 里（`main.c:288`）。
  ⇒ **「S 曲线不转、电流/速度/位置环都正常」不是三条证据，而是「主循环没在跑」的同义句。**
  用户一旦这么描述，直接去查 main 有没有在跑，不要再去查 S 曲线算法。
- **★ 让"整个系统静默停住"只有两条路径，先分清是哪条**：
  ① 异常停机 —— `debug/dbg_probe.c:16-24` 的 `exception_handler`（**09-21 10:21 实测 `g_exc_cause≡0xFFFFFFFF`、`g_exc_epc≡0` 全程不变 ⇒ 本案例中它从未发生**）；
  ② **CPU 被 20kHz 中断吃满** —— 这是本案真正走的路径，根因见文末「★ 第十七轮定案」。
  判据：`g_isr_us_win_max` 远超 50µs 且 `g_isr_win_over` 接近 100。
  路径①的机理如下：
- **★ `exception_handler` 的后果 = `debug/dbg_probe.c:16-24`**：
  `disable_global_irq(MIE)` + `for(;;){}`。关掉全局中断后连 20kHz ADC ISR 也停
  ⇒ **占空比冻结在最后一拍 → 定子上一直压着静态电压矢量 → 直流电流 → 约 0.5s 顶到 i_max=3A → 过流**。
  **"烧录后过一会就过流"与"点 Debug 暂停就过流"是同一种物理后果的两种触发方式。**
  ⇒ 看到"停住 + 过一会过流"，第一件事就是读 `g_exc_cause`/`g_exc_epc`，而不是继续读控制变量。
- **三变量判据（地址随每次重编刷新，用 nm 核对）**：
  `g_exc_cause`≠0xFFFFFFFF ⇒ 踩异常（`g_exc_epc` 即出错指令，objdump 定位）；
  `g_heart_isr` 与 `g_heart_main` **一起冻结** ⇒ 异常停机 / 调试器 halt；**只有 main 冻结而 isr 在涨** ⇒ 被 ISR 饿死（看 `g_isr_win_over`）。
  再次强调：**mepc/mcause 不要去 SES 的 Register 面板找**（那里是外设 SFR 组），本工程已把它们做成全局变量。
- **"代码里没法卡住"的证明方法（可复用）**：把 `g_main_step` 卡住点前后两条打点之间的语句逐个列出，
  确认①无循环、②无函数指针、③无外设握手等待、④所有被调函数是纯读写/纯算术。
  全部成立 ⇒ **不要再在这段代码里找 bug**，问题在"CPU 整体没在推进"（异常停机 / 调试器 halt / 被中断饿死）这一层。
  第十四轮就是靠这条从"卡在 main loop"跳到"CPU 停止推进"的。

## ★ 第十七轮定案（2026-09-21 10:41）：整份固件按 -O0 编译 —— "电机不转"的真正根因
- **证据链（三条，互相独立）**：
  1. `riscv32-unknown-elf-readelf --debug-dump=info demo.elf | grep DW_AT_producer`
     → `GNU C17 14.3.0 -fmessage-length=0 -march=... -mabi=ilp32f ...`，**命令行里没有任何 `-O` 选项 ⇒ GCC 默认 -O0**。
  2. `bldc_foc.emProject` 末尾 `<configuration Name="Debug" ... gcc_optimization_level="None" />`（SES 的"无优化"）；
     产物在 `Output/Debug/Exe/demo.elf` ⇒ 实际编译的就是这个配置。
  3. 机制（`cmake/ide/segger.cmake:769-773`）：生成 SES 工程时用正则 `(-O[0-3s])` 从 `target_gcc_cflags` 里**提取** -O
     写成 `gcc_opt_level`；`scripts/ide/segger/embedded_studio_proj_gen.py` 的 `get_gcc_opt_level()` 把**空串**映射成 `"None"`
     ⇒ 工程里一个 `-O` 都没有时，Debug 配置就静默落到 `-O0`。**这是 SDK 的静默陷阱：默认不设任何优化。**
- **实测数据（J-Scope 10:41，地址已用 nm 逐行核对）**：
  | 变量 | 读数 | 判读 |
  |---|---|---|
  | `g_enc_us_max` | 117 / 117 / 117（恒） | 编码器段 117µs（8MHz×5 字节 SPI 本身只需 5µs） |
  | `g_loop_us_win_max` | 139 / 148 / 143.8 | 控制环段 ~144µs |
  | `g_isr_us_win_max` | 221 / 239 / 231 | **单拍 230µs = 50µs 预算的 4.6 倍** |
  | `g_isr_win_over` | 100 / 100 / 100 | **每一拍都超预算，CPU 100% 在中断里** |
- **因果闭合**：ISR 超预算 4.6 倍 → CPU 全在中断 → main 零进度（`g_heart_main`=1、`g_boot_step`=17、`g_main_step`=1）
  → 只在 `while(1)` 里的 S 曲线往复触发永不执行 → 电机不转。
  电流/速度/位置环不依赖 main（给定上电一次性下发、之后全在中断闭环）⇒「那三个正常、S 曲线不转」是同一条事实的两种说法。
  用户"不开 J-Scope 重烧后仍不转"⇒ **排除调试器 halt 假象，冻死是真实发生的**。
- **修复（2 处，均已落地）**：
  1. `CMakeLists.txt` → `sdk_compile_options("-O2")`（SDK 自己的写法，见 `samples/lwip/mhd_wifi_demo`）；
     `segger.cmake` 会把它提取成 `gcc_opt_level=-O2` → SES 的 `Level 2 balanced`。**CMake 重生成工程也不会丢。**
  2. `bldc_foc.emProject` 的 Debug 配置 `gcc_optimization_level="None"` → `"Level 2 balanced"`（不改 CMake 也能立刻重编生效）。
     注意：CMake 重生成 .emProject 会覆盖这一行，所以 1 才是持久的。
- **量化**（用 SES 工具链把 ISR 调用链上的文件分别编 -O0/-O2，比 `.text`）：
  `hpm_mcl_loop` 8640→5976(1.45x)、`hpm_mcl_control` 7074→4398(1.61x)、`hpm_mcl_encoder` 3958→2682(1.48x)、
  `hpm_spi_drv` 3792→2312(1.64x)；合计 23716→15502 = **1.53x**（纯体积比，运行期收益通常更大）。
- **-O2 安全性前提（已逐项核实）**：`debug/dbg_probe.h` 全部探针都是 `volatile`（82 处）；
  `g_sys_ms`(`drv_timer.h:26`)、`g_encoder_isr_enable`(`sens_encoder.c:10`)、`adc_buff`(`drv_adc.c`) 均 volatile
  ⇒ 不影响 J-Scope 观测，也不会破坏中断共享语义。SDK 自身样例就用 `-O2`，无样例需要 `-fno-strict-aliasing`。
- **★ 修复已实测生效（09-21 10:57 抓拍，用户确认"修复了"）**：
  | 变量 | 修复前 | 修复后 |
  |---|---|---|
  | `g_isr_win_over` | 100 | **0** |
  | `g_isr_us_win_max` | 231µs | **31µs**（预算 50µs，占用 62%） |
  | `g_enc_us_win_max` | 117µs | **8µs** |
  | `g_loop_us_win_max` | 144µs | **22µs** |
  ⇒ 中断不再吃满 CPU，main 正常推进。**"-O0 是根因"这条彻底闭环。**
  同时说明 20kHz **不需要**降到 10kHz：-O2 后 31µs 还有将近一倍余量。
  **判据固化：以后凡"电机不转但中断心跳正常"，先量 `g_isr_win_over` 与 `g_isr_us_win_max`（预算 50µs）。**
- **顺带记下（本轮未改）**：`src/driver/drv_spi.c` 的 `spi1_config()` **忽略了 `spi_master_timing_init()` 的返回值**。
  该函数在 `clk_src/8MHz` 得不到"整数、偶数且 ≤510"的分频时返回 `status_invalid_argument` 且**根本不写 TIMING 寄存器**
  （`hpm_spi_drv.c:353-374`）⇒ SPI 会跑在复位默认分频上。若 -O2 后编码器段仍异常长，先查这一条。

## 约定
- CAN 遥测：0x201（100ms，速度/位置/电流/状态）、0x202（500ms，诊断计数/启动步骤/错误码）。
- CAN_CTRL_ENABLE=1 时自动阶跃测试被跳过，电机上电不转，需发 0x101 才运行（设计行为）。
- **"关 CAN"= 改 app_cfg.h 的 CAN_CTRL_ENABLE=0 并重编**；只拔 CAN 盒不改宏 = 测试块被编译剔除 + 无命令 → 电机永不转（09-21 实锤，g_sc_dwell≡0 是判据）。
- **S 曲线测试的运动方式 = `app_cfg.h` 的 `SCURVE_REPEAT_ENABLE`（09-21 第十七轮新增，默认 0）**：
  1=往复（走完 D → 停 800ms → 取反方向再走，原行为）；0=**单向单次**（只走一次 D，完成后位置环保持终点）。
  实现要点：`main.c` 触发块用 `sc_armed` 闸门——`scurve_move_delta()` 成功后单次模式置 `sc_armed=0`。
  **不要**用"把 `sc_dir` 置 0 让它自然退出"的写法：`scurve_move_delta(0)` 会走 `D < SCURVE_MIN_DIST` 分支并 `move_cnt++`，
  使 `g_sc_move_cnt` 每 800ms 空涨一次，破坏"只走了一次"的判据。
  `sc_dwell` 初值 = `SCURVE_DWELL_MS`（起始 800ms 等待，留给 J-Scope 起采样）。
  单次模式判据：`g_sc_move_cnt≡1`、`g_sc_state` 终值 = DONE(2)、`g_sc_dwell` 归 0 不再回升。
- **单次 S 曲线理论值（vmax=25 / amax=150 / jmax=4000，D=12π=37.70rad≈6 圈，形状①七段）**：
  `tj=37.5ms`、`ta=204.2ms`、`tv=1303.8ms`、**`t_all=1712ms`**；峰值 25 rad/s(238.7rpm)、峰值加速度 150 rad/s²。
  DONE 之后：`ctrl_foc.c:74` 取 `g_pos_ref=g_sc.p`，修正器权限仅 ±`POS_CORR_MAX`=6 rad/s 叠加在 `g_sc.v` 上 ⇒ 自然稳住终点。
  （`POS_TARGET_DELTA` 注释写"1 圈(2π)"、实际是 12π —— 这条不符**至今未改**，6 圈=1.71s，1 圈则 t_all≈456ms。）
- **判读铁律（09-21 补）**：`g_main_step≡0` + `g_boot_step` 停在 17（而非 21）= main 死在 main.c:188→203（进 while(1) 之前），
  即中断侧死循环/halt，属"零进度"而非"被 20kHz 中断饿死"（被饿死会让 g_main_step 非 0，且应急让路能救回）。
  健康值：main 活着 = `g_boot_step`=21，`g_main_step` 循环 1/2/22/3/4/5。`g_exc_cause`=0xFFFFFFFF 表示没有异常。
- **J-Scope 符号表核对法**：用 rv32 objdump/自解 symtab 对比截图里的 Address 列，地址逐行一致才认读数（第九轮踩过陈旧地址的坑）。
- **★ 已作废的推理：`g_sc_tick/g_sys_ms ≈ 16.7`**（09-21 第十六轮更正）。
  `g_sc_tick` **不是** ISR 拍镜像 —— 它由 `ctrl_foc.c:168` 在 `position_loop_post()` 里**每 5 拍才写一次**。
  ⇒ 用这个比值推"20kHz 只跑到 16.7kHz / `g_sys_ms` 虚高 20%"**整条作废**；据此改的 `drv_timer.c` 已还原（还原是对的）。
  判 ISR 真实拍率只能看 `g_heart_isr` 对真实秒数（J-Scope X 轴）。
  （`BOARD_BLDC_TMR_RELOAD` 与 `clock_gptmr2` 实际频率是否差 20% **至今未实测确认**，不要再当结论用。）
- **★ 第十六轮新增（09-21 10:21，判据出结果）**：`g_exc_cause≡0xFFFFFFFF` + `g_exc_epc≡0`（J-Scope 曲线用像素解析确认全程水平）、
  `g_heart_main≡1`（全程水平）、`g_heart_isr` 单调斜线 22319→152827
  ⇒ **异常假设证伪；中断健康；main 拿不到 CPU**。
  卡点 = `main.c:235`→`:240` 之间的 `dbg_probe_keep()`/`app_monitor_update()` —— 两者纯读取纯算术，**不要去这两处找 bug**。
- **★ 让"中断一重，main 就看起来冻死"的放大器**：主循环尾部原是 `board_delay_ms(1)` = **48 万拍忙等**
  ⇒ main 每圈必须**独占累积 480000 个 CPU 拍**。中断留 1% CPU ⇒ 一圈 100ms；留 0.2% ⇒ 一圈 500ms ⇒ 抓拍里完全看不出动。
  **已改**：换成"等 `g_sys_ms` 前进一格"+100 万拍兜底（语义不变，1 圈≈1ms）。
  ⚠ 但这只是**放大器、不是根因**：中断真正超预算时（第十七轮实测 230µs / 预算 50µs），
  main 连一条指令都拿不到，换掉忙等也不解决。两层要同时看。
  （`g_sys_ms` 是 `volatile`，见 `drv_timer.h:26`，所以这个等法在 -O2 下不会被优化成死循环。）
- **`g_heart_isr` 单调上升只证明中断在跑，不证明中断没吃满 CPU**；
  "应急让路"（`ISR_RELIEF_ENABLE=1`）只跳过 `position_loop_pre/hpm_mcl_loop/position_loop_post`，
  **`scurve_isr_tick`、编码器 SPI 读、ADC 取样、窗口统计仍逐拍执行** ⇒ 它救不回 main。
- **★ 更正（第十七轮）：不能靠"小函数有没有栈帧"否定 -O0。**
  当时看 `scurve_is_idle` 等小函数无栈帧 + 用 `gp` 相对寻址，就反推"代码是优化过的、-O0 假设不成立"——**这个反推是错的**。
  事实相反：`.emProject` 的 Debug 配置是 `gcc_optimization_level="None"`，DWARF producer 里也确实一个 `-O` 都没有；
  同一份 ELF 里链进了 `__adddf3/__muldf3` 这类软双精度库函数，正是 -O0 的旁证。
  **判优化等级只认两处：① DWARF 的 `DW_AT_producer` 里有没有 `-O*`；② `.emProject` 的 `gcc_optimization_level`。**
- **已排除的嫌疑（第十六轮逐个查过）**：CAN（`CAN_CTRL_ENABLE=0` 时 can_app.c 整个文件被 `#if` 包成空实现）；
  FLASH/XIP 退化（`FLASH_PARAM_CALIB_ONCE=0` 时 `drv_flash_load_theta` 只做 XIP 指针读，`drv_flash_init()` 只在真正擦写时调）；
  L1 缓存（`segger/startup.s:197-208` 默认 `l1c_ic_enable`/`l1c_dc_enable`）；SPI（8MHz，单次连读约 5µs）；`board_delay_ms` 实现正常。
- **判读铁律（09-21 第十二轮补）**：若 `g_boot_step` 停在 17 且 `g_heart_main≡0`，而 17→18 之间**机器码里没有任何可阻塞语句**（objdump 可证：只有 3 次存储 + 2 个纯数学 jal），则不要再怀疑那段代码本身，转去查：
  ① **板上跑的是不是当前这份镜像**（SES Target 日志的 Download 进度有没有走到完成；截图已出现停在 "0.0 KB of 70.9 KB ETA" 的情况）；
  ② 控制流被异常/坏返回地址劫走 —— 读 SES Registers 里的 **`mepc`（=main 真正停住的地址）与 `mcause`**，这是最快的一步。
- **不要为了绕过症状新增第二条控制通道（09-21 反例）**：曾加过 `SCURVE_AUTOSTART_ISR`（main 停摆时由 20kHz 中断接管 S 曲线往复触发），
  被用户否决并全部撤回 —— 它没解决根因，只让电机真的动起来，把下面那条"位置环饱和"路径暴露成实机过流。
  用户原话："你只解决问题，不要去加这种没有意义的东西"。**先定位为什么不动，不要另开一条路让它动。**
- **过流根因铁律（09-21 实锤）：步 16 使能出力时，MCL 自带位置环还没关。**
  `motor.c` 里 `enable_position_loop = true`（位置模式默认），关它的那行在 `main.c` 步 17 之后；
  而 `motor.h` 明写：`enable_position_loop=true` 但 `ref_position.enable=false` 时**位置环照样在跑**，
  err 是巨大负值 → **输出饱和 → 积分顶死**。
  → 于是"出力使能"与"关位置环"之间有一个窗口：只要 main 在这个窗口里被卡住（**实测就是停在 `g_boot_step`=17**），
    饱和力矩被永久保持 → 速度环积分 ~0.5s 顶到 i_max=3A → 稳恒 3A → **过流**。
    症状"过一会就过流 / 有时一 Debug 就过流"的随机性 = main 是否卡在这个窗口。
  **已修**：`hpm_mcl_loop_enable()` 从步 16 下移到步 21 之前（命令侧全部就绪后）。
  效果：main 卡住时环路是关的 → 占空比停在 pwm_init 初值（输出全关）→ 只不转，不过流。
- **调试暂停会过流，而且现在没有保护**：`drv_pwm.c` 的 `pwm_config_fault_source()` 目前只使能 `pwm_fault_source_external_0`，
  **09-20 加过的 `pwm_fault_source_debug` 已不在代码里**（git 里也从未提交过）。
  缺了它，任何 halt/暂停都会把"最后一拍占空比"当静态电压矢量一直压在电机上 → 定子直流过流。
  加回来的代价：每次 halt/断点都锁存一次 PWM 故障，要靠 app_keeper 清（`KEEPER_MAX_ACTION=8` 会被消耗完）。
