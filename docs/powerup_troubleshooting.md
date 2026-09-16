# 冷上电"时转时不转 / 起步过冲"排查手册

> 症状特征：**烧录后必转，断电重新上电时好时坏，偶尔还先过冲一下。**
>
> 这个组合几乎可以确诊为"上电时序"问题，而不是控制参数问题。
> 因为调试器下载是在板子**已经上电数分钟**之后才让程序跑起来的，
> 母线电容、电流采样运放、ADC 基准、磁编码器的上电爬升期天然被跳过了；
> 冷上电则恰好相反。两者的代码一模一样，差别只在"程序开始跑的那一刻，硬件稳没稳"。

---

## 一、优先看这几个探针

J-Scope 加载 ELF 按符号名勾选（不要手抄地址）。上电后如果没转，按这个顺序看：

| 探针 | 正常值 | 异常含义 |
|---|---|---|
| `g_pwm_fault` | 0 | **1 = PWM 硬件过流故障被锁存**，六路输出被硬件钉死为 0。最容易被忽略的一条 |
| `g_fault_src` | 0 | 位图：bit0=模拟量/过流 bit1=环路 bit2=驱动 bit3=编码器 |
| `g_fault_cnt` | 0 | >0 说明 MCL 的 detect 判过故障并关断过输出 |
| `g_mid_u` / `g_mid_v` | ~2048 | 偏离中点太多 = 零点校准取到了脏数据 |
| `g_mid_wait_us` | 小几 ms | 接近上限（10000）说明相电流 ADC 的 PWM 触发链没起来 |
| `g_mid_retry` | 0 | >0 说明零点校准第一次取到的数不合理 |
| `g_seq3_retry` | 0 | 等于 5 = MT6835 连读自检没过，已退回三次单字节读 |
| `g_angle_jump` | 0 | >0 说明拦下过角度撕裂/干扰毛刺 |
| `g_boot_vbus` | ≈24 | 明显偏低说明读母线时电源还没爬起来 |
| `g_exc_cause` | 0xFFFFFFFF | 不是这个值 = 踩过异常（低 5 位：2=非法指令 5=读 fault 7=写 fault） |

---

## 二、已修的四个坑（按危害排序）

### 1. PWM 硬件过流故障在上电瞬间被误锁存 ★头号嫌疑

`pwm_init()` 里使能了外部过流故障源（PA10，低有效），并且：

```c
pwm_pair_config.pwm[x].fault_recovery_trigger = pwm_fault_recovery_on_fault_clear;
fault_config.fault_recover_at_rising_edge     = false;   // 禁止硬件自动恢复
```

即**一旦锁存就必须软件写 FAULTCLR 才能恢复**。而工程里从来没人写过 `pwm_clear_fault()`。

冷上电时 TL331 比较器和电流采样运放还在爬升，输出完全可能是低电平 →
故障在 PWM 启动的瞬间就被锁住 → **六路输出永远是 0**。

这个故障最阴险的地方在于它是"静默"的：CPU 正常、20kHz 中断正常、心跳正常、
`g_pwm_enabled` 也还是 1（那只是我们自己的软件标志，绕不过硬件故障），
看代码看波形都看不出来，只有 PWM 的 `SR.FAULT` 位会说话。

**修复**：`main()` 里 `pwm_init()` 之后先等 `PWM_FAULT_SETTLE_MS`(50ms) 让比较器稳定，
再读故障位 → 记录到 `g_pwm_fault` → 调 `pwm_fault_clear()` 清除 → 然后才 `enable_all_pwm_output()`。
**只在上电阶段清**；运行期间真过流，锁存就是保护，不能碰。

### 2. `detect.callback.user_process` 是空指针 → 一判故障就跳地址 0

`hpm_mcl_detect_loop()` 的实现是：

```c
detect->cfg->callback.disable_output();
detect->cfg->callback.user_process(detect->status);   // 从来没被赋值 → NULL
```

工程里只赋了 `disable_output`，`user_process` 一直没装。于是任何一次故障的
完整后果是"先关断 PWM，紧接着跳转到地址 0" → 非法指令异常 → CPU 原地 trap。
表现为"电机突然不动了"，真正的死因却被埋掉。

**修复**：在 `motor.c` 里装了 `motor0_fault_process()`，把故障源写进 `g_fault_src` 位图并计数。
只记录不自动恢复——过流锁存是保护，擅自恢复可能烧功率级。

### 3. 零点校准取到了上电爬升期的脏数据

原实现是一个裸的 200 次循环，相电流 ADC 由 PWM 触发：

- PWM 没跑起来时 `adc_buff` 全是上电残值，200 次"平均"其实是把同一个脏数加了 200 遍；
- 即便 PWM 在跑（本工程是在 `motor_init()` 里被 `hpm_mcl_drivers_init()` 偷偷启动的），
  采样窗口也正好压在运放/基准的爬升段上。

零点错 = 电流反馈带直流偏置 = 出力不足或起步过冲，而且**只在冷上电时犯病**。

**修复**（`motor_adc_midpoint()`）：
1. 先等 DMA 真的刷新过（`adc_buff` 在 BSS 里初值全 0，非零即说明搬过一帧；
   上限 `ADC_MID_WAIT_CNT_MAX`×`ADC_MID_SAMPLE_US`，耗时记到 `g_mid_wait_us`）；
2. 丢掉前 `ADC_MID_DISCARD_TIMES`(64) 次，避开爬升段；
3. 结果做合理性校验（1/4~3/4 量程），不合理就重来，最多 `ADC_MID_RETRY_MAX`(3) 次；
4. 结果写进 `g_mid_u` / `g_mid_v` 供观察。

另外 `main()` 开头加了 `POWER_UP_SETTLE_MS` 全局稳定延时。

---

## 二·补、启动顺序：为什么"每次上电都要等一下才开始转"

**根因是调用顺序，不是哪个延时设长了。** 原顺序里 `pwm_init()` 排在 `motor_adc_midpoint()` **之后**：

```
… → motor_adc_midpoint() → adc_isr_enable() → timer_init() → pwm_init() → …
```

而相电流 ADC 是 **PWM 比较器触发**的（TRGM 把 PWM 触发路由给 ADC）。PWM 计数器不跑就没有触发，
`adc_buff` 里永远是 BSS 的 0 —— 于是"等首帧"每次都耗满上限。光这一项就白等 100ms。

叠加上其它几项，上电到开始转一共接近 **1 秒**：

| 项目 | 改前 | 改后 |
|---|---|---|
| `POWER_UP_SETTLE_MS` | 500 | **120** |
| 等 ADC 首帧 | 耗满 100（必然） | ~0.1（PWM 已在跑） |
| 丢爬升段 64 次 × 1ms | 64 | 6.4（改 `board_delay_us(100)`） |
| 平均 200 次 × 1ms | 200 | 20 |
| `PWM_FAULT_SETTLE_MS` | 50 | **10** |
| 20ms 冲洗窗口 | 20 | 20 |
| **合计** | **≈ 930ms** | **≈ 180ms** |

改动要点：

1. **`pwm_init()` 提到 `motor_adc_midpoint()` 之前**，紧接着 `disable_all_pwm_output()`。
   `pwm_init()` 里比较器初值是 `PWM_RELOAD+1`（大于重载 → 永不匹配 → 输出关断），
   再 disable 一次双保险，校准期间不会有电流。
2. 零点校准的采样间隔从 `board_delay_ms(1)` 改成 `board_delay_us(ADC_MID_SAMPLE_US=100)`。
   20kHz 触发下 1ms 会漏掉 19 帧 —— 同样的平均次数却要多等 10 倍时间，纯浪费。
3. "等首帧"判据放宽为"非零即可"。原来还要求前后两次值**不同**，
   但电流恒定时 ADC 码值完全可以连续几拍一模一样，判据过严。
4. `POWER_UP_SETTLE_MS` 500→120、`PWM_FAULT_SETTLE_MS` 50→10。
   上电稳定期已经把模拟前端等稳了，后者只需覆盖"故障输入刚使能"的毛刺。

> 剩下那 20ms 冲洗窗口是让测速滤波器的假速度尖峰耗散，别省。
> 另外 S 曲线本身有 ~0.22s 的加速度爬升段（`tj=0.05s, ta=0.217s`），
> 那是规划的柔性起步，不是延迟 —— 想更快就调大 `POS_SCURVE_A_MAX` / `J_MAX`。

### 4. `encoder_abs_rebase()` 与 20kHz 中断竞争 → 起步过冲

rebase 在 main 上下文改 `g_abs_theta` / `g_abs_theta_rdy` / `g_pos_abs` 三个变量，
而 20kHz 中断每拍都在调 `encoder_get_abs_theta()` 改同样三个。

若中断恰好插在"`g_abs_theta` 已清零、`g_abs_theta_rdy` 还没置 false"之间，
它会用上一拍的 `prv` 算出一个 `d`（最大 ±π）并累加进 `g_abs_theta`——
于是刚 rebase 完，位置反馈就带着最多 ±π 的残留，位置环第一拍误差直接跳到 π 量级 →
**起步先猛冲一下**。

**修复**：rebase 用关中断保护（只有三条 store，不做任何 SPI 事务，安全）。

---

## 三、编码器侧的两处加固

### MT6835 连读自检只做一次，冷上电容易误判

`mt6835_seq3_init()` 要求 4 轮"单次连读 vs 三次单字节读"完全一致，
只要一轮不一致就**永久**退回三次单字节读。而冷上电时 MT6835 自身的 POR 可能还没结束。

退回后有两个后果：
- 慢 5 倍（25us vs 5us）；
- 三个字节来自**不同时刻**的角度锁存，在字节进位边界会拼出横跨半个量程的假角度
  → 速度尖峰 → 电流环冲击 → 起步过冲，甚至触发过流保护关断输出。

**修复**：自检改成最多重试 `MT6835_SEQ3_RETRY`(5) 次、每次间隔 20ms，重试次数记到 `g_seq3_retry`。

### 角度撕裂兜底

`encoder_get_abs_theta()` 里加了单拍位移上限 `ENC_MAX_STEP_RAD`(0.35rad ≈ 20°)。
3000rpm 在 20kHz 一拍内只转 0.9°(0.016rad)，这里留了 20 倍余量，正常运动绝不触发，
只有撕裂/干扰毛刺才会撞上。撞上则丢弃本帧、沿用上一次角度，并计入 `g_angle_jump`。

---

## 四、其他

- `read_vbus()` 加了重试（冷上电时 ADC 首帧可能还没落定，单次读取会撞上布局校验失败返回 -1）。
  而 `const_vbus` 是 dq 解耦与电压限幅的输入，喂个 -1 进去比喂标称值危险得多；
  仍失败则兜底为 `MOTOR_VBUS_DEFAULT`(24V)，并把实测值记到 `g_boot_vbus`。
- `app_monitor_update()` 里持续刷新 `g_pwm_fault` 与 `g_vbus`，运行期间也能看。

---

## 五、下次再遇到"上电不转"，30 秒定位流程

1. 看 `g_exc_cause`：不是 `0xFFFFFFFF` → 踩过异常，用 `g_exc_epc` 反汇编定位。
2. 看 `g_pwm_fault`：1 → 过流锁存，说明比较器/采样运放上电时序仍有缝隙，
   加大 `PWM_FAULT_SETTLE_MS` 或用示波器看 PA10 在上电 500ms 内的波形。
3. 看 `g_fault_src` / `g_fault_cnt`：`1`=过流 `2`=环路 `4`=驱动 `8`=编码器。
4. 看 `g_mid_u` / `g_mid_v`：偏离 2048 太多 → 零点还是脏的，加大 `POWER_UP_SETTLE_MS`
   或 `ADC_MID_DISCARD_TIMES`；`g_mid_wait_us` 耗满说明 PWM 触发链没起来。
5. 看 `g_seq3_retry`：等于 5 → 编码器自检没过，查 SPI 与 MT6835 供电。
6. 以上全正常而电机仍不动：才回到控制参数（`g_sc_state` / `g_sc_abort`）。
