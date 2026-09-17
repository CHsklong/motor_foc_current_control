# 位置环 S 曲线规划

> 参考：CSDN《PMSM FOC位置环S曲线控制算法(恒定急动度)》
> https://blog.csdn.net/tilblackout/article/details/124439572
>
> 本文记录该原理在本工程的落地实现，以及相比原文做的三处关键改动和原因。

---

## 1. 原理：为什么是"恒定急动度"

### 1.1 梯形速度的问题

最朴素的做法是恒定加速度：先以 a 加速，匀速，再以 −a 减速。
但这样**加速度在指令上是阶跃的**——起步瞬间从 0 跳到 a，物理上不可实现，
对机械结构也是冲击。

### 1.2 对加速度再微分

对加速度求导就得到**急动度（Jerk/Jolt）**：

```
J = da/dt
```

让 J 恒定，加速度就走**梯形波**；对加速度积分得到速度，速度就是 S 形；
再积分得到位置，就是平滑的 S 曲线。三层关系：

```
加速度 a  ──── 梯形 ────   ┌───┐              ┌───┐
                          ╱     ╲            ╱     ╲
速度   ω  ──── S 形 ──── ╱       ╲──────────╱       ╲
位置   θ  ──── 平滑 ────╱                            ╲──
          加速3段    匀速    减速3段
```

### 1.3 恒定 jerk 段内的闭式解

段内 j 恒定，该段就是时间的三次多项式（τ 为段内局部时间）：

```
a(τ) = a₀ + j·τ
ω(τ) = ω₀ + a₀·τ + ½·j·τ²
θ(τ) = θ₀ + ω₀·τ + ½·a₀·τ² + (1/6)·j·τ³
```

j = 0 时自动退化为匀加速公式，所以七段可以共用这一个函数。

### 1.4 对称梯形加速度的两个不变量

用这两个关系可以绕开冗长的分段积分：

```
v_peak = a_peak · (ta − tj)      加速段末速度
d_acc  = v_peak · ta / 2         加速段位移（平均速度 v/2 × 时间 ta）
```

代入原文的特例 tj = A、ta = 3A：
`v_peak = j·A·(3A−A) = 2JA²`、`d_acc = 2JA²·3A/2 = 3JA³` —— 与原文推导完全一致。

---

## 2. 相比原文的三处改动

### 改动一：给定时长 → 给定 v/a/j 约束

原文是"总时间均分 9 段"，J 由 `J = Δθ/(12A³)` 反推。
问题：**时长是拍脑袋给的，速度/加速度是否超限完全不可控**。
比如 39.27 rad 用 1s 走完，算出来巡航速度 58.9 rad/s，直接撞上 output_max=30，
S 曲线被限幅切成梯形，白做了。

本工程改成给定 `v_max / a_max / j_max`，由规划器反解七段时长，
条件不足时自动退化：

| 情形 | 段数 | 说明 |
|---|---|---|
| 位移够长 | 7 段 | 加速3 + 匀速 + 减速3 |
| 到不了 v_max | 5 段 | 无匀速，v_peak 由位移反解 |
| 到不了 a_max | 4 段 | 三角形加速度，无匀加速段 |

无匀速段时 v_peak 由 `v² + v·(a·tj) − D·a = 0` 解出；
连匀加速段都没有时 `tj = (D/(2j))^(1/3)`。

### 改动二：逐拍积分 → 闭式求值

原文代码是：

```c
Acceleration += jerkApplied * SamplingTime;
Omega        += Acceleration * SamplingTime;
Theta        += Omega * SamplingTime;
```

这三行每拍累积浮点误差，多圈后终点偏差肉眼可见（原文"待解决"一节也提到了）。

本实现只在规划时算好 7 个段起点的状态 `(a, ω, θ)`，运行时用 t 直接闭式求值：

```c
tau = t - tk[i];
seg_advance(jk[i], tau, &a, &w, &th);   /* 三次多项式，一步到位 */
```

而且 `t = tick × ts`，tick 是**整数**，时间本身也不累积误差。
实测终点误差 0（见第 5 节验证结果）。

### 改动三：加了速度前馈（原文没有，但级联 P 环必须有）

这是最关键的一处。原文的位置环是"位置环 PID 直接输出转矩"，
而本工程是**位置环 → 速度给定 → 速度环 → 电流**的级联结构。

如果只把位置给定改成 S 曲线的 θ*(t)，让位置环 P 去追一个移动的目标：

```
e_ss ≈ v_peak / kp = 25 / 20 = 1.25 rad
```

电机始终滞后 1.25 rad，S 曲线结束时还欠着这么远，
剩下的路只能靠积分慢慢补——轨迹完全被破坏。

**所以必须把规划速度 ω*(t) 作为前馈直接加到位置环输出上。**

---

## 3. 实现

### 3.1 文件

| 文件 | 说明 |
|---|---|
| `src/control/planner_scurve.h/.c` | **新增**。规划器，纯算法，不依赖 SDK 业务层 |
| `src/control/ctrl_foc.c` | 集成：分频推进、写给定、限幅、前馈包装 |
| `src/control/ctrl_foc.h` | 新增 `ctrl_pos_init()` / `ctrl_pos_start()` |
| `src/motor/motor_params.h` | 新增 6 个参数 |
| `src/debug/dbg_probe.h/.c` | 新增 4 个探针 |
| `src/app/main.c` | 用 `ctrl_pos_start()` 取代原来的固定给定 |
| `CMakeLists.txt` | 加 `planner_scurve.c` |

### 3.2 API

```c
void scurve_init (scurve_t *s, float ts);
void scurve_start(scurve_t *s, float th0, float delta, float vmax, float amax, float jmax);
void scurve_step (scurve_t *s);    /* 推进一拍，更新 s->th / s->w / s->a */
void scurve_abort(scurve_t *s);
```

对外的业务接口只有两个：

```c
void ctrl_pos_init (void);          /* 必须在 adc_isr_enable() 之前调用 */
void ctrl_pos_start(float delta);   /* 启动一段相对位移 */
```

### 3.3 前馈的注入点（关键设计）

MCL 里的执行顺序是：

```c
position_pid(ref_position, theta_abs, cfg, &exec_ref.speed);   /* 位置环 */
/* 同一拍，紧接着 */
speed_pid(exec_ref.speed, sens_speed, cfg, &exec_ref.iq);      /* 速度环 */
```

**速度环在同一拍就消费了 exec_ref.speed。** 所以：

- 在 `position_loop_post()`（即 `hpm_mcl_loop()` 之后）加前馈 → 已经晚了，
  下一拍位置环又会把 `exec_ref.speed` 覆盖掉，前馈永远不生效。
- 正确做法是**替换 MCL 的位置环 PID 函数指针**，在包装里加前馈：

```c
static hpm_mcl_stat_t position_pid_with_ff(setpoint, feedback, pid_x, output)
{
    s_pos_pid_orig(setpoint, feedback, pid_x, output);   /* 原 MCL 实现 */
    v = *output + s_omega_ff * POS_SCURVE_FF_GAIN;
    *output = clamp(v, ±POSITION_OUTPUT_MAX);
}
```

注册在 `ctrl_pos_init()` 里做，只包一层，重复调用不会套娃。

### 3.4 分频同步

规划器必须按位置环周期（4kHz）推进，而 `position_loop_pre()` 在 20kHz 中断里每拍都跑。
用整数计数分频，避免与 MCL 内部的 `position_ts` 浮点累加错拍：

```c
if (++s_pos_cnt >= s_pos_div) { s_pos_cnt = 0; scurve_step(&s_sc); }
```

`s_pos_div` 由 `PWM_FREQUENCY × (*motor0.loop.const_time.position_ts)` 运行时算出（= 5），
不写死 4kHz/20kHz。

### 3.5 轨迹期与到位期的限幅切换

| 阶段 | 限幅 | 理由 |
|---|---|---|
| `SCURVE_RUN` | `clamp(output_max − \|ω_ff\|, POS_TRACK_VLIM_MIN, output_max)` | 前馈已承担主要运动，PID 用"剩下的额度"补跟随误差；总和由前馈包装统一限到 ±output_max |
| `SCURVE_DONE` | 原来的 `sqrt(2·POS_DECEL_MAX·aerr)` + 死区 | 收拾残差，死区内泄放积分 |

> **【2026-09-15 修正】** `SCURVE_RUN` 这一格原先是 `min(output_max − |ω_ff|, POS_TRACK_VLIM)`，
> 即"预算充足时也只有 8 rad/s"。巡航段无所谓（预算本来就只剩 5），但**起步段是致命的**：
> 此时前馈 ≈ 0，预算本来是满的 30 rad/s，被这一刀砍到 8；一旦静摩擦偏大、前馈还没爬起来，
> 位置环就没有任何裕度把电机拽动 —— 这就是
> "`POS_SCURVE_ENABLE=1` 完全不转、`=0`（固定目标一上来就饱和到 30）却转得好好的"的直接原因。
> 现在只保一个下限 `POS_TRACK_VLIM_MIN`（`POS_TRACK_VLIM` 这个固定上限已删除）。

另外 `position_loop_post()` 的抗饱和判定**必须用 PID 自己的输出**（前馈包装里留的
`s_pos_pid_out`），不能用加完前馈的 `exec_ref.speed`：后者 ≈ 前馈量（可达 25 rad/s），
而 `s_pos_vlim` 只是留给 PID 的预算（巡航时 5），拿它去比会每拍都误判饱和，
积分被永久冻结在 0，位置环退化成纯 P。

---

## 4. 参数（`motor_params.h`）

```c
#define POS_SCURVE_ENABLE  (1)       /* 0=回退到改动前的行为 */
#define POS_SCURVE_V_MAX   (25.0f)   /* 巡航速度 rad/s */
#define POS_SCURVE_A_MAX   (150.0f)  /* 最大加速度 rad/s² */
#define POS_SCURVE_J_MAX   (3000.0f) /* 最大急动度 rad/s³ */
#define POS_SCURVE_FF_GAIN (1.0f)    /* 前馈增益 */
#define POS_TRACK_VLIM_MIN (4.0f)    /* 轨迹期 PID 限幅下限 */
#define POS_SCURVE_ABORT_ERR (3.0f)  /* 失速保护阈值 rad */
```

### 4.0 J-Scope 观测组合

看轨迹跟不跟得上，把这四个放到同一张图即可（全部 20kHz ISR 刷新）：

| 通道 | 规划侧 | 实际侧 |
|---|---|---|
| 位置 | `g_sc_theta`（规划位置，本拍移动的目标点） | `g_pos_abs`（实际位置） |
| 速度 | `g_sc_omega`（规划速度 = 速度前馈量） | `g_spd_fdb`（实际机械角速度） |

配套：`g_pos_err`（跟随误差，轨迹期即 `g_sc_theta − g_pos_abs`）、
`g_ref_speed`（位置环输出 = PID + 前馈）、`g_sc_state`（0=IDLE 1=RUN 2=DONE）、
`g_sc_abort`（失速保护触发次数，>0 说明电机没跟上轨迹）。

### 4.1 失速保护

轨迹在跑但跟随误差超过 `POS_SCURVE_ABORT_ERR`（默认 3 rad，正常跟踪只有 0.0x）时，
判定电机没跟上，直接 `scurve_abort()` 退回"固定目标 + 接近限速"，
保证一定能走到终点。触发次数记在探针 `g_sc_abort`。

当前工况（39.27 rad = 6.25 圈）的规划结果：

```
tj = 0.050s   ta = 0.217s   tv = 1.354s   总时长 1.787s
v_peak = 25 rad/s (239 rpm)   a_peak = 150 rad/s²
```

约束余量核算：`a_max=150` 对应转矩 `J·a = 6.2e-6 × 150 = 9.3e-4 N·m`，
按 Kt=0.054 折算只需 0.017 A，电流环远未饱和。

---

## 5. 验证

### 5.1 规划器数学（`tools/test_scurve.py`）

7 个用例全部通过：终点误差 0、终点 ω/a 为 0、不超速不超加速度、
位置单调、速度无跳变。覆盖了 7 段 / 5 段 / 4 段三种退化与正负方向。

```
python tools/test_scurve.py
```

### 5.2 闭环仿真（`tools/sim_scurve.py`）

电机模型：J=6.2e-6、Kt=0.054、库仑摩擦 0.004 N·m、
速度环 4kHz（kp=0.02 / ki=0.00008）、位置环 4kHz（kp=20 / ki=0.025）、
速度反馈 5ms 时延。

| 方案 | 运动段最大跟随误差 | 到位时间 | 残差 | 末段抖动峰峰 |
|---|---|---|---|---|
| A 改进前（固定目标+接近限速） | 不适用 | 1.421 s | +0.556° | 0 |
| B S 曲线**无前馈** | **26.12 rad (1497°)** | 2.747 s | +0.573° | **4.22 rad (242°)** |
| C S 曲线 + 速度前馈 | **0.065 rad (3.7°)** | 1.798 s | +0.538° | 0 |

**B 这一行就是"为什么必须前馈"的证明**：不加前馈时规划器 1.79s 就走完了，
但电机还差 26 rad 没跟上，只能靠积分去追，末段出现 242° 的大幅摆动。

C 的残差 +0.538° 落在死区 `POS_ARRIVE_DEADBAND = 0.01 rad(0.57°)` 内，符合设计。

---

## 6. 调参指南

| 现象 | 调整 |
|---|---|
| 想更快到位 | 加大 `POS_SCURVE_A_MAX` / `POS_SCURVE_J_MAX`（比提 V_MAX 更有效，因为加速段只占总时长的 12%） |
| 起步仍有冲击 | 减小 `POS_SCURVE_J_MAX`（加速度上升更慢） |
| 到位后摆动 | 加大 `POS_TRACK_VLIM_MIN` 让 PID 修正更快介入；或调小 `POS_SCURVE_V_MAX` 让减速更早开始 |
| 跟随误差偏大 | 提高 `POS_SCURVE_FF_GAIN`（上限 1.0）仍不够，说明速度环偏软，应调速度环 kp/ki |
| 想回到改动前 | `POS_SCURVE_ENABLE = 0`（会用同一套代码走固定目标 + 接近限速） |

注意 `POS_SCURVE_V_MAX` 必须小于 `BOARD_BLDC_SW_FOC_POSITION_OUTPUT_MAX`，
否则 PID 没有修正余量。

---

## 7. 已知边界

1. **单段轨迹**：当前只支持"启动一段、走完结束"。连续多段（队列）需要在
   `ctrl_pos_start()` 前判断 `s_sc.state`，本实现会在运动中被新指令直接打断重规划，
   速度会从当前规划速度跳到 0 再重新起步。要做多段衔接需加"按当前实际速度重规划"。
2. **无转矩前馈**：只做了速度前馈，没有把规划加速度前馈到电流环。
   若追求更高的轨迹精度，可以在规划器里留住加速度并加
   `iq_ff = J·a/Kt`（J 为转动惯量），能进一步减小加减速段的跟随误差。
   （各段起点加速度 `ak[]` 本来就在递推里算好了，加回来只是多一个输出字段。）
3. **仿真模型未含反电动势与母线电压限制**，高速段实机加速度可能达不到设定值，
   表现为实际速度略滞后于规划速度（跟随误差增大但不会失控）。
