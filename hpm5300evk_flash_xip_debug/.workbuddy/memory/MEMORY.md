# 项目长期约定（BLDC FOC / HPM5300EVK）

## 构建与工程结构
- 真正参与构建的源文件由 `CMakeLists.txt` 决定；`hpm5300evk_flash_xip_debug/segger_embedded_studio/bldc_foc.emProject`
  **由 CMake 重新生成**，手改会被覆盖。改构建配置必须改 CMakeLists 再跑 cmake。
- 重新生成命令：
  ```
  cd hpm5300evk_flash_xip_debug
  export HPM_SDK_BASE=D:/sdk_env_v1.12.1/hpm_sdk
  export GNURISCV_TOOLCHAIN_PATH=D:/HQKJ/SDK/toolchains/rv32imac_zicsr_zifencei_multilib_b_ext-win
  D:/sdk_env_v1.12.1/tools/cmake/bin/cmake.exe .
  D:/sdk_env_v1.12.1/tools/ninja/ninja.exe
  ```
- 源码分七层：`app → control → motor → sensor → driver → hal`，`debug` 横切。禁止反向依赖。
- 旧单体实现归档在 `src/_archive/bldc_foc.c.legacy`。

## 【头号大坑】参数宏被 board.h 静默覆盖
`boards/hpm5300evk/board.h` 第 307-312 行定义了整套 `BOARD_BLDC_SW_FOC_*_KP/KI`，
而 `hw_map.h` 会先 `#include "board.h"`。因此工程内 `motor_params.h` 若用 `#ifndef` 守卫，
**整块会被跳过，board.h 里的旧值反而生效**，表现为"改了参数完全没变化"。

- 期间速度环一直跑 board.h 的 0.0074/0.0001，位置环一直跑 154.7/0.113。
- 现行做法：`motor_params.h` 中对这一组宏先 `#undef` 再 `#define` 强制覆盖。
- **以后新增任何 `BOARD_BLDC_*` 参数宏，都必须在本文件里 #undef 一次**，并验证生效：
  ```
  python C:\Users\LENOVO\.workbuddy\skills\hpm-sdk-ses-syntax-check\scripts\check.py --src motor/motor.c
  ```
  或用 `gcc -E -P` 展开后看实际取值。

## MCL 中间件的坑
1. **位置环 PID 与速度环 PID 实现不同**：
   - 速度环 `hpm_mcl_control_pi()`：`integral += ki*err`；`out = kp*err + integral`
   - 位置环 `hpm_mcl_position_pid()`：`integral += err`（不含 ki）；`out = kp*err + ki*integral`
   整定公式不能混用。位置环 integral 单位是 rad·拍，integral_max 要按 `output_max/ki` 量级设。
2. **位置环周期是 `speed_loop_ts`(250us/4kHz)，不是配置的 `position_loop_ts`**
   —— `hpm_mcl_loop_init()` 里 `position_ts = &speed_loop_ts`，自己配的 1ms 不生效。ki 按 4kHz 整定。
3. **速度环/位置环必须按模式互斥 enable**：
   `hpm_mcl_current_foc_loop()` 里 `enable_position_loop=true` 但 `ref_position.enable=false` 时，
   位置环照样在跑，反馈取连续多圈角 → err 巨大负值 → 输出饱和 + 积分顶死 + 每拍多一次 SPI 读。
   跑速度环时要显式关掉 `enable_position_loop`（见 `motor_set_loop_mode()`）。
4. `enable_speed_loop=false` 时 `exec_ref.speed` 被强制置 0。
5. detect 判故障会回调 `disable_all_pwm_output()` 关断输出，电机"突然不动"但无任何提示
   —— 用 `pwm_output_is_enabled()` + `motor0.loop/encoder/analog.status` 观测（已挂 J-Scope 探针）。

## 当前已整定参数（motor_params.h，已验证生效）
- 速度环：kp=0.02 / ki=0.00008，integral_max=±3，output_max=±3（对齐 i_max）
  依据 Kt=0.054 N·m/A、J=6.2e-6、Kt/J=8710；因速度反馈 100Hz 四阶 IIR 有约 5ms 时延，
  ki 从理论 2.55e-4 退到 8e-5。仿真阶跃超调约 26%，嫌大可再降 ki。
- 位置环：kp=20 / ki=0.025，integral_max=±300，output_max=±30
  依据 ωn=10rad/s、ζ=1（kp=2ζωn，ki=ωn²·Ts=100×2.5e-4）；输出是速度给定，30rad/s≈286rpm。
- 位置环增强（`ctrl_foc.c` 的 `position_loop_pre/post()`，每拍包在 `hpm_mcl_loop()` 外面）：
  1. 接近限速 output_max = sqrt(2*POS_DECEL_MAX*(|err|-死区))，POS_DECEL_MAX=150；
  2. clamping 抗饱和：输出撞限幅且 err 同向时撤销本拍 integral；
  3. 平滑死区 POS_ARRIVE_DEADBAND=0.01rad + 到位后积分按 POS_INTEGRAL_LEAK=2.0 泄放到 0。
  三者缺一不可。只做抗饱和不做泄放，末端仍会因"减速段攒下的积分(~130)在死区边界把电机推走"
  而持续极限环。仿真：反转峰值 -5.73→-0.06rad/s，残差 -2.15°→-0.41°，末端由振荡变静止。
- 电流环：Kp=Ls·ωc、Ki=Rs·ωc·Ts（PI 零点对消电机极点 R/L）。

## 调试习惯
- J-Scope 走 HSS 直读 RAM，采样率要 ≥50kHz；观测变量必须在 20kHz 中断里刷新
  （`g_ia/ib/ic` 已移到 `ctrl_foc.c` 的 `isr_adc()`）。
- 探针集中在 `src/debug/dbg_probe.h`，新增后要在 `dbg_probe_keep()` 里加一次引用，
  否则会被 `--gc-sections` 删掉。
