# 编码器零点偏移：改用片内 FLASH 存储

把原来写死在 `app_cfg.h` 的编译期常量 `ENC_THETA_INITIAL`，改为掉电保存在片内 FLASH
最后一个扇区，上电从 FLASH 读取。已实测：位置环、速度环均正常转动，无异响。

---

## 1. 改动文件清单

| 文件 | 改动 |
|---|---|
| `src/driver/drv_flash.h` | **新增**，driver 层 FLASH 参数存储接口 |
| `src/driver/drv_flash.c` | **新增**，ROM API 实现 |
| `src/app/app_cfg.h` | 新增 `FLASH_PARAM_ENABLE`、`FLASH_PARAM_CALIB_ONCE` 两个开关 |
| `src/app/main.c` | 上电读取偏移；对齐分支后可选回写 |
| `src/debug/dbg_probe.h/.c` | 新增 6 个 J-Scope 探针 |
| `CMakeLists.txt` | 加 `sdk_app_src(src/driver/drv_flash.c)` |

---

## 2. 参数区布局

片内 FLASH 最后一个 sector：

| 项目 | 值 |
|---|---|
| ROM API 偏移（擦写用） | `0xFF000`（= 1MB − 4KB） |
| CPU 映射地址（读取用） | `0x800FF000`（= `BOARD_FLASH_BASE_ADDRESS` + 偏移） |
| 实际偏移 | 由 `drv_flash_init()` 探测 `total_size / sector_size` 后动态确定 |

数据结构 `flash_calib_t`，16 字节：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | `magic` | `0x48454E43`（'HENC'） |
| 4 | `version` | 1 |
| 8 | `theta_initial` | float，机械弧度 |
| 12 | `checksum` | 前三字异或 `0xA5A5A5A5`，防半写/位翻转 |

程序本体（`.text` 到 `0x8001A228`，约 105KB）离末尾扇区很远，不会被覆盖。

---

## 3. 运行流程

### 常规运行（`FLASH_PARAM_CALIB_ONCE = 0`，默认）

```
上电 → drv_flash_load_theta(default, auto_program=false)
     → drv_flash_read_calib()  ← 就是一次 XIP 指针读 0x800FF000
     → 命中：返回 FLASH 里的值，g_flash_valid = 1
     → 未命中：返回 ENC_THETA_INITIAL，g_flash_valid = 0（不擦写）
```

**整条路径不调用任何 ROM API**，等价于一次内存访问。

### 标定（`FLASH_PARAM_CALIB_ONCE = 1`）

```
上电 → 未命中 → drv_flash_write_calib()
     → drv_flash_init()        探测几何（会动 XPI）
     → flash_erase_program()   擦除 + 编程（在 ILM 中执行）
```

---

## 4. 三个必须记住的坑

### 坑 1：XIP 下绝不能调用 ROM 的 XPI 探测类 API ⚠️ 最重要

`rom_xpi_nor_get_config()` / `rom_xpi_nor_auto_config()` / `rom_xpi_nor_get_property()`
**全都会**把 XPI 控制器重新初始化，覆盖 BootROM 为 XIP 配好的高速取指状态。

程序**不会死**，但取指变慢 → 20kHz 控制节拍跑不完 → 电流环时序抖动 →
**电机异响、出力不足、速度环起不来**。

实测证据：`g_flash_erase_ms = 0`（压根没擦写）却照样异响 —— 因为只调了
`drv_flash_init()` 里的探测 API。而 `FLASH_PARAM_ENABLE = 0`（完全不碰 FLASH）就正常。

**结论：擦写绕不开这些 API，所以只在专门的标定固件里调用一次，标定完烧回常规固件。**

### 坑 2：擦写代码必须跑在 RAM 里

XIP 下代码在 FLASH 执行，擦除/编程期间 FLASH 不可读，CPU 一取指就跑飞。做法：

1. 擦写函数加 `ATTR_RAMFUNC`（ROM API 的 `static inline` 也带此属性，内联后仍在 RAM 段）
2. 期间 `disable_global_irq(CSR_MSTATUS_MIE_MASK)`
3. **函数体内不能调用任何 FLASH 中的函数** —— 包括 `clock_get_frequency()`。
   周期→毫秒的换算必须放到 ramfunc 外面做
4. 挑 PWM 无输出、控制环未使能的窗口调用（本项目在 20ms 上电冲洗窗口）

已用 `nm` 验证：`flash_erase_program` 链接在 `0x00000806`（ILM），不在 `0x8000xxxx`。

### 坑 3：ROM API 收"相对偏移"，不是映射地址

- `rom_xpi_nor_erase / program / read` 传 `0xFF000`
- CPU 直接读才用 `0x800FF000`

另外 HPM5361 **没有** `HPM_XPI_SOC_SUPPORT_HYBRID_MODE`（只有 HPM5E00/HPM5E31 有），
不走 `0xB0000000` 那个 hybrid 窗口。

---

## 5. J-Scope 观测点

| 变量 | 含义 |
|---|---|
| `g_flash_status` | 最近一次读/写的 `hpm_stat_t`，0 = 成功 |
| `g_flash_valid` | 1 = 本次用的是 FLASH 里的值；0 = 用的编译期常量 |
| `g_flash_theta` | 本次实际使用的零点偏移（rad） |
| `g_flash_erase_ms` | 上次擦写耗时 ms，**0 = 本次上电没擦写**（常规运行应为 0） |
| `g_flash_erase_cycles` | 上次擦写耗时 CPU 周期 |
| `g_flash_ofs` | 实际使用的参数区偏移，正常应为 `0xFF000` |

---

## 6. 使用步骤

### 常规运行（当前状态）

`app_cfg.h`：`FLASH_PARAM_ENABLE = 1`、`FLASH_PARAM_CALIB_ONCE = 0`
直接烧录即可，上电从 FLASH 读偏移。

### 需要更新 FLASH 里的偏移值

1. `app_cfg.h` 置 `FLASH_PARAM_CALIB_ONCE = 1`
   - 想用对齐结果覆盖：同时置 `ENC_SKIP_ALIGN = 0`
   - 想烧 `ENC_THETA_INITIAL` 的值：保持 `ENC_SKIP_ALIGN = 1`
2. 烧录运行一次，J-Scope 看 `g_flash_status == 0` 即写入成功
3. 把 `FLASH_PARAM_CALIB_ONCE` 改回 `0` 重新烧录

调试器下载不会擦除末尾 sector，参数会保留（已实测验证）。

### 排查 FLASH 相关问题

`FLASH_PARAM_ENABLE = 0` → 完全不碰 FLASH，直接用 `ENC_THETA_INITIAL`，
等价于改动前的行为，用于隔离问题。

---

## 7. 构建状态

ninja 全量通过：

```
FLASH:      106784 B     1 MB     10.18%
ILM:          2264 B   128 KB      1.73%
DLM:         35952 B   130304 B    27.59%
```

验证：
- `main` 中无 `drv_flash_init()` 调用（反汇编确认，只在 `write_calib` 内被调用）
- `flash_erase_program` @ `0x00000806`（ILM）
- 实测位置环、速度环正常转动，无异响
