# 片内 FLASH 参数存储 · 从零讲清

> 面向没接触过 FLASH 存储的读者。读完你应该能回答：参数存在哪里、为什么读只要一行代码、
> 为什么写要那么大阵仗、以及这套代码每个开关该怎么用。
>
> 配套的工程记录（坑位清单、验证步骤）见 `docs/flash_calib.md`。

---

## 0. 一句话概览

我们要做的事只有一个：**把一个 float（编码器零点偏移 4.9862 rad）掉电后还能找回来**。

实现是三句话：

| 环节 | 做法 |
|---|---|
| 存哪 | 片内 FLASH 最后一个扇区，偏移 `0xFF000`（CPU 读用 `0x800FF000`） |
| 读 | 把它当普通内存指针解引用，一行代码，不需要任何驱动 |
| 写 | 整扇区擦除成全 1，再把 16 字节写回去；这段代码必须搬到 RAM 里执行 |

真正麻烦的只有"写"。

---

## 1. FLASH 的三条铁律

FLASH 和 RAM 不是一回事。三条规则决定了后面所有代码为什么那样写。

### 铁律一：位只能从 1 变成 0，不能从 0 变回 1

```
擦除后：1 1 1 1 1 1 1 1      ← 空盘状态，全是 1
写入后：1 0 1 1 0 0 1 1      ← program 操作，只能把某些 1 拉成 0
再想改：0 → 1 做不到         ← 只能整块擦除，回到全 1
```

RAM 想写什么写什么；FLASH 只能"单向涂黑"，要擦干净得整块来。

### 铁律二：擦除的最小单位是扇区，不是字节

本工程扇区 4 KB。改 16 字节参数 = 擦掉 4 KB + 写回 16 字节。
这也解释了为什么代码里要挑"电机没在跑"的时机擦写——**擦除期间整块 FLASH 不可读**。

### 铁律三：擦写寿命有限，约 10 万次

听起来很多，但如果代码里每拍都写，几分钟就报废。所以：

```c
#define FLASH_THETA_EPS (0.001f)   /* 与已存值相差小于 0.001 rad 就跳过 */
```

`drv_flash_save_theta()` 里也做了同样的判断，值没变就不擦。

---

## 2. 参数存在芯片的哪个位置

HPM5361 片内有 1 MB FLASH。固件从低地址往上长，我们把参数放在最末尾，两者永不相撞。

```
偏移视角（ROM API 用）        CPU 映射视角（直接读用）
0x00000  ┌──────────────┐     0x80000000
         │  程序本体     │
         │  约 103 KB    │
0x1A228  ├──────────────┤
         │              │
         │  空闲未用     │
         │              │
0xFF000  ├──────────────┤     0x800FF000   ← 参数区（4 KB，只用了前 16 字节）
0x100000 └──────────────┘     0x80100000
```

**两个地址别混**（这是最容易出错的地方）：

| 用途 | 传什么 | 例子 |
|---|---|---|
| 调 ROM API 擦写 | 相对 FLASH 起始的**偏移** | `0xFF000` |
| CPU 直接读数据 | XPI **映射地址** | `0x800FF000` |

两者相差 `0x80000000`，是芯片把 FLASH 映射到 CPU 地址空间时加的基址。

代码里对应的定义（`src/driver/drv_flash.c`）：

```c
#define FLASH_PARAM_SECTOR_SIZE (0x1000UL)                                /* 4 KB */
#define FLASH_PARAM_OFFSET_DFT  (BOARD_FLASH_SIZE - FLASH_PARAM_SECTOR_SIZE)  /* 0xFF000 */
#define FLASH_PARAM_ADDR        (BOARD_FLASH_BASE_ADDRESS + s_param_offset)    /* 0x800FF000 */
```

---

## 3. 存的是什么：16 字节的 `flash_calib_t`

扇区 4 KB，我们只用了开头 16 字节，其余 4080 字节保持擦除态（`0xFF`），留作以后扩展。

```c
typedef struct {
    uint32_t magic;          /* 0x48454E43 = "HENC"，标记这片被写过 */
    uint32_t version;        /* 1，结构版本 */
    float    theta_initial;  /* 零点偏移，4.9862 rad */
    uint32_t checksum;       /* 前三字的校验 */
} flash_calib_t;
```

字节布局：

| 偏移 | 字段 | 值 | 作用 |
|---|---|---|---|
| +0 | magic | `0x48454E43` | 区分"从没写过"和"写过" |
| +4 | version | `1` | 以后改结构布局时用来识别旧数据 |
| +8 | theta_initial | `4.9862f` | 真正要存的数据 |
| +12 | checksum | 计算得出 | 判断数据是否完整 |

校验公式：

```c
checksum = magic ^ version ^ bits(theta) ^ 0xA5A5A5A5
```

（`bits()` 是把 float 按位 reinterpret 成 uint32，用 union 实现。）

**为什么要三重校验？** 因为"读"不会报错——FLASH 里永远有东西，区别只是内容对不对。
如果刚烧完固件、参数区还没写过，你会读到全 `0xFF`，也就是 `theta = NaN`。
没有校验的话，NaN 会一路传进 FOC 计算，电机行为诡异却查不到原因。

---

## 4. 读：为什么只需要一行代码

### 4.1 XIP 是什么

`hpm5300evk_flash_xip_debug` 这个工程名里的 **XIP = eXecute In Place**，
意思是 CPU 直接从 FLASH 里取指令执行，不用先把代码搬进 RAM。

XPI 控制器把整块 FLASH 映射到了 CPU 地址空间 `0x80000000` 开始的位置。
映射之后，**读 FLASH 和读 RAM 在指令层面完全一样**，都是一次 load。

### 4.2 读的完整流程

```
main.c 上电
  └─ drv_flash_load_theta(ENC_THETA_INITIAL, false)
       └─ drv_flash_read_calib()
            ├─ p = (flash_calib_t *)0x800FF000      ← 就是取个指针
            └─ 校验 magic / version / checksum
                 ├─ 通过 → g_flash_valid = 1，返回 FLASH 里的值
                 └─ 不通过 → g_flash_valid = 0，返回 ENC_THETA_INITIAL
```

核心代码就这几行：

```c
hpm_stat_t drv_flash_read_calib(flash_calib_t *out)
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
```

**注意这里没有调用任何 ROM API。** 这是整个方案能成立的关键，见第 6 节。

### 4.3 调用点

`src/app/main.c`：

```c
#if FLASH_PARAM_ENABLE
    float theta_init = drv_flash_load_theta(ENC_THETA_INITIAL, FLASH_PARAM_CALIB_ONCE);
#else
    float theta_init = ENC_THETA_INITIAL;   /* 排查用：完全不碰 FLASH */
#endif
```

放在 `motor_init()` 之前——零点偏移必须在 FOC 开始计算角度之前就位。

第二个参数 `auto_program` 传 `false`：常规运行只读不写。

---

## 5. 写：为什么要搬到 RAM 里执行

### 5.1 困境

XIP 下代码在 FLASH 里跑。而擦除/编程期间整块 FLASH 不可读。于是：

```
平时：  CPU ──取指令──▶ FLASH（可读）           正常
擦写：  CPU ──取指令──▶ FLASH（忙，读不出）      取不到指令 → 跑飞
```

哪怕只是"进中断查一下向量表"这种动作，都需要读 FLASH，照样跑飞。

### 5.2 解法：ATTR_RAMFUNC

把整个擦写函数连同它调用的 ROM API 链接到 **ILM（片上 RAM）**：

```c
ATTR_RAMFUNC static hpm_stat_t flash_erase_program(const uint32_t *buf, uint32_t bytes)
```

`ATTR_RAMFUNC` 会把函数放进 `.ramfunc` 段，链接脚本把它定位到 ILM。
验证方法（构建后执行）：

```bash
riscv32-unknown-elf-nm.exe output/demo.elf | grep flash_erase_program
```

看到地址是 `0x00000806` 这类（0x0 开头，ILM 区）就对了；
如果是 `0x8000xxxx`（0x8 开头，FLASH 区）说明没生效，绝对不能烧。

### 5.3 擦写三原则

```
① 擦写函数 ATTR_RAMFUNC，且体内不调任何库函数
   memcpy / memset / printf 都在 FLASH 里，一调就死。
   数据由调用方在外面准备好，传指针进来。

② 期间关全局中断
   disable_global_irq(CSR_MSTATUS_MIE_MASK);
   ... 擦写 ...
   restore_global_irq(level);
   否则 20kHz 的 ADC 中断一进来就要取指，必死。

③ 挑 PWM 无输出、控制环已停的窗口调用
   擦写要几十毫秒，这段时间没有中断响应，电机处于"无人看管"状态。
```

### 5.4 完整写入流程

```
drv_flash_write_calib()                    ← 在 FLASH 中执行
  ├─ 在栈上准备 buf[4]，没用到的填 0xFFFFFFFF（符合 FLASH 擦除态）
  ├─ drv_flash_init()                      ← 探测扇区几何（会碰 XPI，只在写时调）
  ├─ 关全局中断
  │    ┌─────────────────────────────────────────┐
  │    │ flash_erase_program()  运行在 ILM 0x00000806 │
  │    │   rom_xpi_nor_get_config()  → 取 nor_cfg     │
  │    │   rom_xpi_nor_erase(0xFF000) → 整扇区回全 1   │
  │    │   rom_xpi_nor_program(buf, 16) → 写入数据     │
  │    └─────────────────────────────────────────┘
  ├─ 恢复中断 + fencei()
  └─ 周期数换算成 ms，写进 g_flash_erase_ms 探针
```

两个细节：

- **`buf` 先填 `0xFFFFFFFF`**：擦除后就是全 1，不用的位保持 1 才符合 FLASH 语义。
- **周期→毫秒的换算必须在 ramfunc 外面做**：`clock_get_frequency()` 在 FLASH 里，
  擦写期间调它会跑飞。所以 ramfunc 里只记 `hpm_csr_get_core_cycle()` 的原始差值，
  出来之后再换算。

---

## 6. 最大的坑：ROM 的 XPI 探测 API 会重配 XPI

这是本次调试实际踩到的，也是"为什么常规运行一个 ROM API 都不能碰"的原因。

### 6.1 现象

加了 FLASH 驱动之后，电机异响、速度环起不来。但探针显示：

```
g_flash_erase_ms = 0     ← 根本没擦写过
g_flash_status   = 0     ← 读成功了
g_flash_theta    = 4.9862 ← 值是对的
```

**一次都没擦写，却照样异响。**

### 6.2 根因

取指速率完全由 BootROM 启动时配好的 XPI 状态决定。而 ROM 的这几个 API：

```c
rom_xpi_nor_get_config()    /* 名字像"只查配置" */
rom_xpi_nor_auto_config()   /* 明显会重配 */
rom_xpi_nor_get_property()  /* 名字像"只查属性" */
```

**全都会把 XPI 控制器重新初始化一遍**，按 cfg_option 覆盖频率、位宽、读模式。
BootROM 配好的高速 XIP 状态被冲掉。

后果是程序不死，但取指变慢：

```
20kHz 控制节拍 = 每 50 μs 必须跑完一次电流环

BootROM 配置：ISR 约 20 μs  ████████░░░░░░░░░░░  余量充足
被重配之后  ：ISR 超过 50 μs ███████████████████  超时
```

节拍丢失 → 电流环时序抖动 → 异响、出力不足、速度环起不来。
**程序不会死，所以极难定位。**

### 6.3 对策

| 场景 | 行为 |
|---|---|
| 常规运行 | 只做 XIP 指针读，**一个 ROM API 都不碰** |
| 需要更新参数 | 置 `FLASH_PARAM_CALIB_ONCE = 1` 烧一次标定固件，擦写完成后改回 0 重烧 |

`drv_flash_init()` 因此被从 `main.c` 移除，挪进了 `drv_flash_write_calib()` 内部——
只有真要擦写时才探测几何。

验证手段：

```bash
riscv32-unknown-elf-objdump.exe -d output/demo.elf | grep drv_flash_init
```

确认 `main` 的函数体里没有对 `drv_flash_init` 的调用。

---

## 7. 配置开关怎么用

`src/app/app_cfg.h`：

```c
#define FLASH_PARAM_ENABLE       1   /* 1=从 FLASH 读；0=完全不碰 FLASH */
#define FLASH_PARAM_CALIB_ONCE   0   /* 1=本次固件专门用来烧参数；0=常规运行 */
```

### 日常运行

```
FLASH_PARAM_ENABLE     = 1
FLASH_PARAM_CALIB_ONCE = 0
```

直接烧。这条路径等价于一次内存读取，与 `ENABLE=0` 的唯一区别是 theta 从 FLASH 取。

### 更新 FLASH 里的参数值

1. 把 `FLASH_PARAM_CALIB_ONCE` 改成 `1`
2. 想烧对齐结果的话，同时把 `ENC_SKIP_ALIGN` 改成 `0`
3. 编译烧录，上电运行一次
4. J-Scope 看 `g_flash_status == 0` 表示写入成功
5. 把 `FLASH_PARAM_CALIB_ONCE` 改回 `0`，重新烧录

调试器下载不会擦末尾扇区，所以参数会一直保留。

### 排查问题

```
FLASH_PARAM_ENABLE = 0
```

完全不碰 FLASH，回到改动前的行为，用来隔离问题。

---

## 8. J-Scope 探针

| 变量 | 含义 | 正常值 |
|---|---|---|
| `g_flash_valid` | 1=FLASH 中有有效数据 | 1 |
| `g_flash_theta` | 本次实际使用的零点偏移 | 4.9862 |
| `g_flash_status` | 最近一次读/写的返回码，0=成功 | 0 |
| `g_flash_erase_ms` | 上次擦写耗时 ms，0=本次上电没擦写 | 0（常规运行时） |
| `g_flash_erase_cycles` | 上次擦写耗时 CPU 周期 | — |
| `g_flash_ofs` | 实际使用的参数区偏移 | 0xFF000 |

验收集例子：把 `ENC_THETA_INITIAL` 临时改成 `1.0f` 重新烧录，
如果 `g_flash_theta` 仍然显示 `4.9862`，就证明确实是从 FLASH 读的，编译期常量已失效。

---

## 9. 改动清单

| 文件 | 改动 |
|---|---|
| `src/driver/drv_flash.h` | 新增。`flash_calib_t` 定义 + 5 个 API 声明 |
| `src/driver/drv_flash.c` | 新增。全部实现，含 `ATTR_RAMFUNC` 擦写函数 |
| `src/app/app_cfg.h` | 新增 `FLASH_PARAM_ENABLE`、`FLASH_PARAM_CALIB_ONCE` |
| `src/app/main.c` | 上电读偏移；标定模式下对齐后回写 |
| `src/debug/dbg_probe.h/.c` | 新增 6 个探针 |
| `CMakeLists.txt` | 加 `sdk_app_src(src/driver/drv_flash.c)` |

API 一览：

```c
hpm_stat_t drv_flash_init(void);                                  /* 探测几何（写时才调） */
hpm_stat_t drv_flash_read_calib(flash_calib_t *out);              /* 读 + 校验 */
hpm_stat_t drv_flash_write_calib(const flash_calib_t *in);        /* 擦除 + 写入 */
float      drv_flash_load_theta(float default_val, bool auto_program);
hpm_stat_t drv_flash_save_theta(float theta, bool force);
```

---

## 10. 以后可以改进的地方

现在的方案够用且已验证，但这几点值得知道：

1. **checksum 用异或偏弱。** 异或检测不出"两个字段互换"这类错误，也检测不出位移。
   换成 CRC32 更可靠。对单个 float 参数来说够用了，参数变多时建议换。

2. **没有冗余备份。** 擦到一半掉电会丢参数（校验会挡住，回退到默认值，电机仍能跑，
   只是要重新标定）。重要的话可以做 A/B 双份：写的时候先写 B 再写 A，读的时候挑有效的那份。

3. **`rom_xpi_nor_erase()` 的长度参数目前传的是 16 字节。**
   实测能正常工作（ROM 内部按最小擦除粒度对齐），想更明确可以改成传扇区大小。

4. **要加更多参数时**：往 `flash_calib_t` 里加字段，把 `FLASH_CALIB_VERSION` 加 1，
   旧数据会因版本不匹配自动判为无效并回退默认值，不会读到错位的数据。
