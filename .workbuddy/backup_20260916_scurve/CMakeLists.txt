# Copyright (c) 2021-2024,2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.13)
set(CONFIG_MOTORCTRL_V2 1)
set(CONFIG_HPM_ADC_V2 1)

set(RV_ABI "ilp32f")
set(RV_ARCH "rv32imafc")

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(bldc_foc)
sdk_compile_definitions(-DCONFIG_MCL_HAS_EXTRA_CONFIG="mcl_app_config.h")

# ============================================================================
# 分层源码（七层：hal / driver / sensor / motor / control / debug / app）
#
# 依赖方向（上层可调用下层，反向禁止）：
#     app  ->  control  ->  motor  ->  sensor  ->  driver  ->  hal
#     debug 为横切层，任何层都可引用，自身不依赖业务层
# ============================================================================
sdk_inc(src)
sdk_inc(src/hal)
sdk_inc(src/driver)
sdk_inc(src/sensor)
sdk_inc(src/motor)
sdk_inc(src/control)
sdk_inc(src/debug)
sdk_inc(src/app)

# ---- hal：硬件映射层（统一 SDK 头 + 板级引脚/时钟）----
sdk_app_src(src/hal/hal_bsp.c)

# ---- driver：外设驱动层 ----
sdk_app_src(src/driver/drv_spi.c)
sdk_app_src(src/driver/drv_pwm.c)
sdk_app_src(src/driver/drv_adc.c)
sdk_app_src(src/driver/drv_timer.c)
sdk_app_src(src/driver/drv_qei.c)
sdk_app_src(src/driver/drv_flash.c)

# ---- sensor：传感器层 ----
sdk_app_src(src/sensor/mt6835.c)
sdk_app_src(src/sensor/sens_encoder.c)
sdk_app_src(src/sensor/sens_analog.c)

# ---- motor：电机抽象层 ----
sdk_app_src(src/motor/motor.c)
sdk_app_src(src/motor/motor_hw_foc.c)

# ---- control：控制算法层 ----
sdk_app_src(src/control/ctrl_svpwm.c)
sdk_app_src(src/control/ctrl_foc.c)
sdk_app_src(src/control/planner_scurve.c)

# ---- debug：J-Scope 探针（横切层）----
sdk_app_src(src/debug/dbg_probe.c)

# ---- app：应用层 ----
sdk_app_src(src/app/app_monitor.c)
sdk_app_src(src/app/main.c)

# ---- 固定 SES 调试器为 J-Link ----
# CMake 重新生成 emProject 时，SDK 默认会把 debug_target_connection 写成
# "GDB Server" + openocd(ft2232)，把手动选的 J-Link 覆盖掉，
# 导致 SES 里 Build 成功但 Download 静默失败、板子还在跑旧固件。
# 必须在 generate_ide_projects() 之前调用。
sdk_ses_opt_debug_connection(J-Link)
sdk_ses_opt_debug_jlink_speed(4000)

generate_ide_projects()
