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

# 分层源码根目录：六层架构（app / control / motor / sensor / driver / hal）
sdk_inc(src)
sdk_inc(src/hal)
sdk_inc(src/driver)
sdk_inc(src/sensor)
sdk_inc(src/motor)
sdk_inc(src/control)
sdk_inc(src/app)

# ---- hal：硬件抽象层 ----
sdk_app_src(src/hal/bsp.c)

# ---- driver：外设驱动层 ----
sdk_app_src(src/driver/drv_spi.c)
sdk_app_src(src/driver/drv_pwm.c)
sdk_app_src(src/driver/drv_adc.c)
sdk_app_src(src/driver/drv_timer.c)

# ---- sensor：传感器接口层 ----
sdk_app_src(src/sensor/mt6835.c)
sdk_app_src(src/sensor/sens_encoder.c)
sdk_app_src(src/sensor/sens_analog.c)

# ---- motor：电机抽象层 ----
sdk_app_src(src/motor/motor.c)

# ---- control：控制算法层 ----
sdk_app_src(src/control/ctrl_foc.c)
sdk_app_src(src/control/ctrl_align.c)
sdk_app_src(src/control/ctrl_svpwm.c)

# ---- app：应用层 ----
sdk_app_src(src/app/app_monitor.c)
sdk_app_src(src/app/main.c)

generate_ide_projects()
