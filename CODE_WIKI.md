# RC Drift Car — Code Wiki

> **项目概述**：基于 STM32F411 (BlackPill) 的遥控漂移车闭环控制系统。集成 MPU6050 六轴 IMU、SSD1306 OLED 显示、PWM 舵机/电调驱动，并通过姿态解算 + 陀螺仪反馈实现漂移辅助控制。串口输出支持 VOFA+ 上位机实时可视化与 CSV 文本日志。

---

## 目录

1. [项目整体架构](#1-项目整体架构)
2. [目录结构与构建系统](#2-目录结构与构建系统)
3. [模块职责详解](#3-模块职责详解)
4. [关键类 / 结构体与核心函数](#4-关键类--结构体与核心函数)
5. [控制循环与时序流程](#5-控制循环与时序流程)
6. [依赖关系图](#6-依赖关系图)
7. [硬件连接与外设映射](#7-硬件连接与外设映射)
8. [编译、烧录与运行](#8-编译烧录与运行)
9. [调参与参数表](#9-调参与参数表)
10. [扩展与维护建议](#10-扩展与维护建议)

---

## 1. 项目整体架构

### 1.1 三层软件架构

```
┌──────────────────────────────────────────────────────────┐
│                   APPLICATION LAYER                      │
│                                                          │
│   main.c  —— 系统初始化、500Hz 控制循环、PWM 输入捕获    │
│                                                          │
├──────────────────────────────────────────────────────────┤
│                   SERVICE LAYER                          │
│                                                          │
│   vehicle_control.c  ── 舵机PD/角速度追踪 + 电调曲线    │
│   pid.c              ── 通用 PD 控制器 (带阻尼项)         │
│   imu_filter.c       ── 互补滤波 + 动态α + 1D卡尔曼       │
│   data_logger.c      ── UART 日志 / VOFA+ JustFloat      │
│   oled.c             ── SSD1306 帧缓冲式驱动             │
│                                                          │
├──────────────────────────────────────────────────────────┤
│                   HARDWARE ABSTRACTION                   │
│                                                          │
│   mpu6050.c  ── I2C 寄存器读写 / 量程换算 / 零点校准     │
│   system_config.h —— 全局可调参数集中管理                │
│   STM32F4xx HAL —— ST 官方 HAL 库 (CMSIS-DSP)           │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### 1.2 信息流与数据流

```
  ┌──────────┐     I2C @400kHz     ┌─────────────┐
  │ MPU6050  │ ──────────────────▶ │ mpu6050.c   │ ── raw (g,dps)
  │ (Gyro+Acc)│                     └─────────────┘
                                                  │
                                                  ▼
                                          ┌─────────────┐
                                          │ imu_filter.c│
                                          │  互补滤波   │
                                          │  动态α     │
                                          │  1D卡尔曼  │
                                          └──────┬──────┘
                                                 │ pitch, roll, yaw
                                                 │ gyro_x/y/z
                                                 ▼
  ┌──────────┐     PWM(PA4/PA5)      ┌────────────────────┐
  │  RC 接收机│ ───────────────────▶ │ GPIO_EXTI 捕获     │
  │ (HotRC)   │                       └─────────┬──────────┘
                                                 │ rc_channels[]
                                                 ▼
                                          ┌─────────────┐
                                          │ vehicle     │
                                          │ _control.c  │
                                          │  舵机PID    │
                                          │  电调曲线   │
                                          └──────┬──────┘
                                                 │ PWM 比较值
                                                 ▼
                                          ┌─────────────┐
                                          │ TIM2 CH1/2  │
                                          │ (舵机/电调)  │
                                          └─────────────┘
                                                 │
              ┌──────────────────────┬─────────┴─────────┬─────────────────────┐
              ▼                      ▼                   ▼                     ▼
        ┌────────────┐       ┌─────────────┐       ┌─────────────┐       ┌────────────┐
        │ OLED 显示  │       │ UART 日志    │       │ VOFA+ 协议  │       │ (保留扩展)│
        │ SSD1306    │       │ (CSV 文本)   │       │ JustFloat   │       │            │
        └────────────┘       └─────────────┘       └─────────────┘       └────────────┘
```

### 1.3 关键频率与节拍

| 组件 | 频率 / 周期 | 触发源 |
|------|------------|--------|
| **主控制循环** | 500 Hz (2 ms) | TIM4 中断置标志 → `main()` 轮询执行 |
| **IMU 采样** | 500 Hz | 与主循环同步（I2C 阻塞读取） |
| **OLED 刷新** | ~10 Hz | 主循环内 50 分频计数器 |
| **UART 日志** | 100 Hz | `LOG_DIVIDER = 5` |
| **舵机 PWM** | 50 Hz (20 ms) | TIM2 CH1 PWM |
| **电调 PWM** | 50 Hz (20 ms) | TIM2 CH2 PWM |
| **I2C 总线** | 400 kHz | I2C1 (MPU6050) / I2C2 (OLED) |

---

## 2. 目录结构与构建系统

### 2.1 目录树

```
rc_drift_car/
├── CMakeLists.txt              # 顶层 CMake 构建脚本
├── CMakePresets.json           # Debug / Release 预设
├── CLAUDE.md                   # 协作说明
├── Core文件说明.md             # 中文模块说明
├── CubeMX_外设逐框配置.md      # STM32CubeMX 配置截图说明
├── CubeMX_配置指南.md          # CubeMX 配置指南
│
├── Core/
│   ├── Inc/                    # ====== 头文件 ======
│   │   ├── main.h              # 应用总头 (LED GPIO 等)
│   │   ├── system_config.h     # ★ 全局参数集中配置
│   │   ├── mpu6050.h           # MPU6050 驱动 API
│   │   ├── imu_filter.h        # 姿态解算 API
│   │   ├── pid.h               # PD 控制器 API
│   │   ├── vehicle_control.h   # 舵机 + 电调控制 API
│   │   ├── oled.h              # SSD1306 OLED 驱动 API
│   │   ├── data_logger.h       # 日志记录器 API
│   │   ├── stm32f4xx_hal_conf.h
│   │   ├── stm32f4xx_it.h
│   │   ├── tim.h / usart.h / gpio.h / i2c.h
│   │   └── (其他 HAL 生成文件)
│   │
│   └── Src/                    # ====== 源文件 ======
│       ├── main.c              # ★ 系统初始化 + 500Hz 主循环
│       ├── mpu6050.c           # MPU6050 I2C 驱动实现
│       ├── imu_filter.c        # 互补滤波 + 1D卡尔曼 实现
│       ├── pid.c               # PD 控制器实现
│       ├── vehicle_control.c   # 转向 / 油门控制实现
│       ├── oled.c              # SSD1306 帧缓冲驱动实现
│       ├── data_logger.c       # UART / VOFA+ 日志实现
│       ├── stm32f4xx_it.c      # 中断处理 (HAL 生成)
│       ├── stm32f4xx_hal_msp.c # 外设引脚初始化 (HAL 生成)
│       ├── system_stm32f4xx.c  # 系统时钟 (HAL 生成)
│       ├── syscalls.c          # newlib-nano 系统调用
│       └── sysmem.c            # 内存管理
│
├── Drivers/                    # STM32 HAL 驱动 + CMSIS
│   ├── CMSIS/
│   │   ├── Core/               # Cortex-M4 CMSIS 核心头
│   │   ├── Core_A/             # Cortex-A 系列（保留）
│   │   ├── DSP/                # CMSIS-DSP 库（可用于FFT/滤波）
│   │   └── DAP/                # CMSIS-DAP 调试固件
│   └── (STM32F4xx HAL Driver)  # HAL 库源码（由 CubeMX 管理）
│
└── cmake/                      # 工具链文件 + 启动脚本
    ├── gcc-arm-none-eabi.cmake # ARM GCC 工具链
    └── stm32cubemx/            # CubeMX 生成的 CMake 片段
```

### 2.2 构建系统

**CMake 配置要点**（参见 [CMakeLists.txt](file:///d:/RC1.0/rc_drift_car/CMakeLists.txt)）：

| 项 | 配置 |
|----|------|
| C 标准 | C11 (`-std=c11`) |
| 工具链 | `gcc-arm-none-eabi.cmake`（交叉编译） |
| 目标名 | `rc_drift_car`（ELF 文件，后续 objcopy → `.bin` / `.hex`） |
| 编译宏 | `ARM_MATH_CM4`（启用 CMSIS-DSP 针对 Cortex-M4 的优化） |
| 源文件 | `Core/Src/{mpu6050,imu_filter,pid,vehicle_control,data_logger,oled}.c` |
| include 路径 | `Core/Inc` |
| 链接 | 链接到 `stm32cubemx` 子目标（包含 HAL + 启动文件 + 链接脚本） |

**预设**（参见 [CMakePresets.json](file:///d:/RC1.0/rc_drift_car/CMakePresets.json)）：
- `Debug`：无优化 + 调试信息
- `Release`：优化等级 `-O2`（由 CMake 默认）

---

## 3. 模块职责详解

### 3.1 system_config.h — 全局参数中枢

**文件**：[Core/Inc/system_config.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/system_config.h)

**职责**：所有"可调"参数的**唯一集中存放处**。修改参数不需要改 `.c` 文件。分类管理：

| 分组 | 关键常量 | 说明 |
|------|---------|------|
| **系统时钟** | `CONTROL_LOOP_FREQ_HZ=500` | 主循环频率 |
| **MPU6050** | `GYRO_SCALE_FACTOR=1.20f` | 陀螺仪增益校准系数；`ZERO_RATE_THRESHOLD=3.0f dps` 静止判定阈值 |
| **滤波器** | `COMP_FILTER_ALPHA=0.96f`、`IMU_ALPHA_MAX=0.998f` | 互补滤波基线与上限 |
| **控制模式** | `GYRO_CONTROL_MODE=1` | `0`=增益反打，`1`=角速度追踪（带 PID） |
| **舵机** | `SERVO_PWM_{MIN,CENTER,MAX}`、`STEERING_ANGLE_MAX=35°` | PWM 脉宽范围（us）与机械角度 |
| **电调** | `THROTTLE_EXPO=0.3f`、`THROTTLE_DEADBAND=30us` | 油门指数曲线 + 死区 |
| **遥控器** | `RC_CH_STEERING=0`、`RC_CH_THROTTLE=2` | 通道映射 |
| **日志** | `UART_DEBUG_ENABLE=1`、`VOFA_OUTPUT_ENABLE=1`、`LOG_DIVIDER=5` | 日志开关与分频 |
| **OLED** | `OLED_REFRESH_DIV=50`、`OLED_ENABLE=1` | 显示开关与刷新分频 |

### 3.2 mpu6050.c / mpu6050.h — IMU 底层驱动

**文件**：[Core/Src/mpu6050.c](file:///d:/RC1.0/rc_drift_car/Core/Src/mpu6050.c) / [Core/Inc/mpu6050.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/mpu6050.h)

**功能链**：

```
I2C1 (PB6/PB7)
  │
  ├── WHO_AM_I 验证 (0x68/0x70) —— 通信健康检查
  ├── 采样率分频 SMPLRT_DIV=7 → 1kHz
  ├── DLPF 带宽 ~256Hz (CONFIG=0x00)
  ├── 陀螺仪量程 ±1000 dps (GYRO_CONFIG)
  ├── 加速度计量程 ±4 g (ACCEL_CONFIG)
  │
  ▼
MPU6050_ReadRaw()
  │  读取 14 字节 [ACCEL_XOUT_H .. TEMP_OUT_L .. GYRO_ZOUT_L]
  │  大端 → 小端转换
  ▼
MPU6050_ScaleData()
  │  除以灵敏度 (gyro:32.8 LSB/dps, accel:8192 LSB/g)
  │  gyro_z × GYRO_SCALE_FACTOR 校准
  ▼
输出：{ax, ay, az}[g], {gx, gy, gz}[dps], temp_c[℃]

MPU6050_CalibrateGyro()
  │  启动时静止采样 500 次取平均
  ▼
输出：gyro_offset[3] 作为零偏初值
```

**关键数据结构**：

```c
typedef struct { int16_t ax, ay, az, gx, gy, gz, temp; } MPU6050_RawData;
typedef struct { float   ax, ay, az, gx, gy, gz, temp_c; } MPU6050_ScaledData;
```

### 3.3 imu_filter.c / imu_filter.h — 姿态解算核心

**文件**：[Core/Src/imu_filter.c](file:///d:/RC1.0/rc_drift_car/Core/Src/imu_filter.c) / [Core/Inc/imu_filter.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/imu_filter.h)

**算法栈**（四重融合）：

```
① 软件 IIR 低通滤波（硬件 DLPF 的补充）
   gx_f = α · gx_f_prev + (1-α) · gx_raw
   gy_f = α · gy_f_prev + (1-α) · gy_raw
   gz_f = α · gz_f_prev + (1-α) · gz_raw
   系数 α = IMU_GYRO_SOFT_LPF_ALPHA = 0.88
   目的：抑制电机振动和机械谐振引入的高频噪声
   注意：仅滤波 gx/gy/gz，ax/ay/az 不参与（加速度计瞬态响应不应被平滑）

② 陀螺仪零偏去除（零偏由卡尔曼在线估计）
   gx_corr = gx_f - offset_x
   gy_corr = gy_f - offset_y
   gz_corr = gz_f - offset_z   ← offset_z 由 1D 卡尔曼在线估计

③ 加速度计反算姿态
   pitch_accel = atan2(-ax, sqrt(ay²+az²)) × RAD_TO_DEG
   roll_accel  = atan2( ay, az)                    × RAD_TO_DEG
   (注：加速度计无法测 Yaw)

④ 动态 α 互补滤波
   accel_norm = sqrt(ax²+ay²+az²)
   deviation  = clamp((accel_norm - 1.0) / (2.5 - 1.0), 0, 1)
   α = 0.96 + (0.998 - 0.96) × deviation

   pitch = α·(pitch + gx_corr·dt) + (1-α)·pitch_accel
   roll  = α·(roll  + gy_corr·dt) + (1-α)·roll_accel

⑤ Yaw 纯陀螺积分 + 零漂补偿
   yaw += gz_corr · dt         (归一化到 ±180°)

⑥ 1D 卡尔曼零偏估计（Z 轴）
   状态量 x = gyro_z_offset
   观测 z = gz_raw（静止时应为 0）
   预测：P ← P + Q
   更新：K = P/(P+R), x ← x + K·(z-x), P ← (1-K)·P
   触发条件：遥控器回中 + gyro<3dps + |accel_norm-1|<0.12 持续 3s
```

**滤波状态变量**（模块级 `static`，上电清零）：

```c
static float gyro_soft_lpf_gx = 0.0f;  /* X轴角速度滤波状态 */
static float gyro_soft_lpf_gy = 0.0f;  /* Y轴角速度滤波状态 */
static float gyro_soft_lpf_gz = 0.0f;  /* Z轴角速度滤波状态 */
```

> **⚠️ 两级滤波叠加说明**：经 `imu_filter.c` 软滤波后，`gyro_z` 在进入 `vehicle_control.c` 的 `Steering_Update()` 时还会再做一次 IIR 低通（`GYRO_LPF_ALPHA=0.80`）。两级串联 ≈ 二阶低通，截止频率更低，噪声抑制更强，但相位滞后约 133ms（5τ @500Hz）。若上车感觉响应迟钝，可降低 `IMU_GYRO_SOFT_LPF_ALPHA` 到 `0.80~0.85`，或移除 `vehicle_control.c` 中的第二级滤波。

**关键数据结构**：

```c
typedef struct {
    float yaw, pitch, roll;       // 欧拉角（度）
    float gyro_x, gyro_y, gyro_z; // 角速度（dps，已去零偏）
    float alpha;                  // 当前融合权重
    float gyro_z_offset;          // Z 轴零偏估计
    uint8_t zero_allowed;         // 当前是否允许零偏更新
    uint8_t is_static;            // 是否进入静止状态
} IMU_Attitude;

typedef struct {
    float offset;   // 零偏估计
    float P;        // 估计协方差
    float Q;        // 过程噪声（小：零偏变化慢）
    float R;        // 观测噪声（大：测量噪声高）
} IMU_Kalman1D;
```

**RC 辅助归零机制**：`IMU_SetRCInputs(throttle_norm, steering_norm)` 由主循环调用，传递归一化摇杆值。当 `|throttle|<0.05` 且 `|steering|<0.05` 时判定为"绝对回中"，使用标准 `R` 更新；0.05~0.15 区间为"接近回中"，使用 `R×10`（更保守、置信度更低的观测）。

### 3.4 pid.c / pid.h — 通用 PD 控制器

**文件**：[Core/Src/pid.c](file:///d:/RC1.0/rc_drift_car/Core/Src/pid.c) / [Core/Inc/pid.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/pid.h)

**控制律**：

```
u = Kp · error + Kd · (-angular_velocity)
```

- `error` = 目标值 − 测量值（角度环或角速度环）
- `-angular_velocity` = 阻尼项（**直接使用陀螺仪测量值而非误差差分**，避免噪声放大）
- D 项经过一阶低通：`d_filtered = α·d_prev + (1-α)·Kd·(-ω)`
- 输出限幅：`output_min ≤ u ≤ output_max`

**在本项目中的使用场景**：`pid.h` 定义通用结构，由 `vehicle_control.c` 在 Mode 1 时复合使用（加上手动 I 项）。

### 3.5 vehicle_control.c / vehicle_control.h — 执行层

**文件**：[Core/Src/vehicle_control.c](file:///d:/RC1.0/rc_drift_car/Core/Src/vehicle_control.c) / [Core/Inc/vehicle_control.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/vehicle_control.h)

**转向控制 —— 两种模式**：

| 模式 0 — 增益反打（新手友好） | 模式 1 — 角速度追踪（答辩级） |
|------------------------------|-----------------------------|
| `servo = rc_angle − GYRO_GAIN × gyro_z` | PID: `u = Kp·(target_ω − gyro_z) + Ki·∫ + Kd·(-gyro_z)` |
| 遥控直驱舵机 + 陀螺仪自动修正 | 遥控命令 = 目标角速度；PID 自动追 |
| `GYRO_GAIN=0.10` (度舵机/dps) | `YAW_RATE_MAX=300 dps, Kp=0.30, Ki=0.02, Kd=0.08` |
| 逻辑在 `Steering_Update()` 编译开关内 | 同上；I 项在 `steering→target_yaw_rate<1` 时清零防饱和 |

**电调油门控制**：

```
rc_pwm → 归一化 [-1,1] → ThrottleCurve() → PWM 脉宽
         指数曲线: output = (1-expo)·x + expo·x³  (expo=0.3)
         死区: |output|<0.03 → 中位
         正转: THROTTLE_PWM_NEUTRAL+DEADBAND ~ MAX (1530~2000 us)
         倒车: THROTTLE_PWM_NEUTRAL-DEADBAND ~ MIN (1470~1000 us)
```

**关键数据结构**：

```c
typedef struct {
    float rc_angle;              // 遥控映射角度
    float target_yaw_rate;       // 目标角速度（Mode1）
    float gyro_z_filtered;       // 低通陀螺角速度
    float servo_angle_cmd;       // 舵机指令角
    uint32_t servo_pwm;          // 最终 PWM (us)
    PID_Controller pid;          // 内嵌 PD（I 项外部）
} Steering_Control;

typedef struct {
    float throttle_input;        // 归一化 [-1,1]
    float throttle_output;       // 曲线后 [-1,1]
    uint32_t esc_pwm;            // 电调 PWM (us)
} ESC_Control;
```

### 3.6 oled.c / oled.h — SSD1306 显示驱动

**文件**：[Core/Src/oled.c](file:///d:/RC1.0/rc_drift_car/Core/Src/oled.c) / [Core/Inc/oled.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/oled.h)

**硬件**：128×64 单色 OLED，SSD1306 控制器，I2C2 (PB3/PB10)，地址 `0x3C<<1`。

**架构**：帧缓冲（Framebuffer）模型

```
帧缓冲: uint8_t framebuffer[128×8] = 1024 字节
           ↑ 按页组织，每页 128 列，每列 8 像素（纵向 LSB 在上）
字符: 6×8 像素 ASCII 字模 Font6x8[96][6]
API:
  OLED_Init()        —— 18 步命令序列初始化 SSD1306
  OLED_Clear()       —— 清零帧缓冲
  OLED_SetCursor(c,r) —— 字符坐标（0~20 列, 0~7 行）
  OLED_Print(str)    —— 打印字符串（自动换行）
  OLED_PrintInt(val) —— 打印整数
  OLED_PrintFloat(v,int_d,frac_d) —— 打印定点浮点数
  OLED_Refresh()     —— 分页 I2C 写入 GDDRAM（8 页 × 129 字节）
```

**主循环内的显示内容**（每 50 帧刷新一次）：
- 行 0：`Y:{yaw} P:{pitch}`
- 行 1：`R:{roll} GZ:{gyro_z}`
- 行 2：`GX:{gyro_x} GY:{gyro_y}`
- 行 3：`ST:{servo_pwm} TH:{esc_pwm}`

### 3.7 data_logger.c / data_logger.h — 数据记录

**文件**：[Core/Src/data_logger.c](file:///d:/RC1.0/rc_drift_car/Core/Src/data_logger.c) / [Core/Inc/data_logger.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/data_logger.h)

**双输出模式**（编译开关 `VOFA_OUTPUT_ENABLE`）：

| 模式 | 协议 | 适用场景 | 示例 |
|------|------|---------|------|
| **文本日志** (`VOFA_OUTPUT_ENABLE=0`) | 自定义 `key:value` 空格分隔，`\r\n` 结尾 | MATLAB/Python 离线分析、串口助手查看 | `tick:1250 target:15.20deg yaw:12.80deg gyro_z:-45.30dps ...` |
| **VOFA+ JustFloat** (`VOFA_OUTPUT_ENABLE=1`) | 小端 4 字节 float × N + 帧尾 `00 00 80 7F` (+inf) | VOFA+ 上位机实时 3D 可视化姿态 | `10 × float + 4 帧尾 = 44 字节/帧` |

**VOFA+ 通道映射**：
```
float data[10] = { roll, pitch, yaw, gx, gy, gz, alpha, offset, zero_allowed, is_static };
```

**优化**：为避免 newlib-nano 的 `%f` 链接开销与内存压力，`Logger_Log()` 完全手写定点格式化 (`put_int`, `fmt_f2`, `fmt_f3`)，零堆分配。

### 3.8 main.c — 应用入口

**文件**：[Core/Src/main.c](file:///d:/RC1.0/rc_drift_car/Core/Src/main.c)

**启动顺序**：

```
1. HAL_Init()                    —— STM32 HAL 初始化
2. SystemClock_Config()          —— HSE 25MHz × PLL 400/4 = 100MHz 主频
3. MX_GPIO_Init()                —— LED (PC13) + EXTI (PA4/PA5)
4. MX_I2C1_Init() @400kHz        —— MPU6050 总线
5. MX_TIM2_Init() PWM / 50Hz     —— 舵机 + 电调
6. MX_TIM4_Init() 周期 / 500Hz   —— 主循环节拍
7. MX_USART1_UART_Init() 921600  —— 日志串口
8. MX_I2C2_Init() @400kHz        —— OLED 总线

9. MPU6050_Init()                —— 失败则死循环闪灯报警
10. MPU6050_CalibrateGyro()      —— 500 次静止采样
11. IMU_Filter_Init()            —— 写入零偏初值
12. Steering_Init() / ESC_Init() —— 执行层初始化
13. Logger_Init(&huart1)         —— 绑定日志串口
14. HAL_TIM_PWM_Start(CH1/CH2)   —— 启动舵机/电调 PWM
15. HAL_TIM_Base_Start_IT(TIM4)  —— 启动控制周期中断
16. OLED_Init() + 开机画面       —— OLED 初始化
17. DWT->CTRL 启用 → 用于 PWM 脉宽测量 + 循环时间测量
```

**主循环**（`main()` 的 `while(1)`）：

```c
if (control_loop_flag) {            // TIM4 ISR 置位
    control_loop_flag = 0;
    Control_Loop_500Hz();           // ← 见第 5 节
}
```

**中断回调**：

- `HAL_GPIO_EXTI_Callback(GPIO_PIN_4 / GPIO_PIN_5)`：RC PWM 脉宽测量（上升沿记录 DWT 时间戳，下降沿计算脉宽并写入 `rc_channels[]`）。有效脉宽范围 1000~2000 μs。
- `HAL_TIM_PeriodElapsedCallback(TIM4)`：置位 `control_loop_flag = 1`，**不做其他处理**（避免中断嵌套占用过长时间）。

---

## 4. 关键类 / 结构体与核心函数

### 4.1 结构体速查表

| 结构体 | 定义位置 | 主要字段 | 生命周期 |
|--------|---------|---------|---------|
| `MPU6050_RawData` | [mpu6050.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/mpu6050.h#L37-L41) | `ax,ay,az,gx,gy,gz,temp` (int16_t) | 主循环局部，每帧重填 |
| `MPU6050_ScaledData` | [mpu6050.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/mpu6050.h#L43-L47) | `ax,ay,az[g], gx,gy,gz[dps], temp_c` | 同上 |
| `IMU_Attitude` | [imu_filter.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/imu_filter.h#L17-L28) | `yaw,pitch,roll, gyro_{x,y,z}, alpha, offset, flags` | 主循环 `static`，滤波器内部状态 |
| `IMU_Kalman1D` | [imu_filter.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/imu_filter.h#L31-L36) | `offset, P, Q, R` | `IMU_ZeroDriftComp` 内部 |
| `IMU_ZeroDriftComp` | [imu_filter.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/imu_filter.h#L39-L45) | `kalman, gyro_offset[3], accum_time, samples, is_static` | 模块级静态 |
| `PID_Controller` | [pid.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/pid.h#L16-L27) | `Kp, Kd, output_{min,max}, d_lpf_alpha, d_filtered` | 嵌入 `Steering_Control` |
| `Steering_Control` | [vehicle_control.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/vehicle_control.h#L18-L32) | `rc_angle, target_yaw_rate, gyro_z_filtered, servo_pwm, pid` | main.c 静态 |
| `ESC_Control` | [vehicle_control.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/vehicle_control.h#L40-L44) | `throttle_input, throttle_output, esc_pwm` | main.c 静态 |
| `LogFrame` | [data_logger.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/data_logger.h#L17-L30) | `tick, target_angle, yaw, gyro_z, alpha, offset, pd, pwms...` | 主循环局部 |

### 4.2 核心函数速查表

#### IMU 与传感

| 函数 | 位置 | 作用 | 频率 |
|------|------|------|------|
| `MPU6050_Init(hi2c)` | [mpu6050.c](file:///d:/RC1.0/rc_drift_car/Core/Src/mpu6050.c#L28-L76) | 初始化 + WHO_AM_I 校验，失败返回码（1=I2C，2=ID 不对） | 启动时 1 次 |
| `MPU6050_ReadRaw(raw)` | [mpu6050.c](file:///d:/RC1.0/rc_drift_car/Core/Src/mpu6050.c#L80-L97) | 读取 14 字节寄存器，大小端转换 | 500 Hz |
| `MPU6050_ScaleData(raw, scaled)` | [mpu6050.c](file:///d:/RC1.0/rc_drift_car/Core/Src/mpu6050.c#L101-L115) | LSB → 物理量（g / dps / ℃），含 `GYRO_SCALE_FACTOR` | 500 Hz |
| `MPU6050_CalibrateGyro(offset)` | [mpu6050.c](file:///d:/RC1.0/rc_drift_car/Core/Src/mpu6050.c#L119-L134) | 静止采样 500 次，求零偏 | 启动时 1 次 |
| `IMU_Filter_Init(gyro_offset)` | [imu_filter.c](file:///d:/RC1.0/rc_drift_car/Core/Src/imu_filter.c#L144-L149) | 写入零偏初值，初始化卡尔曼 | 启动时 1 次 |
| `IMU_SetRCInputs(thr, str)` | [imu_filter.c](file:///d:/RC1.0/rc_drift_car/Core/Src/imu_filter.c#L73-L76) | 由主循环传入归一化摇杆值，用于辅助静止判定 | 500 Hz |
| `IMU_Filter_Update(gx,gy,gz,ax,ay,az,dt,att)` | [imu_filter.c](file:///d:/RC1.0/rc_drift_car/Core/Src/imu_filter.c#L151-L195) | 核心融合函数：零偏去除 → 加速计反算 → 动态α → 互补 → Yaw 积分 | 500 Hz |
| `IMU_ZeroDrift_Update(...)` | [imu_filter.c](file:///d:/RC1.0/rc_drift_car/Core/Src/imu_filter.c#L208-L259) | 卡尔曼零偏更新；条件：陀螺+加速度静止 + RC 回中 持续 3s | 500 Hz（内部被调用） |

#### 控制

| 函数 | 位置 | 作用 | 频率 |
|------|------|------|------|
| `PID_Init(pid, Kp, Kd, min, max, α)` | [pid.c](file:///d:/RC1.0/rc_drift_car/Core/Src/pid.c#L18-L30) | 初始化 PD 结构 | 启动时 |
| `PID_Update(pid, error, ω)` | [pid.c](file:///d:/RC1.0/rc_drift_car/Core/Src/pid.c#L32-L63) | `u = Kp·e + LPF(Kd·(-ω))`，带限幅 | 控制循环 |
| `Steering_Init(steer)` | [vehicle_control.c](file:///d:/RC1.0/rc_drift_car/Core/Src/vehicle_control.c#L82-L94) | 角度/角速度/PID 归零 | 启动时 |
| `Steering_SetRC(steer, rc_pwm)` | [vehicle_control.c](file:///d:/RC1.0/rc_drift_car/Core/Src/vehicle_control.c#L96-L104) | `rc_pwm(1000~2000us) → [-1,1] → ×35° 或 ×300dps` | 500 Hz |
| `Steering_Update(steer, gyro_z, dt)` | [vehicle_control.c](file:///d:/RC1.0/rc_drift_car/Core/Src/vehicle_control.c#L106-L186) | Mode 0/1 分支 → 角度限幅 → PWM 换算 | 500 Hz |
| `Steering_GetPWM(steer)` | [vehicle_control.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/vehicle_control.h#L37) | 返回 `servo_pwm` (us) | 500 Hz |
| `ESC_Init(esc)` | [vehicle_control.c](file:///d:/RC1.0/rc_drift_car/Core/Src/vehicle_control.c#L194-L198) | 归零 | 启动时 |
| `ESC_SetThrottle(esc, rc_pwm)` | [vehicle_control.c](file:///d:/RC1.0/rc_drift_car/Core/Src/vehicle_control.c#L200-L222) | 归一化 → 指数曲线 → PWM 脉宽 | 500 Hz |
| `ESC_GetPWM(esc)` | [vehicle_control.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/vehicle_control.h#L48) | 返回 `esc_pwm` (us) | 500 Hz |
| `ThrottleCurve(x, expo)` | [vehicle_control.c](file:///d:/RC1.0/rc_drift_car/Core/Src/vehicle_control.c#L73-L78) | `(1-expo)·x + expo·x³` | 每次油门更新 |

#### 显示与日志

| 函数 | 位置 | 作用 | 频率 |
|------|------|------|------|
| `OLED_Init(hi2c)` | [oled.c](file:///d:/RC1.0/rc_drift_car/Core/Src/oled.c#L165-L220) | 18 步 SSD1306 命令序列初始化 | 启动时 1 次 |
| `OLED_Clear()` / `OLED_Refresh()` | [oled.c](file:///d:/RC1.0/rc_drift_car/Core/Src/oled.c#L224-L239) | 帧缓冲清零 / 分页 I2C 写入 | 10 Hz |
| `OLED_SetCursor(c,r)` | [oled.c](file:///d:/RC1.0/rc_drift_car/Core/Src/oled.c#L253-L258) | 字符级光标定位 | 按需 |
| `OLED_Print(str)` | [oled.c](file:///d:/RC1.0/rc_drift_car/Core/Src/oled.c#L303-L308) | 打印 ASCII（自动换行） | 10 Hz |
| `OLED_PrintInt(v)` | [oled.c](file:///d:/RC1.0/rc_drift_car/Core/Src/oled.c#L313-L336) | 打印整数（手写，无 sprintf） | 10 Hz |
| `OLED_PrintFloat(v, id, fd)` | [oled.c](file:///d:/RC1.0/rc_drift_car/Core/Src/oled.c#L344-L383) | 定点打印浮点数（0~4 位小数） | 10 Hz |
| `Logger_Init(huart)` | [data_logger.c](file:///d:/RC1.0/rc_drift_car/Core/Src/data_logger.c#L16-L18) | 绑定 UART 句柄 | 启动时 1 次 |
| `Logger_Log(frame)` | [data_logger.c](file:///d:/RC1.0/rc_drift_car/Core/Src/data_logger.c#L146-L183) | 手写格式化 → UART 文本帧 | 100 Hz |
| `Logger_SendVOFA(att)` | [data_logger.c](file:///d:/RC1.0/rc_drift_car/Core/Src/data_logger.c#L104-L140) | 10 × float + 帧尾 → JustFloat 协议 | 100 Hz |

---

## 5. 控制循环与时序流程

核心函数：`static void Control_Loop_500Hz(void)` 位于 [main.c:581-715](file:///d:/RC1.0/rc_drift_car/Core/Src/main.c#L581-L715)

### 5.1 单帧执行流水线（每 2 ms）

```
TIM4 周期中断 ── control_loop_flag = 1
        │
        ▼
【主循环检测并执行 Control_Loop_500Hz】
        │
        ├─── 0. 循环时间测量（DWT → 实际 dt，夹在 0.5~20 ms）
        │
        ├─── 1. MPU6050_ReadRaw(&raw)          // I2C 阻塞读 ~400μs
        ├─── 2. MPU6050_ScaleData(raw, scaled)  // 定点 → 浮点物理量
        │
        ├─── 3. 【RC → IMU 辅助归零】
        │      rc_channels → throttle_norm/steering_norm ∈ [-1,1]
        │      IMU_SetRCInputs(thr_norm, str_norm)
        │
        ├─── 4. IMU_Filter_Update(gx,gy,gz,ax,ay,az,dt,&att)
        │      ├── 去除零偏
        │      ├── 加速度计反算 pitch/roll
        │      ├── 动态 α（基于 accel_norm）
        │      ├── 互补滤波融合
        │      ├── Yaw 纯陀螺积分
        │      └── 1D 卡尔曼零偏估计（条件触发）
        │
        ├─── 5. 遥控通道映射
        │      Steering_SetRC(steering, rc_channels[CH_STEERING])
        │      ESC_SetThrottle(esc, rc_channels[CH_THROTTLE])
        │
        ├─── 6. 控制律
        │      Steering_Update(steering, att.gyro_z, dt)
        │      ├── Mode 0: servo = rc_angle - 0.10 × gyro_z
        │      └── Mode 1: PID(target_ω - gyro_z) + Ki·∫ + Kd·(-gyro_z)
        │
        ├─── 7. 硬件输出
        │      __HAL_TIM_SET_COMPARE(TIM2, CH1, Steering_GetPWM())
        │      __HAL_TIM_SET_COMPARE(TIM2, CH2, ESC_GetPWM())
        │
        ├─── 8. OLED（每 50 帧）
        │      清行 → Print → PrintFloat → Refresh
        │
        └─── 9. 日志（每 5 帧）
               ├── VOFA=1: Logger_SendVOFA(&att) → 44 字节 JustFloat
               └── VOFA=0: 构造 LogFrame → Logger_Log() → 文本行

    总计 ~1.2 ms / 帧（在 100 MHz Cortex-M4 + I2C 400kHz）
    留 ~0.8 ms 空闲余量。
```

### 5.2 中断 vs 主循环的职责划分

> **设计原则**：中断里只做"最少必要工作"——置标志、记时间戳。计算全部下沉到主循环。

| 中断 | 中断内行为 | 主循环行为 |
|------|-----------|-----------|
| **TIM4** (500 Hz) | `control_loop_flag = 1` | 执行整帧流水线 |
| **EXTI4/5** (RC PWM 边沿) | 记录 DWT 时间戳 / 计算脉宽 → `rc_channels[]` | 消费 `rc_channels[]` 进行归一化 |
| **USART1** (日志) | HAL 内部处理（本项目用阻塞发送，无 DMA 中断） | `HAL_UART_Transmit()` 阻塞发送 |
| **I2C1/I2C2** | HAL 内部处理（阻塞模式） | `MPU6050_ReadRaw()` 阻塞读 |

**为什么不把控制逻辑放进 TIM4 ISR？**
- I2C 阻塞读 ~400 μs × N 次会破坏其他中断（USART、EXTI）的响应。
- 主循环模型更易调试：可在 `Control_Loop_500Hz()` 入口加断点、加 GPIO 翻转探针测量周期。
- 抖动容忍：实际周期若略偏离 2 ms，`measured_dt` 会反馈回滤波器，积分不会累积误差。

---

## 6. 依赖关系图

### 6.1 头文件 include 层次

```
main.c
├── main.h
│   └── stm32f4xx_hal.h
│
├── system_config.h    ◀── 所有模块的"根配置"
│   └── stm32f4xx_hal.h, math.h, stdio.h, string.h
│
├── mpu6050.h
│   └── system_config.h
│
├── imu_filter.h
│   └── system_config.h
│
├── pid.h
│   └── system_config.h
│
├── vehicle_control.h
│   ├── system_config.h
│   └── pid.h
│
├── data_logger.h
│   ├── system_config.h
│   ├── vehicle_control.h
│   └── imu_filter.h
│
└── oled.h
    └── system_config.h
```

### 6.2 模块依赖矩阵

| 模块 | 依赖 | 被谁依赖 |
|------|------|---------|
| **system_config.h** | HAL (外部) | 所有模块 |
| **mpu6050.c** | system_config.h + HAL I2C | main.c |
| **imu_filter.c** | system_config.h | main.c (通过 `IMU_Attitude` 传参给 data_logger) |
| **pid.c** | system_config.h | vehicle_control.c |
| **vehicle_control.c** | system_config.h, pid.h, math.h | main.c, data_logger.h |
| **oled.c** | system_config.h + HAL I2C | main.c |
| **data_logger.c** | system_config.h, imu_filter.h, vehicle_control.h + HAL UART | main.c |

---

## 7. 硬件连接与外设映射

### 7.1 引脚分配总览

| 功能 | 引脚 | 外设 | 备注 |
|------|------|------|------|
| **LED 状态指示** | PC13 | GPIO_Output | 低电平点亮 |
| **舵机 PWM** | PA0 | TIM2_CH1, PWM @50Hz | 1000~2000 us |
| **电调 PWM** | PA1 | TIM2_CH2, PWM @50Hz | 好盈 1060 有刷 |
| **RC 转向通道** | PA4 | GPIO_EXTI4, 双边沿 | HotRC F-06 CH1 |
| **RC 油门通道** | PA5 | GPIO_EXTI5, 双边沿 | HotRC F-06 CH3 |
| **MPU6050 SCL** | PB6 | I2C1_SCL, 400 kHz | AD0=GND → 地址 0x68 |
| **MPU6050 SDA** | PB7 | I2C1_SDA | |
| **OLED SCL** | PB3 | I2C2_SCL, 400 kHz | SSD1306, 地址 0x3C |
| **OLED SDA** | PB10 | I2C2_SDA | |
| **UART1 TX** | PA9 | USART1_TX, 921600bps | 日志输出 |
| **UART1 RX** | PA10 | USART1_RX | 保留（未使用） |
| **HSE 晶振** | PH0/PH1 | RCC_OSC_IN/OUT | 25 MHz 无源 |
| **SWD 调试** | PA13 (SWDIO), PA14 (SWCLK) | | ST-Link V2 |

### 7.2 系统时钟树

```
HSE (25 MHz)
  │
  └──► PLL  —— M=25, N=400, P=4  →  SYSCLK = 100 MHz
                                              │
                             ┌────────────────┼────────────────┐
                             ▼                ▼                ▼
                     AHB PRESC = 1       APB1 PRESC = 2    APB2 PRESC = 1
                     HCLK = 100 MHz     PCLK1 = 50 MHz   PCLK2 = 100 MHz
                     (core, DMA, GPIO) (TIM2-5, UART4/5) (TIM1/9-11, USART1)
                                          TIMCLK = 2×PCLK1 = 100 MHz

FLASH 延迟 = 3 WS (配合 100 MHz @ 3.3V)
```

### 7.3 TIM2 PWM 配置摘要

- **时钟源**：内部时钟 (CK_INT)
- **预分频器 PSC**：99 → 计数时钟 = 100 MHz / 100 = **1 MHz**（1 μs 分辨率）
- **自动重载 ARR**：19999 → PWM 周期 = (19999+1) × 1 μs = **20 ms** → **50 Hz**
- **比较/捕获 CCR**：初始 1500（舵机中位 / 电调零油门）
- **模式**：PWM1（向上计数，CNT<CCR 时高电平）
- **快速输出使能**：OCFast = Enable

### 7.4 TIM4 控制周期配置摘要

- **PSC**：99 → 1 MHz
- **ARR**：1999 → 周期 = 2 ms，即 **500 Hz**
- **更新中断**：`HAL_TIM_Base_Start_IT(&htim4)` → `TIM4_IRQHandler` → `HAL_TIM_PeriodElapsedCallback`

### 7.5 PWM 输入捕获原理（软件实现）

```
PA4/PA5 输入 RC PWM 信号 (~50Hz, 1~2ms 高电平)
  │
  ├── 上升沿 EXTI: t_rising = DWT->CYCCNT / 100   (换算到 μs)
  ├── 下降沿 EXTI: pulse_width = t_current - t_rising
  └── 若 1000 ≤ pulse_width ≤ 2000:
         rc_channels[CH] = pulse_width
      否则丢弃（抗干扰 / 上电毛刺）

DWT 启用: CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
         DWT->CTRL    |= DWT_CTRL_CYCCNTENA_Msk;
         计数频率 = HCLK = 100 MHz → 1 tick = 10 ns
         除以 100 → 1 tick = 1 μs
```

---

## 8. 编译、烧录与运行

### 8.1 工具链要求

| 工具 | 建议版本 |
|------|---------|
| **ARM GCC** | `arm-none-eabi-gcc 12.3+`（随 CubeMX 附带的版本即可） |
| **CMake** | ≥ 3.22（`cmake_minimum_required` 声明） |
| **Ninja** | 任一最近版本（由 `CMakePresets.json` 指定为生成器） |
| **STM32CubeMX** | 6.10+（生成 HAL/启动文件/链接脚本） |
| **ST-Link Utility** 或 **OpenOCD** | 烧录 + 调试 |
| **VOFA+** | 1.3+（可选，用于 3D 姿态可视化） |

### 8.2 构建命令

```bash
# 配置（Debug）
cmake --preset Debug

# 或配置（Release）
cmake --preset Release

# 编译
cmake --build build/Debug        # 或 build/Release

# 产物位置
#   build/Debug/rc_drift_car.elf    (用于调试)
#   build/Debug/rc_drift_car.bin    (用于 ST-Link 烧录)
#   build/Debug/rc_drift_car.hex    (备选)
```

> 若 `cmake/stm32cubemx/` 子目录不存在，请先用 STM32CubeMX 生成代码，或手工提供 HAL 源 + 启动文件 + 链接脚本 `STM32F411CEUx_FLASH.ld`。

### 8.3 烧录

```bash
# 方式 A：ST-Link CLI（推荐）
STM32_Programmer_CLI -c port=SWD -w build/Debug/rc_drift_car.bin 0x08000000 -v -rst

# 方式 B：OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program build/Debug/rc_drift_car.elf verify reset exit"

# 方式 C：STM32CubeIDE / Keil —— 选择 .elf 文件 F11 下载调试
```

### 8.4 运行与验证步骤

```
【第 1 次上电】
  1. 连接 ST-Link 到目标板，不连接动力电。
  2. 烧录固件后复位 → PC13 LED 应常亮（初始化成功）或：
     - 快速闪烁（~10 Hz）：MPU6050 I2C 通信失败 → 检查 PB6/PB7 接线
     - 慢速闪烁（~2 Hz）：WHO_AM_I 不匹配 → MPU6050 器件损坏/焊接问题
  3. OLED 显示 "RC Drift Car / OLED Ready..." 表示 I2C2 正常。

【静止校准】
  4. 上电首 1 秒保持车辆完全静止（陀螺仪零偏校准在进行）。
  5. 校准后 OLED 显示数值：Yaw/Pitch/Roll 应接近 0，Gyro 接近 0。

【遥控验证】
  6. 打开遥控接收机电源，打方向：
     - OLED 的 ST 值（舵机 PWM）在 500~2500 us 之间随摇杆移动。
  7. 推油门：
     - TH 值（电调 PWM）在 1000~2000 us 之间。
     - 首次使用好盈 1060 电调：上电前推油门到最大 → 听到"滴滴" → 回中 → 完成。

【上位机可视化】
  8. USB-TTL 接 PA9(TX)，波特率 921600, 8N1。
  9. VOFA+ 选 "JustFloat" 协议：
     - 字节序：小端
     - 通道数：10
     - 帧尾：0x7F800000 (+inf)
     - 可观察 Roll/Pitch/Yaw 实时变化。
  10. MATLAB: 关闭 VOFA, 用 `fscanf(s, 'tick:%f ...')` 解析文本日志。
```

---

## 9. 调参与参数表

全部参数集中在 [system_config.h](file:///d:/RC1.0/rc_drift_car/Core/Inc/system_config.h)。下表列出**最常修改的参数**与调节建议：

### 9.1 陀螺仪辅助控制（`GYRO_CONTROL_MODE=1` 推荐）

| 参数 | 默认 | 调节方向 | 现象 |
|------|------|---------|------|
| `YAW_RATE_MAX` | 300 dps | 增大 → 满舵时允许更激进旋转；减小 → 更稳 | 打满舵车身旋转速度上限 |
| `YAW_TRACK_Kp` | 0.30 | 增大会更快追上目标，但可能振荡 | 响应速度 |
| `YAW_TRACK_Ki` | 0.02 | 增大会消除稳态偏差，但可能低速飘 | 保持默认，除非发现持续偏航 |
| `YAW_TRACK_Kd` | 0.08 | 增大抑制超调但会放大噪声 | 抖动大时减小 |
| `GYRO_LPF_ALPHA` | 0.80 | 越大越平滑但相位越滞后 | 电机振动噪声大时调高 |

### 9.2 增益反打模式（`GYRO_CONTROL_MODE=0`）

| 参数 | 默认 | 说明 |
|------|------|------|
| `GYRO_GAIN` | 0.10 | 度舵机 / dps。甩尾越猛→需要越大。典型范围 0.05~0.20 |

### 9.3 互补滤波与软滤波

| 参数 | 默认 | 调节建议 |
|------|------|---------|
| `IMU_GYRO_SOFT_LPF_ALPHA` | **0.88** | 软件 IIR 低通系数（新增）。越大越平滑但相位滞后越大；典型 0.85~0.95。振动严重时提高到 0.92~0.98，响应迟钝时降到 0.80~0.85 |
| `IMU_ALPHA_BASE` | 0.96 | 互补滤波基础 α（静止/平稳时）。越高越信任陀螺仪（长期会漂移） |
| `IMU_ALPHA_MAX` | 0.998 | 互补滤波上限 α（剧烈运动时，几乎纯陀螺仪） |
| `IMU_ACCEL_NORM_AGGRESSIVE` | 2.5 | 触发最高 α 的加速度阈值（单位 g） |

### 9.4 零偏在线估计

| 参数 | 默认 | 说明 |
|------|------|------|
| `ZERO_RATE_THRESHOLD` | 3.0 dps | 静止判定陀螺阈值 |
| `ZERO_RATE_DURATION` | 3.0 s | 持续静止多久才触发更新 |
| `IMU_KALMAN_Q` | 1e-6 | 过程噪声（零偏预期变化速度） |
| `IMU_KALMAN_R` | 1e-2 | 观测噪声（静止时陀螺读数波动） |

### 9.5 舵机 / 电调 PWM 范围

| 参数 | 默认 | 调节场景 |
|------|------|---------|
| `SERVO_PWM_MIN / MAX` | 500 / 2500 us | 根据舵机型号调整；先从 1000/2000 开始 |
| `SERVO_PWM_CENTER` | 1500 us | 视舵机机械中位点微调 |
| `STEERING_ANGLE_MAX` | 35° | 车辆实际最大转角（物理测量后填入） |
| `THROTTLE_PWM_MIN / MAX / NEUTRAL` | 1000 / 2000 / 1500 us | 电调校准后的值 |
| `THROTTLE_DEADBAND` | 30 us | 过小会有怠速抖动，过大响应有台阶 |
| `THROTTLE_EXPO` | 0.3 | 0=线性，越大低油门越精细；竞速车 0.4~0.5 |

### 9.6 MPU6050 校准

`GYRO_SCALE_FACTOR` 是**最关键的手工校准参数**：
1. 设置 `UART_DEBUG_ENABLE=1`, `VOFA_OUTPUT_ENABLE=0`
2. 将车辆绕 Z 轴精确旋转 360°（缓慢匀速，用卷尺/刻度辅助）
3. 记录上位机报告的 `yaw` 变化量 Y_meas
4. `GYRO_SCALE_FACTOR = 360° / Y_meas`，将结果写回
5. 重复直到 `Y_meas ≈ 360° ± 2°`

---

## 10. 扩展与维护建议

### 10.1 可能的优化方向

1. **I2C 从阻塞改 DMA**：当前 `MPU6050_ReadRaw()` 用阻塞 HAL，约 400 μs 锁死 CPU。改为 DMA + 半完成回调可释放大量算力，用于增加卡尔曼阶数或运行 SLAM。
2. **USART 改用 DMA**：`Logger_Log()` 当前阻塞发送 200 字节 ≈ 170 μs（921600 bps）。改成环形缓冲 + DMA 可零 CPU 开销。
3. **扩展为 AHRS（航向姿态参考系统）**：`imu_filter.c` 的 Yaw 目前纯陀螺积分，长期漂移不可避免。若增加磁力计（HMC5883L / QMC5883），可将 Yaw 融合为真北参考。
4. **PID 自动调参**：可在上位机记录阶跃响应（打舵 → 测 `gyro_z`），用 MATLAB `pidtune` 自动推荐 `Kp/Ki/Kd`。
5. **故障安全**：当前 RC 信号丢失时 `rc_channels[]` 保留旧值 → 可能继续保持油门。应增加超时检测（例如 >100 ms 无新脉宽 → 强制舵机回中 + 油门归零）。
6. **看门狗**：启用 IWDG（独立看门狗），在 `Control_Loop_500Hz` 中喂狗，防止硬故障卡死。

### 10.2 常见故障排查

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| **上电 LED 狂闪**（>5 Hz） | MPU6050 I2C 不通 | 检查 PB6/PB7 上拉电阻（4.7kΩ 推荐）、地址线 AD0 接地 |
| **OLED 不亮** | I2C2 引脚冲突 / 初始化顺序 | 用逻辑分析仪确认 0x3C 地址 ACK；CubeMX 中 PB3 不能被 JTAG/SWD 复用 |
| **舵机/电调无反应** | TIM2 PWM 未启动 / CCR=0 | 检查 `HAL_TIM_PWM_Start()` 是否返回 `HAL_OK`；示波器看 PA0/PA1 |
| **Yaw 持续漂移** | `GYRO_SCALE_FACTOR` 未校准 / 零偏估计未触发 | 按第 9.6 节校准；确认静止 3s 以上让卡尔曼更新 |
| **车辆发漂 / 振荡** | `Kp` 或 `GYRO_GAIN` 过大 | 减半增益，逐步回加；也可提高 `GYRO_LPF_ALPHA` 滤波噪声 |
| **串口日志乱码** | 波特率不匹配 / 时钟配置错误 | 确认上位机 921600 8N1；示波器看 USART 脉宽是否 ≈ 1.085 μs |

### 10.3 版本与协作约定

- 所有可调参数**必须**先声明在 `system_config.h`，再被其他模块引用。禁止在 `.c` 中硬编码魔法数字。
- 修改 `GYRO_CONTROL_MODE` 等编译开关后**务必执行 clean rebuild**，避免局部编译旧目标导致未定义行为。
- 若启用 `UART_DEBUG_ENABLE=1`，请勿在上电前连接动力电（日志会占 CPU，且电机意外旋转可能伤人）。
- Git 提交信息格式：`模块: 简短描述`，例如 `vehicle_control: 电调校准流程改为双向中位检测`。

---

*本文档基于 `Core/Src/` 与 `Core/Inc/` 的实际代码自动分析整理。如修改源码，请同步更新相关章节。*
