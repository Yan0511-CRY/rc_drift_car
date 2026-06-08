# Core 文件夹 — 文件功能说明

> 路径: `rc_drift_car/Core/`，分为 `Inc/` (头文件) 和 `Src/` (源文件)

---

## 一、头文件 (Core/Inc/)

### 1. system_config.h — 全局配置中心

**是整个项目最重要的文件。** 所有可调参数集中在这里，调参不用翻其他文件。

| 分类 | 包含内容 |
|------|----------|
| 系统时钟 | 控制循环 500Hz |
| MPU6050 | I2C 地址、采样率、量程、零漂阈值 |
| 互补滤波 | α 系数、时间常数 |
| 转向控制 | Mode 0 增益 / Mode 1 PID 参数、舵机 PWM 范围 |
| 油门控制 | 好盈 1060 PWM 映射、指数曲线系数 |
| 接收机 | 通道映射、引脚定义 (PA4/PA5)、有效脉宽范围 |
| 数据日志 | 波特率、分频系数、VOFA+ 开关 |
| OLED | 从机地址、刷新分频、使能开关 |

---

### 2. main.h — 主程序头文件

CubeMX 自动生成。声明了各外设句柄 (`hi2c1`, `htim2`, `huart1` 等) 和初始化函数。**不需要手动改**。

---

### 3. mpu6050.h — MPU6050 六轴传感器接口

| 内容 | 说明 |
|------|------|
| 寄存器地址宏 | `MPU6050_REG_*`，I2C 寄存器映射表 |
| 量程枚举 | `MPU6050_GYRO_FS_250/500/1000/2000`、`MPU6050_ACCEL_FS_2G/4G/8G/16G` |
| 数据结构 | `MPU6050_RawData` (int16 原始值)、`MPU6050_ScaledData` (物理单位) |
| 函数声明 | `Init`、`ReadRaw`、`ScaleData`、`CalibrateGyro` |

对应源文件: [mpu6050.c](#11-mpu6050c---mpu6050-驱动实现)

---

### 4. imu_filter.h — 姿态解算接口

| 内容 | 说明 |
|------|------|
| 数据结构 | `IMU_Attitude` (pitch/roll/yaw + 三轴角速度)、`IMU_ZeroDriftComp` (零漂状态机) |
| 函数声明 | `IMU_Filter_Init`、`IMU_Filter_Update`、`IMU_ZeroDrift_*` |

对应源文件: [imu_filter.c](#12-imu_filterc---姿态解算实现)

---

### 5. pid.h — PD 控制器接口

| 内容 | 说明 |
|------|------|
| 数据结构 | `PID_Controller` (Kp, Kd, 限幅, 低通系数) |
| 函数声明 | `PID_Init`、`PID_Update` |

对应源文件: [pid.c](#14-pidc---pd-控制器实现)

---

### 6. vehicle_control.h — 车辆执行层接口

| 内容 | 说明 |
|------|------|
| 数据结构 | `Steering_Control` (RC/陀螺/PID 状态 + 最终 PWM)、`ESC_Control` (油门状态) |
| 函数声明 | `Steering_Init/SetRC/Update/GetPWM`、`ESC_Init/SetThrottle/GetPWM`、`ThrottleCurve` |

对应源文件: [vehicle_control.c](#13-vehicle_controlc---车辆执行层实现)

---

### 7. data_logger.h — 串口日志接口

| 内容 | 说明 |
|------|------|
| 数据结构 | `LogFrame` (所有需要记录的运行数据) |
| 函数声明 | `Logger_Init`、`Logger_Log` (CSV 文本)、`Logger_SendVOFA` (二进制 3D 可视化) |

对应源文件: [data_logger.c](#15-data_loggerc---串口日志实现)

---

### 8. oled.h — SSD1306 OLED 显示接口

| 内容 | 说明 |
|------|------|
| 宏定义 | I2C 地址 0x3C、分辨率 128×64、命令/数据字节、行高 |
| 函数声明 | `OLED_Init`、`OLED_Clear`、`OLED_SetCursor`、`OLED_Print`、`OLED_PrintFloat`、`OLED_Refresh` |

对应源文件: [oled.c](#16-oledc---ssd1306-oled-驱动实现)

---

### 9. stm32f4xx_hal_conf.h — HAL 库外设使能配置

CubeMX 自动生成。通过 `#define HAL_XXX_MODULE_ENABLED` 控制哪些外设 HAL 驱动参与编译。**不需要手动改**。

---

### 10. stm32f4xx_it.h — 中断服务函数声明

CubeMX 自动生成。声明 `SysTick_Handler`、`EXTI4_IRQHandler` 等 ISR。**不需要手动改**。

---

## 二、源文件 (Core/Src/)

### CubeMX 生成文件（简介，不需要手动编辑）

| 文件 | 作用 |
|------|------|
| **main.c** | 程序入口，初始化 + 主循环 + 控制循环 + ISR 回调 |
| **stm32f4xx_hal_msp.c** | MSP 层：各外设的引脚/时钟/DMA/NVIC 初始化，CubeMX 生成后覆盖 |
| **stm32f4xx_it.c** | ISR 入口函数（如 `EXTI4_IRQHandler`），调用 HAL 回调，CubeMX 生成后覆盖 |
| **system_stm32f4xx.c** | CMSIS 标准系统初始化：FPU/时钟启动代码，**不要改** |
| **sysmem.c** | newlib-nano 的 `_sbrk` 实现（malloc 底层），**不需要手动改** |
| **syscalls.c** | POSIX 系统调用桩（`_write`/`_read` 等），用于 printf 重定向 |

---

### 11. mpu6050.c — MPU6050 驱动实现

**职责**：通过 I2C1 与 MPU6050 通信，屏蔽所有寄存器操作细节。

| 函数 | 作用 |
|------|------|
| `MPU6050_Init()` | 上电唤醒 → 配采样率(1kHz) → 设量程(±1000dps/±4g) → 校准 LSB 换算系数 |
| `MPU6050_ReadRaw()` | 一次性读 14 字节 (6轴+温度)，大端转小端，返回 `MPU6050_RawData` |
| `MPU6050_ScaleData()` | `RawData` → `ScaledData`，int16 LSB → g 和 °/s |
| `MPU6050_CalibrateGyro()` | 采样 500 次取平均，得到静止零偏 `offset[3]` |

> 调用链: `main.c → Control_Loop_500Hz() → ReadRaw() → ScaleData()`

---

### 12. imu_filter.c — 姿态解算实现

**职责**：将 MPU6050 原始物理量融合为欧拉角 (Pitch/Roll/Yaw) + 角速度。

| 函数 | 作用 |
|------|------|
| `IMU_Filter_Init()` | 写入启动校准得到的陀螺零偏 + 初始化零漂补偿状态机 |
| `IMU_Filter_Update()` | **核心算法**：①去零偏 → ②加速度计反算 Pitch/Roll → ③互补滤波融合 (α=0.96) → ④Yaw 纯积分 + 零漂补偿 |
| `IMU_ZeroDrift_Update()` | 零速检测：\|gz\|<3dps 持续 3 秒 → EMA 缓慢修正 Yaw 零偏 |

> **互补滤波公式**: `angle = 0.96*(angle + gyro*dt) + 0.04*accel_angle`

---

### 13. vehicle_control.c — 车辆执行层实现

**职责**：把遥控操作和 IMU 数据变成舵机和油门的 PWM 值。

| 函数 | 作用 |
|------|------|
| `Steering_Init()` | 舵机状态初始化 + PID 参数赋值 |
| `Steering_SetRC()` | RC 脉宽 → 归一化 → 目标舵角 |
| `Steering_Update()` | **两种模式分支**：Mode 0 增益反打 / Mode 1 角速度 PID 追踪 |
| `Steering_GetPWM()` | 角度 → TIM 比较值，直接喂给 HAL |
| `ESC_Init()` / `ESC_SetThrottle()` / `ESC_GetPWM()` | 油门通道：死区检测 + 指数曲线 + 限幅 |
| `ThrottleCurve()` | 油门映射: `out = (1-expo)*in + expo*in³` |

> **Mode 0**: `servo = RC_angle - GAIN × gyro_z`
> **Mode 1**: `servo = PID(RC_target_yaw_rate, gyro_z)`

---

### 14. pid.c — PD 控制器实现

**职责**：提供 Mode 1 角速度追踪用的 PD 控制器。

| 函数 | 作用 |
|------|------|
| `PID_Init()` | 写入 Kp/Kd、输出限幅、D 项低通系数 |
| `PID_Update()` | `P * error + D * (0 - angular_velocity)`，D 项直接用陀螺仪角速度而非误差差分 |

> 不带 I 项，因为漂移不需要消除稳态误差。D 项用陀螺仪直接测量角速度做阻尼，物理意义更明确。

---

### 15. data_logger.c — 串口日志实现

**职责**：把运行数据发到电脑，支持两种协议。

| 函数 | 作用 |
|------|------|
| `Logger_Init()` | 绑定 UART 句柄 |
| `Logger_Log()` | **CSV 文本模式**：纯手写定点数转字符串 (避免 newlib-nano 浮点 bug)，格式如 `tick:1250 target:15.20deg yaw:12.80deg ...` |
| `Logger_SendVOFA()` | **VOFA+ JustFloat 二进制模式**：6 个 float + 帧尾，用于 3D 姿态可视化 |

---

### 16. oled.c — SSD1306 OLED 驱动实现

**职责**：在 0.96 寸 128×64 OLED 上显示实时 IMU 数据。

| 函数 | 作用 |
|------|------|
| `OLED_Init()` | I2C 初始化序列 (电荷泵/对比度/寻址模式/正显) |
| `OLED_Clear()` | 清空帧缓冲 + 刷新全屏 |
| `OLED_SetCursor()` | 设置列和页 (行) |
| `OLED_Print()` | ASCII 字符串 → 6×8 字库查表 → 写入帧缓冲 |
| `OLED_PrintFloat()` | 浮点数 → 格式化字符串 + 打印 |
| `OLED_Refresh()` | 帧缓冲 → I2C 全屏写入 |

> 显示内容 (4 行): Yaw/Pitch → Roll/GyroZ → GyroX/GyroY → 舵机/油门 PWM

---

## 三、代码调用链 (数据流)

```
接收机 PWM (PA4/PA5 EXTI)
    │
    ▼
rc_channels[] ──────────────────────────┐
                                        │
MPU6050  ──→  ReadRaw()  ──→  ScaleData()  ──→  IMU_Filter_Update()
  │                                                   │
  │ 原始寄存器字节                互补滤波后             │
  │                              Pitch/Roll/Yaw/Gyro   │
  │                                                   ▼
  │                           ┌── Steering_Update()
  │                           │     Mode 0: 增益反打
  │                           │     Mode 1: PID 角速度追踪
  │                           │
  │                           ├── ESC_SetThrottle()
  │                           │     指数曲线油门
  │                           │
  │                           ├── Logger_Log() / SendVOFA()
  │                           │     UART → 电脑
  │                           │
  │                           └── OLED 显示
  │
  ▼
 TIM2 CH1/CH2 → 舵机 + 电调
```

---

## 四、新增依赖 VS CubeMX 自动生成

| 文件 | 来源 | CubeMX 重新生成? |
|------|------|:---:|
| main.c / main.h | CubeMX + 用户代码 | 🔄 合并 (用户区保留) |
| stm32f4xx_hal_msp.c | CubeMX 自动 | 🔄 完全覆盖 |
| stm32f4xx_it.c | CubeMX 自动 | 🔄 完全覆盖 |
| stm32f4xx_hal_conf.h | CubeMX 模板 | ⚠️ 只生成一次 |
| system_stm32f4xx.c | CMSIS 标准 | ⚠️ 不覆盖 |
| sysmem.c / syscalls.c | newlib 适配 | ⚠️ 不覆盖 |
| **mpu6050.c/h** | 👤 用户手写 | ✅ 安全 |
| **imu_filter.c/h** | 👤 用户手写 | ✅ 安全 |
| **pid.c/h** | 👤 用户手写 | ✅ 安全 |
| **vehicle_control.c/h** | 👤 用户手写 | ✅ 安全 |
| **data_logger.c/h** | 👤 用户手写 | ✅ 安全 |
| **oled.c/h** | 👤 用户手写 | ✅ 安全 |
| **system_config.h** | 👤 用户手写 | ✅ 安全 |
| **CMakeLists.txt** (顶层) | 👤 半自动 | ⚠️ 用户源文件需手动加 |
| **CMakeLists.txt** (cmake/) | CubeMX 自动 | 🔄 完全覆盖 |

> 🔄 = 每次都覆盖，不要改这里
> ⚠️ = 生成一次，可能需补内容
> ✅ = 完全由你维护，CubeMX 不会动
