# CubeMX 外设配置完整指南 — RC漂移车 (STM32F411CEU6)

> 适用芯片：STM32F411CEU6 (BlackPill / WeAct)
> CubeMX 版本：6.x
> 最终目标：生成 HAL 库工程，配合 `/Src` `/Inc` 目录下的控制代码

---

## 一、新建工程

1. 打开 STM32CubeMX → **File → New Project**
2. 搜索芯片 `STM32F411CEU6` → 双击选中 → 进入 Pinout 界面

---

## 二、System Core 配置

### 2.1 SYS — 调试接口

| 参数 | 值 |
|------|-----|
| Debug | **Serial Wire** |

> 不开启 SWD 会导致芯片锁死无法烧录，务必设置。

### 2.2 RCC — 时钟源

| 参数 | 值 |
|------|-----|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator** |
| Low Speed Clock (LSE) | Disable |

> BlackPill 板载 25MHz 晶振，选 Crystal。

---

## 三、Clock Configuration — 时钟树

目标：**SYSCLK = 100MHz**（F411 最高频率）

```
HSE: 25 MHz
  ↓ PLL Source = HSE, /25 → 1MHz
  ↓ PLLMUL = x192 (PLLN=192) → 192MHz VCO
  ↓ PLLP = /2 → 96MHz (但这路不用做SYSCLK)
  ↓ 实际路径:
  ↓ /25 → 1MHz, ×400 (PLLN=400) → 400MHz VCO
  ↓ PLLP = /4 → 100MHz PLLCLK
```

逐项填入：

| 参数 | 值 |
|------|-----|
| **HSE** | 25 MHz |
| **PLL Source Mux** | HSE |
| **PLLM** | /25 |
| **PLLN** | x400 |
| **PLLP** | /4 |
| **PLLQ** | /8 |
| **SYSCLK Source** | PLLCLK |
| **APB1 Prescaler** | /2 (→ 50MHz, max for F411) |
| **APB2 Prescaler** | /1 (→ 100MHz, max for F411) |

验证：顶部显示 **HCLK = 100 MHz**, **APB1 = 50 MHz**, **APB2 = 100 MHz**

---

## 四、I2C1 — MPU6050 传感器

### Pinout

| 引脚 | 功能 | 说明 |
|------|------|------|
| PB6 | I2C1_SCL | 时钟线 |
| PB7 | I2C1_SDA | 数据线 |

在 Pinout 界面找到 PB6/PB7 → 分别设为 **I2C1_SCL** / **I2C1_SDA**

### Configuration

| 参数 | 值 | 说明 |
|------|-----|------|
| I2C Speed Mode | **Fast Mode** | 400kHz |
| I2C Speed Frequency | 400 | 400kHz标准快模式 |
| Rise Time | 100ns | 默认即可 |
| Fall Time | 10ns | 默认即可 |
| Clock No Stretch Mode | Disabled | |

### GPIO Settings (CubeMX 会自动配置，确认即可)

| 引脚 | Pull-up |
|------|---------|
| PB6 | Pull-up |
| PB7 | Pull-up |

> 如果 MPU6050 模块上已经有 4.7kΩ 上拉电阻，CubeMX 的片上弱上拉不冲突。

---

## 五、TIM2 — 舵机 + 电调 PWM 输出

### Pinout

| 引脚 | 功能 | 对应 |
|------|------|------|
| PA0 | TIM2_CH1 | 舵机信号线 |
| PA1 | TIM2_CH2 | 电调信号线 |

PA0 → **TIM2_CH1**, PA1 → **TIM2_CH2**

### Configuration — 参数设置

PWM 频率需要 **50Hz**（舵机和电调标准频率）：

**计算公式：** `PWM_Freq = TIM_CLK / (PSC + 1) / (ARR + 1)`

TIM2 挂在 APB1，APB1 = 50MHz，Timer Clock = 50MHz × 2 = **100MHz**（因为 APB1 prescaler ≠ 1，定时器时钟自动 x2）

```
PWM_Freq = 100,000,000 / (PSC + 1) / (ARR + 1) = 50Hz

取 PSC = 99 → 100,000,000 / 100 = 1,000,000Hz 的计数器时钟
ARR = 20000 - 1 = 19999 → 1,000,000 / 20000 = 50Hz ✅
```

| 参数 | 值 |
|------|-----|
| **Counter Mode** | Up |
| **Prescaler (PSC)** | 99 |
| **Counter Period (ARR)** | 19999 |
| **Auto-reload preload** | Enable |
| **Internal Clock Division** | No Division |

### Channel 1 (舵机)

| 参数 | 值 |
|------|-----|
| Mode | **PWM Generation CH1** |
| Pulse | 1500 (中位) |
| Output compare preload | Enable |
| CH Polarity | High |
| Fast Mode | Enable |

### Channel 2 (电调)

| 参数 | 值 |
|------|-----|
| Mode | **PWM Generation CH2** |
| Pulse | 1500 (中位/停止) |
| 其余同 CH1 | |

### GPIO Settings

| 引脚 | Speed | Pull |
|------|-------|------|
| PA0 | High | No pull |
| PA1 | High | No pull |

---

## 六、TIM3 — RC接收机 PPM 输入捕获

### Pinout

| 引脚 | 功能 |
|------|------|
| PA6 | TIM3_CH1 |

PA6 → **TIM3_CH1**

### Configuration

TIM3 同样挂 APB1，Timer Clock = **100MHz**。

输入捕获需要微秒级分辨率（1us 精度足够区分 1000~2000us 脉宽）：

```
PSC = 99 → 100MHz / 100 = 1MHz = 1us 分辨率
ARR = 65535 (最大值，装得下4000us以上的帧间隔)
```

| 参数 | 值 |
|------|-----|
| **Counter Mode** | Up |
| **Prescaler (PSC)** | 99 |
| **Counter Period (ARR)** | 65535 |
| **Internal Clock Division** | No Division |

### Channel 1

| 参数 | 值 |
|------|-----|
| Mode | **Input Capture direct mode** |
| **Polarity Selection** | **Rising Edge** |
| Input Filter | 8 (轻度数字滤波，去抖) |
| Prescaler Division | No division |

### NVIC — 中断使能

在 **NVIC Settings** 页签：

| 中断 | 勾选 |
|------|------|
| **TIM3 global interrupt** | ✅ |

> 重要：不勾选的话 `HAL_TIM_IC_CaptureCallback` 不会被调用。

### GPIO Settings

| 引脚 | Pull |
|------|------|
| PA6 | No pull |

---

## 七、TIM4 — 500Hz 控制循环定时器

> **STM32F411 没有 TIM6/TIM7 基本定时器！** 改用 TIM4。
> TIM4 是通用定时器，但我们不分配任何输出引脚，只当内部中断源用。

### 配置前确认

TIM4 点开后，**Mode 页签**的 Channel1~4 全部保持 **Disable**（不分配任何引脚）。

### Configuration

TIM4 挂 APB1，Timer Clock = **100MHz**（同 TIM2 的 x2 逻辑）

```
500Hz → 周期 2000us
TIM4_CLK = 100,000,000 / (PSC + 1) / (ARR + 1) = 500Hz

取 PSC = 99 → 100,000,000 / 100 = 1,000,000Hz
ARR = 2000 - 1 = 1999 → 1,000,000 / 2000 = 500Hz ✅
```

| 参数 | 值 |
|------|-----|
| **Prescaler (PSC)** | 99 |
| **Counter Period (ARR)** | 1999 |
| **Auto-reload preload** | Enable |

### NVIC — 中断使能

| 中断 | 勾选 |
|------|------|
| **TIM4 global interrupt** | ✅ |

> 同样必须勾选，否则控制循环永远不触发。

---

## 八、USART1 — 数据日志串口

### Pinout

| 引脚 | 功能 |
|------|------|
| PA9 | USART1_TX |
| PA10 | USART1_RX |

PA9 → **USART1_TX**, PA10 → **USART1_RX**

> RX 在数据日志里没用，但建议引出来方便以后扩展（比如用串口调参）。

### Configuration

| 参数 | 值 |
|------|-----|
| **Baud Rate** | 921600 Bits/s |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| Over Sampling | 16 Samples |
| **Hardware Flow Control** | **Disable** |

### DMA Settings (可选，推荐)

如果后续日志数据量大，可以加 DMA：

- Add → **USART1_TX** → DMA Request
- Direction: Memory to Peripheral
- Priority: Low
- Mode: Normal

> 目前的 data_logger.c 用的是阻塞发送 + 1ms timeout，500Hz 降频到 100Hz 输出完全够用，DMA 不是必须的。但如果控制循环负载高，建议加 DMA 释放 CPU。

### GPIO Settings

| 引脚 | Pull |
|------|------|
| PA9 | No pull |
| PA10 | No pull |

---

## 九、GPIO — LED 心跳指示

### Pinout

| 引脚 | 功能 |
|------|------|
| PC13 | GPIO_Output |

PC13 → **GPIO_Output**

### Configuration

| 参数 | 值 |
|------|-----|
| GPIO output level | High (关灯) |
| GPIO mode | Output Push Pull |
| Pull-up/Pull-down | No pull |
| Maximum output speed | Low |
| **User Label** | **LED** |

---

## 十、NVIC 总览

在 **NVIC** 页签确认所有中断已开启：

| 中断线 | 状态 | 优先级 |
|--------|------|--------|
| TIM3 global interrupt | ✅ Enabled | Preempt: 2, Sub: 0 |
| TIM4 global interrupt | ✅ Enabled | Preempt: 3, Sub: 0 |

**优先级策略：**
- TIM3 (PPM输入捕获) 优先级略高 → 确保遥控指令不丢帧
- TIM4 (控制循环) 优先级最低 → 不打断通信中断

I2C 和 UART 的 NVIC 默认由 HAL 自动处理，无需手动配置。

---

## 十一、Project Settings — 生成代码设置

菜单栏 **Project → Project Manager**

### Project 页签

| 参数 | 值 |
|------|-----|
| Project Name | `rc_drift_car` |
| Project Location | 选择你的工程根目录 |
| **Toolchain / IDE** | **MDK-ARM (Keil uVision5)** 或 **STM32CubeIDE** 按你习惯 |
| Use Default Firmware Location | ✅ |

### Code Generator 页签

| 参数 | 勾选 |
|------|------|
| Copy all used libraries into the project folder | ✅ |
| **Generate peripheral initialization as a pair of '.c/.h' files** | ✅ |
| Set all free pins as analog | ✅ (省电) |
| Enable Full Assert | (开发阶段建议 ✅，最终优化时可关) |

---

## 十二、生成代码 & 集成

1. 点击 **GENERATE CODE** 按钮
2. 等待生成完成 → 打开工程

3. 将我们已有的文件复制到工程中：

```
工程根目录/
├── Core/
│   ├── Inc/  ← 复制这6个文件进来:
│   │   ├── system_config.h    (替换或合并)
│   │   ├── mpu6050.h
│   │   ├── imu_filter.h
│   │   ├── pid.h
│   │   ├── vehicle_control.h
│   │   └── data_logger.h
│   └── Src/  ← 复制这5个文件进来:
│       ├── mpu6050.c
│       ├── imu_filter.c
│       ├── pid.c
│       ├── vehicle_control.c
│       └── data_logger.c
```

4. `main.c` 的处理方式：

   **不要直接覆盖 CubeMX 生成的 main.c！** 因为里面有 `System_Clock_Config()`、`MX_XXX_Init()` 等生成代码。

   而是：
   - 保留 CubeMX 的 `main.c`
   - 把我们 [main.c](Src/main.c) 中 **"===== 主循环 ====="及之后的部分**（`HAL_TIM_PWM_Start` 到 while(1) 内的 `Control_Loop_500Hz()`）复制过来
   - 把 **全局变量**（`control_loop_flag`, `mpu_raw`, `steering`, `esc`, `rc_channels` 等）复制到 CubeMX main.c 顶部
   - 把 **回调函数**（`HAL_TIM_IC_CaptureCallback`, `HAL_TIM_PeriodElapsedCallback`）复制到 CubeMX main.c 的用户代码区（`/* USER CODE BEGIN 4 */` 和 `/* USER CODE END 4 */` 之间）
   - 把 `Control_Loop_500Hz()` 函数体也放在 USER CODE 4 区

5. 编译 → 烧录 → 测试

---

## 附录 A：常见问题排查

### A1. MPU6050 初始化失败 (LED 快闪)

| 可能原因 | 检查方法 |
|----------|----------|
| I2C 接线反了 | 万用表测 PB6/PB7 与模块的连接 |
| 模块无上拉电阻 | 示波器看 SCL/SDA 是否有波形过缓 |
| MPU6050 AD0 电平不对 | 测量 AD0 引脚 (0x68 = GND, 0x69 = VCC) |
| CubeMX PLL 时钟配错 | 确认 SYSCLK=100MHz, APB1=50MHz |

### A2. 舵机不转 / 抖动

- 确认电调和舵机都已单独供电（不要用 STM32 的 3.3V 供电）
- 检查 TIM2 ARR=19999 是否生效
- 示波器量 PA0：应该看到 50Hz, 脉宽 500~2500us 的方波

### A3. RC 接收机数值不动

- 确认 TIM3_CH1 中断在 NVIC 已使能
- 确认 PPM 接收机信号线接 PA6
- 在 `HAL_TIM_IC_CaptureCallback` 中打断点观察

### A4. 控制循环不执行

- 确认 TIM4 NVIC 中断已使能
- 确认 `HAL_TIM_Base_Start_IT(&htim4)` 被调用
- 确认 TIM4 Channel1~4 全部 Disable（不分配引脚）
- 在 `control_loop_flag` 的 if 语句处打断点

---

## 附录 B：最小电路连接参考

```
STM32F411 (BlackPill)       外设
─────────────────────       ──────────────────
3.3V                    →   MPU6050 VCC
GND                     →   MPU6050 GND, ESC GND, Servo GND, 接收机 GND
PB6 (I2C1_SCL)          →   MPU6050 SCL
PB7 (I2C1_SDA)          →   MPU6050 SDA
PA0 (TIM2_CH1)          →   舵机信号线
PA1 (TIM2_CH2)          →   电调信号线
PA6 (TIM3_CH1)          →   RC接收机 PPM 输出
PA9 (USART1_TX)         →   USB-TTL模块 RX (接电脑)
GND                     →   USB-TTL模块 GND

独立供电:
2S/3S 锂电池            →   电调电源端
BEC/UBEC (5V输出)       →   舵机电源 + STM32 5V pin
```

> **绝对不要**用 STM32 板载 3.3V 给舵机或电调供电，电流不够且会烧板。
> 舵机需要 5-6V 单独供电（通过 BEC 降压），信号线（白/黄线）才接 STM32。
