# TIM3 / TIM4 / I2C1 / USART1 — CubeMX 逐框配置

> 按下面顺序填，所有填入的数字下面有解释。

---

## TIM3 — RC 接收机 PPM 输入捕获

**作用**：读遥控器信号，1μs 精度测量 PPM 脉宽

### 1. 点开 TIM3

左边 **Pinout & Configuration** → **Timers** → 点 **TIM3**

### 2. 先看 Pinout 图

TIM3 被分配到 PA6。你需要在右边 Pinout 图里找到 **PA6**，点它 → 下拉选 **TIM3_CH1**。如果 PA6 默认已经是 TIM3_CH1，看颜色是亮的就行。

### 3. Mode 页签（Channel 配置）

你需要把 Channel 1 从 Disable 设成输入捕获模式。有一步容易漏：先点 Channel1 旁边的下拉框，找 **Input Capture direct mode**。

你点开 TIM3 后，会先看到一个 Channel 表格。把以下框填好：

**Channel1** 这一行：

| 列标题 | 你选的值 |
|--------|---------|
| Mode | 下拉 → **Input Capture direct mode** |

注意：Channel2/3/4 保持 **Disable**，不用动。

然后同一个表格右边有一列叫 **Polarity Selection**，这一列默认可能在下面看不到——往右拖一下表单。选择：

| 这个参数 | 值 |
|----------|-----|
| Polarity Selection | **Rising Edge** |

### 4. Configuration 页签（定时器参数）

切到 **Configuration** 页签（和 Mode 并列）：

| 框 | 填什么 |
|----|--------|
| **Prescaler (PSC)** | `99` |
| **Counter Mode** | Up（默认） |
| **Counter Period (ARR)** | `65535` |
| Auto-reload preload | Enable（默认就行） |
| Internal Clock Division | No Division（默认） |

表格下方可能有 **Input Filter** 项，找到 Channel1 的 **Input Filter** 填 `8`（轻度数字去抖）。

### 5. NVIC Settings 页签（重要，容易漏）

切到 **NVIC Settings** 页签。

看到一行 **TIM3 global interrupt**，默认 **Enabled 列的方框是空的**——**勾上它**。

勾完后 Preemption Priority 可以填 `2`。

> 不勾这个中断，`HAL_TIM_IC_CaptureCallback` 永远不会被调用，遥控器信号读不到。

---

## TIM4 — 500Hz 控制循环定时器

**作用**：每 2ms 触发一次，通知主循环执行控制算法

> F411 没有 TIM6/TIM7 基本定时器，用 TIM4 代替。TIM4 有输出通道但我们不去配置——只开它的周期中断。

### 1. 点开 TIM4

左边 **Pinout & Configuration** → **Timers** → 点 **TIM4**

TIM4 点开后，**不要在 Pinout 图上给任何引脚分配 TIM4 功能**——我们只要它的中断，不需要引脚输出。

### 2. Mode 页签

TIM4 和 TIM2/TIM3 一样会先显示 Channel 表格。确认 **Channel1~4 全部保持 Disable**（默认就是 Disable，如果看到某个被激活了下拉改为 Disable）。

### 3. Configuration 页签

切到 **Configuration** 页签：

| 框 | 填什么 |
|----|--------|
| **Prescaler (PSC)** | `99` |
| **Counter Period (ARR)** | `1999` |
| Auto-reload preload | Enable |

其余参数用默认值即可（Counter Mode = Up）。

### 4. NVIC Settings 页签

切到 **NVIC Settings**。

看到 **TIM4 global interrupt** 这一行——**勾上**。

Preemption Priority 填 `3`（比 TIM3 的 2 更低，意思是遥控器信号优先级高于控制循环）。

> 不勾这个中断，500Hz 控制循环永远不会跑。

---

## I2C1 — MPU6050 传感器

**作用**：和陀螺仪通信

### 1. Pinout

找到 **PB6** → 点它 → 下拉选 **I2C1_SCL**  
找到 **PB7** → 点它 → 下拉选 **I2C1_SDA**

### 2. Configuration

左边 → **Connectivity** → 点 **I2C1**

切 **Configuration** 页签：

| 框 | 填什么 |
|----|--------|
| **I2C Speed Mode** | **Fast Mode** |
| I2C Speed Frequency | `400`（MHz 单位） |

其余全部默认，不用改。

### 3. NVIC

I2C 的中断不需手动勾——HAL 库自己管理。

---

## USART1 — 串口数据日志

**作用**：把车辆状态以 CSV 格式发到电脑

### 1. Pinout

找到 **PA9** → 点它 → 下拉选 **USART1_TX**  
找到 **PA10** → 点它 → 下拉选 **USART1_RX**

### 2. Configuration

左边 → **Connectivity** → 点 **USART1**

| 框 | 填什么 |
|----|--------|
| **Baud Rate** | `921600` |
| Word Length | 8 Bits（默认） |
| Parity | None（默认） |
| Stop Bits | 1（默认） |
| **Hardware Flow Control** | **Disable**（默认是 Disable 就不用动） |
| Over Sampling | 16 Samples（默认） |

### 3. NVIC

不勾（当前用阻塞发送，不需要中断。以后用 DMA 再改）。

---

## 参数数字验证速查

| TIMx | PSC | ARR | 结果 |
|------|-----|-----|------|
| TIM2 | 99 | 19999 | **50Hz** PWM (1μs分辨率) |
| TIM3 | 99 | 65535 | **1μs 分辨率** 输入捕获 |
| TIM4 | 99 | 1999 | **500Hz** 控制循环 (2000μs周期) |

统一公式：`100MHz ÷ (PSC+1) ÷ (ARR+1)` = 你要的频率

TIM2: `100M ÷ 100 ÷ 20000 = 50Hz`  
TIM4: `100M ÷ 100 ÷ 2000 = 500Hz`
