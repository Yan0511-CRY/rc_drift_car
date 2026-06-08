/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* 用户自定义模块头文件 */
#include "system_config.h"      /* 系统参数配置（PID参数、PWM中值、滤波器系数等） */
#include "mpu6050.h"            /* MPU6050 六轴传感器驱动 */
#include "imu_filter.h"         /* IMU 姿态解算（互补滤波 / 四元数融合） */
#include "pid.h"                /* PID 控制器 */
#include "vehicle_control.h"    /* 车辆控制：舵机转向 + 电调油门 */
#include "data_logger.h"        /* 数据记录器：UART 日志 / VOFA+ JustFloat 协议输出 */
#include "oled.h"               /* SSD1306 OLED 128x64 显示驱动 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* ──────────────── 控制循环标志 ──────────────── */
static volatile uint8_t control_loop_flag = 0; /* TIM4 中断触发，主循环轮询，实现 500Hz 控制频率 */

/* ──────────────── IMU 数据变量 ──────────────── */
static MPU6050_RawData    mpu_raw;         /* 原始 ADC 值（加速度计 / 陀螺仪） */
static MPU6050_ScaledData mpu_scaled;      /* 换算后的物理量（g 和 °/s） */
static IMU_Attitude       imu_att;         /* 姿态数据（欧拉角 + 陀螺角速度） */
static float              gyro_offset[3];  /* 陀螺仪零点偏移校准值 */

/* ──────────────── 执行器控制 ──────────────── */
static Steering_Control   steering;        /* 舵机转向控制结构体 */
static ESC_Control        esc;             /* 电调（油门）控制结构体 */

/* ──────────────── 接收机 PPM 通道 ──────────────── */
static volatile uint32_t  rc_channels[6] = {
    RC_PWM_CENTER, RC_PWM_CENTER, RC_PWM_CENTER,
    RC_PWM_CENTER, RC_PWM_CENTER, RC_PWM_CENTER
};  /* 6 通道 RC PPM 信号脉宽（μs），初始化为中值 */

/* ──────────────── 计数器 ──────────────── */
static uint32_t log_counter  = 0;   /* 日志分频计数器 */
static uint32_t system_tick  = 0;   /* 系统节拍（每 2ms +1） */
static uint32_t oled_counter = 0;   /* OLED 刷新分频计数器 */

extern I2C_HandleTypeDef hi2c2;  /* 用户在 CubeMX 中配置 I2C2 (PB3-SCL, PB10-SDA) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

static void Control_Loop_500Hz(void);  /* 500Hz 实时控制循环：IMU读取 → 姿态解算 → 舵机/油门控制 → 日志/OLED */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  {
    uint8_t init_status;

    /* ── MPU6050 初始化 ── */
    init_status = MPU6050_Init(&hi2c1);
    if (init_status != 0) {
        /* 初始化失败 → 死循环闪灯报警 */
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            /* init_status 1 = I2C 通信失败（快闪 100ms），2 = 芯片 ID 不对（慢闪 500ms） */
            if (init_status == 1) {
                HAL_Delay(100);
            } else {
                HAL_Delay(500);
            }
        }
    }

    /* ── 陀螺仪零点校准 ── */
    MPU6050_CalibrateGyro(gyro_offset);   /* 静止采样 1000 次取平均 */
    IMU_Filter_Init(gyro_offset);         /* 互补滤波器初始化，写入零偏 */

    /* ── 舵机 & 电调控制初始化 ── */
    Steering_Init(&steering);             /* 舵机中值 & PID 参数初始化 */
    ESC_Init(&esc);                       /* 电调中值 & 安全范围初始化 */

    /* ── 数据记录器初始化 ── */
    Logger_Init(&huart1);                 /* 启动 UART DMA 发送 */

    /* ── PWM 输出启动 (TIM2: CH1=舵机, CH2=油门) ── */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    /* 舵机回中、油门归零（防止启动瞬间误动作） */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, THROTTLE_PWM_NEUTRAL);

    /* ── 控制周期中断启动 (TIM4: 500Hz) ── */
    HAL_TIM_Base_Start_IT(&htim4);

    /* ── DWT 周期计数器启用 (PWM 脉宽测量, 1μs分辨率) ── */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#if OLED_ENABLE
    /* ── OLED 启动画面 ── */
    OLED_Init(&hi2c2);
    OLED_Clear();
    OLED_SetCursor(0, 0);
    OLED_Print("RC Drift Car");
    OLED_SetCursor(0, 1);
    OLED_Print("OLED Ready...");
    OLED_Refresh();
#endif
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 轮询控制循环标志位（TIM4 中断置位 → 主循环执行 → 清标志） */
    if (control_loop_flag) {
        control_loop_flag = 0;
        system_tick++;               /* 系统节拍递增（每 2ms） */
        Control_Loop_500Hz();        /* 执行 500Hz 控制循环 */
    }
  /* USER CODE END 3 */
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ────────────────────────────────────────────────────────────────────────────
 *  GPIO EXTI 回调 — PWM 接收机脉宽测量
 *  ───────────────────────────────────────────────────────────────────────────
 *  HotRC F-06 输出独立 PWM 信号 (每通道单独一根线):
 *    - PA4 (EXTI4): CH1 转向, 下降沿测量脉宽
 *    - PA5 (EXTI5): CH3 油门, 下降沿测量脉宽
 *
 *  工作原理:
 *    - 上升沿: 记录 DWT->CYCCNT 时间戳 (1μs 精度)
 *    - 下降沿: 计算脉宽 = 当前时间 - 上升沿时间
 *    - 有效脉宽范围: 1000~2000μs (RC 标准)
 * ─────────────────────────────────────────────────────────────────────────── */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t steer_rising = 0, thr_rising = 0;
    uint32_t now = DWT->CYCCNT / 100;

    if (GPIO_Pin == RC_STEERING_PIN) {
        if (HAL_GPIO_ReadPin(RC_STEERING_PORT, RC_STEERING_PIN)) {
            steer_rising = now;
        } else if (steer_rising > 0) {
            uint32_t pw = now - steer_rising;
            if (pw >= RC_PWM_MIN && pw <= RC_PWM_MAX)
                rc_channels[RC_CH_STEERING] = pw;
        }
    }
    else if (GPIO_Pin == RC_THROTTLE_PIN) {
        if (HAL_GPIO_ReadPin(RC_THROTTLE_PORT, RC_THROTTLE_PIN)) {
            thr_rising = now;
        } else if (thr_rising > 0) {
            uint32_t pw = now - thr_rising;
            if (pw >= RC_PWM_MIN && pw <= RC_PWM_MAX)
                rc_channels[RC_CH_THROTTLE] = pw;
        }
    }
}

/* ────────────────────────────────────────────────────────────────────────────
 *  TIM4 周期中断回调 — 控制循环节拍发生器
 *  ───────────────────────────────────────────────────────────────────────────
 *  TIM4 ARR=1999, PSC=99 → 500Hz 中断频率
 *  该回调仅置位标志位, 实际控制逻辑在主循环中轮询执行
 * ─────────────────────────────────────────────────────────────────────────── */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        control_loop_flag = 1;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Control_Loop_500Hz — 核心实时控制循环
 *  ═══════════════════════════════════════════════════════════════════════════
 *  执行频率: TIM4 中断触发 → 主循环轮询 → 约 500Hz
 *  执行步骤:
 *    ① MPU6050 读取原始数据 (I2C 400kHz)
 *    ② 数据换算 (LSB → g / °/s)
 *    ③ IMU 互补滤波姿态解算 (欧拉角 + 陀螺角速度)
 *    ④ RC 接收机通道值解析 (舵机角度 / 油门量)
 *    ⑤ 舵机 PD 控制器更新 (目标角度 → 陀螺仪反馈 → PWM)
 *    ⑥ 更新 PWM 输出 (TIM2 CH1/CH2)
 *    ⑦ OLED 分频刷新 (约 10Hz)
 *    ⑧ 数据日志分频输出 (UART / VOFA+ 协议)
 * ─────────────────────────────────────────────────────────────────────────── */
static void Control_Loop_500Hz(void) {
    static uint32_t last_cycle = 0;
    uint32_t now_cycle = DWT->CYCCNT;
    float measured_dt = COMP_FILTER_DT;

    if (last_cycle != 0U) {
        uint32_t elapsed_cycles = now_cycle - last_cycle;
        measured_dt = (float)elapsed_cycles / (float)SystemCoreClock;

        if (measured_dt < 0.0005f) measured_dt = COMP_FILTER_DT;
        if (measured_dt > 0.0200f) measured_dt = COMP_FILTER_DT;
    }
    last_cycle = now_cycle;
    float dt = COMP_FILTER_DT;  /* 互补滤波时间常数 (秒) */

    /* ── ① 读取 MPU6050 原始数据 ── */
    if (MPU6050_ReadRaw(&mpu_raw) != 0) {
        return;  /* I2C 读取失败则跳过本次循环 */
    }
    /* ── ② 原始 ADC → 物理量换算 ── */
    MPU6050_ScaleData(&mpu_raw, &mpu_scaled);

    /* ── ③ RC辅助归零信号 → IMU零漂补偿 ── */
    {
        /* 将遥控器PWM转为归一化值 [-1, 1] 传给IMU滤波器用于零偏判定 */
        float thr_norm, str_norm;
        uint32_t rc_pwm;
        rc_pwm = rc_channels[RC_CH_THROTTLE];
        if (rc_pwm <= RC_PWM_CENTER + 20 && rc_pwm >= RC_PWM_CENTER - 20)
            thr_norm = 0.0f;
        else if (rc_pwm > RC_PWM_CENTER)
            thr_norm = (float)(rc_pwm - RC_PWM_CENTER) / (float)(RC_PWM_MAX - RC_PWM_CENTER);
        else
            thr_norm = (float)(rc_pwm - RC_PWM_CENTER) / (float)(RC_PWM_CENTER - RC_PWM_MIN);

        rc_pwm = rc_channels[RC_CH_STEERING];
        if (rc_pwm <= RC_PWM_CENTER + 20 && rc_pwm >= RC_PWM_CENTER - 20)
            str_norm = 0.0f;
        else if (rc_pwm > RC_PWM_CENTER)
            str_norm = (float)(rc_pwm - RC_PWM_CENTER) / (float)(RC_PWM_MAX - RC_PWM_CENTER);
        else
            str_norm = (float)(rc_pwm - RC_PWM_CENTER) / (float)(RC_PWM_CENTER - RC_PWM_MIN);

        IMU_SetRCInputs(thr_norm, str_norm);
        dt = measured_dt;
    }

    /* ── ④ IMU 互补滤波姿态融合 ── */
    IMU_Filter_Update(
        mpu_scaled.gx, mpu_scaled.gy, mpu_scaled.gz,   /* 陀螺角速度 (°/s) */
        mpu_scaled.ax, mpu_scaled.ay, mpu_scaled.az,   /* 加速度计 (g) */
        dt, &imu_att                                    /* 时间常数 & 输出姿态 */
    );

    /* ── ⑤ RC 通道值映射 ── */
    Steering_SetRC(&steering, rc_channels[RC_CH_STEERING]);  /* CH1 → 转向角度 (1000~2000μs) */
    ESC_SetThrottle(&esc,   rc_channels[RC_CH_THROTTLE]);    /* CH2 → 油门量 (1000~2000μs) */

    /* ── ⑥ 舵机 PD 控制器 ── */
    Steering_Update(&steering, imu_att.gyro_z, dt);

    /* ── ⑦ 更新 PWM 输出 ── */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                          Steering_GetPWM(&steering));  /* 舵机 PWM */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                          ESC_GetPWM(&esc));             /* 油门 PWM */

#if OLED_ENABLE
    /* ── ⑦ OLED 分频刷新 (约 10Hz) ── */
    oled_counter++;
    if (oled_counter >= OLED_REFRESH_DIV) {
        oled_counter = 0;

        /* 第0行: Yaw(偏航) / Pitch(俯仰) */
        OLED_SetCursor(0, 0);
        OLED_Print("Y:");
        OLED_PrintFloat(imu_att.yaw, 4, 1);
        OLED_Print(" P:");
        OLED_PrintFloat(imu_att.pitch, 4, 1);

        /* 第1行: Roll(翻滚) / GyroZ(Z轴角速度) */
        OLED_SetCursor(0, 1);
        OLED_Print("R:");
        OLED_PrintFloat(imu_att.roll, 4, 1);
        OLED_Print(" GZ:");
        OLED_PrintFloat(imu_att.gyro_z, 4, 1);

        /* 第2行: GyroX / GyroY (X/Y轴角速度) */
        OLED_SetCursor(0, 2);
        OLED_Print("GX:");
        OLED_PrintFloat(imu_att.gyro_x, 4, 1);
        OLED_Print(" GY:");
        OLED_PrintFloat(imu_att.gyro_y, 4, 1);

        /* 第3行: 舵机 PWM / 油门 PWM */
        OLED_SetCursor(0, 3);
        OLED_Print("ST:");
        OLED_PrintInt(Steering_GetPWM(&steering));
        OLED_Print(" TH:");
        OLED_PrintInt(ESC_GetPWM(&esc));

        OLED_Refresh();  /* 将缓冲区写入 SSD1306 显存 */
    }
#endif

    /* ── ⑧ 数据日志分频输出 ── */
    log_counter++;
    if (log_counter >= LOG_DIVIDER) {
        log_counter = 0;

#if VOFA_OUTPUT_ENABLE
        /* VOFA+ JustFloat 协议: 10 个 float (40 字节) + 帧尾 */
        Logger_SendVOFA(&imu_att);
#else
        /* ASCII 文本帧格式: 8 字段, 逗号分隔, 换行结束 */
        LogFrame frame;
        frame.tick           = system_tick;               /* 系统节拍计数 */
        frame.target_angle   = steering.rc_angle;         /* RC 目标转向角度 */
        frame.current_yaw    = imu_att.yaw;               /* IMU 当前偏航角 */
        frame.gyro_z         = imu_att.gyro_z;            /* Z 轴角速度 */
        frame.imu_alpha      = imu_att.alpha;             /* IMU 动态融合权重 */
        frame.gyro_z_offset  = imu_att.gyro_z_offset;     /* Z 轴陀螺零偏估计 */
        frame.zero_allowed   = imu_att.zero_allowed;      /* 零偏更新允许状态 */
        frame.imu_static     = imu_att.is_static;         /* 零偏更新状态 */
        frame.pd_output      = steering.servo_angle_cmd;  /* PD 控制器输出 */
        frame.servo_pwm      = Steering_GetPWM(&steering);/* 最终舵机 PWM */
        frame.throttle_input = esc.throttle_output;       /* 油门输出值 */
        frame.esc_pwm        = ESC_GetPWM(&esc);          /* 最终电调 PWM */

        Logger_Log(&frame);
#endif
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 用户可在此添加错误上报逻辑（如 LED 闪烁、日志输出等） */
  __disable_irq();   /* 关闭所有中断 */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
