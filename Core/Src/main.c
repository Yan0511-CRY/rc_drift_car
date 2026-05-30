/**
 * main.c — RC漂移车主控程序
 *
 * 架构概览:
 * ┌─────────────────────────────────────────────────────┐
 * │  TIM4_ISR (500Hz)                                   │
 * │    ↓ set flag                                        │
 * │  main loop:                                         │
 * │    ① 读MPU6050 → ② 互补滤波 → ③ 读RC接收机          │
 * │    → ④ PD转向控制 → ⑤ 油门曲线 → ⑥ 输出PWM          │
 * │    → ⑦ 数据日志 (每LOG_DIVIDER次)                   │
 * └─────────────────────────────────────────────────────┘
 *
 * CubeMX 关键配置 (STM32F411CEU6):
 *  - I2C1:  PB6(SCL) / PB7(SDA), Fast Mode 400kHz
 *  - TIM2:  CH1(PA0) 舵机PWM, CH2(PA1) 电调PWM, 50Hz
 *  - TIM3:  CH1(PA6) PPM输入捕获
 *  - TIM4:  通用定时器, 2ms周期 (500Hz控制循环, 无输出引脚)
 *  - USART1: PA9(TX) / PA10(RX), 115200bps
 *  - GPIO:  PC13(LED) 心跳指示
 */
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "system_config.h"
#include "mpu6050.h"
#include "imu_filter.h"
#include "pid.h"
#include "vehicle_control.h"
#include "data_logger.h"

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

/* USER CODE BEGIN PV */

static volatile uint8_t control_loop_flag = 0;

static MPU6050_RawData    mpu_raw;
static MPU6050_ScaledData mpu_scaled;
static IMU_Attitude       imu_att;
static float              gyro_offset[3];

static Steering_Control   steering;
static ESC_Control        esc;

static volatile uint32_t  rc_channels[6] = {
    RC_PWM_CENTER, RC_PWM_CENTER, RC_PWM_CENTER,
    RC_PWM_CENTER, RC_PWM_CENTER, RC_PWM_CENTER
};

static uint32_t log_counter  = 0;
static uint32_t system_tick  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void Control_Loop_500Hz(void);

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  {
    uint8_t init_status;

    init_status = MPU6050_Init(&hi2c1);
    if (init_status != 0) {
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            /* init_status 1 = I2C不通(快闪100ms), 2 = 芯片ID不对(慢闪500ms) */
            if (init_status == 1) {
                HAL_Delay(100);
            } else {
                HAL_Delay(500);
            }
        }
    }

    MPU6050_CalibrateGyro(gyro_offset);
    IMU_Filter_Init(gyro_offset);

    Steering_Init(&steering);
    ESC_Init(&esc);

    Logger_Init(&huart1);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_PWM_CENTER);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, THROTTLE_PWM_NEUTRAL);

    HAL_TIM_Base_Start_IT(&htim4);

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (control_loop_flag) {
        control_loop_flag = 0;
        system_tick++;
        Control_Loop_500Hz();
    }
    /* USER CODE END 3 */
  }
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

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        static uint8_t  ppm_channel = 0;
        static uint32_t last_capture = 0;
        uint32_t capture, pulse_width;

        capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (capture > last_capture) {
            pulse_width = capture - last_capture;
        } else {
            pulse_width = (0xFFFF - last_capture) + capture + 1;
        }
        last_capture = capture;

        if (pulse_width > 4000) {
            ppm_channel = 0;
            return;
        }

        if (ppm_channel < 6) {
            rc_channels[ppm_channel] = pulse_width;
            ppm_channel++;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        control_loop_flag = 1;
    }
}

static void Control_Loop_500Hz(void) {
    float dt = COMP_FILTER_DT;

    if (MPU6050_ReadRaw(&mpu_raw) != 0) {
        return;
    }
    MPU6050_ScaleData(&mpu_raw, &mpu_scaled);

    IMU_Filter_Update(
        mpu_scaled.gx, mpu_scaled.gy, mpu_scaled.gz,
        mpu_scaled.ax, mpu_scaled.ay, mpu_scaled.az,
        dt, &imu_att
    );

    Steering_SetRC(&steering, rc_channels[RC_CH_STEERING]);
    ESC_SetThrottle(&esc,   rc_channels[RC_CH_THROTTLE]);

    Steering_Update(&steering, imu_att.gyro_z, dt);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                          Steering_GetPWM(&steering));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                          ESC_GetPWM(&esc));

    log_counter++;
    if (log_counter >= LOG_DIVIDER) {
        log_counter = 0;

#if VOFA_OUTPUT_ENABLE
        Logger_SendVOFA(&imu_att);
#else
        LogFrame frame;
        frame.tick           = system_tick;
        frame.target_angle   = steering.rc_angle;
        frame.current_yaw    = imu_att.yaw;
        frame.gyro_z         = imu_att.gyro_z;
        frame.pd_output      = steering.servo_angle_cmd;
        frame.servo_pwm      = Steering_GetPWM(&steering);
        frame.throttle_input = esc.throttle_output;
        frame.esc_pwm        = ESC_GetPWM(&esc);

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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
