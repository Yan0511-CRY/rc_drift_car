/**
 * system_config.h — 漂移车全局配置
 *
 * 所有可调参数集中在此，方便调参与移植。
 *
 * 硬件平台：STM32F411CEU6 (BlackPill)
 * 传感器：  MPU6050 (I2C1, PB6-SCL / PB7-SDA)
 * 舵机：    TIM2_CH1 (PA0), 50Hz
 * 电调：    好盈1060有刷电调, TIM2_CH2 (PA1), 50Hz
 * 电机：    540/550 有刷直流电机
 * 遥控器：  5通道PPM接收机 → 定时器输入捕获 (PA6, TIM3_CH1)
 * 串口日志：USART1 (PA9-TX / PA10-RX), 115200bps → 上位机/MATLAB
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ==================== 系统时钟 ==================== */
#define CONTROL_LOOP_FREQ_HZ   500U   // 控制主循环频率 500Hz
#define CONTROL_LOOP_PERIOD_US 2000U  // = 1e6 / 500

/* ==================== MPU6050 参数 ==================== */
#define MPU6050_I2C_ADDR    0x68    // AD0引脚接GND
#define MPU6050_SAMPLE_RATE 1000U   // 陀螺仪采样率 1kHz
#define MPU6050_GYRO_FS     MPU6050_GYRO_FS_1000  // ±1000dps
#define MPU6050_ACCEL_FS    MPU6050_ACCEL_FS_4G   // ±4g

/* 陀螺仪增益校准系数: GYRO_SCALE_FACTOR = 360° / 实测yaw度数
 * 默认1.0，校准后根据实测值修改。标称32.8 LSB/dps与实际芯片可能有偏差 */
#define GYRO_SCALE_FACTOR   1.20f    /* TODO: 精确校准后更新此值 */

/* 启动时零点校准采样次数 */
#define GYRO_CALIB_SAMPLES  500U
/* 零速检测阈值 (dps)，低于此值认为静止，自动更新offset */
#define ZERO_RATE_THRESHOLD 3.0f
/* 零速持续时间 (秒)，持续此时间才触发自动归零 */
#define ZERO_RATE_DURATION  3.0f

/* ==================== 互补滤波器参数 ==================== */
#define COMP_FILTER_ALPHA   0.96f   // 典型的 0.96~0.98，越大越信任陀螺仪
#define COMP_FILTER_DT      0.002f  // = 1/500Hz
#define IMU_ALPHA_BASE      0.96f
#define IMU_ALPHA_MAX       0.998f
#define IMU_ACCEL_NORM_IDLE 1.0f
#define IMU_ACCEL_NORM_AGGRESSIVE 2.5f

/* ==================== IMU 零偏在线估计 ==================== */
#define IMU_KALMAN_P_INIT   1.0f
#define IMU_KALMAN_Q        1e-6f
#define IMU_KALMAN_R        1e-2f
#define IMU_KALMAN_RC_NEAR_R_SCALE 10.0f
#define IMU_RC_CENTER_THRESHOLD      0.05f
#define IMU_RC_NEAR_CENTER_THRESHOLD 0.15f
#define IMU_RC_CENTER_GYRO_MULT      2.0f
#define IMU_RC_NEAR_GYRO_MULT        4.0f
/* Online yaw-bias updates must be stricter than normal zero-rate detection.
 * RC stick center is only a hint; physical IMU still has to look quiet. */
#define IMU_ZERO_UPDATE_RATE_THRESHOLD 1.0f
#define IMU_ZERO_ACCEL_TOLERANCE       0.12f

/* ==================== 陀螺仪转向控制模式 ==================== */
/*
 *  模式 0 — GYRO_GAIN (陀螺仪增益反打, 推荐默认):
 *    舵机 = 遥控角度 - GAIN × 陀螺仪角速度
 *    遥控器直驱舵机，陀螺仪检测到甩尾 → 自动叠加反打修正。
 *    这是RC漂移陀螺仪的标准工作模式。
 *
 *  模式 1 — YAW_RATE_TRACKING (偏航角速度追踪, 进阶):
 *    遥控器 = 目标偏航角速度
 *    PID闭环追踪，error = 目标角速度 - 实测角速度
 *    遥控回中 = 车必须停止旋转 (更激进的闭环控制)
 */
#define GYRO_CONTROL_MODE   0       // 0=增益反打, 1=角速度追踪

/* ---- 模式 0 参数: 增益反打 ---- */
#define GYRO_GAIN           0.10f   // 陀螺仪增益 (度舵机/每dps角速度)
                                    // 越大反打越猛, 0=关闭陀螺仪辅助
                                    // 典型: 0.05(柔和)~0.15(激进)
#define GYRO_LPF_ALPHA      0.80f   // 陀螺仪角速度低通 (0~1, 越大越平滑)

/* ---- 模式 1 参数: 角速度追踪 ---- */
#define YAW_RATE_MAX        300.0f  // 遥控器满舵对应的目标角速度 (dps)
#define YAW_TRACK_KP        0.30f   // P: 角度误差→舵机修正 (度舵机/每dps误差)
#define YAW_TRACK_KI        0.02f   // I: 积分项 (用于消除稳态误差)
#define YAW_TRACK_KD        0.08f   // D: 微分项 (角加速度阻尼)
#define YAW_TRACK_I_MAX     15.0f   // 积分限幅 (防止windup)

/* ---- 通用舵机参数 ---- */
#define SERVO_PWM_MIN       500     // 舵机最小脉宽 us
#define SERVO_PWM_MAX       2500    // 舵机最大脉宽 us
#define SERVO_PWM_CENTER    1500    // 舵机中位
#define STEERING_ANGLE_MAX  35.0f   // 最大机械转向角 (度)

/* ==================== 油门控制 (有刷电调 好盈1060) ==================== */
/* 好盈1060 PWM映射:
 *   1500us = 停止/中位
 *   1500~2000us = 正转 (前进, 线性递增)
 *   1000~1500us = 反转 (倒车/刹车, 线性递增)
 * 首次使用需校准: 开机时给2000us → 听到"滴滴" → 给1500us → 完成 */
#define THROTTLE_PWM_MIN     1000    // 最大倒车
#define THROTTLE_PWM_MAX     2000    // 最大前进
#define THROTTLE_PWM_NEUTRAL 1500    // 停止中位
#define THROTTLE_DEADBAND    30      // 中位死区 us (1060版本±30us)
/* 油门曲线: 指数映射系数 (0=线性, 越大低油门越精细) */
#define THROTTLE_EXPO        0.3f

/* ==================== 遥控器通道映射 ==================== */
#define RC_CH_STEERING      0       // 遥控器CH1 → 转向
#define RC_CH_THROTTLE      2       // 遥控器CH3 → 油门
#define RC_PWM_MIN          1000
#define RC_PWM_MAX          2000
#define RC_PWM_CENTER       1500
#define RC_DEADBAND         20      // 摇杆死区 us

/* PWM 接收机引脚 (HotRC F-06, 独立PWM输出) */
/* CubeMX配置: PA4→GPIO_EXTI4, PA5→GPIO_EXTI5, 双沿触发, 上拉 */
#define RC_STEERING_PORT    GPIOA
#define RC_STEERING_PIN     GPIO_PIN_4
#define RC_THROTTLE_PORT    GPIOA
#define RC_THROTTLE_PIN     GPIO_PIN_5

/* ==================== 数据日志 ==================== */
#define LOG_UART            huart1
#define LOG_BAUDRATE        115200
/* 每 N 次控制循环输出一行日志 (1=每次, 10=每10次) */
#define LOG_DIVIDER         5       // 500Hz / 5 = 100Hz日志输出

/* VOFA+ 上位机 JustFloat 输出开关 (启用则关闭文本日志) */
#define VOFA_OUTPUT_ENABLE  1       // 1=VOFA+陀螺仪3D可视化, 0=文本日志

/* ==================== OLED 显示屏 (SSD1306, I2C2) ==================== */
/* 引脚: PB3-SCL / PB10-SDA (CubeMX中配置为I2C2)
 * 分辨率: 128×64, 驱动芯片: SSD1306
 * 0.96寸蓝色/白色OLED通用模块 */
#define OLED_I2C            hi2c2           /* 用户CubeMX生成的外设句柄 */
#define OLED_REFRESH_DIV    50              /* 每 N 次控制循环刷新一次屏幕 (500/50=10Hz) */
#define OLED_ENABLE         1               /* 1=启用OLED, 0=禁用 */

/* ==================== 引脚定义 (按实际接线修改) ==================== */
#define SERVO_TIM           htim2
#define SERVO_CHANNEL       TIM_CHANNEL_1
#define ESC_TIM             htim2
#define ESC_CHANNEL         TIM_CHANNEL_2
#define RC_TIM              htim3
#define RC_CHANNEL          TIM_CHANNEL_1

#endif /* SYSTEM_CONFIG_H */
