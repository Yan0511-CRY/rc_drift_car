/**
 * data_logger.h — 串口/蓝牙数据记录器 (非阻塞发送)
 *
 * 支持两种输出模式 (通过 VOFA_OUTPUT_ENABLE 选择):
 *   1. VOFA+ JustFloat: 10 通道浮点二进制帧，适配 VOFA+ 上位机 3D 陀螺仪
 *   2. CSV 文本:        14 字段逗号分隔，适配串口助手 / Python 保存
 *
 * 发送机制:
 *   环形缓冲区 + TXE 中断逐字节发送，主循环零等待。
 *   若缓冲区满则丢帧，绝不阻塞 500Hz 控制环。
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "system_config.h"
#include "vehicle_control.h"
#include "imu_filter.h"

typedef struct {
    uint32_t tick;              /* 系统tick (控制循环计数) */
    float    target_angle;      /* 目标转向角 (deg) */
    float    current_yaw;       /* IMU解算Yaw (deg) */
    float    gyro_z;            /* 陀螺仪Z轴角速度 (dps) */
    float    imu_alpha;         /* IMU动态互补滤波权重 */
    float    gyro_z_offset;     /* Z轴陀螺零偏估计 (dps) */
    uint8_t  zero_allowed;      /* 当前是否允许零偏更新 */
    uint8_t  imu_static;        /* 当前是否已进入零偏更新状态 */
    float    pd_output;         /* PD控制器输出 (deg) */
    uint32_t servo_pwm;         /* 舵机输出PWM (us) */
    float    throttle_input;    /* 油门输入归一化值 */
    uint32_t esc_pwm;           /* 电调输出PWM (us) */
} LogFrame;

/* ==================== API ==================== */

/** 初始化日志模块，保存 UART 句柄，清空缓冲区 */
void Logger_Init(UART_HandleTypeDef *huart);

/** VOFA+ JustFloat 二进制帧 (10 float + 帧尾 = 44 bytes) */
void Logger_SendVOFA(const IMU_Attitude *att);

/** CSV 文本帧 (14 字段: time_ms, 姿态, 陀螺, 加速, RC, PWM) */
void Logger_SendCSV(const LogFrame *frame, const IMU_Attitude *att);

/** USART1 TXE 中断处理 (由 stm32f4xx_it.c 的 USART1_IRQHandler 调用) */
void Logger_UART_IRQHandler(void);

#endif /* DATA_LOGGER_H */
