/**
 * data_logger.h — 串口数据记录器
 *
 * 通过UART以CSV格式实时输出车辆状态数据，可直接用串口助手保存为.csv
 * MATLAB/Python 读取后绘图分析。
 *
 * CSV列：时间戳, 目标角度, 当前Yaw, 陀螺仪Z, PD输出, 舵机PWM, 油门输入, 油门PWM
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "system_config.h"
#include "vehicle_control.h"
#include "imu_filter.h"

typedef struct {
    uint32_t tick;              /* 系统tick (ms) */
    float    target_angle;      /* 目标转向角 */
    float    current_yaw;       /* IMU解算Yaw */
    float    gyro_z;            /* 陀螺仪Z轴角速度 */
    float    pd_output;         /* PD控制器输出 */
    uint32_t servo_pwm;         /* 舵机输出PWM */
    float    throttle_input;    /* 油门输入归一化值 */
    uint32_t esc_pwm;           /* 电调输出PWM */
} LogFrame;

void Logger_Init(UART_HandleTypeDef *huart);
void Logger_Log(const LogFrame *frame);
void Logger_SendVOFA(const IMU_Attitude *att);

#endif /* DATA_LOGGER_H */
