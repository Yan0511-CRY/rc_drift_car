/**
 * vehicle_control.h — 车辆执行层控制
 *
 * 转向 (陀螺仪辅助漂移):
 *   Mode 0 (增益反打): servo = RC_angle - GAIN × gyro_z
 *   Mode 1 (角速度追踪): servo = PID(target_yaw_rate, gyro_z)
 *
 * 油门: RC摇杆 → 指数曲线映射 → 电调PWM
 */

#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include "system_config.h"
#include "pid.h"

/* ==================== 转向控制 ==================== */
typedef struct {
    /* 遥控器输入 → 目标 */
    float rc_angle;             /* 遥控器映射的舵机角度 (度) */
    float target_yaw_rate;      /* (Mode 1) 目标偏航角速度 (dps) */

    /* 陀螺仪反馈 */
    float gyro_z_filtered;      /* 低通滤波后的陀螺仪角速度 */

    /* 输出 */
    float servo_angle_cmd;      /* 最终舵机角度指令 (度) */
    uint32_t servo_pwm;         /* 舵机PWM脉宽 (us) */

    /* PID (仅 Mode 1 使用) */
    PID_Controller pid;
} Steering_Control;

void Steering_Init(Steering_Control *steer);
void Steering_SetRC(Steering_Control *steer, uint32_t rc_pwm);
void Steering_Update(Steering_Control *steer, float imu_gyro_z, float dt);
uint32_t Steering_GetPWM(const Steering_Control *steer);

/* ==================== 油门控制 ==================== */
typedef struct {
    float throttle_input;       /* 遥控器输入 [-1.0, 1.0] */
    float throttle_output;      /* 曲线映射后 [-1.0, 1.0] */
    uint32_t esc_pwm;           /* 输出给电调的PWM脉宽 (us) */
} ESC_Control;

void ESC_Init(ESC_Control *esc);
void ESC_SetThrottle(ESC_Control *esc, uint32_t rc_pwm);
uint32_t ESC_GetPWM(const ESC_Control *esc);

float ThrottleCurve(float input, float expo);

#endif /* VEHICLE_CONTROL_H */
