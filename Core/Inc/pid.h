/**
 * pid.h — PD 控制器结构体
 *
 * 在 Mode 1 (角速度追踪) 中用于偏航角速度的闭环控制。
 * D项直接使用陀螺仪角速度而非error差分，避免噪声放大。
 *
 * Mode 0 (增益反打) 不使用此PID，直接用 GYRO_GAIN 公式。
 */

#ifndef PID_H
#define PID_H

#include "system_config.h"

/* ==================== PD控制器结构体 ==================== */
typedef struct {
    float kp;             /* 比例增益 */
    float kd;             /* 微分增益 (作用于角速度) */
    float setpoint;       /* 目标值 */
    float output;         /* 控制器输出 */
    float output_min;     /* 输出下限 */
    float output_max;     /* 输出上限 */

    /* D项低通滤波 */
    float d_lpf_alpha;    /* 滤波系数 (0~1) */
    float d_filtered;     /* 滤波后的D项 */
} PID_Controller;

/* ==================== API ==================== */
void PID_Init(PID_Controller *pid,
              float kp, float kd,
              float out_min, float out_max,
              float lpf_alpha);
float PID_Update(PID_Controller *pid, float error, float angular_velocity);
void  PID_Reset(PID_Controller *pid);

#endif /* PID_H */
