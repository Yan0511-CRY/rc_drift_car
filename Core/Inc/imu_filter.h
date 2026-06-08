/**
 * imu_filter.h — 姿态解算模块
 *
 * 实现：
 *  - 互补滤波 (加速度计的低频精度 + 陀螺仪的高频响应)
 *  - 动态α：根据加速度模值自动调整融合权重，高速时更信任陀螺仪
 *  - Yaw角零漂自动补偿 (遥控器辅助 + 1D卡尔曼 → 更准的零偏估计)
 *  - 可选：卡尔曼滤波骨架 (答辩加分项，注释中标明替换点)
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "system_config.h"

/* ==================== 姿态数据结构 ==================== */
typedef struct {
    float yaw;          /* 偏航角 (deg)，互补滤波输出 */
    float pitch;        /* 俯仰角 (deg) */
    float roll;         /* 滚转角 (deg) */
    float gyro_z;       /* Z轴角速度 (dps)，已去除零偏 */
    float gyro_y;       /* Y轴角速度 (dps) */
    float gyro_x;       /* X轴角速度 (dps) */
    float alpha;        /* 当前动态互补滤波权重 */
    float gyro_z_offset;/* 当前Z轴陀螺零偏估计 (dps) */
    uint8_t zero_allowed; /* 当前是否允许零偏更新 */
    uint8_t is_static;    /* 当前是否已进入零偏更新状态 */
} IMU_Attitude;

/* ==================== 1D卡尔曼零偏估计器 ==================== */
typedef struct {
    float offset;       /* 陀螺仪零偏估计值 (dps) */
    float P;            /* 估计协方差 */
    float Q;            /* 过程噪声 (零偏变化快慢) */
    float R;            /* 观测噪声 (静止时陀螺读数的噪声) */
} IMU_Kalman1D;

/* ==================== 零漂补偿状态机 ==================== */
typedef struct {
    IMU_Kalman1D kalman;              /* 1D卡尔曼零偏估计器 */
    float gyro_offset[3];             /* 三轴陀螺仪零偏 (dps) */
    float zero_rate_accum_time;       /* 零速累计时间 (秒) */
    uint32_t zero_rate_samples;       /* 零速采样计数 */
    uint8_t is_static;                /* 当前是否处于静止状态 */
} IMU_ZeroDriftComp;

/* ==================== RC辅助归零条件 ==================== */
/* 在 main.c 的循环中调用: 传入油门和舵机的标幺值 [-1, 1] */
void IMU_SetRCInputs(float throttle_norm, float steering_norm);

/* ==================== API ==================== */
void IMU_Filter_Init(float gyro_offset[3]);
void IMU_Filter_Update(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt, IMU_Attitude *att);
void IMU_ZeroDrift_Init(void);
void IMU_ZeroDrift_Update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt);

#endif /* IMU_FILTER_H */
