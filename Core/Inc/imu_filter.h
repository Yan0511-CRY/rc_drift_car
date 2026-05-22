/**
 * imu_filter.h — 姿态解算模块
 *
 * 实现：
 *  - 互补滤波 (加速度计的低频精度 + 陀螺仪的高频响应)
 *  - Yaw角零漂自动补偿 (零速检测 → 自动更新offset)
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
} IMU_Attitude;

/* ==================== 零漂补偿状态机 ==================== */
typedef struct {
    float gyro_offset[3];         /* 陀螺仪零偏 (dps) */
    float zero_rate_accum_time;   /* 零速累计时间 (秒) */
    uint32_t zero_rate_samples;   /* 零速采样计数 */
    uint8_t is_static;            /* 当前是否处于静止状态 */
} IMU_ZeroDriftComp;

/* ==================== API ==================== */
void IMU_Filter_Init(float gyro_offset[3]);
void IMU_Filter_Update(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt, IMU_Attitude *att);
void IMU_ZeroDrift_Init(void);
void IMU_ZeroDrift_Update(float gz, float dt);

#endif /* IMU_FILTER_H */
