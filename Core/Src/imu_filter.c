/**
 * imu_filter.c — 姿态解算实现
 *
 * 互补滤波原理:
 *   angle = α·(angle + gyro·dt) + (1-α)·accel_angle
 *
 *   陀螺仪：动态响应快，但积分会漂移
 *   加速度计：长期稳定，但振动噪声大、不能测Yaw
 *   → 互补滤波取两者之长
 *
 * Yaw角的特殊性：
 *   加速度计无法测量Yaw（绕重力方向旋转），所以Yaw纯靠陀螺仪积分。
 *   通过零速检测自动补偿陀螺仪零偏，减缓漂移速度。
 */

#include "imu_filter.h"
#include <math.h>

/* 弧度转角度 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

/* 零漂补偿模块全局状态 */
static IMU_ZeroDriftComp zd_comp;

/* ==================== 互补滤波器 ==================== */

void IMU_Filter_Init(float gyro_offset[3]) {
    zd_comp.gyro_offset[0] = gyro_offset[0];
    zd_comp.gyro_offset[1] = gyro_offset[1];
    zd_comp.gyro_offset[2] = gyro_offset[2];
    IMU_ZeroDrift_Init();
}

void IMU_Filter_Update(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt, IMU_Attitude *att) {
    float accel_pitch, accel_roll;

    /* 1. 去除陀螺仪零偏 */
    gx -= zd_comp.gyro_offset[0];
    gy -= zd_comp.gyro_offset[1];
    gz -= zd_comp.gyro_offset[2];

    /* 2. 从加速度计反算Pitch和Roll
     *    pitch = atan2(-ax, sqrt(ay² + az²))
     *    roll  = atan2( ay, az)
     */
    accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    accel_roll  = atan2f( ay, az) * RAD_TO_DEG;

    /* 3. 互补滤波: Pitch & Roll (融合加速度计+陀螺仪) */
    att->pitch = COMP_FILTER_ALPHA * (att->pitch + gx * dt)
               + (1.0f - COMP_FILTER_ALPHA) * accel_pitch;

    att->roll  = COMP_FILTER_ALPHA * (att->roll  + gy * dt)
               + (1.0f - COMP_FILTER_ALPHA) * accel_roll;

    /* 4. Yaw: 纯陀螺仪积分 + 零漂补偿 */
    IMU_ZeroDrift_Update(gz, dt);
    att->yaw    += gz * dt;
    att->gyro_z  = gz;
    att->gyro_y  = gy;
    att->gyro_x  = gx;

    /* Yaw角归一化到 [-180, 180] */
    while (att->yaw >  180.0f) att->yaw -= 360.0f;
    while (att->yaw < -180.0f) att->yaw += 360.0f;
}

/* ==================== 零漂自动补偿 ==================== */

void IMU_ZeroDrift_Init(void) {
    zd_comp.zero_rate_accum_time = 0.0f;
    zd_comp.zero_rate_samples    = 0;
    zd_comp.is_static            = 0;
}

void IMU_ZeroDrift_Update(float gz, float dt) {
    if (fabsf(gz) < ZERO_RATE_THRESHOLD) {
        zd_comp.zero_rate_accum_time += dt;
        zd_comp.zero_rate_samples++;
    } else {
        zd_comp.zero_rate_accum_time  = 0.0f;
        zd_comp.zero_rate_samples     = 0;
        zd_comp.is_static             = 0;
    }

    /* 持续静止超过阈值 → 将当前角速度视为零偏缓慢修正 */
    if (zd_comp.zero_rate_accum_time > ZERO_RATE_DURATION) {
        if (!zd_comp.is_static) {
            zd_comp.is_static = 1;
            /* 指数移动平均修正零偏: offset_new = 0.99*offset_old + 0.01*current */
            zd_comp.gyro_offset[2] = 0.99f * zd_comp.gyro_offset[2] + 0.01f * gz;
        }
    }
}
