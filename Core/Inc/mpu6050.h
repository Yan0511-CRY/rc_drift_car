/**
 * mpu6050.h — MPU6050 六轴传感器 I2C 驱动 (HAL库)
 *
 * 功能：
 *  - I2C 寄存器读写封装
 *  - 传感器初始化 (量程/采样率/低通滤波)
 *  - 原始数据读取 (加速度+角速度)
 *  - 启动零点自动校准
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "system_config.h"

/* ==================== MPU6050 寄存器地址 ==================== */
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT1    0x6B
#define MPU6050_REG_WHO_AM_I     0x75

/* 量程枚举 (FS_SEL 在 bit[4:3], 需左移3位写入寄存器) */
#define MPU6050_GYRO_FS_250    0x00   /* FS_SEL=0 << 3 */
#define MPU6050_GYRO_FS_500    0x08   /* FS_SEL=1 << 3 */
#define MPU6050_GYRO_FS_1000   0x10   /* FS_SEL=2 << 3 */
#define MPU6050_GYRO_FS_2000   0x18   /* FS_SEL=3 << 3 */

#define MPU6050_ACCEL_FS_2G    0x00   /* AFS_SEL=0 << 3 */
#define MPU6050_ACCEL_FS_4G    0x08   /* AFS_SEL=1 << 3 */
#define MPU6050_ACCEL_FS_8G    0x10   /* AFS_SEL=2 << 3 */
#define MPU6050_ACCEL_FS_16G   0x18   /* AFS_SEL=3 << 3 */

/* ==================== 数据结构 ==================== */
typedef struct {
    int16_t ax, ay, az;      /* 加速度原始值 (带符号16位) */
    int16_t gx, gy, gz;      /* 陀螺仪原始值 (带符号16位) */
    int16_t temp;            /* 温度 */
} MPU6050_RawData;

typedef struct {
    float ax, ay, az;        /* 加速度 g */
    float gx, gy, gz;        /* 角速度 dps */
    float temp_c;            /* 温度 ℃ */
} MPU6050_ScaledData;

/* ==================== API ==================== */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_ReadRaw(MPU6050_RawData *raw);
void    MPU6050_ScaleData(const MPU6050_RawData *raw,
                          MPU6050_ScaledData *scaled);
void    MPU6050_CalibrateGyro(float offset[3]);

#endif /* MPU6050_H */
