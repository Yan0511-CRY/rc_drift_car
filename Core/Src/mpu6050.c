/**
 * mpu6050.c — MPU6050 I2C 驱动实现
 */

#include "mpu6050.h"

/* 由Cubemx生成的I2C句柄，extern自main.c */
extern I2C_HandleTypeDef hi2c1;

/* 量程对应的灵敏度 (LSB per unit) */
static float gyro_lsb_per_dps = 32.8f;   /* ±1000dps 默认 */
static float accel_lsb_per_g  = 8192.0f;  /* ±4g 默认 */

/* ==================== 内部辅助 ==================== */

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *buf, uint8_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR << 1,
                            reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR << 1,
                             reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
}

/* ==================== 初始化 ==================== */

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t whoami;

    /* 检查WHO_AM_I，兼容MPU6050(0x68)和MPU6500(0x70) */
    if (MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &whoami, 1) != HAL_OK) {
        return 1;  /* I2C通信失败 */
    }
    {
        uint8_t id = whoami & 0x7E;
        if (id != 0x68 && id != 0x70) {
            return 2;  /* 芯片ID不匹配 */
        }
    }

    /* 1. 唤醒MPU6050 (退出睡眠模式) */
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT1, 0x00);
    HAL_Delay(100);

    /* 2. 设置采样率分频: SMPLRT = Gyro_Output_Rate / (1 + DIV)
     *    陀螺仪内部采样 8kHz, 经DIV后输出 1kHz:
     *    DIV = 8kHz/1kHz - 1 = 7 */
    MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07);

    /* 3. 设置数字低通滤波器 DLPF: 带宽 ~256Hz (DLPF_CFG=0)
     *    延迟 ~0.98ms, 适合1kHz输出 */
    MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x00);

    /* 4. 设置陀螺仪量程 */
    MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS);

    /* 5. 设置加速度计量程 */
    MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS);

    /* 更新灵敏度换算系数 */
    switch (MPU6050_GYRO_FS) {
        case MPU6050_GYRO_FS_250:  gyro_lsb_per_dps  = 131.0f; break;
        case MPU6050_GYRO_FS_500:  gyro_lsb_per_dps  = 65.5f;  break;
        case MPU6050_GYRO_FS_1000: gyro_lsb_per_dps  = 32.8f;  break;
        case MPU6050_GYRO_FS_2000: gyro_lsb_per_dps  = 16.4f;  break;
    }
    switch (MPU6050_ACCEL_FS) {
        case MPU6050_ACCEL_FS_2G:  accel_lsb_per_g = 16384.0f; break;
        case MPU6050_ACCEL_FS_4G:  accel_lsb_per_g = 8192.0f;  break;
        case MPU6050_ACCEL_FS_8G:  accel_lsb_per_g = 4096.0f;  break;
        case MPU6050_ACCEL_FS_16G: accel_lsb_per_g = 2048.0f;  break;
    }

    return 0;  /* 初始化成功 */
}

/* ==================== 数据读取 ==================== */

uint8_t MPU6050_ReadRaw(MPU6050_RawData *raw) {
    uint8_t buf[14];

    if (MPU6050_ReadReg(MPU6050_REG_ACCEL_XOUT_H, buf, 14) != HAL_OK) {
        return 1;
    }

    /* 大端转小端 */
    raw->ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->az   = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp = (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gz   = (int16_t)((buf[12] << 8) | buf[13]);

    return 0;
}

/* ==================== 单位换算 ==================== */

void MPU6050_ScaleData(const MPU6050_RawData *raw,
                       MPU6050_ScaledData *scaled) {
    /* 角速度: dps */
    scaled->gx = (float)raw->gx / gyro_lsb_per_dps;
    scaled->gy = (float)raw->gy / gyro_lsb_per_dps;
    scaled->gz = (float)raw->gz / gyro_lsb_per_dps * GYRO_SCALE_FACTOR;

    /* 加速度: g */
    scaled->ax = (float)raw->ax / accel_lsb_per_g;
    scaled->ay = (float)raw->ay / accel_lsb_per_g;
    scaled->az = (float)raw->az / accel_lsb_per_g;

    /* 温度 */
    scaled->temp_c = (float)raw->temp / 340.0f + 36.53f;
}

/* ==================== 陀螺仪零点校准 ==================== */

void MPU6050_CalibrateGyro(float offset[3]) {
    MPU6050_RawData raw;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (uint16_t i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        while (MPU6050_ReadRaw(&raw) != 0);  /* 等待读取成功 */
        sum_gx += raw.gx;
        sum_gy += raw.gy;
        sum_gz += raw.gz;
        HAL_Delay(2);  /* 每次间隔2ms, 总共约1秒 */
    }

    offset[0] = (float)sum_gx / (GYRO_CALIB_SAMPLES * gyro_lsb_per_dps);
    offset[1] = (float)sum_gy / (GYRO_CALIB_SAMPLES * gyro_lsb_per_dps);
    offset[2] = (float)sum_gz / (GYRO_CALIB_SAMPLES * gyro_lsb_per_dps);
}
