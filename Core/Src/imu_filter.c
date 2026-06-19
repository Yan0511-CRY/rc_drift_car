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
 * 改进点（2026.06）:
 *   1. 动态α — 根据加速度模值(∥a∥)自适应调整信任权重
 *      高速漂移时(∥a∥>>1g) → α趋近1 → 几乎纯靠陀螺仪
 *      平稳运行时(∥a∥≈1g) → α恢复0.96 → 加速度计修正长期漂移
 *   2. RC遥控器辅助零偏判定 — 用遥控信号判断"真·直线行驶"
 *      油门/舵机都回中 + 陀螺低谷 → 判定为可归零状态，快速收敛offset
 *   3. 1D卡尔曼做零偏估计 — 替换EMA，有理论依据的收敛追踪
 *
 * Yaw角的特殊性：
 *   加速度计无法测量Yaw（绕重力方向旋转），所以Yaw纯靠陀螺仪积分。
 *   通过零速检测+卡尔曼滤波自动补偿陀螺仪零偏，减缓漂移速度。
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

/* 三轴角速度 IIR 低通滤波状态（静态，上电自动清零） */
static float gyro_soft_lpf_gx = 0.0f;
static float gyro_soft_lpf_gy = 0.0f;
static float gyro_soft_lpf_gz = 0.0f;

/* RC辅助归零信号（从main.c传入）*/
static float rc_throttle_norm = 0.0f;   /* -1.0 ~ 1.0 */
static float rc_steering_norm = 0.0f;   /* -1.0 ~ 1.0 */

static uint8_t last_zero_allowed = 0;

/* ==================== 1D卡尔曼 ==================== */

static void Kalman1D_Init(IMU_Kalman1D *k, float init_offset) {
    k->offset = init_offset;
    k->P      = IMU_KALMAN_P_INIT;
    k->Q      = IMU_KALMAN_Q;
    k->R      = IMU_KALMAN_R;
}

static float Kalman1D_UpdateWithRScale(IMU_Kalman1D *k,
                                       float measurement,
                                       float r_scale) {
    /* 预测：零偏变化缓慢，offset_{k|k-1} = offset_{k-1} */
    k->P = k->P + k->Q;

    /* 更新：测量值 = 当前陀螺读数（静止时应≈0） */
    float R = k->R * r_scale;
    float K = k->P / (k->P + R);             /* 卡尔曼增益 */
    k->offset = k->offset + K * (measurement - k->offset);
    k->P = (1.0f - K) * k->P;

    return k->offset;
}

static float Kalman1D_Update(IMU_Kalman1D *k, float measurement) {
    return Kalman1D_UpdateWithRScale(k, measurement, 1.0f);
}

/* ==================== RC辅助归零 ==================== */

void IMU_SetRCInputs(float throttle_norm, float steering_norm) {
    rc_throttle_norm = throttle_norm;
    rc_steering_norm = steering_norm;
}

/* 判断是否处于"可归零状态"：
 *   - 油门回中（不加速/刹车）
 *   - 舵机回中（直线行驶或静止）
 *   - 或陀螺仪已判定静止（低角速度）
 */
static uint8_t zero_update_mode(float corrected_gx, float corrected_gy,
                                float corrected_gz,
                                float ax, float ay, float az) {
    float thr_abs = fabsf(rc_throttle_norm);
    float str_abs = fabsf(rc_steering_norm);
    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    /* 条件A: 去偏后的角速度低（原有零速检测）*/
    uint8_t gyro_static = (fabsf(corrected_gx) < ZERO_RATE_THRESHOLD)
                       && (fabsf(corrected_gy) < ZERO_RATE_THRESHOLD)
                       && (fabsf(corrected_gz) < IMU_ZERO_UPDATE_RATE_THRESHOLD);
    uint8_t accel_static = (fabsf(accel_norm - 1.0f) < IMU_ZERO_ACCEL_TOLERANCE);

    /* 条件B: 遥控器回中（油门<5% 且 舵机<5%）*/
    uint8_t rc_centered = (thr_abs < IMU_RC_CENTER_THRESHOLD)
                       && (str_abs < IMU_RC_CENTER_THRESHOLD);

    /* 条件C: 遥控器基本回中+陀螺不太活跃（直线行驶不是急加速/刹车）*/
    uint8_t rc_near_center = (thr_abs < IMU_RC_NEAR_CENTER_THRESHOLD)
                          && (str_abs < IMU_RC_NEAR_CENTER_THRESHOLD);

    if (!gyro_static || !accel_static) {
        return 0;
    }

    if (rc_centered) {
        return 1;  /* fast update */
    }

    if (rc_near_center) {
        return 2;  /* conservative update */
    }

    return 0;
}

/* ==================== 动态α计算 ==================== */

static float compute_dynamic_alpha(float ax, float ay, float az) {
    /* 计算加速度模值 */
    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    /* 限幅防止除零 */
    if (accel_norm < 0.1f) accel_norm = 0.1f;

    /*
     * 加速度偏差: accel_norm 偏离 1g 的程度
     *   accel_norm = 1.0  → 偏差=0  → α=ALPHA_BASE (信任加速度计)
     *   accel_norm = 2.5+ → 偏差=1  → α=ALPHA_MAX (几乎纯陀螺仪)
     */
    float deviation = (accel_norm - IMU_ACCEL_NORM_IDLE)
                    / (IMU_ACCEL_NORM_AGGRESSIVE - IMU_ACCEL_NORM_IDLE);
    if (deviation < 0.0f) deviation = 0.0f;
    if (deviation > 1.0f) deviation = 1.0f;

    float alpha = IMU_ALPHA_BASE + (IMU_ALPHA_MAX - IMU_ALPHA_BASE) * deviation;
    return alpha;
}

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
    /* ── 软件 IIR 低通滤波（三轴统一处理） ──
     * 作用时机：在原始数据进入互补滤波/积分之前
     * 公式：y[n] = α * y[n-1] + (1-α) * x[n]
     * 目的：抑制电机振动和机械谐振引入的高频噪声
     * 注意：只滤波 gx/gy/gz，不滤波 ax/ay/az（加速度计不需要） */
    gyro_soft_lpf_gx = IMU_GYRO_SOFT_LPF_ALPHA * gyro_soft_lpf_gx
                     + (1.0f - IMU_GYRO_SOFT_LPF_ALPHA) * gx;
    gyro_soft_lpf_gy = IMU_GYRO_SOFT_LPF_ALPHA * gyro_soft_lpf_gy
                     + (1.0f - IMU_GYRO_SOFT_LPF_ALPHA) * gy;
    gyro_soft_lpf_gz = IMU_GYRO_SOFT_LPF_ALPHA * gyro_soft_lpf_gz
                     + (1.0f - IMU_GYRO_SOFT_LPF_ALPHA) * gz;

    /* 用滤波后的角速度替换原始值，后续所有计算都使用滤波结果 */
    gx = gyro_soft_lpf_gx;
    gy = gyro_soft_lpf_gy;
    gz = gyro_soft_lpf_gz;

    float accel_pitch, accel_roll;

    /* 1. 去除陀螺仪零偏（零偏由卡尔曼在线估计） */
    gx -= zd_comp.gyro_offset[0];
    gy -= zd_comp.gyro_offset[1];
    gz -= zd_comp.gyro_offset[2];

    /* 2. 从加速度计反算Pitch和Roll
     *    pitch = atan2(-ax, sqrt(ay² + az²))
     *    roll  = atan2( ay, az)
     */
    accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    accel_roll  = atan2f( ay, az) * RAD_TO_DEG;

    /* 3. 动态α: 根据加速度模值自适应调整融合权重
     *    高速漂移时离心力大 → accel_norm >> 1g → α趋近1
     *    平稳运行时 → accel_norm ≈ 1g → α恢复基线0.96
     */
    float alpha = compute_dynamic_alpha(ax, ay, az);

    /* 4. 互补滤波: Pitch & Roll (融合加速度计+陀螺仪) */
    att->pitch = alpha * (att->pitch + gx * dt)
               + (1.0f - alpha) * accel_pitch;

    att->roll  = alpha * (att->roll  + gy * dt)
               + (1.0f - alpha) * accel_roll;

    /* 5. Yaw: 纯陀螺仪积分 + 卡尔曼零偏补偿 */
    IMU_ZeroDrift_Update(gx, gy, gz, ax, ay, az, dt);
    att->yaw    += gz * dt;
    att->gyro_z  = gz;
    att->gyro_y  = gy;
    att->gyro_x  = gx;
    att->alpha = alpha;
    att->gyro_z_offset = zd_comp.gyro_offset[2];
    att->zero_allowed = last_zero_allowed;
    att->is_static = zd_comp.is_static;

    /* Yaw角归一化到 [-180, 180] */
    while (att->yaw >  180.0f) att->yaw -= 360.0f;
    while (att->yaw < -180.0f) att->yaw += 360.0f;
}

/* ==================== 零漂自动补偿 (1D卡尔曼+RC辅助) ==================== */

void IMU_ZeroDrift_Init(void) {
    zd_comp.zero_rate_accum_time = 0.0f;
    zd_comp.zero_rate_samples    = 0;
    zd_comp.is_static            = 0;

    /* 初始化1D卡尔曼，初始offset=当前位置（启动时已做500次平均校准）*/
    Kalman1D_Init(&zd_comp.kalman, zd_comp.gyro_offset[2]);
}

void IMU_ZeroDrift_Update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt) {
    /*
     * 1D卡尔曼做零偏估计：
     *   状态量 = 陀螺零偏 (offset)
     *   观测量 = 当前陀螺仪读数（静止时应为0，所以 measurement = gz + offset）
     *          = 实际上传入的是去除offset之前的原始值
     *
     * 但是我们传入的是去除offset之后的gz，所以需要加上当前offset作为测量值
     * 在静止状态时，off+去除后的gz ≈ gz_raw ≈ 0 → off ≈ -gz
     *
     * 简化写法：直接用 当前offset + gz（原始值重建）作为卡尔曼观测
     */
    float raw_gz = gz + zd_comp.kalman.offset;  /* 重建原始值 */

    uint8_t update_mode = zero_update_mode(gx, gy, gz, ax, ay, az);
    last_zero_allowed = (update_mode != 0U);

    /* RC辅助判定：是否允许进行归零 */
    if (update_mode != 0U) {
        zd_comp.zero_rate_accum_time += dt;
        zd_comp.zero_rate_samples++;

        /* 持续可归零状态超过阈值 → 触发卡尔曼更新 */
        if (zd_comp.zero_rate_accum_time > ZERO_RATE_DURATION) {
            if (!zd_comp.is_static) {
                zd_comp.is_static = 1;
            }
            if (update_mode == 1U) {
                Kalman1D_Update(&zd_comp.kalman, raw_gz);
            } else {
                Kalman1D_UpdateWithRScale(&zd_comp.kalman, raw_gz,
                                          IMU_KALMAN_RC_NEAR_R_SCALE);
            }

            /* 将卡尔曼估计的零偏写回全局offset */
            zd_comp.gyro_offset[2] = zd_comp.kalman.offset;
        }
    } else {
        /* 非静止/非归零状态，重置计时但不重置卡尔曼（卡尔曼保持当前估计值）*/
        zd_comp.zero_rate_accum_time  = 0.0f;
        zd_comp.zero_rate_samples     = 0;
        zd_comp.is_static             = 0;

        /*
         * 即使不静止，也让卡尔曼做一次预测（P增大，表示不确定性增加）
         * 这样下次静止时，卡尔曼能更快收敛到新值
         */
        zd_comp.kalman.P += zd_comp.kalman.Q;
    }
}
