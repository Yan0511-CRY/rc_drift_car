/**
 * vehicle_control.c — 车辆执行层实现
 *
 * ============== 转向控制: 陀螺仪辅助漂移 ==============
 *
 * Mode 0 — 增益反打 (GYRO_GAIN):
 *   ┌──────────┐    rc_angle     ┌──────────────┐   servo_pwm
 *   │ RC摇杆   │ ─────────────→  │              │ ──────────→ 舵机
 *   └──────────┘                 │  servo =      │
 *                                │  rc_angle     │
 *   ┌──────────┐   gyro_z        │  - GAIN*gyro  │
 *   │ 陀螺仪   │ ─────────────→  │              │
 *   └──────────┘                 └──────────────┘
 *
 *   车尾右甩(gyro_z>0) → 舵机自动往左打 → 反打修正
 *   车尾左甩(gyro_z<0) → 舵机自动往右打 → 反打修正
 *   遥控器回中 → 舵机回中 + 陀螺仪修正仍然生效
 *
 * Mode 1 — 角速度追踪 (YAW_RATE_TRACKING):
 *   ┌──────────┐  target_rate   ┌──────┐   error   ┌─────┐  servo
 *   │ RC摇杆   │ ────────────→  │  Σ   │ ───────→  │ PID │ ────→ 舵机
 *   └──────────┘                │ e=r-y│           └─────┘
 *                               │      │
 *   ┌──────────┐   gyro_z       │      │
 *   │ 陀螺仪   │ ────────────→  │  (-) │
 *   └──────────┘                └──────┘
 *
 *   遥控器打多少 = 期望车转多快，PID 自动追。
 *   更激进的控制，适合答辩展示闭环控制能力。
 */

#include "vehicle_control.h"
#include <math.h>

/* ==================== 辅助函数 ==================== */
static float clamp_float(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static uint32_t clamp_u32(uint32_t val, uint32_t min, uint32_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static float rc_to_normalized(uint32_t pwm) {
    float val;
    if (pwm <= RC_PWM_CENTER + RC_DEADBAND &&
        pwm >= RC_PWM_CENTER - RC_DEADBAND) {
        return 0.0f;
    }
    if (pwm > RC_PWM_CENTER) {
        val = (float)(pwm - RC_PWM_CENTER)
            / (float)(RC_PWM_MAX - RC_PWM_CENTER);
    } else {
        val = (float)(pwm - RC_PWM_CENTER)
            / (float)(RC_PWM_CENTER - RC_PWM_MIN);
    }
    return clamp_float(val, -1.0f, 1.0f);
}

static float angle_to_pwm(float angle_deg) {
    float ratio = angle_deg / STEERING_ANGLE_MAX;
    ratio = clamp_float(ratio, -1.0f, 1.0f);
    return (float)SERVO_PWM_CENTER
         + ratio * (float)(SERVO_PWM_MAX - SERVO_PWM_CENTER);
}

/* ==================== 油门曲线 ==================== */

float ThrottleCurve(float input, float expo) {
    float sign = (input >= 0.0f) ? 1.0f : -1.0f;
    float abs_in = fabsf(input);
    float abs_out = (1.0f - expo) * abs_in + expo * abs_in * abs_in * abs_in;
    return sign * abs_out;
}

/* ==================== 转向控制 ==================== */

void Steering_Init(Steering_Control *steer) {
    steer->rc_angle        = 0.0f;
    steer->target_yaw_rate = 0.0f;
    steer->gyro_z_filtered = 0.0f;
    steer->servo_angle_cmd = 0.0f;
    steer->servo_pwm       = SERVO_PWM_CENTER;

    /* 初始化PID (在Mode 1时启用) */
    PID_Init(&steer->pid,
             YAW_TRACK_KP, YAW_TRACK_KD,
             -STEERING_ANGLE_MAX, STEERING_ANGLE_MAX,
             0.0f);  // Mode 1不需要D项低通(另有角速度滤波)
}

void Steering_SetRC(Steering_Control *steer, uint32_t rc_pwm) {
    float norm = rc_to_normalized(rc_pwm);

    /* RC → 舵机基础角度 (两种模式都需要) */
    steer->rc_angle = norm * STEERING_ANGLE_MAX;

    /* RC → 目标偏航角速度 (Mode 1专用) */
    steer->target_yaw_rate = norm * YAW_RATE_MAX;
}

void Steering_Update(Steering_Control *steer, float imu_gyro_z, float dt) {
    /*
     * 先对陀螺仪角速度做低通滤波，减少电机振动噪声
     * y[n] = α·y[n-1] + (1-α)·x[n]
     */
    steer->gyro_z_filtered = GYRO_LPF_ALPHA * steer->gyro_z_filtered
                           + (1.0f - GYRO_LPF_ALPHA) * imu_gyro_z;

#if GYRO_CONTROL_MODE == 0
    /* ================================================================
     * Mode 0: 陀螺仪增益反打 (GYRO_GAIN)
     *
     * 公式: servo_angle = rc_angle - GYRO_GAIN × gyro_z
     *
     * 物理直觉:
     *   - 入弯: 你打方向右转 → rc_angle = +15°
     *   - 起漂: 后轮打滑, 车尾开始甩 → gyro_z 飙升到 +200dps
     *   - 反打: GYRO_GAIN × 200 = 20°的反打修正
     *   - 舵机: 15° - 20° = -5° (前轮自动往左打!)
     *   - 你甚至不需要主动反打，陀螺仪已经帮你打了
     *
     * GAIN越大 → 反打越猛 → 车越不容易spin out
     * GAIN越小 → 接近纯手动 → 更考验你的手速
     * ================================================================ */
    {
        float gyro_correction = GYRO_GAIN * steer->gyro_z_filtered;
        steer->servo_angle_cmd = steer->rc_angle - gyro_correction;
    }

#else
    /* ================================================================
     * Mode 1: 偏航角速度追踪 (YAW_RATE_TRACKING)
     *
     * RC摇杆 = 你想让车转多快 (dps), PID自动控制舵机来达到这个转速
     *
     * 例如: 打半舵 = 目标150dps, 车实际90dps
     *   → error = 150 - 90 = 60dps
     *   → PID输出正舵角 → 加大转向 → 追到150dps
     *
     * 这是完整的闭环控制, 答辩时强调:
     *   "遥控指令不再是直接控制舵机, 而是作为角速度设定值
     *    输入到PID控制器, 实现偏航角速度的闭环跟踪"
     * ================================================================ */
    {
        float error = steer->target_yaw_rate - steer->gyro_z_filtered;

        /* 用PID追踪目标角速度 (这里复用pid.h的PD结构, I项手动加) */
        float p_out = steer->pid.kp * error;

        /* D项: 角加速度阻尼 (gyro_z的变化率 = 角加速度)
         * 直接用filtered gyro_z做微分: -Kd * gyro_z
         * 抑制角速度的剧烈变化 (防止舵机剧烈摆动) */
        float d_out = steer->pid.kd * (-steer->gyro_z_filtered);

        /* I项: 积分消除稳态误差 (比如持续侧风或路面倾斜)
         * 使用静态变量保存积分值 */
        static float i_accum = 0.0f;
        i_accum += error * dt;
        i_accum = clamp_float(i_accum,
                              -YAW_TRACK_I_MAX / YAW_TRACK_KI,
                               YAW_TRACK_I_MAX / YAW_TRACK_KI);
        float i_out = YAW_TRACK_KI * i_accum;

        /* 遥控回中时清零积分, 避免车停不下来 */
        if (fabsf(steer->target_yaw_rate) < 1.0f) {
            i_accum = 0.0f;
            i_out   = 0.0f;
        }

        steer->servo_angle_cmd = p_out + i_out + d_out;
    }
#endif

    /* 限幅 → PWM */
    steer->servo_angle_cmd = clamp_float(steer->servo_angle_cmd,
                                         -STEERING_ANGLE_MAX,
                                          STEERING_ANGLE_MAX);
    steer->servo_pwm = (uint32_t)angle_to_pwm(steer->servo_angle_cmd);
    steer->servo_pwm = clamp_u32(steer->servo_pwm,
                                 SERVO_PWM_MIN, SERVO_PWM_MAX);
}

uint32_t Steering_GetPWM(const Steering_Control *steer) {
    return steer->servo_pwm;
}

/* ==================== 油门控制 ==================== */

void ESC_Init(ESC_Control *esc) {
    esc->throttle_input  = 0.0f;
    esc->throttle_output = 0.0f;
    esc->esc_pwm         = THROTTLE_PWM_NEUTRAL;
}

void ESC_SetThrottle(ESC_Control *esc, uint32_t rc_pwm) {
    float abs_throttle;

    esc->throttle_input = rc_to_normalized(rc_pwm);
    esc->throttle_output = ThrottleCurve(esc->throttle_input, THROTTLE_EXPO);

    abs_throttle = fabsf(esc->throttle_output);

    if (abs_throttle < 0.03f) {
        esc->esc_pwm = THROTTLE_PWM_NEUTRAL;
    } else if (esc->throttle_output > 0.0f) {
        esc->esc_pwm = THROTTLE_PWM_NEUTRAL + THROTTLE_DEADBAND
                     + (uint32_t)((esc->throttle_output - 0.03f) / 0.97f
                                  * (THROTTLE_PWM_MAX - THROTTLE_PWM_NEUTRAL
                                     - THROTTLE_DEADBAND));
    } else {
        esc->esc_pwm = THROTTLE_PWM_NEUTRAL - THROTTLE_DEADBAND
                     + (uint32_t)((esc->throttle_output + 0.03f) / 0.97f
                                  * (THROTTLE_PWM_NEUTRAL - THROTTLE_PWM_MIN
                                     - THROTTLE_DEADBAND));
    }
    esc->esc_pwm = clamp_u32(esc->esc_pwm, THROTTLE_PWM_MIN, THROTTLE_PWM_MAX);
}

uint32_t ESC_GetPWM(const ESC_Control *esc) {
    return esc->esc_pwm;
}
