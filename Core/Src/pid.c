/**
 * pid.c — PD 控制器实现
 *
 * 漂移舵机控制采用 PD 结构：
 *   u = Kp * e + Kd * ω
 *
 * 其中：
 *   e  = target_angle - current_angle  (角度误差)
 *   ω  = gyro_z                       (Z轴角速度，即车体旋转角速度)
 *
 * 为什么不用 PID (不加 I 项)？
 *   - 漂移操控不需要消除稳态误差，反而是要持续偏航滑动
 *   - I 项会积累转角误差导致回正延迟，破坏漂移手感
 */

#include "pid.h"

void PID_Init(PID_Controller *pid,
              float kp, float kd,
              float out_min, float out_max,
              float lpf_alpha) {
    pid->kp          = kp;
    pid->kd          = kd;
    pid->setpoint    = 0.0f;
    pid->output      = 0.0f;
    pid->output_min  = out_min;
    pid->output_max  = out_max;
    pid->d_lpf_alpha = lpf_alpha;
    pid->d_filtered  = 0.0f;
}

float PID_Update(PID_Controller *pid, float error, float angular_velocity) {
    float p_term, d_term_raw, output;

    /* P项: 比例控制 */
    p_term = pid->kp * error;

    /* D项: 直接使用陀螺仪角速度 (本质是阻尼项，抑制过冲和振荡)
     *
     * 为什么是 (0 - angular_velocity) 而非 (error - last_error)/dt ?
     *   - 陀螺仪直接测量的是车体旋转速率
     *   - 当车头正在向右旋转(ω>0)而我们还在指令左转时，
     *     D项会输出反向力矩来阻尼过度旋转
     *   - 这种"测量微分"比"误差微分"噪声更小，物理意义更直接
     */
    d_term_raw = pid->kd * (-angular_velocity);

    /* D项一阶低通滤波: 平滑陀螺仪噪声
     *   y[n] = α·y[n-1] + (1-α)·x[n]
     */
    pid->d_filtered = pid->d_lpf_alpha * pid->d_filtered
                    + (1.0f - pid->d_lpf_alpha) * d_term_raw;

    /* 总输出 */
    output = p_term + pid->d_filtered;

    /* 限幅 */
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->output = output;
    return output;
}

void PID_Reset(PID_Controller *pid) {
    pid->output     = 0.0f;
    pid->d_filtered = 0.0f;
}
