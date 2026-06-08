/**
 * data_logger.c — 串口数据记录器实现
 *
 * 输出格式: "tick:1250 target:15.20deg yaw:12.80deg gyro_z:-45.30dps pd:8.50deg servo:1680us thr:0.700 esc:1850us\r\n"
 */

#include "data_logger.h"

// 全局变量：保存UART句柄用于日志输出
static UART_HandleTypeDef *log_uart = NULL;

/**
 * @brief 初始化日志记录器
 * @param huart UART句柄指针
 */
void Logger_Init(UART_HandleTypeDef *huart) {
    log_uart = huart;
}

/* 
 * 避免newlib-nano浮点格式问题，完全手写定点数转字符串
 * 将整数转换为字符串并添加到缓冲区
 */
static void put_int(char *buf, int *pos, int val) {
    char tmp[12];       // 临时缓冲区存储反转的数字字符
    int idx = 0;        // 临时缓冲区索引
    int neg = 0;        // 负数标志
    
    // 检查是否为负数
    if (val < 0) { 
        neg = 1; 
        val = -val;     // 转为正数处理
    }
    
    // 处理零值情况
    if (val == 0) { 
        tmp[idx++] = '0'; 
    }
    else { 
        // 逐位提取数字并存入临时缓冲区（逆序）
        while (val > 0) { 
            tmp[idx++] = '0' + (val % 10); 
            val /= 10; 
        } 
    }
    
    // 添加负号（如果需要）
    if (neg) tmp[idx++] = '-';
    
    // 将临时缓冲区内容按正确顺序复制到目标缓冲区
    while (idx > 0) buf[(*pos)++] = tmp[--idx];
}

/**
 * @brief 格式化浮点数为两位小数的字符串
 * @param buf 目标缓冲区
 * @param pos 当前位置指针
 * @param val 要格式化的浮点数
 */
static void fmt_f2(char *buf, int *pos, float val) {
    // 将浮点数乘以100并四舍五入得到整数值
    int s = (int)(val * 100.0f + (val >= 0 ? 0.5f : -0.5f));
    int i = s / 100;    // 整数部分
    int f = s % 100;    // 小数部分（以整数形式表示）
    
    // 处理负数的小数部分
    if (f < 0) f = -f;
    
    // 输出整数部分
    put_int(buf, pos, i);
    buf[(*pos)++] = '.';                // 小数点
    buf[(*pos)++] = '0' + (f / 10);     // 十分位
    buf[(*pos)++] = '0' + (f % 10);     // 百分位
}

/**
 * @brief 格式化浮点数为三位小数的字符串
 * @param buf 目标缓冲区
 * @param pos 当前位置指针
 * @param val 要格式化的浮点数
 */
static void fmt_f3(char *buf, int *pos, float val) {
    // 将浮点数乘以1000并四舍五入得到整数值
    int s = (int)(val * 1000.0f + (val >= 0 ? 0.5f : -0.5f));
    int i = s / 1000;   // 整数部分
    int f = s % 1000;   // 小数部分（以整数形式表示）
    
    // 处理负数的小数部分
    if (f < 0) f = -f;
    
    // 输出整数部分
    put_int(buf, pos, i);
    buf[(*pos)++] = '.';                // 小数点
    buf[(*pos)++] = '0' + (f / 100);    // 十分位
    buf[(*pos)++] = '0' + ((f / 10) % 10); // 百分位
    buf[(*pos)++] = '0' + (f % 10);     // 千分位
}

/*
 * VOFA+ JustFloat 协议输出
 * 帧格式: 每个float占4字节(小端) + 帧尾 00 00 80 7F
 * 通道: roll, pitch, yaw, gx, gy, gz, alpha, offset, zero_allowed, is_static
 */
void Logger_SendVOFA(const IMU_Attitude *att) {
    uint8_t buf[44];  // 10 floats * 4 + 4 tail
    uint8_t *p = buf;

    if (log_uart == NULL) return;

    float data[10];
    data[0] = att->roll;                    // 横滚角
    data[1] = att->pitch;                   // 俯仰角
    data[2] = att->yaw;                     // 偏航角
    data[3] = att->gyro_x;                  // X轴陀螺仪值
    data[4] = att->gyro_y;                  // Y轴陀螺仪值
    data[5] = att->gyro_z;                  // Z轴陀螺仪值
    data[6] = att->alpha;                   // 滤波系数
    data[7] = att->gyro_z_offset;           // Z轴陀螺仪偏移
    data[8] = (float)att->zero_allowed;     // 零点允许标志
    data[9] = (float)att->is_static;        // 静态状态标志

    // 将每个浮点数转换为4字节的小端格式
    for (int i = 0; i < 10; i++) {
        uint32_t bits;
        memcpy(&bits, &data[i], 4);         // 获取浮点数的二进制表示
        *p++ = (uint8_t)(bits);             // 字节0
        *p++ = (uint8_t)(bits >> 8);        // 字节1
        *p++ = (uint8_t)(bits >> 16);       // 字节2
        *p++ = (uint8_t)(bits >> 24);       // 字节3
    }
    
    /* 帧尾: float +inf (0x7F800000, little-endian) */
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    // 通过UART发送数据
    HAL_UART_Transmit(log_uart, buf, 44, 20);
}

/**
 * @brief 记录并发送日志帧数据
 * @param frame 日志帧结构体指针
 */
void Logger_Log(const LogFrame *frame) {
    char buf[220];          // 输出缓冲区
    int pos = 0;            // 当前写入位置

    if (log_uart == NULL) return;

    // 构建日志字符串
    pos += sprintf(buf + pos, "tick:%lu", (unsigned long)frame->tick);  // 系统滴答计数
    buf[pos++] = ' '; buf[pos++] = 't'; buf[pos++] = 'a'; buf[pos++] = 'r';
    buf[pos++] = 'g'; buf[pos++] = 'e'; buf[pos++] = 't'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->target_angle);                             // 目标角度
    buf[pos++] = 'd'; buf[pos++] = 'e'; buf[pos++] = 'g';
    buf[pos++] = ' '; buf[pos++] = 'y'; buf[pos++] = 'a'; buf[pos++] = 'w'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->current_yaw);                              // 当前偏航角
    buf[pos++] = 'd'; buf[pos++] = 'e'; buf[pos++] = 'g';
    buf[pos++] = ' '; buf[pos++] = 'g'; buf[pos++] = 'y'; buf[pos++] = 'r'; buf[pos++] = 'o'; buf[pos++] = '_'; buf[pos++] = 'z'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->gyro_z);                                   // Z轴陀螺仪值
    buf[pos++] = 'd'; buf[pos++] = 'p'; buf[pos++] = 's';
    buf[pos++] = ' '; buf[pos++] = 'a'; buf[pos++] = 'l'; buf[pos++] = 'p'; buf[pos++] = 'h'; buf[pos++] = 'a'; buf[pos++] = ':';
    fmt_f3(buf, &pos, frame->imu_alpha);                                // IMU滤波系数
    buf[pos++] = ' '; buf[pos++] = 'o'; buf[pos++] = 'f'; buf[pos++] = 'f'; buf[pos++] = 's'; buf[pos++] = 'e'; buf[pos++] = 't'; buf[pos++] = ':';
    fmt_f3(buf, &pos, frame->gyro_z_offset);                            // 陀螺仪Z轴偏移
    buf[pos++] = ' '; buf[pos++] = 'z'; buf[pos++] = 'e'; buf[pos++] = 'r'; buf[pos++] = 'o'; buf[pos++] = ':';
    put_int(buf, &pos, frame->zero_allowed);                            // 零点允许标志
    buf[pos++] = ' '; buf[pos++] = 's'; buf[pos++] = 't'; buf[pos++] = 'a'; buf[pos++] = 't'; buf[pos++] = 'i'; buf[pos++] = 'c'; buf[pos++] = ':';
    put_int(buf, &pos, frame->imu_static);                              // IMU静态状态
    buf[pos++] = ' '; buf[pos++] = 'p'; buf[pos++] = 'd'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->pd_output);                                // PD控制器输出
    buf[pos++] = 'd'; buf[pos++] = 'e'; buf[pos++] = 'g';
    buf[pos++] = ' '; buf[pos++] = 's'; buf[pos++] = 'e'; buf[pos++] = 'r'; buf[pos++] = 'v'; buf[pos++] = 'o'; buf[pos++] = ':';
    pos += sprintf(buf + pos, "%luus thr:", (unsigned long)frame->servo_pwm); // 舵机PWM输出
    fmt_f3(buf, &pos, frame->throttle_input);                           // 油门输入
    buf[pos++] = ' '; buf[pos++] = 'e'; buf[pos++] = 's'; buf[pos++] = 'c'; buf[pos++] = ':';
    pos += sprintf(buf + pos, "%luus\r\n", (unsigned long)frame->esc_pwm);    // ESC PWM输出及换行

    // 通过UART发送日志数据
    HAL_UART_Transmit(log_uart, (uint8_t *)buf, (uint16_t)pos, 20);
}