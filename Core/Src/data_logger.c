/**
 * data_logger.c — 串口/蓝牙数据记录器 (非阻塞发送)
 *
 * 发送架构:
 *   ┌──────────────┐    enqueue    ┌────────────────┐    TXE ISR    ┌──────────┐
 *   │ Control Loop │ ──────────── >│  Ring Buffer   │ ─────────── > │  USART1  │
 *   │   (500Hz)    │  (零等待)     │  (256 bytes)   │  (逐字节)    │   TX Pin │
 *   └──────────────┘               └────────────────┘               └──────────┘
 *
 *   若缓冲区剩余空间不足 → 丢弃当前帧，绝不停顿控制环。
 *
 * 输出模式:
 *   VOFA_OUTPUT_ENABLE=1 → VOFA+ JustFloat (44 bytes 二进制)
 *   VOFA_OUTPUT_ENABLE=0 → CSV 文本 (14 字段)
 */

#include "data_logger.h"

/* ==================== 环形缓冲区 ==================== */
static UART_HandleTypeDef *log_uart = NULL;

static volatile uint8_t  tx_buf[LOG_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;   /* 写指针 (主循环修改) */
static volatile uint16_t tx_tail = 0;   /* 读指针 (ISR 修改) */

/* 缓冲区可用空间 (主循环调用) */
static inline uint16_t buf_available(void) {
    return (uint16_t)((LOG_TX_BUF_SIZE - 1) -
                      ((tx_head - tx_tail) & (LOG_TX_BUF_SIZE - 1)));
}

/*
 * 将 data 拷入环形缓冲区并启动 TXE 中断发送。
 * 若空间不足则整帧丢弃 (返回 0)。
 */
static int uart_enqueue(const uint8_t *data, uint16_t len) {
    if (log_uart == NULL) return 0;
    if (len > buf_available()) return 0;   /* 空间不足 → 丢帧 */

    uint16_t h = tx_head;
    for (uint16_t i = 0; i < len; i++) {
        tx_buf[h] = data[i];
        h = (h + 1) & (LOG_TX_BUF_SIZE - 1);
    }
    tx_head = h;                          /* 更新写指针 */

    /* 内存屏障: 确保 ISR 看到最新的 head 值 */
    __DMB();

    /* 启动 TXE 中断 (若已在发送中，此操作无副作用) */
    __HAL_UART_ENABLE_IT(log_uart, UART_IT_TXE);
    return 1;
}

/* ==================== ISR: TXE 中断处理 ==================== */

/**
 * @brief USART1 TXE 中断回调
 *        由 stm32f4xx_it.c 的 USART1_IRQHandler() 调用。
 *        逐字节从环形缓冲区取出数据写入 DR。
 */
void Logger_UART_IRQHandler(void) {
    if (log_uart == NULL) return;

    /* 仅处理 TXE 中断 (发送数据寄存器空) */
    if (__HAL_UART_GET_IT_SOURCE(log_uart, UART_IT_TXE) &&
        __HAL_UART_GET_FLAG(log_uart, UART_FLAG_TXE))
    {
        if (tx_tail != tx_head) {
            /* 有数据 → 写 DR */
            log_uart->Instance->DR = (uint8_t)(tx_buf[tx_tail] & 0xFF);
            tx_tail = (tx_tail + 1) & (LOG_TX_BUF_SIZE - 1);
        } else {
            /* 缓冲区空 → 关 TXE 中断，等下次 enqueue 再开 */
            __HAL_UART_DISABLE_IT(log_uart, UART_IT_TXE);
        }
    }
}

/* ==================== 初始化 ==================== */

void Logger_Init(UART_HandleTypeDef *huart) {
    log_uart = huart;
    tx_head = 0;
    tx_tail = 0;
}

/* ==================== 数字格式化工具 ==================== */

/* 整数 → 字符串 (避免 newlib-nano 浮点 printf 问题) */
static void put_int(char *buf, int *pos, int val) {
    char tmp[12];
    int idx = 0;
    int neg = 0;

    if (val < 0) { neg = 1; val = -val; }
    if (val == 0) { tmp[idx++] = '0'; }
    else { while (val > 0) { tmp[idx++] = '0' + (val % 10); val /= 10; } }
    if (neg) tmp[idx++] = '-';
    while (idx > 0) buf[(*pos)++] = tmp[--idx];
}

/* 格式化 unsigned long */
static void put_ulong(char *buf, int *pos, uint32_t val) {
    char tmp[12];
    int idx = 0;
    if (val == 0) { tmp[idx++] = '0'; }
    else { while (val > 0) { tmp[idx++] = '0' + (int)(val % 10); val /= 10; } }
    while (idx > 0) buf[(*pos)++] = tmp[--idx];
}

/* 浮点 → 2位小数 */
static void fmt_f2(char *buf, int *pos, float val) {
    int s = (int)(val * 100.0f + (val >= 0 ? 0.5f : -0.5f));
    int i = s / 100;
    int f = s % 100;
    if (f < 0) f = -f;
    put_int(buf, pos, i);
    buf[(*pos)++] = '.';
    buf[(*pos)++] = '0' + (f / 10);
    buf[(*pos)++] = '0' + (f % 10);
}

/* 浮点 → 3位小数 */
static void fmt_f3(char *buf, int *pos, float val) {
    int s = (int)(val * 1000.0f + (val >= 0 ? 0.5f : -0.5f));
    int i = s / 1000;
    int f = s % 1000;
    if (f < 0) f = -f;
    put_int(buf, pos, i);
    buf[(*pos)++] = '.';
    buf[(*pos)++] = '0' + (f / 100);
    buf[(*pos)++] = '0' + ((f / 10) % 10);
    buf[(*pos)++] = '0' + (f % 10);
}

/* ==================== VOFA+ JustFloat 输出 ==================== */

/*
 * 帧格式: 10 floats × 4 bytes (小端) + 帧尾 00 00 80 7F (+Inf)
 * 通道: roll, pitch, yaw, gx, gy, gz, alpha, offset, zero_allowed, is_static
 * 总长度: 44 bytes
 */
void Logger_SendVOFA(const IMU_Attitude *att) {
    uint8_t buf[44];
    uint8_t *p = buf;

    float data[10];
    data[0] = att->roll;
    data[1] = att->pitch;
    data[2] = att->yaw;
    data[3] = att->gyro_x;
    data[4] = att->gyro_y;
    data[5] = att->gyro_z;
    data[6] = att->alpha;
    data[7] = att->gyro_z_offset;
    data[8] = (float)att->zero_allowed;
    data[9] = (float)att->is_static;

    for (int i = 0; i < 10; i++) {
        uint32_t bits;
        memcpy(&bits, &data[i], 4);
        *p++ = (uint8_t)(bits);
        *p++ = (uint8_t)(bits >> 8);
        *p++ = (uint8_t)(bits >> 16);
        *p++ = (uint8_t)(bits >> 24);
    }

    /* 帧尾: float +inf (0x7F800000, little-endian) */
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    uart_enqueue(buf, 44);
}

/* ==================== CSV 文本输出 ==================== */

/*
 * 帧格式 (逗号分隔, \r\n 结尾):
 * time_ms,roll,pitch,yaw,gx,gy,gz,ax,ay,az,target_angle,throttle_norm,servo_pwm,esc_pwm
 *
 * 示例:
 * 12500,1.23,-0.45,15.20,-0.30,0.12,-45.30,0.02,-0.98,0.05,15.20,0.700,1680,1850\r\n
 */
void Logger_SendCSV(const LogFrame *frame, const IMU_Attitude *att) {
    char buf[256];
    int pos = 0;

    /* time_ms: 用控制循环计数 × 2ms 得到毫秒 */
    put_ulong(buf, &pos, (uint32_t)(frame->tick * 2));
    buf[pos++] = ',';

    /* 姿态: roll, pitch, yaw */
    fmt_f2(buf, &pos, att->roll);       buf[pos++] = ',';
    fmt_f2(buf, &pos, att->pitch);      buf[pos++] = ',';
    fmt_f2(buf, &pos, att->yaw);        buf[pos++] = ',';

    /* 陀螺仪: gx, gy, gz */
    fmt_f2(buf, &pos, att->gyro_x);     buf[pos++] = ',';
    fmt_f2(buf, &pos, att->gyro_y);     buf[pos++] = ',';
    fmt_f2(buf, &pos, att->gyro_z);     buf[pos++] = ',';

    /* 加速度计: ax, ay, az */
    fmt_f2(buf, &pos, att->accel_x);    buf[pos++] = ',';
    fmt_f2(buf, &pos, att->accel_y);    buf[pos++] = ',';
    fmt_f2(buf, &pos, att->accel_z);    buf[pos++] = ',';

    /* 控制量: target_angle (目标转向角 deg), throttle_norm (归一化油门) */
    fmt_f2(buf, &pos, frame->target_angle);   buf[pos++] = ',';
    fmt_f3(buf, &pos, frame->throttle_input);  buf[pos++] = ',';

    /* PWM 输出: servo, esc */
    put_ulong(buf, &pos, frame->servo_pwm);   buf[pos++] = ',';
    put_ulong(buf, &pos, frame->esc_pwm);

    buf[pos++] = '\r';
    buf[pos++] = '\n';

    uart_enqueue((uint8_t *)buf, (uint16_t)pos);
}
