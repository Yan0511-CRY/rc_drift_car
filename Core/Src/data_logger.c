/**
 * data_logger.c — 串口数据记录器实现
 *
 * 输出格式: "tick:1250 target:15.20deg yaw:12.80deg gyro_z:-45.30dps pd:8.50deg servo:1680us thr:0.700 esc:1850us\r\n"
 */

#include "data_logger.h"

static UART_HandleTypeDef *log_uart = NULL;

void Logger_Init(UART_HandleTypeDef *huart) {
    log_uart = huart;
}

/* 避免newlib-nano浮点格式问题，完全手写定点数转字符串 */
static void put_int(char *buf, int *pos, int val) {
    char tmp[12];
    int idx = 0, neg = 0;
    if (val < 0) { neg = 1; val = -val; }
    if (val == 0) { tmp[idx++] = '0'; }
    else { while (val > 0) { tmp[idx++] = '0' + (val % 10); val /= 10; } }
    if (neg) tmp[idx++] = '-';
    while (idx > 0) buf[(*pos)++] = tmp[--idx];
}

static void fmt_f2(char *buf, int *pos, float val) {
    int s = (int)(val * 100.0f + (val >= 0 ? 0.5f : -0.5f));
    int i = s / 100, f = s % 100;
    if (f < 0) f = -f;
    put_int(buf, pos, i);
    buf[(*pos)++] = '.';
    buf[(*pos)++] = '0' + (f / 10);
    buf[(*pos)++] = '0' + (f % 10);
}

static void fmt_f3(char *buf, int *pos, float val) {
    int s = (int)(val * 1000.0f + (val >= 0 ? 0.5f : -0.5f));
    int i = s / 1000, f = s % 1000;
    if (f < 0) f = -f;
    put_int(buf, pos, i);
    buf[(*pos)++] = '.';
    buf[(*pos)++] = '0' + (f / 100);
    buf[(*pos)++] = '0' + ((f / 10) % 10);
    buf[(*pos)++] = '0' + (f % 10);
}

void Logger_Log(const LogFrame *frame) {
    char buf[160];
    int pos = 0;

    if (log_uart == NULL) return;

    pos += sprintf(buf + pos, "tick:%lu", (unsigned long)frame->tick);
    buf[pos++] = ' '; buf[pos++] = 't'; buf[pos++] = 'a'; buf[pos++] = 'r';
    buf[pos++] = 'g'; buf[pos++] = 'e'; buf[pos++] = 't'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->target_angle);
    buf[pos++] = 'd'; buf[pos++] = 'e'; buf[pos++] = 'g';
    buf[pos++] = ' '; buf[pos++] = 'y'; buf[pos++] = 'a'; buf[pos++] = 'w'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->current_yaw);
    buf[pos++] = 'd'; buf[pos++] = 'e'; buf[pos++] = 'g';
    buf[pos++] = ' '; buf[pos++] = 'g'; buf[pos++] = 'y'; buf[pos++] = 'r'; buf[pos++] = 'o'; buf[pos++] = '_'; buf[pos++] = 'z'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->gyro_z);
    buf[pos++] = 'd'; buf[pos++] = 'p'; buf[pos++] = 's';
    buf[pos++] = ' '; buf[pos++] = 'p'; buf[pos++] = 'd'; buf[pos++] = ':';
    fmt_f2(buf, &pos, frame->pd_output);
    buf[pos++] = 'd'; buf[pos++] = 'e'; buf[pos++] = 'g';
    buf[pos++] = ' '; buf[pos++] = 's'; buf[pos++] = 'e'; buf[pos++] = 'r'; buf[pos++] = 'v'; buf[pos++] = 'o'; buf[pos++] = ':';
    pos += sprintf(buf + pos, "%luus thr:", (unsigned long)frame->servo_pwm);
    fmt_f3(buf, &pos, frame->throttle_input);
    buf[pos++] = ' '; buf[pos++] = 'e'; buf[pos++] = 's'; buf[pos++] = 'c'; buf[pos++] = ':';
    pos += sprintf(buf + pos, "%luus\r\n", (unsigned long)frame->esc_pwm);

    HAL_UART_Transmit(log_uart, (uint8_t *)buf, (uint16_t)pos, 20);
}
