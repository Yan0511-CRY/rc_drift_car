/**
 * oled.h — SSD1306 0.96寸 OLED I2C 驱动
 *
 * 分辨率: 128×64
 * I2C地址: 0x3C (SA0=GND)
 * 通信接口: I2C2 (PB3-SCL / PB10-SDA, 用户自行在CubeMX配置)
 * 字体: 6×8 ASCII, 21列×8行
 *
 * 使用方法:
 *   OLED_Init(&hi2c2);
 *   OLED_Clear();
 *   OLED_SetCursor(0, 0);
 *   OLED_Print("Hello");
 *   OLED_Refresh();  // 全屏刷新
 */

#ifndef OLED_H
#define OLED_H

#include "system_config.h"

/* ==================== SSD1306 硬件参数 ==================== */
#define OLED_I2C_ADDR       0x3C
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_PAGES          (OLED_HEIGHT / 8)   /* 8页 */

/* 字体尺寸 */
#define OLED_FONT_W          6
#define OLED_FONT_H          8
#define OLED_COLS            (OLED_WIDTH / OLED_FONT_W)   /* 21列 */
#define OLED_ROWS            (OLED_HEIGHT / OLED_FONT_H)  /* 8行 */

/* ==================== API ==================== */
void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_Fill(unsigned char data);
void OLED_SetCursor(unsigned char col, unsigned char row);
void OLED_Print(const char *str);
void OLED_PrintInt(int32_t val);
void OLED_PrintFloat(float val, unsigned char int_digits, unsigned char frac_digits);
void OLED_Refresh(void);

/* 快捷：清屏→绘制→刷新，一步到位 */
void OLED_ClearAndRefresh(void);

#endif /* OLED_H */
