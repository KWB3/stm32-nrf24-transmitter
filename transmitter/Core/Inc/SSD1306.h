#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f1xx_hal.h" // HAL 라이브러리

// OLED 설정
#define OLED_I2C_ADDRESS    0x3C // SSD1306 I2C 주소 (7비트, 보통 0x3C 또는 0x3D)
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

// 함수 프로토타입
void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_WriteCommand(uint8_t command);
void SSD1306_WriteData(uint8_t *data, uint16_t size);
void SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t state);
void SSD1306_UpdateScreen(void);
void SSD1306_Clear(void);
void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t font_size);
void SSD1306_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t font_size);

#endif
