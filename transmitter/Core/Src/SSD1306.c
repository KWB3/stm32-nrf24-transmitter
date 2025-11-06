#include "ssd1306.h"
#include "font_5x7.h"
#include "string.h"
// I2C 핸들러
static I2C_HandleTypeDef *oled_i2c;

// 디스플레이 버퍼 (128x64 해상도, 1비트/픽셀 -> 128 * 64 / 8 = 1024 바이트)
static uint8_t display_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// 초기화 명령어 (SSD1306 데이터시트 기반)
static const uint8_t init_sequence[] = {
    0xAE,       // 디스플레이 OFF
    0xD5, 0x80, // 클럭 설정
    0xA8, 0x3F, // 멀티플렉스 비율 (64)
    0xD3, 0x00, // 디스플레이 오프셋
    0x40,       // 시작 라인
    0x8D, 0x14, // 차지 펌프 활성화
    0x20, 0x00, // 메모리 주소 모드 (수평)
    0xA1,       // 세그먼트 리매핑
    0xC8,       // COM 출력 스캔 방향
    0xDA, 0x12, // COM 핀 설정
    0x81, 0xCF, // 콘트라스트 설정
    0xD9, 0xF1, // 프리차지 기간
    0xDB, 0x40, // VCOMH 설정
    0xA4,       // 전체 디스플레이 ON
    0xA6,       // 정상 디스플레이 (A7: 반전)
    0xAF        // 디스플레이 ON
};

// I2C 초기화
void SSD1306_Init(I2C_HandleTypeDef *hi2c) {
    oled_i2c = hi2c;
    // 초기화 명령어 전송
    for (uint8_t i = 0; i < sizeof(init_sequence); i++) {
        SSD1306_WriteCommand(init_sequence[i]);
    }
    SSD1306_Clear();
    SSD1306_UpdateScreen();
}

// 명령어 전송
void SSD1306_WriteCommand(uint8_t command) {
    uint8_t data[2] = {0x00, command}; // 제어 바이트: 0x00 (명령)
    HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
}

// 데이터 전송
void SSD1306_WriteData(uint8_t *data, uint16_t size) {
    uint8_t buffer[1 + size];
    buffer[0] = 0x40; // 제어 바이트: 0x40 (데이터)
    memcpy(&buffer[1], data, size);
    HAL_I2C_Master_Transmit(oled_i2c, OLED_I2C_ADDRESS << 1, buffer, size + 1, HAL_MAX_DELAY);
}

// 화면 지우기
void SSD1306_Clear(void) {
    memset(display_buffer, 0, sizeof(display_buffer));
}

// 화면 업데이트
void SSD1306_UpdateScreen(void) {
    uint8_t page;
    for (page = 0; page < OLED_HEIGHT / 8; page++) {
        SSD1306_WriteCommand(0xB0 + page); // 페이지 주소 설정
        SSD1306_WriteCommand(0x00); // 열 시작 주소 (낮은 4비트)
        SSD1306_WriteCommand(0x10); // 열 시작 주소 (높은 4비트)
        SSD1306_WriteData(&display_buffer[page * OLED_WIDTH], OLED_WIDTH);
    }
}

// 픽셀 설정
void SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t state) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint16_t index = x + (y / 8) * OLED_WIDTH;
    uint8_t bit = 1 << (y % 8);
    if (state) {
        display_buffer[index] |= bit;
    } else {
        display_buffer[index] &= ~bit;
    }
}
/*
void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t font_size) {
    if (ch < 32 || ch > 126) return; // 지원하지 않는 문자 무시
    uint8_t font_index = ch - 32; // ASCII 오프셋 (32부터 시작)
    for (uint8_t i = 0; i < 5; i++) { // 5열 순회
        uint8_t column = font_5x7[font_index][i];
        for (uint8_t j = 0; j < 7; j++) { // 7행 순회
            SSD1306_SetPixel(x + i, y + j, (column >> j) & 0x01);
        }
    }
}
*/

void SSD1306_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t font_size) {
    uint8_t font_index;
    if (ch >= 32 && ch <= 126) {
        font_index = ch - 32; // ASCII 32~126
    } else if (ch == 127) {
        font_index = 95; // °C (새로운 문자)
    } else {
        return; // 지원하지 않는 문자
    }
    for (uint8_t i = 0; i < 5; i++) { // 5열 순회
        uint8_t column = font_5x7[font_index][i];
        for (uint8_t j = 0; j < 7; j++) { // 7행 순회
            SSD1306_SetPixel(x + i, y + j, (column >> j) & 0x01);
        }
    }
}

void SSD1306_DrawString(uint8_t x, uint8_t y,const char *str, uint8_t font_size) {
    while (*str) {
        SSD1306_DrawChar(x, y, *str, font_size);
        x += 6; // 5픽셀(문자) + 1픽셀(간격)
        str++;
    }
}
