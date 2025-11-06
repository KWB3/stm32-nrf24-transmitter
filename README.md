# stm32-nrf24-transmitter

STM32F103C8T6(Blue Pill)보드를 이용해 조이스틱 입력값과 버튼 신호를 NRF24L01P 무선 모듈로 송신하는 송신기 프로젝트


사용 부품 / 모듈 목록

STM32F103C8T6 (Blue Pill) x1 - 메인 MCU, 송신 제어 및 패킷 처리

조이스틱 모듈 (5핀, 2축) x2 - 총 4개 ADC 입력 (X1, Y1, X2, Y2)

OLED 디스플레이 (0.96", I2C, SSD1306) x1 - 조이스틱 및 버튼 상태 표시

NRF24L01P 무선 통신 모듈 x1 - 2.4GHz 무선 송신 담당

푸시버튼 x4 - 기능 입력용, 풀업 구성

저항 4.7kΩ x4 - 버튼 풀업용

세라믹 캐패시터 0.1µF x4 - 디바운스 및 노이즈 제거

LM2596S DC-DC 모듈 x1 - 안정적인 전원 공급 (7.4V → 3.3V 변환)

만능기판 (9cm × 15cm) x1 - 전체 회로 납땜 및 구성


송신 데이터 패킷

|HEADER(0xAB)|시퀀스|x1|y1|x2|y2|btn_flags|checksum|TAIL(0xBA)|

Byte0   : 0xAB        // HEADER (시작 마커)

Byte1   : seq         // 시퀀스 번호 (0 ~ 255)

Byte2   : x1          // 좌스틱 X  0~255

Byte3   : y1          // 좌스틱 Y 

Byte4   : x2          // 우스틱 X 

Byte5   : y2          // 우스틱 Y 

Byte6   : btn_flags   // 버튼 비트맵: bit0..3=버튼1..4, 나머지 예약

Byte7   : checksum    // simple checksum (sum of bytes1..6) & 0xFF

Byte8   : 0xBA


개발 환경

IDE: STM32CubeIDE

언어: C

디버깅: ST-Link V2




https://youtu.be/m-f6WffGbFc
