/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SSD1306.h"
#include "font_5x7.h"
#include "nrf24l01p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define UART_TEST
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*
Byte0   : 0xAB          // HEADER (시작 마커)
Byte1   : seq           // 시퀀스 번호 (0..255)
Byte2   : x1  	  		// 좌스틱 X (0~255)
Byte3   : y1   			// 좌스틱 Y (0~255)
Byte4   : x2 		 	// 우스틱 X (0~255)  
Byte5   : y2	 	 	// 우스틱 Y (0~255)
Byte6   : btn_flags		// 버튼 비트맵: bit0..3=버튼1..4, 나머 예약
Byte7   : checksum		// simple checksum (예: sum of bytes1..6) & 0xFF
Byte8   : 0xBA          // TAIL (끝 마커)
 */
volatile uint16_t adcval[4];
uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = {0};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3,(uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}

void oledUpdate(const char *x1, const char *y1, const char *x2, const char *y2)
{
	SSD1306_Clear();
	SSD1306_DrawString(4, 0, x1, 7);
	SSD1306_DrawString(4, 12, y1, 7);
	SSD1306_DrawString(4, 24, x2, 7);
	SSD1306_DrawString(4, 36, y2, 7);
	SSD1306_UpdateScreen();
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	char x1[15];
	char y1[15];
	char x2[15];
	char y2[20];
	uint8_t bt_state;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(200);	// oled 전원 안정화를 위한 딜레이 추가 .
	SSD1306_Init(&hi2c1);
	HAL_ADC_Start_DMA(&hadc1, adcval, 4);
	nrf24l01p_tx_init(2500, _1Mbps);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	uint8_t count = 0;
	tx_data[0] = 0xAB;
	tx_data[8] = 0xBA;

#ifdef UART_TEST
	uint8_t tx_text[50] ={0};
#endif

	while (1)
	{
		bt_state = !HAL_GPIO_ReadPin(bt1_GPIO_Port, bt1_Pin)<<0 | !HAL_GPIO_ReadPin(bt2_GPIO_Port, bt2_Pin)<<1 | !HAL_GPIO_ReadPin(bt3_GPIO_Port, bt3_Pin)<<2 | !HAL_GPIO_ReadPin(bt4_GPIO_Port, bt4_Pin)<<3;

		if(count > 255)
		{
			count = 0;
		}
		tx_data[1] = count;
		tx_data[2] = (uint8_t)((adcval[0]*255)/4095);
		tx_data[3] = (uint8_t)((adcval[1]*255)/4095);
		tx_data[4] = 255 - (uint8_t)((adcval[2]*255)/4095);
		tx_data[5] = 255 - (uint8_t)((adcval[3]*255)/4095);
		tx_data[6] = bt_state;
		tx_data[7] = (tx_data[1] + tx_data[2] + tx_data[3]+tx_data[4]+tx_data[5]+tx_data[6]) &0xFF;

		sprintf(x1, "x1 = %03d", tx_data[2]);
		sprintf(y1, "y1 = %03d", tx_data[3]);
		sprintf(x2, "x2 = %03d", tx_data[4]);
		sprintf(y2, "y2 = %03d, 0x%02X", tx_data[5], bt_state);

		oledUpdate(x1, y1, x2, y2);

#ifdef UART_TEST
		sprintf(tx_text, "%02x, %02x, x1=%02x, y1=%02x, x2=%02x, y2=%02x, %02x, %02x, %02x \r\n", tx_data[0], tx_data[1], tx_data[2], tx_data[3], tx_data[4], tx_data[5], tx_data[6], tx_data[7], tx_data[8]);
		HAL_UART_Transmit(&huart3, tx_text, 50, HAL_MAX_DELAY);
#endif
		nrf24l01p_tx_transmit(tx_data);
		HAL_Delay(20);
		count++;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
	/* EXTI2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
	{
		nrf24l01p_tx_irq();
	}

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
