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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1;	// UFM-01 (flow sensor)
UART_HandleTypeDef huart2;	// PC/VCP

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void Error_Handler(void);
static void MC_ADC1_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UFM-01: Read (no ID) command
static const uint8_t UFM_READ_CMD[7] = {0xFE, 0xFE, 0x11, 0x5B, 0x0F, 0x6A, 0x16};

// RX / Parser State
static uint8_t 	rx_byte;		// single-byte buffer fir IRQ RX
static uint8_t 	pkt[96];		// frame buffer
static int 		pkt_len = 0;
static uint32_t	last_poll_ms = 0;

// Helpers
static uint8_t cs_sum(const uint8_t*b, int n) {
	uint32_t s = 0;
	for (int i = 0; i < n; ++i) s += b[i];
	return (uint8_t)(s & 0xFF);
}

// Look for tags and publish one line on USART2
static void parse_and_publish(int stop_idx) {
	if (stop_idx < 2) return;
	uint8_t cs_calc = cs_sum(pkt, stop_idx);
	uint8_t cs_rx = pkt[stop_idx - 1];
	if (cs_calc != cs_rx) return;

	// Find FLOW tag (0x0B) and TEMP tag (0x0D)
	int flow_tag = -1, temp_tag = -1;
	for (int i = 0; i + 5 < stop_idx && flow_tag < 0; ++i)
		if (pkt[i] == 0x0B) flow_tag = i;
	for (int i =0; i + 4 < stop_idx && temp_tag < 0; ++i)
		if (pkt[i] == 0x0D) temp_tag = i;

	float	flow_lph = NAN, temp_c = NAN;
	uint16_t status = 0;

	if (flow_tag >= 0 && flow_tag + 4 < stop_idx) {
		// 4 bytes little-endian, then optional sign flag (0x80)
		uint32_t uf = (uint32_t)pkt[flow_tag+1]
					| ((uint32_t)pkt[flow_tag+2] << 8)
					| ((uint32_t)pkt[flow_tag+3] << 16)
					| ((uint32_t)pkt[flow_tag+4] << 24);
		int32_t flow_raw = (int32_t)uf;
		flow_lph = flow_raw / 100.0f;
		if (flow_tag + 5 < stop_idx && pkt[flow_tag+5] == 0x80)
			flow_lph = -flow_lph;
	}

	if (temp_tag >= 0 && temp_tag + 3 < stop_idx) {
		// 3 bytes little-endian
		uint32_t ut = (uint32_t)pkt[temp_tag+1]
					| ((uint32_t)pkt[temp_tag+2] << 8)
					| ((uint32_t)pkt[temp_tag+3] << 16);
		temp_c = ut / 100.0f;
	}

	// Fallback status: last two bytes before CS (common placement)
	if (stop_idx >= 3) {
		status = (uint16_t)pkt[stop_idx - 3] | ((uint16_t)pkt[stop_idx - 2] << 8);
	}

	char line[96];
	int n = snprintf(line, sizeof(line),
			"FLOW=%s, TEMP=%s, STAT=%04X\r\n",
			isnan(flow_lph) ? "NA" : ({ static char f[16]; snprintf(f, sizeof(f), "%.2f", flow_lph); f; }),
			isnan(temp_c)	? "NA" : ({ static char t[16]; snprintf(t, sizeof(t), "%.2f", temp_c); t; }),
			status);

	HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 50);
}

// HAL RX complete callback: re-arm and parse
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uint8_t b = rx_byte;

		if (pkt_len < (int)sizeof(pkt)) {
			pkt[pkt_len++] = b;

			// Parse on STOP 0x16 or if buffer too long (resync)
			if (b == 0x16) {
				parse_and_publish(pkt_len - 1);
				pkt_len = 0;
			} else if (pkt_len >= 64) {
				pkt_len = 0;	// no STOP seem -> resync
			}
		} else {
			pkt_len = 0;
		}

		// Re-arm for next byte
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_USART1_UART_Init();	// UFM-01 @ 2400 8E1
  MX_USART2_UART_Init();	// PC log @ 115200 8N1

 // Start IRQ receive from sensor
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  const char *banner = "NUCLEO-L432KC UFM-01 bridge ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)banner, strlen(banner), 50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
	uint32_t now = HAL_GetTick();
	if (now - last_poll_ms >= 250) {		// poll ~4Hz
		last_poll_ms = now;
		HAL_UART_Transmit(&huart1, (uint8_t*)UFM_READ_CMD, sizeof(UFM_READ_CMD), 20);
	}

	// READ COMMANDS FROM PC ON USART2 HERE IF NEEDED
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	// No user GPIO required; UART pins configured in their init functions
}


static void MX_USART1_Init(void)
{
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// PA9 = USART1_TX (AF7), PA10 = USART1_RX (AF7)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= GPIO_PIN_9;
	GPIO_InitStruct.Mode		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		= GPIO_PULLUP;
	GPIO_InitStruct.Speed		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	= GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin			= GPIO_PIN_10;
	GPIO_InitStruct.Mode		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		= GPIO_PULLUP;
	GPIO_InitStruct.Speed		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	= GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	huart1.Instance				= USART1;
	huart1.Init.BaudRate		= 2400;
	huart1.Init.WordLength		= UART_WORDLENGTH_9B;		// 8 data + parity
	huart1.Init.StopBits		= UART_STOPBITS_1;
	huart1.Init.Parity			= UART_PARITY_EVEN;			// 8E1
	huart1.Init.Mode			= UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl		= UART_HWCONTROL_NONE;
	huart1.Init.OverSampling	= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }

	// Enable IRQ for interrupt-driven RX
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_USART2_UART_Init(void)
{
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// PA2 = USART2_TX (AF7), PA15 = USART2_RX (AF3)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin			= GPIO_PIN_2;
	GPIO_InitStruct.Mode		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		= GPIO_PULLUP;
	GPIO_InitStruct.Speed		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	= GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin			= GPIO_PIN_15;
	GPIO_InitStruct.Mode		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull		= GPIO_PULLUP;
	GPIO_InitStruct.Speed		= GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate	= GPIO_AF3_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	huart2.Instance				= USART2;
	huart2.Init.BaudRate		= 115200;
	huart2.Init.WordLength		= UART_WORDLENGTH_8B;
	huart2.Init.StopBits		= UART_STOPBITS_1;
	huart2.Init.Parity			= UART_PARITY_NONE;		// 8N1
	huart2.Init.Mode			= UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl		= UART_HWCONTROL_NONE;
	huart2.Init.OverSampling	= UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
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

#ifdef  USE_FULL_ASSERT
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
