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
#include "stm32l4xx_hal_uart.h"
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
UART_HandleTypeDef huart1;		// UFM-01 (flow sensor)
UART_HandleTypeDef huart2;		// PC/VCP

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);
//static void MC_ADC1_Init(void);

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

// SANITY CHECK HELPER
static void log_hex(uint8_t b) {
    char s[6];
    int n = snprintf(s, sizeof(s), "%02X ", b);
    HAL_UART_Transmit(&huart2, (uint8_t*)s, n, 50);
}

// Look for tags and publish one line on USART2
static void parse_and_publish(int n_bytes) {
	if (n_bytes < 8) return;		// too short to be valid

	// Stop byte and checksum positions depend on frame length
	if (pkt[n_bytes - 1] != 0x16) return;		// must end with 0x16


	// Compute checksum over everything up to ST2 (the two status bytes before checksum)
	// Checksum is byte 37 (before 0x16) and covers bytes 0..ST2
	uint8_t cs_rx = pkt[n_bytes - 2];
	uint8_t cs_calc = cs_sum(pkt, n_bytes - 2);
	if (cs_rx != cs_calc) return;		// Bad checksum; ignore

	// Verify header: 0x3C, second start byte is mode-dependent
	if (pkt[0] != 0x3C) return;
	uint8_t start2 = pkt[1];		// 0x32 (active), 0x96 (passive+ID), 0x64 (passive no-ID)

	// Search tags: Acc Flow (0x0A, 0x1A), Inst Flow (0x0B), Temp (0x0D)
	int acc_flag = -1, inst_flag = -1, temp_flag = -1;
	for (int i = 0; i < n_bytes; ++i) {
		// Accumulated flow flag can be 0x0A or 0x1A per fatasheet
		if ((pkt[i] == 0x0A || pkt[i] == 0x1A) && acc_flag < 0) acc_flag = i;
		if (pkt[i] == 0x0B && inst_flag < 0) inst_flag = i;
		if (pkt[i] == 0x0D && temp_flag < 0) temp_flag = i;
	}

	// Instant flow (0x0B): byte [i+1.. i+4] value LE, [i+5] sign/scale
	float inst_lph = NAN;
	if (inst_flag >= 0 && inst_flag + 5 < n_bytes) {
		uint32_t v = (uint32_t)pkt[inst_flag+1]
					| ((uint32_t)pkt[inst_flag+2] << 8)
					| ((uint32_t)pkt[inst_flag+3] << 16)
					| ((uint32_t)pkt[inst_flag+4] << 24);
		uint8_t sign_scale = pkt[inst_flag+5];
		// LSB = 0.01 L/h; sign bit (Bit7) indicates negative when set
		float f = (float)v * 0.01f;
		if (sign_scale & 0x80) f = -f;
		inst_lph = f;
	}

	// Temperature (0x0D): next 3 bytes LE, value / 100 degC
	float temp_c = NAN;
	if (temp_flag >= 0 && temp_flag + 3 < n_bytes) {
		uint32_t t = (uint32_t)pkt[temp_flag+1]
					| ((uint32_t)pkt[temp_flag+2] << 8)
					| ((uint32_t)pkt[temp_flag+3] << 16);
		temp_c = (float)t / 100.0f;
	}

	// Accumulated flow: 6 bytes (4B value LE, then 2 bytes LSB/mode)
	// Only decode the 4B numeric part for display in liters or m^3 depending on flag
	double acc_val = NAN;
	if (acc_flag >=0 && acc_flag + 6 < n_bytes) {
		uint32_t v = (uint32_t)pkt[acc_flag+1]
					| ((uint32_t)pkt[acc_flag+2] << 8)
					| ((uint32_t)pkt[acc_flag+3] << 16)
					| ((uint32_t)pkt[acc_flag+4] << 24);
		// LSB shown: 0.001 L if flag 0x0A, 0.001 m^3 if 0x1A
		if (pkt[acc_flag] == 0x0A)	acc_val = (double)v * 0.001;	//litres
		else /*0x1A*/				acc_val = (double)v * 0.001;	//m^3
	}

	char line[128];
	int n = snprintf(line, sizeof(line),
			"mode=%02X, inst=%.2f, temp=%s, acc=%s, len=%d\r\n",
			start2,
			isnan(inst_lph) ? NAN : inst_lph,
			isnan(temp_c) ? "NA" : ({ static char tbuf[16]; snprintf(tbuf, sizeof(tbuf), "%.2fC", temp_c); tbuf; }),
			isnan(acc_val)	? "NA" : ({ static char abuf[24]; snprintf(abuf, sizeof(abuf), "%.3f", acc_val); abuf; }),
			n_bytes);

	HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 50);
}

// HAL RX complete callback: re-arm and parse
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uint8_t b = rx_byte;
		log_hex(b); 		//sanity check

		if (pkt_len < (int)sizeof(pkt)) {
			pkt[pkt_len++] = b;

			// Parse on STOP 0x16 or if buffer too long (resync)
			if (b == 0x16) {					// Stop byte per datasheet
				parse_and_publish(pkt_len);		// pkt_len = total bytes incl 0x16
				pkt_len = 0;
			} else if (pkt_len >= (int)sizeof(pkt) - 1) {
				pkt_len = 0;					// overflow -> resync
			}
		} else {
			pkt_len = 0;
		}

		// Re-arm for next byte
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);	// re-arm
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uint32_t err = HAL_UART_GetError(huart);
		// Clear common error flags (PE/FE/NE/ORE)
		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);

		// Re-arm single-byte RX
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

		// Print error to PC
		char msg[48];
		int n = snprintf(msg, sizeof(msg), "UART1 ERR=%081X\r\n", err);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, n, 50);
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
  MX_GPIO_Init();
  MX_USART1_UART_Init();		// UFM-01 @ 2400 8E1
  MX_USART2_UART_Init();		// PC log @ 115200 8N1

  /* USER CODE BEGIN 2 */

 // Start IRQ receive from sensor
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  const char *banner = "NUCLEO-L432KC UFM-01 bridge ready\r\n";
//  const char *hb = "HB\r\n";
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);	// start RX first
  HAL_UART_Transmit(&huart2, (uint8_t*)banner, strlen(banner), 50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
	  // HAL_UART_Transmit(&huart2, (uint8_t*)hb, 4, 50);		// sanity check
	  // HAL_Delay(500);

	uint32_t now = HAL_GetTick();
	if (now - last_poll_ms >= 250) {		// poll ~4Hz
		last_poll_ms = now;
		HAL_UART_Transmit(&huart1, (uint8_t*)UFM_READ_CMD, sizeof(UFM_READ_CMD), 20);
	}
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
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

	// Enable IRQ for interrupt-driven RX
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;				// 8 data + parity
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;					// 8E1
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;  	//HEllo
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
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

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;				// 8N1
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;		//Hello
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();		// No user GPIO required; UART pins configured in their init functions

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
