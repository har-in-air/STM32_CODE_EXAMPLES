/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
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
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////////////////////////////////////////////////////////////////////////
//
// Direct Digital Synthesis of I2S (24bit @ 48KHz) waveform
// (in this example, a sine wave).
// User configurable inputs are :
// Desired frequency = SINE_WAVE_FREQ_HZ
// Amplitude = VOLUME
// Implemented on WeAct STM32F411CEU6 Black Pill dev board. Will also
// work with trivial mods on the cheaper STM32F401CCU6 Black Pill board.
// credits : https://github.com/dimtass/stm32f407_dds_dac,
//           https://www.youtube.com/watch?v=YDC5zaEZGhM
//           https://github.com/YetAnotherElectronicsChannel/STM32_PDM_Microphone
///////////////////////////////////////////////////////////////////////////////


#define SINE_WAVE_FREQ_HZ 	1000.0f
#define VOLUME				0.25f // [0.001 - 0.999]

///////////////////////////////////////////////////////////////////////////////

#define _2PI            6.283185307f
#define _PI             3.14159265f

#define FS_HZ				48000.0f

#define SINE_TABLE_SIZE 	1024

// convert float to 24bit 2's complement
#define SCALE_24BITS(fval)  	( (fval) * (((int32_t)1) << 23) )

// 16.16 resolution fixed point values
#define FLOAT_2_FIXED16(fval)  	( (fval) * (((uint32_t)1) << 16) )

// 24bit 2's complement sine wave table
int32_t SineTable[SINE_TABLE_SIZE + 1];	// +1 for interpolation

#define MAX_PHASE_ACCUM (((uint32_t)SINE_TABLE_SIZE) << 16)

uint32_t PhaseAccumulator = 0;
uint32_t PhaseIncrement;

#define TXBUF_SIZE 	512
// TxBuf contains stereo L, R 24bit samples encoded as [16msb,8lsb:0]
// i.e. each stereo sample requires 4 uint16_ts, so TxBuf[512]
// contains 128 stereo 24bit samples
uint16_t TxBuf[TXBUF_SIZE];

enum DmaState {DMA_IN_PROGRESS, DMA_HALF_COMPLETE, DMA_COMPLETE};
enum DmaState TxState = DMA_IN_PROGRESS;

void sineTable_Init(void);
void dds_Calculate(uint16_t* buffer, int num_samples);

void i2s_ClockConfig();
HAL_StatusTypeDef i2s_Config_I2SPR(uint32_t regVal);
void print_Msg(char* format, ...);

#define TOGGLE_RED_LED() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
#define TOGGLE_GRN_LED() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);

#define BLU_LED_ON()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
#define BLU_LED_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);


// dma completion state

void HAL_I2S_TxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
	TxState = DMA_HALF_COMPLETE;
	}

void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef *hi2s) {
	TxState = DMA_COMPLETE;
	}

void sineTable_Init(void){
	// table[k] = sin(2*pi*k/N)
	for(int inx = 0; inx < SINE_TABLE_SIZE; inx++) {
		float sineval = VOLUME * sinf(_2PI * ((float)inx/(float)SINE_TABLE_SIZE) );
		SineTable[inx] = SCALE_24BITS(sineval);
		}
	// Used for interpolating when integer part of PhaseAccumulator == SINE_TABLE_SIZE-1
	SineTable[SINE_TABLE_SIZE] = SineTable[0];
	}


void dds_Calculate(uint16_t* buffer, int num_samples) {
	// @ Fs = 48000Hz, it takes 64/48000 = 1.333mS to transmit 64 stereo samples
	// The DDS calculation needs to execute in less than this time.
	// Running at 96MHz the STM32F411CEU6 completes this in 61.3uS without interpolation, 83.2uS with interpolation
	BLU_LED_OFF(); // monitor pulse strobe width on logic analyzer or scope for execution time
	for(int inx = 0; inx < num_samples; inx++) {
		PhaseAccumulator += PhaseIncrement;
		while (PhaseAccumulator >= MAX_PHASE_ACCUM) PhaseAccumulator -= MAX_PHASE_ACCUM;
		uint32_t tableIndex = (PhaseAccumulator >> 16); // integer portion of PhaseAccumulator

		// without interpolation
		// int32_t sine24 = SineTable[tableIndex];

		// interpolated value
		int32_t v1 = SineTable[tableIndex]; // nearest neighbours
		int32_t v2 = SineTable[tableIndex+1];
		uint32_t fractional = PhaseAccumulator & 65535;
		int32_t sine24 = v1 + ((v2-v1)*(int32_t)fractional)/65536; // linear interpolation
		uint32_t usine24 =  (uint32_t) sine24;
		// l and r channels are identical 24bit 2's complement left-justified in a 32bit frame
		buffer[4*inx] = buffer[4*inx+2] = (uint16_t)((usine24 >> 8) & 0x0000FFFF);
		buffer[4*inx + 1] = buffer[4*inx+3] = (uint16_t)((usine24 & 0x000000FF) << 8);
    	}
	BLU_LED_ON();
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  print_Msg("\r\nDDS generation of 24-bit I2S stereo 1kHz sine wave @ Fs = 48kHz\r\n");
  // initialize sine wave table
  sineTable_Init();
  // table increment for every generated sinewave sample in 16.16 fixed point resolution
  PhaseIncrement = FLOAT_2_FIXED16(SINE_WAVE_FREQ_HZ * (float)SINE_TABLE_SIZE / FS_HZ );
  BLU_LED_ON(); // strobe used for monitoring execution time of DDS buffer recalculation

  // populate entire TxBuf
  dds_Calculate(TxBuf, TXBUF_SIZE/4);

  // set i2s clock pll registers for 48kHz 24bit stream
  i2s_ClockConfig();

  // The I2S peripheral internally transfers data in 16bit chunks, so the source buffer needs
  // to be uint16_t.
  // When I2S2 is configured for 24bit data in 32bit frame, the HAL API for I2S tx/rx DMA
  // requires us to specify the number of uint32_t to be transferred, and then transfers twice that
  // number of uint16_ts (shrug)
  // Tx source is a circular buffer with 128 stereo 24bit samples.
  // We re-calculate 64 samples while the other 64 samples are being transmitted.
  HAL_I2S_Transmit_DMA(&hi2s2, TxBuf, TXBUF_SIZE/2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if (TxState == DMA_HALF_COMPLETE) {
	    	// DMA has transmitted the first half of TxBuf, we
	    	// can re-calculate it  while DMA transmits the second half
	    	// each half contains 64 stereo samples
     	    dds_Calculate(&TxBuf[0], TXBUF_SIZE/8);
	    	TxState = DMA_IN_PROGRESS;
	    	TOGGLE_RED_LED();
	    	}

	    if (TxState == DMA_COMPLETE) {
	    	// DMA has transmitted the second half of TxBuf, we
	    	// can re-calculate it while DMA transmits the first half
     	    dds_Calculate(&TxBuf[TXBUF_SIZE/2], TXBUF_SIZE/8);
	    	TxState = DMA_IN_PROGRESS;
	    	TOGGLE_GRN_LED();
	    	}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
#ifdef  STM32F411xE
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
#endif
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB3 PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


// I2S clock configuration for Fs=48kHz, 24/32 data format
// ref : Tech Ref Manual RM0383
void i2s_ClockConfig() {
	RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;
    uint32_t N, R, I2SDIV, ODD, MCKOE, I2S_PR;
#ifdef USE_MCLK_OUT
    MCKOE = 1;
#else
    MCKOE = 0;
#endif
    // PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input)  (PLLI2SN/PLLM)
    // I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR

    HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
    N = 384;
    R = 5;
    I2SDIV = 12;
    ODD = 1;

    RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
#ifdef STM32F411xE
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SM = 25; // want 1Mhz clock at PLL input
#endif
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = N;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SR = R;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
    I2S_PR = (MCKOE<<9) | (ODD<<8) | I2SDIV;
    i2s_Config_I2SPR(I2S_PR);
	}


HAL_StatusTypeDef i2s_Config_I2SPR(uint32_t regVal) {
	uint32_t tickstart = 0U;
    __HAL_RCC_PLLI2S_DISABLE();
    tickstart = HAL_GetTick();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  != RESET) {
    	if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE) {
    		return HAL_TIMEOUT;
         	 }
      	}

    SPI2->I2SPR = regVal;

    __HAL_RCC_PLLI2S_ENABLE();
    tickstart = HAL_GetTick();
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLI2SRDY)  == RESET)    {
    	if((HAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)      {
    		return HAL_TIMEOUT;
    		}
    	}
   return HAL_OK;
   }


void print_Msg(char* format, ...) {
	char sz[100];
	va_list args;
	va_start(args, format);
	vsprintf(sz, format, args);
	HAL_UART_Transmit(&huart2, (uint8_t *)sz, strlen(sz), HAL_MAX_DELAY);
	va_end(args);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
