/* USER CODE BEGIN Header */
// DevEBox STM32F407VGT6 dev board
// Demonstrates reception of PDM data from two  MP45DT02 PDM microphones on I2S2 port
// The two microphones are wired for L and R channels, i.e. generate data on rising
// and falling edges of the clock and tristated otherwise. So their data lines can be
// connected together.
// I2S2 master receive PDM configured for Fs = 32khz, 24bit data in 32bit Frame
// => BCK = 32kHz*64bits = 2.048MHz
// BCK is routed to TIM1_ETR (PE7) and divided by 2 to generate BCK/2 = 1.024MHz
// at TIM1_CH1 (PE9).
// This is the clock for the two PDM microphones. So I2S2 samples at BCK, but the two
// PDM microphones generate 1bit data valid on their BCK/2 rising edge and falling edge
// respectively.
// The result is a stream of interleaved L and R 1-bit data streams on I2S2 data input line.

// Note : 1MHz is the minimum clock for the MP45DT02 PDM microphone.

// I2S3 master transmit I2S configured for Fs = 32khz, 16bit data in 16bit Frame
// => BCK = 32kHz*32bits = 1.024MHz

// Incoming DMA stream PDM data is converted to PCM using the
// PDM2PCM library after de-interleaving the L and R PDM channels and separately
// decimating and filtering them. A FIFO is used between the filter PCM output and
// the transmit as I2S2 and I2S3 are not synced.
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void PDM_DeInterleave(uint16_t* pPDMBuf, int nSamples, uint8_t* pPDMBufL, uint8_t* pPDMBufR);

#define OUT_FS_KHZ  				32
#define DECIMATION_FACTOR			32

#define PDM2PCM_OUT_NSAMPLES 		(OUT_FS_KHZ)
#define PDM2PCM_IN_SAMPLES_BYTES	((OUT_FS_KHZ * DECIMATION_FACTOR)/8)
#define PDM2PCM_IN_SAMPLES_HWORDS   (PDM2PCM_IN_SAMPLES_BYTES/2)

#define RXBUF_NSAMPLES 				(PDM2PCM_IN_SAMPLES_HWORDS * 2 * 2)
#define TXBUF_NSAMPLES 				(PDM2PCM_OUT_NSAMPLES * 2 * 2)

uint16_t PDMRxBuf[RXBUF_NSAMPLES] = {0};

uint8_t PDMBufL[RXBUF_NSAMPLES/2] = {0};
uint8_t PDMBufR[RXBUF_NSAMPLES/2] = {0};

// PDM2PCM library generates PDM2PCM_OUT_NSAMPLES for each call
uint16_t PCMBufL[PDM2PCM_OUT_NSAMPLES];
uint16_t PCMBufR[PDM2PCM_OUT_NSAMPLES];
uint16_t I2STxBuf[TXBUF_NSAMPLES] = {0};

// dma completion state
enum DmaState {DMA_IN_PROGRESS, DMA_HALF_COMPLETE, DMA_COMPLETE};
enum DmaState RxState = DMA_IN_PROGRESS;
enum DmaState TxState = DMA_IN_PROGRESS;

// 256 entry FIFO and uint8_t pointers allow automatic roll-over to 0
uint16_t FIFOBuf[256];
uint8_t FIFOwPtr = 0;
uint8_t FIFOrPtr = 0;

// flag to start reading out from the FIFO when enough data is buffered
int FIFOReadEnabled = 0;

void FifoWrite(uint16_t data) {
	FIFOBuf[FIFOwPtr] = data;
	FIFOwPtr++;
	}

uint16_t FifoRead() {
	uint16_t val = FIFOBuf[FIFOrPtr];
	FIFOrPtr++;
	return val;
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
  MX_CRC_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_PDM2PCM_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // The I2S peripherals internally transfer data in 16bit chunks, so the rx and tx buffers need
  // to be half-word buffers.
  // we've configured I2S2 for 24bit data in 32bit frame, HAL API requires us to request the
  // number of words, it multiplies that by 2 to transfer the required number of half-words

  HAL_I2S_Receive_DMA(&hi2s2, PDMRxBuf, RXBUF_NSAMPLES/2);
  if (HAL_OK != HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)) {
      Error_Handler();
      }

  // we've configured I2S2 for 16bit data in 16bit frame, HAL API requires us to request the
  // number of half-words
  HAL_I2S_Transmit_DMA(&hi2s3, I2STxBuf, TXBUF_NSAMPLES);


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if (RxState == DMA_HALF_COMPLETE) {
	    	PDM_DeInterleave(PDMRxBuf, RXBUF_NSAMPLES/2, PDMBufL, PDMBufR);
	    	PDM_Filter(PDMBufL, PCMBufL, &PDM1_filter_handler);
	    	PDM_Filter(PDMBufR, PCMBufR, &PDM1_filter_handler);
	    	for (int i = 0; i < PDM2PCM_OUT_NSAMPLES; i++) {
	    		FifoWrite(PCMBufL[i]);
	    		FifoWrite(PCMBufR[i]);
	    		}
	    	// enough of a buffer to start output streaming
	    	if ((FIFOwPtr - FIFOrPtr) > 128) {
	    		FIFOReadEnabled = 1;
	    		}
	    	RxState = DMA_IN_PROGRESS;
	    	}

	    if (RxState == DMA_COMPLETE) {
	    	PDM_DeInterleave(&PDMRxBuf[RXBUF_NSAMPLES/2], RXBUF_NSAMPLES/2, PDMBufL, PDMBufR);
	    	PDM_Filter(PDMBufL, PCMBufL, &PDM1_filter_handler);
	    	PDM_Filter(PDMBufR, PCMBufR, &PDM1_filter_handler);
	    	for (int i = 0; i < PDM2PCM_OUT_NSAMPLES; i++) {
	    		FifoWrite(PCMBufL[i]);
	    		FifoWrite(PCMBufR[i]);
	    		}
	    	RxState = DMA_IN_PROGRESS;
	    	}

	    if (TxState == DMA_HALF_COMPLETE) {
	    	if (FIFOReadEnabled == 1) {
				for (int i = 0; i < PDM2PCM_OUT_NSAMPLES; i++) {
					I2STxBuf[2*i] = FifoRead();
					I2STxBuf[2*i + 1] = FifoRead();
					}
	    		}
	    	TxState = DMA_IN_PROGRESS;
	    	}

	    if (TxState == DMA_COMPLETE) {
	    	if (FIFOReadEnabled == 1) {
	    		uint16_t* pBuf = &I2STxBuf[TXBUF_NSAMPLES/2];
				for (int i = 0; i < PDM2PCM_OUT_NSAMPLES; i++) {
					pBuf[2*i] = FifoRead();
					pBuf[2*i + 1] = FifoRead();
				 	}
				}
	    	TxState = DMA_IN_PROGRESS;
	    	}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}



/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_I2S_TxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
	TxState = DMA_HALF_COMPLETE;
}

void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef *hi2s) {
	TxState = DMA_COMPLETE;
}

void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
	RxState = DMA_HALF_COMPLETE;
}

void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef *hi2s) {
	RxState = DMA_COMPLETE;
}

static void MX_TIM1_Init(void){
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1; // we want to divide TIM1_ETR by 2, so period is 2 clocks
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK){
	  Error_Handler();
  	  }
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK){
    Error_Handler();
  }

  // now we need to configure the output channel
  // only set fields required for output channel pwm mode
  TIM_OC_InitTypeDef pwmConfig = {0};
  pwmConfig.OCMode = TIM_OCMODE_PWM1;
  // when counter < CCR1, we want the channel output high
  pwmConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  // we want a square wave at 50% duty cycle, that divides the TIM1_ETR clock by 2
  pwmConfig.Pulse = 1;
  // this function will call HAL_TIM_PWM_MspInit which we need to define
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwmConfig, TIM_CHANNEL_1) != HAL_OK)  {
      Error_Handler();
  	  }

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_base){
	// low level initialization
	if(htim_base->Instance == TIM1)  {
		// enable clock for TIM2 peripheral
		__HAL_RCC_TIM1_CLK_ENABLE();
		// enable clock to GPIOE so we can use PE7 as ETR and PE9 as output channel
		__HAL_RCC_GPIOE_CLK_ENABLE();

		// PE9 : TIM1_CH1
		// PE7 : TIM1_ETR
		GPIO_InitTypeDef config;
		config.Pin = GPIO_PIN_7 | GPIO_PIN_9;
		config.Mode = GPIO_MODE_AF_PP;
	    config.Pull = GPIO_NOPULL;
	    config.Speed = GPIO_SPEED_FREQ_LOW;
		config.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOE, &config);

		// pwm output, no interrupt required
		}
	}

void PDM_DeInterleave(uint16_t* pPDMBuf, int nSamples, uint8_t* pPDMBufL, uint8_t* pPDMBufR){
	uint8_t left, right;
	for (int inx = 0; inx < nSamples; inx++) {
		uint16_t sample = pPDMBuf[inx];
		left = right = 0;
		if (sample & 0x8000) left |= 0x80;
		if (sample & 0x4000) right |= 0x80;
		if (sample & 0x2000) left |= 0x40;
		if (sample & 0x1000) right |= 0x40;
		if (sample & 0x0800) left |= 0x20;
		if (sample & 0x0400) right |= 0x20;
		if (sample & 0x0200) left |= 0x10;
		if (sample & 0x0100) right |= 0x10;
		if (sample & 0x0080) left |= 0x08;
		if (sample & 0x0040) right |= 0x08;
		if (sample & 0x0020) left |= 0x04;
		if (sample & 0x0010) right |= 0x04;
		if (sample & 0x0008) left |= 0x02;
		if (sample & 0x0004) right |= 0x02;
		if (sample & 0x0002) left |= 0x01;
		if (sample & 0x0001) right |= 0x01;
		pPDMBufL[inx] = left;
		pPDMBufR[inx] = right;
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
	int counter = 0;
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		while (counter < 0x3FFFF) {counter++;}
		counter = 0;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
