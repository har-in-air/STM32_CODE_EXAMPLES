/* USER CODE BEGIN Header */
// DevEBox STM32F407VGT6 dev board
// Demonstrates reception of PDM data from an ST MP45DT02 PDM microphone on I2S2 port
// I2S2 is configured as timing slave to I2S3 ( BCK and WS taken from I2S3 )
// I2S3 configured as master transmit 16bit in 16bit frame, Fs = 32khz.
// So PDM clock frequency = BCK = 32*32 = 1.024
// Incoming DMA stream PDM data is converted to PCM using the
// PDM2PCM library. The PCM data is transmitted using DMA to I2S3 to a MAX98357A I2S
// power amplifier and speaker. A FIFO is used between the filter PCM output and
// the transmit as I2S2 and I2S3 are enabled at different times.
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define OUT_FS_KHZ  			32

// I2S3 configured for Fs = 32khz, 16bit data in 16bit Frame => BCK = 32*32 = 1.024MHz
// Note : 1MHz is the minimum clock for the PDM microphone. So we won't be
// able to use a lower Fs
#define DECIMATION_FACTOR		32

#define PDM2PCM_OUT_SAMPLES 		OUT_FS_KHZ
#define PDM2PCM_IN_SAMPLES_BYTES	(OUT_FS_KHZ*DECIMATION_FACTOR/8)
#define PDM2PCM_IN_SAMPLES_HWORDS   (PDM2PCM_IN_SAMPLES_BYTES/2)

#define RXBUF_NSAMPLES 			(PDM2PCM_IN_SAMPLES_HWORDS * 2)
#define TXBUF_NSAMPLES 			(PDM2PCM_OUT_SAMPLES*2*2)

uint16_t PDMRxBuf[RXBUF_NSAMPLES] = {0};
// PDM2PCM library generates PDM2PCM_OUT_SAMPLES for each call
uint16_t PCMBuf[PDM2PCM_OUT_SAMPLES];
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
  /* USER CODE BEGIN 2 */

  // The I2S peripherals internally transfer data in 16bit chunks, so the rx and tx buffers need
  // to be half-word buffers.
  // we've configured I2S2 for 16bit data in 16bit frame, HAL API requires us to request the
  // number of half-words
  HAL_I2S_Transmit_DMA(&hi2s3, I2STxBuf, TXBUF_NSAMPLES);

  // I2S2 is a timing slave (BCK and WS from IS3) so needs to start up after I2S3
  HAL_I2S_Receive_DMA(&hi2s2, PDMRxBuf, RXBUF_NSAMPLES);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if (RxState == DMA_HALF_COMPLETE) {
	    	PDM_Filter(PDMRxBuf, PCMBuf, &PDM1_filter_handler);
	    	for (int i = 0; i < PDM2PCM_OUT_SAMPLES; i++) {
	    		FifoWrite(PCMBuf[i]);
	    		}
	    	// enough of a buffer to start output streaming
	    	if ((FIFOwPtr - FIFOrPtr) > 128) {
	    		FIFOReadEnabled = 1;
	    		}
	    	RxState = DMA_IN_PROGRESS;
	    	}

	    if (RxState == DMA_COMPLETE) {
	    	PDM_Filter(&PDMRxBuf[RXBUF_NSAMPLES/2], PCMBuf, &PDM1_filter_handler);
	    	for (int i = 0; i < PDM2PCM_OUT_SAMPLES; i++) {
	    		FifoWrite(PCMBuf[i]);
	    		}
	    	RxState = DMA_IN_PROGRESS;
	    	}

	    if (TxState == DMA_HALF_COMPLETE) {
	    	if (FIFOReadEnabled == 1) {
				for (int i = 0; i < PDM2PCM_OUT_SAMPLES; i++) {
					uint16_t data = FifoRead();
					I2STxBuf[2*i] = data; // L channel (16 bits in 16bit frame)
					I2STxBuf[2*i + 1] = data; //  set R channel to L channel
					}
	    		}
	    	TxState = DMA_IN_PROGRESS;
	    	}

	    if (TxState == DMA_COMPLETE) {
	    	if (FIFOReadEnabled == 1) {
	    		uint16_t* pBuf = &I2STxBuf[TXBUF_NSAMPLES/2];
				for (int i = 0; i < PDM2PCM_OUT_SAMPLES; i++) {
					uint16_t data = FifoRead();
					pBuf[2*i] = data; // L channel (16 bits in 16bit frame)
					pBuf[2*i + 1] = data; //  set R channel to L channel
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
  hi2s2.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
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
