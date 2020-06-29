/* USER CODE BEGIN Header */
// DevEBox STM32F407VGT6 dev board
// Stream 24bit @ 32kHz I2S data from INMP441 microphone and compute realtime FFT
// with fixed point Q15 and floating point F32 arm cmsis FFT library calls.
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
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
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define BUFFER_SIZE 128


// input buffer receiving  I2S stream from INMP441 microphone
uint16_t RxBuf[BUFFER_SIZE*8];

// rx dma completion state
enum RxDmaState {RXBUF_WAIT, RXBUF_HALF_FULL, RXBUF_FULL};
enum RxDmaState RxState = RXBUF_WAIT;


// ping-pong buffers, one receives data from input stream while other is
// processed for FFT.
// Note : the incoming data needs to be copied as the FFT call does
// in-place processing on the input buffer (i.e. destroys the contents).
q15_t 	Buf[2][BUFFER_SIZE];
q15_t* 	pBufRcv;
q15_t* 	pBufProc;
int 	RcvInx;

// test fixed point arithmetic versus floating point fft
//#define DO_FFT_Q15
#define DO_FFT_F32


#ifdef DO_FFT_Q15

// fft has real and imaginary components interleaved [r0,i0,r1,i1,...]
q15_t FFT_Q15[BUFFER_SIZE*2];

// fft magnitude
q15_t FFTMag_Q15[BUFFER_SIZE];

arm_rfft_instance_q15  		realFFT_inst;
#endif

#ifdef DO_FFT_F32
// extra array needed to convert Q15 windowed data into F32 input data
float Buf_F32[BUFFER_SIZE];

// fft has real and imaginary components
float FFT_F32[BUFFER_SIZE*2];
// fft magnitude
float FFTMag_F32[BUFFER_SIZE];

arm_rfft_fast_instance_f32  realFFT_inst;
#endif


const q15_t Hanning[];

void printMsg(char* format, ...) {
	char sz[100];
	va_list args;
	va_start(args, format);
	vsprintf(sz, format, args);
	HAL_UART_Transmit(&huart1, (uint8_t *)sz, strlen(sz), HAL_MAX_DELAY);
	va_end(args);
	}

void processData(void);


// on button press, print out the FFT magnitude array to uart
int DumpFFTFlag = 0;

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	RcvInx = 0;
	pBufRcv = Buf[RcvInx];
	pBufProc = Buf[1-RcvInx];

#ifdef DO_FFT_Q15
    // length of real data array = 128, inverseFlag = 0, doBitReverse = 1
    if (ARM_MATH_SUCCESS != arm_rfft_init_q15(&realFFT_inst, BUFFER_SIZE, 0, 1)){
    	Error_Handler();
    	}
#endif

#ifdef DO_FFT_F32

    if (ARM_MATH_SUCCESS != arm_rfft_fast_init_f32(&realFFT_inst, BUFFER_SIZE)) {
    	Error_Handler();
    	}
#endif

  // The I2S stream has 32bit data frames. The 0, 2, 4 .. indices are the mic data samples (I2S L channel),
  // the 1,3,5 ... frames are 0 (I2S R channel).
  // In the 32bit L frame the data is packed as 24bit MSb first, we will just use the upper 16 bits.
  // The I2S peripheral can only transfer data in 16bit chunks, so RxBuf is a 16bit data buffer.
  // When configured for 24bit data, we need to call the receive DMA api with the number of 32bit frames, and
  // the function will transfer twice that number of half-words. So if we call with 512, we will get 1024 half-words
  // on receive complete.
  // As per the above description, only 1 of 4 half-words is significant, and we need to extract every 4th half-word
  // from RxBuf for processing.
  // This call asks for BUFFER_SIZE*4*2 half-words
  HAL_I2S_Receive_DMA(&hi2s2, RxBuf, BUFFER_SIZE*4);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// DMA uses RxBuf in circular mode, so we expect to get a repeating sequence of
	// RXBUF_HALF_FULL and RXBUF_FULL events
	// interval between PA5 toggles gives us the transfer time for BUFFER_SIZE mic data samples
	if (RxState == RXBUF_HALF_FULL) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		// extract every 4th half-word from the lower half of RxBuf
		for (int inx = 0; inx < BUFFER_SIZE; inx++) {
			pBufRcv[inx] = RxBuf[4*inx];
			}
		processData();
		RxState = RXBUF_WAIT;
		}
	else
	if (RxState == RXBUF_FULL) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		// extract every 4th half-word from the upper half of rxBuf
		for (int inx = 0; inx < BUFFER_SIZE; inx++) {
			pBufRcv[inx] = RxBuf[(BUFFER_SIZE*4) + (4*inx)];
			}
		processData();
		RxState = RXBUF_WAIT;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void processData(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	// swap the incoming and processing data buffers
	RcvInx ^= 1;
	pBufRcv = Buf[RcvInx];
	pBufProc = Buf[1-RcvInx];
	for(int inx = 0; inx < BUFFER_SIZE; inx++){
		pBufProc[inx] = ((q31_t)pBufProc[inx] * Hanning[inx])>>15;
#ifdef DO_FFT_F32
		Buf_F32[inx] = (float)pBufProc[inx];
#endif
		}

#ifdef DO_FFT_Q15
	// The output FFT has both real and imaginary parts interleaved [r0, i0, r1, i1, ...]
	arm_rfft_q15( &realFFT_inst, pBufProc, FFT_Q15);
    //Scale the input before computing magnitude
    for(int inx = 0; inx < BUFFER_SIZE*2; inx++){
        FFT_Q15[inx] <<= 6;
        }
    //FFT function returns the real + imaginary values.   We need to compute the magnitude
    arm_cmplx_mag_q15(FFT_Q15, FFTMag_Q15, BUFFER_SIZE);
#endif

#ifdef DO_FFT_F32
    arm_rfft_fast_f32(&realFFT_inst, Buf_F32, FFT_F32, 0);
    arm_cmplx_mag_f32(FFT_F32, FFTMag_F32, BUFFER_SIZE);
#endif

    // on PA0 button press, dump the FFT magnitude array to UART.
    // Only the first half is valid data.
    // The frequency spacing between indices corresponds to Fs/BUFFER_SIZE
    // So if we're sampling at Fs = 32kHz, BUFFER_SIZE = 128, our FFT frequency resolution
    // is 32khz/128 = 250Hz
    if (DumpFFTFlag){
    	DumpFFTFlag = 0;
    	printMsg("\r\n");
    	for (int inx = 0; inx < BUFFER_SIZE/2; inx++) {
#ifdef DO_FFT_Q15
    		printMsg("%u,", FFTMag_Q15[inx]);
#endif
#ifdef DO_FFT_F32
    		printMsg("%.1f,", FFTMag_F32[inx]);
#endif
    		}
    	}
    // PA4 pulse high time = fft processing time
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}


void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef *hi2s) {
	RxState = RXBUF_HALF_FULL;
	}


void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef *hi2s) {
	RxState = RXBUF_FULL;
	}


void HAL_GPIO_EXTI_Callback(uint16_t gpioPin) {
	DumpFFTFlag = 1;
	}

// fixed point Q_15 Hanning window array can be computed in Octave with signal package loaded
// w = round(hann(128)*32768)

const q15_t Hanning[BUFFER_SIZE] =
{
    0	,
    20	,
    80	,
    180	,
    320	,
    499	,
    717	,
    973	,
    1267	,
    1597	,
    1965	,
    2367	,
    2804	,
    3273	,
    3775	,
    4308	,
    4871	,
    5461	,
    6078	,
    6721	,
    7387	,
    8075	,
    8784	,
    9511	,
    10255	,
    11014	,
    11786	,
    12569	,
    13362	,
    14162	,
    14967	,
    15776	,
    16587	,
    17397	,
    18204	,
    19007	,
    19804	,
    20592	,
    21370	,
    22136	,
    22887	,
    23623	,
    24341	,
    25039	,
    25717	,
    26371	,
    27001	,
    27606	,
    28182	,
    28730	,
    29247	,
    29734	,
    30187	,
    30607	,
    30991	,
    31341	,
    31653	,
    31928	,
    32165	,
    32364	,
    32523	,
    32643	,
    32723	,
    32763	,
    32763	,
    32723	,
    32643	,
    32523	,
    32364	,
    32165	,
    31928	,
    31653	,
    31341	,
    30991	,
    30607	,
    30187	,
    29734	,
    29247	,
    28730	,
    28182	,
    27606	,
    27001	,
    26371	,
    25717	,
    25039	,
    24341	,
    23623	,
    22887	,
    22136	,
    21370	,
    20592	,
    19804	,
    19007	,
    18204	,
    17397	,
    16587	,
    15776	,
    14967	,
    14162	,
    13362	,
    12569	,
    11786	,
    11014	,
    10255	,
    9511	,
    8784	,
    8075	,
    7387	,
    6721	,
    6078	,
    5461	,
    4871	,
    4308	,
    3775	,
    3273	,
    2804	,
    2367	,
    1965	,
    1597	,
    1267	,
    973	,
    717	,
    499	,
    320	,
    180	,
    80	,
    20	,
    0
};
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
