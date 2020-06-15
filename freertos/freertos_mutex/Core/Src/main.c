/* USER CODE BEGIN Header */
// WeAct v1.3 STM32F411CEU6 dev board
// mutual exclusion using binary semaphore to control task access to
// the uart (shared resource)
//
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;

void vTask1(void* pParams);
void vTask2(void* pParams);

xSemaphoreHandle xBinarySemaphore;


//#define USE_SWO

#ifdef USE_SWO

// printf retargeted to SWV ITM Data Console
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
}
#endif

void printUartSz(char *msg){
	for(int inx = 0; inx < strlen(msg); inx++)	{
		while (!(huart1.Instance->SR & UART_FLAG_TXE));
		huart1.Instance->DR = (uint16_t) msg[inx];
		}
	while (!(huart1.Instance->SR & UART_FLAG_TC));
	}

void printUart(char *format,...){
	char str[120];
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	printUartSz(str);
	va_end(args);
 	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  DWT->CTRL |= (1 << 0); // enable CYCCNT for SystemView

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_RCC_DeInit(); // 16 MHz HSI
  SystemCoreClockUpdate();
  HAL_Delay(500); // leave this here so that JLink/STlink can connect to cpu after reset


#ifdef USE_SWO
  printf("hello from SWO\r\n");
#endif

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_Delay(2000);
  printUart("SystemCoreClock %lu\r\n", SystemCoreClock);

  SEGGER_SYSVIEW_Conf();
  vSetVarulMaxPRIGROUPValue();
  SEGGER_SYSVIEW_Start();

  printUart("Demo of Mutual exclusion using binary semaphore\r\n");

  vSemaphoreCreateBinary(xBinarySemaphore);

  if (xBinarySemaphore != NULL)	 {
	// Create one of the two tasks
	xTaskCreate(vTask1,		// Pointer to the function that implements the task.
				"Task 1",	// Text name for the task.  This is to facilitate debugging only.
				500,		// Stack depth in words.
				NULL,		// We are not using the task parameter.
				1,			// This task will run at priority 1.
				NULL );		// We are not using the task handle.

	// Create the other task in exactly the same way. */
	xTaskCreate( vTask2, "Task 2", 500, NULL, 1, NULL );

	//makes sema available for the first time
	xSemaphoreGive(xBinarySemaphore);

	// Start the scheduler so our tasks start executing.
	vTaskStartScheduler();
	}
  else{
	printUart("binary semaphore creation failed\r\n");
	}

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // should never get here
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  printUart("hello udemy %d\r\n", counter++);
	  HAL_Delay(1000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  huart1.Init.BaudRate = 115200;

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void vTask1( void *pvParameters ){
	const char *pcTaskName = "Task 1 is running\r\n";
	while(1) {
		// before printing , take the semaphore
		// semaphore value goes to 0
		xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

		// display task name
		printUart("%s", pcTaskName);

		// give the semaphore here.
		// increases the bin sema value back to 1
		xSemaphoreGive(xBinarySemaphore);

		// block this task for 500ms
		vTaskDelay( pdMS_TO_TICKS(500) );
		}
	}


void vTask2( void *pvParameters ){
	const char *pcTaskName = "Task 2 is running\r\n";
	while(1) {
		// before printing , take the semaphore
		// semaphore value goes to 0
		xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

		// display task name
		printUart("%s", pcTaskName);

		// give the semaphore here.
		// increases the bin sema value back to 1
		xSemaphoreGive(xBinarySemaphore);

		// block this task for 500ms
		vTaskDelay( pdMS_TO_TICKS(500) );
		}
	}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	// toggle the board LED rapidly
	int counter = 0;
	while (1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
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
