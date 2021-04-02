/* USER CODE BEGIN Header */
// Weact v 1.3 STM32F411CEU6 dev board
// Demos task notification from isr, queues, software timer

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

TaskHandle_t xTaskMenuDisplay = NULL;
TaskHandle_t xTaskUartWrite = NULL;
TaskHandle_t xTaskCmdHandler = NULL;
TaskHandle_t xTaskCmdProcessor = NULL;

QueueHandle_t queueCommand = NULL;
QueueHandle_t queueUartWrite = NULL;

TimerHandle_t ledTimerHandle = NULL;

typedef struct APP_CMD_{
	uint8_t COMMAND_NUM;
	uint8_t COMMAND_ARGS[10];
	} APP_CMD;

uint8_t commandBuffer[20];
uint8_t commandLen =0;

char menu[]={"\
\r\nLED_ON             ----> 1 \
\r\nLED_OFF            ----> 2 \
\r\nLED_TOGGLE         ----> 3 \
\r\nLED_TOGGLE_OFF     ----> 4 \
\r\nLED_READ_STATUS    ----> 5 \
\r\nEXIT_APP           ----> 0 \
\r\nType your option here : "};


#define LED_ON_COMMAND 			1
#define LED_OFF_COMMAND 		2
#define LED_TOGGLE_COMMAND 		3
#define LED_TOGGLE_STOP_COMMAND 4
#define LED_READ_STATUS_COMMAND 5

void task_uart_write(void* pParams);
void task_command_handler(void* pParams);
void task_command_processor(void* pParams);
void task_menu_display(void* pParams);

void print_msg(char *format,...);
void print_uart_sz(char *msg);
void busy_delay(uint32_t delay_in_ms);
uint8_t get_command_code(uint8_t *buffer);
void led_on(void);
void led_off(void);
void ledToggle(TimerHandle_t xTimer);
void led_toggle_start(uint32_t duration);
void led_toggle_stop(void);


void busy_delay(uint32_t delay_in_ms){
	uint32_t current_tick_count = xTaskGetTickCount();
	uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ ) /1000 ;
	while(xTaskGetTickCount() <  (current_tick_count + delay_in_ticks));
	}

void print_uart_sz(char *msg){
	for(int inx = 0; inx < strlen(msg); inx++)	{
		while (!(huart2.Instance->SR & UART_FLAG_TXE));
		huart2.Instance->DR = (uint16_t) msg[inx];
		}
	while (!(huart2.Instance->SR & UART_FLAG_TC));
	}

void print_msg(char *format,...){
	char str[120];
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	print_uart_sz(str);
	va_end(args);
 	}

void EXTI0_IRQHandler(void){
	traceISR_ENTER();
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	traceISR_EXIT();
	}

void USART2_IRQHandler(void) {
	// do NOT call the HAL irq handler !
	traceISR_ENTER();
	uint16_t dataByte;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t isrflags   = READ_REG(huart2.Instance->SR);
	if(isrflags & USART_SR_RXNE)	{
		// RX not empty interrupt received
		// reading the DR register clears the RXNE interrupt
		dataByte = huart2.Instance->DR & (uint16_t)0xFF;
		commandBuffer[commandLen++] = (uint8_t)dataByte;
		if(dataByte == '\r')		{
			// user is finished entering the data
			// reset the commandLen variable
			commandLen = 0;
			//notify the command handling task
			xTaskNotifyFromISR(xTaskCmdHandler, 0, eNoAction, &xHigherPriorityTaskWoken);
			// notify the menu display task. This has lower priority, so TaskCmdHandler and
			// TaskCmdProcessor output will be printed first, then the menuDisplay output
			xTaskNotifyFromISR(xTaskMenuDisplay, 0, eNoAction, &xHigherPriorityTaskWoken);
			}
		}

	// if the above freertos apis wake up any higher priority task, then yield the processor to the
	// higher priority task which is just woken up.
	if(xHigherPriorityTaskWoken)	{
		taskYIELD();
		}
	traceISR_EXIT();
	}


uint8_t get_command_code(uint8_t *buffer){
	return buffer[0]-48;
	}

void led_on(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}

void led_off(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}

void ledToggle(TimerHandle_t xTimer){
	traceTIMER_ENTER(xTimer);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	traceTIMER_EXIT();
	}

void led_toggle_start(uint32_t duration){
	if(ledTimerHandle == NULL)	{
		ledTimerHandle = xTimerCreate("LED_TIMER", duration, pdTRUE, NULL, ledToggle);
		}
	xTimerStart(ledTimerHandle,portMAX_DELAY);
	}


void led_toggle_stop(void){
	 xTimerStop(ledTimerHandle,portMAX_DELAY);
	}


void read_led_status(char *taskMsg){
	sprintf(taskMsg , "\r\nLED status is : %s\r\n", HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) ? "OFF" : "ON");
	xQueueSend(queueUartWrite, &taskMsg, portMAX_DELAY);
	}

void print_error_message(char *taskMsg){
	sprintf( taskMsg,"\r\nInvalid command received\r\n");
	xQueueSend(queueUartWrite, &taskMsg,portMAX_DELAY);
	}


void task_menu_display(void *params){
	char *pData = menu;
	while(1)	{
		xQueueSend(queueUartWrite, &pData, portMAX_DELAY);
		//wait here until someone notifies.
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		}
	}

void task_command_handler(void *params){
	uint8_t commandCode=0;
	APP_CMD *newCmd;
	while(1)	{
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
		newCmd = (APP_CMD*) pvPortMalloc(sizeof(APP_CMD));

		taskENTER_CRITICAL();
		commandCode = get_command_code(commandBuffer);
		taskEXIT_CRITICAL();

		newCmd->COMMAND_NUM = commandCode;
		xQueueSend(queueCommand, &newCmd, portMAX_DELAY);
		}
	}


void task_command_processor(void *params){
	APP_CMD *newCmd;
	char taskMsg[50];
	uint32_t toggleDuration = pdMS_TO_TICKS(50);

	while(1)	{
		xQueueReceive(queueCommand, (void*)&newCmd, portMAX_DELAY);
		switch (newCmd->COMMAND_NUM) {
		case LED_ON_COMMAND:
			led_on();
			break;

		case LED_OFF_COMMAND:
			led_off();
		    break;

		case LED_TOGGLE_COMMAND:
			led_toggle_start(toggleDuration);
			break;

		case LED_TOGGLE_STOP_COMMAND:
			led_toggle_stop();
		    break;

		case LED_READ_STATUS_COMMAND:
			read_led_status(taskMsg);
		    break;

		default :
			print_error_message(taskMsg);
			break;
			}

		// free the allocated memory for the new command
		vPortFree(newCmd);
		}
	}




void task_uart_write(void *params){
	char *pData = NULL;
	while(1)	{
		xQueueReceive(queueUartWrite, &pData, portMAX_DELAY);
		print_uart_sz(pData);
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
  HAL_RCC_DeInit(); // 16 MHz HSI
  SystemCoreClockUpdate();


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  print_msg("SystemCoreClock %lu\r\n", SystemCoreClock);
  print_msg("Queue Demo\r\n");
  DWT->CTRL |= (1 << 0); // enable CYCCNT for SystemView

  SEGGER_SYSVIEW_Conf();
  vSetVarulMaxPRIGROUPValue();
  SEGGER_SYSVIEW_Start();

	queueCommand = xQueueCreate( 10, sizeof(APP_CMD*));
	queueUartWrite = xQueueCreate(10, sizeof(char*));

	if((queueCommand != NULL) && (queueUartWrite != NULL))	{
		xTaskCreate(task_uart_write, "tskUartWrite", 500, NULL, 2, &xTaskUartWrite);
		xTaskCreate(task_menu_display, "tskMenuDisplay", 500, NULL, 1, &xTaskMenuDisplay);
		xTaskCreate(task_command_handler, "tskCmdHandler", 500, NULL, 2, &xTaskCmdHandler);
		xTaskCreate(task_command_processor, "tskCmdProcessor", 500, NULL, 2, &xTaskCmdProcessor);
	    vTaskStartScheduler();
		}
	else{
		print_msg("Queue creation failed\r\n");
		}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  print_msg("main loop %d\r\n", counter++);
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  huart2.Instance->CR1 |= USART_CR1_RXNEIE;
  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
