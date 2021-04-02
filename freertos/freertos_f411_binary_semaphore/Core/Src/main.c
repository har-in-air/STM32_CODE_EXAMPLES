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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

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

void print_uart_sz(char *msg);
void print_msg(char *format,...);
void srand1( uint32_t seed );
uint32_t rand1( void );
static void vManagerTask( void *pvParameters );
static void vEmployeeTask( void *pvParameters );
void EmployeeDoWork(uint32_t TicketId);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TSK_MGR_PRIORITY 3
#define TSK_EMP_PRIORITY 1


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


// Declare a variable of type xSemaphoreHandle.  This is used to reference the
// semaphore that is used to synchronize both manager and employee task
xSemaphoreHandle xWork;

// this is the queue which manager uses to put the work ticket id
xQueueHandle xWorkQueue;


static uint32_t miState;

uint32_t rand1( void ){
    miState ^= (miState << 13);
    miState ^= (miState >> 17);
    miState ^= (miState << 15);

    return (miState * 1332534557) & 0x7FFFFFFF;
	}

void srand1( uint32_t seed ){
    // a zero seed will not work!
    if (seed == 0)
        seed = 0x55aaff00;
    miState = seed;
	}

void vManagerTask( void *pvParameters ){
	 uint32_t  xWorkTicketId;
	 portBASE_TYPE xStatus;
	 // seed random number generator for work ticket ID
	 srand1(769);
	 // The semaphore is created in the 'empty' state, meaning the semaphore must
	 // first be given using the xSemaphoreGive() API function before it
	 //	can subsequently be taken (obtained)
	 xSemaphoreGive( xWork);

	 while(1)  {
		 // issue a work ticket id (random number)
		 xWorkTicketId = ( rand1() % 0x7FF);

		 // Sends work ticket id to the work queue
		 xStatus = xQueueSend( xWorkQueue, &xWorkTicketId , portMAX_DELAY ); //Post an item on back of the queue

		 if( xStatus != pdPASS ){
			 print_msg("Could not send to the queue.\r\n");
		 	 }
		 else{
			//  Manager notifying the employee by "Giving" semaphore
			xSemaphoreGive( xWork);
			// after assigning the work , just yield the processor because nothing to do
			taskYIELD();
		 	}
	 	 }
	}

static void vEmployeeTask( void *pvParameters ){
	uint32_t xWorkTicketId;
	portBASE_TYPE xStatus;
	while(1)  {
		// First Employee tries to take the semaphore, if
		// it is available that means there is a task assigned by manager,
		// otherwise employee task will be blocked
		xSemaphoreTake( xWork, 0 );

		// get the ticket id from the work queue
		xStatus = xQueueReceive( xWorkQueue, &xWorkTicketId, 0 );

		if( xStatus == pdPASS )	{
			// employee may decode the xWorkTicketId in this function to do the work
			EmployeeDoWork(xWorkTicketId);
			}
		else{
			// We did not receive anything from the queue.  This must be an error as
			// this task should only run when the manager assigns at least one piece of work.
			print_msg("Employee task : Queue is empty , nothing to do.\r\n");
			}
	}
}

void EmployeeDoWork(uint32_t TicketId){
	// implement the work according to Ticket ID
	print_msg("Employee task : Working on Ticket id : %d\r\n", TicketId);
	// use the random ticket ID value as delay
	vTaskDelay(TicketId);
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	TaskHandle_t task_mgr_handle;
	TaskHandle_t task_emp_handle;
	BaseType_t status;


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  DWT->CTRL |= (1 << 0); // enable CYCCNT for SystemView
  //Start Recording
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

  print_msg("SystemCoreClock %lu\r\n", SystemCoreClock);


   print_msg("Demo of Binary semaphore usage between 2 Tasks \r\n");

   // Before a semaphore is used it must be explicitly created
   // In this example a binary semaphore is created
   vSemaphoreCreateBinary( xWork );

   // The queue is created to hold a maximum of 1 element of size uint32_t
   xWorkQueue = xQueueCreate( 1, sizeof(uint32_t) );

   // Check the semaphore and queue was created successfully
   if( (xWork != NULL) && (xWorkQueue != NULL) )  {
 	  // The manager task  will be synchronized with the Employee task.
 	  // Created with a high priority
       status = xTaskCreate( vManagerTask, "Manager", 500, NULL, TSK_MGR_PRIORITY, &task_mgr_handle );
       // configASSERT defined FreeRTOSConfig.h, traps in a loop if it fails
       configASSERT(status == pdPASS);

       // Create a employee task with less priority than manager
       status = xTaskCreate( vEmployeeTask, "Employee", 500, NULL, TSK_EMP_PRIORITY, &task_emp_handle );
       configASSERT(status == pdPASS);

       vTaskStartScheduler(); // never returns if successful
   	  }
   else {
	   print_msg("Queue/Semaphore create failed.. \r\n");
   	   }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // This code is used to test CubeMx generated code before adding FreeRTOS tasks.
  // It will not be executed if vTaskStartScheduler() runs successfully
  int counter = 0;
  while (1)  {
	  // blink on-board LED and print a message on USART2
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  print_msg("Counter = %d\r\n", counter++);
	  HAL_Delay(1000);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
  __disable_irq();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
