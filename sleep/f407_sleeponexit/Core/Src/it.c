/*
 * it.c
 *
 *  Created on: May 30, 2020
 *      Author: hari
 */
#include "main.h"
#include "it.h"

void SysTick_Handler(void){
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;


void TIM6_DAC_IRQHandler(void){
	// pin toggling to see how much time is spent in the HAL irq handler
	// includes time spent in the uart transmit msg in the delayElapsed callback
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

	HAL_TIM_IRQHandler(&htim6);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	}
