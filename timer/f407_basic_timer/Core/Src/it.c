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

// irq handler name from startup file
extern TIM_HandleTypeDef htim6;

void TIM6_DAC_IRQHandler(void){
	// check source of interrupt, acknowledge and call the specific callback
	// in our case HAL_TIM_PeriodElapsedCallback must be implemented
	HAL_TIM_IRQHandler(&htim6);
	}
