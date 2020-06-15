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


void EXTI0_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	}
