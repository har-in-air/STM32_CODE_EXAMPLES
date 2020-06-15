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

extern TIM_HandleTypeDef htim2;


void TIM2_IRQHandler(void){
	HAL_TIM_IRQHandler(&htim2);
    }
