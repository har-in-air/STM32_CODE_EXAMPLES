#include <stdint.h>
#include "cmsis/device/system_stm32f4xx.h"
#include "cmsis/device/stm32f4xx.h"
#include "util/logger.h"

static void configure_clock();

// STM32F411CEU6 Black Pill development board
// from cubemx clock tree diagram
// HSE = 25MHz
// HCLK = 96MHz  (core, memory, DMA )
// PCLK1 = 48MHz
// PCLK2 = 96MHz
// USB Clk = 48MHz
// PLL  : M = 25, N = 384, P = 4, Q = 8
// AHB Prescaler  = 1
// APB1 Prescaler = 2
// APB2 Prescaler = 1

void configure_clock() {
	// flash latency 3 wait states as per ref manual 3.4.1 table 5 for HCLK = 96MHz, VCC = 3v3
	MODIFY_REG(FLASH->ACR,
			FLASH_ACR_LATENCY, // field mask to clear
			_VAL2FLD(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS) // field value to set
			);
	// enable HSE
	SET_BIT(RCC->CR, RCC_CR_HSEON);
	// wait until HSE is stable
	while (!READ_BIT(RCC->CR, RCC_CR_HSERDY));
	// configure PLL while it is disabled
	// source HSE, HCLK = 96MHz, note PLLP field = 1 for divide by 4
	MODIFY_REG(RCC->PLLCFGR,
			RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC, 
			_VAL2FLD(RCC_PLLCFGR_PLLM, 25) | // M = 25
			_VAL2FLD(RCC_PLLCFGR_PLLN, 384) | // N = 384
			_VAL2FLD(RCC_PLLCFGR_PLLP, 1) | // 1 => P = 4
			_VAL2FLD(RCC_PLLCFGR_PLLQ, 8) |  // Q = 8
			_VAL2FLD(RCC_PLLCFGR_PLLSRC, 1) // pll source = HSE
			);
	// enable PLL module
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	// wait until PLL output is stable
	while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY));
	// switch system clock to PLL output
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_SW,
			_VAL2FLD(RCC_CFGR_SW, RCC_CFGR_SW_PLL)
			);
	//configure the AHB, APB Prescalers
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2,
			_VAL2FLD(RCC_CFGR_HPRE, 0) |  // HCLK = SYSCLK/1
			_VAL2FLD(RCC_CFGR_PPRE1, 4) | // APB1 PCLK1 = HCLK/2
			_VAL2FLD(RCC_CFGR_PPRE2, 0) // APB2 PCLK2 = HCLK/1
			);
	// wait until pll is used as system clock
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	// disable HSI to save power
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);
	}


// SystemInit called by startup_stm32f411ceux.s,
// declared in system_stm32f4xx.h
void SystemInit(void) {
	configure_clock();
	}
