#include <stdint.h>
#include "cmsis/device/stm32f4xx.h"
#include "cmsis/device/system_stm32f4xx.h"
#include "util/logger.h"

static void configure_core_clock();

// from cubemx clock tree diagram
// HSE = 8MHz on STM32F407VGT6 DevEBox dev board
// PLL  : M = 8, N = 336, P = 4, Q = 7
// AHB Prescaler  = 1
// HCLK = 84MHz  (core, memory, DMA )
// USB Clk = 48MHz

// APB1 Prescaler = 2
// APB2 Prescaler = 1
// PCLK1 = 42MHz
// PCLK2 = 84MHz

static void configure_core_clock() {
	// configure flash latency
	// 2 wait states as per ref manual 3.5.1 table 10 for HCLK = 84MHz, VCC = 3v3
	MODIFY_REG(FLASH->ACR,
			FLASH_ACR_LATENCY, // field to clear
			_VAL2FLD(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS) // value to write in cleared field
			);

	// enable HSE
	SET_BIT(RCC->CR, RCC_CR_HSEON);
	// wait until HSE is stable
	while (!READ_BIT(RCC->CR, RCC_CR_HSERDY));

	// configure PLL while it is disabled
	MODIFY_REG(RCC->PLLCFGR,
			RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC, // mask to clear
			_VAL2FLD(RCC_PLLCFGR_PLLM, 8) |  // M = 8
			_VAL2FLD(RCC_PLLCFGR_PLLN, 336) | // N = 336
			_VAL2FLD(RCC_PLLCFGR_PLLP, 1) | // 1 for P=4
			_VAL2FLD(RCC_PLLCFGR_PLLQ, 7) | // Q = 7
			_VAL2FLD(RCC_PLLCFGR_PLLSRC, 1) // source = HSE
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
			_VAL2FLD(RCC_CFGR_HPRE, 0) | // HCLK = SYSCLK/1
			_VAL2FLD(RCC_CFGR_PPRE1, 4) | // APB1 PCLK1 = HCLK/2
			_VAL2FLD(RCC_CFGR_PPRE2, 0) // APB2 PCLK2 = HCLK/1
			);

	// wait until pll is used as system clock
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// disable HSI to save power
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);
	}


// SystemInit() is declared in system_stm32f4xx.h and called by startup_stm32f407vgtx.s
void SystemInit(void){
	configure_core_clock();
	}
