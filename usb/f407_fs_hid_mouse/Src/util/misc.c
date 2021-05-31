#include "util/logger.h"
#include "util/misc.h"

uint32_t SystemCoreClock;// declared in system_stm32f4xx.h

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

uint32_t get_pclk1_freq(void){
	return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos]);
	}

void config_swdio_pins() {
	// Enables the clock for GPIOA
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

	// Configures PA13 and PA14 as SWDIO pins (AF0)
	// Note: PB3 is used for SWO but it's dynamically configured by the debug application via the SWD adapter
	MODIFY_REG(GPIOA->AFR[1],
		GPIO_AFRH_AFSEL13 | GPIO_AFRH_AFSEL14,
		_VAL2FLD(GPIO_AFRH_AFSEL13, 0) | _VAL2FLD(GPIO_AFRH_AFSEL14, 0)
		);
	}


void system_core_clock_update(){
	uint32_t pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
	// Get SYSCLK source
	uint32_t source = RCC->CFGR & RCC_CFGR_SWS;

	switch (source)  {
		case 0x00:  // HSI
		SystemCoreClock = HSI_VALUE;
		break;

		case 0x04:  // HSE
		SystemCoreClock = HSE_VALUE;
		break;

		case 0x08:  // PLL
		// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
		// SYSCLK = PLL_VCO / PLL_P
		pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
		pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
		if (pllsource != 0)      {
			// HSE
			pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			}
		else  {
			// HSI
			pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			}
		pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
		SystemCoreClock = pllvco/pllp;
		break;

		default:
		SystemCoreClock = HSI_VALUE;
		break;
		}
	uint32_t hclk_prescaler  = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	SystemCoreClock >>= hclk_prescaler;
	}
