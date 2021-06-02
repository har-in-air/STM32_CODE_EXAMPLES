#include "btn.h"

// BtnL = PB4
// BtnR = PB5

void btn_init() {
	// Enable the GPIOB peripheral in 'RCC_AHBENR'.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// no need to initialize the registers as they default to
	// input with no pullup or pull down, but do it anyway

	// input 00
	// output 01
	// alt func 10
	// analog 11
	MODIFY_REG(GPIOB->MODER,
			GPIO_MODER_MODER4 | GPIO_MODER_MODER5,
			_VAL2FLD(GPIO_MODER_MODER4, 0) | _VAL2FLD(GPIO_MODER_MODER5, 0)
		);

	// no pu or pd 00
	// pu 01
	// pd 02
	MODIFY_REG(GPIOB->PUPDR,
			GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5,
			_VAL2FLD(GPIO_PUPDR_PUPD4, 0) | _VAL2FLD(GPIO_PUPDR_PUPD5, 0)
		);
	}


// TTP223  modules generate logic 1 when pressed
uint8_t btn_read(uint32_t gpio_pin) {
	 return READ_BIT(GPIOB->IDR, gpio_pin)  ? 1 : 0;
	}
