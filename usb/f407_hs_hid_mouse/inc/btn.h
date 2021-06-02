#ifndef BTN_H_
#define BTN_H_

#include <stdint.h>

#include "../inc/cmsis/device/stm32f4xx.h"

// mouse buttons use PD4 and PD5
#define GPIO_BTN_L		(1U << 4)
#define GPIO_BTN_R		(1U << 5)

void btn_init();
uint8_t btn_read(uint32_t gpio_pin);

#endif /* BTN_H_ */
