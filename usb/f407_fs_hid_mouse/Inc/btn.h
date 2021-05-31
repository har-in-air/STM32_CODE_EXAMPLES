#ifndef BTN_H_
#define BTN_H_

#include <stdint.h>
#include "cmsis/device/stm32f4xx.h"


#define BTN_L		4
#define BTN_R		5

void btn_init();
uint8_t btn_read(uint32_t gpio_pin);

#endif /* BTN_H_ */
