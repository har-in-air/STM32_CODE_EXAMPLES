#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);
extern I2S_HandleTypeDef hi2s1;
extern DMA_HandleTypeDef hdma_spi1_rx;

#ifndef false
#define false 0
#endif
#ifndef true
#define true 1
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
