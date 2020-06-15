#ifndef _W25QXXCONFIG_H
#define _W25QXXCONFIG_H

#include "main.h"



#define _W25QXX_SPI                   hspi1
#define _W25QXX_CS_GPIO               GPIOA
#define _W25QXX_CS_PIN                GPIO_PIN_15
#define _W25QXX_USE_FREERTOS          0
#define _W25QXX_DEBUG                 0

#define CHIP_DESELECT()	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_SET)
#define CHIP_SELECT()	HAL_GPIO_WritePin(_W25QXX_CS_GPIO,_W25QXX_CS_PIN,GPIO_PIN_RESET)

#endif
