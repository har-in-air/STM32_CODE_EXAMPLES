#ifndef USB_DEVICE_H_
#define USB_DEVICE_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init();

#endif
