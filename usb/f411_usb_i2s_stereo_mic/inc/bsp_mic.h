#ifndef BSP_MIC_H_
#define BSP_MIC_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_audio_if.h"
#include "usbd_audio_in.h"

void bsp_mic_init();
HAL_StatusTypeDef bsp_mic_start();
HAL_StatusTypeDef bsp_mic_stop();
HAL_StatusTypeDef bsp_mic_pause();
HAL_StatusTypeDef bsp_mic_resume();
void bsp_mic_led_set_state(int state);
void bsp_mic_set_led();
void bsp_mic_set_volume(int16_t volume);
void bsp_mic_i2s_half_complete();
void bsp_mic_i2s_complete();


#endif /* BSP_MIC_H_ */
