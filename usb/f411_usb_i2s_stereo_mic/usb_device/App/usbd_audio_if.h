#ifndef USBD_AUDIO_IF_H_
#define USBD_AUDIO_IF_H_

#include "usbd_audio_in.h"

// Note : On the STM32F411, the internal I2S PLL clock in
// 24/32 master receive mode @ Fs = 48kHz is actually 47.048kHZ
// (off by nearly 2%)
#define MIC_SAMPLE_FREQUENCY_HZ 	48000
#define MIC_NUM_CHANNELS 			2


extern USBD_AUDIO_ItfTypeDef 	USBD_AUDIO_fops;

#endif
