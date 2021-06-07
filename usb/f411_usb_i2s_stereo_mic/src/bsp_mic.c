#include "main.h"
#include "bsp_mic.h"
#include "vol_ctrl.h"

// We process 10mS worth of samples at a time as we want to use the SVC/GREQ STM32 libraries
// Sampling rate 48000Hz

// Each INMP441 I2S microphone sample is 64bits (L channel 24bits in a 32bit frame, R channel 24bits in a 32bit frame)
// i.e.  each microphone sample requires 4 * uint16_t.
// The microphone samples are received into I2SRcvBuffer. This is a circular buffer with 20mS of samples.
// When DMA half complete callback is received  the first 10mS half of I2SRcvBuffer is processed
// to extract 16bit L and 16bit R channel data, process it (volume/equalization etc.) and then
// transmit it to the USB controller.
// Similarly, when the DMA complete callback is received, the second 10mS half of I2SRcvBuffer is processed
// to extract the 16bit stereo data and transmit it to the USB controller.
// This repeats in a circular loop.

// Input I2S 24/32 stereo format => for (10mS+10mS) circular buffer
// we need [(48000/1000) * 10 * 2 * 2] * 2 = 3840 uint16_t's

#define I2S_RCV_BUFFER_10MS_SIZE_HWORDS   (48 * 10 * 2 * 2)

static volatile uint16_t I2SRcvBuffer[I2S_RCV_BUFFER_10MS_SIZE_HWORDS * 2];

// ProcessBuffer contains 10mS of extracted L and R 16bit samples from the 1/2 I2SRcvBuffer that is processed
// every DMA half-complete or complete callback
// Output 16bit stereo => for 10ms buffer we need (48000/1000) * 10 * 2 = 960 uint16_t's

#define PROCESS_BUFFER_10MS_SIZE_HWORDS   (48 * 10 * 2)

static int16_t ProcessBuffer[PROCESS_BUFFER_10MS_SIZE_HWORDS];

// USBTxBuffer contains the 10mS of L and R 16bit samples being transmitted to the USB controller. This also
// needs to be double buffered, as first half of the buffer is transmitted at DMA half complete callback,
// second half at DMA complete callback
static int16_t USBTxBuffer[PROCESS_BUFFER_10MS_SIZE_HWORDS * 2];

static int IsRunning = false;
static int IsMuted = false;

static void bsp_mic_send_data(volatile uint16_t *data_in, int16_t *data_out);


void bsp_mic_init(){
	vol_init();
	IsRunning = false;
	IsMuted = false;
	bsp_mic_set_led();
	}


// Start the I2S DMA transfer into the circular I2SRcvBuffer
// called from usbd_audio_if.c
inline HAL_StatusTypeDef bsp_mic_start() {
	HAL_StatusTypeDef status;
	// For I2S 24/32 data format, the HAL api requires us to specify the transfer size in words.
	// Size of the entire circular buffer in words = (I2S_RCV_BUFFER_10MS_SIZE_HWORDS*2)/2
	if ((status = HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*)I2SRcvBuffer, I2S_RCV_BUFFER_10MS_SIZE_HWORDS)) == HAL_OK) {
		IsRunning = true;
		}
	return status;
	}


// Stop the I2S DMA transfer
inline HAL_StatusTypeDef bsp_mic_stop() {
	HAL_StatusTypeDef status;
	if ((status = HAL_I2S_DMAStop(&hi2s1)) == HAL_OK) {
		IsRunning = false;
		}
	return status;
	}


// Pause I2S DMA transfer (soft-mute)
inline HAL_StatusTypeDef bsp_mic_pause() {
	HAL_StatusTypeDef status;
	if ((status = HAL_I2S_DMAPause(&hi2s1)) == HAL_OK) {
		IsRunning = false;
		}
	return status;
	}


// Resume I2S DMA transfer
inline HAL_StatusTypeDef bsp_mic_resume() {
	HAL_StatusTypeDef status;
	if ((status = HAL_I2S_DMAResume(&hi2s1)) == HAL_OK) {
		IsRunning = true;
		}
	return status;
	}



// black pill onboard LED for mic running/muted status
inline void bsp_mic_led_set_state(int state){
	// led needs active low to turn on
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}


// Light the live LED if we are unmuted (hard and soft)
inline void bsp_mic_set_led()  {
	bsp_mic_led_set_state(IsRunning && (!IsMuted));
	}

// Set the volume gain
inline void bsp_mic_set_volume(int16_t volume) {
	// reduce resolution from 1/256dB to 1/2dB
	volume /= 128;
	// ensure SVC library limits are respected
	if (volume < -160) {
		volume = -160;
		}
	else
	if (volume > 72) {
		volume = 72;
		}
//	vol_set_volume(volume);
	}


// 1. Transform the current half-I2SRcvBuffer I2S data into 16 bit PCM samples in a holding buffer
// 2. Transmit over USB to the host
// We've got 10ms to complete this method before the next DMA transfer will be ready.
inline void bsp_mic_send_data(volatile uint16_t *data_in, int16_t *data_out) {
	// send if we're not muted and we're connected
	if ((!IsMuted) && IsRunning) {
		// incoming 64 bit stereo I2S microphone sample is packaged as 24bits left-justified in 32bit frame. L channel then R channel
		// data_in[n]   = L 16MSb
		// data_in[n+1] = L 8LSb + pad byte
		// data_in[n+2] = R 16MSb
		// data_in[n+3] = R 8LSb + pad byte
		// extract the L and R channel 16 MSbits
		for (int inx = 0; inx < PROCESS_BUFFER_10MS_SIZE_HWORDS; inx++) {
			ProcessBuffer[inx] = 4*(int16_t)data_in[inx*2];     // 16MSb from each channel, *4 => +12dB
			}

		// specify the number of stereo samples for processing
		//vol_process_buffer(ProcessBuffer, PROCESS_BUFFER_10MS_SIZE_HWORDS/2);

#if 1
		// process the L and R 16bit samples
		for (int inx = 0; inx < PROCESS_BUFFER_10MS_SIZE_HWORDS/2; inx += 2) {
			int16_t near = ProcessBuffer[inx]; // l channel is mic pointing towards speaker
			int16_t far = ProcessBuffer[inx+1]; // r channel is mic pointing away from speaker
			ProcessBuffer[inx] = 2*near - far;
			ProcessBuffer[inx+1] = 2*far - near;
			}

#endif

		// copy the L & R data from the processed buffer to USBTxBuffer
		for (int inx = 0; inx < PROCESS_BUFFER_10MS_SIZE_HWORDS; inx++) {
			data_out[inx] = ProcessBuffer[inx];
			}

		// transmit the buffer  to the usb controller
		// here we need to specify the number of stereo samples
		if (USBD_AUDIO_Data_Transfer(&hUsbDeviceFS, data_out, PROCESS_BUFFER_10MS_SIZE_HWORDS/2) != USBD_OK) {
			Error_Handler();
			}
		}
	}



// I2S DMA half-complete HAL callback to process the first 10mS of the data while the
// DMA device continues to run onward to fill the second half of the buffer.
inline void bsp_mic_i2s_half_complete() {
	bsp_mic_send_data(I2SRcvBuffer, USBTxBuffer);
	}


// I2S DMA complete HAL callback to process the second 10mS
// of the data while the DMA  fills the first half of the buffer
inline void bsp_mic_i2s_complete() {
	bsp_mic_send_data(&I2SRcvBuffer[I2S_RCV_BUFFER_10MS_SIZE_HWORDS], &USBTxBuffer[PROCESS_BUFFER_10MS_SIZE_HWORDS]);
	}
