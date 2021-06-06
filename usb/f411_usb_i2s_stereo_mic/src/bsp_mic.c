#include "main.h"
#include "bsp_mic.h"

// Each INMP441 I2S microphone sample is 64bits (32 L frame, 32 R frame) i.e. 4 half-words.

// SampleBuffer is a circular buffer. When DMA half complete callback is received
// the first half of SampleBuffer is processed to extract 16bit L and R channel data and
// send it to the USB controller.
// When the DMA complete callback is received, the second half
// of SampleBuffer is processed and the 16bit data is sent to the USB controller.
// This repeats in a circular loop.
// The full buffer contains 20mS of samples = 7680 bytes
static volatile uint16_t SampleBuffer[MIC_SAMPLES_PER_PACKET * 2 * 2];

// ProcessBuffer contains the extracted L and R 16bit samples from the 1/2 SampleBuffer that is processed
// at DMA callback
// 1920 bytes
static int16_t ProcessBuffer[MIC_SAMPLES_PER_PACKET];

// SendBuffer contains the L and R 16bit samples being transmitted to the USB controller. First half
// of the buffer is transmitted at DMA half complete callback, second half at DMA complete
// callback
// 3840 bytes
static int16_t SendBuffer[MIC_SAMPLES_PER_PACKET*2];

static int IsRunning = false;
static int IsMuted = false;

static void bsp_mic_send_data(volatile uint16_t *data_in, int16_t *data_out);


void bsp_mic_init(){
	IsRunning = false;
	IsMuted = false;
	bsp_mic_set_led();
	}


// Start the I2S DMA transfer (called from usbd_audio_if.cpp)
inline HAL_StatusTypeDef bsp_mic_start() {
	HAL_StatusTypeDef status;
	// For I2S 24/32 data format, we need to specify the transfer size in words.
	// HAL will internally multiply by 2 to transfer size*2 half-words
	// This is the size of the entire circular buffer
	if ((status = HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*)SampleBuffer, MIC_SAMPLES_PER_PACKET * 2)) == HAL_OK) {
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
	}


// 1. Transform the current half-SampleBuffer I2S data into 16 bit PCM samples in a holding buffer
// 2. Transmit over USB to the host
// We've got 10ms to complete this method before the next DMA transfer will be ready.
inline void bsp_mic_send_data(volatile uint16_t *data_in, int16_t *data_out) {
	// send if we're not muted and we're connected
	if ((!IsMuted) && IsRunning) {
		// incoming 64 bit I2S microphone sample is packaged as 24bits left-justified in 32bit frame, L channel then R channel
		// data_in[n]   = L 16MSb
		// data_in[n+1] = L 8LSb + pad byte
		// data_in[n+2] = R 16MSb
		// data_in[n+3] = R 8LSb + pad byte
		// extract the L and R channel 16 MSbits
		int16_t *dest = ProcessBuffer;
		int cnt = MIC_SAMPLES_PER_PACKET / 2;
		while (cnt--) {
			*dest++ = 8*(int16_t)data_in[0];     // 16MSb from 24bit L channel, +12dB
			*dest++ = 8*(int16_t)data_in[2];     // 16MSb from 24bit R channel, +12dB
			data_in += 4;
			}

#if 1
		// process the L and R 16bit samples
		{
		int16_t *src = ProcessBuffer;
		cnt = MIC_SAMPLES_PER_PACKET/2;
		while (cnt--) {
			int16_t near = *src; // l channel is mic pointing towards speaker
			int16_t far = *(src+1); // r channel is mic pointing away from speaker
			*src = 2*near - far;
			*(src+1) = 2*far - near;
			src += 2;
			}
		}
#endif

		// copy the L & R data from the processed buffer to SendBuffer
		int16_t *src = ProcessBuffer;
		dest = data_out;
		cnt = MIC_SAMPLES_PER_PACKET;
		while (cnt--) {
			*dest++ = *src++;
			}

		// send the 10mS of processed L and R samples  to the usb controller
		if (USBD_AUDIO_Data_Transfer(&hUsbDeviceFS, data_out, MIC_SAMPLES_PER_PACKET) != USBD_OK) {
			Error_Handler();
			}
		}
	}



// I2S DMA half-complete HAL callback to process the first MIC_MS_PER_PACKET/2  milliseconds (10ms)
// of the data while the DMA device continues to run onward to fill the second half of the buffer.
inline void bsp_mic_i2s_half_complete() {
	bsp_mic_send_data(SampleBuffer, SendBuffer);
	}


// I2S DMA complete HAL callback to process the second MIC_MS_PER_PACKET/2 milliseconds
// of the data while the DMA in circular mode wraps back to the start of the buffer
inline void bsp_mic_i2s_complete() {
	bsp_mic_send_data(&SampleBuffer[MIC_SAMPLES_PER_PACKET*2], &SendBuffer[MIC_SAMPLES_PER_PACKET]);
	}
