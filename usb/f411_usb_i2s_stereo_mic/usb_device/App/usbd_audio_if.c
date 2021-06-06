#include "usbd_audio_if.h"
#include "bsp_mic.h"

static int8_t Audio_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
static int8_t Audio_DeInit(uint32_t options);
static int8_t Audio_Record();
static int8_t Audio_VolumeCtl(int16_t Volume);
static int8_t Audio_MuteCtl(uint8_t cmd);
static int8_t Audio_Stop();
static int8_t Audio_Pause();
static int8_t Audio_Resume();
static int8_t Audio_CommandMgr(uint8_t cmd);

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops = {
	Audio_Init,
	Audio_DeInit,
	Audio_Record,
	Audio_VolumeCtl,
	Audio_MuteCtl,
	Audio_Stop,
	Audio_Pause,
	Audio_Resume,
	Audio_CommandMgr,
	};

/**
 * @brief  Initializes the AUDIO media low layer over USB FS IP
 * @param  AudioFreq: Audio frequency used to play the audio stream.
 * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
 * @param  options: Reserved for future use
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */

static int8_t Audio_Init(uint32_t AudioFreq, uint32_t Volume, uint32_t options) {
	bsp_mic_init();
	return USBD_OK;
	}

/**
 * @brief  De-Initializes the AUDIO media low layer
 * @param  options: Reserved for future use
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */

static int8_t Audio_DeInit(uint32_t options) {
	return USBD_OK;
	}



/**
 * @brief  Start audio recording engine
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */

static int8_t Audio_Record() {
	return bsp_mic_start();
	}

/**
 * @brief  Controls AUDIO Volume.
 * @param  vol: volume level
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */

static int8_t Audio_VolumeCtl(int16_t vol) {
	bsp_mic_set_volume(vol);
	return USBD_OK;
	}

/**
 * @brief  Controls AUDIO Mute.
 * @param  cmd: command opcode
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */

static int8_t Audio_MuteCtl(uint8_t cmd) {
	return USBD_OK;
	}

/**
 * @brief  Stops audio acquisition
 * @param  none
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */

static int8_t Audio_Stop() {
	return bsp_mic_stop();
	}

/**
 * @brief  Pauses audio acquisition
 * @param  none
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */

static int8_t Audio_Pause() {
	return bsp_mic_pause();
	}

/**
 * @brief  Resumes audio acquisition
 * @param  none
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */

static int8_t Audio_Resume() {
	return bsp_mic_resume();
	}

/**
 * @brief  Manages command from usb
 * @param  None
 * @retval BSP_ERROR_NONE in case of success, AUDIO_ERROR otherwise
 */

static int8_t Audio_CommandMgr(uint8_t cmd) {
	return USBD_OK;
	}

/**
 * Implement the HAL interrupt callbacks that process completed milliseconds of data
 */


void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	bsp_mic_i2s_half_complete();
	}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	bsp_mic_i2s_complete();
	}



