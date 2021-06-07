#include <stdlib.h>
#include "main.h"
#include "svc_glo.h"
#include "vol_ctrl.h"

// needed by svc library as per UM1642 table 1 : resource summary
uint8_t SvcPersistentMem[1368];
uint8_t SvcScratchMem[2880];

#if 0
buffer_t SvcInputBuffer;
buffer_t SvcOutputBuffer;
#else
buffer_t SvcBuffer;
#endif

static svc_static_param_t 	static_param;
static svc_dynamic_param_t 	dynamic_param;


void vol_init() {
	// CRC initialization and reset is required by STM32 Software Volume Control (SVC) library
	__HAL_RCC_CRC_CLK_ENABLE();
	CRC->CR = CRC_CR_RESET;

	// allocate space required by the SVC library
	//SvcPersistentMem = malloc(svc_persistent_mem_size);  // 1368
	//SvcScratchMem = malloc(svc_scratch_mem_size);  // 2880

	// reset the library
	if (svc_reset(SvcPersistentMem, SvcScratchMem) != SVC_ERROR_NONE) {
		Error_Handler();
		}

	// set the static parameters. These defaults are from ST's recommendations.
	static_param.delay_len = 80;
	static_param.joint_stereo = 1;
	if (svc_setParam(&static_param, SvcPersistentMem) != SVC_ERROR_NONE) {
		Error_Handler();
		}
	// set the initial volume
	vol_set_volume(18);

#if 0
	// buffer constants
	SvcInputBuffer.nb_channels = 2;
	SvcInputBuffer.nb_bytes_per_Sample = 2;
	SvcInputBuffer.mode = INTERLEAVED;

	SvcOutputBuffer.nb_channels = 2;
	SvcOutputBuffer.nb_bytes_per_Sample = 2;
	SvcOutputBuffer.mode = INTERLEAVED;
#else
	SvcBuffer.nb_channels = 2;
	SvcBuffer.nb_bytes_per_Sample = 2;
	SvcBuffer.mode = INTERLEAVED;
#endif
	}


/**
 * Set the volume level. The range is -80db to +36db. Positive values amplify, negative
 * values attenuate. The range is in 0.5dB steps therefore volume parameter range is -160..72
 */

void vol_set_volume(int16_t volume) {
	// unmuted, target volume provided externally
	dynamic_param.mute = 0;
	dynamic_param.target_volume_dB = volume;
	// enable compression, high quality, use timings from ST's sample application
	dynamic_param.enable_compr = 1;
	dynamic_param.quality = 1;
	dynamic_param.attack_time = 2103207220;
	dynamic_param.release_time = 2146924480;

	int32_t error = svc_setConfig(&dynamic_param, SvcPersistentMem);
	if (error != SVC_ERROR_NONE) {
		Error_Handler();
		}
	}


/**
 * Process a sample buffer
 */

inline void vol_process_buffer(int16_t *iobuffer, int32_t nSamples) {
	// initialise the buffer parameters
#if 0
	SvcInputBuffer.buffer_size = nSamples;
	SvcOutputBuffer.buffer_size = nSamples;
	SvcInputBuffer.nb_channels = 2;
	SvcOutputBuffer.nb_channels = 2;
	SvcInputBuffer.nb_bytes_per_Sample = 2;
	SvcOutputBuffer.nb_bytes_per_Sample = 2;
	SvcInputBuffer.data_ptr = iobuffer;
	SvcOutputBuffer.data_ptr = iobuffer;
	SvcInputBuffer.mode = INTERLEAVED;
	SvcOutputBuffer.mode = INTERLEAVED;
	// call the library method
	int32_t error = svc_process(&SvcInputBuffer, &SvcOutputBuffer, SvcPersistentMem);
#else
	SvcBuffer.buffer_size = nSamples;
	SvcBuffer.data_ptr = iobuffer;
	int32_t error = svc_process(&SvcBuffer, &SvcBuffer, SvcPersistentMem);
#endif
	if (error != SVC_ERROR_NONE) {
		Error_Handler();
		}
	}
