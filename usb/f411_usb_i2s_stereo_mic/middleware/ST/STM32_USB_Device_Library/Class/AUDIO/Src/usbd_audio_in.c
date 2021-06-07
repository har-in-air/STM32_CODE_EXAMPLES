#include "usbd_audio_in.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


/** @defgroup USBD_AUDIO_IN
 *
 * 	This file provides the Audio Input core functions.
 *
 *           This driver implements the following aspects:
 *             - Device descriptor management
 *             - Configuration descriptor management
 *             - Standard AC Interface Descriptor management
 *             - 1 Audio Streaming Interface
 *             - 1 Audio Streaming Endpoint
 *             - 1 Audio Terminal Input
 *             - Audio Class-Specific AC Interfaces
 *             - Audio Class-Specific AS Interfaces
 *             - AudioControl Requests: mute and volume control
 *             - Audio Synchronization type: Asynchronous
 *             - Multiple frequencies and channel number configurable using ad hoc
 *               init function
 *
 *          The current audio class version supports the following audio features:
 *             - Pulse Coded Modulation (PCM) format
 *             - Configurable sampling rate
 *             - Bit resolution: 16
 *             - Configurable Number of channels
 *             - Volume control
 *             - Mute/Unmute capability
 *             - Asynchronous Endpoints
 *
 * @note     This driver has been developed starting from the usbd_audio.c file
 *           included within the standard Cube Package for STM32F4
 * @{
 */


static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t *length);
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetResolution(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/* This dummy buffer with 0 values will be sent when there is no available data */
static uint8_t IsocInBuffDummy[48 * 4 * 2];
static int16_t VOL_CUR;

static USBD_AUDIO_HandleTypeDef haudioInstance;


USBD_ClassTypeDef USBD_AUDIO = {
	USBD_AUDIO_Init,
	USBD_AUDIO_DeInit,
	USBD_AUDIO_Setup,
	USBD_AUDIO_EP0_TxReady,
	USBD_AUDIO_EP0_RxReady,
	USBD_AUDIO_DataIn,
	USBD_AUDIO_DataOut,
	USBD_AUDIO_SOF,
	USBD_AUDIO_IsoINIncomplete,
	USBD_AUDIO_IsoOutIncomplete,
	USBD_AUDIO_GetCfgDesc,
	USBD_AUDIO_GetCfgDesc,
	USBD_AUDIO_GetCfgDesc,
	USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
/* NOTE: This descriptor has to be filled using the Descriptor Initialization function */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ + 9] __ALIGN_END;

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
USB_LEN_DEV_QUALIFIER_DESC,
USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00, };


/**
 * @brief  USBD_AUDIO_Init
 *         Initialize the AUDIO interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  if (haudioInstance.state != STATE_USB_WAITING_FOR_INIT) {
    return USBD_FAIL;
  }

  USBD_AUDIO_HandleTypeDef *haudio;
  pdev->pClassData = &haudioInstance;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  uint16_t packet_dim = haudio->packetDimension;
  uint16_t wr_rd_offset = (AUDIO_IN_PACKET_NUM / 2) * haudio->dataAmount / haudio->packetDimension;
  haudio->wr_ptr = wr_rd_offset * packet_dim;
  haudio->rd_ptr = 0;
  haudio->timeout = 0;

  ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->Init(haudio->frequency, 0, haudio->channels);

  USBD_LL_OpenEP(pdev,  AUDIO_IN_EP,  USBD_EP_TYPE_ISOC,  AUDIO_IN_PACKET_BYTES);
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  USBD_LL_Transmit(pdev, AUDIO_IN_EP, IsocInBuffDummy, packet_dim);
  haudio->state = STATE_USB_IDLE;
  return USBD_OK;
}

/**
 * @brief  USBD_AUDIO_DeInit
 *         DeInitialize the AUDIO layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  /* Close EP IN */
  USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  /* DeInit  physical Interface components */
  if (pdev->pClassData != NULL) {
    ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->DeInit(0);
    haudioInstance.state = STATE_USB_WAITING_FOR_INIT;
  }
  return USBD_OK;
}

/**
 * @brief  USBD_AUDIO_Setup
 *         Handle the AUDIO specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint8_t ret = USBD_OK;
  haudio = pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
  /* AUDIO Class Requests -------------------------------*/
  case USB_REQ_TYPE_CLASS:
    switch (req->bRequest) {
    case AUDIO_REQ_GET_CUR:
      AUDIO_REQ_GetCurrent(pdev, req);
      break;

    case AUDIO_REQ_SET_CUR:
      AUDIO_REQ_SetCurrent(pdev, req);
      break;

    case AUDIO_REQ_GET_MIN:
      AUDIO_REQ_GetMinimum(pdev, req);
      break;

    case AUDIO_REQ_GET_MAX:
      AUDIO_REQ_GetMaximum(pdev, req);
      break;

    case AUDIO_REQ_GET_RES:
      AUDIO_REQ_GetResolution(pdev, req);
      break;

    default:
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
    }
    break;

    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest) {
    case USB_REQ_GET_DESCRIPTOR:
      if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE) {

        pbuf = USBD_AUDIO_CfgDesc + 18;
        len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);

        USBD_CtlSendData(pdev, pbuf, len);
      }
      break;

    case USB_REQ_GET_INTERFACE:
      USBD_CtlSendData(pdev, (uint8_t*) haudio->alt_setting, 1);
      break;

    case USB_REQ_SET_INTERFACE:
      if ((uint8_t) (req->wValue) < USBD_MAX_NUM_INTERFACES) {
        haudio->alt_setting = (uint8_t) (req->wValue);
      } else {
        /* Call the error management function (command will be nacked */
        USBD_CtlError(pdev, req);
      }
      break;
    }
  }
  return ret;
}

/**
 * @brief  USBD_AUDIO_GetCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t *length) {
  *length = sizeof(USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}

/**
 * @brief  USBD_AUDIO_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	USBD_AUDIO_HandleTypeDef *haudio;
	haudio = pdev->pClassData;
	uint32_t length_usb_pck;
	uint16_t app;
	uint16_t IsocInWr_app = haudio->wr_ptr;
	uint16_t true_dim = haudio->buffer_length;
	uint16_t packet_dim = haudio->packetDimension;
	uint16_t channels = haudio->channels;
	length_usb_pck = packet_dim;
	haudio->timeout = 0;
	if (epnum == (AUDIO_IN_EP & 0x7F)) {
		if (haudio->state == STATE_USB_IDLE) {
			haudio->state = STATE_USB_REQUESTS_STARTED;
			((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->Record();
			}
		if (haudio->state == STATE_USB_BUFFER_WRITE_STARTED) {
			haudio->rd_ptr = haudio->rd_ptr % (true_dim);
			if (IsocInWr_app < haudio->rd_ptr) {
				app = ((true_dim) - haudio->rd_ptr) + IsocInWr_app;
				}
			else {
				app = IsocInWr_app - haudio->rd_ptr;
				}
			if (app >= (packet_dim * haudio->upper_threshold)) {
				length_usb_pck += channels * 2;
			}
			else if (app <= (packet_dim * haudio->lower_threshold)) {
				length_usb_pck -= channels * 2;
				}
			USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*) (&haudio->buffer[haudio->rd_ptr]), length_usb_pck);
			haudio->rd_ptr += length_usb_pck;

			if (app < haudio->buffer_length / 10) {
				((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->Stop();
				haudio->state = STATE_USB_IDLE;
				haudio->timeout = 0;
				memset(haudio->buffer, 0, (haudio->buffer_length + haudio->dataAmount));
				}
			} // STATE_USB_BUFFER_WRITE_STARTED
		else {
			USBD_LL_Transmit(pdev, AUDIO_IN_EP, IsocInBuffDummy, length_usb_pck);
			}
		}
	return USBD_OK;
	}

/**
 * @brief  USBD_AUDIO_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */

static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev) {
	USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassData;
	if (haudio->control.unit != AUDIO_OUT_STREAMING_CTRL) {
		return USBD_OK;
		}

	switch (haudio->control.cmd) {
		case AUDIO_CTRL_REQ_SET_CUR_VOLUME:
			((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->VolumeCtl(VOL_CUR);
			haudio->control.cmd = 0;
			haudio->control.len = 0;
			haudio->control.unit = 0;
			haudio->control.data[0] = 0;
			haudio->control.data[0] = 0;
			break;

		case AUDIO_CTRL_REQ_SET_CUR_EQUALIZER:
			// TODO:
			break;
		}

	return USBD_OK;
	}


/**
 * @brief  USBD_AUDIO_EP0_TxReady
 *         handle EP0 TRx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev) {
	/* Only OUT control data are processed */
	return USBD_OK;
	}

/**
 * @brief  USBD_AUDIO_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev) {
	return USBD_OK;
	}

/**
 * @brief  USBD_AUDIO_IsoINIncomplete
 *         handle data ISO IN Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	return USBD_OK;
	}

/**
 * @brief  USBD_AUDIO_IsoOutIncomplete
 *         handle data ISO OUT Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	return USBD_OK;
	}

/**
 * @brief  USBD_AUDIO_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	return USBD_OK;
	}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length) {
	*length = sizeof(USBD_AUDIO_DeviceQualifierDesc);
	return USBD_AUDIO_DeviceQualifierDesc;
	}

/**
 * @brief  AUDIO_REQ_GetMaximum
 *         Handles the VOL_MAX Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMaximum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassData;
	uint8_t bControlSelector = req->wValue >> 8;

	switch (bControlSelector) {
		case FEATURE_VOLUME:
		(haudio->control.data)[0] = (uint16_t) VOL_MAX & 0xFF;
		(haudio->control.data)[1] = ((uint16_t) VOL_MAX & 0xFF00) >> 8;
		break;

		case FEATURE_GRAPHIC_EQ:
		break;
		}

	USBD_CtlSendData(pdev, haudio->control.data, req->wLength);
	}


/**
 * @brief  AUDIO_REQ_GetMinimum
 *         Handles the VOL_MIN Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMinimum(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassData;
	uint8_t bControlSelector = req->wValue >> 8;

	switch (bControlSelector) {
		case FEATURE_VOLUME:
		(haudio->control.data)[0] = (uint16_t) VOL_MIN & 0xFF;
		(haudio->control.data)[1] = ((uint16_t) VOL_MIN & 0xFF00) >> 8;
		/* Send the current mute state */
		break;

		case FEATURE_GRAPHIC_EQ:
		break;
		}

	USBD_CtlSendData(pdev, haudio->control.data, req->wLength);
	}


/**
 * @brief  AUDIO_Req_GetResolution
 *         Handles the VOL_RES Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetResolution(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassData;
	uint8_t bControlSelector = req->wValue >> 8;

	switch (bControlSelector) {
		case FEATURE_VOLUME:
		(haudio->control.data)[0] = (uint16_t) VOL_RES & 0xFF;
		(haudio->control.data)[1] = ((uint16_t) VOL_RES & 0xFF00) >> 8;
		break;

		default:
		break;
		}

	USBD_CtlSendData(pdev, haudio->control.data, req->wLength);
	}


/**
 * @brief  AUDIO_Req_GetCurrent
 *         Handles the GET_CUR Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassData;
	uint8_t bControlSelector = req->wValue >> 8;

	char buffer[20];
	itoa(bControlSelector, buffer, 10);
	puts(buffer);

	switch (bControlSelector) {
		case FEATURE_VOLUME:
		(haudio->control.data)[0] = (uint16_t) VOL_CUR & 0xFF;
		(haudio->control.data)[1] = ((uint16_t) VOL_CUR & 0xFF00) >> 8;
		break;

		default:
		break;
		}

	USBD_CtlSendData(pdev, haudio->control.data, req->wLength);
	}

/**
 * @brief  AUDIO_Req_SetCurrent
 *         Handles the SET_CUR Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	if (req->wLength == 0) {
		return;
		}
	USBD_AUDIO_HandleTypeDef *haudio;
	haudio = pdev->pClassData;
	uint8_t bControlSelector = req->wValue >> 8;
	switch (bControlSelector) {
		case FEATURE_VOLUME:
		haudio->control.cmd = AUDIO_CTRL_REQ_SET_CUR_VOLUME; /* Set the request value */
		haudio->control.len = req->wLength; /* Set the request data length */
		haudio->control.unit = HIBYTE(req->wIndex); /* Set the request target unit */
		USBD_CtlPrepareRx(pdev, (uint8_t*) &VOL_CUR, req->wLength);
		break;

		default :
		break;
		}
	}


/**
 * @brief  USBD_AUDIO_Data_Transfer
 *         Fills the USB internal buffer with audio data from user
 * @param pdev: device instance
 * @param audioData: audio data to be sent via USB
 * @param dataAmount: number of PCM samples to be copied
 * @note Depending on the calling frequency, a coherent amount of samples must be passed to
 *       the function. E.g.: assuming a Sampling frequency of 16 KHz and 1 channel,
 *       you can pass 16 PCM samples if the function is called each millisecond,
 *       32 samples if called every 2 milliseconds and so on.
 * @retval status
 */
uint8_t USBD_AUDIO_Data_Transfer(USBD_HandleTypeDef *pdev, int16_t *audioData, uint16_t PCMSamples) {
	USBD_AUDIO_HandleTypeDef *haudio;
	haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
	if (haudioInstance.state == STATE_USB_WAITING_FOR_INIT) {
		return USBD_BUSY;
		}
	uint16_t dataAmount = PCMSamples * 2 * haudio->channels; //Bytes
	uint16_t true_dim = haudio->buffer_length;
	uint16_t current_data_Amount = haudio->dataAmount;
	uint16_t packet_dim = haudio->packetDimension;

	if (haudio->state == STATE_USB_REQUESTS_STARTED || current_data_Amount != dataAmount) {
		//USB parameters definition, based on the amount of data passed
		haudio->dataAmount = dataAmount;
		uint16_t wr_rd_offset = (AUDIO_IN_PACKET_NUM / 2) * dataAmount / packet_dim;
		haudio->wr_ptr = wr_rd_offset * packet_dim;
		haudio->rd_ptr = 0;
		haudio->upper_threshold = wr_rd_offset + 1;
		haudio->lower_threshold = wr_rd_offset - 1;
		haudio->buffer_length = (packet_dim * (dataAmount / packet_dim) * AUDIO_IN_PACKET_NUM);

		// Memory allocation for data buffer, depending (also) on data amount passed to the transfer function
		if (haudio->buffer != NULL) {
			USBD_free(haudio->buffer);
			}
		haudio->buffer = USBD_malloc(haudio->buffer_length + haudio->dataAmount);
		if (haudio->buffer == NULL) {
			return USBD_FAIL;
			}
		memset(haudio->buffer, 0, (haudio->buffer_length + haudio->dataAmount));
		haudio->state = STATE_USB_BUFFER_WRITE_STARTED;
		}
	else
	if (haudio->state == STATE_USB_BUFFER_WRITE_STARTED) {
		if (haudio->timeout++ == TIMEOUT_VALUE) {
			haudio->state = STATE_USB_IDLE;
			((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->Stop();
			haudio->timeout = 0;
			}
		memcpy((uint8_t*) &haudio->buffer[haudio->wr_ptr], (uint8_t*) (audioData), dataAmount);
		haudio->wr_ptr += dataAmount;
		haudio->wr_ptr = haudio->wr_ptr % (true_dim);
		if ((haudio->wr_ptr - dataAmount) == 0) {
			memcpy((uint8_t*) (((uint8_t*) haudio->buffer) + true_dim), (uint8_t*) haudio->buffer, dataAmount);
			}
		}
	return USBD_OK;
	}

/**
 * @brief  USBD_AUDIO_RegisterInterface
 * @param  fops: Audio interface callback
 * @retval status
 */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_AUDIO_ItfTypeDef *fops) {
  if (fops != NULL) {
    pdev->pUserData = fops;
  }
  return 0;
}

/**
 * @brief  Configures the microphone descriptor on the base of the frequency
 *         and channels number informations. These parameters will be used to
 *         init the audio engine, trough the USB interface functions.
 * @param  samplingFrequency: sampling frequency
 * @param  Channels: number of channels
 * @retval status
 */
void USBD_AUDIO_Init_Microphone_Descriptor(USBD_HandleTypeDef *pdev, uint32_t samplingFrequency, uint8_t Channels) {
  uint16_t index;
  uint8_t AUDIO_CONTROLS;
  USBD_AUDIO_CfgDesc[0] = 0x09; /* bLength */
  USBD_AUDIO_CfgDesc[1] = 0x02; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[2] = ((USB_AUDIO_CONFIG_DESC_SIZ + Channels - 1) & 0xff); /* wTotalLength */
  USBD_AUDIO_CfgDesc[3] = ((USB_AUDIO_CONFIG_DESC_SIZ + Channels - 1) >> 8);
  USBD_AUDIO_CfgDesc[4] = 0x02; /* bNumInterfaces */
  USBD_AUDIO_CfgDesc[5] = 0x01; /* bConfigurationValue */
  USBD_AUDIO_CfgDesc[6] = 0x00; /* iConfiguration */
  USBD_AUDIO_CfgDesc[7] = 0x80; /* bmAttributes  BUS Powered*/
  USBD_AUDIO_CfgDesc[8] = 0x32; /* bMaxPower = 100 mA*/
  /* USB Microphone Standard interface descriptor */
  USBD_AUDIO_CfgDesc[9] = 9; /* bLength */
  USBD_AUDIO_CfgDesc[10] = USB_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[11] = 0x00; /* bInterfaceNumber */
  USBD_AUDIO_CfgDesc[12] = 0x00; /* bAlternateSetting */
  USBD_AUDIO_CfgDesc[13] = 0x00; /* bNumEndpoints */
  USBD_AUDIO_CfgDesc[14] = USB_DEVICE_CLASS_AUDIO; /* bInterfaceClass */
  USBD_AUDIO_CfgDesc[15] = AUDIO_SUBCLASS_AUDIOCONTROL; /* bInterfaceSubClass */
  USBD_AUDIO_CfgDesc[16] = AUDIO_PROTOCOL_UNDEFINED; /* bInterfaceProtocol */
  USBD_AUDIO_CfgDesc[17] = 0x00; /* iInterface */
  /* USB Microphone Class-specific AC Interface Descriptor */
  USBD_AUDIO_CfgDesc[18] = 9; /* bLength */
  USBD_AUDIO_CfgDesc[19] = AUDIO_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[20] = AUDIO_CONTROL_HEADER; /* bDescriptorSubtype */
  USBD_AUDIO_CfgDesc[21] = 0x00; /* 1.00 *//* bcdADC */
  USBD_AUDIO_CfgDesc[22] = 0x01;
  USBD_AUDIO_CfgDesc[23] = 0x25 + Channels; /* wTotalLength = 37+AUDIO_CHANNELS*/
  USBD_AUDIO_CfgDesc[24] = 0x00;
  USBD_AUDIO_CfgDesc[25] = 0x01; /* bInCollection */
  USBD_AUDIO_CfgDesc[26] = 0x01; /* baInterfaceNr */
  /* USB Microphone Input Terminal Descriptor */
  USBD_AUDIO_CfgDesc[27] = AUDIO_INPUT_TERMINAL_DESC_SIZE; /* bLength */
  USBD_AUDIO_CfgDesc[28] = AUDIO_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[29] = AUDIO_CONTROL_INPUT_TERMINAL; /* bDescriptorSubtype */
  USBD_AUDIO_CfgDesc[30] = 0x01; /* bTerminalID */
  USBD_AUDIO_CfgDesc[31] = 0x01; /* wTerminalType AUDIO_TERMINAL_USB_MICROPHONE   0x0201 */
  USBD_AUDIO_CfgDesc[32] = 0x02;
  USBD_AUDIO_CfgDesc[33] = 0x00; /* bAssocTerminal */
  USBD_AUDIO_CfgDesc[34] = Channels; /* bNrChannels */
  if (Channels != 2) {
    USBD_AUDIO_CfgDesc[35] = 0x00; /* wChannelConfig 0x0000  Mono */
    USBD_AUDIO_CfgDesc[36] = 0x00;
  } else {
    USBD_AUDIO_CfgDesc[35] = 0x03; /* wChannelConfig 0x0003  Stereo */
    USBD_AUDIO_CfgDesc[36] = 0x00;
  }
  USBD_AUDIO_CfgDesc[37] = 0x00; /* iChannelNames */
  USBD_AUDIO_CfgDesc[38] = 0x00; /* iTerminal */
  /* USB Microphone Audio Feature Unit Descriptor */
  USBD_AUDIO_CfgDesc[39] = 0x07 + Channels + 1; /* bLength */
  USBD_AUDIO_CfgDesc[40] = AUDIO_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[41] = AUDIO_CONTROL_FEATURE_UNIT; /* bDescriptorSubtype */
  USBD_AUDIO_CfgDesc[42] = 0x02; /* bUnitID */
  USBD_AUDIO_CfgDesc[43] = 0x01; /* bSourceID */
  USBD_AUDIO_CfgDesc[44] = 0x01; /* bControlSize */
  index = 47;
  if (Channels == 1) {
    AUDIO_CONTROLS = (FEATURE_VOLUME);
    USBD_AUDIO_CfgDesc[45] = AUDIO_CONTROLS;
    USBD_AUDIO_CfgDesc[46] = 0x00;
  } else {
    AUDIO_CONTROLS = (FEATURE_VOLUME);
    USBD_AUDIO_CfgDesc[45] = 0x00;
    USBD_AUDIO_CfgDesc[46] = AUDIO_CONTROLS;
    USBD_AUDIO_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }
  if (Channels > 2) {
    USBD_AUDIO_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }
  if (Channels > 3) {
    USBD_AUDIO_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }
  if (Channels > 4) {
    USBD_AUDIO_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }
  if (Channels > 5) {
    USBD_AUDIO_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }
  if (Channels > 6) {
    USBD_AUDIO_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }
  if (Channels > 7) {
    USBD_AUDIO_CfgDesc[index] = AUDIO_CONTROLS;
    index++;
  }
  USBD_AUDIO_CfgDesc[index] = 0x00; /* iTerminal */
  index++;
  /*USB Microphone Output Terminal Descriptor */
  USBD_AUDIO_CfgDesc[index++] = 0x09; /* bLength */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_CONTROL_OUTPUT_TERMINAL; /* bDescriptorSubtype */
  USBD_AUDIO_CfgDesc[index++] = 0x03; /* bTerminalID */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* wTerminalType AUDIO_TERMINAL_USB_STREAMING 0x0101*/
  USBD_AUDIO_CfgDesc[index++] = 0x01;
  USBD_AUDIO_CfgDesc[index++] = 0x00;
  USBD_AUDIO_CfgDesc[index++] = 0x02;
  USBD_AUDIO_CfgDesc[index++] = 0x00;
  /* USB Microphone Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
  USBD_AUDIO_CfgDesc[index++] = 9; /* bLength */
  USBD_AUDIO_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* bInterfaceNumber */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* bAlternateSetting */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* bNumEndpoints */
  USBD_AUDIO_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO; /* bInterfaceClass */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOSTREAMING; /* bInterfaceSubClass */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED; /* bInterfaceProtocol */
  USBD_AUDIO_CfgDesc[index++] = 0x00;
  /* USB Microphone Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  USBD_AUDIO_CfgDesc[index++] = 9; /* bLength */
  USBD_AUDIO_CfgDesc[index++] = USB_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* bInterfaceNumber */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* bAlternateSetting */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* bNumEndpoints */
  USBD_AUDIO_CfgDesc[index++] = USB_DEVICE_CLASS_AUDIO; /* bInterfaceClass */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_SUBCLASS_AUDIOSTREAMING; /* bInterfaceSubClass */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_PROTOCOL_UNDEFINED; /* bInterfaceProtocol */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* iInterface */
  /* USB Microphone Audio Streaming Interface Descriptor */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_STREAMING_INTERFACE_DESC_SIZE; /* bLength */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_STREAMING_GENERAL; /* bDescriptorSubtype */
  USBD_AUDIO_CfgDesc[index++] = 0x03; /* bTerminalLink */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* bDelay */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  USBD_AUDIO_CfgDesc[index++] = 0x00;
  /* USB Microphone Audio Type I Format Interface Descriptor */
  USBD_AUDIO_CfgDesc[index++] = 0x0B; /* bLength */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_INTERFACE_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_STREAMING_FORMAT_TYPE; /* bDescriptorSubtype */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_FORMAT_TYPE_I; /* bFormatType */
  USBD_AUDIO_CfgDesc[index++] = Channels; /* bNrChannels */
  USBD_AUDIO_CfgDesc[index++] = 0x02; /* bSubFrameSize */
  USBD_AUDIO_CfgDesc[index++] = 16; /* bBitResolution */
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* bSamFreqType */
  USBD_AUDIO_CfgDesc[index++] = samplingFrequency & 0xff; /* tSamFreq 8000 = 0x1F40 */
  USBD_AUDIO_CfgDesc[index++] = (samplingFrequency >> 8) & 0xff;
  USBD_AUDIO_CfgDesc[index++] = samplingFrequency >> 16;
  /* Endpoint 1 - Standard Descriptor */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_STANDARD_ENDPOINT_DESC_SIZE; /* bLength */
  USBD_AUDIO_CfgDesc[index++] = 0x05; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_IN_EP; /* bEndpointAddress 1 in endpoint*/
  USBD_AUDIO_CfgDesc[index++] = 0x05; /* bmAttributes */
  USBD_AUDIO_CfgDesc[index++] = ((samplingFrequency / 1000 + 2) * Channels * 2) & 0xFF; /* wMaxPacketSize */
  USBD_AUDIO_CfgDesc[index++] = ((samplingFrequency / 1000 + 2) * Channels * 2) >> 8;
  USBD_AUDIO_CfgDesc[index++] = 0x01; /* bInterval */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* bRefresh */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* bSynchAddress */
  /* Endpoint - Audio Streaming Descriptor*/
  USBD_AUDIO_CfgDesc[index++] = AUDIO_STREAMING_ENDPOINT_DESC_SIZE; /* bLength */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_ENDPOINT_DESCRIPTOR_TYPE; /* bDescriptorType */
  USBD_AUDIO_CfgDesc[index++] = AUDIO_ENDPOINT_GENERAL; /* bDescriptor */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* bmAttributes */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* bLockDelayUnits */
  USBD_AUDIO_CfgDesc[index++] = 0x00; /* wLockDelay */
  USBD_AUDIO_CfgDesc[index++] = 0x00;

  haudioInstance.packetDimension = (samplingFrequency / 1000 * Channels * 2);
  haudioInstance.frequency = samplingFrequency;
  haudioInstance.buffer_length = haudioInstance.packetDimension * AUDIO_IN_PACKET_NUM;
  haudioInstance.channels = Channels;
  haudioInstance.upper_threshold = 5;
  haudioInstance.lower_threshold = 2;
  haudioInstance.state = STATE_USB_WAITING_FOR_INIT;
  haudioInstance.wr_ptr = 3 * haudioInstance.packetDimension;
  haudioInstance.rd_ptr = 0;
  haudioInstance.dataAmount = 0;
  haudioInstance.buffer = 0;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
