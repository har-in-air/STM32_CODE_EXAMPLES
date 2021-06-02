#ifndef USB_USB_DEVICE_H_
#define USB_USB_DEVICE_H_

#include "../../inc/usb/usb_standards.h"

typedef struct {
	USB_DEVICE_STATE_e 				device_state; // current USB device state.
	USB_CONTROL_TRANSFER_STAGE_e 	control_transfer_stage; //  current control transfer stage (for endpoint0).
	uint8_t 						config_index; // The selected USB configuration.

	void *							ptr_out_buffer;
	uint32_t 						out_data_size;
	const void *					ptr_in_buffer;
	uint32_t 						in_data_size;
} USB_DEVICE_t;

#endif /* USB_USB_DEVICE_H_ */
