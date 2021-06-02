#ifndef USB_USBD_DESCRIPTORS_H_
#define USB_USBD_DESCRIPTORS_H_

#include <stdint.h>

#include "../../inc/usb/hid/usb_hid_standards.h"
#include "../../inc/usb/usb_standards.h"

// host requests descriptors with a specified number of bytes as in most cases the size
// is standard. For configuration descriptors with associated interface and endpoint
// descriptors, this is not valid, as this is device firmware dependent.
// USB_CONFIG_DESC_t is standard with a size of 9 bytes (bLength).
// But USB_CONFIG_DESC_t.wTotalLength specifies the total count of bytes in the device specific
// USB_CONFIG_DESC_SET_t

// note : order of the descriptors matters in Windows. HID descriptor must come before the
// endpoint descriptor
typedef struct {
	USB_CONFIG_DESC_t 		config_desc;
	USB_INTERFACE_DESC_t 	interface_desc;
	USB_HID_DESC_t 			mouse_hid_desc;
	USB_ENDPOINT_DESC_t 	mouse_ep_desc;
} USB_CONFIG_DESC_SET_t;


extern const USB_DEVICE_DESC_t Device_Descriptor;
extern const USB_CONFIG_DESC_SET_t Config_Desc_Set;
extern const uint8_t HID_Report_Descriptor[];
extern const int HID_Report_Desc_Size_Bytes;

#endif /* USB_USBD_DESCRIPTORS_H_ */
