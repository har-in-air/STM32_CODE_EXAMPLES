#include "usb/usbd_descriptors.h"

const USB_DEVICE_DESC_t Device_Descriptor = {
    .bLength            = sizeof(USB_DEVICE_DESC_t),
    .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB             = 0x0200, // 0xJJMN
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTOCOL_NONE,
    .bMaxPacketSize0    = 8,
    .idVendor           = 0x6666, // for prototype devices
    .idProduct          = 0x13AA, // random product 
    .bcdDevice          = 0x0100, // product version 1.00
    .iManufacturer      = 0, // no manufacturer string 
    .iProduct           = 0, // no product name string 
    .iSerialNumber      = 0, // no serial number string 
    .bNumConfigurations = 1,
};

// not defined as struct but an array
// describes the app specific HID_REPORT_t structure
// for mouse with 2 buttons and dx,dy
// set usage page equivalent to setting namespace
// order must be same as in the HID_REPORT_t structure
const uint8_t HID_Report_Descriptor[] = {
	// set usage page to generic desktop
	HID_USAGE_PAGE(HID_PAGE_DESKTOP),
	HID_USAGE(HID_DESKTOP_MOUSE), // desktop mouse is available inside desktop
	HID_COLLECTION(HID_APPLICATION_COLLECTION),
		HID_USAGE(HID_DESKTOP_POINTER),
		HID_COLLECTION(HID_PHYSICAL_COLLECTION),
			HID_USAGE(HID_DESKTOP_X),
			HID_USAGE(HID_DESKTOP_Y), // no need to duplicate for x and y (compressed)
			// value fits in a signed byte
			HID_LOGICAL_MINIMUM(-127),
			HID_LOGICAL_MAXIMUM(127),
			// 8bits
			HID_REPORT_SIZE(8),
			// x and y reports
			HID_REPORT_COUNT(2),
			HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE),

			// set usage page to button page
			HID_USAGE_PAGE(HID_PAGE_BUTTON),
			// minimum number of buttons = 1
			HID_USAGE_MINIMUM(1),
			// maximum number of buttons = 2
			HID_USAGE_MAXIMUM(2),
			// value 0 is logical minimum
			HID_LOGICAL_MINIMUM(0),
			// value 1 is logical maximum
			HID_LOGICAL_MAXIMUM(1),
			// size of logical value = 1bit
			HID_REPORT_SIZE(1),
			// group of 2 buttons = 2bits
			HID_REPORT_COUNT(2),
			// this is an input report, its variable, and absolute (not relative)
			HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),
			// pad the rest of the byte (6bits) specified as constant
			HID_REPORT_SIZE(1),
			HID_REPORT_COUNT(6),
			HID_INPUT(HID_IOF_CONSTANT),
		HID_END_COLLECTION,
	HID_END_COLLECTION
};

const int HID_Report_Desc_Size_Bytes = sizeof(HID_Report_Descriptor);


const USB_CONFIG_DESC_SET_t Config_Desc_Set = {
	.config_desc = {
		.bLength                = sizeof(USB_CONFIG_DESC_t),
		.bDescriptorType        = USB_DESCRIPTOR_TYPE_CONFIGURATION,
		.wTotalLength           = sizeof(USB_CONFIG_DESC_SET_t), // size of the set
		.bNumInterfaces         = 1, // only one function : mouse, so one group of endpoints
		.bConfigurationValue    = 1, // index of this configuration descriptor
		.iConfiguration         = 0, // id of string descriptor describing this configuration (0 = none)
		.bmAttributes           = 0x80 | 0x40, // b7=1 reserved : b6=1 => self-powered : b5=0 => remote wakeup not supported
		.bMaxPower              = 25 // *2 mA => max current drawn = 50mA
	},
	.interface_desc = {
		.bLength                = sizeof(USB_INTERFACE_DESC_t),
		.bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
		.bInterfaceNumber       = 0,
		.bAlternateSetting      = 0,
		.bNumEndpoints          = 1,
		.bInterfaceClass        = USB_CLASS_HID,
		.bInterfaceSubClass     = USB_SUBCLASS_NONE,
		.bInterfaceProtocol     = USB_PROTOCOL_NONE,
		.iInterface             = 0 // id of string descriptor for interface, 0 = none
	},
    .mouse_ep_desc = {
        .bLength                = sizeof(USB_ENDPOINT_DESC_t),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress       = 0x83, // 0x80 => IN endpoint, endpoint #3
        .bmAttributes           = USB_ENDPOINT_TYPE_INTERRUPT,  // periodic transfer of data, not control/bulk/isochronous
        .wMaxPacketSize         = 64, // max possible with full speed
        .bInterval              = 50 // periodic interval between data transfers = 50 frames
    },
    .mouse_hid_desc = { // has one or more hid report descriptors
        .bLength                = sizeof(USB_HID_DESC_t),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_HID,
        .bcdHID                 = 0x0100, // compliant with hid version 1.0
        .bCountryCode           = USB_HID_COUNTRY_NONE, // no localization requirement
        .bNumDescriptors        = 1, // one descriptor associated with this hid descriptor
        .bDescriptorType0       = USB_DESCRIPTOR_TYPE_HID_REPORT, // which will be a HID report descriptor
        .wDescriptorLength0     = sizeof(HID_Report_Descriptor) // with this specified size
    }
};

