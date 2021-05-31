#include "usb/usbd_driver.h"
#include "usb/usbd_descriptors.h"
#include "util/logger.h"
#include "btn.h"
#include "mpu6050.h"


void hid_write_mouse_report(){
	//log_debug("Sending USB HID mouse report.");
	uint8_t btn_l = btn_read(BTN_L);
	uint8_t btn_r = btn_read(BTN_R);
	mpu6050_read_accel();
	int8_t dax = -Ayi;
	int8_t day = Axi;

	HID_REPORT_t hid_report = {
		.dx = dax,
		.dy = day,
		.buttons = (btn_l<<1) | btn_r
		};
    USB_Driver.write_packet(
		(Config_Desc_Set.mouse_ep_desc.bEndpointAddress & 0x0F),
		&hid_report,
		sizeof(HID_REPORT_t)
		);
	}


