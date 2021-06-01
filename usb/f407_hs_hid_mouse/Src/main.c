#include "usb/usb_device.h"
#include "usb/usbd_framework.h"
#include "util/logger.h"
#include "util/misc.h"
#include "btn.h"
#include "i2c.h"
#include "mpu6050.h"

USB_DEVICE_t USB_Device;
uint32_t Buffer[8];

int main(void){
	system_core_clock_update();
	log_info("F407 SystemCoreClock = %u", SystemCoreClock);
	btn_init();
    i2c_init();
    mpu6050_init();

	USB_Device.ptr_out_buffer = Buffer;
	usbd_initialize(&USB_Device);

	while(1){
		usbd_poll();
		}
	}
