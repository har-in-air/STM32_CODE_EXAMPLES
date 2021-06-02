#include "usb/usbd_framework.h"
#include "util/logger.h"
#include "util/misc.h"
#include "btn.h"
#include "i2c.h"
#include "mpu6050.h"

int main(void){
	config_swdio_pins();
	system_core_clock_update();
	log_info("F411 : SystemCoreClock = %u", SystemCoreClock);
	btn_init();
	i2c_init();
	mpu6050_init();
	usbf_initialize();

	while(1){
		usbf_poll();
		}
	}
