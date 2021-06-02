#include "usb/usb_device.h"
#include "usb/usbd_framework.h"
#include "util/logger.h"
#include "util/misc.h"
#include "btn.h"
#include "i2c.h"
#include "mpu6050.h"


int main(void){
	config_swdio_pins();
	system_core_clock_update();
	log_info("F407 : SystemCoreClock = %u", SystemCoreClock);
	btn_init();
    i2c_init();
    mpu6050_init();
	usbd_initialize();

	while(1){
		usbd_poll();
		}
	}


