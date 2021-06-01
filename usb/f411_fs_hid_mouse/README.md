# USB FS HID Mouse

USB full-speed HID mouse implemented with WeAct STM32F411CEU6 "Black Pill" development board using MPU6050 accelerometer for pointer movement and two TTP223 capacitive buttons for left and right mouse buttons.

## Credits

* Udemy [USB-Behind the scenes](https://www.udemy.com/course/usb-behind-the-scenes-hands-on-hid-firmware-development/) code archive, developed for an STM32F429 Discovery board using the USB_OTG_HS interface (PB14, PB15) with internal FS PHY and PB13 for VBUS sensing. Highly recommended to take this course so you can understand the development progression from driver to framework to hid application.


## Development Environment

### Software

* Ubuntu 20.04 AMDx64
* STM32CubeIDE v1.6.0
* CubeMX MCU Firmware version FW_F4 V1.26.1
* Wireshark

### Hardware

* WeAct "Black Pill" STM32F411CEU6 development board 
* STLink v2 clone with [mod for SWO trace](http://eeblog.co.uk/2018/11/29/swo-with-cubemx-using-st-link-clones/)
* GY-521 MPU6050 6-axis accelerometer+gyroscope
* TTP223 capacitive switches

  
## Project Notes

* USB full-speed HID mouse uses the development board micro-usb interface connected to the USB_OTG_FS interface (PA11, PA12).
* USB device configured as bus-powered without VBUS sensing (i.e. device assumes it is connected to USB bus if powered on). 
* The STLINK clone adapter ground, SWO, SWDIO and SWCLK pins are connected to the development board
* Left and right mouse buttons uses capacitive TTP223 switch modules connected to GPIO pins PB4 and PB5. The switch modules are powered by the board 3V3 supply and are in default configuration : logic high when pressed, logic low otherwise.
* Mouse pointer x and y movement uses MPU6050 accelerometer readings : roll left and right, pitch up and down.
* MPU6050 module is powered by the board 3V3 supply and uses I2C1 interface pins PB6 (SCL) and PB7 (SDA). The accelerometer readings are polled at the USB HID configured interval of 50 frames = 50mS.
* Debug printf logging via SWO trace (pin PB3).
* The software does not use HAL or LL libraries - only CMSIS headers.
    

