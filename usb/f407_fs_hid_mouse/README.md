# USB FS HID Self-powered Mouse

USB full-speed HID self-powered mouse with VBUS sensing, implemented with WeAct STM32F407VGT6 development board using MPU6050 accelerometer for pointer movement and two TTP223 capacitive buttons for left and right mouse buttons.

## Credits

* Udemy [USB-Behind the scenes](https://www.udemy.com/course/usb-behind-the-scenes-hands-on-hid-firmware-development/) code archive, developed for an STM32F429 Discovery board using the USB_OTG_HS interface (PB14, PB15) with internal FS PHY and PB13 for VBUS sensing. Highly recommended to take this course so you can understand the development progression from driver to framework to hid application.


## Development Environment

### Software

* Ubuntu 20.04 AMDx64
* STM32CubeIDE v1.6.0
* CubeMX MCU Firmware version FW_F4 V1.26.1
* Wireshark

### Hardware

* DevEBox STM32F407VGT6 development board 
* STLink v2 clone with [mod for SWO trace](http://eeblog.co.uk/2018/11/29/swo-with-cubemx-using-st-link-clones/)
* GY-521 MPU6050 6-axis accelerometer+gyroscope
* TTP223 capacitive switches

<img src = "docs/f407_hid_mouse.jpg"/>

  
## Project Notes

* USB full-speed HID mouse uses the on-board micro-usb interface connected to the USB_OTG_FS peripheral PA11, PA12 pins,  and PA9 for VBUS sensing.
* The micro-usb connector VBUS 5V supply is connected via ferrite bead L1 to the board 5V supply. L1 is actually a 0-ohm resistor on the dev board (see photo below). I disconnected L1 and soldered a resistor divider with 5K6 and 10K resistors from VBUS to ground to get a sensed voltage ~3.3V which is connected to PA9.  
  
<img src = "docs/f407_fs_vbus_sense.jpg"/>

* Since the USB 5V supply has been disconnected, the STLink adapter connector 5V pin now supplies power for the development board via one of the board 5V pins. So now the USB device firmware needs to be configured as self-powered with VBUS sensing. 
* You could optionally leave the board in factory state and configure the USB driver as USB bus-powered, no VBUS sensing. This assumes the device is always connected to USB on power-on. This is a little trickier to re-flash and debug with the STLink clone adapter once usb device firmware has been flashed. You can opt for this device configuration if required, after confirming the  "self-powered with VBUS sensing" configuration works.
* Left and right mouse buttons use capacitive TTP223 switch modules connected to GPIO pins PD4 and PD5. The switch modules are powered by the board 3.3V supply and are in default configuration : logic high when pressed, logic low otherwise.
* Mouse pointer x and y movement use MPU6050 accelerometer readings : roll left and right, pitch up and down.
* The MPU6050 module is powered by the board 3.3V supply and uses I2C1 interface pins PB6 (SCL) and PB7 (SDA). The accelerometer and buttons are polled at the USB HID configured interval of 50 frames (50mS).
* Debug printf logging in STM32CubeIDE ITM Data Console via SWO trace using pin PB3.
* The firmware uses only CMSIS headers, no HAL or LL libraries 
    

