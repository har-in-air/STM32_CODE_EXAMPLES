# USB FS stereo I2S microphone

USB full-speed microphone (sampling rate 48kHz, stereo 16-bit resolution) implemented with WeAct STM32F411CEU6 "Black Pill" development board and two INMP411 I2S microphones. 
This is configured as bus-powered, with no VBUS sensing i.e. the device assumes it is connected to a USB host if powered on. 

## Credits

* [Andy Brown's USB microphone](https://andybrown.me.uk/2021/03/13/usb-microphone/). I have refactored the code as a C project, extended it from mono to stereo microphone recording, and am using the internal I2S PLL to generate the clocks.

## Development Environment

### Software

* Ubuntu 20.04 AMDx64
* STM32CubeIDE v1.6.0
* CubeMX MCU Firmware version FW_F4 V1.26.1

### Hardware

* WeAct "Black Pill" STM32F411CEU6 development board 
* STLink v2 clone with [mod for SWO trace](http://eeblog.co.uk/2018/11/29/swo-with-cubemx-using-st-link-clones/)
* INMP441 24-bit I2S MEMS microphone modules (breakout boards)

  
## Project Notes

* USB full-speed HID mouse uses the on-board micro-usb interface connected to the USB_OTG_FS peripheral pins PA11, PA12.
* USB device configured as bus-powered without VBUS sensing
* The I2S1 peripheral is configured as 
  * half-duplex master receive
  * internal PLL generates the BCK and WS clocks to the microphones
  * 24/32 I2S Philips standard
* In the demo code bsp_mic.c, the two microphones are not used in the traditional stereo L and R configuration. The L microphone
is assumed to be pointing towards the speaker, and the R microphone points 180 degrees away from the speaker. The L and R data are processed to exaggerate the lobing, i.e. L_channel = 2*L_channel - R_channel, and R channel = 2*R_channel  - L_channel. This was just done as an example. 
* I used Audacity on my Ubuntu PC to record and playback the stereo audio. 

