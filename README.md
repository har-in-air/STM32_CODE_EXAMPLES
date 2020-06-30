# STM32 Cortex M4 code examples

 I've been
taking Udemy online embedded software courses related to Cortex M microcontrollers. The instructor was using Keil and OpenSystem Workbench IDEs, and I was implementing the code exercises using STM32CubeIDE.

So I thought this would make a good repository of STM32CubeIDE code examples demonstrating Cortex M peripherals and feature usage.

Some of the code exercises required us to understand the HAL apis so we used CubeMX only to pull in the required peripheral libraries and header files. Or programming from scratch using only CubeMX generated startup code and linker files, plus manually added CMSIS and microcontroller
header files. I've commented these projects in more detail. Others use CubeMX generated initialization code. 

## Credits

* [Udemy code repository](https://github.com/niekiran) Note these were implemented for a Nucleo STM32F446 board.
* Project specific credits are in the associated README files.


## Development Environment

* Ubuntu 20.04 AMDx64
* STM32CubeIDE v1.2.0
* FreeRTOS 10.3.1
* SystemView 3.12
* JLink jtag/swd debugger
* STLink v2 clone with [mod for SWO trace](http://eeblog.co.uk/2018/11/29/swo-with-cubemx-using-st-link-clones/)
* WeAct v1.3 STM32F411CEU6 dev board (FreeRTOS, semi-hosting, SWO demos)
* DevEBox STM32F407VGT6 dev board (everything else)
* PL2303 HXD USB-UART adapter - [max data rate 12Mbps](https://www.sjoerdlangkemper.nl/2019/03/20/usb-to-serial-uart/)

## Project Notes

You will find relevant notes in the project sub-directories

