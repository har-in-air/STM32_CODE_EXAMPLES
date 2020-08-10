1. In user application project,
  edit  `STM32F407VGTX_FLASH.ld` and change FLASH ORIGIN to 0x8008000, LENGTH to 992K
```
MEMORY
{
  CCMRAM    (xrw)    : ORIGIN = 0x10000000,   LENGTH = 64K
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  FLASH    (rx)    : ORIGIN = 0x8008000,   LENGTH = 992K
}
```
2. In user application project,
edit  `system_stm32f4xx.c`, set the vector table offset correctly
```
#define VECT_TAB_OFFSET  0x8000 
```

3. User application project : clean and build

3. Use STM32CubeProgrammer to erase the full chip

4. Use STM32CubeIDE to flash the user application

5. Verify with STM32CubeProgrammer that addresses below 0x8008000 are all 0XFF 

6. Use STM32CubeIDE flash the bootloader


### STM32CubeProgrammer not launching in Ubuntu 20.04

[Here is the solution](https://community.st.com/s/question/0D50X0000BsRNNV/stm32cubeprogrammer-execute-fail-with-could-not-find-or-load-main-class-comstappmain)

Go to `https://bell-sw.com/pages/downloads/`, download and install the Debian package `Liberica Full JRE 8u252+9 x86 64 bit for Linux`
```
sudo dpkg -i bellsoft-jre8u252+9-linux-amd64-full.deb
```

### STM32CubeIDE : Flashing the target using STM32CubeProgrammer and STLink without starting a debug session

[Here is the solution](https://community.st.com/s/question/0D50X0000Ap2L8T/flash-and-run-without-debug)

* Create file `makefile.targets` at the root of the STM32CubeIDE project:
```
STM32CubeProgrammer := '/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI'
 
flash: all
	-@echo 'Download hex/bin to MCU Flash ...'
#	$(STM32CubeProgrammer) -c port=SWD -d $(wildcard *.hex) -Rst
	$(STM32CubeProgrammer) -c port=SWD -d $(wildcard *.bin) 0x8000000 -Rst
 
.PHONY: flash
```
* Click Debug, Release or project folder in Project explorer. 
* Press Shift+F9. 
* Add target "flash". Select new target. Click build. Now you can just press F9.

