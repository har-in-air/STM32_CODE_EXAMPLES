# Openocd flashing STM32CubeIDE generated STM32F411CEU6 binary

PROJECT = f411_swo_demo

OCDDIR = /usr/local/share/openocd


OCDFLAGS = -f $(OCDDIR)/scripts/interface/stlink-v2.cfg \
		   -f $(OCDDIR)/scripts/target/stm32f4x.cfg \


release: ./Release/$(PROJECT).bin
	sudo openocd $(OCDFLAGS) -f ./openocd_release.cfg

debug: ./Debug/$(PROJECT).bin
	sudo openocd $(OCDFLAGS) -f ./openocd_debug.cfg

monitor:
	minicom -b 4000000 -D /dev/ttyUSB0
