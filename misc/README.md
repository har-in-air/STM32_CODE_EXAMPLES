## SWO trace 

* in Cubemx, ensure SYS->Debug is set to 'Asychronous Trace' so that SWO, SWD and SWCLK
pins are assigned alternate function (green)

* in main.c redefine  _write function or __io_putchar() so that
 printf is redirected to ITM module. Add only one of these.
``` 
int _write(int file, char* szMsg, int len) {
	for (int inx = 0; inx < len; inx++){
		ITM_SendChar(*szMsg++);
	    }
	return len;
    }
-OR-
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
    }    
```
* Add #include <stdio.h> 
* Go to STM32CubeIDE->project->Debug->Configuration->Debugger
    * select STLink GDB Server
    * enable Serial Wire Viewer
    * set the mcu clock to match with actual mcu clock set in project
* Flash the cpu, when the debugger is halted in main, 
 go to Window -> View > Enable SWV ITM Data Console window.
* Click on ITM console window settings icon,  enable channel 0
* Click on ITM console window record icon (red circle) to start trace.   
* Click on Debugger resume / step and you will see printf outputs on the ITM console window.
* Click on ITM console window record icon again to stop tracing


### SWO trace using STLink v2 clone
* Isolate interface connector pin #9 from 5V supply (cut the pcb trace next to pin#9,  
do not damage pin#10 connection to 5V supply!)
* Connect STM32F103 pin 31 (PA10) via 22ohm to connector pin #9.

### SWO trace using JLink
If you see error unable to connect under reset, connect both reset pin and SWO pin
from JLink. Or press the board reset button until you see HALT in the STM32CubeIDE console
window showing the JLink debugger programming progress, then immediately release.

## ARM Semihosting on STM32CubeIDE
For printf through SWD interface (slow), no need for UART initialization
 
* Add Linker arguments
```
-specs=rdimon.specs -lc -lrdimon
```
to Project-Properties-C/C++Build-Linker-Miscellaneous (otherflags)

* Project->Debug As->Debug Configuration->select IDE <project> Debug->Startup
paste below 
    monitor arm semihosting enable
in initialization commands textbox

* in main.c add
```
extern void initialize_monitor_handles();

initialise_monitor_handles(); // add this before any printf statements
```
* Click on project syscalls.c, properties -> check exclude Resource from build to avoid
multiple function definition error

* Project->Debug As->Debug Configurations->IDE < project>->Debug-> Debugger
    * Select Autostart local GDB Server
    * Debug Probe ST-Link (OpenOCD)
    * openocd commmand = /usr/bin/openocd
    * openocd options = -f /usr/local/share/openocd/scripts/interface/stlink-v2.cfg
     
* Configuration script
    * First select Automated Generation, apply, then copy the generated script to
    another file with a different name, e.g. freertos_udemy_myDebug.cfg and 
    edit it so that the line is now
    ```
        reset_config srst_nogate connect_assert_srst
    ```
    * The change is required to avoid connection error "target not halted".
    * Now go back to Configuration Script and select User defined and select the edited
    script. The auto generated script is always regenerated on build so any changes
    will be lost. 
    
* Note semihosting only works when Debug Probe is set to STLINK Openocd.
  Semihosting does not work with STlink GDB Server, results in error 
  not able to execute the initialization command
    "monitor arm semihosting enable "


