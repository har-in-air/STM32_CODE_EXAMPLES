# Integrating FreeRTOS with STM32CubeIDE

## Environment

* STM32CubeIDE version v1.6.0
* MCU Firmware version FW_F4 V1.26.0
* FreeRTOSv202012.00 downloaded from FreeRTOS.org
* WeAct STM32F411CEU6 "Black Pill" development board
* Reference project : `freertos_f411_hello_world`

## STM32CubeIDE with CubeMX setup 

* Create a new STM32CubeIDE project for your specific micro. For the rest of this description,
the project `freertos_f411_hello_world` is used as a reference.
* Use CubeMx to configure on-board LED, button gpio, uart, system clock as required. In `freertos_f411_test`,
USART2 for printing messages, PC13 onboard LED, system clock = 96MHz using the 25MHz board crystal as source.
* FreeRTOS uses the SysTick timer for its tick and so does HAL by default. To prevent this conflict, use CubeMx -> Pinout & Configuration -> System Core -> SYS  to change HAL `Timebase source` from
SysTick to any free timer e.g. TIM10 for the STM32F411 in our reference project. 
* Regenerate code from CubeMx and now you will see a new file `stm32f4xx_hal_timebase_tim.c` in the 
Src directory  for HAL tick generation.
* Add some basic test code in main.c in the while(1) loop - toggle an LED, print to UART, use HAL_Delay().  
* Build and run it to ensure everything is working with the selected system clock and HAL tick
timebase source. 

## Integration

* In your project folder, create a new folder `/ThirdParty`
* In `/ThirdParty`, create a new folder `/FreeRTOS`.
* In `/ThirdParty`, select project->Properties->C/C++ build, ensure "Exclude resource from build" is unchecked.
* Download FreeRTOS from freertos.org and unzip the archive. In our example FreeRTOSv202012.00 is the version used.
* In the unzipped FreeRTOS archive navigate to `/FreeRTOS/Source`
* Copy everything (`/include, /portable, *.c`) to your project `/FreeRTOS` folder
* In your project `FreeRTOS/portable` folder, delete all items except `/GCC`, `/MemMang` and `readme.txt`
* In `Core/Src` folder, select sysmem.c->Properties, check "Exclude resource from build" as FreeRTOS will
take care of heap management.
* In `ThirdParty/FreeRTOS/portable/MemMang`, check "Exclude resource from build" for all .c files except
heap_4.c
* In `ThirdParty/FreeRTOS/portable/GCC` folder, delete all folders except our project MCU-specific folder `/ARM_CM4F`.  
This has a file `port.c` which has architecture-dependent FreeRTOS code.
* Select Project properties->C/C++ build->Settings->Tool Settings->MCU GCC Compiler->Include Paths
   * Add `workspace/your_project/ThirdParty/FreeRTOS/include`
   * Add `workspace/your_project/ThirdParty/FreeRTOS/portable/GCC/ARM-CM4F`
* In the downloaded archive folder `FreeRTOS/Demo`, search for an MCU-compatible project folder. For the STM32F411 this is
`CORTEX_M4F_STM32F407ZG-SK`. From this folder copy `FreeRTOSConfig.h` to your project folder `/ThirdParty/FreeRTOS`
* Add `ThirdParty/FreeRTOS/` to the project `...MCU GCC compiler->Include` path
* Edit your project copy of `FreeRTOSConfig.h` 
	* `SystemCoreClock` needs to be visible to our compiler.
```
#if defined (__ICCARM__) || defined(__GNUC__) || defined(__CC_ARM)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif
```
    * If you are not implementing these hooks, ensure they are disabled. Else you will get build errors 
    with undefined references to `vApplicationTickHook`, etc.
```
#define configUSE_TICK_HOOK             	0
#define configUSE_IDLE_HOOK             	0
#define configCHECK_FOR_STACK_OVERFLOW		0
#define configUSE_MALLOC_FAILED_HOOK	  	0
```
* With CubeMx navigate to Pinout & Configuration -> NVIC -> Code Generation.
Uncheck code generation for `System service call ...`, `Pendable request ...` and `Time base ...` as 
these interrupt handlers are defined in `FreeRTOS ... port.c`. Regenerate code for your project.
* Build and run your basic test code (e.g. blinking LED and printing to UART) to ensure nothing is broken.
* For testing FreeRTOS basic functionality, the project `freertos_f411_test` 
has two tasks that print messages over the USART2 interface. You can build and run this code to test 
successful integration of FreeRTOS into your STM32CubeIDE project. Note that if `vTaskStartScheduler()` 
runs successfully, any code you previously placed in the  `while(1)` loop to test CubeMx generated code 
will no longer be executed, as vTaskStartScheduler will never return.

	

