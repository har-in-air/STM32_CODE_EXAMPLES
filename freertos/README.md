# Integrating FreeRTOS with STM32CubeIDE

## STM32CubeIDE with CubeMX setup 

* Create a new STM32CubeIDE project for your specific micro 
* Use CubeMx to configure on-board LED, Button gpio, uart, clock 
* As per FreeRTOS, for better performance use a spare timer (e.g. TIM1) for time base, not SysTick
* Build code and test
* Under project folder, create a new folder /ThirdParty
* Under /ThirdParty, create a new folder /FreeRTOS. Here you can add other modules like graphics, lwIP etc
* Under /FreeRTOS create a new folder /org (needed for compatibility with Segger SystemView)

## Integrating FreeRTOS 

* Download freertos from freertos.org and unzip the archive
* We don't need the /FreeRTOS-Plus subdirectory
* From FreeRTOS archive copy the folder /FreeRTOS/Source to the project /org directory
* From FreeRTOS archive copy the folder /FreeRTOS/License to the project /org directory
* In /org/source/portable, delete all items except /GCC, /MemMang and readme.txt
* In /MemMang, delete all items except heap_4.c and README.url
* In /GCC, delete all except your MCU-specific directory e.g. /ARM_CM4F.  
This has a file port.c which has the cpu dependent code for FreeRTOS.
* Go to STM32CubeIDE and refresh project to show the ThirdParty folder and check the tree
* Click on ThirdParty, menu Properties->C/C++ build, uncheck "exclude resource from build"
* Go to Project properties, C/C++ build->Settings->Tool Settings->MCU GCC Compiler->Include Paths
   * add workspace/<your project>/ThirdParty/FreeRTOS/org/Source/include
   * add workspace/<your project>/ThirdParty/FreeRTOS/org/Source/portable/GCC/ARM-CM4F
* Create a new /Config project subdirectory    
* Goto archive FreeRTOS/Demo, search for a mcu compatible project, eg.
CORTEX_M4F_STM32F407ZG-SK and copy FreeRTOSConfig.h to /Config
* In project click on /Config, Properties - uncheck "exclude resource from build" 
* Add /Config path to the project GCC compiler include path
* Edit FreeRTOSConfig.h 
    * Move  'extern  uint32_t SystemCoreClock' to outside
the #ifdef __ICCARM__, otherwise it will not be declared to the compiler

    * Set
``` 
#define configUSE_TICK_HOOK             0
#define configCHECK_FOR_STACK_OVERFLOW	0
#define configUSE_MALLOC_FAILED_HOOK	0
```
or you will  get undefined references to vApplicationTickHook, etc.

* Edit project stm32f4xx_it.c . Comment out SV_Handler, PendSV_Handler and SysTickHandler as they
are defined in FreeRTOS ... port.c

* Test build and run of your code blinking led and printing to uart

## Testing with internal RC oscillator

To test using default power-on  clock using HSI internal oscillator, even if you
have the external clock and pll already defined using CubeMx, add couple of lines of code
```
/* Configure the system clock */
SystemClock_Config();

/* USER CODE BEGIN SysInit */ 
// Add these two lines of code
HAL_RCC_DeInit(); // 16 MHz HSI
SystemCoreClockUpdate();

/* USER CODE END SysInit */
```




