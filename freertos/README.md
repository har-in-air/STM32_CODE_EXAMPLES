# 1. Integrating FreeRTOS with STM32CubeIDE

## STM32CubeIDE with CubeMX setup 

* Create a new STM32CubeIDE project for your specific micro 
* Use CubeMx to configure on-board LED, Button gpio, uart, clock 
* As per FreeRTOS, for better performance use a spare timer (e.g. TIM1) for time base, not SysTick
* Build code and test
* Under project folder, create a new folder /ThirdParty
* Under /ThirdParty, create a new folder /FreeRTOS. Here you can add other modules like graphics, lwIP etc
* Under /FreeRTOS create a new folder /org (needed for compatibility with Segger SystemView)

## Integrating FreeRTOS 

* Download Freertos from freertos.org and unzip the archive
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
   * add workspace/your project/ThirdParty/FreeRTOS/org/Source/include
   * add workspace/your project/ThirdParty/FreeRTOS/org/Source/portable/GCC/ARM-CM4F
* Create a new /Config project subdirectory    
* Goto archive FreeRTOS/Demo, search for a mcu compatible project, eg.
CORTEX_M4F_STM32F407ZG-SK and copy FreeRTOSConfig.h to /Config
* In project click on /Config, Properties - uncheck "exclude resource from build" 
* Add /Config path to the project GCC compiler include path
* Edit FreeRTOSConfig.h 
    * Move  `extern  uint32_t SystemCoreClock` to outside `#ifdef __ICCARM__` otherwise it will not be declared to the compiler
    * If you are not going to implement these hooks, ensure these settings
``` 
#define configUSE_TICK_HOOK             0
#define configCHECK_FOR_STACK_OVERFLOW	0
#define configUSE_MALLOC_FAILED_HOOK	  0
```
or you will  get undefined references to vApplicationTickHook, etc.
* Edit project stm32f4xx_it.c . Comment out SV_Handler, PendSV_Handler and SysTickHandler as they
are defined in FreeRTOS ... port.c
* Test build and run of your code to ensure nothing is broken (e.g. blinking led and printing to uart)

# 2. Integrating SystemView into FreeRTOS project in STM32CubeIDE

* Download Systemview target sources matching with binary release and extract
SystemView_Src_V331
```
    /Config
        Global.h
        SEGGER_RTT_Conf.h
        SEGGER_SYSVIEW_Conf.h
    /Sample
        /FreeRTOSV10
            /Config
                /Cortex-M
                    SEGGER_SYSVIEW_Config_FreeRTOS.c
            /Patch
                FreeRTOSV10_Core.patch
            SEGGER_SYSVIEW_FreeRTOS.c
            SEGGER_SYSVIEW_FreeRTOS.h        
    /SEGGER
        Everything except for /Syscalls
```        
* Create the following project folder structure under `/stm32cubeide_project/ThirdParty`, and make sure none of them are
   excluded from build, now populate tree from the source archive
``` 
SEGGER
    /Config
        Global.h
        SEGGER_RTT_Conf.h
        SEGGER_SYSVIEW_Conf.h   
        SEGGER_SYSVIEW_Config_FreeRTOS.c        
    /OS
        SEGGER_SYSVIEW_FreeRTOS.c
        SEGGER_SYSVIEW_FreeRTOS.h        
    /Patch
        /FreeRTOSv10.3.1
            FreeRTOSV10_Core.patch
    /SEGGER
        Everything from src /SEGGER except for /Syscalls
```    
* include path for C/C++build, add
 ```
        SEGGER/Config
        SEGGER/OS
        SEGGER/SEGGER
 ```       
* include path for ASM build (because .S file in /SEGGER includes portmacro.h)
```
        FreeRTOS/org/Source/portable/GCC/ARM_CM4F
```    
* Patching 
    * RClick on project  /FreeRTOS, menu Team     - Apply Path - select patch file
    * browse for FreeRTOSV10_Core.patch and select, next
    * finish patching  
* Edit FreeRTOSConfig.h
   * Add two #defines to list
```   
    #define INCLUDE_xTaskGetIdleTaskHandle 1
    #define INCLUDE_pxTaskGetStackStart     1
```    
    Add `#include "SEGGER_SYSVIEW_FreeRTOS.h"` at end of file
* Edit SEGGER_SYSVIEW_Conf.h
``` 
    #define SEGGER_SYSVIEW_CORE SEGGER_SYSVIEW_CORE_CM3        
    #define SEGGER_SYSVIEW_RTT_BUFFER_SIZE (1024*8)   // allocate 8KBytes for trace buffer, OK if you have 128kbytes SRAM
```    
* Edit SEGGER_SYSVIEW_Config_FreeRTOS.c
```
    #define SYSVIEW_APP_NAME    "FreeRTOS Udemy"
    #define SYSVIEW_DEVICE_NAME  "STM32F411CEU" 
    #define SYSVIEW_RAM_BASE    0x20000000  // (start of SRAM1 for your micro)
```    
* Enable DWT CycleCounter : first statement in main.c should be
```
    DWT->CTRL |= (1<<0);  
```    
* Add SystemView config and start statements in main.c before any FreeRTOS api calls. To avoid configAssert error in SEGGER_SYSVIEW_Start() caused by 
 un-initialized variable ulMaxPRIGROUP, we need to edit port.c to 
 add an initialization function vSetVarulMaxPRIGROUPValue() and call it before SEGGER_SYSVIEW_Start().
 [Source](https://forum.segger.com/index.php/Thread/6046-SOLVED-Systemview-stuck-in-configASSERT-with-FreeRTOS-STM32CubeMX/)
```  
    SEGGER_SYSVIEW_Conf();
    vSetVarulMaxPRIGROUPValue();
    SEGGER_SYSVIEW_Start();    
``` 
  * Add declaration below in file portmacro.h
```
#ifdef configASSERT
	void vSetVarulMaxPRIGROUPValue( void );
#endif
```
  * Add function below at end of file port.c
```
#if( configASSERT_DEFINED == 1 )
void vSetVarulMaxPRIGROUPValue( void )
{
	volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
	volatile uint8_t ucMaxPriorityValue;
	/* Determine the number of priority bits available.  First write to all
	possible bits. */
	*pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;
	/* Read the value back to see how many bits stuck. */
	ucMaxPriorityValue = *pucFirstUserPriorityRegister;
	/* Calculate the maximum acceptable priority group value for the number
	of bits read back. */
	ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;
	while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
	{
		ulMaxPRIGROUPValue--;
		ucMaxPriorityValue <<= ( uint8_t ) 0x01;
	}
#ifdef __NVIC_PRIO_BITS
	{
		/* Check the CMSIS configuration that defines the number of
		priority bits matches the number of priority bits actually queried
		from the hardware. */
		configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == __NVIC_PRIO_BITS );
	}
#endif
#ifdef configPRIO_BITS
	{
		/* Check the FreeRTOS configuration that defines the number of
		priority bits matches the number of priority bits actually queried
		from the hardware. */
		configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == configPRIO_BITS );
	}
#endif
	/* Shift the priority group value back to its position within the AIRCR
	register. */
	ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
	ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;
}
#endif /* conifgASSERT_DEFINED */
``` 
* To trace software timer callbacks, add this to `SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.h` and add the
directory `SEGGER/OS` to the C/C++ build settings include paths. See the freertos_idlehook project example.
```
#define traceTIMER_ENTER(pxTimer)					SEGGER_SYSVIEW_RecordEnterTimer((U32)pxTimer)
#define traceTIMER_EXIT()							SEGGER_SYSVIEW_RecordExitTimer()
 ```
 

