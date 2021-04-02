################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/main.o: ../Core/Src/main.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32f4xx_hal_msp.o: ../Core/Src/stm32f4xx_hal_msp.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32f4xx_hal_timebase_tim.o: ../Core/Src/stm32f4xx_hal_timebase_tim.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_hal_timebase_tim.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/stm32f4xx_it.o: ../Core/Src/stm32f4xx_it.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/stm32f4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/syscalls.o: ../Core/Src/syscalls.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/system_stm32f4xx.o: ../Core/Src/system_stm32f4xx.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/system_stm32f4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

