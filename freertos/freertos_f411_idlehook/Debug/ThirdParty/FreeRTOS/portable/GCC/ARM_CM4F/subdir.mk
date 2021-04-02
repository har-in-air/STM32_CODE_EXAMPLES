################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.o: ../ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.c ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F/port.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

