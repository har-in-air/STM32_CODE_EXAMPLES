################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/croutine.c \
../ThirdParty/FreeRTOS/event_groups.c \
../ThirdParty/FreeRTOS/list.c \
../ThirdParty/FreeRTOS/queue.c \
../ThirdParty/FreeRTOS/stream_buffer.c \
../ThirdParty/FreeRTOS/tasks.c \
../ThirdParty/FreeRTOS/timers.c 

OBJS += \
./ThirdParty/FreeRTOS/croutine.o \
./ThirdParty/FreeRTOS/event_groups.o \
./ThirdParty/FreeRTOS/list.o \
./ThirdParty/FreeRTOS/queue.o \
./ThirdParty/FreeRTOS/stream_buffer.o \
./ThirdParty/FreeRTOS/tasks.o \
./ThirdParty/FreeRTOS/timers.o 

C_DEPS += \
./ThirdParty/FreeRTOS/croutine.d \
./ThirdParty/FreeRTOS/event_groups.d \
./ThirdParty/FreeRTOS/list.d \
./ThirdParty/FreeRTOS/queue.d \
./ThirdParty/FreeRTOS/stream_buffer.d \
./ThirdParty/FreeRTOS/tasks.d \
./ThirdParty/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/croutine.o: ../ThirdParty/FreeRTOS/croutine.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/croutine.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/FreeRTOS/event_groups.o: ../ThirdParty/FreeRTOS/event_groups.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/event_groups.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/FreeRTOS/list.o: ../ThirdParty/FreeRTOS/list.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/list.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/FreeRTOS/queue.o: ../ThirdParty/FreeRTOS/queue.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/queue.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/FreeRTOS/stream_buffer.o: ../ThirdParty/FreeRTOS/stream_buffer.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/stream_buffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/FreeRTOS/tasks.o: ../ThirdParty/FreeRTOS/tasks.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/tasks.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/FreeRTOS/timers.o: ../ThirdParty/FreeRTOS/timers.c ThirdParty/FreeRTOS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/FreeRTOS/timers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

