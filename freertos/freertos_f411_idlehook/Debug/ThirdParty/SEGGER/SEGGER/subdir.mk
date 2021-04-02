################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/SEGGER/SEGGER_RTT.c \
../ThirdParty/SEGGER/SEGGER/SEGGER_RTT_printf.c \
../ThirdParty/SEGGER/SEGGER/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../ThirdParty/SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./ThirdParty/SEGGER/SEGGER/SEGGER_RTT.o \
./ThirdParty/SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.o \
./ThirdParty/SEGGER/SEGGER/SEGGER_RTT_printf.o \
./ThirdParty/SEGGER/SEGGER/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./ThirdParty/SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./ThirdParty/SEGGER/SEGGER/SEGGER_RTT.d \
./ThirdParty/SEGGER/SEGGER/SEGGER_RTT_printf.d \
./ThirdParty/SEGGER/SEGGER/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/SEGGER/SEGGER_RTT.o: ../ThirdParty/SEGGER/SEGGER/SEGGER_RTT.c ThirdParty/SEGGER/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/SEGGER/SEGGER/SEGGER_RTT.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.o: ../ThirdParty/SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.S ThirdParty/SEGGER/SEGGER/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -x assembler-with-cpp -MMD -MP -MF"ThirdParty/SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
ThirdParty/SEGGER/SEGGER/SEGGER_RTT_printf.o: ../ThirdParty/SEGGER/SEGGER/SEGGER_RTT_printf.c ThirdParty/SEGGER/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/SEGGER/SEGGER/SEGGER_RTT_printf.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/SEGGER/SEGGER/SEGGER_SYSVIEW.o: ../ThirdParty/SEGGER/SEGGER/SEGGER_SYSVIEW.c ThirdParty/SEGGER/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/Config" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/OS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/SEGGER/SEGGER" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/hari/STM32CubeIDE/workspace_1.5.1/freertos_f411_idlehook/ThirdParty/FreeRTOS/include" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"ThirdParty/SEGGER/SEGGER/SEGGER_SYSVIEW.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

