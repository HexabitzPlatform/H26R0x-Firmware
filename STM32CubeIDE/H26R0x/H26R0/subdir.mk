################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/startup_stm32f091xc.s 

C_SRCS += \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_dma.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_gpio.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_it.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_rtc.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_timers.c \
D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_uart.c 

OBJS += \
./H26R0/H26R0.o \
./H26R0/H26R0_dma.o \
./H26R0/H26R0_gpio.o \
./H26R0/H26R0_it.o \
./H26R0/H26R0_rtc.o \
./H26R0/H26R0_timers.o \
./H26R0/H26R0_uart.o \
./H26R0/startup_stm32f091xc.o 

S_DEPS += \
./H26R0/startup_stm32f091xc.d 

C_DEPS += \
./H26R0/H26R0.d \
./H26R0/H26R0_dma.d \
./H26R0/H26R0_gpio.d \
./H26R0/H26R0_it.d \
./H26R0/H26R0_rtc.d \
./H26R0/H26R0_timers.d \
./H26R0/H26R0_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H26R0/H26R0.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0.c H26R0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH26R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H26R0 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H26R0/H26R0.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H26R0/H26R0_dma.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_dma.c H26R0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH26R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H26R0 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H26R0/H26R0_dma.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H26R0/H26R0_gpio.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_gpio.c H26R0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH26R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H26R0 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H26R0/H26R0_gpio.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H26R0/H26R0_it.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_it.c H26R0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH26R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H26R0 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H26R0/H26R0_it.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H26R0/H26R0_rtc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_rtc.c H26R0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH26R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H26R0 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H26R0/H26R0_rtc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H26R0/H26R0_timers.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_timers.c H26R0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH26R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H26R0 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H26R0/H26R0_timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H26R0/H26R0_uart.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/H26R0_uart.c H26R0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH26R0 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H26R0 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H26R0/H26R0_uart.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H26R0/startup_stm32f091xc.o: D:/Hexabitz/for\ Release/Modules\ firmware/H26R0x/H26R0/startup_stm32f091xc.s H26R0/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"H26R0/startup_stm32f091xc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@" "$<"

