################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hal/leds.c 

OBJS += \
./Core/Src/hal/leds.o 

C_DEPS += \
./Core/Src/hal/leds.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/hal/leds.o: ../Core/Src/hal/leds.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/hal/leds.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

