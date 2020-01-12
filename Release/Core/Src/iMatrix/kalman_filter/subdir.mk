################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/iMatrix/kalman_filter/kalman_filter.c 

OBJS += \
./Core/Src/iMatrix/kalman_filter/kalman_filter.o 

C_DEPS += \
./Core/Src/iMatrix/kalman_filter/kalman_filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/iMatrix/kalman_filter/kalman_filter.o: ../Core/Src/iMatrix/kalman_filter/kalman_filter.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/iMatrix/kalman_filter/kalman_filter.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

