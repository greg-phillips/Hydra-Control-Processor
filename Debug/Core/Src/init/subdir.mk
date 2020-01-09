################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/init/init.c 

OBJS += \
./Core/Src/init/init.o 

C_DEPS += \
./Core/Src/init/init.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/init/init.o: ../Core/Src/init/init.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/init/init.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

