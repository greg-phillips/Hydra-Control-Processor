################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/i2c/sgp30/sgp30.c 

OBJS += \
./Core/Src/i2c/sgp30/sgp30.o 

C_DEPS += \
./Core/Src/i2c/sgp30/sgp30.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/i2c/sgp30/sgp30.o: ../Core/Src/i2c/sgp30/sgp30.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/i2c/sgp30/sgp30.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

