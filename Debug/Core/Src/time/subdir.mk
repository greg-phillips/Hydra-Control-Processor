################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/time/ck_time.c \
../Core/Src/time/utc_time.c 

OBJS += \
./Core/Src/time/ck_time.o \
./Core/Src/time/utc_time.o 

C_DEPS += \
./Core/Src/time/ck_time.d \
./Core/Src/time/utc_time.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/time/ck_time.o: ../Core/Src/time/ck_time.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/time/ck_time.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/time/utc_time.o: ../Core/Src/time/utc_time.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/time/utc_time.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

