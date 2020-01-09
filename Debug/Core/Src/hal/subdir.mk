################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/hal/co2_voc.c \
../Core/Src/hal/leds.c \
../Core/Src/hal/lux.c \
../Core/Src/hal/temp_humidity.c 

OBJS += \
./Core/Src/hal/co2_voc.o \
./Core/Src/hal/leds.o \
./Core/Src/hal/lux.o \
./Core/Src/hal/temp_humidity.o 

C_DEPS += \
./Core/Src/hal/co2_voc.d \
./Core/Src/hal/leds.d \
./Core/Src/hal/lux.d \
./Core/Src/hal/temp_humidity.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/hal/co2_voc.o: ../Core/Src/hal/co2_voc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/hal/co2_voc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/hal/leds.o: ../Core/Src/hal/leds.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/hal/leds.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/hal/lux.o: ../Core/Src/hal/lux.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/hal/lux.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/hal/temp_humidity.o: ../Core/Src/hal/temp_humidity.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/hal/temp_humidity.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

