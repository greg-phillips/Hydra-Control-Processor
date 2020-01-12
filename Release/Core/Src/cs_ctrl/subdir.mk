################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cs_ctrl/cs_manager.c \
../Core/Src/cs_ctrl/hal_event.c \
../Core/Src/cs_ctrl/hal_sample.c \
../Core/Src/cs_ctrl/sensors.c 

OBJS += \
./Core/Src/cs_ctrl/cs_manager.o \
./Core/Src/cs_ctrl/hal_event.o \
./Core/Src/cs_ctrl/hal_sample.o \
./Core/Src/cs_ctrl/sensors.o 

C_DEPS += \
./Core/Src/cs_ctrl/cs_manager.d \
./Core/Src/cs_ctrl/hal_event.d \
./Core/Src/cs_ctrl/hal_sample.d \
./Core/Src/cs_ctrl/sensors.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/cs_ctrl/cs_manager.o: ../Core/Src/cs_ctrl/cs_manager.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cs_ctrl/cs_manager.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/cs_ctrl/hal_event.o: ../Core/Src/cs_ctrl/hal_event.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cs_ctrl/hal_event.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/cs_ctrl/hal_sample.o: ../Core/Src/cs_ctrl/hal_sample.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cs_ctrl/hal_sample.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/cs_ctrl/sensors.o: ../Core/Src/cs_ctrl/sensors.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/cs_ctrl/sensors.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

