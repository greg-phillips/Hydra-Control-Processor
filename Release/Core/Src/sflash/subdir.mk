################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sflash/memory_manager.c \
../Core/Src/sflash/spi_flash.c 

OBJS += \
./Core/Src/sflash/memory_manager.o \
./Core/Src/sflash/spi_flash.o 

C_DEPS += \
./Core/Src/sflash/memory_manager.d \
./Core/Src/sflash/spi_flash.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sflash/memory_manager.o: ../Core/Src/sflash/memory_manager.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sflash/memory_manager.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/sflash/spi_flash.o: ../Core/Src/sflash/spi_flash.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/sflash/spi_flash.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

