################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/product/controls_def.c \
../Core/Src/product/hal_ble_device_rssi_levels_2037468041.c \
../Core/Src/product/hal_co2_equivalent_2088797904.c \
../Core/Src/product/hal_functions.c \
../Core/Src/product/hal_humidity_342048076.c \
../Core/Src/product/hal_mass_concentration_pm_10_0_734215216.c \
../Core/Src/product/hal_mass_concentration_pm_1_0_32791331.c \
../Core/Src/product/hal_mass_concentration_pm_2_5_1698993330.c \
../Core/Src/product/hal_mass_concentration_pm_4_0_1858953557.c \
../Core/Src/product/hal_number_concentration_pm_0_5_1642774029.c \
../Core/Src/product/hal_number_concentration_pm_10_0_1909326236.c \
../Core/Src/product/hal_number_concentration_pm_1_0_1984590476.c \
../Core/Src/product/hal_number_concentration_pm_2_5_1751774023.c \
../Core/Src/product/hal_number_concentration_pm_4_0_891153645.c \
../Core/Src/product/hal_particulate_matter_index_1695740845.c \
../Core/Src/product/hal_temperature_1913614964.c \
../Core/Src/product/hal_typical_particle_size_159299661.c \
../Core/Src/product/hal_voc_1189979747.c \
../Core/Src/product/product.c \
../Core/Src/product/sensors_def.c 

OBJS += \
./Core/Src/product/controls_def.o \
./Core/Src/product/hal_ble_device_rssi_levels_2037468041.o \
./Core/Src/product/hal_co2_equivalent_2088797904.o \
./Core/Src/product/hal_functions.o \
./Core/Src/product/hal_humidity_342048076.o \
./Core/Src/product/hal_mass_concentration_pm_10_0_734215216.o \
./Core/Src/product/hal_mass_concentration_pm_1_0_32791331.o \
./Core/Src/product/hal_mass_concentration_pm_2_5_1698993330.o \
./Core/Src/product/hal_mass_concentration_pm_4_0_1858953557.o \
./Core/Src/product/hal_number_concentration_pm_0_5_1642774029.o \
./Core/Src/product/hal_number_concentration_pm_10_0_1909326236.o \
./Core/Src/product/hal_number_concentration_pm_1_0_1984590476.o \
./Core/Src/product/hal_number_concentration_pm_2_5_1751774023.o \
./Core/Src/product/hal_number_concentration_pm_4_0_891153645.o \
./Core/Src/product/hal_particulate_matter_index_1695740845.o \
./Core/Src/product/hal_temperature_1913614964.o \
./Core/Src/product/hal_typical_particle_size_159299661.o \
./Core/Src/product/hal_voc_1189979747.o \
./Core/Src/product/product.o \
./Core/Src/product/sensors_def.o 

C_DEPS += \
./Core/Src/product/controls_def.d \
./Core/Src/product/hal_ble_device_rssi_levels_2037468041.d \
./Core/Src/product/hal_co2_equivalent_2088797904.d \
./Core/Src/product/hal_functions.d \
./Core/Src/product/hal_humidity_342048076.d \
./Core/Src/product/hal_mass_concentration_pm_10_0_734215216.d \
./Core/Src/product/hal_mass_concentration_pm_1_0_32791331.d \
./Core/Src/product/hal_mass_concentration_pm_2_5_1698993330.d \
./Core/Src/product/hal_mass_concentration_pm_4_0_1858953557.d \
./Core/Src/product/hal_number_concentration_pm_0_5_1642774029.d \
./Core/Src/product/hal_number_concentration_pm_10_0_1909326236.d \
./Core/Src/product/hal_number_concentration_pm_1_0_1984590476.d \
./Core/Src/product/hal_number_concentration_pm_2_5_1751774023.d \
./Core/Src/product/hal_number_concentration_pm_4_0_891153645.d \
./Core/Src/product/hal_particulate_matter_index_1695740845.d \
./Core/Src/product/hal_temperature_1913614964.d \
./Core/Src/product/hal_typical_particle_size_159299661.d \
./Core/Src/product/hal_voc_1189979747.d \
./Core/Src/product/product.d \
./Core/Src/product/sensors_def.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/product/controls_def.o: ../Core/Src/product/controls_def.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/controls_def.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_ble_device_rssi_levels_2037468041.o: ../Core/Src/product/hal_ble_device_rssi_levels_2037468041.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_ble_device_rssi_levels_2037468041.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_co2_equivalent_2088797904.o: ../Core/Src/product/hal_co2_equivalent_2088797904.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_co2_equivalent_2088797904.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_functions.o: ../Core/Src/product/hal_functions.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_functions.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_humidity_342048076.o: ../Core/Src/product/hal_humidity_342048076.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_humidity_342048076.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_mass_concentration_pm_10_0_734215216.o: ../Core/Src/product/hal_mass_concentration_pm_10_0_734215216.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_mass_concentration_pm_10_0_734215216.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_mass_concentration_pm_1_0_32791331.o: ../Core/Src/product/hal_mass_concentration_pm_1_0_32791331.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_mass_concentration_pm_1_0_32791331.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_mass_concentration_pm_2_5_1698993330.o: ../Core/Src/product/hal_mass_concentration_pm_2_5_1698993330.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_mass_concentration_pm_2_5_1698993330.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_mass_concentration_pm_4_0_1858953557.o: ../Core/Src/product/hal_mass_concentration_pm_4_0_1858953557.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_mass_concentration_pm_4_0_1858953557.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_number_concentration_pm_0_5_1642774029.o: ../Core/Src/product/hal_number_concentration_pm_0_5_1642774029.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_number_concentration_pm_0_5_1642774029.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_number_concentration_pm_10_0_1909326236.o: ../Core/Src/product/hal_number_concentration_pm_10_0_1909326236.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_number_concentration_pm_10_0_1909326236.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_number_concentration_pm_1_0_1984590476.o: ../Core/Src/product/hal_number_concentration_pm_1_0_1984590476.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_number_concentration_pm_1_0_1984590476.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_number_concentration_pm_2_5_1751774023.o: ../Core/Src/product/hal_number_concentration_pm_2_5_1751774023.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_number_concentration_pm_2_5_1751774023.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_number_concentration_pm_4_0_891153645.o: ../Core/Src/product/hal_number_concentration_pm_4_0_891153645.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_number_concentration_pm_4_0_891153645.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_particulate_matter_index_1695740845.o: ../Core/Src/product/hal_particulate_matter_index_1695740845.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_particulate_matter_index_1695740845.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_temperature_1913614964.o: ../Core/Src/product/hal_temperature_1913614964.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_temperature_1913614964.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_typical_particle_size_159299661.o: ../Core/Src/product/hal_typical_particle_size_159299661.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_typical_particle_size_159299661.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/hal_voc_1189979747.o: ../Core/Src/product/hal_voc_1189979747.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/hal_voc_1189979747.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/product.o: ../Core/Src/product/product.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/product.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/product/sensors_def.o: ../Core/Src/product/sensors_def.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -DUSE_HAL_DRIVER -DDEBUG -DSTM32G031xx -fstack-usage -MMD -MP -MF"Core/Src/product/sensors_def.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

