################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/time/subdir.mk
-include Core/Src/init/subdir.mk
-include Core/Src/iMatrix/kalman_filter/subdir.mk
-include Core/Src/hal/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Hydra\ 1.0.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Hydra\ 1.0.list \


# All Target
all: Hydra\ 1.0.elf secondary-outputs

# Tool invocations
Hydra\ 1.0.elf: $(OBJS) $(USER_OBJS) C:\Users\greg.phillips\STM32CubeIDE\workspace_1.1.0\Hydra\ 1.0\STM32G031K8UX_FLASH.ld
	arm-none-eabi-gcc -o "Hydra 1.0.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\greg.phillips\STM32CubeIDE\workspace_1.1.0\Hydra 1.0\STM32G031K8UX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Hydra 1.0.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Hydra\ 1.0.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Hydra 1.0.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
