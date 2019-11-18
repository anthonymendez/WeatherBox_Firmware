################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Drivers/CCS811/subdir.mk
-include Drivers/BME280/subdir.mk
-include Core/Startup/subdir.mk
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
WeatherBox_Firmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
WeatherBox_Firmware.list \


# All Target
all: WeatherBox_Firmware.elf secondary-outputs

# Tool invocations
WeatherBox_Firmware.elf: $(OBJS) $(USER_OBJS) C:\Users\sdp\STM32CubeIDE\workspace_1.1.0\WeatherBox_Firmware\STM32L073RZTX_FLASH.ld
	arm-none-eabi-gcc -o "WeatherBox_Firmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\sdp\STM32CubeIDE\workspace_1.1.0\WeatherBox_Firmware\STM32L073RZTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="WeatherBox_Firmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

WeatherBox_Firmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "WeatherBox_Firmware.list"
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
