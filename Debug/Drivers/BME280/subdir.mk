################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BME280/bme280.c 

OBJS += \
./Drivers/BME280/bme280.o 

C_DEPS += \
./Drivers/BME280/bme280.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BME280/bme280.o: ../Drivers/BME280/bme280.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L073xx -DDEBUG -c -I"C:/Users/sdp/STM32CubeIDE/workspace_1.1.0/WeatherBox_Firmware/Drivers/BME280" -I"C:/Users/sdp/STM32CubeIDE/workspace_1.1.0/WeatherBox_Firmware/Drivers/CCS811" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BME280/bme280.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

