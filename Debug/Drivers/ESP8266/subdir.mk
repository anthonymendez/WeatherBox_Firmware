################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ESP8266/esp8266.c 

OBJS += \
./Drivers/ESP8266/esp8266.o 

C_DEPS += \
./Drivers/ESP8266/esp8266.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ESP8266/esp8266.o: ../Drivers/ESP8266/esp8266.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L073xx -DDEBUG -c -I"C:/Users/sdp/STM32CubeIDE/workspace_1.1.0/WeatherBox_Firmware/Drivers/BME280" -I"C:/Users/sdp/STM32CubeIDE/workspace_1.1.0/WeatherBox_Firmware/Drivers/CCS811" -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I"C:/Users/sdp/STM32CubeIDE/workspace_1.1.0/WeatherBox_Firmware/Drivers/ESP8266" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/ESP8266/esp8266.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

