################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Arduino_libs/avr/dtostrf.c 

C_DEPS += \
./Arduino_libs/avr/dtostrf.d 

OBJS += \
./Arduino_libs/avr/dtostrf.o 


# Each subdirectory must supply rules for building sources it contributes
Arduino_libs/avr/%.o Arduino_libs/avr/%.su: ../Arduino_libs/avr/%.c Arduino_libs/avr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F4 -DARDUINO=100 '-D__weak=__attribute__((weak))' '-D__packed=__attributes__((__packed__))' -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Arduino_libs-2f-avr

clean-Arduino_libs-2f-avr:
	-$(RM) ./Arduino_libs/avr/dtostrf.d ./Arduino_libs/avr/dtostrf.o ./Arduino_libs/avr/dtostrf.su

.PHONY: clean-Arduino_libs-2f-avr

